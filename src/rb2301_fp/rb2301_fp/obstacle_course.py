import argparse
import configparser
import heapq
import math
import os
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

np.set_printoptions(2, suppress=True, threshold=np.inf) # Print numpy arrays to specified d.p., suppress scientific notation and do not truncate
set_logger_level("obstaclecourse", level=LoggingSeverity.INFO) # Configure to either LoggingSeverity.INFO or LoggingSeverity.DEBUG  


_PACKAGE_DIR = os.path.dirname(os.path.realpath(__file__))

def wrap_deg(angle:float) -> float:
    '''Wrap an angle in degrees into [-180, 180).'''
    return (angle + 180.0) % 360.0 - 180.0

def stamp_seconds(stamp) -> float:
    '''A message header's stamp in seconds'''
    return stamp.sec + stamp.nanosec * 1e-9

def optitrack_to_maze(x:float, y:float, heading_deg:float, frame:dict) -> tuple:
    '''Optitrack-frame pose (x, y, heading in degrees) -> CA2 maze-frame pose (see optitrack_variables.config).'''
    dx, dy = x - frame["origin_x"], y - frame["origin_y"]
    r = math.radians(frame["rotation_deg"])
    c, s = math.cos(r), math.sin(r)
    return (c * dx + s * dy, -s * dx + c * dy,
            wrap_deg(heading_deg - frame["rotation_deg"] - frame["heading_offset_deg"]))

def maze_to_course(x:float, y:float, heading_deg:float, course:dict) -> tuple:
    '''CA2 maze-frame pose -> final-project course frame (origin/rotation of the course in the maze frame, [course] in the config).'''
    dx, dy = x - course["origin_x"], y - course["origin_y"]
    r = math.radians(course["rotation_deg"])
    c, s = math.cos(r), math.sin(r)
    return (c * dx + s * dy, -s * dx + c * dy, wrap_deg(heading_deg - course["rotation_deg"]))

def load_frame(robot_number:int) -> tuple:
    '''Read optitrack_variables.config: returns (frame, course) for this robot.'''
    parser = configparser.ConfigParser()
    path = os.path.join(_PACKAGE_DIR, "optitrack_variables.config")
    if not parser.read(path):
        raise FileNotFoundError(f"Could not read {path}")
    frame = {k: parser.getfloat("frame", k) for k in ("origin_x", "origin_y", "rotation_deg")}
    raw = parser.get("frame", "heading_offset_deg", fallback="auto").strip().lower()
    # Rigid bodies are created facing +y of the maze frame, so Motive's zero heading is maze +y (90 deg)
    frame["heading_offset_deg"] = wrap_deg(-(frame["rotation_deg"] + 90.0)) if raw == "auto" else float(raw)
    if parser.has_section("heading_offsets") and parser.has_option("heading_offsets", str(robot_number)):
        frame["heading_offset_deg"] = parser.getfloat("heading_offsets", str(robot_number))
    course = {k: parser.getfloat("course", k, fallback=0.0) for k in ("origin_x", "origin_y", "rotation_deg")}
    return frame, course

# Robot size, from CA1
ROBOT_WIDTH = 0.225 # m
ROBOT_LENGTH = 0.289 # m
# m. Any closer to a moving obstacle and the robot can touch it, whatever its heading: half the robot's diagonal, plus a
# cell's diagonal, because the hits and the path are both cell centres, each up to half a cell's diagonal off
SWEEP_CLEARANCE = math.hypot(ROBOT_WIDTH, ROBOT_LENGTH) / 2 + math.sqrt(2) * 0.025

# LiDAR mounting, from the URDF (laser_joint). It faces backwards, so beam angle 0 points behind the robot
LIDAR_X = 0.114 # m ahead of the robot's centre
LIDAR_YAW = math.pi # rad, from the robot's heading to the LiDAR's angle 0

# Driving
DT = 0.05 # s, timer period
MAX_SPEED = 0.3 # m/s, the real robot's limit
MAX_ACCEL = 0.5 # m/s^2. Speed ramps up at most this fast, but drops at once
MAX_TURN = 0.6 # rad/s, move_2D's turn limit on the real robot
MIN_TURN = 0.3 # rad/s. From CA2: the real robot doesn't turn any slower than this
TURN_GAIN = 1.5 # rad/s of turn per rad of heading error, when turning in place
LOOKAHEAD = 0.2 # m, how far along the path the carrot is
TURN_IN_PLACE = math.radians(60) # turn in place when the carrot is further off the heading than this
ALIGN_TOL = math.radians(3) # heading errors smaller than this count as zero
ARRIVE_TOL = 0.05 # m, how close to the end of a path counts as arriving
STOP_DISTANCE = 0.1 # m. Stop if a LiDAR return is this close in front of the robot's front edge
REPLAN_PERIOD = 1.0 # s

# Planning
INFLATED = 1000.0 # cost of a cell within half the robot's width of an occupied cell
CLEARANCE_WEIGHT = 4.0 # extra cost of a cell right next to an inflated one. Keeps paths in the middle of gaps
EIGHT_NEIGHBOURS = [(dr, dc) for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc]
STEPS = [(dr, dc, math.hypot(dr, dc)) for dr, dc in EIGHT_NEIGHBOURS] # (row, col) step and its length in cells

EXPLORE, HOLD, CROSS = "EXPLORE", "HOLD", "CROSS"

def dilate(mask:np.ndarray, offsets) -> np.ndarray:
    '''Grow the True cells of mask: a cell becomes True if a True cell is any (row, col) offset away from it'''
    grown = mask.copy()
    rows, cols = mask.shape
    for dr, dc in offsets:
        # grown[r + dr, c + dc] |= mask[r, c], for every cell at once
        grown[max(dr, 0):rows + min(dr, 0), max(dc, 0):cols + min(dc, 0)] |= \
            mask[max(-dr, 0):rows + min(-dr, 0), max(-dc, 0):cols + min(-dc, 0)]
    return grown

def disk(radius:float, cell_size:float) -> list:
    '''(row, col) offsets of every cell within radius, rounded up to whole cells'''
    r = int(np.ceil(radius / cell_size))
    return [(dr, dc) for dr in range(-r, r + 1) for dc in range(-r, r + 1) if dr * dr + dc * dc <= r * r]

def search(cost:np.ndarray, start:tuple, goal_mask:np.ndarray=None, goal:tuple=None) -> tuple:
    '''Lowest-cost search over 8-connected cells from start (row, col).
    With goal_mask, it stops at the first True cell it reaches: the nearest one by path cost. Without it, it reaches
    every cell it can. With goal (row, col) it is A*, with the octile distance to goal as the heuristic, otherwise Dijkstra.
    A path can leave an inflated cell but never enter one from a cell that isn't, so a robot that has drifted
    close to a wall can still get out.
    Returns (reached, parent, g): the goal cell reached (None if none), and each cell's parent and cost from start'''
    cost = cost.copy()
    cost[[0, -1], :] = np.inf # block the border, so neighbours never fall outside the grid
    cost[:, [0, -1]] = np.inf
    cost = cost.tolist() # lists are faster than numpy arrays for one cell at a time
    if goal_mask is not None:
        goal_mask = goal_mask.tolist()

    def heuristic(r, c):
        if goal is None:
            return 0.0
        dr, dc = abs(r - goal[0]), abs(c - goal[1])
        return max(dr, dc) + (math.sqrt(2) - 1) * min(dr, dc)

    g = {start: 0.0} # lowest cost found so far from start to each cell
    parent = {start: None}
    heap = [(heuristic(*start), start)]
    closed = set()
    while heap:
        _, cell = heapq.heappop(heap)
        if cell in closed:
            continue
        closed.add(cell)
        r, c = cell
        if goal_mask is not None and goal_mask[r][c]:
            return cell, parent, g
        inflated = cost[r][c] >= INFLATED
        for dr, dc, length in STEPS:
            nr, nc = r + dr, c + dc
            step_cost = cost[nr][nc]
            if step_cost == math.inf or (step_cost >= INFLATED and not inflated):
                continue
            new_g = g[cell] + length * step_cost
            if new_g < g.get((nr, nc), math.inf):
                g[(nr, nc)] = new_g
                parent[(nr, nc)] = cell
                heapq.heappush(heap, (new_g + heuristic(nr, nc), (nr, nc)))
    return None, parent, g

def plan_path(cost:np.ndarray, start:tuple, goal_mask:np.ndarray, goal:tuple=None):
    '''Path from start to the nearest True cell of goal_mask as a list of (row, col), or None if none can be reached'''
    cell, parent, _ = search(cost, start, goal_mask, goal)
    if cell is None:
        return None
    path = []
    while cell is not None:
        path.append(cell)
        cell = parent[cell]
    return path[::-1]

def plan_to(cost:np.ndarray, start:tuple, goal:tuple):
    '''A* path from start to goal, both (row, col), or None if goal can't be reached'''
    goal_mask = np.zeros(cost.shape, dtype=bool)
    goal_mask[goal] = True
    return plan_path(cost, start, goal_mask, goal)

class LidarNode(Node):
    def __init__(self):
        """Node constructor"""
        super().__init__("LidarNode")
        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # Subscribe to lidar
        self.is_simulation = None

    def sub_scan_callback(self, msg):
        """Scan subscriber"""
        if len(msg.ranges) <= 360: 
            self.is_simulation = True
        else:
            self.is_simulation = False

class ObstacleCourseNode(Node):
    '''Node to navigate obstacle course, using pose from either gazebo odometer or optitrack and lidar scan data'''
    def __init__(self, is_simulation:bool=True, calibrate:bool=False):
        super().__init__('obstaclecourse')
        self.get_logger().info("Starting ObstacleCourseNode")

        self.is_simulation = is_simulation
        self.calibrate = calibrate # Real robot: only print poses, never drive
        self.raw_pose = None # Pose exactly as Optitrack reported it (real robot only)
        self.frame = self.course = None
        self._ticks = 0

        if is_simulation:
            self.max_translate_velocity = 1.4
        else:
            self.max_translate_velocity = 0.3 # Please keep this in place; 0.3m/s is more than fast enough 

        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # Subscribe to LiDAR scan data

        # Subscribe to the dynamic_pose topic from Gazebo that publishes ground-truth pose data
        if self.is_simulation:
            self.subscription = self.create_subscription(Odometry, 'odom', self.odometer_callback, 2)
        else:
            qos_profile = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
            # On the Bingda robots ROS_DOMAIN_ID is the robot's number, so the Optitrack topic is picked automatically
            domain = os.environ.get("ROS_DOMAIN_ID", "")
            self.robot_number = int(domain) if domain.isdigit() and 1 <= int(domain) <= 99 else 3
            self.frame, self.course = load_frame(self.robot_number)
            self.get_logger().info(f"Reading pose from /vrpn_mocap/bingda_{self.robot_number:03d}/pose")

            self.map_sub = self.create_subscription( 
                PoseStamped,
                f'/vrpn_mocap/bingda_{self.robot_number:03d}/pose',
                self.optitrack_callback, 
                qos_profile
                )
            
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) # Publish to cmd_vel node       
        self.timer = self.create_timer(0.05, self.timer_callback)  # Runs at 20Hz. Can be changed.

        self.pose = None
        self.pose_history = deque(maxlen=100) # recent (stamp, x, y, heading), to find the pose when each scan was taken

        self.course_map = None # created at the first scan with a pose
        self.scan_count = 0 # number of scans added to the map
        self.turning = False # True while turning in place. The map skips those scans
        self.speed = 0.0 # last forward speed sent, for ramping up
        self.state = EXPLORE
        self.path = self.target = None # current path (world points) and where it leads
        self.to_end = False # True once nothing is left to explore and the target is the end of the course
        self.plan_time = -math.inf # when the path was last planned

    def sub_scan_callback(self, msg):
        """Scan subscriber. Also adds the scan to the map"""
        self.scan = msg
        # This callback runs separately from timer_callback, so the pose check there doesn't cover it
        if self.pose is None or self.turning:
            return
        if self.course_map is None:
            self.course_map = CourseMap(self.pose[0], self.pose[1])
            self.start = self.pose.copy() # only frontiers ahead of this pose count

        # The scan can wait in the queue while another callback runs, so the latest pose may be newer than the scan
        pose = self.pose_at(stamp_seconds(msg.header.stamp))
        ranges = np.array(msg.ranges)
        ranges[ranges < msg.range_min] = np.nan
        heading = np.deg2rad(pose[2])
        angles = heading + LIDAR_YAW + msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        lidar_xy = pose[:2] + LIDAR_X * np.array((np.cos(heading), np.sin(heading)))
        self.scan_count += 1
        self.course_map.update(lidar_xy, angles, ranges, CourseMap.CELL_SIZE / msg.angle_increment, self.scan_count)

    def pose_at(self, t:float) -> np.ndarray:
        '''Pose [x, y, heading] at stamp t, interpolated between the recent poses. The latest pose if t isn't between them,
        such as when the pose source stops publishing while the robot stands still, or its clock differs from the LiDAR's'''
        history = np.array(self.pose_history)
        if len(history) < 2 or not history[0, 0] <= t <= history[-1, 0]:
            return self.pose
        i = min(max(int(np.searchsorted(history[:, 0], t)), 1), len(history) - 1)
        (t0, x0, y0, h0), (t1, x1, y1, h1) = history[i - 1], history[i]
        f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return np.array((x0 + f * (x1 - x0), y0 + f * (y1 - y0), wrap_deg(h0 + f * wrap_deg(h1 - h0))))

    def yaw_from_quaternion(self, q):
        '''Returns yaw angle (in rad) for orientation based on given quaternion input q'''
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def optitrack_callback(self, msg:PoseStamped):
        '''Callback to calculate 2D pose info from Optitrack node. Pose info includes x and y coordinates, as well as heading in degrees.
        This callback will run everytime the rclpy executor spins'''
        x, y = msg.pose.position.x, msg.pose.position.y
        heading = np.rad2deg(self.yaw_from_quaternion(msg.pose.orientation))
        self.raw_pose = np.array((x,y,heading))
        x, y, heading = optitrack_to_maze(x, y, heading, self.frame) # CA2 maze frame, see optitrack_variables.config
        self.pose = np.array((x,y,heading))
        self.pose_history.append((stamp_seconds(msg.header.stamp), *self.pose))
        return self.pose

    def to_course(self, pose=None):
        '''Convert a pose [x, y, heading] (default: self.pose) from the CA2 maze frame to the final-project course frame. Optional; see [course] in optitrack_variables.config'''
        pose = self.pose if pose is None else pose
        return np.array(maze_to_course(pose[0], pose[1], pose[2], self.course)) if self.course else np.array(pose)

    def odometer_callback(self, msg):
        '''Callback to calculate 2D pose info from Gazebo odomoter. Pose info includes x and y coordinates, as well as heading in degrees.
        This callback will run everytime the rclpy executor spins'''
        latest_pose_msg = msg.pose.pose 
        heading = np.rad2deg(self.yaw_from_quaternion(latest_pose_msg.orientation))
        self.pose = np.array((latest_pose_msg.position.x, latest_pose_msg.position.y, heading))
        self.pose_history.append((stamp_seconds(msg.header.stamp), *self.pose))
        return self.pose

    def move_2D(self, x:float=0.0, y:float=0.0, turn:float=0.0):
        '''Publishes a Twist message to ROS to move a robot. Inputs are x and y linear velocities, as well as turn (z-axis yaw) angular velocity.'''
        twist_msg = Twist()
        x = np.clip(x, -self.max_translate_velocity, self.max_translate_velocity)
        y = np.clip(y, -self.max_translate_velocity, self.max_translate_velocity)
        turn = np.clip(turn, -self.max_translate_velocity*2, self.max_translate_velocity*2)
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.publisher_.publish(twist_msg)

    def timer_callback(self):
        """Controller loop. Insert path planning and PID control logic here"""
        if self.pose is None:
            print("No pose detected")
            return # Does not run if no pose received from Odom or Optitrack

        if self.calibrate and self.raw_pose is not None:
            self._ticks += 1
            if self._ticks % 10 == 0: # twice a second
                ox, oy, oh = self.raw_pose
                self.get_logger().info(f"optitrack ({ox:+.3f}, {oy:+.3f}, {oh:+7.1f} deg)  ->  maze ({self.pose[0]:+.3f}, {self.pose[1]:+.3f}, {self.pose[2]:+7.1f} deg)")
            return # Calibration mode: look, don't drive

        ###### INSERT CODE HERE ######
        if self.course_map is None:
            return # no scan has been added to the map yet
        if self.state == EXPLORE:
            self.explore()
        elif self.state == HOLD:
            self.hold()
        else:
            self.cross()
        ###### INSERT CODE HERE ######

    def explore(self):
        '''Follow the path to the target, replanning every REPLAN_PERIOD. Hold when it leads past a moving obstacle'''
        self.replan_if_due()
        if self.path is None:
            self.drive(0.0, 0.0) # nothing left to explore: stay, keep mapping and replanning
            return
        path = self.path_ahead()
        if self.find_crossing(path) is None:
            self.follow(path)
            return
        self.state = HOLD
        self.was_blocked, self.clear_since, self.windows = False, None, []
        self.get_logger().info(f"{self.state}: moving obstacle ahead")

    def hold(self):
        '''Approach a moving obstacle's sweep zone and face it. Meanwhile, time how long the crossing stays clear each
        time the obstacle leaves it, and cross at the start of a window that should be long enough.
        Everything is recomputed every tick, because the sweep zone grows while the robot watches'''
        self.replan_if_due()
        path = None if self.path is None else self.path_ahead()
        crossing = None if path is None else self.find_crossing(path)
        if crossing is None:
            self.state = EXPLORE
            self.get_logger().info(f"{self.state}: no moving obstacle ahead any more")
            return
        start, end = crossing
        out = min(end + int(np.ceil(ARRIVE_TOL / CourseMap.CELL_SIZE)), len(path) - 1) # ARRIVE_TOL past the zone's reach
        if start == 0: # already within reach, because the zone grew around the robot: leave by the shortest route
            cost, blocked, reach = self.course_map.planning_grids()
            outside = ~dilate(reach, disk(ARRIVE_TOL, CourseMap.CELL_SIZE)) & ~blocked # ARRIVE_TOL clear of reach
            cells = plan_path(cost, self.course_map.to_cell(self.pose[:2]), outside)
            if cells is not None:
                self.cross_path = self.course_map.cells_to_world(np.array(cells))
                self.state = CROSS
                self.get_logger().info(f"{self.state}: within reach of a moving obstacle, leaving")
            else:
                self.drive(0.0, 0.0) # no way out: stay put rather than keep the last command
            return

        blocked = self.crossing_blocked(path[start:end])
        window_started = self.was_blocked and not blocked
        if blocked != self.was_blocked:
            now = self.get_clock().now().nanoseconds * 1e-9
            if blocked and self.clear_since is not None:
                self.windows.append(now - self.clear_since) # a clear window just ended
            if not blocked:
                self.clear_since = now
            self.was_blocked = blocked

        if not self.follow(path[:start]):
            return # still driving to the hold point, just outside the zone's reach
        # Face the far end of the crossing. It follows the sweep zone, so unlike the carrot it barely moves when the path
        # is replanned, and the robot doesn't keep turning (and skipping scans) while it times the windows
        dx, dy = path[out] - self.pose[:2]
        alpha = math.radians(wrap_deg(math.degrees(math.atan2(dy, dx)) - self.pose[2]))
        if abs(alpha) > ALIGN_TOL:
            self.turn_in_place(alpha)
            return
        self.drive(0.0, 0.0)

        # The obstacle repeats its motion, so the window one cycle ago, two windows back, shows how long the one
        # starting now will last. Near one end of the obstacle's path, windows alternate long and short
        crossing_length = np.sum(np.linalg.norm(np.diff(path[start - 1:out + 1], axis=0), axis=1))
        cross_time = crossing_length / MAX_SPEED + MAX_SPEED / (2 * MAX_ACCEL) # + time lost ramping up
        if window_started and len(self.windows) >= 2 and self.windows[-2] >= cross_time:
            self.cross_path = path[:out + 1]
            self.state = CROSS
            self.get_logger().info(f"{self.state}: expecting {self.windows[-2]:.1f} s clear, need {cross_time:.1f} s")

    def cross(self):
        '''Drive through the crossing, or out of reach, without stopping. Then go back to exploring'''
        if self.follow(self.cross_path, safety=False, reverse=True):
            self.state = EXPLORE
            self.plan_time = -math.inf # replan at once
            self.get_logger().info(f"{self.state}: crossed")

    def replan_if_due(self):
        '''Replan if REPLAN_PERIOD has passed since the last plan'''
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.plan_time >= REPLAN_PERIOD:
            self.plan_time = now
            self.replan()

    def path_ahead(self) -> np.ndarray:
        '''The current path from the point closest to the robot'''
        return self.path[np.argmin(np.linalg.norm(self.path - self.pose[:2], axis=1)):]

    def replan(self):
        '''Plan to the current target if it is still worth reaching, otherwise to the nearest frontier.
        With no frontier left, head for the end of the course: the reachable cell farthest from the start.
        Sets self.path (world points) and self.target, both None if nothing can be reached'''
        cost, blocked, reach = self.course_map.planning_grids()
        allowed = self.course_map.ahead(self.start) & ~blocked & ~reach # where a target may be
        frontier = self.course_map.frontier(allowed)
        start = self.course_map.to_cell(self.pose[:2])
        cells = None
        if self.target is not None:
            goal = self.course_map.to_cell(self.target)
            if frontier[goal] or (self.to_end and not frontier.any()):
                cells = plan_to(cost, start, goal)
        if cells is None:
            cells = plan_path(cost, start, frontier) if frontier.any() else None
            self.to_end = False
        if cells is None:
            _, _, g = search(cost, self.course_map.to_cell(self.start[:2]))
            candidates = ((self.course_map.occupancy_map < CourseMap.FREE) & allowed).tolist()
            end = max((cell for cell in g if candidates[cell[0]][cell[1]]), key=g.get, default=None)
            cells = None if end is None else plan_to(cost, start, end)
            self.to_end = True
        self.path = None if cells is None else self.course_map.cells_to_world(np.array(cells))
        self.target = None if cells is None else self.path[-1]
        self.get_logger().debug(f"Planned in {self.get_clock().now().nanoseconds * 1e-9 - self.plan_time:.2f} s")

    def find_crossing(self, path:np.ndarray):
        '''(start, end) indices of the first stretch of path within SWEEP_CLEARANCE of the sweep zone, or None'''
        near = self.course_map.distances(self.course_map.dynamic_map >= 0, path) < SWEEP_CLEARANCE
        if not near.any():
            return None
        start = int(np.argmax(near))
        clear = np.flatnonzero(~near[start:])
        return start, (start + int(clear[0]) if len(clear) else len(path))

    def crossing_blocked(self, crossing:np.ndarray) -> bool:
        '''True if either of the last two scans saw a moving obstacle within SWEEP_CLEARANCE of the crossing (world points).
        Two, so one scan that misses the obstacle doesn't open a window that isn't there'''
        moving_now = self.course_map.dynamic_map >= self.scan_count - 1
        return bool(np.any(self.course_map.distances(moving_now, crossing) < SWEEP_CLEARANCE))

    def carrot(self, path:np.ndarray) -> tuple:
        '''Remaining path length from the robot, and the angle (rad) from its heading to the carrot:
        the first path point at least LOOKAHEAD away, after the point closest to the robot'''
        distances = np.linalg.norm(path - self.pose[:2], axis=1)
        i = int(np.argmin(distances))
        remaining = distances[i] + np.sum(np.linalg.norm(np.diff(path[i:], axis=0), axis=1))
        far = np.flatnonzero(distances[i:] >= LOOKAHEAD)
        dx, dy = (path[i + far[0]] if len(far) else path[-1]) - self.pose[:2]
        return remaining, math.radians(wrap_deg(math.degrees(math.atan2(dy, dx)) - self.pose[2]))

    def follow(self, path:np.ndarray, safety:bool=True, reverse:bool=False) -> bool:
        '''Drive along path with pure pursuit. Returns True once the robot has reached the end.
        With reverse, back up instead of turning round when the carrot is behind the robot'''
        remaining, alpha = self.carrot(path)
        if remaining < ARRIVE_TOL:
            self.drive(0.0, 0.0)
            return True
        if reverse and abs(alpha) > math.pi / 2:
            backward = math.atan2(math.sin(alpha + math.pi), math.cos(alpha + math.pi)) # the carrot's angle from the rear
            self.drive(-MAX_SPEED * math.cos(backward), 2 * MAX_SPEED * math.cos(backward) * math.sin(backward) / LOOKAHEAD)
            return False
        if abs(alpha) > TURN_IN_PLACE:
            self.turn_in_place(alpha)
            return False
        if safety and self.blocked_ahead():
            self.drive(0.0, 0.0) # wait for the map and the next plan to route around it
            return False
        # Ramp here rather than in drive(), so the turn rate matches the speed actually sent. Otherwise the robot spins
        # at the full rate while it is still pulling away, which leaves the arc and maps scans mid-spin
        speed = min(MAX_SPEED * math.cos(alpha), max(self.speed, 0.0) + MAX_ACCEL * DT)
        turn = 2 * speed * math.sin(alpha) / LOOKAHEAD # turn rate along the arc through the carrot
        if abs(turn) < MIN_TURN:
            turn = 0.0 if abs(alpha) < ALIGN_TOL else math.copysign(MIN_TURN, alpha)
        self.drive(speed, turn)
        return False

    def turn_in_place(self, alpha:float):
        '''Turn on the spot towards a heading error of alpha (rad)'''
        self.drive(0.0, math.copysign(np.clip(TURN_GAIN * abs(alpha), MIN_TURN, MAX_TURN), alpha))

    def drive(self, speed:float, turn:float):
        '''Send a forward speed (m/s) and turn rate (rad/s). Forward speed ramps up by at most MAX_ACCEL, but drops at once'''
        self.speed = min(speed, max(self.speed, 0.0) + MAX_ACCEL * DT)
        self.turning = speed == 0.0 and turn != 0.0
        self.move_2D(self.speed, 0.0, turn)

    def blocked_ahead(self) -> bool:
        '''True if a LiDAR return is within STOP_DISTANCE in front of the robot's front edge'''
        ranges = np.nan_to_num(np.array(self.scan.ranges), posinf=0.0) # no return -> 0, which never counts
        angles = LIDAR_YAW + self.scan.angle_min + np.arange(len(ranges)) * self.scan.angle_increment
        x, y = LIDAR_X + ranges * np.cos(angles), ranges * np.sin(angles) # from the robot's centre
        ahead = (x > ROBOT_LENGTH / 2) & (x < ROBOT_LENGTH / 2 + STOP_DISTANCE) # beside the body, stopping can't help
        return bool(np.any(ahead & (np.abs(y) < ROBOT_WIDTH / 2)))

class CourseMap:
    '''Two grids over the same cells, in the same frame as self.pose, that grow as the robot sees more.
    occupancy_map holds walls and static obstacles as log-odds. dynamic_map holds, for each cell,
    the number of the last scan that saw a moving obstacle there (-1 = never)'''
    CELL_SIZE = 0.025 # 2.5 cm square cell
    GROW_MARGIN = 1.0 # 1m of unknown space added when map grows

    L_OCC = 0.85
    L_FREE = -0.4
    L_MIN, L_MAX = -4.0, 4.0

    OCCUPIED = 1.0
    FREE = -1.0
    CONFIDENT_FREE = -2.0 # a hit is a moving obstacle if every cell within DYNAMIC_MARGIN of it is below this
    DYNAMIC_MARGIN = 3 # cells. Covers pose error, so a wall hit that lands a little in front of the wall isn't dynamic

    def __init__(self, start_x, start_y):
        """
           y
        4.0 ┌──────┬──────┬──────┬──────┐
            │[2,0] │[2,1] │[2,2] │[2,3] │   row 2
        3.0 ├──────┼──────┼──────┼──────┤
            │[1,0] │[1,1] │[1,2] │[1,3] │   row 1
        2.0 ├──────┼──────┼──────┼──────┤
            │[0,0] │[0,1] │[0,2] │[0,3] │   row 0
        1.0 └──────┴──────┴──────┴──────┘
            2.0    3.0    4.0    5.0    6.0  x
            col 0  col 1  col 2  col 3
        """
        self.occupancy_map = np.zeros((1, 1))
        self.dynamic_map = np.full((1, 1), -1)
        self.origin = np.array((start_x, start_y))

    def to_cells(self, xy: np.ndarray) -> np.ndarray:
        """
        World coordinate to cell coordiate.
        NOTE: since input is x, y this fn returns (col, row) that u need to flip when indexing
        """
        return np.floor((xy - self.origin) / self.CELL_SIZE).astype(int)

    def to_cell(self, xy:np.ndarray) -> tuple:
        '''One world point -> its cell as (row, col), ready for indexing'''
        col, row = self.to_cells(xy[None, :])[0]
        return int(row), int(col)

    def cells_to_world(self, cells:np.ndarray) -> np.ndarray:
        '''Cells (N, 2) as (row, col) -> world positions (N, 2) of their centres'''
        return self.origin + (cells[:, ::-1] + 0.5) * self.CELL_SIZE

    def to_flat_cells(self, xy:np.ndarray) -> np.ndarray:
        """
        returns idx of cell if matrix is flattened
        """

        cells = self.to_cells(xy)

        return np.unique(cells[:, 1] * self.occupancy_map.shape[1] + cells[:, 0])

    def grow_map_to_fit(self, xy: np.ndarray):
        cells = self.to_cells(xy)
        rows, cols = self.occupancy_map.shape

        # Keep a border of DYNAMIC_MARGIN cells, so every cell the dynamic test checks around a hit is inside the map
        m = self.DYNAMIC_MARGIN
        below = np.maximum(m - cells.min(axis = 0), 0) # cells needed before col m, row m
        above = np.maximum(cells.max(axis=0) - (cols - 1 - m, rows - 1 - m), 0) # cells needed after col, row -1-m

        if not (below.any() or above.any()):
            # no need to grow map
            return

        margin = int(self.GROW_MARGIN / self.CELL_SIZE)

        before = np.where(below > 0, below + margin, 0)
        after = np.where(above > 0, above + margin, 0)

        pad = ((before[1], after[1]), (before[0], after[0]))
        self.occupancy_map = np.pad(self.occupancy_map, pad)
        self.dynamic_map = np.pad(self.dynamic_map, pad, constant_values=-1)

        self.origin = self.origin - before * self.CELL_SIZE

    def update(self, lidar_xy: np.ndarray, angles: np.ndarray, ranges: np.ndarray, max_range: float, scan_number: int):
        valid = np.isfinite(ranges)
        angles, ranges = angles[valid], ranges[valid]

        is_hit = ranges <= max_range
        ranges = np.minimum(ranges, max_range)
        directions = np.stack((np.cos(angles), np.sin(angles)), axis = 1)
        ends = lidar_xy + ranges[:, None] * directions

        self.grow_map_to_fit(np.vstack((lidar_xy, ends)))

        steps = np.arange(0.0, max_range, self.CELL_SIZE / 2)
        before_end = steps[None, :] < ranges[:, None] - self.CELL_SIZE

        samples = lidar_xy + steps[None, :, None] * directions[:, None, :]
        free = self.to_flat_cells(samples[before_end])
        hits = self.to_flat_cells(ends[is_hit])

        grid = self.occupancy_map.reshape(-1)
        # Beams can't pass through anything static, so a hit where beams keep passing is a moving obstacle.
        # Checking the cells around it too stops hits just in front of a wall, from noise or pose error, from counting
        cols = self.occupancy_map.shape[1]
        offsets = np.arange(-self.DYNAMIC_MARGIN, self.DYNAMIC_MARGIN + 1)
        neighbourhood = (offsets[:, None] * cols + offsets).ravel()
        dynamic = np.all(grid[hits[:, None] + neighbourhood] < self.CONFIDENT_FREE, axis=1)

        grid[free] += self.L_FREE
        grid[hits[~dynamic]] += self.L_OCC

        np.clip(grid, self.L_MIN, self.L_MAX, out = grid)
        self.dynamic_map.reshape(-1)[hits[dynamic]] = scan_number

    def planning_grids(self) -> tuple:
        '''Returns (cost, blocked, reach), all the same shape as occupancy_map.
        blocked: within half the robot's width of an occupied cell, so the robot's centre shouldn't go there.
        reach: within SWEEP_CLEARANCE of the sweep zone, where a moving obstacle can touch the robot.
        cost: cost of entering each cell. inf if occupied, INFLATED if blocked, otherwise higher the closer it is to a wall'''
        occupied = self.occupancy_map > self.OCCUPIED
        blocked = dilate(occupied, disk(ROBOT_WIDTH / 2, self.CELL_SIZE))

        # Clearance: how many cells to the nearest blocked cell, counted up to the robot's width
        clear_max = int(np.ceil(ROBOT_WIDTH / self.CELL_SIZE))
        clearance = np.full(blocked.shape, clear_max)
        near = blocked
        for distance in range(clear_max):
            clearance[near & (clearance == clear_max)] = distance
            near = dilate(near, EIGHT_NEIGHBOURS)

        cost = 1.0 + CLEARANCE_WEIGHT * (clear_max - clearance) / clear_max
        cost[blocked] = INFLATED
        cost[occupied] = np.inf
        reach = dilate(self.dynamic_map >= 0, disk(SWEEP_CLEARANCE, self.CELL_SIZE))
        return cost, blocked, reach

    def ahead(self, pose:np.ndarray) -> np.ndarray:
        '''True for cells ahead of the line through pose's position at right angles to its heading'''
        rows, cols = self.occupancy_map.shape
        x = self.origin[0] + (np.arange(cols) + 0.5) * self.CELL_SIZE
        y = self.origin[1] + (np.arange(rows) + 0.5) * self.CELL_SIZE
        heading = np.deg2rad(pose[2])
        return (x[None, :] - pose[0]) * np.cos(heading) + (y[:, None] - pose[1]) * np.sin(heading) > 0

    def frontier(self, allowed:np.ndarray) -> np.ndarray:
        '''Free cells next to an unknown cell, inside allowed'''
        log_odds = self.occupancy_map
        unknown = (log_odds >= self.FREE) & (log_odds <= self.OCCUPIED)
        return (log_odds < self.FREE) & dilate(unknown, EIGHT_NEIGHBOURS) & allowed

    def distances(self, mask:np.ndarray, points:np.ndarray) -> np.ndarray:
        '''Distance from each world point (N, 2) to the centre of the nearest True cell of mask. inf if there are none'''
        if not mask.any():
            return np.full(len(points), np.inf)
        centres = self.cells_to_world(np.argwhere(mask))
        return np.min(np.linalg.norm(points[:, None] - centres[None], axis=2), axis=1)


def main(args=None):
    arg_parser = argparse.ArgumentParser(description="RB2301 final project")
    arg_parser.add_argument('--calibrate', action='store_true', help="Real robot: only print the Optitrack pose and the maze-frame pose twice a second, never drive")
    cli_args, ros_args = arg_parser.parse_known_args(args=args)
    print("Starting obstacle course")
    rclpy.init(args=ros_args)

    # Use lidar scan length to determine if in simulation or not
    lidar = LidarNode()
    while lidar.is_simulation is None:
        rclpy.spin_once(lidar)
    is_simulation = lidar.is_simulation
    lidar.destroy_node()

    # Start spinning the obstacle node and only stop once SystemExit error is raised within the node callback
    obstacle = ObstacleCourseNode(is_simulation, cli_args.calibrate)
    try:
        rclpy.spin(obstacle)
    except SystemExit:
        print("Shutting down")
    obstacle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()