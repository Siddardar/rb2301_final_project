import argparse
import configparser
import math
import os
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

def bounce(y0:np.ndarray, vy:float, t:np.ndarray, lo:float, hi:float) -> np.ndarray:
    '''Position at times t of a point starting at y0 with speed vy that bounces back and forth between lo and hi'''
    span = max(hi - lo, 1e-3)
    m = np.mod(np.clip(y0, lo, hi) - lo + vy * t, 2 * span)
    return lo + np.where(m < span, m, 2 * span - m)


# Robot size (m). Gazebo's pose is the rear axle (rear wheels at x = 0 in the URDF), the body centre is ahead of it
ROBOT_WIDTH = 0.225
ROBOT_LENGTH = 0.289
SIM_BODY_OFFSET, REAL_BODY_OFFSET = 0.074, 0.0 # m, body centre ahead of the pose. Optitrack's rigid body is taken as the body centre
SIM_LIDAR_X, REAL_LIDAR_X = 0.114, 0.040 # m, LiDAR ahead of the pose
LIDAR_YAW = math.pi # rad, the LiDAR is mounted backwards (laser_joint yaw), so its angle 0 points behind the robot

DIRECTION_DODGE = True # Stage 2: while crossing, move towards the side the moving obstacle is leaving

DT = 0.05 # s, timer period
LOOKAHEAD = 0.6 # m, how far ahead each candidate direction has to be free
WALL_MARGIN = 0.08 # m kept from walls and static obstacles where there's room ...
MIN_MARGIN = 0.03 # m ... and where there isn't
STOP_GAP = 0.05 # m. Never drive closer than this to anything in front of (or behind, when reversing) the body
LANE_LOOK = 1.2 # m. A lane has to be clear this far ahead (or up to the wall at the end of the corridor)
LANE_CARROT = 0.45 # m. Steer at a point this far ahead in the chosen lane (closer when something is close ahead)
STUCK_TIME = 2.0 # s. Moved less than STUCK_MOVE in this long while trying to drive: turn on the spot to a clear heading and go forward,
STUCK_MOVE = 0.05 # m
UNSTICK_CLEAR = 0.3 # m ... a heading with at least this much free ahead ...
ROTATE_TIMEOUT = 4.0 # s ... turning for at most this long ...
UNSTICK_FORWARD_TIME = 1.0 # s ... then driving along it for this long
RECOVER_TIME = 1.0 # s. No clear heading to turn to: back off for this long instead
BLOCKED_TIME = 8.0 # s. A lane the robot got stuck in is avoided for this long
HEADING_GAIN = 2.0 # rad/s of turn per rad of heading error
TURN_ACCEL = 4.0 # rad/s^2. Turn rate changes at most this fast, so turns start and end smoothly
TURN_IN_PLACE = math.radians(60) # Turn on the spot when the heading error is larger than this
MIN_TURN = 0.3 # rad/s. Neither robot (real or Gazebo) turns on the spot any slower than this

CORNER_EAST_FRONT = 0.7 # m from the body centre to the wall ahead at which the first corridor turns south
CORNER_SOUTH_FRONT = 0.95 # m ... at which the second corridor turns east
OPEN_RANGE = 0.5 # m. A side counts as open when most returns in that direction are further than this ...
OPEN_JUMP = 0.3 # m ... and further than this beyond where the side wall was earlier in the corridor

DOOR_MIN, DOOR_MAX = 0.28, 0.65 # m, accepted door widths
DOOR_CARROT = 0.35 # m. Aim this far ahead along the line through the middle of the door
DOOR_LINE_UP = 0.55 # m. Line up at a point this far in front of the door (body centre)
LINE_UP_PIVOT = 20.0 # deg. When the line-up point is further off the heading than this, turn on the spot to face it
DOOR_STANDOFF = 0.3 # m from the body front to the door wall at which alignment is checked
DOOR_TOL = 0.05 # m, largest sideways error allowed into the door

OBSTACLE_HALF = 0.15 # m, half the moving obstacle's width (0.3 m box)
OBSTACLE_THICK = 0.025 # m, half its thickness
WAIT_GAP = 0.15 # m kept between the body front and a moving obstacle's path while waiting
OBSTACLE_MARGIN = 0.06 # m kept from a moving obstacle when crossing its path
CROSS_CARROT = 0.4 # m, aim this far ahead when crossing
CROSS_BRAKE = 4.0 # 1/s. Brake late and hard when crossing, so the body clears the obstacle's path quickly
DODGE_SHIFT = 0.2 # m, sideways shift towards the side the moving obstacle is leaving (DIRECTION_DODGE)
STATIC_WINDOW = 2.5 # s. Something in the way that moved less than STATIC_MOVE in this long is static, not moving
STATIC_MOVE = 0.05 # m
WAIT_TIMEOUT = 15.0 # s. Never wait longer than this (the slowest obstacle takes 12 s to come back), go round instead
LOST_TIMEOUT = 1.5 # s. Nothing seen in the way for this long: go
MIN_TRACK = 0.8 # s. Track a moving obstacle at least this long before trusting its speed

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

    def sub_scan_callback(self, msg):
        """Scan subscriber"""
        if len(msg.ranges) <= 360:
            self.last_scan = np.array(msg.ranges)
        else:
            self.last_scan = np.array(msg.ranges)[::2] 

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

    def setup_run(self):
        '''One-off set-up on the first tick: course frame, sim/real tuning and the state machine'''
        sim = self.is_simulation
        self.start_pose = np.array(self.pose) # The course frame: the start pose is (0, 0) facing 0 deg
        self.body_offset = SIM_BODY_OFFSET if sim else REAL_BODY_OFFSET
        self.lidar_x = SIM_LIDAR_X if sim else REAL_LIDAR_X
        self.cruise = 0.5 if sim else 0.25 # m/s
        self.slow = 0.2 if sim else 0.1 # m/s, through the door and when reversing
        self.cross_speed = 0.6 if sim else 0.3 # m/s, past the moving obstacles
        self.max_turn = 1.5 if sim else 0.6 # rad/s
        self.accel = 1.0 if sim else 0.5 # m/s^2
        self.slew = 90.0 if sim else 45.0 # deg/s, how fast the wanted heading swings round at a corner
        self.speed = self.turn = self.heading_cmd = 0.0
        self.scan_ref = self.moved_from = None
        self.lane = self.lane_heading = self.blocked = None # Lane planner: chosen lane, its corridor, a lane to avoid
        self.door, self.door_hits = None, 0
        self.line_up = DOOR_LINE_UP # m from the door wall to line up at (closer if something stands there)
        self.bounds = None # (bottom, top) y of the corridor past the door
        self.k = 0 # which moving obstacle is next
        self.obstacles = [self.new_obstacle(), self.new_obstacle()]
        self.t_start = self.t_state = self.t_log = self.t_stopped = self.clock()
        self.state = None
        self.set_state("CORRIDOR_EAST")

    def clock(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, state:str):
        self.get_logger().info(f"[{self.clock() - self.t_start:6.1f}s] {self.state} -> {state}")
        self.state, self.t_state = state, self.clock()
        self.t_stopped = self.t_state
        self.unstick = None # Getting unstuck: (mode, target heading, until)
        self.side_walls = [] # Where the side wall was, earlier in this corridor

    def sense(self):
        '''Pose and LiDAR returns in the course frame, refreshed every tick'''
        self.now = self.clock()
        x0, y0, h0 = self.start_pose
        c0, s0 = math.cos(math.radians(h0)), math.sin(math.radians(h0))
        dx, dy = self.pose[0] - x0, self.pose[1] - y0
        self.cp = np.array((c0 * dx + s0 * dy, -s0 * dx + c0 * dy, wrap_deg(self.pose[2] - h0)))
        h = math.radians(self.cp[2])
        c, s = math.cos(h), math.sin(h)
        self.body = self.cp[:2] + self.body_offset * np.array((c, s))
        half_x = ROBOT_LENGTH / 2 * abs(c) + ROBOT_WIDTH / 2 * abs(s) # Body's half-extent along the course's x
        self.front_x, self.rear_x = self.body[0] + half_x, self.body[0] - half_x
        self.pts = self.scan_points()
        self.cpts = self.cp[:2] + self.pts @ np.array(((c, s), (-s, c)))
        self.new_scan = self.last_scan is not self.scan_ref
        self.scan_ref = self.last_scan

    def scan_points(self) -> np.ndarray:
        '''Valid LiDAR returns as (N, 2) points in the robot frame (origin at the pose, +x forward)'''
        ranges = np.asarray(self.last_scan, dtype=float)
        n = len(ranges)
        if self.is_simulation:
            angles = -3.14 + np.arange(n) * 6.28 / (n - 1) + LIDAR_YAW # URDF: 360 samples from -3.14 to 3.14
        else:
            angles = -math.pi + np.arange(n) * 2 * math.pi / n + LIDAR_YAW
        ok = np.isfinite(ranges) & (ranges > 0.06) & (ranges < 8.0)
        r, a = ranges[ok], angles[ok]
        pts = np.column_stack((self.lidar_x + r * np.cos(a), r * np.sin(a)))
        on_robot = (np.abs(pts[:, 0] - self.body_offset) < ROBOT_LENGTH / 2 + 0.03) & (np.abs(pts[:, 1]) < ROBOT_WIDTH / 2 + 0.03)
        return pts[~on_robot]

    def sector(self, direction:float, half_width:float=10.0, percentile:float=0.0) -> float:
        '''Distance from the body centre to the returns within half_width deg of a course-frame direction (deg).
        percentile 0 is the closest; a high percentile asks "is this side mostly open?"'''
        d = self.cpts - self.body
        bearing = np.degrees(np.arctan2(d[:, 1], d[:, 0]))
        sel = np.abs((bearing - direction + 180.0) % 360.0 - 180.0) < half_width
        return float(np.percentile(np.hypot(d[sel, 0], d[sel, 1]), percentile)) if sel.any() else 10.0

    def front_free(self, ignore:np.ndarray=None, behind:bool=False) -> float:
        '''Free distance from the body's front edge (or rear edge) to the nearest return in the robot's own lane'''
        pts = self.pts if ignore is None else self.pts[~ignore]
        lane = np.abs(pts[:, 1]) < ROBOT_WIDTH / 2 + 0.01
        if behind:
            d = (self.body_offset - ROBOT_LENGTH / 2) - pts[lane, 0]
        else:
            d = pts[lane, 0] - (self.body_offset + ROBOT_LENGTH / 2)
        d = d[d > -0.03]
        return float(d.min()) if len(d) else 10.0

    def steer(self, heading:float, ignore:np.ndarray=None) -> tuple:
        '''Obstacle avoidance. Returns (direction to drive in, rad from the robot's heading; free distance that way).
        Of the directions whose swept corridor (robot width + margin, LOOKAHEAD long) is clear, takes the one closest
        to the wanted course-frame heading (deg), keeping WALL_MARGIN where there's room and MIN_MARGIN where there isn't'''
        wanted = np.clip(math.radians(wrap_deg(heading - self.cp[2])), -math.pi / 2, math.pi / 2)
        pts = self.pts if ignore is None else self.pts[~ignore]
        if len(pts) == 0:
            return wanted, LOOKAHEAD
        thetas = np.radians(np.arange(-90, 91, 3))
        px, py = pts[:, 0] - self.body_offset, pts[:, 1] # From the body centre
        c, s = np.cos(thetas)[:, None], np.sin(thetas)[:, None]
        along, lateral = px * c + py * s, -px * s + py * c
        for margin in (WALL_MARGIN, MIN_MARGIN):
            hit = (along > 0.0) & (along < LOOKAHEAD) & (np.abs(lateral) < ROBOT_WIDTH / 2 + margin)
            free = np.where(hit, along, LOOKAHEAD).min(axis=1)
            clear = free >= LOOKAHEAD
            if clear.any():
                i = int(np.argmin(np.where(clear, np.abs(thetas - wanted), np.inf)))
                return float(thetas[i]), float(free[i])
        i = int(np.argmax(free - 0.2 * np.abs(thetas - wanted))) # Nothing clear: the most open way, not too far round
        return float(thetas[i]), float(free[i])

    def lane_steer(self, heading:float):
        '''Corridor driving along a course-frame heading (deg): of the sideways positions ("lanes") between the side
        walls, take the one with the most clearance (up to WALL_MARGIN) from everything over the stretch ahead, keeping
        to the previous lane, and steer onto it smoothly. Returns the heading error (rad), or None if no lane fits'''
        h = math.radians(heading)
        c, s = math.cos(h), math.sin(h)
        d = self.cpts - self.body
        along, lat = d[:, 0] * c + d[:, 1] * s, -d[:, 0] * s + d[:, 1] * c
        # Wall at the end of the corridor: nearest return in each 5 cm strip across the way ahead. An end wall fills
        # every strip, an obstacle only some, so a high percentile over the strips is the end wall, not the obstacle
        band = (along > 0.05) & (np.abs(lat) < 0.5)
        strips = np.full(20, LANE_LOOK + 1.0)
        np.minimum.at(strips, np.clip(((lat[band] + 0.5) / 0.05).astype(int), 0, 19), along[band])
        end = float(np.percentile(strips, 80))
        reach = min(LANE_LOOK, end - 0.15)
        window = (along > -ROBOT_LENGTH / 2) & (along < reach) & (np.abs(lat) < 1.5)
        if reach < 0.2 or not window.any():
            return None
        beside = np.abs(along) < 0.3 # Lanes stay between the walls (or obstacles) right beside the robot
        left = lat[beside & (lat > 0)].min() if (beside & (lat > 0)).any() else 1.0
        right = lat[beside & (lat < 0)].max() if (beside & (lat < 0)).any() else -1.0
        offsets = np.arange(-1.0, 1.0001, 0.02)
        offsets = offsets[(offsets > right) & (offsets < left)]
        if len(offsets) == 0:
            return None
        clear = np.abs(lat[window][None, :] - offsets[:, None]).min(axis=1)
        fits = clear >= ROBOT_WIDTH / 2 + MIN_MARGIN
        if not fits.any():
            return None
        body_lat = -self.body[0] * s + self.body[1] * c # Lanes are remembered across the corridor, not from the robot
        if self.lane_heading != heading:
            self.lane_heading, self.lane = heading, None
        score = np.minimum(clear, ROBOT_WIDTH / 2 + WALL_MARGIN) - 0.15 * np.abs(offsets)
        if self.lane is not None:
            score -= 0.3 * np.abs(body_lat + offsets - self.lane)
        if self.blocked is not None and self.blocked[1] == heading and self.now < self.blocked[2]:
            score -= 0.3 * (np.abs(body_lat + offsets - self.blocked[0]) < 0.2) # Got stuck there: try another way
        i = int(np.argmax(np.where(fits, score, -np.inf)))
        self.lane = body_lat + offsets[i]
        carrot = float(np.clip(self.front_free() + 0.1, 0.2, LANE_CARROT)) # Turn harder round something close
        return self.heading_error(heading + math.degrees(math.atan2(offsets[i], carrot)))

    def heading_error(self, heading:float) -> float:
        '''Course-frame heading (deg) -> error from the robot's heading (rad)'''
        return math.radians(wrap_deg(heading - self.cp[2]))

    def slew_heading(self, target:float):
        '''Swing the wanted heading towards target at a limited rate, so corners become smooth arcs'''
        step = self.slew * DT
        self.heading_cmd = wrap_deg(self.heading_cmd + np.clip(wrap_deg(target - self.heading_cmd), -step, step))

    def drive(self, speed:float, error:float, ignore:np.ndarray=None, brake:float=1.5):
        '''Car-like motion: forward speed and turn only. Turn is a P controller on the heading error with a limited
        turn acceleration; speed slows with the heading error and with the free distance ahead, ramps up gently and
        drops at once'''
        if speed > 0:
            speed = min(speed, max(0.0, brake * (self.front_free(ignore) - STOP_GAP)))
        elif speed < 0 and self.front_free(behind=True) < STOP_GAP:
            speed = 0.0
        speed = 0.0 if abs(error) > TURN_IN_PLACE else speed * math.cos(error) ** 2
        turn = float(np.clip(HEADING_GAIN * error, -self.max_turn, self.max_turn))
        if speed == 0.0 and abs(error) > math.radians(1.5) and abs(turn) < MIN_TURN: # Slower turns stall on the spot
            turn = math.copysign(MIN_TURN, turn)
        step = TURN_ACCEL * DT
        self.turn += float(np.clip(turn - self.turn, -step, step))
        if abs(speed) > abs(self.speed) and speed * self.speed >= 0:
            self.speed = math.copysign(min(abs(speed), abs(self.speed) + self.accel * DT), speed)
        else:
            self.speed = speed
        if self.moved_from is None or math.hypot(*(self.body - self.moved_from)) > STUCK_MOVE: # Stuck = hasn't really moved
            self.moved_from, self.t_stopped = self.body.copy(), self.now
        self.move_2D(self.speed, 0.0, self.turn)

    def stop(self):
        self.speed = self.turn = 0.0
        self.move_2D(0.0, 0.0, 0.0)

    def avoid(self, heading:float, speed:float):
        '''Drive along a course-frame heading round static obstacles: in a lane on straight stretches, by the
        clearest direction while the wanted heading is still swinging round a corner or when no lane fits'''
        if self.unstick is not None and self.unstick_step(): # Getting unstuck
            return
        if self.now - self.t_stopped > STUCK_TIME:
            if self.lane is not None:
                self.blocked = (self.lane, heading, self.now + BLOCKED_TIME)
            self.lane = None
            target = self.free_heading(heading)
            if target is not None:
                self.get_logger().info(f"Stuck: turning on the spot to {target:+.0f} deg, then forward")
                self.unstick = ("rotate", target, self.now + ROTATE_TIMEOUT)
            else:
                self.get_logger().info("Stuck and no clear way to turn to: backing off")
                self.unstick = ("back", 0.0, self.now + RECOVER_TIME)
            self.t_stopped = self.now
            self.unstick_step()
            return
        error = self.lane_steer(heading) if abs(wrap_deg(self.heading_cmd - heading)) < 45.0 else None
        if error is None:
            error, free = self.steer(self.heading_cmd)
            speed *= min(1.0, free / LOOKAHEAD + 0.3)
        self.drive(speed, error)

    def free_heading(self, heading:float):
        '''Stuck: the heading (course frame, deg) to turn to on the spot so the way ahead is clear, the one nearest the
        wanted heading. Only headings the body can turn to without sweeping into anything count. None if there is none'''
        pts = self.pts
        if len(pts) == 0:
            return heading
        thetas = np.radians(np.arange(-90, 91, 5))
        c, s = np.cos(thetas)[:, None], np.sin(thetas)[:, None]
        x, y = pts[:, 0] * c + pts[:, 1] * s, -pts[:, 0] * s + pts[:, 1] * c # Returns as seen after turning by theta
        front = self.body_offset + ROBOT_LENGTH / 2
        hits = ((x > self.body_offset - ROBOT_LENGTH / 2 - 0.02) & (x < front + 0.02) &
                (np.abs(y) < ROBOT_WIDTH / 2 + 0.02)).any(axis=1) # Body would touch something at this angle
        ahead = x - front
        free = np.where((np.abs(y) < ROBOT_WIDTH / 2 + MIN_MARGIN) & (ahead > -0.03), ahead, 10.0).min(axis=1)
        zero = len(thetas) // 2
        reachable = np.zeros(len(thetas), dtype=bool) # Every angle on the way round has to be free too
        for step in (1, -1):
            i = zero
            while 0 <= i < len(thetas) and not hits[i]:
                reachable[i] = True
                i += step
        ok = reachable & (free > UNSTICK_CLEAR)
        ok[zero] = False # Straight ahead is what got stuck
        if not ok.any():
            return None
        i = int(np.argmin(np.where(ok, np.abs(thetas - self.heading_error(heading)), np.inf)))
        return wrap_deg(self.cp[2] + math.degrees(thetas[i]))

    def unstick_step(self) -> bool:
        '''One tick of getting unstuck: turn on the spot to the free heading, then drive forward along it for a moment
        (or back off straight). Returns False once done, so normal driving takes over again'''
        mode, target, until = self.unstick
        if mode == "rotate":
            error = self.heading_error(target)
            if abs(error) < math.radians(3.0) or self.now > until:
                self.unstick = ("forward", target, self.now + UNSTICK_FORWARD_TIME)
            self.drive(0.0, error)
        elif mode == "forward":
            if self.now > until or self.front_free() < STOP_GAP + 0.02:
                self.unstick = None
                self.t_stopped = self.now
                return False
            self.drive(self.cruise * 0.5, self.heading_error(target))
        else: # back
            if self.now > until:
                self.unstick = None
                self.t_stopped = self.now
                return False
            self.drive(-self.slow, 0.0)
        self.t_stopped = self.now # Recovery in progress isn't being stuck
        return True

    def corridor(self, heading:float, next_state:str, front_limit:float, open_side:float, progress:bool):
        '''Drive along a corridor with obstacle avoidance; turn into the next one once the wall ahead is near and the
        side it continues on is open'''
        self.slew_heading(heading)
        self.avoid(heading, self.cruise)
        # Where the open side's wall is, as a distance along that side's direction, sampled while going straight
        side = self.sector(open_side, 12.0, 50)
        u = np.array((math.cos(math.radians(open_side)), math.sin(math.radians(open_side))))
        wall = float(self.body @ u) + side
        opened = side > OPEN_RANGE and len(self.side_walls) >= 5 and wall > np.median(self.side_walls) + OPEN_JUMP
        if abs(wrap_deg(self.cp[2] - heading)) < 15.0 and side < 2.0 and not opened:
            self.side_walls.append(wall)
        # The wall ahead: most of a wide sector, so a static obstacle in front doesn't count
        if progress and opened and self.sector(heading, 25.0, 70) < front_limit * 1.05:
            self.set_state(next_state)

    def find_door(self):
        '''The door in the wall ahead as (wall x, gap middle y, gap width) in the course frame, or None.
        The wall facing the robot is the x with the most returns; the door is a gap of the right width along it,
        with returns from beyond the wall showing through'''
        x, y = self.body
        ahead = self.cpts[(self.cpts[:, 0] > x + 0.3) & (self.cpts[:, 0] < x + 2.5)]
        if len(ahead) < 10:
            return None
        counts = (np.abs(ahead[:, 0][:, None] - ahead[:, 0][None, :]) < 0.04).sum(axis=1)
        if counts.max() < 8:
            return None
        on_wall = np.abs(ahead[:, 0] - ahead[np.argmax(counts), 0]) < 0.04
        x_wall = float(np.median(ahead[on_wall, 0]))
        ys = np.sort(ahead[on_wall, 1])
        best = None
        for lo, hi in zip(ys[:-1], ys[1:]):
            if not DOOR_MIN < hi - lo < DOOR_MAX:
                continue
            # Something beyond the wall has to show through the gap, else it's a shadow of something in front
            b_lo, b_hi = math.atan2(lo - y, x_wall - x), math.atan2(hi - y, x_wall - x)
            d = self.cpts - self.body
            bearing = np.arctan2(d[:, 1], d[:, 0])
            through = self.cpts[(bearing > b_lo) & (bearing < b_hi), 0]
            if len(through) and np.median(through) < x_wall + 0.2:
                continue
            if best is None or abs(hi - lo - 0.4) < abs(best[2] - 0.4):
                best = (x_wall, (lo + hi) / 2, hi - lo)
        return best

    def update_door(self, locked:bool=False):
        '''Average door sightings that agree; start again on one that doesn't (unless locked on)'''
        seen = self.find_door()
        if seen is None:
            return
        if self.door is not None and abs(seen[0] - self.door[0]) < 0.1 and abs(seen[1] - self.door[1]) < 0.1:
            self.door = 0.7 * np.array(self.door) + 0.3 * np.array(seen)
            self.door_hits += 1
        elif not locked:
            self.door, self.door_hits = np.array(seen), 1

    def line_up_distance(self) -> float:
        '''How far in front of the door wall (body centre) to line up: DOOR_LINE_UP, or as much less as it takes for
        the body, facing the door, to fit clear of everything there'''
        x_wall, y_gap = self.door[0], self.door[1]
        P = self.cpts
        nearest = ROBOT_LENGTH / 2 + 0.08 # Closest it can be and still not touch the door frame
        for d in np.arange(DOOR_LINE_UP, nearest, -0.02):
            inside = (np.abs(P[:, 0] - (x_wall - d)) < ROBOT_LENGTH / 2 + 0.04) & (np.abs(P[:, 1] - y_gap) < ROBOT_WIDTH / 2 + 0.04)
            if not inside.any():
                return float(d)
        return nearest

    def new_obstacle(self) -> dict:
        '''What is known about one moving obstacle: its x, centre-y history, edges, speed, turn-round points, and the
        lanes the robot holds while waiting for it and crosses it in'''
        return {"x": None, "t": [], "y": [], "lo": None, "hi": None, "vy": 0.0, "speed": 0.0, "moving": False,
                "edges": None, "seen": None, "lane": None, "cross_lane": None}

    def update_bounds(self):
        '''Corridor's bottom and top y past the door, from the returns in a short band beside the robot (clear of the
        door wall behind and the moving obstacles ahead)'''
        P = self.cpts
        band = P[(P[:, 0] > self.body[0] - 0.05) & (P[:, 0] < self.body[0] + 0.25)]
        above, below = band[band[:, 1] > self.body[1] + 0.1, 1], band[band[:, 1] < self.body[1] - 0.1, 1]
        if len(above) < 3 or len(below) < 3:
            return
        top, bottom = float(np.median(above[above < above.min() + 0.05])), float(np.median(below[below > below.max() - 0.05]))
        if not 0.8 < top - bottom < 3.0:
            return
        self.bounds = (bottom, top) if self.bounds is None else \
            (0.8 * self.bounds[0] + 0.2 * bottom, 0.8 * self.bounds[1] + 0.2 * top)

    def track_obstacle(self):
        '''Find the next moving obstacle in the scan: a short (<= 0.45 m) cluster of returns across the corridor ahead.
        Records its edges and centre history, estimates its sideways speed and learns where it turns round'''
        o = self.obstacles[self.k]
        bottom, top = self.bounds
        P = self.cpts
        P = P[(P[:, 0] > self.body[0]) & (P[:, 0] < self.body[0] + 2.0) & (P[:, 1] > bottom + 0.05) & (P[:, 1] < top - 0.05)]
        P = P[np.argsort(P[:, 0])]
        clusters, start = [], 0
        for i in range(1, len(P) + 1):
            if i == len(P) or P[i, 0] - P[start, 0] > 0.08:
                c = P[start:i]
                if len(c) >= 3 and np.ptp(c[:, 1]) <= 0.45:
                    clusters.append(c)
                start = i
        if o["x"] is not None:
            clusters = [c for c in clusters if abs(np.median(c[:, 0]) - o["x"]) < 0.15]
        elif self.k > 0 and self.obstacles[0]["x"] is not None:
            clusters = [c for c in clusters if np.median(c[:, 0]) > self.obstacles[0]["x"] + 0.3]
        if not clusters:
            return
        c = clusters[0] # Nearest
        x_c = float(np.median(c[:, 0]))
        o["x"] = x_c if o["x"] is None else 0.8 * o["x"] + 0.2 * x_c
        lo_edge, hi_edge = float(c[:, 1].min()), float(c[:, 1].max())
        centre = (lo_edge + hi_edge) / 2
        if hi_edge - lo_edge < 2 * OBSTACLE_HALF - 0.04: # Partly hidden against a wall: go by the edge still in view
            if lo_edge < bottom + 0.1:
                centre = hi_edge - OBSTACLE_HALF
            elif hi_edge > top - 0.1:
                centre = lo_edge + OBSTACLE_HALF
        o["edges"] = (min(lo_edge, centre - OBSTACLE_HALF), max(hi_edge, centre + OBSTACLE_HALF))
        o["seen"] = self.now
        o["t"].append(self.now)
        o["y"].append(centre)
        while o["t"][0] < self.now - 3.0:
            o["t"].pop(0); o["y"].pop(0)
        t, y = np.array(o["t"]), np.array(o["y"])
        recent = t > self.now - 0.6
        if recent.sum() >= 4 and np.ptp(t[recent]) > 0.25:
            vy = float(np.polyfit(t[recent], y[recent], 1)[0])
            if o["moving"] and vy * o["vy"] < 0 and abs(o["vy"]) > 0.08: # Turned round: that's an end of its travel
                o["lo" if o["vy"] < 0 else "hi"] = float(y[recent].min() if o["vy"] < 0 else y[recent].max())
            o["vy"], o["moving"] = vy, abs(vy) > 0.05
            o["speed"] = max(o["speed"], abs(vy)) # It moves at a constant speed: slower fits straddle a turn-round

    def is_static(self) -> bool:
        '''In view for STATIC_WINDOW and moved less than STATIC_MOVE in that time: a static obstacle, don't wait for it'''
        o = self.obstacles[self.k]
        t, y = np.array(o["t"]), np.array(o["y"])
        if len(t) < 5 or t[-1] - t[0] < STATIC_WINDOW:
            return False
        window = t > self.now - STATIC_WINDOW
        return float(np.ptp(y[window])) < STATIC_MOVE

    def crossing_lane(self) -> float:
        '''y to cross at. Stage 2 (DIRECTION_DODGE): shift towards the side the obstacle is leaving'''
        o = self.obstacles[self.k]
        y = float(self.body[1])
        if DIRECTION_DODGE and o["moving"]:
            bottom, top = self.bounds
            y -= math.copysign(DODGE_SHIFT, o["vy"])
            y = float(np.clip(y, bottom + ROBOT_WIDTH / 2 + 0.1, top - ROBOT_WIDTH / 2 - 0.1))
        return y

    def crossing_safe(self, lane:float, margin:float=OBSTACLE_MARGIN) -> bool:
        '''Simulate crossing from here, at cross speed, drifting to lane, against the obstacle bouncing between its
        ends: safe if the body never overlaps its path while the obstacle is there (plus margin)'''
        o = self.obstacles[self.k]
        if o["x"] is None or o["edges"] is None or len(o["t"]) < 4 or o["t"][-1] - o["t"][0] < MIN_TRACK:
            return False
        bottom, top = self.bounds
        half = max(OBSTACLE_HALF, (o["edges"][1] - o["edges"][0]) / 2)
        lo = o["lo"] if o["lo"] is not None else bottom + half
        hi = o["hi"] if o["hi"] is not None else top - half
        # A body slanted towards the lane reaches further along and across than a straight one
        tilt = max(math.atan2(abs(lane - self.body[1]), CROSS_CARROT), math.radians(abs(self.cp[2])))
        half_x = ROBOT_LENGTH / 2 * math.cos(tilt) + ROBOT_WIDTH / 2 * math.sin(tilt)
        half_y = ROBOT_WIDTH / 2 * math.cos(tilt) + ROBOT_LENGTH / 2 * math.sin(tilt)
        t = np.arange(0.0, 5.0, DT)
        v = np.minimum(self.cross_speed, max(self.speed, 0.0) + self.accel * t)
        free, x, travelled = self.front_free(self.obstacle_mask()), np.empty(len(t)), 0.0
        for i in range(len(t)): # Braking for whatever is beyond (the end wall) slows the crossing down
            v[i] = min(v[i], max(0.0, CROSS_BRAKE * (free - travelled - STOP_GAP)))
            travelled += v[i] * DT
            x[i] = self.body[0] + travelled
        rear_done = x - half_x > o["x"] + OBSTACLE_THICK + 0.03
        if not rear_done.any():
            return False
        progress = np.clip((x - self.body[0]) / max(0.3, o["x"] - self.body[0]), 0.0, 1.0)
        y_robot = self.body[1] + (lane - self.body[1]) * progress
        vy = math.copysign(max(abs(o["vy"]), o["speed"]), o["vy"]) if o["moving"] else 0.0
        y_obstacle = bounce(np.mean(o["edges"]), vy, t, lo, hi)
        in_path = (x + half_x > o["x"] - OBSTACLE_THICK - margin) & (x - half_x < o["x"] + OBSTACLE_THICK + margin)
        overlap = np.abs(y_robot - y_obstacle) < half + half_y + margin + 0.1 * abs(o["vy"])
        until = int(np.argmax(rear_done)) + 1
        return not (in_path[:until] & overlap[:until]).any()

    def obstacle_mask(self) -> np.ndarray:
        '''Returns on the current moving obstacle's path, so the front stop doesn't brake for it mid-crossing'''
        o = self.obstacles[self.k]
        if o["x"] is None:
            return None
        return np.abs(self.cpts[:, 0] - o["x"]) < 0.08

    def wait_x(self) -> float:
        '''Body-centre x to wait at in front of the next moving obstacle'''
        return self.obstacles[self.k]["x"] - OBSTACLE_THICK - WAIT_GAP - ROBOT_LENGTH / 2

    def passed_obstacle(self) -> bool:
        '''Body's rear past the obstacle (one never seen is taken to be 0.6 m after the previous one, or the door)'''
        o = self.obstacles[self.k]
        x_o = o["x"] if o["x"] is not None else (self.obstacles[0]["x"] if self.k else self.door[0]) + 0.6
        return self.rear_x > x_o + OBSTACLE_THICK + (0.12 if self.k == len(self.obstacles) - 1 else 0.06)

    def cross(self, lane:float):
        self.obstacles[self.k]["cross_lane"] = lane
        self.set_state("OBSTACLE_CROSS")

    def next_obstacle(self):
        self.k += 1
        if self.k >= len(self.obstacles):
            self.stop()
            self.set_state("DONE")
            self.get_logger().info(f"SUCCESS: passed both moving obstacles in {self.now - self.t_start:.1f} s")
        else:
            self.set_state("OBSTACLE_WAIT")

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
        if not hasattr(self, "last_scan"):
            return # Waits for the first LiDAR scan
        if not hasattr(self, "state"):
            self.setup_run()
        self.sense()
        state = self.state

        # Section 1: zig-zag corridor with static obstacles. Avoidance steers round them; corners are smooth arcs
        if state == "CORRIDOR_EAST":
            self.corridor(0.0, "CORRIDOR_SOUTH", CORNER_EAST_FRONT, -90.0, progress=self.body[0] > 0.9)
        elif state == "CORRIDOR_SOUTH":
            self.corridor(-90.0, "TO_DOOR", CORNER_SOUTH_FRONT, 0.0, progress=self.body[1] < -1.4)

        # Section 2: small door. Find the gap in the wall ahead, line up on it, drive straight through
        elif state == "TO_DOOR":
            self.slew_heading(0.0)
            if self.new_scan:
                self.update_door()
            self.avoid(0.0, self.cruise * 0.6)
            if self.door is not None and self.door_hits >= 3:
                self.set_state("DOOR_LINE_UP" if self.door[0] - self.front_x > DOOR_STANDOFF + 0.1 else "DOOR_RETRY")
        elif state == "DOOR_LINE_UP": # Smooth arc to a point straight in front of the door
            x_wall, y_gap = self.door[0], self.door[1]
            if self.new_scan and x_wall - self.front_x > 0.5:
                self.update_door(locked=True)
            if self.new_scan: # Something standing on the line-up point: line up closer to the door instead
                self.line_up = min(self.line_up, self.line_up_distance())
            # The robot turns about its pose, so the pose goes to the point that puts the body centre on the door's line
            dx, dy = x_wall - self.line_up - self.body_offset - self.cp[0], y_gap - self.cp[1]
            distance = math.hypot(dx, dy)
            if distance < 0.04:
                self.set_state("DOOR_ALIGN")
            elif x_wall - self.front_x < self.line_up - ROBOT_LENGTH / 2 - 0.05 or self.now - self.t_stopped > STUCK_TIME:
                self.get_logger().info("Can't reach the line-up point: backing off to try again")
                self.set_state("DOOR_RETRY") # Overshot it towards the door wall, or blocked on the way
            else:
                bearing = math.degrees(math.atan2(dy, dx))
                if distance > 0.4: # Far: smooth arcs round anything that isn't the door wall
                    direction, free = self.steer(bearing, ignore=self.cpts[:, 0] > x_wall - 0.1)
                else:
                    direction = self.heading_error(bearing)
                if abs(direction) > math.radians(LINE_UP_PIVOT): # Point well off to the side: face it first, then drive
                    self.drive(0.0, direction)
                    self.t_stopped = self.now # Turning on the spot isn't being stuck
                else:
                    self.drive(min(self.cruise * 0.6, max(0.03, 1.2 * distance)), direction)
        elif state == "DOOR_ALIGN": # Turn on the spot to face through the door
            self.drive(0.0, self.heading_error(0.0))
            if abs(self.cp[2]) < 2.0 and abs(self.turn) < 0.1:
                aligned = abs(self.door[1] - self.body[1]) < DOOR_TOL
                self.set_state("DOOR_PASS" if aligned else "DOOR_RETRY")
        elif state == "DOOR_PASS":
            x_wall, y_gap = self.door[0], self.door[1]
            self.drive(self.slow, self.heading_error(math.degrees(math.atan2(y_gap - self.body[1], DOOR_CARROT))))
            if self.rear_x > x_wall + 0.05:
                self.set_state("OBSTACLE_WAIT")
            elif self.now - self.t_stopped > 2.0: # Stuck on the door frame
                self.set_state("DOOR_RETRY")
        elif state == "DOOR_RETRY": # Back off straight, then line up again
            self.drive(-self.slow, self.heading_error(0.0))
            if self.body[0] < self.door[0] - self.line_up - 0.3 or self.now - self.t_stopped > 2.0:
                self.set_state("DOOR_LINE_UP")

        # Section 3: moving obstacles, one at a time. Wait in front of each and cross when the simulated crossing is
        # clear; something that doesn't move (or a wait that is too long) is gone round instead
        elif state in ("OBSTACLE_WAIT", "OBSTACLE_CROSS", "OBSTACLE_AVOID"):
            if self.new_scan:
                self.update_bounds()
                if self.bounds is not None:
                    self.track_obstacle()
            if self.bounds is None and self.now - self.t_state > 1.0: # Can't measure it: assume a 1.6 m corridor
                self.bounds = (float(self.body[1]) - 0.8, float(self.body[1]) + 0.8)
                self.get_logger().info("Corridor width not measured: assuming 1.6 m")
            if self.bounds is None:
                self.stop()
                return
            o = self.obstacles[self.k]
            if state == "OBSTACLE_WAIT": # Drive to the waiting point in front of it (holding the lane), cross when clear
                if o["lane"] is None:
                    o["lane"] = float(self.body[1])
                hold = self.heading_error(math.degrees(math.atan2(o["lane"] - self.body[1], CROSS_CARROT)))
                to_wait = self.slow if o["x"] is None else 2.0 * (self.wait_x() - self.body[0])
                self.drive(float(np.clip(to_wait, -self.slow, self.cruise)), hold)
                lane = self.crossing_lane()
                if self.now - (o["seen"] or self.t_state) > LOST_TIMEOUT:
                    self.get_logger().info(f"Obstacle {self.k + 1} not in sight: crossing")
                    self.cross(float(self.body[1]))
                elif self.is_static():
                    self.get_logger().info(f"Obstacle {self.k + 1} has not moved for {STATIC_WINDOW} s: static, going round it")
                    self.set_state("OBSTACLE_AVOID")
                elif self.now - self.t_state > WAIT_TIMEOUT:
                    self.get_logger().info(f"Waited {WAIT_TIMEOUT} s for obstacle {self.k + 1}: going round it")
                    self.set_state("OBSTACLE_AVOID")
                elif o["moving"] and self.crossing_safe(lane):
                    side = "none" if lane == float(self.body[1]) else ("left" if lane > self.body[1] else "right")
                    self.get_logger().info(f"Obstacle {self.k + 1} moving {'left' if o['vy'] > 0 else 'right'} "
                                           f"({o['vy']:+.2f} m/s): crossing, shift {side}")
                    self.cross(lane)
            elif state == "OBSTACLE_CROSS":
                heading = math.degrees(math.atan2(o["cross_lane"] - self.body[1], CROSS_CARROT))
                self.drive(self.cross_speed, self.heading_error(heading), ignore=self.obstacle_mask(), brake=CROSS_BRAKE)
                if self.passed_obstacle():
                    if self.k + 1 < len(self.obstacles):
                        self.obstacles[self.k + 1]["lane"] = o["cross_lane"]
                    self.next_obstacle()
                elif o["x"] is not None and self.front_x < o["x"] - OBSTACLE_THICK - 0.02 \
                        and not self.crossing_safe(o["cross_lane"], margin=OBSTACLE_MARGIN / 2): # Hysteresis
                    self.get_logger().info("Obstacle coming back before the robot is in its path: wait again")
                    self.set_state("OBSTACLE_WAIT")
            else: # OBSTACLE_AVOID
                self.heading_cmd = 0.0
                self.avoid(0.0, self.slow)
                if self.passed_obstacle():
                    self.next_obstacle()

        elif state == "DONE":
            self.stop()

        if self.now - self.t_log > 1.0:
            self.t_log = self.now
            o = self.obstacles[min(self.k, 1)]
            door = "" if self.door is None else f" door x={self.door[0]:.2f} y={self.door[1]:.2f} w={self.door[2]:.2f}"
            obs = "" if o["x"] is None else f" obstacle{min(self.k, 1) + 1} x={o['x']:.2f} vy={o['vy']:+.2f}"
            self.get_logger().info(f"{self.state:17s} pose ({self.cp[0]:+.2f}, {self.cp[1]:+.2f}, {self.cp[2]:+6.1f})"
                                   f" v={self.speed:.2f}{door}{obs}")
        ###### INSERT CODE HERE ######
                

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