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


# No map: the course is driven in stages, each from the latest scan. Stage 1 avoids obstacles heading along the first
# corridor until a way opens on the right. Stage 2 turns right and does the same until a way opens on the left.
# Then the robot turns left, goes through the door, and passes the two moving obstacles one at a time
ROBOT_WIDTH, ROBOT_LENGTH = 0.225, 0.289 # m
SIM_BODY_OFFSET, REAL_BODY_OFFSET = 0.074, 0.0 # m, body centre ahead of the pose (Gazebo's pose is the rear axle)
SIM_LIDAR_X, REAL_LIDAR_X = 0.114, 0.040 # m, LiDAR ahead of the pose
LIDAR_YAW = math.pi # rad, the LiDAR is mounted backwards, so its angle 0 points behind the robot

DT = 0.05 # s, timer period
MARGIN = 0.05 # m kept from walls and static obstacles
LOOKAHEAD = 1.0 # m. Steering counts a lane's free distance up to this: any further is just as good
STEER_COST = 0.3 # m of free distance a direction gives up per rad it is away from the wanted heading
STOP_GAP = 0.05 # m. Never drive closer than this to anything in front of the body
HEADING_GAIN = 2.0 # rad/s of turn per rad of heading error
TURN_IN_PLACE = math.radians(60) # Turn on the spot when the heading error is larger than this
ALIGN_TOL = math.radians(3) # Heading errors smaller than this count as zero
MIN_TURN = 0.3 # rad/s. Neither robot turns on the spot any slower than this
OPEN = ROBOT_LENGTH # m. A side has opened when its wall is this much further away than it was along the corridor

DOOR_LINE_UP = 1.5 * ROBOT_LENGTH # m from the door wall to the body centre, where the robot lines up on the door
DOOR_TOL = 0.05 # m. How close to the line-up point counts as there. Lining up checks the robot's lane fits the door

WAIT_GAP = 0.1 # m between the body front and a moving obstacle's line while waiting
MOVING_SPEED = 0.05 # m/s. An obstacle slower than this isn't moving, or is turning round
CROSS_SAFETY = 1.2 # Cross only if the obstacle can't be back for this many times the time the crossing takes

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
        '''First tick with a pose and a scan. The course frame is the start pose: (0, 0), facing 0 deg along the first corridor'''
        sim = self.is_simulation
        self.start_pose = np.array(self.pose)
        self.body_offset = SIM_BODY_OFFSET if sim else REAL_BODY_OFFSET
        self.lidar_x = SIM_LIDAR_X if sim else REAL_LIDAR_X
        self.cruise = 0.5 if sim else 0.25 # m/s
        self.cross_speed = 0.8 if sim else 0.3 # m/s, past the moving obstacles
        self.max_turn = 1.5 if sim else 0.6 # rad/s
        self.accel = 1.0 if sim else 0.5 # m/s^2. Speed ramps up at most this fast, but drops at once
        self.speed = 0.0
        self.side_walls = [] # (how far along this corridor, where the wall on the side being watched was)
        self.slow = 0.2 if sim else 0.1 # m/s, through the door
        self.door = None # (wall x, gap middle y, gap width) in the course frame, once seen
        self.line_up = DOOR_LINE_UP # m from the door wall to line up at, closer if something stands there
        self.obstacle = None # the moving obstacle being passed, see new_obstacle()
        self.passed = 0 # moving obstacles passed
        self.set_state("STAGE1")

    def set_state(self, state:str):
        self.get_logger().info(f"{getattr(self, 'state', None)} -> {state}")
        self.state = state
        self.side_walls = []

    def sense(self):
        '''Pose and LiDAR returns in the course frame, refreshed every tick'''
        self.now = self.get_clock().now().nanoseconds * 1e-9
        x0, y0, h0 = self.start_pose
        c0, s0 = math.cos(math.radians(h0)), math.sin(math.radians(h0))
        dx, dy = self.pose[0] - x0, self.pose[1] - y0
        self.cp = np.array((c0 * dx + s0 * dy, -s0 * dx + c0 * dy, wrap_deg(self.pose[2] - h0)))
        h = math.radians(self.cp[2])
        self.body = self.cp[:2] + self.body_offset * np.array((math.cos(h), math.sin(h)))
        self.front_x = self.body[0] + ROBOT_LENGTH / 2 # only used facing 0 deg, past the door
        self.rear_x = self.body[0] - ROBOT_LENGTH / 2

        ranges = np.asarray(self.last_scan, dtype=float)
        n = len(ranges)
        if self.is_simulation:
            angles = -3.14 + np.arange(n) * 6.28 / (n - 1) + LIDAR_YAW # URDF: 360 samples from -3.14 to 3.14
        else:
            angles = -math.pi + np.arange(n) * 2 * math.pi / n + LIDAR_YAW
        ok = np.isfinite(ranges) & (ranges > 0.06) & (ranges < 8.0)
        r, a = ranges[ok], angles[ok]
        pts = np.column_stack((self.lidar_x - self.body_offset + r * np.cos(a), r * np.sin(a))) # from the body centre
        on_robot = (np.abs(pts[:, 0]) < ROBOT_LENGTH / 2 + 0.03) & (np.abs(pts[:, 1]) < ROBOT_WIDTH / 2 + 0.03)
        self.pts = pts[~on_robot] # robot frame: x ahead, y left
        c, s = math.cos(h), math.sin(h)
        self.cpts = self.body + self.pts @ np.array(((c, s), (-s, c))) # course frame

    def heading_error(self, heading:float) -> float:
        '''Course-frame heading (deg) -> error from the robot's heading (rad)'''
        return math.radians(wrap_deg(heading - self.cp[2]))

    def lane_free(self, pts:np.ndarray, direction:float, margin:float=MARGIN) -> float:
        '''How far the robot's lane is free from the body centre, along a direction (rad) in the frame of pts'''
        along = pts[:, 0] * math.cos(direction) + pts[:, 1] * math.sin(direction)
        lateral = -pts[:, 0] * math.sin(direction) + pts[:, 1] * math.cos(direction)
        hit = (along > 0) & (np.abs(lateral) < ROBOT_WIDTH / 2 + margin)
        return float(along[hit].min()) if hit.any() else math.inf

    def steer(self, heading:float, reach:float=LOOKAHEAD) -> float:
        '''Obstacle avoidance: the direction whose lane is most open, near the wanted course-frame heading (deg).
        Returns the heading error (rad) to drive with. The heading is only the general way to go, so at a corner the
        robot doesn't turn straight to it, but to the open lane nearest it. Lanes free for reach (no more than
        LOOKAHEAD) are all as good: when heading for a point, its distance'''
        wanted = self.heading_error(heading)
        # Directions within 90 deg of the wanted heading, never back the way the robot came. Free distance counts up to
        # LOOKAHEAD, less STEER_COST for how far round a direction is, so the robot doesn't take a lane that's open but
        # leads the wrong way
        off = np.radians(np.arange(-90, 91, 3))
        free = np.array([self.lane_free(self.pts, wanted + o) for o in off])
        score = np.minimum(free, min(reach, LOOKAHEAD)) - STEER_COST * np.abs(off)
        return math.radians(wrap_deg(math.degrees(wanted + off[np.argmax(score)])))

    def drive(self, speed:float, error:float):
        '''Forward speed and turn only. Turns on the spot when the heading error is over TURN_IN_PLACE; otherwise slows
        with the heading error and with the free distance ahead. Speed ramps up by at most accel, but drops at once'''
        front = self.lane_free(self.pts, 0.0, 0.0) - ROBOT_LENGTH / 2
        if abs(error) > TURN_IN_PLACE:
            speed = 0.0
        speed = min(speed * math.cos(error) ** 2, max(0.0, 1.5 * (front - STOP_GAP)))
        turn = float(np.clip(HEADING_GAIN * error, -self.max_turn, self.max_turn))
        if speed == 0.0 and abs(error) > ALIGN_TOL and abs(turn) < MIN_TURN: # slower turns stall on the spot
            turn = math.copysign(MIN_TURN, turn)
        self.speed = speed if speed <= self.speed else min(speed, self.speed + self.accel * DT)
        self.move_2D(self.speed, 0.0, turn)

    def stop(self):
        self.speed = 0.0
        self.move_2D(0.0, 0.0, 0.0)

    def side_opened(self, heading:float, side:float) -> bool:
        '''Drive along a course-frame heading (deg) with obstacle avoidance. True once the wall on one side (side, deg:
        -90 right, +90 left) has stepped back by OPEN from where it was along this corridor, and the robot's lane that
        way is clear past it'''
        self.drive(self.cruise, self.steer(heading))
        direction = math.radians(heading + side)
        u = np.array((math.cos(direction), math.sin(direction)))
        wall = float(self.body @ u) + self.lane_free(self.cpts - self.body, direction) # how far the wall is that way, along u
        # The farthest it has been is the wall: an obstacle on that side only ever makes it look closer. Taken over at
        # least two robot lengths, so a robot that starts beside an obstacle has seen the wall past it
        along = float(self.body @ (math.cos(math.radians(heading)), math.sin(math.radians(heading))))
        seen = np.array(self.side_walls).reshape(-1, 2)
        if len(seen) and np.ptp(seen[:, 0]) >= 2 * ROBOT_LENGTH and wall > seen[:, 1].max() + OPEN:
            return True
        if abs(self.heading_error(heading)) < math.radians(15): # while going straight, where the wall was
            self.side_walls.append((along, wall))
        return False

    def find_door(self):
        '''The door in the wall ahead as (wall x, gap middle y, gap width) in the course frame, or None. Walls are
        rectilinear, so the door wall is a line of returns at one x. The door is a gap in it that the robot's lane
        fits through, with wall at least the robot's width either side, so it isn't the gap between two obstacles that
        happen to line up, and returns from beyond the wall showing through it, so it isn't a shadow of something in front'''
        P = self.cpts[(self.cpts[:, 0] > self.body[0] + 0.2) & (np.abs(self.cpts[:, 1] - self.body[1]) < 1.5)]
        if len(P) == 0:
            return None
        d = P - self.body
        bearing = np.arctan2(d[:, 1], d[:, 0])
        lines, counts = np.unique(np.round(P[:, 0] / 0.04) * 0.04, return_counts=True)
        for x_wall in lines[counts >= 8]: # nearest first
            ys = np.sort(P[np.abs(P[:, 0] - x_wall) < 0.04, 1])
            gaps = np.diff(ys)
            piece = np.concatenate(([0], np.cumsum(gaps >= ROBOT_WIDTH))) # pieces of wall: no gap the robot fits through
            length = np.array([np.ptp(ys[piece == k]) for k in range(piece[-1] + 1)])
            for i in np.flatnonzero((gaps > ROBOT_WIDTH + 2 * MARGIN) & (gaps < 3 * ROBOT_WIDTH)):
                lo, hi = ys[i], ys[i + 1]
                if min(length[piece[i]], length[piece[i + 1]]) < ROBOT_WIDTH:
                    continue
                b_lo = math.atan2(lo - self.body[1], x_wall - self.body[0])
                b_hi = math.atan2(hi - self.body[1], x_wall - self.body[0])
                through = P[(bearing > b_lo) & (bearing < b_hi), 0]
                if len(through) and np.median(through) > x_wall + 0.2:
                    return float(x_wall), float(lo + hi) / 2, float(hi - lo)
        return None

    def line_up_distance(self) -> float:
        '''How far in front of the door wall (body centre) the robot can line up: DOOR_LINE_UP, or as much less as it
        takes for the body, facing the door, to be clear of every return by MARGIN'''
        x_wall, y_gap, _ = self.door
        P = self.cpts
        beside = np.abs(P[:, 1] - y_gap) < ROBOT_WIDTH / 2 + MARGIN
        for d in np.arange(DOOR_LINE_UP, ROBOT_LENGTH / 2 + MARGIN, -0.02):
            if not (beside & (np.abs(P[:, 0] - (x_wall - d)) < ROBOT_LENGTH / 2 + MARGIN)).any():
                return float(d)
        return ROBOT_LENGTH / 2 + MARGIN

    def new_obstacle(self) -> dict:
        '''What is known about the moving obstacle ahead: its line (x), depth, half-width, recent centre-y, the speeds
        measured while it moves, its latest direction (v), and the lowest and highest its centre has been'''
        return {"x": None, "depth": 0.0, "half": 0.0, "t": [], "y": [], "speeds": [], "v": 0.0, "lo": math.inf, "hi": -math.inf}

    def track_obstacle(self):
        '''Find the moving obstacle ahead: the nearest returns ahead of the body front, between the corridor's walls.
        Keeps its recent centre-y, its speed, and how far it goes each way'''
        o, P = self.obstacle, self.cpts
        beside = P[np.abs(P[:, 0] - self.body[0]) < 0.1] # the corridor's walls, either side of the robot
        above, below = beside[beside[:, 1] > self.body[1], 1], beside[beside[:, 1] < self.body[1], 1]
        if len(above) == 0 or len(below) == 0:
            return
        P = P[(P[:, 0] > self.front_x) & (P[:, 1] < above.min() - MARGIN) & (P[:, 1] > below.max() + MARGIN)]
        if len(P) == 0:
            return
        P = P[P[:, 0] < P[:, 0].min() + 0.15] # nearest returns only, not whatever is beyond
        o["x"] = float(P[:, 0].min())
        o["depth"] = max(o["depth"], 2 * float(np.ptp(P[:, 0]))) # LiDAR only sees the near half of it
        o["half"] = max(o["half"], float(np.ptp(P[:, 1])) / 2)
        o["t"].append(self.now)
        o["y"].append(float(P[:, 1].min() + P[:, 1].max()) / 2)
        o["lo"], o["hi"] = min(o["lo"], o["y"][-1]), max(o["hi"], o["y"][-1])
        while o["t"][0] < self.now - 1.0:
            o["t"].pop(0), o["y"].pop(0)
        t, y = np.array(o["t"]), np.array(o["y"])
        recent = t > self.now - 0.5
        if recent.sum() < 4 or np.ptp(t[recent]) < 0.25:
            return
        v = float(np.polyfit(t[recent], y[recent], 1)[0])
        if abs(v) > MOVING_SPEED:
            o["v"] = v
            o["speeds"].append(abs(v))

    def crossing_safe(self, safety:float=CROSS_SAFETY) -> bool:
        '''True if the moving obstacle is moving away from the robot's lane, and can't be back in it until well after
        the robot has crossed its line. It moves back and forth, so it goes as far as it has been seen to go, but no
        further: it may turn round right away if it's further than ever'''
        o = self.obstacle
        if o["x"] is None or len(o["speeds"]) == 0:
            return False
        y, half, v = o["y"][-1], o["half"], o["v"]
        speed = float(np.percentile(o["speeds"], 90)) # its top speed, but not a noisy one
        lane_lo, lane_hi = self.body[1] - ROBOT_WIDTH / 2 - MARGIN, self.body[1] + ROBOT_WIDTH / 2 + MARGIN
        if v > 0 and y - half >= lane_hi: # above the lane, moving up
            end, back = max(o["hi"], y), lane_hi + half
        elif v < 0 and y + half <= lane_lo: # below the lane, moving down
            end, back = min(o["lo"], y), lane_lo - half
        else:
            return False # in the lane, or coming towards it
        back_in = (abs(end - y) + abs(end - back)) / speed # s until it is back in the lane
        distance = o["x"] + o["depth"] + MARGIN - self.rear_x
        crossing = distance / self.cross_speed + self.cross_speed / (2 * self.accel) # s, including speeding up
        return back_in > safety * crossing

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
            return # waits for the first LiDAR scan
        if not hasattr(self, "state"):
            self.setup_run()
        new_scan = self.last_scan is not getattr(self, "scan_ref", None)
        self.scan_ref = self.last_scan
        self.sense()

        if self.state == "STAGE1": # along the first corridor until a way opens on the right
            if self.side_opened(0.0, -90.0):
                self.set_state("STAGE2")
        elif self.state == "STAGE2": # right, then along that corridor until a way opens on the left
            if self.side_opened(-90.0, 90.0):
                self.set_state("DOOR_FIND")
        # The door: find it in the wall ahead, drive to a point straight in front of it, face it, and go straight through
        elif self.state == "DOOR_FIND": # left, avoiding obstacles, until the door shows
            self.drive(self.cruise * 0.6, self.steer(0.0))
            if new_scan:
                self.door = self.find_door()
                if self.door is not None:
                    self.set_state("DOOR_LINE_UP")
        elif self.state == "DOOR_LINE_UP":
            x_wall, y_gap, _ = self.door
            # The robot turns about its pose, so the pose goes where facing the door puts the body centre on its line
            dx, dy = x_wall - self.line_up - self.body_offset - self.cp[0], y_gap - self.cp[1]
            distance = math.hypot(dx, dy)
            if distance < DOOR_TOL:
                self.set_state("DOOR_ALIGN")
            else:
                if new_scan and x_wall - self.body[0] > DOOR_LINE_UP + 0.2: # update it while it's still well ahead
                    seen = self.find_door()
                    if seen is not None and abs(seen[0] - x_wall) < 0.1 and abs(seen[1] - y_gap) < 0.1:
                        self.door = seen
                if new_scan and distance > 0.2: # the robot's own body is filtered out, but not until it's there
                    self.line_up = min(self.line_up, self.line_up_distance())
                self.drive(min(self.cruise * 0.6, distance), self.steer(math.degrees(math.atan2(dy, dx)), distance))
        elif self.state == "DOOR_ALIGN": # turn on the spot to face through the door
            error = self.heading_error(0.0)
            self.drive(0.0, error)
            if abs(error) < ALIGN_TOL:
                x_wall, y_gap, width = self.door
                # Lined up if the robot's lane fits through the gap; otherwise go back to the line-up point
                fits = abs(self.body[1] - y_gap) < (width - ROBOT_WIDTH) / 2 - MARGIN / 2
                self.set_state("DOOR_PASS" if fits else "DOOR_LINE_UP")
        elif self.state == "DOOR_PASS": # straight through, steering onto the door's middle
            x_wall, y_gap, _ = self.door
            self.drive(self.slow, self.heading_error(math.degrees(math.atan2(y_gap - self.body[1], DOOR_LINE_UP))))
            if self.rear_x > x_wall + MARGIN:
                self.obstacle = self.new_obstacle()
                self.set_state("WAIT")

        # Stage 3: the moving obstacles, one at a time. Wait in front of each one's line, keeping the lane the robot came
        # through the door in, and cross once it's moving away and can't be back before the robot is past
        elif self.state == "WAIT":
            o = self.obstacle
            if new_scan:
                self.track_obstacle()
            to_line = self.cruise * 0.5 if o["x"] is None else 2.0 * (o["x"] - WAIT_GAP - self.front_x)
            self.drive(float(np.clip(to_line, 0.0, self.cruise)), self.heading_error(0.0))
            if self.crossing_safe():
                self.set_state("CROSS")
        elif self.state == "CROSS":
            o = self.obstacle
            if new_scan and self.front_x < o["x"]: # past its line, the nearest returns ahead are the next obstacle's
                self.track_obstacle()
            self.drive(self.cross_speed, self.heading_error(0.0))
            if self.rear_x > o["x"] + o["depth"] + MARGIN:
                self.passed += 1
                self.obstacle = self.new_obstacle()
                self.set_state("WAIT" if self.passed < 2 else "DONE")
            elif self.front_x < o["x"] - STOP_GAP and not self.crossing_safe(1.0): # it could be back first: wait again
                self.set_state("WAIT")
        else: # DONE
            self.stop()

        if self.now - getattr(self, "t_log", -math.inf) > 1.0:
            self.t_log = self.now
            o = self.obstacle
            seen = "" if o is None or o["x"] is None else \
                f" | obstacle x={o['x']:.2f} y={o['y'][-1]:+.2f} v={o['v']:+.2f} goes {o['lo']:+.2f} to {o['hi']:+.2f}"
            self.get_logger().info(f"{self.state:6s} pose ({self.cp[0]:+.2f}, {self.cp[1]:+.2f}, {self.cp[2]:+6.1f}) "
                                   f"v={self.speed:.2f}{seen}")
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