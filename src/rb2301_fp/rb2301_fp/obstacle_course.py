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

# Obstacle course, leg by leg (no map): forward along the first corridor, turn right, forward along the second, turn
# left back to the starting heading (saved at the start: it is the direction the gate is entered in), find the gate's
# gap with the LiDAR, slide sideways to line up with it and drive straight through. Past the gate the heading is held
# and the robot only moves forward, left and right: it stops in front of each swinging obstacle, waits until it has
# swung clear, slides away from it and dashes past. Done after two.
ROBOT_WIDTH, ROBOT_LENGTH = 0.225, 0.289 # m
SIM_BODY_OFFSET, REAL_BODY_OFFSET = 0.074, 0.0 # m, body centre ahead of the pose (Gazebo's pose is the rear axle)
SIM_LIDAR_X, REAL_LIDAR_X = 0.114, 0.040 # m, LiDAR ahead of the pose
LIDAR_YAW = math.pi # rad. The LiDAR is mounted backwards: its angle 0 points behind the robot
MARGIN = ROBOT_WIDTH / 4 # m, clearance kept from walls and obstacles
CRUISE = 0.5 # m/s, normal speed (capped by the robot's own limit)
ACCEL = 1.0 # m/s^2, speeds change at most this fast
TURN_RATE = 1.0 # rad/s, top turn rate when rotating on the spot
TURNS = [-90.0, 0.0] # deg: after leg 1 turn right; after leg 2 turn back to the starting heading

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
        '''First tick: save the starting heading and set up'''
        self.start_heading = float(self.pose[2])
        self.body_offset = SIM_BODY_OFFSET if self.is_simulation else REAL_BODY_OFFSET
        self.lidar_x = SIM_LIDAR_X if self.is_simulation else REAL_LIDAR_X
        self.cruise = min(CRUISE, self.max_translate_velocity)
        self.dash_speed = min(2 * CRUISE, self.max_translate_velocity) # Past the moving obstacles
        self.dt = self.timer.timer_period_ns * 1e-9
        self.state, self.leg, self.target = "LEG", 0, self.start_heading
        self.vx = self.vy = 0.0
        self.play = None # How far the body can move across a corridor (its width less the body and margins), measured
        self.open_from = None # Where the side opening to the next corridor starts, along the leg
        self.way_on = True
        self.margin = MARGIN # Halved only to squeeze through a gap too narrow for the full margin
        self.in_gate = False
        self.watch, self.crossed = [], 0
        self.t_start = self.t_log = self.t_state = self.clock()

    def clock(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, state:str, message:str):
        self.state, self.t_state = state, self.now
        self.get_logger().info(f"[{self.now - self.t_start:5.1f}s] {message}")

    def scan_points(self):
        '''LiDAR returns relative to the body centre, robot frame (x forward, y left), without the robot itself'''
        r = np.asarray(self.last_scan, dtype=float)
        n = len(r)
        a = (-3.14 + np.arange(n) * 6.28 / (n - 1) if self.is_simulation else -math.pi + np.arange(n) * 2 * math.pi / n) + LIDAR_YAW
        ok = np.isfinite(r) & (r > 0)
        p = np.column_stack((self.lidar_x + r[ok] * np.cos(a[ok]) - self.body_offset, r[ok] * np.sin(a[ok])))
        self.pts = p[~((np.abs(p[:, 0]) < ROBOT_LENGTH / 2) & (np.abs(p[:, 1]) < ROBOT_WIDTH / 2))]

    def free_ahead(self, offset:float=0.0, margin:float=None) -> float:
        '''How far the front can go straight ahead, keeping the margin, with the body shifted sideways by offset'''
        margin = self.margin if margin is None else margin
        p = self.pts
        hit = (np.abs(p[:, 1] - offset) < ROBOT_WIDTH / 2 + margin) & (p[:, 0] > ROBOT_LENGTH / 2)
        return float(p[hit, 0].min() - ROBOT_LENGTH / 2 - margin) if hit.any() else np.inf

    def side_room(self, side:int) -> float:
        '''How far the body can slide to the left (side 1) or right (side -1), keeping the margin'''
        p = self.pts
        beside = (np.abs(p[:, 0]) < ROBOT_LENGTH / 2) & (side * p[:, 1] > 0)
        return float((side * p[beside, 1]).min() - ROBOT_WIDTH / 2 - self.margin) if beside.any() else np.inf

    def heading_error(self, target:float) -> float:
        return math.radians(wrap_deg(target - self.pose[2]))

    def stop_speed(self, room:float, top:float, accel:float=ACCEL) -> float:
        '''Speed that can still stop within room'''
        return min(top, math.sqrt(2 * accel * max(room, 0.0)))

    def dash_time(self, distance:float, top:float, accel:float) -> float:
        '''Time to cover distance from a standstill to a stop'''
        if distance < top ** 2 / accel:
            return 2 * math.sqrt(distance / accel)
        return distance / top + top / accel

    def go(self, forward:float, sideways:float, turn:float=None, accel:float=ACCEL):
        '''Drive with smooth speed changes; holds the target heading unless a turn rate is given'''
        step = accel * self.dt
        self.vx += float(np.clip(forward - self.vx, -step, step))
        self.vy += float(np.clip(sideways - self.vy, -step, step))
        if turn is None:
            turn = 2.0 * self.heading_error(self.target)
        self.move_2D(self.vx, self.vy, turn)

    def slide_speed(self, offset:float) -> float:
        '''Sideways speed towards offset (m, left positive), slowing to stop before anything beside'''
        side = 1 if offset > 0 else -1
        return side * min(self.stop_speed(self.side_room(side), self.cruise / 2), 2.0 * abs(offset))

    def lane_offset(self, look:float) -> float:
        '''Sideways offset to drive at: centred between the walls, or the nearest offset with a clear way past an
        obstacle in the way (keeping half the margin if the gap is too narrow for the full one)'''
        self.margin = MARGIN
        left, right = self.side_room(1), self.side_room(-1)
        if max(left, right) < 3 * ROBOT_LENGTH: # Between two walls: measure the corridor and keep to its middle
            self.play = max(self.play or 0.0, left + right)
            centre = (left - right) / 2
        else:
            centre = 0.0
        self.way_on = True # Whether there is a way past whatever is ahead
        if self.free_ahead(0.0) >= look:
            return centre
        for margin in (MARGIN, MARGIN / 2):
            self.margin = margin
            reach = (-min(self.side_room(-1), 1.0), min(self.side_room(1), 1.0))
            offsets = np.arange(reach[0], reach[1] + 0.005, 0.005)
            free = np.array([self.free_ahead(o) for o in offsets])
            clear = free >= look
            if clear.any(): # In each clear run, the offset nearest the centre but a little inside the run's edges
                runs = [r for r in np.split(np.arange(len(offsets)), np.where(np.diff(clear.astype(int)) != 0)[0] + 1) if clear[r[0]]]
                inset = [min(MARGIN / 2, np.ptp(offsets[r]) / 2) for r in runs]
                aims = [float(np.clip(centre, offsets[r[0]] + i, offsets[r[-1]] - i)) for r, i in zip(runs, inset)]
                return min(aims, key=lambda o: abs(o - centre))
        self.way_on, self.margin = False, MARGIN
        return float(offsets[np.argmax(free)]) if len(offsets) else 0.0

    def gate_offset(self):
        '''The gap in the wall ahead: the middle of the run of sideways offsets whose straight way reaches well past
        the wall (the wall's own distance is the typical one). None if there is no gap the body fits through'''
        offsets = np.arange(-1.0, 1.0, 0.01)
        free = np.array([self.free_ahead(o) for o in offsets])
        wall = float(np.median(free[np.isfinite(free)])) if np.isfinite(free).any() else np.inf
        through = free > wall + ROBOT_LENGTH
        if not through.any():
            return None, wall
        runs = np.split(np.arange(len(offsets)), np.where(np.diff(through.astype(int)) != 0)[0] + 1)
        runs = [r for r in runs if through[r[0]]]
        best = min(runs, key=lambda r: abs(offsets[r].mean()))
        return float(offsets[best].mean()), wall

    def obstacle_ahead(self):
        '''The nearest thing ahead between the side walls: (distance from the front, its left and right edges)'''
        left, right = self.side_room(1), self.side_room(-1)
        p = self.pts
        x = p[:, 0] - ROBOT_LENGTH / 2
        inside = (x > 0) & (p[:, 1] < left + ROBOT_WIDTH / 2) & (p[:, 1] > -right - ROBOT_WIDTH / 2)
        if not inside.any():
            return None
        line = x[inside].min()
        face = inside & (x < line + MARGIN)
        return float(line), float(p[face, 1].min()), float(p[face, 1].max())

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
        if not hasattr(self, "t_start"):
            self.setup_run()
        self.now = self.clock()
        self.scan_points()
        self.margin = MARGIN
        h0 = math.radians(self.start_heading)
        along = math.cos(h0) * self.pose[0] + math.sin(h0) * self.pose[1] # Position along / across the starting heading
        across = -math.sin(h0) * self.pose[0] + math.cos(h0) * self.pose[1]
        left, right = self.side_room(1), self.side_room(-1)
        look = ROBOT_LENGTH + self.cruise ** 2 / (2 * ACCEL) # Far enough ahead to stop and still slide round

        if self.state == "DONE":
            self.move_2D(0.0, 0.0, 0.0)
            return

        if self.state == "LEG": # Forward along a corridor, centred, sliding round obstacles, until it turns
            side = -1 if self.leg == 0 else 1 # The side the next corridor opens to
            h = math.radians(self.target)
            along_leg = math.cos(h) * self.pose[0] + math.sin(h) * self.pose[1]
            offset = self.lane_offset(look) # (May halve the margin for a narrow gap)
            ahead = self.free_ahead()
            if self.open_from is None and self.play is not None and self.side_room(side) > self.play + MARGIN:
                self.open_from = along_leg - ROBOT_LENGTH / 2 # The opening's near edge has just passed the body's rear
            if self.open_from is None and ahead < 0.02 and not self.way_on and self.side_room(side) > self.side_room(-side):
                self.open_from = along_leg # Blocked right across: the end wall, its opening hidden by something
            to_stop = ahead
            if self.open_from is not None: # Stop with the body in the middle of the opening (its far edge: the wall ahead)
                to_stop = min(ahead, (self.open_from + along_leg + ROBOT_LENGTH / 2 + MARGIN + ahead) / 2 - along_leg)
                if to_stop < 0.02 and abs(self.vx) < 0.05:
                    self.target, self.open_from = self.start_heading + TURNS[self.leg], None
                    self.set_state("ROTATE", f"End of leg {self.leg + 1}: turning {'right' if side < 0 else 'left'}")
                    return
            self.go(self.stop_speed(to_stop, self.cruise), self.slide_speed(offset))

        elif self.state == "ROTATE": # On the spot, once there's room to swing the body round
            axle = np.array((-self.body_offset, 0.0)) # The robot turns about its pose
            swing = math.hypot(ROBOT_LENGTH / 2 + self.body_offset, ROBOT_WIDTH / 2)
            d = np.hypot(*(self.pts - axle).T)
            if d.min() < swing + MARGIN / 2: # Too close to something: move away from it first
                away = -(self.pts[np.argmin(d)] - axle) / d.min()
                self.go(*(away * self.cruise / 4), 0.0)
                return
            err = self.heading_error(self.target)
            if abs(err) < math.radians(1.5):
                self.vx = self.vy = 0.0
                self.leg += 1
                if self.leg == 1:
                    self.set_state("LEG", "Leg 2: forward")
                else:
                    self.set_state("GATE", "Facing the starting heading: looking for the gate")
                return
            self.go(0.0, 0.0, math.copysign(min(max(2.0 * abs(err), TURN_RATE / 4), TURN_RATE), err)) # Never too slow to turn

        elif self.state == "GATE": # Line up with the gap (sideways only, starting heading held) and drive through
            if max(left, right) < ROBOT_WIDTH / 2: # Between the gate's edges: keep centred, straight on
                self.in_gate = True
                self.go(self.stop_speed(self.free_ahead(), self.cruise / 2), self.slide_speed((left - right) / 2))
                return
            if self.in_gate: # Out the other side
                self.watch = []
                self.set_state("MOVERS", "Through the gate: watching for the moving obstacles")
                return
            offset, wall = self.gate_offset()
            if offset is None or not (-right <= offset <= left): # Not seen (or not reachable) yet: keep going
                self.go(self.stop_speed(self.free_ahead() - ROBOT_LENGTH, self.cruise), self.slide_speed(self.lane_offset(look)))
                return
            aligned = abs(offset) < MARGIN / 4
            forward = self.free_ahead() if aligned else self.free_ahead() - ROBOT_LENGTH / 2 # Not lined up: stop short of the wall
            self.go(self.stop_speed(forward, self.cruise / 2), 0.0 if aligned else self.slide_speed(offset))

        elif self.state == "MOVERS": # Starting heading held: only forward, left and right from here
            seen = self.obstacle_ahead()
            if seen is None: # Nothing ahead
                self.go(self.stop_speed(self.free_ahead(), self.cruise), 0.0)
                return
            line, lo, hi = seen
            if line > 2 * MARGIN: # Creep up to just short of its path first: less to cross
                self.watch = []
                self.go(self.stop_speed(line - MARGIN, self.cruise / 2), 0.0)
                return
            self.go(0.0, 0.0) # Stand still and watch it
            if abs(self.vx) > 0.02 or abs(self.vy) > 0.02:
                return
            self.watch = [w for w in self.watch if self.now - w[0] < 1.0] + [(self.now, (lo + hi) / 2)]
            t, centre = np.array(self.watch).T
            if np.ptp(t) < 0.8:
                return
            if np.ptp(centre) < MARGIN: # Not moving: a static obstacle, slide round it
                self.go(self.stop_speed(self.free_ahead(), self.cruise / 2), self.slide_speed(self.lane_offset(line + ROBOT_LENGTH)))
                return
            v = float(np.polyfit(t - t[0], centre, 1)[0])
            band = ROBOT_WIDTH / 2 + 2 * MARGIN # Half the robot's lane
            going = 1 if v > 0 else -1 # The side it is swinging to
            lead, trail = (hi, lo) if going > 0 else (-lo, -hi) # Its edges, measured towards that side
            wall = self.side_room(going) + ROBOT_WIDTH / 2 + MARGIN # Where it must turn round, at the latest
            back_in = (2 * max(wall - lead, 0.0) + trail - band) / abs(v) # Until it is back in the lane
            distance = line + ROBOT_LENGTH + 2 * MARGIN # Until the body is past its path
            need = self.dash_time(distance, self.dash_speed, 2 * ACCEL)
            if trail > band and back_in > need + 2 * MARGIN / abs(v): # Out of the lane, leaving, and can't be back in time
                self.slide = (-going, across, min(self.side_room(-going), ROBOT_WIDTH / 2))
                self.dash_to = along + distance
                self.set_state("DASH", f"Moving obstacle swinging {'left' if going > 0 else 'right'} at {abs(v):.2f} m/s "
                                       f"is clear for {back_in:.1f} s ({need:.1f} s needed): going, sliding "
                                       f"{'right' if going > 0 else 'left'} away from it")

        elif self.state == "DASH": # Straight past the obstacle's line, braking for anything in the way
            if self.dash_to - along < 0.01:
                self.crossed += 1
                if self.crossed >= 2:
                    self.move_2D(0.0, 0.0, 0.0)
                    self.set_state("DONE", f"SUCCESS: past both moving obstacles in {self.now - self.t_start:.1f} s")
                    return
                self.watch = []
                self.set_state("MOVERS", "Past moving obstacle 1: watching for the next")
                return
            room = min(self.free_ahead(), self.dash_to - along) # Stops just past it: the next one may be close behind
            side, start, goal = self.slide # Sliding away from where it went, as it goes
            moved = abs(across - start)
            slide = side * self.stop_speed(min(goal - moved, self.side_room(side)), self.cruise / 2) if moved < goal else 0.0
            self.go(self.stop_speed(room, self.dash_speed, 2 * ACCEL), slide, accel=2 * ACCEL)

        if self.now - self.t_log > 1.0:
            self.t_log = self.now
            self.get_logger().info(f"{self.state} leg {self.leg + 1} pose ({self.pose[0]:+.2f}, {self.pose[1]:+.2f}, "
                                   f"{self.pose[2]:+5.0f}) speed ({self.vx:+.2f}, {self.vy:+.2f}) ahead {min(self.free_ahead(), 9.9):.2f}")
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