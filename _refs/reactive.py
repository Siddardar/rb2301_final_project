import argparse
import configparser
import math
import os
import subprocess
import threading
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


# ---------------------------------------------------------------------------------------------------------------------
# Part 1: drive down the first corridor, find the right turn, go through it and face down the new corridor (plan.md).
# Scan-based: no map. Everything is in the start frame: origin at the start, +x the start heading, angles
# counter-clockwise. No constant describes the course.

# The robot
ROBOT_LENGTH, ROBOT_WIDTH = 0.289, 0.225 # m, CA1
BODY_X = 0.07 # m, body centre ahead of the pose (simulation: the pose is the rear axle)
LIDAR_X, LIDAR_YAW = 0.114, math.pi # m, rad: the LiDAR sits ahead of the pose, mounted backwards (URDF laser_joint)

# What we choose
SPEED = 0.2 # m/s, forward speed
LOOKAHEAD = 0.4 # m, pure pursuit: turn rate = 2 * SPEED * sin(angle off) / LOOKAHEAD, forward speed SPEED * cos
MARGIN = 0.05 # m kept clear on each side of the body
STRIP = 0.3 # m, how far ahead of the body a heading has to be clear. Longer blocks every heading angled at a near wall
HEADING_STEP = 5.0 # deg between the candidate headings
FORWARD = 45.0 # deg: if no heading this close to the current one is clear, search for a gap instead (last resort)
WINDOW = 5.0 # deg either side of straight ahead / straight right (start frame) for the beams used to find the turn
JUMP = 2.0 # a beam straight right reaching this many times further right than anything on the first scan: the turn
HEADING_TOL = 45.0 # deg: through the turn and this close to -90 counts as heading down the new corridor
STRAFE_SPEED = 0.1 # m/s, slow, so the robot can stop inside a gap only a little wider than itself
STUCK_TIME, STUCK_DIST = 3.0, 0.05 # s, m: moving less than this in this long means stuck
AHEAD_TIME = 0.5 # s: before turning, check the body (plus MARGIN) still fits where it will be this far ahead


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

        # Part 1
        self.start = None # pose on the first tick, in the pose's own frame
        self.points = None # latest scan's points, start frame
        self.reach = None # how far right anything reached on the first scan (start frame), normally the right wall
        self.end_x = -math.inf # furthest x reached straight ahead so far: the end wall (obstacles are always nearer)
        self.corner_x = None # x of the first beam straight right that passed the jump test: the turn's near corner
        self.right_hits = [] # y of every hit by a beam straight right, before the turn is found
        self.wall_y = None # the right wall's line: the median of right_hits, set when the turn is found
        self.aligning = False # past the waypoint: turning to face -90 deg
        self.chosen = 0.0 # heading chosen on the last tick, deg
        self.search = None # gap search in progress (last resort): {"heading": wanted, "side": +1 left / -1 right, "switched"}
        self.last_move = None # (time, x, y) when the robot last moved STUCK_DIST
        # Part 2
        self.section = 1
        self.door = None # (centre x, centre y, direction through it as a unit vector x, y), start frame
        self.door_locked = False # within LOOKAHEAD of the door's line: stop updating it
        self.shown = None # waypoint last drawn in Gazebo
        self.drawing = False # a `gz service` call to draw it is running

    def sub_scan_callback(self, msg):
        """Scan subscriber"""
        ranges = np.array(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        if len(ranges) > 360:
            ranges, angles = ranges[::2], angles[::2]
        self.last_scan = ranges
        if self.pose is None or self.start is None or self.calibrate:
            return
        valid = np.isfinite(ranges) & (ranges > msg.range_min)
        x, y, heading = self.start_frame()
        h = math.radians(heading)
        lx, ly = x + LIDAR_X * math.cos(h), y + LIDAR_X * math.sin(h)
        a = angles[valid] + h + LIDAR_YAW
        ex, ey = lx + ranges[valid] * np.cos(a), ly + ranges[valid] * np.sin(a)
        self.points = np.stack((ex, ey), axis=1)

        if self.reach is None: # first scan: how far right anything reaches
            self.reach = float(-ey.min())
            self.get_logger().info(f"Furthest right on the first scan: {self.reach:.3f} m")
        if self.section == 2:
            self.find_door(ex, ey, a)
        elif not self.aligning: # find the turn's two edges
            direction = np.degrees(a) % 360 # beam directions in the start frame, 0 = straight ahead
            ahead = (direction <= WINDOW) | (direction >= 360 - WINDOW)
            if ahead.any():
                self.end_x = max(self.end_x, float(ex[ahead].max()))
            right = abs(direction - 270) <= WINDOW
            far_right = right & (ey < -JUMP * self.reach)
            if self.corner_x is None:
                self.right_hits.extend(ey[right & ~far_right].tolist())
            if far_right.any() and self.corner_x is None:
                self.corner_x = float(ex[far_right].min())
                # Most beams straight right hit the wall; hits on obstacles beside the robot, or on things seen through
                # the opening just before the corner, are too few to move the median
                self.wall_y = float(np.median(self.right_hits))
                self.get_logger().info(f"Right turn: corner at x = {self.corner_x:.2f}, end wall at x = {self.end_x:.2f}, "
                                       f"right wall at y = {self.wall_y:.2f}")

    def find_door(self, ex, ey, a):
        '''Section 2: the door is a notch in the left wall, a run of beams reaching more than ROBOT_LENGTH further left
        than the beams either side of it (the posts), with both posts on the left wall and far enough apart for the
        robot plus margins. A set-back wall (the
        simulation) is only a step up, never a notch, so the same test works whether or not the door is flush'''
        if self.door_locked:
            return
        # The corridor's left wall: section 1's end wall carries on as the new corridor's left wall after a right turn.
        # Measured as the furthest point straight ahead, so obstacles beside the robot can't pull it nearer
        left_x = self.end_x
        direction = (np.degrees(a) + 180) % 360 - 180 # start frame, 0 = straight left of the corridor
        side = np.flatnonzero(abs(direction) < 90) # every beam pointing left, in angle order
        side = side[np.argsort(direction[side])]
        x, y = ex[side], ey[side]
        groups = np.split(np.arange(len(x)), np.flatnonzero(abs(np.diff(x)) > ROBOT_LENGTH) + 1)
        bx, by, _ = self.body()
        best = None
        for p in range(len(groups) - 2):
            post1 = groups[p][-1]
            if x[post1] < left_x - MARGIN: # a post has to be on the left wall
                continue
            for q in range(p + 1, len(groups)):
                if x[groups[q]].min() > x[post1] + ROBOT_LENGTH: # still deeper than the first post: inside the notch
                    continue
                post2 = groups[q][0]
                inner = np.concatenate(groups[p + 1:q]) if q > p + 1 else np.array([], int)
                # Both posts on the same wall (the corridor's end can hide a post from upstream, and then pair with
                # the far one), and the notch deeper than both
                # ... and the posts far enough apart for the robot plus margins (a gap between wall panels isn't a door)
                if (len(inner) and x[post2] >= left_x - MARGIN and abs(x[post2] - x[post1]) <= ROBOT_LENGTH
                        and x[inner].min() > x[post2] + ROBOT_LENGTH
                        and math.hypot(x[post2] - x[post1], y[post2] - y[post1]) >= ROBOT_WIDTH + 2 * MARGIN):
                    cx, cy = (x[post1] + x[post2]) / 2, (y[post1] + y[post2]) / 2
                    nx, ny = y[post2] - y[post1], -(x[post2] - x[post1]) # perpendicular to the posts' line
                    norm = math.hypot(nx, ny)
                    nx, ny = (nx, ny) if (cx - bx) * nx + (cy - by) * ny > 0 else (-nx, -ny) # away from the robot
                    if best is None or math.hypot(cx - bx, cy - by) < math.hypot(best[0] - bx, best[1] - by):
                        best = (cx, cy, nx / norm, ny / norm)
                break
        if best is not None:
            if self.door is None:
                self.get_logger().info(f"Door at ({best[0]:.2f}, {best[1]:.2f}), left wall at x = {left_x:.2f}")
            self.door = best

    def start_frame(self):
        '''Pose in the start frame'''
        x0, y0, h0 = self.start
        r = math.radians(h0)
        dx, dy = self.pose[0] - x0, self.pose[1] - y0
        return math.cos(r) * dx + math.sin(r) * dy, -math.sin(r) * dx + math.cos(r) * dy, wrap_deg(self.pose[2] - h0)

    def body(self):
        '''Body centre (x, y, heading deg) in the start frame'''
        x, y, heading = self.start_frame()
        return x + BODY_X * math.cos(math.radians(heading)), y + BODY_X * math.sin(math.radians(heading)), heading

    def waypoint(self):
        '''Middle of the right turn: halfway between its near corner and the end wall, on the right wall's line'''
        return None if self.corner_x is None else ((self.corner_x + self.end_x) / 2, self.wall_y)

    def free(self, direction, half_width):
        '''How far it's clear from the body centre in this direction (deg, start frame): the nearest scan point in a
        strip half_width either side'''
        bx, by, _ = self.body()
        r = math.radians(direction)
        dx, dy = self.points[:, 0] - bx, self.points[:, 1] - by
        along = dx * math.cos(r) + dy * math.sin(r)
        inside = (along >= 0) & (abs(-dx * math.sin(r) + dy * math.cos(r)) <= half_width)
        return float(along[inside].min()) if inside.any() else math.inf

    def fits(self, speed, turn, time):
        '''Does the body, plus MARGIN all round, fit (no scan point inside it) where it will be after driving at this
        speed and turn rate for this long? The robot turns about its pose (rear axle)'''
        x, y, heading = self.start_frame()
        h0 = math.radians(heading)
        h = h0 + turn * time
        if abs(turn) < 1e-6:
            x, y = x + speed * time * math.cos(h0), y + speed * time * math.sin(h0)
        else: # along the arc
            x, y = x + speed / turn * (math.sin(h) - math.sin(h0)), y - speed / turn * (math.cos(h) - math.cos(h0))
        dx, dy = self.points[:, 0] - (x + BODY_X * math.cos(h)), self.points[:, 1] - (y + BODY_X * math.sin(h))
        along, side = dx * math.cos(h) + dy * math.sin(h), -dx * math.sin(h) + dy * math.cos(h)
        return not np.any((abs(along) <= ROBOT_LENGTH / 2 + MARGIN) & (abs(side) <= ROBOT_WIDTH / 2 + MARGIN))

    def clear_heading(self, target):
        '''Of the candidate headings within 90 deg of the current one, the clear one closest to target (deg), or None
        when none within FORWARD of the current heading is clear. Clear: no scan point in the strip ahead of the body,
        as wide as the body plus MARGIN each side. Ties go to the side chosen last tick, so the robot doesn't flip
        between two equal gaps'''
        heading = self.body()[2]
        best, forward = None, False
        for k in range(-int(90 / HEADING_STEP), int(90 / HEADING_STEP) + 1):
            h = heading + k * HEADING_STEP
            if self.free(h, ROBOT_WIDTH / 2 + MARGIN) <= ROBOT_LENGTH / 2 + STRIP:
                continue
            forward = forward or abs(k * HEADING_STEP) <= FORWARD
            score = (round(abs(wrap_deg(h - target)) / HEADING_STEP), abs(wrap_deg(h - self.chosen)))
            if best is None or score < best[0]:
                best = (score, h)
        return best[1] if forward else None

    def search_gap(self):
        """Last resort, when nothing forward is clear: if facing more than two heading steps off the heading we want
        (0 before the turn, -90 at it), turn to it first; then strafe along until the strip straight ahead is clear. That's the gap; normal steering takes over. Strafe first
        to the side with more room, switch once if it's blocked. If both are, back off once (slide MARGIN away from the
        nearest point) and retry; if they're still blocked, stop. Before each step of turning on the
        spot, check the body still fits at the next heading; if not, slide directly away from the nearest point"""
        x, y, heading = self.start_frame() # the robot turns about the pose (rear axle)
        search = self.search
        err = wrap_deg(search["heading"] - heading)
        if "turn" not in search: # turn only if more than two heading steps off; otherwise search at the current
            search["turn"] = abs(err) > 2 * HEADING_STEP # heading (a lane beside a crooked wall runs a few deg off)
        if search["turn"] and abs(err) <= HEADING_STEP:
            search["turn"] = False
        if search["turn"]: # turn to the wanted heading, if there's room
            turn = math.copysign(2 * SPEED / LOOKAHEAD, err)
            if not self.fits(0.0, turn, math.radians(HEADING_STEP) / abs(turn)): # no room: slide away from the nearest point
                self.slide_away()
            else:
                self.move_2D(0.0, 0.0, turn)
            return
        # Gap found? After backing off, the strip is longer by the back-off distance, so whatever blocked it before
        # still counts and the robot strafes past it instead of driving straight back into it
        strip = STRIP + (MARGIN if search.get("backed_off") else 0.0)
        if self.free(heading, ROBOT_WIDTH / 2 + MARGIN) > ROBOT_LENGTH / 2 + strip:
            self.get_logger().info("Gap found")
            self.search = None
            return
        if search.get("backing") is not None: # backing off: slide away from the nearest point by MARGIN, then retry
            if math.hypot(x - search["backing"][0], y - search["backing"][1]) < MARGIN:
                self.slide_away()
                return
            search.update(backing=None, side=None, switched=False)
        room = {side: self.free(heading + 90 * side, ROBOT_LENGTH / 2 + MARGIN) for side in (1, -1)} # left, right
        if search["side"] is None:
            search["side"] = max(room, key=room.get)
        if room[search["side"]] <= ROBOT_WIDTH / 2 + MARGIN:
            if search["switched"] or room[-search["side"]] <= ROBOT_WIDTH / 2 + MARGIN:
                if search.get("backed_off"):
                    self.finish("No clear heading: no gap either side")
                self.get_logger().info("No gap either side: backing off once and retrying")
                search.update(backing=(x, y), backed_off=True)
                self.slide_away()
                return
            search["side"], search["switched"] = -search["side"], True
        self.move_2D(0.0, STRAFE_SPEED * search["side"], 0.0)

    def slide_away(self):
        '''Slide straight away from the nearest scan point, without turning'''
        x, y, heading = self.start_frame()
        d = np.hypot(self.points[:, 0] - x, self.points[:, 1] - y)
        near = int(np.argmin(d))
        ax, ay = (x - self.points[near, 0]) / d[near], (y - self.points[near, 1]) / d[near]
        h = math.radians(heading)
        self.move_2D(STRAFE_SPEED * (math.cos(h) * ax + math.sin(h) * ay),
                     STRAFE_SPEED * (-math.sin(h) * ax + math.cos(h) * ay), 0.0)

    def show_waypoint(self, waypoint):
        '''Simulation only, for watching: draw the waypoint as a green sphere just above the walls in the Gazebo
        window, which the launch file sets to a bird's-eye view (Gazebo's /marker
        service). Runs `gz service` in the background, one call at a time, so the controller never waits for it.
        Each call takes about a second (it starts the gz tool), and `gz service` reports success even on a timeout'''
        if not self.is_simulation or waypoint is None or self.drawing:
            return
        if self.shown is not None and math.hypot(waypoint[0] - self.shown[0], waypoint[1] - self.shown[1]) < 0.02:
            return
        self.shown = waypoint
        x0, y0, h0 = self.start # start frame -> Gazebo's world frame (the odometry's frame)
        c, s = math.cos(math.radians(h0)), math.sin(math.radians(h0))
        wx, wy = x0 + c * waypoint[0] - s * waypoint[1], y0 + s * waypoint[0] + c * waypoint[1]
        green = "{r: 0, g: 1, b: 0, a: 1}"
        req = (f'action: ADD_MODIFY, ns: "waypoint", id: 1, type: SPHERE, scale: {{x: 0.2, y: 0.2, z: 0.2}}, '
               f'pose: {{position: {{x: {wx}, y: {wy}, z: 0.6}}}}, material: {{ambient: {green}, diffuse: {green}}}') # just above the walls
        cmd = ["gz", "service", "-s", "/marker", "--reqtype", "gz.msgs.Marker", "--reptype", "gz.msgs.Empty",
               "--timeout", "3000", "--req", req]
        def run():
            out = subprocess.run(cmd, capture_output=True, text=True)
            if out.returncode != 0 or "timed out" in out.stdout + out.stderr:
                self.get_logger().warn("Couldn't draw the waypoint in Gazebo (gz service /marker)")
                self.shown = None # try again next tick
            self.drawing = False
        self.drawing = True
        threading.Thread(target=run, daemon=True).start()

    def finish(self, reason):
        self.move_2D(0.0, 0.0, 0.0)
        self.get_logger().info(f"Finished: {reason}")
        raise SystemExit

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
        if self.start is None:
            self.start = np.array(self.pose, dtype=float)
        if self.points is None:
            return # waiting for the first scan
        bx, by, heading = self.body()
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_move is None or math.hypot(bx - self.last_move[1], by - self.last_move[2]) >= STUCK_DIST:
            self.last_move = (now, bx, by)
        elif now - self.last_move[0] > STUCK_TIME:
            self.finish("Stuck")

        if self.section == 1:
            # Where to go: straight down the corridor, then to the right turn, then down the new corridor
            waypoint = self.waypoint()
            self.show_waypoint(waypoint)
            if waypoint is not None and not self.aligning and math.hypot(waypoint[0] - bx, waypoint[1] - by) < LOOKAHEAD:
                self.aligning = True
                self.get_logger().info(f"At the right turn ({waypoint[0]:.2f}, {waypoint[1]:.2f})")
            if self.corner_x is not None and by < self.wall_y and abs(wrap_deg(heading + 90)) < HEADING_TOL:
                self.get_logger().info("Section 1 done: through the right turn, heading down the new corridor")
                self.section, self.search = 2, None
            if self.aligning: # keep turning toward -90 until through the turn
                target = -90.0
            elif waypoint is not None:
                target = math.degrees(math.atan2(waypoint[1] - by, waypoint[0] - bx))
            else:
                target = 0.0
        if self.section == 2:
            # Down the corridor until the door is found, then along the door's centre line and through it
            target = -90.0
            if self.door is not None:
                cx, cy, nx, ny = self.door
                self.show_waypoint((cx, cy))
                through = math.degrees(math.atan2(ny, nx)) # direction through the door
                along = (bx - cx) * nx + (by - cy) * ny # how far past the door's line (negative: before it)
                # done once the whole body (plus MARGIN) is past the door's line
                if along > ROBOT_LENGTH / 2 + MARGIN and abs(wrap_deg(heading - through)) < HEADING_TOL:
                    self.finish("Section 2 done: through the door")
                if not self.door_locked and along > -LOOKAHEAD:
                    self.door_locked = True
                    self.get_logger().info(f"Door locked at ({cx:.2f}, {cy:.2f}), heading through it {through:.0f} deg")
                # pure pursuit on the centre line: aim LOOKAHEAD past the robot's projection onto it
                px, py = cx + (along + LOOKAHEAD) * nx, cy + (along + LOOKAHEAD) * ny
                target = math.degrees(math.atan2(py - by, px - bx))

        chosen = None if self.search else self.clear_heading(target)
        if chosen is None: # nothing clear within FORWARD of the heading: search for a gap
            if self.search is None:
                if self.section == 2:
                    # face the door only once near its line (locked); further up the corridor, face down it
                    wanted = math.degrees(math.atan2(self.door[3], self.door[2])) if self.door_locked else -90.0
                else:
                    wanted = -90.0 if self.corner_x is not None and bx >= self.corner_x else 0.0
                self.search = {"heading": wanted, "side": None, "switched": False}
                self.get_logger().info(f"Nothing clear ahead: searching for a gap facing {wanted:.0f} deg")
            self.search_gap()
            return
        self.chosen = chosen
        alpha = math.radians(wrap_deg(chosen - heading))
        if not self.fits(SPEED * math.cos(alpha), 2 * SPEED * math.sin(alpha) / LOOKAHEAD, AHEAD_TIME):
            if not self.fits(SPEED, 0.0, AHEAD_TIME): # turning would swing the body into something, and so would straight
                self.search = {"heading": chosen, "side": None, "switched": False}
                self.search_gap()
                return
            alpha = 0.0 # go straight until there's room to turn
        self.move_2D(SPEED * math.cos(alpha), 0.0, 2 * SPEED * math.sin(alpha) / LOOKAHEAD) # slower on sharp turns
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
    except (SystemExit, KeyboardInterrupt):
        print("Shutting down")
    try:
        obstacle.move_2D(0.0, 0.0, 0.0) # Gazebo keeps the last velocity
    except Exception:
        pass
    obstacle.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()