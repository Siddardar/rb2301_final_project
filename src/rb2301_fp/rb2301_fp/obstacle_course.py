import argparse
import configparser
import heapq
import math
import os
import time
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
# Sections 1 and 2 (plan.md, a-star.md): scan-based detection of the right turn and the door gives the waypoints; a map
# that grows as the LiDAR sees more, and A* on it, get the robot to them. Everything is in the start frame: origin at the
# start, +x the start heading, angles counter-clockwise. No constant describes the course.

# The robot
ROBOT_LENGTH, ROBOT_WIDTH = 0.289, 0.225 # m, CA1
BODY_X = 0.07 # m, body centre ahead of the pose (simulation: the pose is the rear axle)
LIDAR_X, LIDAR_YAW = 0.114, math.pi # m, rad: the LiDAR sits ahead of the pose, mounted backwards (URDF laser_joint)

# What we choose
SPEED = 0.2 # m/s, forward speed
LOOKAHEAD = 0.4 # m, pure pursuit: turn rate = 2 * SPEED * sin(angle off) / LOOKAHEAD, forward speed SPEED * cos
MARGIN = 0.05 # m kept clear on each side of the body
CELL = 0.025 # m, map resolution
GROW = 1.0 # m of unknown border added on a side when the map grows
AHEAD = 1.0 # m: before a waypoint is known, the goal is any cell this much further along the wanted direction
WINDOW = 5.0 # deg either side of straight ahead / straight right (start frame) for the beams that measure the walls
OCC_WEIGHT = 4 # a cell is occupied while hits * OCC_WEIGHT > passes: a wall seen at a shallow angle (beams skimming
               # along it) stays occupied, a stray hit in open space still clears
HEADING_TOL = 45.0 # deg: through the turn / door and this close to its direction counts as done
STUCK_TIME, STUCK_DIST = 3.0, 0.05 # s, m: moving less than this in this long means stuck
INFLATE = ROBOT_WIDTH / 2 + MARGIN # m: occupied cells are grown by this; the body centre stays outside
CLEARANCE_COST = 3.0 # a step within 2 * MARGIN beyond the inflated cells costs this many times a normal one
PATH_LOOKAHEAD = 0.25 # m, pure pursuit along the A* path (shorter than LOOKAHEAD, so it cuts corners less)
REPLAN_PERIOD = 0.5 # s between A* plans. Mapping still uses every scan; planning on every scan was too slow on the robot


class Grid:
    '''Map in the start frame that grows when a scan reaches outside it. Each cell counts beams that ended in it (hits)
    and went through it (passes); occupied if hits * OCC_WEIGHT > passes. Indexed [row (y), col (x)]'''
    def __init__(self, x, y):
        self.x0, self.y0 = x - GROW, y - GROW # start-frame position of cell [0, 0]'s corner
        n = int(round(2 * GROW / CELL))
        self.hits = np.zeros((n, n), np.int32)
        self.passes = np.zeros((n, n), np.int32)

    def cell(self, x, y):
        return (np.floor((np.asarray(y) - self.y0) / CELL).astype(int), np.floor((np.asarray(x) - self.x0) / CELL).astype(int))

    def centre(self, row, col):
        return self.x0 + (col + 0.5) * CELL, self.y0 + (row + 0.5) * CELL

    def fit(self, xs, ys):
        '''Pad the map so the points (xs, ys) are inside it, plus GROW of unknown border on any side that grew'''
        rows, cols = self.hits.shape
        border = int(round(GROW / CELL))
        pad = [max(0, math.ceil((self.y0 - ys.min()) / CELL)), max(0, math.ceil((ys.max() - self.y0) / CELL) + 1 - rows),
               max(0, math.ceil((self.x0 - xs.min()) / CELL)), max(0, math.ceil((xs.max() - self.x0) / CELL) + 1 - cols)]
        if any(pad):
            below, above, left, right = [p + border if p else 0 for p in pad]
            self.hits = np.pad(self.hits, ((below, above), (left, right)))
            self.passes = np.pad(self.passes, ((below, above), (left, right)))
            self.x0, self.y0 = self.x0 - left * CELL, self.y0 - below * CELL
            return True
        return False

    def flat(self, x, y):
        row, col = self.cell(x, y)
        return np.unique(row * self.hits.shape[1] + col)

    def add_scan(self, ox, oy, ex, ey, ranges):
        '''Beams from (ox, oy) ending at (ex, ey). Passes stop one cell short of the hit. True if the map grew'''
        grew = self.fit(np.append(ex, ox), np.append(ey, oy))
        c, s = (ex - ox) / ranges, (ey - oy) / ranges
        steps = np.arange(0.0, ranges.max(), CELL / 2)
        along = steps[None, :] <= (ranges - CELL)[:, None]
        hits = self.flat(ex, ey)
        passed = np.setdiff1d(self.flat((ox + c[:, None] * steps)[along], (oy + s[:, None] * steps)[along]), hits)
        self.passes.reshape(-1)[passed] += 1
        self.hits.reshape(-1)[hits] += 1
        return grew

    def grown(self, radius):
        '''Occupied cells grown by radius (m)'''
        occupied = self.hits * OCC_WEIGHT > self.passes
        rows, cols = np.nonzero(occupied)
        out = np.zeros_like(occupied)
        k = int(math.ceil(radius / CELL))
        for dr in range(-k, k + 1):
            for dc in range(-k, k + 1):
                if math.hypot(dr, dc) * CELL <= radius:
                    out[np.clip(rows + dr, 0, out.shape[0] - 1), np.clip(cols + dc, 0, out.shape[1] - 1)] = True
        return out

    def blocked(self, bx, by):
        '''Cells the body centre can't be in (occupied cells grown by INFLATE, except around the robot itself), and the
        cells near them (within 2 * MARGIN further), which cost more to cross'''
        blocked = self.grown(INFLATE)
        k = int(math.ceil(INFLATE / CELL))
        r, c = self.cell(bx, by)
        blocked[max(r - k, 0):r + k + 1, max(c - k, 0):c + k + 1] = False # the robot is standing there
        return blocked, self.grown(INFLATE + 2 * MARGIN) & ~blocked


def astar(blocked, near, start, goal=None, radius=0.0, direction=None, distance=0.0):
    '''8-connected A* from start (row, col). The goal is either the cells within radius (cells) of goal (row, col), or
    any cell at least distance (cells) further than start along direction (unit (dx, dy)). Steps into near cells cost
    CLEARANCE_COST times more. Returns (cells or None, cells expanded, the expanded cell closest to the goal, how far
    that cell is from the goal in cells), the last three for debugging'''
    sr, sc = start
    if goal is not None:
        h = lambda r, c: max(0.0, math.hypot(r - goal[0], c - goal[1]) - radius)
    else:
        dx, dy = direction
        h = lambda r, c: max(0.0, distance - ((c - sc) * dx + (r - sr) * dy))
    rows, cols = blocked.shape
    cost, parent = {start: 0.0}, {}
    heap = [(h(sr, sc), 0.0, start)]
    expanded, best, best_h = 0, start, h(sr, sc)
    while heap:
        _, g, (r, c) = heapq.heappop(heap)
        if g > cost[(r, c)]:
            continue
        expanded += 1
        if h(r, c) < best_h:
            best, best_h = (r, c), h(r, c)
        if h(r, c) == 0.0:
            path = [(r, c)]
            while path[-1] in parent:
                path.append(parent[path[-1]])
            return path[::-1], expanded, best, 0.0
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                n = (r + dr, c + dc)
                if (dr or dc) and 0 <= n[0] < rows and 0 <= n[1] < cols and not blocked[n]:
                    ng = g + math.hypot(dr, dc) * (CLEARANCE_COST if near[n] else 1.0)
                    if ng < cost.get(n, math.inf):
                        cost[n], parent[n] = ng, (r, c)
                        heapq.heappush(heap, (ng + h(*n), ng, n))
    return None, expanded, best, best_h


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
    def __init__(self, is_simulation:bool=True, calibrate:bool=False, snapshots:str=None, exit_on_finish:bool=False):
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

        # Queue of 1: while the node is busy, only the newest scan waits. An older scan would be placed with the robot's
        # current pose, drawn rotated by however much the robot turned since (smeared walls)
        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 1) # Subscribe to LiDAR scan data

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

        self.start = None # pose on the first tick, in the pose's own frame
        self.points = None # latest scan's points, start frame
        self.grid = None # the map, made on the first scan
        self.path = None # (x, y) points to follow, from A*
        self.section = 1
        # Section 1: the right turn
        self.right_hits = [] # y of every hit by a beam straight right, in section 1
        self.end_x = -math.inf # furthest x reached straight ahead in section 1: the first corridor's end wall, which
                               # carries on as the new corridor's left wall (obstacles are always nearer)
        self.wall_y = None # the right wall's line: the median of right_hits
        # Section 2: the door
        self.door = None # (centre x, centre y, direction through it as a unit vector x, y), start frame
        self.door_locked = False # at the line-up point: stop updating it, drive straight through
        self.rejected = [] # centres of doors with no path to them: ignore notches near these
        self.last_move = None # (time, x, y) when the robot last moved STUCK_DIST
        # Map snapshots for tools/view_map.py (--snapshots DIR)
        self.snapshots = snapshots
        if snapshots:
            os.makedirs(snapshots, exist_ok=True)
        self.trail = [] # body centre (x, y, heading) every tick, start frame
        self.scan_count = 0
        self.last_snapshot = -math.inf # time.monotonic() of the last snapshot
        self.last_plan = -math.inf # time.monotonic() of the last A* plan
        # Stopping: by default the robot stops and holds (the node keeps running, logging and mapping); with --exit
        # (the simulation tests), the node exits instead, as it used to
        self.exit_on_finish = exit_on_finish
        self.halted = None # reason the robot is holding still for good (e.g. section 2 done)
        self.log_times = {} # key -> last time a rate-limited log line was printed

    def sub_scan_callback(self, msg):
        """Scan subscriber"""
        ranges = np.array(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        if len(ranges) > 360:
            ranges, angles = ranges[::2], angles[::2]
        self.last_scan = ranges
        if self.pose is None:
            return
        if self.start is None:
            self.start = np.array(self.pose, dtype=float)
        self.scan_count += 1
        valid = np.isfinite(ranges) & (ranges > msg.range_min)
        x, y, heading = self.start_frame()
        h = math.radians(heading)
        lx, ly = x + LIDAR_X * math.cos(h), y + LIDAR_X * math.sin(h)
        a = angles[valid] + h + LIDAR_YAW
        ex, ey = lx + ranges[valid] * np.cos(a), ly + ranges[valid] * np.sin(a)
        self.points = np.stack((ex, ey), axis=1)
        if self.grid is None:
            self.grid = Grid(lx, ly)
            self.log(None, 0, f"Start pose {np.round(self.start, 3).tolist()} (pose frame); map made at the LiDAR ({lx:.2f}, {ly:.2f})")
        if self.grid.add_scan(lx, ly, ex, ey, ranges[valid]):
            rows, cols = self.grid.hits.shape
            self.log(None, 0, f"Map grew to {rows} x {cols} cells, x {self.grid.x0:.2f}..{self.grid.x0 + cols * CELL:.2f}, "
                              f"y {self.grid.y0:.2f}..{self.grid.y0 + rows * CELL:.2f}")
        self.log("scan", 1.0, f"Scan {self.scan_count}: {int(valid.sum())}/{len(ranges)} valid beams, range_min "
                              f"{msg.range_min:.2f}; robot (start frame) ({x:.3f}, {y:.3f}, {heading:.1f} deg); "
                              f"section {self.section}")

        if self.section == 2:
            self.find_door(ex, ey, a)
        elif abs(heading) < HEADING_TOL: # section 1, still driving along the first corridor: the right wall's line,
            # from the beams straight right. Most of them hit the wall; hits on obstacles beside the robot, or on things
            # seen through the opening, are too few to move the median. Not once turning into the opening: then
            # "straight right" points down the new corridor
            direction = np.degrees(a) % 360
            right = abs(direction - 270) <= WINDOW
            self.right_hits.extend(ey[right].tolist())
            ahead = (direction <= WINDOW) | (direction >= 360 - WINDOW)
            if ahead.any():
                self.end_x = max(self.end_x, float(ex[ahead].max()))
            if self.right_hits:
                old = self.wall_y
                self.wall_y = float(np.median(self.right_hits))
                if old is None or abs(self.wall_y - old) > 0.02:
                    self.log(None, 0, f"Right wall's line y = {self.wall_y:.3f} (median of {len(self.right_hits)} hits "
                                      f"straight right); end wall x = {self.end_x:.3f}")
        if not self.calibrate and time.monotonic() - self.last_plan >= REPLAN_PERIOD: # calibration mode: map only
            self.last_plan = time.monotonic()
            self.replan()

    def replan(self):
        '''A* on the map from the body centre to the current goal (a-star.md). No path: stop'''
        if self.section == 2 and self.door_locked:
            return # straight through the door along its centre line, no A*
        if self.wall_y is None:
            return
        t0 = time.monotonic()
        bx, by, _ = self.body()
        start = tuple(int(v) for v in self.grid.cell(bx, by))
        blocked, near = self.grid.blocked(bx, by)
        cells = None
        if self.section == 2 and self.door is not None: # near the line-up point in front of the door
            cx, cy, nx, ny = self.door
            goal = tuple(int(v) for v in self.grid.cell(cx - LOOKAHEAD * nx, cy - LOOKAHEAD * ny))
            goal_text = f"line-up point ({cx - LOOKAHEAD * nx:.2f}, {cy - LOOKAHEAD * ny:.2f}) of the door at ({cx:.2f}, {cy:.2f})"
            cells, expanded, best, short = astar(blocked, near, start, goal=goal, radius=MARGIN / CELL)
            if cells is None: # no way to this door: forget it and go back to the goal line
                bxy = self.grid.centre(*best)
                self.log(None, 0, f"No path to the {goal_text}: searched {expanded} cells, got to ({bxy[0]:.2f}, "
                                  f"{bxy[1]:.2f}), {short * CELL:.2f} m short. Ignoring this door")
                self.rejected.append((cx, cy))
                self.door = None
        if cells is None:
            # Goal line: section 1, any cell AHEAD below the right wall's line (only reachable through the right turn,
            # which A* finds by itself); section 2, any cell AHEAD further down the corridor than the robot. The map
            # only covers what's been seen, so grow it (as unknown space) to include the goal line first
            distance = by - (self.wall_y - AHEAD) if self.section == 1 else AHEAD
            rows_before, x0, y0 = self.grid.hits.shape, self.grid.x0, self.grid.y0
            self.grid.fit(np.array([bx]), np.array([by - distance - CELL]))
            if self.grid.hits.shape != rows_before or (self.grid.x0, self.grid.y0) != (x0, y0):
                start = tuple(int(v) for v in self.grid.cell(bx, by))
                blocked, near = self.grid.blocked(bx, by)
            goal_text = f"goal line y <= {by - distance:.2f}" + (" (AHEAD below the right wall's line)" if self.section == 1
                                                                   else " (AHEAD further down the corridor)")
            cells, expanded, best, short = astar(blocked, near, start, direction=(0.0, -1.0), distance=distance / CELL)
        ms = (time.monotonic() - t0) * 1000
        if cells is None:
            occupied = self.grid.hits * OCC_WEIGHT > self.grid.passes
            bxy = self.grid.centre(*best)
            r, c = start
            self.log("nopath", 0.5, f"NO PATH to the {goal_text}. Searched {expanded} cells in {ms:.0f} ms; closest it "
                     f"got: ({bxy[0]:.2f}, {bxy[1]:.2f}), {short * CELL:.2f} m short. Robot ({bx:.2f}, {by:.2f}) cell "
                     f"{start}: hits {self.grid.hits[r, c]}, passes {self.grid.passes[r, c]}. Map {occupied.shape[0]} x "
                     f"{occupied.shape[1]}: {int(occupied.sum())} occupied, {int(blocked.sum())} blocked cells")
            if self.exit_on_finish:
                self.finish("No path")
            self.path = None # stop and keep replanning on every scan
            return
        self.path = np.array([self.grid.centre(r, c) for r, c in cells])
        length = float(np.hypot(*np.diff(self.path, axis=0).T).sum()) if len(self.path) > 1 else 0.0
        self.log("plan", 1.0, f"Plan to the {goal_text}: {len(cells)} cells, {length:.2f} m, ends ({self.path[-1][0]:.2f}, "
                              f"{self.path[-1][1]:.2f}); searched {expanded} cells; {ms:.0f} ms including the map")

    def find_door(self, ex, ey, a):
        '''Section 2: the door is a notch in the left wall, a run of beams reaching more than ROBOT_LENGTH further left
        than the beams either side of it (the posts). A door's posts are ends of a wall: in the map, the wall carries on
        past each post, away from the door (an obstacle's edge doesn't). The posts are also on the same wall and far
        enough apart for the robot plus margins. A set-back wall (the simulation) is only a step up, never a notch, so
        the same test works whether or not the door is flush, and on a tilted course'''
        if self.door_locked:
            return
        direction = (np.degrees(a) + 180) % 360 - 180 # start frame, 0 = straight left of the corridor
        side = np.flatnonzero(abs(direction) < 90) # every beam pointing left, in angle order
        side = side[np.argsort(direction[side])]
        x, y = ex[side], ey[side]
        groups = np.split(np.arange(len(x)), np.flatnonzero(abs(np.diff(x)) > ROBOT_LENGTH) + 1)
        bx, by, _ = self.body()
        doors, rejected = [], {} # rejected: reason -> count, for the log
        for p in range(len(groups) - 2):
            post1 = groups[p][-1]
            for q in range(p + 1, len(groups)):
                if x[groups[q]].min() > x[post1] + ROBOT_LENGTH: # still deeper than the first post: inside the notch
                    continue
                post2 = groups[q][0]
                inner = np.concatenate(groups[p + 1:q]) if q > p + 1 else np.array([], int)
                # Both posts on the same wall (the corridor's end can hide a post from upstream, and then pair with
                # the far one), the notch deeper than both, the posts far enough apart for the robot plus margins (a
                # gap between wall panels isn't a door), and the wall carrying on past each post (not an obstacle)
                # ... and both posts near the new corridor's left wall (within a robot length, which allows for a
                # course tilted a few degrees): not a gap between obstacles out in the corridor
                why = None
                if not len(inner):
                    why = "" # neighbouring groups, no notch between them: not worth logging
                elif abs(x[post2] - x[post1]) > ROBOT_LENGTH:
                    why = "posts not on the same wall"
                elif min(x[post1], x[post2]) < self.end_x - ROBOT_LENGTH:
                    why = "posts not near the left wall"
                elif x[inner].min() <= x[post2] + ROBOT_LENGTH:
                    why = "not deep enough"
                elif math.hypot(x[post2] - x[post1], y[post2] - y[post1]) < ROBOT_WIDTH + 2 * MARGIN:
                    why = "too narrow"
                elif not (self.wall_continues(x[post1], y[post1], x[post2], y[post2])
                          and self.wall_continues(x[post2], y[post2], x[post1], y[post1])):
                    why = "no wall past a post"
                if why:
                    rejected[why] = rejected.get(why, 0) + 1
                if why is None:
                    cx, cy = (x[post1] + x[post2]) / 2, (y[post1] + y[post2]) / 2
                    nx, ny = y[post2] - y[post1], -(x[post2] - x[post1]) # perpendicular to the posts' line
                    norm = math.hypot(nx, ny)
                    nx, ny = (nx, ny) if (cx - bx) * nx + (cy - by) * ny > 0 else (-nx, -ny) # away from the robot
                    doors.append((cx, cy, nx / norm, ny / norm))
                break
        # The door is in the left wall, so it points roughly left (0 deg); skip doors already found to have no path
        notches = len(doors)
        pointing = [d for d in doors if abs(math.degrees(math.atan2(d[3], d[2]))) < HEADING_TOL]
        doors = [d for d in pointing if all(math.hypot(d[0] - rx, d[1] - ry) > LOOKAHEAD for rx, ry in self.rejected)]
        if notches - len(pointing):
            rejected["not pointing left"] = notches - len(pointing)
        if len(pointing) - len(doors):
            rejected["near a door with no path"] = len(pointing) - len(doors)
        self.log("door", 1.0, f"Door search: {len(doors)} candidate(s) "
                              f"{[(round(d[0], 2), round(d[1], 2), round(math.degrees(math.atan2(d[3], d[2])))) for d in doors]}"
                              f"; rejected {rejected or 'none'}; current door "
                              f"{None if self.door is None else (round(self.door[0], 2), round(self.door[1], 2))}")
        if self.door is not None:
            doors.append(self.door) # keep the current one unless a door closer to the goal line turns up
        if doors:
            best = min(doors, key=lambda d: d[1]) # closest to the goal line: furthest down the corridor
            if self.door is None or math.hypot(best[0] - self.door[0], best[1] - self.door[1]) > LOOKAHEAD:
                self.get_logger().info(f"Door at ({best[0]:.2f}, {best[1]:.2f})")
            self.door = best

    def wall_continues(self, px, py, ox, oy):
        '''Does the map show wall carrying on from the post (px, py), away from the other post (ox, oy), along their
        line? Samples every cell out to 2 * MARGIN, allowing one cell either side; most must be occupied'''
        d = math.hypot(px - ox, py - oy)
        ux, uy = (px - ox) / d, (py - oy) / d # outward along the posts' line
        occupied = self.grid.hits * OCC_WEIGHT > self.grid.passes
        rows, cols = occupied.shape
        steps = np.arange(1, int(round(2 * MARGIN / CELL)) + 1)
        found = 0
        for k in steps:
            hit = False
            for j in (-1, 0, 1): # one cell either side of the line
                r, c = self.grid.cell(px + k * CELL * ux - j * CELL * uy, py + k * CELL * uy + j * CELL * ux)
                hit = hit or (0 <= r < rows and 0 <= c < cols and occupied[r, c])
            found += hit
        return found > len(steps) / 2

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

    def save_snapshot(self):
        '''Save the map and the robot's state for tools/view_map.py: map_<scan>.npz in the --snapshots directory,
        written to a .tmp file first so the viewer never copies half a file'''
        if not self.snapshots or self.grid is None:
            return
        g = self.grid
        occupied = g.hits * OCC_WEIGHT > g.passes
        occupancy = np.where(occupied, 1, np.where(g.passes > 0, -1, 0)).astype(np.int8) # 1 occupied, -1 free, 0 unknown
        name = os.path.join(self.snapshots, f"map_{self.scan_count:06d}.npz")
        state = "calibrate" if self.calibrate else f"section {self.section}" + (", door locked" if self.door_locked else "")
        with open(name + ".tmp", "wb") as f:
            np.savez_compressed(f, occupancy=occupancy, origin=np.array((g.x0, g.y0)), cell_size=CELL,
                                pose=np.array(self.body() if self.start is not None else (np.nan,) * 3),
                                start=np.zeros(3), trail=np.array(self.trail, dtype=float).reshape(-1, 3),
                                path=np.zeros((0, 2)) if self.path is None else self.path,
                                target=self.path[-1] if self.path is not None else np.full(2, np.nan),
                                wall_y=np.nan if self.wall_y is None else self.wall_y,
                                door=np.full(4, np.nan) if self.door is None else np.array(self.door),
                                state=state, scan_count=self.scan_count)
        os.replace(name + ".tmp", name)

    def finish(self, reason):
        '''Stop. With --exit, also exit the node (the simulation tests); otherwise hold still for good, still mapping
        and logging, until Ctrl+C'''
        self.move_2D(0.0, 0.0, 0.0)
        self.get_logger().info(f"Finished: {reason}")
        if self.exit_on_finish:
            raise SystemExit
        self.halted = reason

    def log(self, key, period, message):
        '''Info log, at most once every period seconds per key (key None: always)'''
        now = time.monotonic()
        if key is not None:
            if now - self.log_times.get(key, -math.inf) < period:
                return
            self.log_times[key] = now
        self.get_logger().info(message)

    def steer(self, cx, cy, lookahead):
        '''Pure pursuit toward (cx, cy), about lookahead away: turn on the arc through it, slower the further off it
        is; turn in place if it's behind. The robot turns about its body centre, not its pose (rear axle): a small
        sideways velocity of -turn * BODY_X keeps the body centre where A* planned it, so turning on the spot doesn't
        swing the front of the body sideways'''
        bx, by, heading = self.body()
        alpha = math.radians(wrap_deg(math.degrees(math.atan2(cy - by, cx - bx)) - heading))
        turn = 2 * SPEED * (math.sin(alpha) if abs(alpha) <= math.pi / 2 else math.copysign(1.0, alpha)) / lookahead
        speed = SPEED * max(math.cos(alpha), 0.0)
        self.move_2D(speed, -turn * BODY_X, turn)
        return speed, turn, math.degrees(alpha)

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

        if self.snapshots and self.start is not None: # map snapshots for tools/view_map.py, once a second
            self.trail.append(tuple(self.body()))
            if time.monotonic() - self.last_snapshot >= 1.0: # by the clock: control ticks run late when the robot is busy
                self.last_snapshot = time.monotonic()
                self.save_snapshot()

        if self.calibrate and self.raw_pose is not None:
            self._ticks += 1
            if self._ticks % 10 == 0: # twice a second
                ox, oy, oh = self.raw_pose
                self.get_logger().info(f"optitrack ({ox:+.3f}, {oy:+.3f}, {oh:+7.1f} deg)  ->  maze ({self.pose[0]:+.3f}, {self.pose[1]:+.3f}, {self.pose[2]:+7.1f} deg)")
            return # Calibration mode: look, don't drive

        ###### INSERT CODE HERE ######
        if self.start is None:
            self.start = np.array(self.pose, dtype=float)
        if self.halted is not None:
            self.move_2D(0.0, 0.0, 0.0)
            self.log("halted", 5.0, f"Holding still: {self.halted}. Ctrl+C to stop")
            return
        bx, by, heading = self.body()
        if self.path is None: # no plan yet, or no path right now: stand still and wait for the next plan
            self.move_2D(0.0, 0.0, 0.0)
            self.log("waiting", 1.0, f"No path: standing still at ({bx:.2f}, {by:.2f}, {heading:.1f} deg), replanning "
                                     f"on every scan")
            self.last_move = None
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_move is None or math.hypot(bx - self.last_move[1], by - self.last_move[2]) >= STUCK_DIST:
            self.last_move = (now, bx, by)
        elif now - self.last_move[0] > STUCK_TIME:
            if self.exit_on_finish:
                self.finish("Stuck")
            self.log("stuck", 2.0, f"WARNING: moved less than {STUCK_DIST} m in {now - self.last_move[0]:.1f} s, at "
                                   f"({bx:.2f}, {by:.2f}, {heading:.1f} deg)")

        if self.section == 1:
            if by < self.wall_y and abs(wrap_deg(heading + 90)) < HEADING_TOL:
                self.get_logger().info("Section 1 done: through the right turn, heading down the new corridor")
                self.section = 2
        elif self.door is not None:
            cx, cy, nx, ny = self.door
            lx, ly = cx - LOOKAHEAD * nx, cy - LOOKAHEAD * ny # line-up point
            through = math.degrees(math.atan2(ny, nx)) # direction through the door
            along = (bx - cx) * nx + (by - cy) * ny # how far past the door's line (negative: before it)
            # done once the whole body (plus MARGIN) is past the door's line
            if along > ROBOT_LENGTH / 2 + MARGIN and abs(wrap_deg(heading - through)) < HEADING_TOL:
                self.finish("Section 2 done: through the door")
                return
            if not self.door_locked and math.hypot(bx - lx, by - ly) < 2 * MARGIN: # at the line-up point
                self.door_locked = True
                self.get_logger().info(f"Door locked at ({cx:.2f}, {cy:.2f}), heading through it {through:.0f} deg")
            if self.door_locked: # pure pursuit along the door's centre line, LOOKAHEAD past the robot's projection
                px, py = cx + (along + LOOKAHEAD) * nx, cy + (along + LOOKAHEAD) * ny
                speed, turn, alpha = self.steer(px, py, LOOKAHEAD)
                self.log("drive", 0.5, f"Through the door: robot ({bx:.2f}, {by:.2f}, {heading:.1f} deg), {along:+.2f} m "
                                       f"past its line; aim ({px:.2f}, {py:.2f}), {alpha:+.0f} deg off; speed "
                                       f"{speed:.2f}, turn {turn:+.2f}")
                return

        # Pure pursuit along the planned path: the carrot is the first point PATH_LOOKAHEAD past the closest one
        d = np.hypot(self.path[:, 0] - bx, self.path[:, 1] - by)
        ahead = np.flatnonzero((np.arange(len(d)) > np.argmin(d)) & (d >= PATH_LOOKAHEAD))
        carrot = self.path[ahead[0] if len(ahead) else -1]
        speed, turn, alpha = self.steer(carrot[0], carrot[1], PATH_LOOKAHEAD)
        self.log("drive", 0.5, f"Drive (section {self.section}): robot ({bx:.2f}, {by:.2f}, {heading:.1f} deg); carrot "
                               f"({carrot[0]:.2f}, {carrot[1]:.2f}), {alpha:+.0f} deg off; speed {speed:.2f}, turn "
                               f"{turn:+.2f}; {len(self.path)} path points left of the plan")
        ###### INSERT CODE HERE ######
                

def main(args=None):
    arg_parser = argparse.ArgumentParser(description="RB2301 final project")
    arg_parser.add_argument('--calibrate', action='store_true', help="Real robot: only print the Optitrack pose and the maze-frame pose twice a second, never drive")
    arg_parser.add_argument('--snapshots', metavar='DIR', help="Save a map snapshot to DIR once a second, for tools/view_map.py")
    arg_parser.add_argument('--exit', action='store_true', help="Exit when finished, stuck or with no path (the simulation tests). By default the robot stops and holds, and the node keeps running")
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
    obstacle = ObstacleCourseNode(is_simulation, cli_args.calibrate, cli_args.snapshots, cli_args.exit)
    try:
        rclpy.spin(obstacle)
    except (SystemExit, KeyboardInterrupt):
        print("Shutting down")
    try:
        obstacle.move_2D(0.0, 0.0, 0.0) # Gazebo keeps the last velocity
    except Exception:
        pass
    obstacle.save_snapshot() # the final state, for tools/view_map.py
    obstacle.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()