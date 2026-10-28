# Final project plan

This plan describes how the robot gets through the obstacle course on the real
robot, as implemented in `src/rb2301_fp/rb2301_fp/obstacle_course.py`. The code
doesn't know the layout of the course, the order of its sections, or where the
finish is. The robot maps what it sees, drives toward space it hasn't seen yet,
times moving obstacles before crossing their path, and stops at the far end of
the course once there's nothing left to explore.

## Design rules

- **No course parameters.** The code has no constants that describe the course,
  such as door positions, the number of moving obstacles, or a finish line. It
  only has constants that describe the robot and its sensors. These are listed
  in [Constants](#constants).
- **No special case for each section.** The zig-zag, the static obstacles, and
  the door all use the same map and planner. Moving obstacles add one rule. The
  sections can come in any order.
- **Keep it simple.** The code has no fallbacks or timeouts. If the robot has
  nowhere left to go, it stops. If no crossing window is long enough, it waits.
  If the course has no wall at the end, the robot keeps driving.

## Overview

The robot runs three loops:

| Loop | Rate | What it does |
|---|---|---|
| Mapping | Every LiDAR scan | Updates the map and finds moving obstacles. |
| Planning | Once a second, inside the control loop | Chooses where to go next and plans a path there. |
| Control | Every timer tick, 20 Hz | Runs the state machine and sends `move_2D()` commands. |

Mapping and planning run at the same time: the map never stops updating, and
each plan uses the latest map. The robot doesn't map first and plan afterward.

ROS runs one callback at a time, so the loops take turns instead of running in
parallel. That's fine while each callback is short. In offline tests on a
laptop, a map update took about 5 ms and a plan about 6 ms, with a worst case of
56 ms. Expect the robot's computer to be several times slower.

The sections that follow explain each part.

## 1. Pose: mapping instead of SLAM

SLAM (simultaneous localization and mapping) builds a map and works out the
robot's pose at the same time. You need it when nothing else tells the robot
where it is, so the robot has to use the map to locate itself.

On the real robot, Optitrack gives the pose with millimeter accuracy. Any pose a
LiDAR-based SLAM method estimates would be less accurate. The robot only needs
the mapping half: it takes the Optitrack pose as correct and places each LiDAR
point on the map from that pose.

The map uses the same frame as `self.pose`, which `optitrack_callback()` has
already converted to the CA2 maze frame.

## 2. The map

### 2.1 Cells

`CourseMap` holds the map as 2D numpy arrays. Each element is a cell: a square
of floor `CELL_SIZE` (2.5 cm) wide. Converting between world coordinates and
cells uses that size:

```python
col, row = np.floor((xy - origin) / CELL_SIZE)
```

Row 0, column 0 is the cell at `origin`. Rows increase with y, and columns
increase with x.

**Why 2.5 cm.** This value balances three things:

- **Narrow gaps stay open on the grid.** Each wall edge can be up to one cell
  off, so a gap can shrink by up to two cells. At 2.5 cm, a gap loses at most
  5 cm. At 10 cm, a door 0.4 m wide loses half its width.
- **It's close to the sensor's accuracy.** LiDAR range error is around 1–2 cm.
  Smaller cells make the map bigger, not more accurate.
- **Planning stays fast.** A 10 m × 10 m grid has 160,000 cells at 2.5 cm and
  1,000,000 at 1 cm. The search code visits cells one at a time, so the count
  matters.

### 2.2 A map that grows

The map doesn't need to know the size of the course. It starts as a single cell
at the robot's first pose. Before each scan is added, `grow_map_to_fit()` checks
whether every beam fits inside the grid. If one doesn't, it pads the grid on
that side with `np.pad()`, adding the missing cells plus `GROW_MARGIN` (1 m) of
unknown space, and moves `origin` to match.

Padding copies the whole array, but the margin means it only happens a few
times per run. The grid always keeps a border of `DYNAMIC_MARGIN` cells, so
every cell the dynamic test checks around a hit is inside the grid
(section 2.5).

### 2.3 Two grids

`CourseMap` keeps two grids of the same shape:

- `occupancy_map` holds walls and static obstacles as log-odds. The planner
  uses it.
- `dynamic_map` records where moving obstacles have been. The crossing rule in
  section 6 uses it.

Both grids live in one class so they always grow together.

### 2.4 Occupancy map: log-odds

Each cell stores the probability p that the cell is occupied, as a log-odds
value:

```
l = log(p / (1 - p))
```

| p | l | Meaning |
|---|---|---|
| 0.5 | 0 | Unknown. Every cell starts here. |
| 0.9 | +2.2 | Probably occupied. |
| 0.1 | −2.2 | Probably free. |

Combining a new reading with what the map already knows uses Bayes' rule, which
multiplies probabilities. In log-odds form, the multiplication becomes an
addition. For each scan:

- Every cell where a beam ends gets `L_OCC` (+0.85) added.
- Every cell a beam passes through gets `L_FREE` (−0.4) added. The beam is
  sampled every half cell, and stops one cell before its end so noise doesn't
  clear the wall it hit.

`L_FREE` is smaller because a beam passes through many cells but ends in only
one, so one pass says less than one hit.

Each scan updates a cell at most once, even when several beams cross it, which
happens near the robot on every scan. `to_flat_cells()` removes repeated cells
with `np.unique()` before the update.

After each scan, every cell is clamped to [−4, 4]. Without a limit, a cell seen
as free 500 times would need 500 hits to change. With the limit, a few
consistent readings are enough to change any cell.

Three thresholds turn the value into a class:

| Class | Value |
|---|---|
| Occupied | l > 1 |
| Free | l < −1 |
| Unknown | Anything in between |

Each beam starts at the LiDAR, `LIDAR_X` (0.114 m) ahead of the robot's
center. The LiDAR is mounted facing backward (`laser_joint` in the URDF has a
yaw of π), so a beam's angle is the robot's heading plus `LIDAR_YAW` (π) plus
`angle_min + i * angle_increment`. With `angle_min` = −π, beam 0 points forward,
as in CA1. Section 10, step 2, checks this on the real robot.

### 2.5 Static or dynamic

Before a scan updates the map, the robot checks the cell each hit lands in. The
test is that **a beam can't pass through anything static**:

- If every cell within `DYNAMIC_MARGIN` (3 cells, 7.5 cm) of the hit is
  **confidently free** (l < `CONFIDENT_FREE`, −2), many beams have passed
  through that spot, so
  nothing static can be there. The hit is **dynamic**. Its scan number goes into
  `dynamic_map`, and it doesn't add `L_OCC` to the occupancy map.
- Otherwise, the hit is **static** and adds `L_OCC` to its cell as usual.

The beam's pass-through cells update the occupancy map in both cases.

**Why the margin.** Noise or a small pose error can put a wall hit a few cm in
front of the wall. That cell may be confidently free, but the wall within the
margin isn't, so the hit doesn't count as dynamic. In replays of simulation runs
with the pose deliberately 40 ms off, a margin of one cell marked up to 190 wall
cells as dynamic, two cells up to 28, and three cells at most 7. The moving
obstacles were detected the same with all three, since they move through open
floor.

**Why −2.** It's stricter than the free threshold of −1. A cell needs about 6
beams through it, with no hits, to reach −2. At about 10 scans a second, that's
about 0.6 s of seeing a spot as empty, from wherever the robot happens to be.
The robot doesn't need to stop and scan from one place first.

### 2.6 Dynamic map

`dynamic_map` stores, for each cell, the number of the last scan that had a
dynamic hit in it. Cells start at −1, meaning never. This one array answers
both questions the crossing rule asks:

- **Where are moving obstacles now?** Cells whose value is the latest scan
  number (`dynamic_map == scan_count`).
- **Where can moving obstacles go?** Cells whose value isn't −1
  (`dynamic_map >= 0`). This is the **sweep zone**.

Using scan numbers instead of times means the dynamic map has no time constants
to tune.

### 2.7 Range limit

The robot uses a beam to update the map only up to a maximum range. Past that
range, two things go wrong:

- **Heading error moves points.** A point's position error is about range ×
  heading error. With a 1° error, a point at 3 m lands 5 cm, or two cells, from
  where it belongs, so walls look thicker or doubled.
- **Beams spread apart.** Neighboring beams land about range × beam spacing
  apart. Once that is more than one cell, beams skip cells, walls get gaps, and
  the planner can route through those gaps.

The limit comes from the cell size and the beam spacing, so you don't tune it.
`sub_scan_callback()` computes it from each scan:

```python
max_range = CourseMap.CELL_SIZE / msg.angle_increment  # 0.025 / 0.0087 rad ≈ 2.9 m for 720 beams
```

A beam longer than `max_range` marks cells as free up to `max_range`, with no
hit. A beam that returns `inf`, `nan`, or less than `range_min` is skipped
entirely. An `inf` can mean either open space or a dark surface that absorbed
the beam, so it isn't safe to treat as open space.

Ignoring far points loses nothing, because the robot sees those walls again as
it gets closer.

### 2.8 The pose when the scan was taken

The scan and the pose arrive at different times, and a scan can also wait in
the queue while another callback, such as a plan, runs. While the robot turns,
pairing a scan with the latest pose gives a heading error of the turn rate times
that delay. In the simulation, one scan paired with a pose 0.1 s newer, taken
while the robot turned at 0.6 rad/s, put a row of hits 5 cm in front of a wall.
Those hits counted as dynamic, and the robot held there for good.

So the pose callbacks keep the last 100 poses with their stamps, and
`pose_at()` interpolates the pose at the scan's own stamp. If the stamp isn't
between the stored poses, it uses the latest pose. That happens when the pose
source stops publishing while the robot stands still (Gazebo's odometry does),
or when the two clocks disagree.

As a second guard, `drive()` sets `self.turning` whenever the robot turns with
no forward speed, and `sub_scan_callback()` skips scans while it's set. Mapping
resumes as soon as the robot drives forward or stops.

## 3. Moving obstacles

The robot doesn't model how a moving obstacle moves. It doesn't need to know
whether the obstacle slides, swings, or rotates. It only needs the two things
the dynamic map stores (section 2.6): where moving obstacles are now, and the
sweep zone they move through.

When the robot first sees a moving obstacle, the cells it occupies aren't
confidently free yet, so its hits count as static and the occupancy map marks
those cells as occupied. The planner avoids them, which is the safe mistake. As
the obstacle moves away, beams pass through those cells and lower their values.
Once a cell and the cells around it drop below −2, the obstacle's hits there count as
dynamic and stop adding `L_OCC`. After a cycle or two of the obstacle's motion,
the occupancy map shows free floor where the obstacle moves, and the dynamic map
shows the sweep zone.

This works when the obstacle is away from a cell more often than it's in it. A
cell drifts toward free only when the passes outweigh the hits:
`(1 − p) × 0.4 > p × 0.85`, which gives p < 0.32. A cell the obstacle occupies
longer than that can stay occupied in the occupancy map, and the planner treats
it as a wall. To raise the limit, make `L_FREE` stronger, at the risk of real
walls fading and then being marked dynamic.

In practice the limit is pessimistic. The LiDAR only hits the side of the
obstacle facing the robot, and once a cell is confidently free, hits there no
longer add `L_OCC`. In the offline tests (section 10), the ends of the test
obstacle's track were occupied about a third of the time, and no cell in the
track was left as a wall.

## 4. Where to go

### 4.1 Frontiers

The robot doesn't know where the finish is, so it explores. A **frontier** is a
free cell next to an unknown cell: the edge of what the robot has seen. Driving
to a frontier shows the robot more of the course.

A target, whether a frontier or the end of the course, must be in an
**allowed** cell:

- **Ahead of the start line.** The start line passes through the robot's pose
  at the first scan, at right angles to its heading. Cells behind it are
  ignored, so the robot never drives back out of the course. This is the one
  thing the code needs from the setup: **place the robot facing into the
  course.**
- **Not blocked.** The robot's center can't reach a cell within half its width
  of a wall (section 5.2).
- **Not within reach of a moving obstacle** (section 6.1). A target there would
  leave the robot parked where it can be hit.

Rules for choosing a frontier:

1. **The robot goes to the nearest frontier,** measured by path cost, not
   straight-line distance.
2. **The robot keeps its target while the target is still a frontier.** This
   stops the robot from switching between two frontiers that are about the same
   distance away.

### 4.2 The end of the course

When no frontier is left, the robot has seen everything ahead of it. That can
happen before the robot reaches the end. For example, from a door the robot can
often see the whole last section. Stopping there would leave the rest of the
course undriven.

So with no frontier left, the target becomes the **end of the course**: the
allowed free cell that is farthest from the start position by path cost. The
robot keeps that target until a new frontier appears, for example because a
moving obstacle moved and showed space behind it.

Once the robot reaches the end, it stops there. It keeps mapping and
replanning, and drives on if a new frontier appears. The run ends when you press
`Ctrl+C`. If no target can be reached at all, the robot stands still and keeps
replanning.

## 5. Path planning

### 5.1 Dijkstra to choose a target, A* to reach it

`search()` does three jobs, and each one suits a different search.

**Choosing a frontier: Dijkstra.** A* needs to know its goal before it starts,
because its heuristic is the estimated distance to that goal. When the robot
needs a new target, the goal is "the nearest frontier by path cost", and the
robot can't know which frontier that is until it has searched.

Picking the frontier with the shortest straight-line distance and running A* to
it doesn't work. A frontier on the other side of a thin wall can be 0.3 m away
in a straight line but 4 m away by path, while another frontier 1 m ahead is
actually closer.

Dijkstra's algorithm is A* with the heuristic set to zero. It visits cells in
order of path cost from the robot, spreading out evenly in every direction. The
first frontier it visits is the nearest one by path, and the path to it is
already known. One search picks the target and plans the path. The search stops
there, so it rarely visits the whole grid.

**Finding the end of the course: Dijkstra without a goal.** The search runs from
the start position until it has reached every cell it can, and returns each
cell's path cost. The end is the allowed free cell with the highest cost.

**Reaching a known target: A*.** While the robot keeps the same target, the goal
is known, so `plan_to()` uses A* with the octile distance as the heuristic: the
length of the shortest 8-connected path on an empty grid. A* visits only the
cells toward the target, so it's faster than Dijkstra for this job.

All three share the same code. The only differences are the goal and the
heuristic.

### 5.2 Which cells the robot can enter

`planning_grids()` gives every cell a cost. The robot is treated as a circle as
wide as the robot, so it can't enter a cell within half the robot's width of an
occupied cell. Every occupied cell is grown by half the robot's width, rounded
up to whole cells. This is called **inflation**.

| Cell | Cost |
|---|---|
| Occupied | `inf`. Never entered. |
| Inflated | `INFLATED` (1000). A path can leave an inflated cell but never enter one from a cell that isn't. |
| Everything else | 1 to about 4.6, from the clearance cost in section 5.3. |

The rule for inflated cells means a robot that has drifted close to a wall can
still plan its way out, but no plan leads into one.

Inflation uses the half-width, not a larger safety margin. A larger margin can
close a narrow door on the grid. Section 5.3 keeps the robot away from walls
instead.

Unknown cells count as enterable. Otherwise, the robot couldn't plan into space
it hasn't seen yet.

The planner doesn't read the dynamic map. Once the occupancy map has settled,
the sweep zone is free floor, so the planner plans through it. Section 6
handles the timing.

### 5.3 Clearance cost

With inflation alone, the shortest path hugs the inflated cells, so the robot
drives with no margin. To keep the path in the middle of corridors and gaps,
each cell's cost depends on its **clearance**, the number of cells to the
nearest inflated cell:

```python
cost = 1.0 + CLEARANCE_WEIGHT * (clear_max - clearance) / clear_max
```

- A cell next to an inflated cell costs about 4.6.
- A cell `clear_max` or more cells away costs 1.
- `clear_max` is the robot's width in cells, so the rule scales with the robot,
  not the course.

The robot centers itself in a corridor of any width without a margin parameter,
and it still fits through a gap that is only a little wider than itself.
`CLEARANCE_WEIGHT` (4) is high enough that the path swings wide around corners
and static obstacles instead of cutting close.

Clearance comes from growing the inflated cells one cell at a time with
`dilate()` and recording when each cell is reached. Only distances up to
`clear_max` matter, so this takes 9 steps for the 0.225 m robot at 2.5 cm.

### 5.4 When to replan

The robot replans every `REPLAN_PERIOD` (1 s), because the map keeps changing.
It also replans as soon as it finishes a crossing (section 6). Each plan works
from the latest map, so a newly seen wall changes the path within a second.

The path is a list of cell centers. It isn't shortened, because pure pursuit
(section 7) already smooths out the 45° steps of a grid path.

## 6. Crossing moving obstacles

This is the one rule that only applies to moving obstacles.

### 6.1 The crossing

`SWEEP_CLEARANCE` is half the robot's diagonal plus one cell's diagonal, about
0.22 m. Any closer to a moving obstacle and the robot can touch it, whatever its
heading. The extra cell is there because the sweep zone and the path are both
cell centers, and each can be up to half a cell's diagonal from the real
point. Two areas
follow from it:

- **Reach:** every cell within `SWEEP_CLEARANCE` of the sweep zone. Targets are
  never placed here (section 4.1).
- **Crossing:** the first stretch of the path within `SWEEP_CLEARANCE` of the
  sweep zone. `find_crossing()` returns where it starts and ends along the path.

When the path has a crossing, the robot switches from `EXPLORE` to `HOLD`.

### 6.2 Hold and time the windows

While holding, the robot recomputes the path and the crossing on every tick.
The sweep zone keeps growing while the robot watches the obstacle, so a hold
point chosen once could end up inside it.

On each tick in `HOLD`, the robot does the following:

1. **Times the windows.** If either of the last two scans saw a moving obstacle
   within `SWEEP_CLEARANCE` of the crossing, the crossing is blocked. Two scans,
   so a single scan that misses the obstacle doesn't open a short window that
   isn't there. In the simulation, one such 0.08 s window swapped the long and
   short windows, and the robot crossed into the obstacle. The robot records
   how long the crossing stays clear each time, from the moment it turns clear
   until it's blocked again. Each of these is a **window**.
2. **Drives to the hold point,** the last path point before the crossing.
3. **Faces the far end of the crossing,** turning in place until it's within
   `ALIGN_TOL` (3°), then stops. The far end follows the sweep zone, so it
   barely moves when the path is replanned. Facing the carrot instead kept the
   robot turning, and scans taken while turning in place aren't mapped, so the
   window timing went stale.
4. **Decides whether to cross.** It crosses only at the start of a window, and
   only if the window before last was long enough:

   ```python
   cross_time = crossing_length / MAX_SPEED + MAX_SPEED / (2 * MAX_ACCEL)
   if window_started and len(windows) >= 2 and windows[-2] >= cross_time:
   ```

   `crossing_length` runs from the hold point to `ARRIVE_TOL` past the end of
   the crossing. The second term is the time lost ramping up to full speed.

**Why the window before last.** The obstacle repeats its motion, so the window
one full cycle ago shows how long the one starting now will last. The last
window isn't a good guess: near one end of the obstacle's track, the windows
alternate long and short, because the obstacle passes the crossing on its way
out and again on its way back.

**Why not cross as soon as the obstacle leaves.** In testing, a fast obstacle
came back before the robot was across. Timing the windows needs no period,
speed, or motion model, and it works for any obstacle that repeats its motion.

**When no window is long enough.** The last two windows make up one full cycle,
the long one and the short one. If neither is as long as `cross_time`, waiting at
this spot can't help. The robot records the middle of the crossing, goes back to
`EXPLORE`, and replans at once. From then on, cells within a robot's width of that
point cost `INFLATED`, so the new path crosses the obstacle's path somewhere else.
The windows are longest near the ends of the obstacle's path, where its two
passes merge into one, so each new try moves the crossing toward an end. In
the simulation, with the obstacle taking 2.06 s from end to end, crossings in
the middle 0.7 m of its 1.2 m path never had a window longer than 2.09 s. Only
lanes within about 0.2 m of either end did.

### 6.3 Cross

In `CROSS`, the robot follows the path through the crossing and stops
`ARRIVE_TOL` past it. The LiDAR stop (section 8) is off during the crossing. If
the robot stopped in the middle of the sweep zone, the obstacle would hit it.
Continuing is the safer choice. Once across, the robot replans at once and goes
back to `EXPLORE`. If the path crosses several sweep zones, the robot holds
before each one.

### 6.4 Escape

If the robot finds itself within reach of the sweep zone, usually because the
zone grew around it, it leaves by the shortest route. It plans to the nearest
cell that is at least `ARRIVE_TOL` clear of the reach area and not blocked, then
follows that path in `CROSS`. In this one case, the robot can drive backward
(section 7), because turning around would keep it within reach for longer. If
there's no route out, it stops and stays in `HOLD`.

## 7. Driving

The base is holonomic, but the robot drives like a car: it moves forward and
turns to face where it's going. It never drives sideways.

**Pure pursuit.** `carrot()` finds the path point closest to the robot, then
the first point after it that is at least `LOOKAHEAD` (0.2 m) away. That point
is the carrot. `follow()` drives along the arc through the carrot:

```python
speed = MAX_SPEED * math.cos(alpha)          # alpha: angle from the heading to the carrot
turn = 2 * speed * math.sin(alpha) / LOOKAHEAD
```

This is the geometry behind carrot chasing. It gives a turn rate that matches
the speed, instead of a separate PID controller for heading. The robot slows
down as `alpha` grows.

**Minimum turn rate.** The real robot doesn't turn slower than `MIN_TURN`
(0.3 rad/s, from CA2). Smaller turn rates are raised to `MIN_TURN`, or set to 0
once `alpha` is within `ALIGN_TOL`.

**Turn in place.** If `alpha` is larger than 60°, the robot stops and turns in
place until it faces the carrot. The arc for a large angle would leave the path.
The turn rate is `TURN_GAIN × alpha`, between `MIN_TURN` and `MAX_TURN`.

**Reverse.** During a crossing or an escape (section 6), if the carrot is more
than 90° off the heading, the robot backs toward it with pure pursuit from the
rear instead of turning around. If the carrot is more than 60° off the rear as
well, the robot first turns in place until its rear faces the carrot, as it does
when driving forward. Without this, a carrot about 90° to the side scales both
the speed and the turn rate by about cos 90° ≈ 0, and the robot stops dead. In
the simulation it sat at 5 mm/s in front of the moving obstacles in the middle
of an escape.

**Speed.** The robot drives at up to `MAX_SPEED` (0.3 m/s). Forward speed rises
by at most `MAX_ACCEL` (0.5 m/s²) so the robot doesn't jerk, but drops at once,
so it can always stop in time. `follow()` ramps the speed before working out the
turn rate, so the turn matches the speed actually sent. Otherwise the robot
would spin at the full rate while pulling away from a stop.

**Arriving.** The robot has reached the end of a path when the path left from
its closest point is shorter than `ARRIVE_TOL` (5 cm).

## 8. Safety

**LiDAR stop.** If any LiDAR return is within `STOP_DISTANCE` (0.1 m) in front of
the robot's front edge, and within its width, the robot stops. This catches map
mistakes. It uses the raw scan, not the map, so it reacts within one scan. The
next plan routes around whatever the map now shows there.

The stop only looks in front of the robot. A return beside the body would stop
the robot next to a wall it's driving past, and stopping there can't prevent a
collision anyway. During a crossing, the stop is off (section 6.3).

## 9. State machine

| State | What the robot does | Moves to |
|---|---|---|
| `EXPLORE` | Follows the path to the current target. With no path, stands still. | `HOLD` when the path has a crossing. |
| `HOLD` | Drives to the hold point, faces the crossing, and times the windows. | `CROSS` at the start of a long enough window, or to escape. `EXPLORE` when the path no longer has a crossing. |
| `CROSS` | Drives through the crossing, or out of reach, without stopping. | `EXPLORE` at the end of the crossing path. |

Mapping carries on in every state, except while the robot turns in place
(section 2.8). Replanning carries on in `EXPLORE` and `HOLD`.

## Constants

Every constant describes the robot or its sensors. None describes the course.

| Constant | Value | Source |
|---|---|---|
| `CELL_SIZE` | 0.025 m | Decided before testing (section 2.1). |
| `GROW_MARGIN` | 1 m | Section 2.2. |
| `ROBOT_WIDTH`, `ROBOT_LENGTH` | 0.225 m, 0.289 m | CA1. |
| `LIDAR_X`, `LIDAR_YAW` | 0.114 m, π | URDF `laser_joint` (section 2.4). |
| `SWEEP_CLEARANCE` | Computed | Half the robot's diagonal plus a cell's diagonal (section 6.1). |
| `L_OCC`, `L_FREE` | +0.85, −0.4 | Standard log-odds values (section 2.4). |
| `L_MIN`, `L_MAX` | −4, +4 | Section 2.4. |
| `OCCUPIED`, `FREE` | +1, −1 | Section 2.4. |
| `CONFIDENT_FREE` | −2 | Section 2.5. |
| `DYNAMIC_MARGIN` | 3 cells | Pose error (section 2.5). |
| `max_range` | Computed | `CELL_SIZE / angle_increment` (section 2.7). |
| `INFLATED` | 1000 | Section 5.2. |
| `CLEARANCE_WEIGHT` | 4 | Tuning (section 5.3). |
| `REPLAN_PERIOD` | 1 s | Section 5.4. |
| `MAX_SPEED` | 0.3 m/s | Course rules (`max_translate_velocity`). |
| `MAX_ACCEL` | 0.5 m/s² | Tuning (section 7). |
| `MAX_TURN` | 0.6 rad/s | The turn limit in `move_2D()`. |
| `MIN_TURN` | 0.3 rad/s | CA2. |
| `TURN_GAIN` | 1.5 | Tuning (section 7). |
| `LOOKAHEAD` | 0.2 m | Tuning (section 7). |
| `TURN_IN_PLACE` | 60° | Tuning (section 7). |
| `ALIGN_TOL` | 3° | Tuning (sections 6.2 and 7). |
| `ARRIVE_TOL` | 0.05 m | Tuning (section 7). |
| `STOP_DISTANCE` | 0.1 m | Tuning (section 8). |

## Code layout

Everything is in `src/rb2301_fp/rb2301_fp/obstacle_course.py`, using numpy and
the Python standard library only:

- Module-level constants: the robot, driving, and planning constants in the
  table above.
- `dilate()` and `disk()`: grow a grid mask by a set of cell offsets, such as
  the cells within a radius.
- `search()`: the shared Dijkstra and A* search. `plan_path()` and `plan_to()`
  turn its result into a list of cells.
- `ObstacleCourseNode`:
  - `sub_scan_callback()`: stores the scan and adds it to the map.
  - `timer_callback()`: runs `explore()`, `hold()`, or `cross()` for the
    current state.
  - `replan()`: chooses the target and plans the path (sections 4 and 5).
  - `find_crossing()` and `crossing_blocked()`: the crossing rule (section 6).
  - `carrot()`, `follow()`, `turn_in_place()`, and `drive()`: driving
    (section 7).
  - `blocked_ahead()`: the LiDAR stop (section 8).
- `CourseMap`: both grids, growing, the log-odds update, and the grids the
  planner needs (`planning_grids()`, `ahead()`, `frontier()`, `distances()`).

## 10. Testing

### Done so far

The code was tested offline, with ROS stubbed out, in a simulated course: a
corridor with a corner, a static box, a door 0.4 m wide, an obstacle sliding
across the corridor, and a dead end. The simulated LiDAR has 720 beams with 1 cm
of range noise.

| Obstacle period | Result |
|---|---|
| 8 s, 10 s, 12 s | Waits, crosses, and stops at the far end. |
| 4 s, 6 s | No window is long enough, so the robot waits instead of colliding. |

In every run, the obstacle stayed at least 0.215 m from the robot's center, more
than the robot's half-diagonal of 0.183 m. No cell in the obstacle's track was
left as a wall, and no cell outside it was marked dynamic.

### On the real robot

Test each step before starting the next one:

1. **Calibrate.** Run with `--calibrate` and check that the pose matches where
   the robot is.
2. **Check the LiDAR direction.** The code uses the simulation's mounting,
   `LIDAR_YAW` = π and `LIDAR_X` = 0.114 m. Place an object in front of the
   robot and confirm that the map shows it there. If the map comes out rotated
   by 180°, set `LIDAR_YAW` to 0. Both `sub_scan_callback()` and
   `blocked_ahead()` use it.
3. **Mapping.** Push the robot through the course by hand and log the map. Walls
   should be thin and continuous, and only the moving obstacles should appear in
   the dynamic map.
4. **Driving.** Check that the robot follows a path, turns in place, and stops
   for an object in front of it.
5. **Full run without moving obstacles.** Explore the zig-zag and the door.
6. **Crossing.** Test `HOLD` and `CROSS` against an obstacle you move by hand,
   then on the real moving obstacles.

The robot runs ROS Foxy, so Python 3.8. `np.nan_to_num(posinf=...)` needs numpy
1.17 or later.

## Known limits

These limits are accepted to keep the code simple:

- If no window is long enough anywhere along a moving obstacle's path, the
  robot runs out of places to try and stands still on the near side.
- The sweep zone is learned as the obstacle moves. If the robot reaches a track
  before it has seen the obstacle cover the part next to its path, it can stop
  closer than `SWEEP_CLEARANCE` and has to escape once the zone grows. In
  Gazebo this happened right after the gate.
- If a static cell is wrongly marked dynamic near the path, the robot waits for
  a window that never comes.
- A cell that a moving obstacle occupies more than about 30% of the time can
  stay occupied in the occupancy map (section 3).
- If the course has no wall at the end, the robot keeps exploring.
- The robot must start facing into the course.
