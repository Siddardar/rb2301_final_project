# Final project plan

This plan describes how the robot gets through the obstacle course on the real
robot. The code doesn't know the layout of the course, the order of its
sections, or where the finish is. The robot maps what it sees, drives toward
space it hasn't seen yet, waits for moving obstacles to clear its path, and
stops when there's nothing left to explore.

## Design rules

- **No course parameters.** The code has no constants that describe the course,
  such as door positions, the number of moving obstacles, or a finish line. It
  only has constants that describe the robot and its sensors. These are listed
  in [Constants](#constants).
- **No special case for each section.** The zig-zag, the static obstacles, and
  the door all use the same map and planner. Moving obstacles add one rule. The
  sections can come in any order.
- **Keep it simple.** The plan has no fallbacks or timeouts. If the robot has
  nowhere left to go, it stops. If the course has no wall at the end, the robot
  keeps driving.

## Overview

The robot runs three loops:

| Loop | Rate | What it does |
|---|---|---|
| Mapping | Every LiDAR scan, about 10 Hz | Updates the map and finds moving obstacles. |
| Planning | When needed, at most about 1 Hz | Chooses where to go next and plans a path there. |
| Control | Every timer tick, 20 Hz | Runs the state machine and sends `move_2D()` commands. |

Mapping and planning run at the same time: the map never stops updating, and
each plan uses the latest map. The robot doesn't map first and plan afterward.

In practice, ROS runs one callback at a time by default, so the loops take turns
instead of running in parallel. That's fine while each callback is short. A
mapping update is a few vectorized numpy calls and takes a few milliseconds. A
plan can take longer. While it runs, the robot misses scans and control ticks.
If that becomes a problem, planning moves to a background thread that works on a
copy of the map (section 5.5).

The sections that follow explain each part.

## 1. Pose: mapping instead of SLAM

SLAM (simultaneous localization and mapping) builds a map and works out the
robot's pose at the same time. You need it when nothing else tells the robot
where it is, so the robot has to use the map to locate itself.

On the real robot, Optitrack gives the pose with millimeter accuracy. Any pose a
LiDAR-based SLAM method estimates would be less accurate. The robot only needs
the mapping half: it takes the Optitrack pose as correct and places each LiDAR
point on the map from that pose.

## 2. The occupancy grid

### 2.1 Cells

The map is a 2D numpy array. Each element is a cell: a square of floor whose
size you choose. Converting between world coordinates and cells uses that size:

```python
RESOLUTION = 0.025                      # metres per cell
col = int((x - origin_x) / RESOLUTION)
row = int((y - origin_y) / RESOLUTION)
```

The origin is chosen so the grid covers a generous area around the start pose,
for example 10 m × 10 m.

**Why 2.5 cm.** This value balances three things:

- **Narrow gaps stay open on the grid.** Each wall edge can be up to one cell
  off, so a gap can shrink by up to two cells. At 2.5 cm, a gap loses at most
  5 cm. At 10 cm, a door 0.4 m wide loses half its width.
- **It's close to the sensor's accuracy.** LiDAR range error is around 1–2 cm.
  Smaller cells make the map bigger, not more accurate.
- **Planning stays fast.** A 10 m × 10 m grid has 160,000 cells at 2.5 cm and
  1,000,000 at 1 cm. Python search code visits cells one at a time, so the count
  matters.

The resolution is decided once before testing and then stays fixed. If the real
gaps are wide, 5 cm also works and makes planning about four times faster.

### 2.2 Two maps

The robot keeps two grids of the same size and resolution:

- The **static map** holds walls and static obstacles. The planner uses it.
- The **dynamic map** records where moving obstacles have been. The crossing
  rule in section 6 uses it.

Each map has one job, so neither has to represent both kinds of obstacle.

### 2.3 Static map: log-odds

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
addition. For each beam, the robot computes the cells the beam crosses:

- The cell where the beam ends gets `L_OCC` added, for example +0.85.
- Every cell before it gets `L_FREE` added, for example −0.4.

`L_FREE` is smaller because a beam passes through many cells but ends in only
one, so one pass says less than one hit.

After each scan, every cell is clamped to [−4, 4]. Without a limit, a cell seen
as free 500 times would need 500 hits to change. With the limit, a few
consistent readings are enough to change any cell.

Three thresholds turn the value into a class:

| Class | Value |
|---|---|
| Occupied | l > 1 |
| Free | l < −1 |
| Unknown | Anything in between |

The updates use `np.add.at()`, because plain `grid[rows, cols] += value` updates
a cell only once even when several beams cross it, which happens near the robot
on every scan.

Each beam's angle comes from the scan message (`angle_min + i *
angle_increment`) plus the LiDAR's mounting yaw. Each beam starts at the LiDAR's
position, not at the robot's center. Section 10, step 2, checks both on the real robot.

### 2.4 Static or dynamic

Before a scan point updates the static map, the robot checks the cell the point
lands in. The test is that **a beam can't pass through a wall**:

- If the cell is **confidently free** (l < −2), many beams have passed through
  it, so nothing static can be there. The point is **dynamic**. It goes into the
  dynamic map and doesn't add a hit to the static map.
- Otherwise, the point is **static** and adds `L_OCC` to the cell as usual.

The beam's pass-through cells update the static map in both cases.

The threshold −2 is stricter than the free threshold −1. A cell at a wall edge
sometimes gets a grazing beam through it, but it also gets hits, so it rarely
reaches −2. This keeps wall edges from being treated as dynamic. Check the value
on recorded data (section 10, step 3).

### 2.5 Dynamic map

The dynamic map stores, for each cell, the number of the last scan that had a
dynamic point in it. Cells start at −1, meaning never. This one array answers
both questions the crossing rule asks:

- **Where are moving obstacles now?** Cells whose value is the latest scan
  number.
- **Where can moving obstacles go?** Cells whose value isn't −1. This is the
  **sweep zone**.

Using scan numbers instead of times means the dynamic map has no time constants
to tune.

### 2.6 Range limit

The robot uses a beam to update the map only up to a maximum range. Past that
range, two things go wrong:

- **Heading error moves points.** A point's position error is about range ×
  heading error. With a 1° error, a point at 3 m lands 5 cm, or two cells, from
  where it belongs, so walls look thicker or doubled.
- **Beams spread apart.** Neighboring beams land about range × beam spacing
  apart. Once that is more than one cell, beams skip cells, walls get gaps, and
  the planner can route through those gaps.

The limit comes from the cell size and the beam spacing, so you don't tune it:

```python
MAX_RANGE = RESOLUTION / angle_increment  # 0.025 / 0.0087 rad ≈ 2.9 m for 720 beams
```

A beam longer than `MAX_RANGE` marks cells as free up to `MAX_RANGE`, with no
hit. A beam that returns `inf` or `0` is skipped entirely. An `inf` can mean
either open space or a dark surface that absorbed the beam, so it isn't safe to
treat as open space.

The real LiDAR publishes about 720 beams. The starter `sub_scan_callback()`
keeps every second one. The map uses all of them, which doubles `MAX_RANGE`.

Ignoring far points loses nothing, because the robot sees those walls again as
it gets closer.

### 2.7 No updates while turning in place

The scan and the Optitrack pose arrive at slightly different times. While the
robot turns in place, that small delay becomes a large heading error, which
smears walls across the map. The robot doesn't update the map during the
turn-in-place step (section 7). It scans as usual once it drives forward again.

## 3. Moving obstacles

The robot doesn't model how a moving obstacle moves. It doesn't need to know
whether the obstacle slides, swings, or rotates.

It only needs the two things the dynamic map stores (section 2.5): where moving
obstacles are now, and the sweep zone they move through.

When the robot first sees a moving obstacle, the cells it occupies aren't
confidently free yet, so its points count as static and the static map marks
those cells as occupied. The planner avoids them, which is the safe mistake. As
the obstacle moves away, beams pass through those cells and lower their values.
Once a cell drops below −2, the obstacle's points in it count as dynamic and stop
adding hits. After a cycle or two of the obstacle's motion, the static map shows
free floor where the obstacle moves, and the dynamic map shows the sweep zone.

This works when the obstacle is away from a cell more often than it is in it.
With the example values, a cell drifts toward free when the obstacle is in it
less than about 30% of the time, because 0.3 × 0.85 ≈ 0.7 × 0.4. A cell the
obstacle occupies longer than that stays occupied in the static map, and the
planner treats it as a wall.

## 4. Where to go: frontiers

The robot doesn't know where the finish is, so it explores. A **frontier** is a
free cell next to an unknown cell: the edge of what the robot has seen. Driving
to a frontier shows the robot more of the course.

Rules for choosing a frontier:

1. **Only frontiers ahead of the start line count.** The start line passes
   through the start position, at right angles to the start heading. Frontiers
   behind it are ignored, so the robot never drives back out of the course. This
   is the one thing the code needs from the setup: **place the robot facing into
   the course.**
2. **The robot goes to the nearest frontier,** measured by path length, not
   straight-line distance.
3. **The robot keeps its target until the target is no longer a frontier,**
   which happens once the robot sees the unknown cells around it. This stops the
   robot from switching between two frontiers that are about the same distance
   away.

**Stopping.** If no frontier is reachable ahead of the start line, everything
around the robot is mapped and closed off, so the robot stops. It keeps mapping
while it waits. If a new frontier appears, for example because a moving obstacle
moved and showed space behind it, the robot drives on. The run ends when you
press `Ctrl+C`.

## 5. Path planning

### 5.1 Dijkstra to choose a target, A* to reach it

The planner does two different jobs, and each one suits a different search.

**Choosing a target: Dijkstra.** A* needs to know its goal before it starts,
because its heuristic is the estimated distance to that goal. When the robot
needs a new target, the goal is "the nearest frontier by path length", and the
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

**Reaching a known target: A*.** While the robot keeps the same target (rule 3 in
section 4), the goal is known, so replanning uses A* with the straight-line
distance as the heuristic. A* visits only the cells toward the target, so it is
faster than Dijkstra for this job.

Both searches share the same code. The only difference is the heuristic: zero
for Dijkstra, the straight-line distance for A*.

### 5.2 Which cells the robot can enter

The robot is treated as a circle as wide as the robot, so it can enter a cell
only if no occupied cell is within half the robot's width. To do this, every
occupied cell is grown by half the robot's width (called **inflation**), and the planner
stays out of grown cells.

Inflation uses the half-width, not a larger safety margin. A larger margin can
close a narrow door on the grid. Section 5.3 keeps the robot away from walls
instead.

Unknown cells count as enterable. Otherwise, the robot couldn't plan into space
it hasn't seen yet.

The planner doesn't read the dynamic map. Once the static map has settled, the
sweep zone is free floor, so the planner plans through it. Section 6 handles the
timing.

### 5.3 Clearance cost

With inflation alone, the shortest path hugs the inflated walls, so the robot
drives with no margin. To keep the path in the middle of corridors and gaps,
each cell's cost depends on its **clearance**, the distance to the nearest
inflated cell:

```python
cost = 1 + max(0, (CLEAR_MAX - clearance) / CLEAR_MAX)
```

- A cell right next to an inflated cell costs 2.
- A cell `CLEAR_MAX` or more away costs 1.
- `CLEAR_MAX` is the robot's width, so the rule scales with the robot, not the
  course.

The robot centers itself in a corridor of any width without a margin parameter,
and it still fits through a gap that is only a little wider than itself.

Clearance comes from shrinking the free area one cell at a time with numpy
slicing, and recording when each cell disappears. Only distances up to
`CLEAR_MAX` matter, so this takes about `CLEAR_MAX / RESOLUTION` steps, which is
8 steps for a 0.2 m robot at 2.5 cm.

### 5.4 Path shortening

A grid path moves in steps of 45°, so it has many small turns. After the search,
the robot removes every path point it can skip: if the straight line from point A
to point C has at least as much clearance as the path through B, B is dropped.
This produces long straight segments, as Theta* does, without cutting corners.
Fewer turns also means fewer turns in place.

### 5.5 When to replan

The robot replans when any of these happens:

- The target is no longer a frontier.
- A newly occupied cell blocks the current path.
- One second has passed since the last plan, because the map keeps changing.

Planning starts inside the timer callback. If it is too slow on the robot's
computer, it moves to a background thread. Measure the time first.

## 6. Crossing moving obstacles

This is the one rule that only applies to moving obstacles.

1. **Hold.** When the planned path enters the sweep zone, the robot drives to the
   last path point before the zone (section 2.5), leaving half its length as
   space. It stops
   there, facing along the path. The cells where the path crosses the sweep zone,
   grown by half the robot's width, are the **crossing cells**.
2. **Wait for the obstacle to leave.** While the robot holds, it checks every
   scan whether the dynamic map marks any crossing cell with the latest scan
   number. It waits until the crossing goes from blocked to clear.
3. **Cross.** At that moment, the robot drives through the crossing at full
   speed.

**Why wait for the obstacle to leave instead of for a clear crossing.** A clear
crossing could be about to close: the obstacle might be on its way back. The
moment an obstacle leaves the crossing is the start of the longest clear window,
because the obstacle is moving away. This works for any obstacle that moves back
and forth or around a cycle, and it needs no period, speed, or motion model.

**During the crossing,** the safety stop (section 8) ignores dynamic points. If
the robot stopped in the middle of the sweep zone, the obstacle would hit it.
Continuing at full speed is the safer choice. If the path crosses several sweep
zones, the robot holds before each one.

## 7. Driving

The base is holonomic, but the robot drives like a car: it moves forward and
turns to face where it's going. It never drives sideways.

**Pure pursuit.** The robot picks a target point a fixed distance `LOOKAHEAD`
ahead along the path. It then drives along the arc that passes through that
point:

```python
alpha = angle_to_target - heading           # wrapped to [-pi, pi)
turn = 2 * v * np.sin(alpha) / LOOKAHEAD     # turn rate for that arc
```

This is the geometry behind carrot chasing. It gives a turn rate that matches the
speed, instead of a separate PID controller for heading.

**Turn in place.** If `alpha` is larger than 60°, the robot stops and turns in
place until it faces the target. The arc for a large angle would leave the path.

**Speed.** The robot drives at 0.3 m/s and slows down as `alpha` grows. Speed and
turn rate change by at most a fixed acceleration per tick, so the robot doesn't
jerk.

## 8. Safety

- **LiDAR stop.** If a static scan point is within `STOP_DISTANCE` of the robot's
  footprint in the direction of travel, the robot stops. This catches map
  mistakes. During a crossing it ignores dynamic points (section 6).
- **Lost pose.** If no Optitrack pose arrives for 0.2 s, the robot stops until
  the pose comes back.

## 9. State machine

| State | What the robot does | Moves to |
|---|---|---|
| `EXPLORE` | Follows the path to the current frontier. | `HOLD` when the path enters a sweep zone. `IDLE` when no frontier is reachable. |
| `HOLD` | Waits at the hold point, facing the crossing. | `CROSS` when the crossing goes from blocked to clear. |
| `CROSS` | Drives through the crossing at full speed. | `EXPLORE` once the robot is out of the sweep zone. |
| `IDLE` | Stands still and keeps mapping. | `EXPLORE` when a frontier appears. |

## Constants

Every constant describes the robot or its sensors. None describes the course.

| Constant | Value | Source |
|---|---|---|
| `RESOLUTION` | 0.025 m | Decided before testing (section 2.1). |
| `ROBOT_WIDTH`, `ROBOT_LENGTH` | Measure | The real robot. |
| LiDAR offset and mounting yaw | Measure | The real robot (section 10, step 2). |
| `L_OCC`, `L_FREE` | +0.85, −0.4 | Standard log-odds values (section 2.3). |
| `L_MIN`, `L_MAX` | −4, +4 | Section 2.3. |
| Occupied and free thresholds | +1, −1 | Section 2.3. |
| `CONFIDENT_FREE` | −2 | Sensor noise (section 2.4). |
| `MAX_RANGE` | Computed | `RESOLUTION / angle_increment` (section 2.6). |
| `CLEAR_MAX` | `ROBOT_WIDTH` | Section 5.3. |
| `LOOKAHEAD` | 0.25 m | Tuning (section 7). |
| `MAX_SPEED` | 0.3 m/s | Course rules (`max_translate_velocity`). |
| `MAX_ACCEL` | Tune | Tuning (section 7). |
| `TURN_IN_PLACE` | 60° | Tuning (section 7). |
| `STOP_DISTANCE` | 0.15 m | Tuning (section 8). |

## Code layout

Everything stays in `src/rb2301_fp/rb2301_fp/obstacle_course.py`, using numpy
only:

- `StaticMap`: the log-odds grid, inflation, and clearance.
- `DynamicMap`: the last-seen scan number for each cell.
- `plan_path()`: the shared Dijkstra and A* search, frontier test, and path
  shortening.
- `pure_pursuit()`: the turn rate for a path and pose.
- `sub_scan_callback()`: stores the full scan and updates both maps.
- `timer_callback()`: runs the state machine and sends `move_2D()` commands.

## 10. Build and test order

Each step is tested before the next one starts.

1. **Record data.** Extend `--calibrate` to save each pose and scan to a `.npz`
   file. Push the robot through the course by hand while it records. Steps 3
   and 4 then run on your laptop with real data, without the robot.
2. **Check the LiDAR.** On the recording, place an object in front of the robot
   and confirm which beam sees it and in which direction the angles increase.
   Measure the LiDAR's offset from the Optitrack position.
3. **Mapping.** Build the grid from the recording and plot it. Walls should be
   thin and continuous, and only the moving obstacles should appear in the
   dynamic map. Check the `CONFIDENT_FREE` threshold in section 2.4 here.
4. **Planning.** Run the frontier search and A* on recorded maps, and plot
   the paths.
5. **Driving.** On the robot, follow a path to a fixed point with pure pursuit
   and the safety stop.
6. **Full run without moving obstacles.** Explore the zig-zag and the door.
7. **Crossing.** Test `HOLD` and `CROSS` against an obstacle you move by hand,
   then on the real moving obstacles.

## Known limits

These limits are accepted to keep the code simple:

- If a moving obstacle never enters the crossing while the robot holds, the robot
  waits there.
- If the course has no wall at the end, the robot keeps exploring.
- The robot must start facing into the course.
- A cell that a moving obstacle occupies more than about 30% of the time stays
  occupied in the static map (section 3).
