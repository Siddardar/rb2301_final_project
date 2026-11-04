# Map and A* version (trial)

A trial replacement for the reactive steering in `plan.md`: the clear-heading
steering, gap search, strafing, back-off and footprint check. The scan-based
detection stays exactly as it is:

- **Section 1:** the right turn's corner, the end wall, the right wall's line,
  and the waypoint halfway along the opening.
- **Section 2:** the door as a notch with two posts.

Only how the robot gets to each goal changes.

## The map

- **Grid:** a hits/passes count grid in the start frame, with 2.5 cm cells
  (`CELL`). A cell is **occupied** if `hits > passes`.
- **Grows on demand:** when a scan has points outside the map, the map is padded
  on that side by the missing cells plus `GROW` (1 m) of unknown border. It only
  ever covers the area the LiDAR has seen. That keeps A* bounded: when there's no
  route, it gives up quickly instead of searching a huge empty grid.
- **No sections:** one map for the whole run. Sections only change the goal.

## Planning

- **Blocked cells:** occupied cells grown by `INFLATE` = `ROBOT_WIDTH / 2 +
  MARGIN` (0.1625 m). Cells around the robot itself are unblocked, so it can
  plan away from something it's next to. Unknown cells are passable.
- **Clearance cost:** a free cell within `2 × MARGIN` beyond the blocked cells
  costs `CLEARANCE_COST` (3) times as much to cross. Paths keep to the middle
  when there's room, but can still squeeze through narrow gaps. Without it, the
  path hugged the blocked edge, and the robot touched obstacles in two of three
  runs.
- **A\*:** 8-connected, replanned after every scan. It has two kinds of goal:
  - **Goal line,** used before a waypoint is known: any cell at least `AHEAD`
    (1 m) further along a direction than the robot. This is the A* version of
    "aim for 0°" or "aim for −90°". It finds the cheapest way to make that much
    progress, going round whatever is in front.
  - **Goal area:** any cell within a radius of a point. Being near the point is
    good enough, so an obstacle sitting exactly on it doesn't make the goal
    unreachable.
- **No path:** stop and exit with `No path`. That's the clean "no route" failure.

## Goals

| Stage | Goal |
|---|---|
| Section 1, turn not found | Goal line, 0° |
| Section 1, turn found | Goal line: any cell `AHEAD` below the right wall's line. The scan waypoint is only the handover marker, drawn in Gazebo, so A* finds its own way through the opening and round anything in its mouth |
| Section 2, door not found | Goal line, −90° |
| Section 2, door found | Area within `MARGIN` of the line-up point, `LOOKAHEAD` in front of the door's centre |
| Section 2, door locked (inside the line-up area or past it) | No A*: drive straight along the door's centre line, as before. A 0.4 m door leaves only 7.5 cm of free path once both sides are inflated, too little for a grid |

The finish conditions are unchanged. Section 1 is done when the body is past
the right wall's line, heading within 45° of −90°. Section 2 is done when the
whole body is past the door's line.

## Driving

Pure pursuit along the planned path:

- **Carrot:** the first path point `PATH_LOOKAHEAD` (0.25 m) past the closest
  one. That's shorter than `LOOKAHEAD`, so it cuts corners less. The straight
  drive through the door uses `LOOKAHEAD`.
- **Steering:** `turn = 2·SPEED·sin(α)/L` and `speed = SPEED·cos(α)`, never
  negative, where `L` is the lookahead in use (`PATH_LOOKAHEAD` on the path,
  `LOOKAHEAD` through the door). If the carrot is behind the robot, it turns in
  place.

**No footprint guard.** One was tried: before each move, check the body plus
`MARGIN` still fits 0.5 s ahead, and otherwise go straight, turn on the spot or
wait. It stopped the contacts, but the robot then got stuck at the start in all
three test cases. It's been removed; see `improvements.md`.

There's no strafing.

## Seeing the map

With `--snapshots DIR`, the node saves `map_<scan>.npz` to `DIR` once a second,
and once more on exit:

- the map (occupied, free, unknown, using the same rule as the planner)
- the A* path and its end
- the right wall's line
- the door (centre and direction)
- the robot, its trail, and the section

`tools/view_map.py` draws them. It needs only numpy and matplotlib, so it runs on
the Mac. `./robot-test.sh NN --map` passes `--snapshots` on the robot, copies the
snapshots to `maps/<session>` and shows them live.

With `--calibrate`, the node now builds the map without planning or driving, so
you can push the robot through the course by hand and watch the map form.

## Speed on the real robot

On the robot, one plan (including inflating the map) took 150–375 ms, against
about 20 ms in the simulation. Scans and the 20 Hz control loop share one
thread, so planning on every scan starved both. So:

- **Plans every `REPLAN_PERIOD` (0.5 s).** Every scan still goes into the map,
  and the robot follows its current path between plans.
- **Scan queue of 1.** While the node is busy, only the newest scan waits. A
  scan that waited longer would be placed with the robot's newer pose, rotated by
  however much the robot had turned since, which smeared walls into arcs.
- **Snapshots by the clock** (every 1 s), not by control ticks, which ran late.

## Test results

Three random layouts. The first try had no clearance cost, used `LOOKAHEAD` for
the path, and gave the turn's waypoint as an area goal:

| Layout | First try | Now |
|---|---|---|
| 1 | Stuck near a drawer in the mouth | Done, 40 s, 14 cm closest to an obstacle (needed the lock fix below) |
| 2 | Done, but touched a bin and a drawer | Done, 40 s, 8.5 cm closest |
| 3 | Stuck, in contact with a drawer in the mouth | Done, 43 s, 6.7 cm closest |

**Lock fix.** The line-up point is `LOOKAHEAD` in front of the door, and the
door used to lock only strictly closer than that. The robot reached its A* goal
exactly there, its path shrank to one point, and it stopped. The door now locks
once the robot is inside the line-up area (`along > −(LOOKAHEAD + MARGIN)`).

**Door frame clearance is about 3–4 cm.** The robot arrives at the line-up point
at an angle (one run at −38°). The centre-line drive then has only 0.4 m to
straighten up, so it enters the door slightly off-centre.

## What it removes

`clear_heading`, `free`, `fits`, `search_gap`, `slide_away`, and the constants
`STRIP`, `HEADING_STEP`, `FORWARD`, `STRAFE_SPEED` and `AHEAD_TIME`.

## Known risks

- **Inflation by the half-width only.** The robot's corners can still swing
  closer than `MARGIN` on tight turns.
- **Scans taken while turning** can smear walls in the map.
- **The door posts** still use the end wall's x as the left wall. The tilt fix
  (posts must lie on a long wall in the map) comes later.
- **Moving obstacles** (section 3) will smear across the map. Section 3 needs its
  own approach.
