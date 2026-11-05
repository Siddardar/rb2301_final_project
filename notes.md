# Notes

Working notes for the RB2301 final project: how to run things, what the robot
gives you, and what earlier attempts learned.

## Rules for the code (from the user)

- **No hard-coded coordinates.** The code must not contain any positions from
  the map: no door x/y, corridor lengths, wall positions, turn points, or
  finish line. Everything about where things are comes from the LiDAR at run
  time.
- **Section structure may be used.** The code may know the order of the
  sections, as long as it detects each change from the LiDAR, never from a
  position:
  1. **Section 1:** from the start until the right turn.
  2. **Section 2:** from the right turn until the robot is through the door.
  3. **Section 3:** the moving obstacles, after the door.
- **Frames are confirmed.** Don't spend time checking them. Heading 0° is the
  direction the robot faces at the start, and angles are positive
  counter-clockwise, so the right turn is toward −90°.
- Robot and sensor constants, such as the robot's size and LiDAR offset, are
  fine. They describe the robot, not the course.
- **Drive forward and steer.** Normal driving is `move_2D(speed, 0, turn)`, slowing on sharp turns. The robot can
  strafe, but only as a last resort, when nothing within 45° of its heading is clear.
- **Keep it simple.** No unnecessary checks or fallbacks. On the real robot nothing exits early: when there's no path
  it stops, logs the details and keeps replanning; when it's done it holds still until Ctrl+C.
- The user runs things from a Mac, at least for viewing maps, so debugging
  tools must work without ROS.

## What the node gives you

Everything goes in `ObstacleCourseNode.timer_callback()` in
`src/rb2301_fp/rb2301_fp/obstacle_course.py`, between the two
`###### INSERT CODE HERE ######` lines. Helpers can go anywhere in the file.

- `self.pose`: `[x, y, heading_deg]`. Heading is 0 along +x and
  counter-clockwise positive. It's `None` until the first pose arrives.
- `self.last_scan`: numpy array of ranges. It doesn't exist until the first scan
  arrives, so check with `hasattr()`.
- `self.move_2D(x, y, turn)`: velocities in the robot's own frame. x is
  forward, y is left, and turn is rad/s counter-clockwise.
- The timer runs at 20 Hz (0.05 s).

Speed limits are clipped inside `move_2D()`: `max_translate_velocity` for
linear speed, and twice that in rad/s for turning.

## What earlier attempts tried

All four were abandoned, and we're starting from scratch, so this is only a
record of what was tried. Don't reuse their designs or the old `PLAN.md`.
Branch `fresh` is back to the starter code.

| Commit (branch) | Approach |
|---|---|
| `3a6358e` (`main`) | General explorer: a log-odds occupancy grid, frontier exploration with Dijkstra and A*, pure pursuit, and timing crossing windows from a "dynamic" map. No course knowledge at all. Also adds `tools/view_map.py` and `robot-test.sh`. |
| `ea39603` (`testing`) | Reactive state machine with hard-coded corner distances, lane steering, door detection, a moving-obstacle tracker with a bounce model, and stuck recovery. About 1,100 lines. |
| `5dc44e1` (`fresh`) | No map, done in stages: follow a corridor until one side opens, line up on the door, then track each moving obstacle's speed and cross when it's safe. |
| `2f8e2fd` (`fresh`) | Leg by leg, with no map: drive each corridor centred, slide sideways to line up with the door, then hold the heading, wait at each moving obstacle, and dash past. |

## Real robot

The full steps are in `README.md`, under "Real robot". In short:

- `rsync` the package to the robot, build with `colb`, start `basecontrol`,
  `rplidar`, and `vrpn`, then run `./fp.sh`. Add `--calibrate` to only print
  the pose.
- `./robot-test.sh NN` does the sync, build, and launch in one step.
- The pose is in the **CA2 maze frame**, so the start isn't `(0, 0)` and
  heading 0 isn't necessarily the start direction. Work relative to the start
  pose (record it on the first tick) rather than in absolute coordinates. This
  also follows from the no-hard-coded-coordinates rule.
- The robot runs ROS Foxy with Python 3.8. Don't use `match`, `X | Y` type
  hints, or other newer syntax.
- The real course is "real-world", so its dimensions aren't exact. That's
  another reason everything has to be measured with the LiDAR.
