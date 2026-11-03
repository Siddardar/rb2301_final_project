# Notes

Working notes for the RB2301 final project: how to run things, what the robot
gives you, and what earlier attempts learned. The course map is in `plan.md`.

## Rules for the code (from the user)

- **No hard-coded coordinates.** The code must not contain any positions from
  the map: no door x/y, corridor lengths, wall positions, turn points, or
  finish line. The map in `plan.md` helps us understand the course, but it
  must not affect what the robot does. Everything about where things are comes
  from the LiDAR at run time.
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
- **Keep it simple.** No unnecessary checks or fallbacks. When something fails, stop and exit with the reason, rather
  than trying to recover. Test runs are capped at 2 minutes.
- The user runs things from a Mac, at least for viewing maps, so debugging
  tools must work without ROS.

## Running the simulation

This repo root is the ROS 2 workspace: `src/`, `fp.sh`, and `gz_fp.sh` are
already in place. The machine has Ubuntu 24.04 with ROS 2 Jazzy and Gazebo
Harmonic in `/opt/ros/jazzy`.

Build once, and again after adding files or changing `setup.py`:

```bash
cd ~/Projects/rb2301_final_project
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

With `--symlink-install`, edits to `obstacle_course.py` take effect without a
rebuild.

**The VM needs a rendering fix.** This machine is a UTM/QEMU VM on an Apple M2 Pro. Its virtual GPU only does
OpenGL 2.1, and Gazebo's renderer needs 3.3. The fix, copied from `~/Projects/rb2301_ca1`, is in
`fp_gazebo.launch.py`: keep `LIBGL_ALWAYS_SOFTWARE=0` and set `MESA_LOADER_DRIVER_OVERRIDE=kms_swrast`. Don't use
`LIBGL_ALWAYS_SOFTWARE=1`: on Mesa 25 it segfaults Gazebo. Headless Gazebo (`-s`, `--headless-rendering`) doesn't
work here at all. After launch, `/scan` can take over a minute to start publishing.

Run in two terminals, both from the repo root:

```bash
./gz_fp.sh   # terminal 1: Gazebo. Wait until the course and robot appear.
./fp.sh      # terminal 2: the node
```

- To start somewhere else, use `ros2 launch rb2301_gz fp_gazebo.launch.py
  x:=3.8 y:=-2.6 yaw:=0.0`. Yaw is in radians. The commented line in
  `gz_fp.sh` starts the robot past the door.
- If Gazebo hangs on "Requesting list of world names", an old Gazebo is still
  running. Run `pkill -9 ruby`. `gz_fp.sh` also runs it when the launch exits.
- `./fp.sh` waits for the first LiDAR scan, so start Gazebo first.

Useful checks while it runs:

```bash
ros2 topic list                       # /scan /odom /cmd_vel /imu /clock /tf /joint_states
ros2 topic echo /odom --once
ros2 topic echo /scan --once --field ranges
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}"   # stop the robot
```

## The random map

- Every launch runs `obstacle_modifier.py`, which copies
  `obstacle_course_world_fp_original.sdf` to `obstacle_course_world_fp.sdf` and
  randomises it. Never edit the original.
- The generated file is tracked in git, so it shows as modified after every
  launch. Don't commit those changes by accident.
- The launch only prints the static obstacle positions. To get the actual
  offsets, door position, and obstacle speeds for a run, read the generated
  `.sdf`. This is useful for checking the code, but the code must not read it.
- For repeatable tests, set the `randomise_...` switches at the top of
  `obstacle_modifier.py` to `False`.
- The ranges and edge cases are in `plan.md`.

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
- `self.is_simulation`: set from the scan size.
- The timer runs at 20 Hz (0.05 s).

Speed limits are clipped inside `move_2D()`:

| | Linear | Turn |
|---|---|---|
| Simulation | 1.4 m/s | 2.8 rad/s |
| Real robot | 0.3 m/s (don't change it) | 0.6 rad/s |

## The simulated robot

- **Drive.** The Gazebo `VelocityControl` plugin applies the `cmd_vel` x, y and
  turn directly, so the robot is holonomic in the simulation and never slips.
  We don't use sideways motion anyway (see the rules).
- **Size.** 0.289 m long and 0.225 m wide (from CA1).
- **Pose origin.** `base_footprint`, at the rear axle. The front wheels are at
  x = +0.144 m, so the body centre is ahead of the pose, not on it.
- **Odometry.** `/odom` is ground truth with no noise. Gazebo stops
  publishing it while the robot stands still, so the pose timestamp can go
  stale.
- **LiDAR.**
  - 360 beams at 10 Hz, range 0.05 to 10 m, no noise. Beams with no return come
    back as `inf`.
  - Mounted at x = +0.114 m from the pose origin and about 0.1 m above the
    floor, **facing backward** (`laser_joint` yaw = π).
  - Because of that mounting and `angle_min` = −3.14, index `i` points about
    `i` degrees counter-clockwise from the robot's forward direction:
    `scan[0]` is forward, `scan[90]` is left, `scan[180]` is behind, and
    `scan[270]` is right. The step is 6.28 / 359 rad, about 1.002°.
  - The real LiDAR has more beams and the node keeps every second one.
- **Obstacle visibility.** The coke can is 0.145 m tall and only 0.06 m wide,
  so at the LiDAR height it returns only a few beams. Check that it shows up.
- **Moving obstacles** have no collision in the simulation, so touching them
  doesn't stop the robot. They still show up on the LiDAR. Treat any contact as
  a failure.

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
- `./robot-test.sh NN` (from the `main` branch) does the sync, build, and
  launch in one step.
- The pose is in the **CA2 maze frame**, so the start isn't `(0, 0)` and
  heading 0 isn't necessarily the start direction. Work relative to the start
  pose (record it on the first tick) rather than in absolute coordinates. This
  also follows from the no-hard-coded-coordinates rule.
- The robot runs ROS Foxy with Python 3.8. Don't use `match`, `X | Y` type
  hints, or other newer syntax.
- The real course is "real-world", so its dimensions won't match the
  simulation exactly. That's another reason everything has to be measured
  with the LiDAR.
