# RB2301 Final Project - Obstacle Course

The robot has to get through three sections of a "real-world" obstacle course: a zig-zag corridor with static obstacles, a small door, and dynamic swinging obstacles. You get the robot's pose and LiDAR scan. There is no occupancy grid, and the walls are rectilinear. Any approach is allowed.

Everything you write goes in one file, inside `ObstacleCourseNode.timer_callback()` between the two `###### INSERT CODE HERE ######` lines:

```
src/rb2301_fp/rb2301_fp/obstacle_course.py
```

What you have: `self.pose` (`[x, y, heading]`, metres and degrees, from Optitrack), `self.last_scan` (LiDAR ranges, set once the first scan arrives) and `self.move_2D(x, y, turn)` to drive.

This branch (`irl`) is for the real robot only: the Gazebo package, the simulation scripts and the simulation-only code paths are removed. The simulation is on `fresh`.

## Real robot

Same flow as the slides. `NN` is your robot's number in two digits (robot 7 is `07`, address `192.168.1.207`), `<group>` is your group number. You edit on your laptop and copy the package to the robot.

**1. Make your workspace on the robot** (ssh in as usual):

```bash
ssh bingda@192.168.1.2NN
mkdir -p ~/Downloads/rb2301_fp_<group>/src
```

**2. Copy the package and the script** from your laptop's own terminal (not the ssh one). Repeat after every edit:

```bash
rsync -auvx --delete ~/Documents/rb2301/src/rb2301_fp bingda@192.168.1.2NN:~/Downloads/rb2301_fp_<group>/src
rsync -auvx --delete ~/Documents/rb2301/fp.sh bingda@192.168.1.2NN:~/Downloads/rb2301_fp_<group>
```

**3. Build on the robot** (ssh terminal):

```bash
cd ~/Downloads/rb2301_fp_<group>
colb                    # alias for colcon build --symlink-install
```

**4. Run each of these in its own ssh terminal** (or use `tmux` on the robot to split one ssh into several terminals):

```bash
basecontrol             # alias for ros2 launch base_control_ros2 base_control.launch.py
rplidar                 # alias for ros2 launch rplidar_ros rplidar.launch.py
vrpn                    # alias for ros2 launch vrpn_mocap client.launch.yaml server:=192.168.1.199 port:=3883
cd ~/Downloads/rb2301_fp_<group> && ./fp.sh
```

`./robot-test.sh NN` does the copy, build and launch in one step (`./robot-test.sh -h` for options; `--map` shows the map live with `tools/view_map.py`).

Add `--calibrate` to the last command (`./fp.sh --calibrate`) to only print your pose and never drive. Stay out of the Optitrack area while the robot runs.

Notes:

- **Pose frame.** On the real robot `self.pose` is in the **CA2 maze frame**: metres from the corner of the CA2 maze, heading 0 along its +x axis and +90 along +y (counter-clockwise positive). The mapping from Optitrack to that frame is in `src/rb2301_fp/rb2301_fp/optitrack_variables.config`, already filled in. Don't edit it. If the calibration changes you will get a new file to drop in.
- **Course coordinates.** The node works relative to its start pose, so it doesn't need course coordinates. If you want them anyway, `self.to_course()` converts a pose from the CA2 maze frame to the course frame. It needs the position of the course start point in the maze frame (`[course]` in the config), which is not set yet, so for now it returns the pose unchanged.
- **Robot number.** The node reads the Optitrack topic of its own robot from `ROS_DOMAIN_ID` (already set on each robot) and prints it at start-up (`/vrpn_mocap/bingda_0NN/pose`).
- **Rigid bodies** are created in Motive with the robot facing +y of the CA2 maze. If you reset one, face the robot +y first.
- **Checking the pose.** Run the node with `--calibrate`, for example `./fp.sh --calibrate` on the robot. It prints the raw Optitrack pose and the maze-frame pose twice a second and never sends a drive command, so you can move the robot by hand and read off coordinates (for example the position of the gate). Start `rplidar` first, otherwise the node waits for the LiDAR.
