#!/usr/bin/env bash
# Set up and run the final project on a Bingda robot from your laptop.
set -euo pipefail

usage() {
    cat <<'HELP'
Usage: ./robot-test.sh NN [options]

  NN                  Two-digit robot number (01-54; 07 -> 192.168.1.207)
  --group GROUP       Workspace suffix (default: 11)
  --calibrate         Print poses without driving
  --map               Show the robot's map live on this computer (needs numpy and
                      matplotlib; see tools/view_map.py)
  --setup-only        Sync and build, then exit without launching anything
  --no-build          Sync only; reuse the existing build
  --terminal BACKEND  auto, gnome, or tmux (default: auto)
  -h, --help          Show this help

Examples:
  ./robot-test.sh 07
  ./robot-test.sh 07 --group 11
  ./robot-test.sh 07 --calibrate --terminal tmux
  ./robot-test.sh 07 --calibrate --map

Uses SSH keys when sshpass is absent; otherwise uses ROBOT_PASSWORD (default:
bingda). Install sshpass for automatic password login. Linux with a desktop uses
gnome-terminal; macOS or headless Linux uses tmux. Source is located relative to
this script, so it can be run from any directory.

Controls: ENTER = start, s = pause, r = resume, q / Ctrl+C = stop everything.

The node saves a map snapshot once a second on the robot, in the session's log
directory. With --map, they're also copied to maps/<session> here. The viewer
stays open after the session ends, so you can step back through the run.
HELP
}

die() { echo "Error: $*" >&2; exit 1; }
quote_command() { printf '%q ' "$@"; }

if [[ ${1:-} == --help || ${1:-} == -h ]]; then usage; exit 0; fi
[[ ${1:-} =~ ^[0-9]{2}$ ]] || { usage >&2; exit 1; }
robot_num=$1
robot_decimal=$((10#$robot_num))
# 192.168.1.2NN must be a valid host address, not .200 or .255.
(( robot_decimal >= 1 && robot_decimal <= 54 )) || die 'Robot number must be 01-54.'
shift
group=11
calibrate=0
setup_only=0
map=0
build=1
terminal=auto
while (( $# )); do
    case "$1" in
        --group|--terminal)
            (( $# >= 2 )) || die "Missing value for $1"
            case "$1" in
                --group) group=$2 ;;
                --terminal) terminal=$2 ;;
            esac
            shift 2 ;;
        --calibrate) calibrate=1; shift ;;
        --map) map=1; shift ;;
        --setup-only) setup_only=1; shift ;;
        --no-build) build=0; shift ;;
        -h|--help) usage; exit 0 ;;
        *) die "Unknown option: $1" ;;
    esac
done
[[ $group =~ ^[a-zA-Z0-9_-]+$ ]] || die 'Group must contain only letters, digits, underscores, or hyphens.'
case "$terminal" in auto|gnome|tmux) ;; *) die 'Terminal must be auto, gnome, or tmux.' ;; esac

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
local_src="$script_dir/src/rb2301_fp"
local_run="$script_dir/fp.sh"
[[ -f $local_src/rb2301_fp/obstacle_course.py ]] || die "Missing $local_src/rb2301_fp/obstacle_course.py"
[[ -x $local_run ]] || die "Missing or not executable: $local_run"
for tool in ssh rsync; do command -v "$tool" >/dev/null || die "Install $tool first."; done
if (( map && ! setup_only )); then
    python=python3
    [[ ! -x $script_dir/.venv/bin/python3 ]] || python=$script_dir/.venv/bin/python3
    "$python" -c 'import numpy, matplotlib' 2>/dev/null ||
        die "--map needs numpy and matplotlib. Install them with: python3 -m venv '$script_dir/.venv' && '$script_dir/.venv/bin/pip' install numpy matplotlib"
fi
if (( ! setup_only )); then
    if [[ $terminal == auto ]]; then
        if [[ -n ${DISPLAY:-}${WAYLAND_DISPLAY:-} ]] && command -v gnome-terminal >/dev/null; then
            terminal=gnome
        else
            terminal=tmux
        fi
    fi
    if [[ $terminal == gnome ]]; then
        command -v gnome-terminal >/dev/null || die 'Install gnome-terminal or use --terminal tmux.'
    else
        command -v tmux >/dev/null || die 'Install tmux (macOS: brew install tmux; Ubuntu: sudo apt install tmux), or use --terminal gnome.'
        [[ -z ${TMUX:-} ]] || die 'Run this script outside tmux so it can open its own five-window session.'
    fi
fi

target="bingda@192.168.1.2${robot_num}"
remote_ws="/home/bingda/Downloads/rb2301_fp_${group}"
session="$(date +%Y%m%d-%H%M%S)-$$"
state="$remote_ws/.robot-test/$session"
runner="$state/runner.sh"
# Keep the SSH socket path below the Unix socket length limit on macOS.
local_tmp=$(mktemp -d /tmp/fp-robot.XXXXXX)
chmod 700 "$local_tmp"
ssh_options=(-o StrictHostKeyChecking=accept-new -o ConnectTimeout=10
    -o ServerAliveInterval=5 -o ServerAliveCountMax=3
    -o ControlMaster=auto -o ControlPersist=600 -o "ControlPath=$local_tmp/ssh")
# Bash 3.2 (macOS) treats empty arrays as unset under `set -u`.
auth=(env)
if command -v sshpass >/dev/null; then
    export SSHPASS="${ROBOT_PASSWORD:-bingda}"
    auth=(sshpass -e)
fi
remote() { "${auth[@]}" ssh "${ssh_options[@]}" "$target" "$@"; }
remote_runner() {
    local command
    command=$(quote_command bash -ic 'exec bash "$@"' robot-test "$runner" "$remote_ws" "$state" "$robot_decimal" "$calibrate" "$@")
    remote "$command"
}
prepared=0
tmux_session="fp-${robot_num}-${session}"
cleanup() {
    local result=$?
    trap - EXIT INT TERM
    if (( prepared )); then remote_runner stop || true; fi
    if [[ $terminal == tmux ]] && (( ! setup_only )); then
        tmux kill-session -t "$tmux_session" 2>/dev/null || true
    fi
    ssh "${ssh_options[@]}" -O exit "$target" >/dev/null 2>&1 || true
    rm -rf -- "$local_tmp"
    exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "Robot $robot_num: $target | group $group"
echo "Workspace: $remote_ws"
# One authenticated master connection also serves rsync and the output windows.
"${auth[@]}" ssh "${ssh_options[@]}" -MNf "$target"

cat > "$local_tmp/runner.sh" <<'REMOTE'
#!/usr/bin/env bash
set -eo pipefail
ws=$1
state=$2
robot=$3
calibrate=$4
role=$5
lock=/tmp/rb2301-fp-robot-test-bingda.lock
zero='{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

terminate_group() {
    local pid=$1 signal tick
    kill -CONT -- "-$pid" 2>/dev/null || true
    for signal in INT TERM KILL; do
        kill -"$signal" -- "-$pid" 2>/dev/null || true
        for tick in {1..10}; do
            kill -0 -- "-$pid" 2>/dev/null || return 0
            sleep 0.1
        done
    done
}

case "$role" in
    claim)
        if ! mkdir "$lock" 2>/dev/null; then
            echo "Another robot-test session owns $lock. Quit that session first." >&2
            echo 'If it was interrupted, check for running ROS processes before removing the stale lock.' >&2
            exit 1
        fi
        printf '%s\n' "$state" > "$lock/owner"
        exit ;;
    stop)
        touch "$state/quit"
        # Give each service's trap time to stop its own process group.
        for tick in {1..80}; do
            active=0
            for name in base lidar pose planner; do
                [[ ! -f $state/$name.active ]] || active=1
            done
            (( active )) || break
            sleep 0.1
        done
        # Also recover a process group whose terminal died before its trap ran.
        for file in "$state"/*.pid; do
            [[ -f $file ]] || continue
            read -r pid < "$file"
            [[ $pid =~ ^[0-9]+$ ]] && terminate_group "$pid"
            rm -f "$file"
        done
        if [[ -f $lock/owner ]] && [[ $(cat "$lock/owner") == "$state" ]]; then
            rm -f "$lock/owner"
            rmdir "$lock"
        fi
        exit ;;
esac

if ! command -v ros2 >/dev/null; then source /opt/ros/foxy/setup.bash; fi
if [[ -z ${ROS_DOMAIN_ID:-} ]]; then export ROS_DOMAIN_ID="$robot"; fi
[[ $ROS_DOMAIN_ID =~ ^[0-9]+$ ]] && (( 10#$ROS_DOMAIN_ID == robot )) || {
    echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset} does not match robot $robot; check the robot's environment." >&2
    exit 1
}
cd "$ws"
case "$role" in
    build)
        colcon build --symlink-install --packages-select rb2301_fp
        exit ;;
    prepare)
        [[ -f install/setup.bash ]] || { echo 'No build found; run without --no-build.' >&2; exit 1; }
        source install/setup.bash
        for tool in setsid python3; do command -v "$tool" >/dev/null; done
        for package in base_control_ros2 rplidar_ros vrpn_mocap rb2301_fp; do ros2 pkg prefix "$package" >/dev/null; done
        exit ;;
esac
source install/setup.bash
if [[ $role == control ]]; then
    trap 'touch "$state/quit"' EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
    trap 'exit 129' HUP
    echo "ENTER: start | s: pause | r: resume | q / Ctrl+C: stop everything"
    echo "Waiting for a live /vrpn_mocap/bingda_$(printf '%03d' "$robot")/pose and /scan before starting."
    if (( ! calibrate )); then
        echo 'The robot starts exploring at once. Place it at the start, facing into the course, before pressing ENTER.'
    fi
    while [[ ! -f $state/quit ]]; do
        # Timeout lets a service failure stop the control terminal without input.
        if read -r -s -n 1 -t 1 command; then
            :
        else
            read_status=$?
            # Bash returns >128 on a timeout, and 1 on EOF (including Ctrl+D).
            (( read_status > 128 )) && continue
            break
        fi
        case "$command" in
            '') touch "$state/start"; echo 'Start requested; planner waits for the pose and scan check.' ;;
            s) touch "$state/pause"; echo 'Pause requested.' ;;
            r) rm -f "$state/pause"; echo 'Resume requested.' ;;
            q) break ;;
            *) echo 'Use ENTER, s, r, or q.' ;;
        esac
    done
    if [[ -f $state/failed ]]; then cat "$state/failed" >&2; exit 1; fi
    exit 0
fi

pid=
zero_pid=
ready_pid=
finish() {
    local result=$?
    trap - EXIT INT TERM
    if (( result > 0 && result < 128 )) && [[ ! -f $state/quit ]]; then
        printf '%s exited with status %s. See %s/%s.log\n' "$role" "$result" "$state" "$role" > "$state/failed"
    fi
    touch "$state/quit"
    [[ -z $ready_pid ]] || terminate_group "$ready_pid"
    [[ -z $pid ]] || terminate_group "$pid"
    [[ -z $zero_pid ]] || terminate_group "$zero_pid"
    rm -f "$state/$role.pid" "$state/$role.active"
    if [[ $role == planner ]]; then rm -f "$state/zero.pid" "$state/ready.pid"; fi
    exit "$result"
}
trap finish EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP
[[ ! -f $state/quit ]] || exit 0
touch "$state/$role.active"
exec > >(tee -a "$state/$role.log") 2>&1
case "$role" in
    base) args=(ros2 launch base_control_ros2 base_control.launch.py) ;;
    lidar) args=(ros2 launch rplidar_ros rplidar.launch.py) ;;
    pose) args=(ros2 launch vrpn_mocap client.launch.yaml server:=192.168.1.199 port:=3883) ;;
    planner)
        echo 'Waiting for an Optitrack pose and a LiDAR scan (45 second timeout)...'
        setsid python3 - "$robot" <<'PY' &
import sys, time
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
rclpy.init()
node = rclpy.create_node('robot_test_input_check')
poses, scans = [], []
topic = '/vrpn_mocap/bingda_%03d/pose' % int(sys.argv[1])
node.create_subscription(PoseStamped, topic, lambda msg: poses.append(msg), qos_profile_sensor_data)
node.create_subscription(LaserScan, 'scan', lambda msg: scans.append(msg), qos_profile_sensor_data)
deadline = time.monotonic() + 45
while not (poses and scans) and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
if not poses:
    sys.exit('No pose received on ' + topic + '. Check vrpn, Motive, and ROS_DOMAIN_ID.')
if not scans:
    sys.exit('No scan received on /scan. Check rplidar.')
print('Live pose received on ' + topic + ' and scan received on /scan', flush=True)
PY
        ready_pid=$!
        echo "$ready_pid" > "$state/ready.pid"
        while kill -0 "$ready_pid" 2>/dev/null; do
            [[ ! -f $state/quit ]] || exit 0
            sleep 0.1
        done
        wait "$ready_pid"
        ready_pid=
        rm -f "$state/ready.pid"
        touch "$state/ready"
        echo 'Pose and scan check passed. Press ENTER in the control terminal to start.'
        while [[ ! -f $state/start || -f $state/pause ]]; do
            [[ ! -f $state/quit ]] || exit 0
            sleep 0.1
        done
        [[ ! -f $state/quit ]] || exit 0
        args=(./fp.sh --snapshots "$state/maps")
        (( ! calibrate )) || args+=(--calibrate) ;;
    *) echo "Unknown service: $role" >&2; exit 1 ;;
esac
echo "Starting $role..."
setsid "${args[@]}" </dev/null &
pid=$!
echo "$pid" > "$state/$role.pid"
paused=0
while kill -0 "$pid" 2>/dev/null; do
    [[ ! -f $state/quit ]] || exit 0
    if [[ $role == planner ]]; then
        if [[ -f $state/pause ]] && (( ! paused )); then
            kill -STOP -- "-$pid"
            if (( ! calibrate )); then
                setsid ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "$zero" </dev/null > "$state/pause.log" 2>&1 &
                zero_pid=$!
                echo "$zero_pid" > "$state/zero.pid"
            fi
            paused=1
            echo 'Paused (drive mode also publishes zero velocity at 10 Hz).'
        elif [[ ! -f $state/pause ]] && (( paused )); then
            if [[ -n $zero_pid ]]; then
                terminate_group "$zero_pid"
                wait "$zero_pid" 2>/dev/null || true
                zero_pid=
                rm -f "$state/zero.pid"
            fi
            kill -CONT -- "-$pid"
            paused=0
            echo 'Resumed.'
        fi
        if [[ -n $zero_pid ]] && ! kill -0 "$zero_pid" 2>/dev/null; then
            echo 'Pause publisher failed; shutting down.' >&2
            exit 1
        fi
    fi
    sleep 0.1
done
wait "$pid"
REMOTE

echo 'Creating workspace and syncing the final project package...'
remote "$(quote_command mkdir -p "$remote_ws/src" "$state")"
rsync_shell=$(quote_command ssh "${ssh_options[@]}")
rsync -az -e "$rsync_shell" "$local_tmp/runner.sh" "$target:$runner"
# Claim the robot before syncing: replacing a live session's code is unsafe.
remote_runner claim
prepared=1
# --delete applies only to this group's package; --checksum catches edits
# even if a robot-side file happens to have a newer modification time.
rsync -azc --delete --exclude '__pycache__/' --exclude '*.pyc' \
    -e "$rsync_shell" "$local_src/" "$target:$remote_ws/src/rb2301_fp/"
rsync -azc -e "$rsync_shell" "$local_run" "$target:$remote_ws/fp.sh"
if (( build )); then
    echo 'Building on the robot...'
    remote_runner build
fi
if (( setup_only )); then echo 'Setup complete.'; exit 0; fi
remote_runner prepare
echo "Logs on robot: $state"
if (( map )); then
    local_maps="$script_dir/maps/$session"
    mkdir -p "$local_maps"
    # BatchMode: once the session ends and the shared connection closes, copying fails instead of asking for a password
    viewer_rsh=$(quote_command ssh "${ssh_options[@]}" -o BatchMode=yes)
    "$python" "$script_dir/tools/view_map.py" "$local_maps" --remote "$target:$state/maps" --rsh "$viewer_rsh" \
        > "$local_maps/viewer.log" 2>&1 &
    echo "Map viewer started. Snapshots are copied to $local_maps"
fi

pane_command() {
    local role=$1 command
    command=$(quote_command bash -ic 'exec bash "$@"' robot-test "$runner" "$remote_ws" "$state" "$robot_decimal" "$calibrate" "$role")
    quote_command ssh "${ssh_options[@]}" -tt "$target" "$command"
}
if [[ $terminal == gnome ]]; then
    for role in base lidar pose planner; do
        gnome-terminal --title="FP $robot_num: $role" -- bash -c "$(pane_command "$role")"
    done
    echo 'Use this terminal for controls; closing an output terminal stops the session.'
    # A PTY makes Ctrl+C and keyboard input work in the remote control loop.
    ssh "${ssh_options[@]}" -tt "$target" "$(quote_command bash -ic 'exec bash "$@"' robot-test "$runner" "$remote_ws" "$state" "$robot_decimal" "$calibrate" control)"
else
    tmux new-session -d -s "$tmux_session" -n base "$(pane_command base)"
    for role in lidar pose planner control; do
        tmux new-window -t "$tmux_session:" -n "$role" "$(pane_command "$role")"
    done
    tmux select-window -t "$tmux_session:control"
    echo 'tmux: Ctrl+B then n/p switches windows. q in control stops the session.'
    tmux attach-session -t "$tmux_session"
    # Detaching is also treated as shutdown, avoiding an unattended controller.
fi
