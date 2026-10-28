#!/usr/bin/env python3
"""
Show the maps that obstacle_course.py saves: the snapshots it saves once a second with --snapshots, and
fp_map.npz, which it saves on exit.

No ROS needed, only numpy and matplotlib. To install them on your laptop:

    python3 -m venv .venv
    .venv/bin/pip install numpy matplotlib

./robot-test.sh NN --map starts this for you, copying the snapshots from the robot as they're saved. To look at
a run afterwards:

    .venv/bin/python3 tools/view_map.py maps/<session>                  # step through all of a run's snapshots
    .venv/bin/python3 tools/view_map.py maps/<session>/map_000420.npz   # start at one snapshot
    .venv/bin/python3 tools/view_map.py fp_map.npz                      # the map saved on exit

Left and right arrow keys step through the snapshots. On the newest one, the view follows new snapshots as they come.

Grey is the occupancy map: white free, black occupied, mid grey unknown. Orange is the sweep zone, where moving
obstacles have been, and red is where the last two scans saw one, as the crossing rule uses it.
"""
import argparse
import glob
import math
import os
import subprocess
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

ROBOT_LENGTH, ROBOT_WIDTH = 0.289, 0.225 # m, as in obstacle_course.py
SWEEP_COLOUR, MOVING_COLOUR = (1.0, 0.7, 0.3), (0.85, 0.1, 0.1)


class View:
    '''One figure, updated in place so zooming and panning survive each new snapshot'''
    def __init__(self):
        self.figure, self.ax = plt.subplots(figsize=(9, 9))
        ax = self.ax
        self.image = ax.imshow(np.zeros((1, 1, 3)), origin="lower", interpolation="nearest")
        self.trail, = ax.plot([], [], color="tab:purple", lw=1, alpha=0.6)
        self.start_line, = ax.plot([], [], color="tab:gray", lw=1, ls="--")
        self.path, = ax.plot([], [], color="tab:blue", lw=2)
        self.target, = ax.plot([], [], marker="*", ms=16, color="tab:green", ls="none")
        self.robot, = ax.plot([], [], color="tab:cyan", lw=2)
        self.heading, = ax.plot([], [], color="tab:cyan", lw=2)
        ax.set_aspect("equal")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.legend(handles=[Patch(color=SWEEP_COLOUR, label="sweep zone"), Patch(color=MOVING_COLOUR, label="moving now"),
                           Line2D([], [], color="tab:blue", lw=2, label="path"),
                           Line2D([], [], marker="*", ms=12, color="tab:green", ls="none", label="target"),
                           Line2D([], [], color="tab:cyan", lw=2, label="robot"),
                           Line2D([], [], color="tab:purple", lw=1, label="trail"),
                           Line2D([], [], color="tab:gray", lw=1, ls="--", label="start line")],
                  loc="upper left", fontsize="small", framealpha=0.8)

    def show(self, snap, title:str):
        occupancy = snap["occupancy"].astype(float)
        dynamic = snap["dynamic"]
        image = np.repeat((0.5 - occupancy / 8)[:, :, None], 3, axis=2) # log-odds -4 (free) to 4 (occupied): white to black
        image[dynamic >= 0] = SWEEP_COLOUR
        if "scan_count" in snap:
            image[dynamic >= int(snap["scan_count"]) - 1] = MOVING_COLOUR
        rows, cols = occupancy.shape
        (x0, y0), cell = snap["origin"], float(snap["cell_size"])
        self.image.set_data(image)
        self.image.set_extent((x0, x0 + cols * cell, y0, y0 + rows * cell)) # rescales unless you've zoomed

        trail = snap["trail"]
        self.trail.set_data(trail[:, 0], trail[:, 1])
        if "pose" in snap: # fp_map.npz from before snapshots were added has the map and trail only
            path, target = snap["path"], snap["target"]
            self.path.set_data(path[:, 0], path[:, 1])
            self.target.set_data(*(([target[0]], [target[1]]) if np.all(np.isfinite(target)) else ([], [])))
            x, y, heading = snap["pose"]
            c, s = math.cos(math.radians(heading)), math.sin(math.radians(heading))
            corners = np.array([(1, 1), (-1, 1), (-1, -1), (1, -1), (1, 1)]) * (ROBOT_LENGTH / 2, ROBOT_WIDTH / 2)
            self.robot.set_data(x + corners[:, 0] * c - corners[:, 1] * s, y + corners[:, 0] * s + corners[:, 1] * c)
            self.heading.set_data([x, x + ROBOT_LENGTH / 2 * c], [y, y + ROBOT_LENGTH / 2 * s])
            sx, sy, sh = snap["start"]
            c, s = math.cos(math.radians(sh)), math.sin(math.radians(sh))
            self.start_line.set_data([sx - s, sx + s], [sy + c, sy - c]) # 1 m either side, at right angles to the start heading
            title += f"  |  {snap['state']}  |  scan {int(snap['scan_count'])}"
        self.ax.set_title(title)
        self.figure.canvas.draw_idle()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("source", help="a directory of snapshots, or one .npz file")
    parser.add_argument("--remote", metavar="HOST:DIR", help="copy new snapshots from this directory every second")
    parser.add_argument("--rsh", default="ssh", help="the ssh command rsync uses for --remote")
    args = parser.parse_args()

    if os.path.isdir(args.source):
        directory, first = args.source, None
    else:
        directory = os.path.dirname(args.source) or "."
        first = os.path.join(directory, os.path.basename(args.source))

    def list_files():
        '''The snapshots in directory, oldest first. A file that isn't one of them, such as fp_map.npz, on its own'''
        found = sorted(glob.glob(os.path.join(directory, "map_*.npz")))
        return found if first is None or first in found else [first]

    files, index, shown = [], -1, None
    view = View()

    def on_key(event):
        nonlocal index
        if event.key == "left" and files:
            index = max(index - 1, 0)
        elif event.key == "right" and files:
            index = min(index + 1, len(files) - 1)
    view.figure.canvas.mpl_connect("key_press_event", on_key)

    while plt.fignum_exists(view.figure.number):
        if args.remote:
            # Fails until the node saves its first snapshot, and after the session ends: keep showing what's here
            subprocess.run(["rsync", "-az", "--exclude", "*.tmp", "-e", args.rsh, args.remote.rstrip("/") + "/", directory + "/"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        following = index == len(files) - 1 # on the newest snapshot, so keep showing the newest
        files = list_files()
        if index < 0 and first is not None:
            index = files.index(first)
        elif following:
            index = len(files) - 1
        if index >= 0 and files[index] != shown:
            shown = files[index]
            with np.load(shown) as snap:
                view.show(dict(snap), f"{os.path.basename(shown)} ({index + 1} of {len(files)})")
        elif index < 0:
            view.ax.set_title("Waiting for the first snapshot...")
        plt.pause(1.0)


if __name__ == "__main__":
    main()
