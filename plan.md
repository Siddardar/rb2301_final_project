# Plan

We build the code one part at a time. Each part is tested before the next one
starts. Part 1 covers section 1, Part 2 section 2. Ideas not yet built are in
`improvements.md`. The course map at the end is for
our understanding only; nothing from it goes into the code.

## Part 1: section 1, from the start through the right turn

### Goal

Drive down the first corridor around any obstacles, find the right turn, and
curve into it. Section 1 is done once the robot is through the opening and
heading roughly down the new corridor. That's a state, not an exact point or
heading. Section 2 will start from there with the same steering and a target of
−90°. For now, the robot stops and exits. If the robot is stuck or has no way forward, it stops and exits with the
reason.

The code is in `obstacle_course.py`: the "Part 1" block of constants, the Part 1
methods of `ObstacleCourseNode`, and the `INSERT CODE HERE` section of
`timer_callback()`. There's no map and no path planner. Each tick works from the
latest scan and the pose.

### The start frame

Everything is relative to the pose on the first tick:

- The origin is the start position.
- +x is the direction the robot faces at the start (0°).
- +y is to the left, and angles are positive counter-clockwise.

Steering uses the **body centre**, `BODY_X` ahead of the pose. Each scan is
converted to points in this frame, from the LiDAR's position (`LIDAR_X` ahead of
the pose, mounted backwards). Beams that are `inf`, `nan`, or at or below
`range_min` are dropped. There's no range limit.

### Finding the right turn

The turn is found from two edges: its near corner, where the right wall ends,
and its far edge, the end wall of the corridor.

1. **Reference distance (first scan).** The furthest any point reaches to the
   right on the first scan, `reach`. That's normally the right wall, about
   0.6 m. If the opening is already visible from the start, it can be up to
   about 1 m.
2. **End wall.** Every scan, the beams within ±`WINDOW` (5°) of straight ahead
   give the furthest x seen straight ahead so far, `end_x`. Obstacles are
   always nearer than the end wall, so once any beam has got past them, this is
   the end wall, and it never goes back down.
3. **Near corner (the jump).** A beam within ±5° of straight right that ends
   more than `JUMP` (2) times `reach` to the right went through the opening.
   Nothing else on the right is that far away: tilts and obstacles change
   distances gradually, not by metres. The first such beam's x is `corner_x`.
4. **Right wall's line.** The median y of every hit by those straight-right
   beams while driving down the corridor, fixed when the corner is found. Nearly
   all of them hit the wall. A few hit an obstacle beside the robot, or peek
   into the opening just before the corner and hit something in the new
   corridor. There are too few of those to move the median. (Taking the
   furthest-right hit instead put the line up to 0.9 m too far right in
   testing.)
5. **Waypoint.** Halfway between `corner_x` and `end_x`, on the right wall's
   line, so the handover point doesn't depend on what the first scan happened
   to see. It's recomputed every tick, in case `end_x` grows.

Steps 2 and 3 stop once the robot has reached the turn.

This works when the robot is placed a few degrees off the corridor, when the
right wall is tilted, and with an obstacle beside the robot at the start (see
test results). It relies on one piece of section knowledge: in section 1, the
opening runs up to the corridor's end wall.

### Where to steer

Each tick there's a **target direction**:

| Stage | Target |
|---|---|
| Turn not found yet | 0°, straight down the corridor |
| Turn found | toward the waypoint |
| Within `LOOKAHEAD` (0.4 m) of the waypoint | −90°, down the new corridor. This stage is kept once reached |

**Section 1 is done** when the body centre is past the right wall's line and
the heading is within `HEADING_TOL` (45°) of −90°.

### Clear-heading steering

1. **Candidates:** headings every `HEADING_STEP` (5°) within ±90° of the
   current heading.
2. **Clear:** a heading is clear if no scan point lies in its strip: from the
   body centre to `STRIP` (0.3 m) past the front, and as wide as the body plus
   `MARGIN` (5 cm) on each side.
3. **Choice:** the clear heading closest to the target. Near-ties go to the side
   chosen last tick, so the robot doesn't flip between two equal gaps.
4. **Driving:** pure pursuit toward the chosen heading.
   ```python
   turn = 2 * SPEED * sin(alpha) / LOOKAHEAD   # alpha: chosen heading minus current heading
   speed = SPEED * cos(alpha)                  # slower on sharp turns
   ```

A longer strip blocks every heading angled toward a near wall in a 1.2 m
corridor. 0.6 m failed in testing; 0.3 m works.

**Room to turn.** The strip only covers what's ahead, but the robot turns about
its rear axle, so its front corner swings sideways. Before each move, the robot
checks that its body, plus `MARGIN` all round, still fits (no scan point inside
it) where it will be `AHEAD_TIME` (0.5 s) later on the commanded arc. If it
won't, the robot drives straight this tick. If straight won't fit either, it
starts a gap search toward the chosen heading.

Before this check, the side of the body passed obstacles with as little as
2 mm in testing. A first version used a full turning circle around the rear
axle instead. That needed about 58 cm of clear width, so the robot got stuck in
a 50 cm gap it could easily turn in.

### Gap search (last resort)

If no heading within ±`FORWARD` (45°) of the current one is clear, the robot
searches for a gap. This covers an obstacle close in front at the start, and an
obstacle sitting in the opening's mouth.

1. **Wanted heading:** −90° if the corner has been found and the robot is past
   it, otherwise 0°.
2. **Turn on the spot** to the wanted heading (within `HEADING_STEP`). Before
   each step, check the body still fits at the next heading. If it doesn't,
   slide straight away from the nearest scan point at `STRAFE_SPEED` instead.
4. **Gap found?** If the strip straight ahead is clear, normal steering takes
   over.
5. **Otherwise strafe** at `STRAFE_SPEED` (0.1 m/s, slow enough to stop inside a
   gap only a little wider than the robot):
   - First to the side with more room.
   - If that side is blocked, switch once.
   - **If both sides are blocked, back off once:** slide `MARGIN` straight away
     from the nearest point, then retry the strafe from the start. Until the
     gap is found, the strip ahead is longer by `MARGIN`. Otherwise whatever
     blocked it would be 5 cm further away after backing off, and the robot
     would drive straight back into it.
   - If both sides are still blocked, stop with
     `No clear heading: no gap either side`.

This reuses the strafing move, so it isn't a new mechanism. A doorpost-style
waypoint (the widest gap along the right wall's line) was considered for
obstacles in the mouth and dropped in favour of this.

### Stopping when something's wrong

- **Stuck:** the body centre moved less than `STUCK_DIST` (5 cm) in
  `STUCK_TIME` (3 s).
- **No clear heading:** the gap search found no gap on either side.

Stopping raises `SystemExit`, and `main()` sends a zero velocity on the way
out, because Gazebo keeps the last command.

### Watching it

In the simulation, the node draws the waypoint as a green sphere in the
Gazebo window, using Gazebo's `/marker` service in the background. That lets
you see where the robot is aiming. It does nothing on the real robot.

### Constants

What describes the robot:

| Constant | Value | Source |
|---|---|---|
| `ROBOT_LENGTH`, `ROBOT_WIDTH` | 0.289 m, 0.225 m | CA1 |
| `BODY_X` | 0.07 m in the simulation | Body centre ahead of the pose. Measure on the real robot |
| `LIDAR_X`, `LIDAR_YAW` | 0.114 m, π | URDF `laser_joint` |

What we choose:

| Constant | Value | What it does |
|---|---|---|
| `SPEED` | 0.2 m/s | Forward speed |
| `STRAFE_SPEED` | 0.1 m/s | Speed when sliding during the gap search |
| `LOOKAHEAD` | 0.4 m | Pure pursuit, and how close to the waypoint before turning to −90° |
| `MARGIN` | 0.05 m | Clearance kept each side of the body |
| `STRIP` | 0.3 m | How far past the front a heading must be clear |
| `HEADING_STEP` | 5° | Spacing of candidate headings |
| `FORWARD` | 45° | Search for a gap if nothing this close to the current heading is clear |
| `WINDOW` | 5° | Beams this close to straight ahead / straight right find the turn |
| `JUMP` | 2 | How many times the first scan's reach counts as "through the turn" |
| `HEADING_TOL` | 45° | Through the turn and this close to −90° counts as heading down the new corridor |
| `STUCK_TIME`, `STUCK_DIST` | 3 s, 0.05 m | When to give up |
| `AHEAD_TIME` | 0.5 s | How far ahead the body must still fit before turning |

### Testing

Each run uses a random layout, rerolled if an obstacle spawns on the robot, and
is capped at 2 minutes. The special cases:

- **Crooked:** the robot spawned at +4° yaw.
- **Tilted:** the right wall rotated 2.5°, far end further right.
- **Drawer right:** a drawer beside the robot at the start, face parallel to the
  wall.
- **Drawer ahead:** a drawer across the robot's full width, face 16 cm ahead.
- **Bin in the mouth:** a bin on the right wall's line 20 cm past the corner,
  leaving a 19 cm gap (too narrow) on the near side and 55 cm on the far side.
  The deeper obstacles are removed.

Results with the current code (footprint check and back-off included):

| Test | Result | Closest to an obstacle |
|---|---|---|
| Tilted (stuck with the turning-circle version) | Done, 27 s | 5.0 cm |
| Drawer in the mouth (2 mm before the footprint check) | Done, 17 s | 8.1 cm |
| Drawer ahead | Done, 26 s, gap search facing 0° | 6.3 cm |
| Crooked | Done, 23 s | 4.8 cm |
| Random A (coke can and drawer below the mouth) | Done, 31 s: both strafe sides blocked, backed off once, then found the gap | 5.3 cm |
| Random B | Done, 22 s, but finished heading −123° (see Known issues) | 3.7 cm (bin, about 5.7 cm at LiDAR height) |

### Known issues

- **Overshoot at the finish.** "Done" only needs the heading within 45° of
  −90°, so a robot that turned past −90° can finish at, say, −123°.
- **Many short searches.** When the footprint check blocks both turning and
  going straight, a gap search starts and usually ends a tick later. In one run
  that happened 19 times. It's harmless, but noisy in the log.
- **Same layout, different side.** Which side of a mouth obstacle the robot
  reaches first depends on small timing differences, so the same layout can
  pass one way one run and the other way the next.
- **Layouts with no route** (two obstacles staggered across the mouth) still
  stop with `No clear heading`.
- **`BODY_X`** is only known for the simulation.

## Part 2: section 2, from the right turn through the door

### Goal

Carry on from the end of section 1 (through the right turn, heading roughly
−90°). Drive down the corridor around obstacles, find the door on the left, and
drive fully through it. Section 2 is done when the whole body, plus `MARGIN`, is
past the door's line (body centre `ROBOT_LENGTH / 2 + MARGIN` past it) and the
heading is within `HEADING_TOL` of the direction through the door. Then stop and
exit.

Everything stays in the start frame, so "down the corridor" is −90° and "left"
is 0°. The code has to work both in the simulation, where the door is in a wall
set back about 0.6 m from the corridor's left wall, and on the real course,
where the door is roughly flush with it.

### Finding the door: two doorposts

The door is a **notch** in the left wall: a narrow stretch where the beams
suddenly reach much further, bounded on both sides by two posts on the wall. The
simulation's set-back door wall makes a **step** where the corridor's wall ends:
the distance jumps up and stays up. A notch jumps up and comes back down to the
same wall. So the notch test works in both places without knowing any distances.
It's the same idea as the two doorposts in CA1.

Every scan, while the door isn't locked (see below):

1. **The corridor's left wall, `left_x`:** section 1's end wall (`end_x`).
   After a right turn at the end of a corridor, the end wall carries on as the
   new corridor's left wall. It's measured as the furthest point straight ahead,
   so obstacles can't pull it nearer. (A median of straight-left hits in
   section 2 was tried first. A bin beside the robot at the start of section 2
   pulled it in by up to 0.6 m, and obstacle edges then passed as door posts.)
2. **Sweep the left side:** take the beams pointing anywhere to the left (start-
   frame direction between −90° and 90°), in angle order. Split them into
   groups wherever neighbouring beams' end x differs by more than
   `ROBOT_LENGTH`. That's a jump big enough for the body to fit into.
3. **Notch:** a run of groups whose beams all reach more than `ROBOT_LENGTH`
   further left than the beams on either side of the run. Those two neighbouring
   beams are the **posts**. A notch only counts as a door if:
   - **Both posts are on the left wall:** x at least `left_x − MARGIN`. That
     rules out gaps between obstacles, or between an obstacle and the wall.
   - **Both posts are on the same wall:** their x within `ROBOT_LENGTH` of each
     other. From upstream, the corridor's end can hide the door's near post.
     Without this rule, the far post paired with the corridor's corner, and the
     door came out 0.3 m short at 23°.
   - **The posts are at least `ROBOT_WIDTH + 2 × MARGIN` apart.** A gap between
     wall panels isn't a door. With walls crooked by ±8°, a seam opened at a
     corner and passed every other rule.
4. **Door:** the centre is the posts' midpoint. The direction through it is
   perpendicular to the line between the posts, pointing away from the robot.
   If more than one notch is found, take the nearest.

Moving boxes behind the door only change the ranges between the posts. They
stay further than the posts, so the notch survives. The bend is a rising edge
with no matching fall back to the same wall, so it's never a notch.

### Driving through

| Stage | Target |
|---|---|
| Door not found yet | −90°, down the corridor |
| Door found | pure pursuit along the door's centre line: aim at the point `LOOKAHEAD` past the robot's projection onto the line, in the direction through the door |

Following the centre line brings the robot onto the line some way before the
door, so it goes through straight. The door has only 3.75 cm to spare each side
with the margins.

**Locking the door:** once the robot is within `LOOKAHEAD` of the door's line,
the door estimate is fixed. From that close, the posts are beside the robot and
can drop out of view.

The clear-heading steering, footprint check and gap search are the same as in
section 1. The gap search's wanted heading in section 2 is the direction through
the door once the door is locked, and −90° before that. Facing the door from
further up the corridor would point the robot at the corridor wall; one run did
that and strafed up and down the corridor until the time limit.

**Gap search: when to turn.** The gap search only turns to its wanted heading
if the robot is more than two heading steps (10°) off. Otherwise it strafes at
its current heading. A lane beside a crooked wall runs a few degrees off −90°,
and forcing the turn in a lane barely wider than the robot made it bounce
between the wall and an obstacle. A 45° limit was tried first, but 18° off down
a corridor points the strip into a wall.

**If the door is never seen** (for example, an obstacle hides it as the robot
passes), the robot reaches the bottom wall and the gap search facing −90° can't
find it. It stops with `No clear heading`. That's a known gap.

### Constants

None new. `ROBOT_LENGTH` is the jump that counts as a notch, `MARGIN` is the
tolerance for posts being on the wall, and `LOOKAHEAD` is the centre-line
pursuit distance and the lock distance.

### Things to watch

- **The last obstacle sits near the door.** It's placed around the room's
  corner, so it can block the approach or hide a post.
- **Moving boxes just behind the door:** the first box's track is only about
  0.58 m past the door wall in the simulation. The robot stops as soon as it's
  through.
- **Grazing angles:** far along a wall, neighbouring beams land further apart.
  The split jump is a robot length, so normal walls don't split into notches.
- **Tilt on the real course:** the posts must be within `MARGIN` of the end
  wall's x. On a flush real door, a corridor tilted a few degrees from the start
  heading could put the posts more than `MARGIN` off over 2 m. The simulation
  doesn't test this, because its door wall is set back 0.6 m.

### Test results

Each run is capped at 2 minutes. Crooked means every wall panel was rotated by
its own random angle about its centre.

| Test | Result |
|---|---|
| Random A, B | Done, about 42 s each. Door within 3 cm of the real one |
| Random C (drawer blocking the corridor) | Done, 63 s |
| Random D, E | No route with the margins (gaps of 0.308–0.322 m against 0.325 m needed). These failed with false doors before `left_x` came from the end wall. Not rerun since |
| Crooked ±3° | Done, 61 s |
| Crooked ±5° | Done, 56 s |
| Crooked ±8° | Done, 78 s, after the post-width rule and the gap-search turning rule |

## The course map (reference only)

All numbers are in the simulation's world frame, in metres. The robot spawns at
`(0, 0)` facing +x. The map below shows the **nominal** layout: no random
offsets, and the door at its default position from
`obstacle_course_world_fp_original.sdf`. Each launch moves parts of it (see
[What changes each launch](#what-changes-each-launch)).

Scale: one character is 0.1 m across, and one line is 0.2 m down.

```
   y (m)
  +0.6  +-----------+-----------+-----------+
        |                                   |
        |                                   |
   0.0  |     R>    s0                      |
        |                                   |
        |                                   |
  -0.6  +-----------+-----------+     s1    |
                                |           |
                                |           |
                                |           |
                                |     s2    |
                                |           |
  -1.8                          |           +-----+-----+-----------+
                                |                 #     :     :     |
                                |            door>      :     :     |
                                |     s3                :     :     |
                                |                 #     M1    M2    |
                                |                 #     :     :     |
                                |                 #     :     :     |
                                |                 #     :     :     |
  -3.4                          +-----------+-----+-----+-----------+
      -0.6    0    0.6         1.8         3.0   3.6   4.2   4.8   5.4   x (m)
```

| Symbol | Meaning |
|---|---|
| `-` `\|` `+` | Fixed walls. Each wall is a 1.2 m panel, 1 cm thick and 0.5 m tall. |
| `#` | The door wall. The gap in it is the door, 0.4 m wide. |
| `R>` | Robot start at `(0, 0)`, facing +x. |
| `s0` to `s3` | Static obstacles, at their nominal positions. |
| `M1`, `M2` | Moving obstacles. The `:` marks show the line each one slides along. |

## Walls

Each wall's model origin is offset from the panel. A wall at pose
`(px, py, yaw)` gives these segments:

- yaw 0: from `(px - 0.6, py + 0.6)` to `(px + 0.6, py + 0.6)`
- yaw 90°: from `(px - 0.6, py - 0.6)` to `(px - 0.6, py + 0.6)`

The nominal segments:

| Wall(s) | Segment | What it is |
|---|---|---|
| 3 | x = −0.6, y −0.6 to 0.6 | Back wall behind the start |
| 1, 4, 6 | y = 0.6, x −0.6 to 3.0 | Corridor 1, left side |
| 2, 5 | y = −0.6, x −0.6 to 1.8 | Corridor 1, right side |
| 8 | x = 3.0, y −0.6 to 0.6 | End of corridor 1 |
| 7, 9, 11 | x = 1.8, y −3.4 to −0.6 | Corridor 2, west side |
| 10 | x = 3.0, y −1.8 to −0.6 | Corridor 2, east side |
| 13 | y = −3.4, x 1.8 to 3.0 | Bottom of corridor 2 |
| 12, 15 | y = −1.8, x 3.0 to 5.4 | Corridor 3, north side |
| 14, 16 | y = −3.4, x 3.0 to 5.4 | Corridor 3, south side |
| 17, 18 | x = 5.4, y −3.4 to −1.8 | End wall |
| configurable_left | x = 3.6, y −2.057 to −0.857 | Door wall above the gap |
| configurable_right | x = 3.6, y −3.657 to −2.457 | Door wall below the gap |

The door walls are longer than the corridor is wide, so they stick out past
the corridor walls. The nominal door gap is y −2.457 to −2.057, centred on
−2.257.

## Key distances

| What | Distance |
|---|---|
| Corridor 1 | 1.2 m wide (y −0.6 to 0.6), from the back wall to x = 3.0 |
| Opening from corridor 1 into corridor 2 | x 1.8 to 3.0, on the right-hand side |
| Corridor 2 | 1.2 m wide (x 1.8 to 3.0), from y −0.6 down to −3.4 |
| Corridor 3 | 1.6 m wide (y −3.4 to −1.8), from x 3.0 to 5.4 |
| Door wall to `M1` line, `M1` to `M2`, `M2` to end wall | 0.6 m each, about 0.57 m of free space after the box and wall thickness |
| `M1` and `M2` track (box centre) | y −2.0 to −3.2. The box covers y −1.85 to −3.35. |
| Robot body | 0.289 m long, 0.225 m wide |

## What changes each launch

`obstacle_modifier.py` regenerates `obstacle_course_world_fp.sdf` from the
original on every `./gz_fp.sh`. Each item has a `randomise_...` switch at the
top of that file.

| Item | Range | Effect |
|---|---|---|
| `x_offset` (zig-zag size) | −0.8 to 0 | Every wall whose model x is above 1.0, and the door, `M1`, `M2`, `s1`–`s3`, shift by this in x. Corridor 1 gets shorter: its end wall moves to x 2.2–3.0. Everything after it moves with it. |
| `y_offset` (zig-zag size) | −0.6 to 0 | Walls whose model y is below −2.6 (11, 13, 14, 16, 18) shift down by this. Corridor 2 gets longer, and corridor 3 gets wider (1.6 to 2.2 m). The north wall of corridor 3 stays at y = −1.8. The door, `M1`, `M2`, and `s1`–`s3` shift down too. |
| Door position | Centre y from −2.7 to −2.2, plus `y_offset` | The door width stays at 0.4 m. Door x is 3.6 + `x_offset`. |
| Obstacle speed | Half-period 2 s to 6 s, each box separately | 1.2 m each way, so 0.2 to 0.6 m/s. The comment in the code says 2 to 5 s, but the code gives 2 to 6 s. |
| Static obstacle positions | ±0.4 m in x, ±0.3 m in y | `s0` nominal `(0.6, 0)`. `s1`–`s3` nominal `(2.4, −0.6)`, `(2.4, −1.4)`, `(2.4, −2.4)`, plus both offsets. |
| Static obstacle type | One of three, chosen at random for each | Coke can (0.06 m diameter, 0.145 m tall), drawer (0.41 × 0.48 m), or litter bin (about 0.45 m across). |

### Edge cases worth knowing

- `s0` can land as close as x = 0.2, right in front of the robot at the start.
- A litter bin offset by 0.4 m in corridor 2 almost touches a wall, but the
  other side always has about 0.75 m free.
- The box tracks shift down with `y_offset`, but the north wall of corridor 3
  doesn't. That leaves a strip along y = −1.8 that the boxes never enter, of
  width 0.05 − `y_offset` (up to 0.65 m). Once `y_offset` is below about
  −0.18, the strip is wider than the robot. This is a simulation artefact, so
  don't rely on it for the real course.
- With the door at its lowest and `y_offset` near −0.6, a slit of up to 0.1 m opens
  between the top of the upper door wall and the corridor 3 north wall. The
  robot can't fit through it, but the LiDAR sees through it.
