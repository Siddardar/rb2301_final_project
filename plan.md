# Plan

We build the code one part at a time. Part 1 covers section 1, Part 2 section 2
and Part 3 section 3. Parts 1 and 2 describe the first, reactive version;
sections 1 and 2 now use the map and A* in `a-star.md`. Ideas not yet built are
in `improvements.md`.

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

Stopping holds the robot still, and `main()` sends a zero velocity on the way
out, because the base keeps the last command.

### Constants

What describes the robot:

| Constant | Value | Source |
|---|---|---|
| `ROBOT_LENGTH`, `ROBOT_WIDTH` | 0.289 m, 0.225 m | CA1 |
| `BODY_X` | 0.07 m | Body centre ahead of the pose. Measure on the real robot |
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
- **`BODY_X`** hasn't been measured on the real robot.

## Part 2: section 2, from the right turn through the door

### Goal

Carry on from the end of section 1 (through the right turn, heading roughly
−90°). Drive down the corridor around obstacles, find the door on the left, and
drive fully through it. Section 2 is done when the whole body, plus `MARGIN`, is
past the door's line (body centre `ROBOT_LENGTH / 2 + MARGIN` past it) and the
heading is within `HEADING_TOL` of the direction through the door.

Everything stays in the start frame, so "down the corridor" is −90° and "left"
is 0°. On the real course the door is roughly flush with the corridor's left
wall.

### Finding the door: two doorposts

The door is a **notch** in the left wall: a narrow stretch where the beams
suddenly reach much further, bounded on both sides by two posts on the wall. A
set-back door wall would make a **step** instead: the distance jumps up and
stays up. A notch jumps up and comes back down to the same wall, so the test
doesn't need to know any distances. It's the same idea as the two doorposts in
CA1.

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
- **Grazing angles:** far along a wall, neighbouring beams land further apart.
  The split jump is a robot length, so normal walls don't split into notches.
- **Tilt on the real course:** the posts must be within `MARGIN` of the end
  wall's x. On a flush real door, a corridor tilted a few degrees from the start
  heading could put the posts more than `MARGIN` off over 2 m.

## Part 3: section 3, the moving obstacles

It starts when section 2 is done (the whole body past the left wall's line).
It's reactive only: no map and no A*. Everything comes from the latest scan,
relative to the body, plus the left wall's line from section 1.

1. **Face 0°.** Turn in place, about the body centre, until the heading is
   within `WINDOW` (5°) of the start heading.
2. **Into the left pocket.** The robot's back must stay `MARGIN` clear of
   whatever is behind it: the door frame, and the wall with the door in it,
   which isn't necessarily straight. This is measured with the LiDAR, not the
   left wall's line.
   - **The strip:** points behind the back, from the body's right side up to
     `MARGIN` short of section 3's left wall. That's where the body will sweep
     as it slides. Section 3's own wall runs on behind the back, and would
     otherwise always read as touching. Before that wall is seen, the strip
     is just the body's width plus `MARGIN`, re-checked every tick.
   - **Back:** drive forward, holding 0°, until the strip is `MARGIN` clear.
   - **Slide:** then slide left until the left side is `WALL_GAP` (0.10 m) off
     section 3's left wall. If the strip gets closer than `MARGIN` (a slanted
     wall, a frame), it stops sliding and moves forward until clear. It slows
     near the wall, but never below half of `SPEED`, because the real robot
     didn't strafe at a quarter.
   - **Before the wall is seen** beside the body: slide at `SPEED`, holding 0°.
3. **Follow the wall** from here on, while waiting and dashing:
   - **The wall:** every scan, take the nearest surface on the left beside the
     body, from its back to its front, and fit a line to it. That gives the
     wall's angle and distance.
   - **Heading:** turn to stay parallel to the wall, so a slanted wall works too.
   - **Distance:** strafe gently (at most half of `SPEED`) to hold `WALL_GAP`.
     That cancels sideways drift, which turning can't.
   - **Why only points beside the body:** the cardboard sheets are always ahead
     of it or behind it, so they can't throw off the fit.
   - **No wall seen** in a scan: keep the last correction.
4. **Each moving obstacle, `MOVING_OBSTACLES` (2) times:**
   - **The path:** scan points ahead of the body's front, and within the body's
     half-width sideways.
   - **Wait** in place until something in the path is within the reach of the
     front: `OBSTACLE_GAP` (1.5) robot widths. That's the obstacle, seen for
     the first time, and how far ahead it is gives its line.
   - **Reach:** if nothing blocks the path for `GROW_AFTER` (2 s), the reach
     grows by `GROW_BY` (1.5), and again every 2 s after that. It stops
     `MARGIN` short of the most free space seen ahead during the wait, because
     the end wall is static and would otherwise block forever. It resets to
     `OBSTACLE_GAP` robot widths at the start of each wait.
   - **Move up** along the wall at `SPEED` until the front is `APPROACH_GAP`
     (0.15 m) from the obstacle's line. This is measured by the pose, because
     the obstacle may have left the path by then. It stops early if anything
     in the path is within `APPROACH_GAP`. If the obstacle is already within
     `APPROACH_GAP + MARGIN`, the robot doesn't move up.
     - **Why:** from where the robot first saw it, the dash was too long to
       finish before the obstacle came back.
   - **Wait close** until something in the path is within
     `APPROACH_GAP + 2·MARGIN` (0.25 m). That's the obstacle on its next pass.
     The extra `MARGIN` allows for the move-up stopping a little short. Then
     wait until the path is free again: it has just gone by. If it's still in
     the path when the robot arrives, the dash starts as soon as it clears.
   - **Dash** along the wall at `DASH_SPEED` (0.4 m/s) for the obstacle's
     distance plus the robot's length plus `MARGIN`: about 0.5 m from close
     up. The distance is the furthest it was from the front while blocking.
     The obstacles are sheets of cardboard, so their depth doesn't count. Then
     stop.
   - **Stop early** if something in the path is within `MARGIN`: the course's
     end wall, or the next obstacle coming in. In run 161016 the second dash
     needed 0.61 m, but the end wall was 0.57 m away. The robot pushed against
     the wall, never covered the distance, and so never finished.
5. **After the second dash,** the course is done: the robot holds still.

**History:**
- Version 1 slid left to the wall. In run 142709 it stopped only 4.8 cm from
  the wall, then drifted 9 cm left over 1.1 m while holding 0° by Optitrack.
  That's why the robot now follows the wall.
- Version 2 slid left until `WALL_GAP` from the wall. In run 145431 the slide
  stalled at 0.13 m, at a quarter of `SPEED`. That run had also started section
  3 too early, because of a wrong left wall.
- Version 3 used A* into the pocket. In run 161016 it worked in the end, but
  spent 14 s with no path while the pocket goal sat up against the door frame.

**Notes:**
- `move_2D` caps speed at `max_translate_velocity`, so the dash can't go faster
  than that.
