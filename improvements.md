# Potential improvements

Ideas and known issues discussed but not addressed yet. The reactive version is
described in `plan.md`, and the map and A* version in `a-star.md`.

## A* version

### Contact near obstacles on tight turns

**The issue.** Obstacles are inflated by half the robot's width plus `MARGIN`.
That keeps the sides 5 cm clear, but the corners stick out further (half the
diagonal, about 0.18 m from the body centre) and swing out on turns. The
cells right around the robot are also unblocked, so it can plan away from
something it starts next to. When a drawer sits close in front of the start
with only a tight gap beside it, the robot turns into the gap and a corner
touches the drawer. This happened in 3 of 12 special cases (0 cm, 0 cm, 0.8 cm),
and once the robot came within about 1 cm of a bin in a narrow crooked lane.

**Tried: a footprint guard** (removed). Before each move, check the body plus
`MARGIN` still fits 0.5 s later on the commanded arc; if not, go straight, then
turn on the spot, then wait. The contacts stopped, but all three cases got stuck
at the start within 7–11 s:

- A* plans a path that turns into the tight gap.
- The guard forbids every move from there: forward, straight, and turning back.

The planner (centre clearance only) and the guard (whole-body clearance)
disagree in gaps only a few centimetres wider than the robot.

**Possible fixes:**

- **Slide as the last resort.** When nothing fits, move toward the carrot
  without turning (sideways or diagonally; the base is holonomic), if the body
  still fits after the slide. This is the A* version of the reactive gap
  search's strafing. About 6–8 lines.
- **Inflate more.** Half the diagonal would protect the corners, but it closes
  gaps like the 40 cm lane beside a crooked wall.
- **Follow the path without turning near obstacles.** Translate along the path
  when an obstacle is within the turning circle, and only turn in open space.

### Door frame clearance since turning about the body centre

**The issue.** The robot now turns about its body centre: a sideways velocity of
`-turn × BODY_X` keeps the body centre where A* planned it. That fixed a robot
that spun about its rear axle into a drawer. But in the straight drive through
the door, the back half of the body is still inside the door frame, and it now
swings sideways as much as the front on every small heading correction. Door
frame clearance dropped from about 3–4 cm to 0–2 cm, touching once (−0.1 cm).

**Possible fix:** turn about the body centre only while following the A* path,
and turn about the rear axle in the straight drive through the door, as before.

### Grazing in gaps barely wider than the robot

**The issue.** In one layout, the route went between a drawer in the mouth of
the turn and a bin below it (a 44 cm gap), turning from −90° to −180° on the
way. The robot cleared the drawer by 1.3 cm in one run and grazed it (0 cm) in
the next. Inflating by half the robot's width doesn't protect the corners in a
gap that tight, even when turning about the body centre. Same family as the
contact issue above.

## Section 1

### Widest gap along the handover line

**Idea.** Put the waypoint in the middle of the widest free stretch of the
handover line (the right wall's line), instead of halfway between the corner
and the end wall. Recompute it every tick, so it moves as the robot turns and
sees more.

**How:**

1. Take the scan points in a band around the line: from the line down about one
   robot length (0.29 m) into the new corridor, plus `MARGIN` above it. Only use
   points from about a robot length before the detected corner up to the end
   wall.
2. Sort them by x. The gaps between neighbouring points are the free stretches.
   Along the wall they're only the beam spacing. In the opening they're real
   gaps.
3. The waypoint is the middle of the widest gap.
4. **Hysteresis:** keep the current gap while it's still wider than the robot
   plus margins, and only switch when it isn't. That stops the waypoint flipping
   between two similar gaps, without a new constant.

**Why:**

- **Aims at the open side early.** With an obstacle in the mouth, the robot
  heads for the open side before it gets blocked, so the gap search and back-off
  are rarely needed at the mouth. They stay as the last resort.
- **Fixes late corners.** The wall's own points show where it really ends. In
  one run, a drawer in the mouth made the corner register 25 cm late.

**Watch out for:**

- **Band thickness.** Too thin ignores obstacles just inside the new corridor.
  Too thick starts counting section 2's obstacles.
- **Hidden far edges.** From upstream, the robot only sees the near side of an
  obstacle on the line, so the gap beyond it looks wider than it is, by up to
  the obstacle's radius. It corrects as the robot turns.
- **Size:** about 15–20 lines in `waypoint()`.

### Sliding the waypoint by lane depth (not recommended)

The same idea as above, but the waypoint slides toward the x where a robot-wide
lane is clear the furthest down the new corridor.

Without a map, an unseen area has no scan points, so it looks clear. The
waypoint would be drawn to areas hidden behind obstacles. Avoiding that needs a
visibility check (did a beam actually pass each spot?), which would be the most
complicated code so far. The widest-gap version above avoids this.

### Smaller items

- **Overshoot at the finish.** "Done" accepts a heading within 45° of −90°, so
  one run finished at −123°. A tighter check, or a short "straighten up" step,
  would tidy it.
- **Noisy short searches.** When the footprint check blocks both turning and
  going straight, a gap search starts and usually ends a tick later: 19 times in
  one run. It's harmless, but it clutters the log and the motion.
- **Back-off scope.** The back-off is allowed once per gap search, not once per
  run.
- **`BODY_X` on the real robot.** Only known for the simulation (0.07 m). It
  depends on where the Optitrack rigid body's centre is.
