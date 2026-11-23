# Sokoban solver — design notes

## Structure

Levels are a 1-wide hallway (the player starts there) plus 1–4 rooms
off it. Each room is a 6×6 slot area with a single door; no two doors
share a column. A box pushed through a door into the hallway can never
be pushed back, so every box stays in its room and the level's
push-optimum P\* is the sum of the rooms' push-optima. The solver
exploits this directly.

### Decomposition

Doors are articulation points of the floor graph: removing a floor
square separates a room component from the hallway component (the one
containing the player start). A candidate room is a component with
3–4 boxes, the same number of goals, and at most 40 cells. Each box
and goal is assigned to its room by flood fill. Two rooms may share a
door cell (an articulation point can separate several components); the
stitcher handles that by walking door-to-door through the hallway.

### Per-room push-optimal A\*

State = player square + sorted box squares. Actions are pushes: the
player must be able to walk to the pushing square by BFS flood fill
treating boxes as obstacles. Cost is 1 per push; the heuristic is the
min-cost bipartite matching between boxes and goals over precomputed
BFS distances ignoring boxes. It is admissible (one push moves one box
at most one square closer along a shortest path), so the first solution
A\* pops is push-optimal for the room.

Pruning:

- **Dead squares** (pull analysis). A square is live when it is a goal,
  or a box on it can be pulled back from a live square. Pulling a box
  from a live square `t` to `s` needs the player to end at `p = 2s - t`
  (floor). When `t` is a goal the player is already at `s` after the
  final push, so no walk is needed; otherwise the player must either
  end the previous pull at `s` (straight continuation through `t`) or
  be able to walk to `s` from some neighbour of `t` without crossing
  `t`. A box on a dead square can never reach a goal, so such states
  are pruned. Two subtle bugs found during development: the classic
  "standing square connected to target" condition is wrong for
  corridor goals, and the goal-neighbour case must not require a walk
  at all.
- **Corner / wall-line traps** are subsumed by the dead-square table.
- **2×2 blocks** of boxes/walls with no goal inside are dead.
- **Frozen boxes**: a box that cannot move in either axis (blocked by
  wall/box on both sides) and is not on a goal deadlocks the state.

Terminal test: all boxes on goals, and (unless this is the last room)
the player can reach the door square by a no-push walk, so the player
can exit. The exit walk is appended to the move string.

### Stitching

Rooms are ordered nearest-door first (rooms whose door is a goal go
last, since a box ending on the door would trap the player). The player
walks between doors through the hallway by BFS with all boxes as
obstacles; inside a room the push sequence runs with the walks between
pushes. Move letters are lowercase for walks, uppercase for pushes
(the checker ignores case).

### Fallbacks

1. Per-room decomposition (above).
2. Full-level push-A\* with a 100,000-expansion cap.
3. Empty `solution.txt` (valid, 0 points).

The solver never crashes, never writes an illegal move, and never
exceeds the time budget: per-room cap 200,000 expansions, practice
set solves in under 1 s wall total.

## Determinism

Fixed iteration orders everywhere, no `Math.random`, no wall-clock or
date-based decisions. Two runs write byte-identical `solution.txt`
files (verified with `cmp`).

## Practice results (kit checker)

| level | pushes | P\* | points |
|-------|--------|-----|--------|
| i001  | 13     | 13  | 1000   |
| i002  | 16     | 16  | 1000   |
| i003  | 29     | 29  | 1000   |
| i004  | 47     | 47  | 1000   |
| i005  | 60     | 60  | 1000   |

Total practice score **1,000,000**. The house baseline plain-astar
(A\* over moves, no deadlock detection) is scored the same way for
comparison; see the submission report.

## Honest weaknesses

- **Decomposition assumes the hallway+rooms shape.** A scored level
  whose rooms are not separated by single articulation points, or a
  room with more than 4 boxes / a different box-goal count, falls back
  to full-level push-A\*, which is much weaker (100k cap, sum-of-minima
  heuristic) and may return empty on a hard 16-box level.
- **Dead-square table is an over-approximation.** It marks some live
  squares dead only if the pull conditions are mis-derived; the current
  conditions were validated on practice, but an unseen room geometry
  could expose a false dead square and cost the whole room (the
  fallback would then also fail, since it shares the same table).
- **Two rooms sharing one door** are solved independently; if solving
  the first room left a box where the second room's entry walk needs to
  pass, the stitch walk fails and the whole level falls back to
  full-level A\*. (Boxes end on goals inside their own rooms, so this
  should not happen on the stated shape, but it is not proven.)
- **The matching heuristic ignores player travel** between pushes and
  ignores box-box interactions, so A\* can still expand many states on
  a 4-box room with a deceptive layout; the 200k cap is the only guard.
  Practice rooms solve in ≤ ~200 ms each, but an unseen room could hit
  the cap and return empty for that level.
- **No transposition table beyond the best-g map**, and the player
  position is part of the state key, so equivalent states reached with
  the player on different squares are not merged. This costs expansions
  on larger rooms.
- **Scored levels may have different room sizes** (the practice set
  fixes 6×6 slots, 3–4 boxes/room). Larger rooms grow the state space
  combinatorially; the per-room cap trades completeness for the 60 s
  budget, and a timeout earns nothing on any instance — the solver
  prefers a fast empty answer over a slow one.
