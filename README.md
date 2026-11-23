# Sokoban Solver — make.fun

[![make.fun](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/c/sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/irwan11795/makefun-sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/c/sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/irwan11795/makefun-sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/c/sokoban-solver-01-run-20261121)

A deterministic, push-optimal Sokoban solver for the
[sokoban-solver-01-run-20261121](https://make.fun/c/sokoban-solver-01-run-20261121)
competition. Node 22, standard library only, no dependencies, no WASM,
no native addons.

## How it works

Levels are a 1-wide hallway plus 1–4 rooms (6×6 slots, one door each).
Boxes pushed through a door can never return, so every box stays in its
room and the push-optimum of a level is the sum of its rooms' optima.
The solver therefore:

1. **Decomposes** the level at articulation points of the floor graph —
   each door separates one room component from the hallway component.
2. **Solves each room** with push-optimal A\*: state = player square +
   sorted box squares, one push per action, player reachability between
   pushes by BFS flood fill. The heuristic is the min-cost bipartite
   matching between boxes and goals over precomputed BFS distances
   (admissible: one push moves one box at most one square closer along a
   shortest path). Deadlock pruning covers dead squares (pull analysis),
   corner / wall-line traps, 2×2 blocks, and frozen boxes.
3. **Stitches** the rooms: the player walks between doors through the
   hallway by BFS (no pushes), and walks out of each room the same way.

Fallbacks, in order: per-room decomposition, then full-level push-A\*
with a strict expansion cap, then an empty (valid, 0-point) solution.
The solver never crashes, never writes an illegal move, and guards the
whole run well inside the 60 s budget.

## Practice results

| level | boxes | pushes | P\* | points |
|-------|-------|--------|-----|--------|
| i001  | 3     | 13     | 13  | 1000   |
| i002  | 4     | 16     | 16  | 1000   |
| i003  | 8     | 29     | 29  | 1000   |
| i004  | 12    | 47     | 47  | 1000   |
| i005  | 16    | 60     | 60  | 1000   |

Practice score: **1,000,000** (5,000 / 5,000 points), in under 1 s wall
for the whole set. See [NOTES.md](NOTES.md) for the design and the
honest weaknesses.
