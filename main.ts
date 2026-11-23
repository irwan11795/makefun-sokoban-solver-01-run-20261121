// Sokoban solver: per-room push-optimal A* over a door decomposition.
//
// Levels are a 1-wide hallway (the player starts there) with 1-4 rooms off
// it, each opening through one door; a box pushed into the hallway can never
// come back, so every box stays in its room and a level's push-optimal count
// P* is the sum of its rooms' optima. We find the doors as articulation
// points of the floor graph, solve each room with a push-optimal A* (cost 1
// per push, min-cost matching heuristic over BFS distances, dead-square /
// frozen-box / 2x2 pruning), and stitch the rooms with no-push hallway
// walks. Fallbacks, in order: full-level push-A*, then an empty solution.
// Never crashes, never writes an illegal move, never reads the clock:
// budgets are expansion caps only, so two runs write identical bytes.

import { mkdirSync, readdirSync, readFileSync, statSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

const DIRS: [number, number, string][] = [
  [0, -1, 'u'],
  [0, 1, 'd'],
  [-1, 0, 'l'],
  [1, 0, 'r'],
];

// Work budgets (expansions, never wall time: determinism is by work, not clock).
const ROOM_EXPANSIONS = 200_000;
const FULL_EXPANSIONS = 100_000;

interface Level {
  w: number;
  h: number;
  wall: Uint8Array;
  goal: Uint8Array;
  boxes: number[];
  player: number;
}

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const w = Math.max(...rows.map((r) => r.length));
  const h = rows.length;
  const wall = new Uint8Array(w * h).fill(1);
  const goal = new Uint8Array(w * h);
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goal[i] = 1;
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goal, boxes, player };
}

// Floor neighbours of cell i (bounds-checked, walls excluded).
function floorNb(lv: Level, i: number): number[] {
  const { w, h, wall } = lv;
  const x = i % w;
  const y = (i / w) | 0;
  const out: number[] = [];
  if (x > 0 && !wall[i - 1]) out.push(i - 1);
  if (x + 1 < w && !wall[i + 1]) out.push(i + 1);
  if (y > 0 && !wall[i - w]) out.push(i - w);
  if (y + 1 < h && !wall[i + w]) out.push(i + w);
  return out;
}

// Neighbours restricted to a floor mask (mask cells are usable floor).
function maskNb(lv: Level, mask: Uint8Array, i: number): number[] {
  const { w, h } = lv;
  const x = i % w;
  const y = (i / w) | 0;
  const out: number[] = [];
  if (x > 0 && mask[i - 1]) out.push(i - 1);
  if (x + 1 < w && mask[i + 1]) out.push(i + 1);
  if (y > 0 && mask[i - w]) out.push(i - w);
  if (y + 1 < h && mask[i + w]) out.push(i + w);
  return out;
}

// Connected components of the floor with `blocked` cells removed.
function components(lv: Level, blocked: Uint8Array): number[][] {
  const { w, h, wall } = lv;
  const seen = new Uint8Array(w * h);
  const comps: number[][] = [];
  for (let i = 0; i < w * h; i++) {
    if (wall[i] || blocked[i] || seen[i]) continue;
    const comp: number[] = [];
    const stack = [i];
    seen[i] = 1;
    while (stack.length > 0) {
      const u = stack.pop()!;
      comp.push(u);
      for (const v of floorNb(lv, u)) {
        if (!blocked[v] && !seen[v]) {
          seen[v] = 1;
          stack.push(v);
        }
      }
    }
    comps.push(comp);
  }
  return comps;
}

// BFS distances over a floor mask from `from` (-1 = unreachable).
function bfsDist(lv: Level, mask: Uint8Array, from: number): Int32Array {
  const dist = new Int32Array(lv.w * lv.h).fill(-1);
  const q = [from];
  dist[from] = 0;
  for (let h = 0; h < q.length; h++) {
    const u = q[h]!;
    for (const v of maskNb(lv, mask, u)) {
      if (dist[v] === -1) {
        dist[v] = dist[u]! + 1;
        q.push(v);
      }
    }
  }
  return dist;
}

// BFS walk path (move letters, lowercase) from `from` to `to` over a floor
// mask, treating `boxes` as obstacles; null when unreachable.
function walkPath(
  lv: Level,
  mask: Uint8Array,
  boxes: Set<number>,
  from: number,
  to: number,
): string | null {
  if (from === to) return '';
  const prev = new Int32Array(lv.w * lv.h).fill(-2);
  const prevMove = new Array<string>(lv.w * lv.h).fill('');
  const q = [from];
  prev[from] = -1;
  for (let h = 0; h < q.length; h++) {
    const u = q[h]!;
    for (const [dx, dy, letter] of DIRS) {
      const v = u + dy * lv.w + dx;
      if (!mask[v] || boxes.has(v) || prev[v] !== -2) continue;
      prev[v] = u;
      prevMove[v] = letter;
      if (v === to) {
        let moves = '';
        for (let t = v; t !== from; t = prev[t]!) moves = prevMove[t] + moves;
        return moves;
      }
      q.push(v);
    }
  }
  return null;
}

// Reachable set from `from` over a floor mask with `boxes` as obstacles.
function reachableSet(lv: Level, mask: Uint8Array, boxSet: Set<number>, from: number): Uint8Array {
  const seen = new Uint8Array(lv.w * lv.h);
  const q = [from];
  seen[from] = 1;
  for (let h = 0; h < q.length; h++) {
    const u = q[h]!;
    for (const v of maskNb(lv, mask, u)) {
      if (!boxSet.has(v) && !seen[v]) {
        seen[v] = 1;
        q.push(v);
      }
    }
  }
  return seen;
}

interface Room {
  door: number;
  cells: number[];
  boxes: number[];
  goals: number[];
  floor: Uint8Array;
  entry: number;
}

// Split a level into rooms. A room is a component cut off by one articulation
// point (its door) whose boxes and goals balance (counting a box/goal sitting
// on the door itself). Among candidates with the same box set we keep the
// largest component, so the room floor includes the hallway stub by its door
// and the player has room to stand when pushing a box off the door. Returns
// null when the split does not cover every box and goal exactly once.
function decompose(lv: Level): Room[] | null {
  const { w, h, wall, goal, boxes, player } = lv;
  const boxSet = new Set(boxes);
  const goalSet = new Set<number>();
  for (let i = 0; i < w * h; i++) if (goal[i]) goalSet.add(i);
  const floorCells: number[] = [];
  for (let i = 0; i < w * h; i++) if (!wall[i]) floorCells.push(i);
  interface Cand {
    door: number;
    cells: number[];
    boxes: number[];
    goals: number[];
  }
  const cands: Cand[] = [];
  for (const v of floorCells) {
    const blocked = new Uint8Array(w * h);
    blocked[v] = 1;
    for (const comp of components(lv, blocked)) {
      if (comp.includes(player)) continue;
      const cb = comp.filter((i) => boxSet.has(i));
      const cg = comp.filter((i) => goalSet.has(i));
      const boxesHere = boxSet.has(v) ? [...cb, v] : cb;
      const goalsHere = goalSet.has(v) ? [...cg, v] : cg;
      if (boxesHere.length === 0) continue;
      if (boxesHere.length !== goalsHere.length) continue;
      if (comp.length > 40) continue;
      boxesHere.sort((a, b) => a - b);
      goalsHere.sort((a, b) => a - b);
      cands.push({ door: v, cells: comp, boxes: boxesHere, goals: goalsHere });
    }
  }
  const byKey = new Map<string, Cand>();
  for (const c of cands) {
    const k = c.boxes.join(',');
    const old = byKey.get(k);
    if (!old || c.cells.length > old.cells.length) byKey.set(k, c);
  }
  const picked = [...byKey.values()];
  const seenB = new Set<number>();
  const seenG = new Set<number>();
  for (const r of picked) {
    for (const b of r.boxes) {
      if (seenB.has(b)) return null;
      seenB.add(b);
    }
    for (const g of r.goals) {
      if (seenG.has(g)) return null;
      seenG.add(g);
    }
  }
  if (seenB.size !== boxes.length || seenG.size !== goalSet.size) return null;
  // Room floor = component + door + door neighbours that hold no box of
  // another room (other rooms' boxes live in their components or on their
  // doors, so excluding every foreign box cell keeps walks legal).
  const allCells = new Set<number>();
  for (const r of picked) for (const c of r.cells) allCells.add(c);
  const rooms: Room[] = picked.map((r) => {
    const floor = new Uint8Array(w * h);
    for (const c of r.cells) floor[c] = 1;
    floor[r.door] = 1;
    for (const nb of floorNb(lv, r.door)) {
      if (!allCells.has(nb) && !boxSet.has(nb)) floor[nb] = 1;
    }
    // Entry square: the door, unless a box sits on it, in which case the
    // nearest hallway neighbour (nearest to the player start, ties by index).
    let entry = r.door;
    if (boxSet.has(r.door)) {
      const options = floorNb(lv, r.door).filter((n) => !allCells.has(n) && !boxSet.has(n));
      if (options.length === 0) return null as unknown as Room;
      const full = new Uint8Array(w * h);
      for (const c of floorCells) full[c] = 1;
      const d = bfsDist(lv, full, player);
      options.sort((a, b) => (d[a]! - d[b]!) || (a - b));
      entry = options[0]!;
      floor[entry] = 1;
    }
    return { door: r.door, cells: r.cells, boxes: r.boxes, goals: r.goals, floor, entry };
  });
  if (rooms.some((r) => r === null)) return null;
  return rooms;
}

// Dead squares by pull analysis: a square is live when it is a goal, or a
// box on it can be pulled back from a live square. Pulling a box from a live
// square t to s needs the player to end at p = 2s - t (floor). When t is a
// goal the player is already at s after the final push, so no walk is
// needed; otherwise the player must either end the previous pull at s
// (straight continuation through t) or be able to walk to s from some
// neighbour of t without crossing t. A box on a dead square can never reach
// a goal, so such states are pruned.
function deadSquares(lv: Level, floor: Uint8Array, goals: number[]): Uint8Array {
  const live = new Uint8Array(lv.w * lv.h);
  for (const g of goals) live[g] = 1;
  const goalSet = new Set(goals);
  // Can the player walk from `from` to any neighbour of `t` (other than `s`)
  // without stepping on `t`?
  const reachesNeighbourOf = (from: number, t: number, s: number): boolean => {
    const seen = new Uint8Array(lv.w * lv.h);
    const q = [from];
    seen[from] = 1;
    for (let h = 0; h < q.length; h++) {
      const u = q[h]!;
      for (const v of maskNb(lv, floor, u)) {
        if (v === t || seen[v]) continue;
        // v is a neighbour of t and not s: the player can stand by t.
        if (Math.abs((v % lv.w) - (t % lv.w)) + Math.abs(((v / lv.w) | 0) - ((t / lv.w) | 0)) === 1 && v !== s) return true;
        seen[v] = 1;
        q.push(v);
      }
    }
    return false;
  };
  let changed = true;
  while (changed) {
    changed = false;
    for (let s = 0; s < lv.w * lv.h; s++) {
      if (!floor[s] || live[s]) continue;
      for (const [dx, dy] of DIRS) {
        const t = s + dy * lv.w + dx;
        const p = s - dy * lv.w - dx;
        if (!floor[t] || !live[t] || !floor[p]) continue;
        // Goal neighbour: after the final push the player is already at s,
        // so no walk is needed — the box reaches the goal in one push.
        if (goalSet.has(t)) {
          live[s] = 1;
          changed = true;
          break;
        }
        // (a) straight continuation: the chain came through the square
        // opposite s across t, so the player ends the previous pull at s.
        const q = s + 2 * dy * lv.w + 2 * dx;
        if (floor[q] && live[q]) {
          live[s] = 1;
          changed = true;
          break;
        }
        // (b) the player walks to s from some neighbour of t.
        if (reachesNeighbourOf(s, t, s)) {
          live[s] = 1;
          changed = true;
          break;
        }
      }
    }
  }
  const dead = new Uint8Array(lv.w * lv.h);
  for (let i = 0; i < dead.length; i++) if (floor[i] && !live[i]) dead[i] = 1;
  return dead;
}

// Min-cost assignment of boxes to goals over precomputed BFS distances.
// Brute force for n <= 4 (24 permutations), sum-of-minima above that.
// Infinity when some box cannot reach any goal.
function matchingHeuristic(boxes: number[], goals: number[], distFromGoal: Int32Array[]): number {
  const n = boxes.length;
  if (n <= 4) {
    const perm = goals.map((_, i) => i);
    let best = Infinity;
    const rec = (i: number, cost: number): void => {
      if (cost >= best) return;
      if (i === n) {
        best = cost;
        return;
      }
      for (let j = i; j < n; j++) {
        const t = perm[i]!;
        perm[i] = perm[j]!;
        perm[j] = t;
        const d = distFromGoal[perm[i]!]![boxes[i]!]!;
        if (d >= 0) rec(i + 1, cost + d);
        const t2 = perm[i]!;
        perm[i] = perm[j]!;
        perm[j] = t2;
      }
    };
    rec(0, 0);
    return best;
  }
  let sum = 0;
  for (const b of boxes) {
    let m = Infinity;
    for (let g = 0; g < goals.length; g++) {
      const d = distFromGoal[g]![b]!;
      if (d >= 0 && d < m) m = d;
    }
    if (m === Infinity) return Infinity;
    sum += m;
  }
  return sum;
}

// Frozen boxes in the current configuration: a box is frozen when every
// push direction is blocked by a wall or by another frozen box. A frozen box
// off its goal dead-ends the state.
function frozenBoxes(lv: Level, floor: Uint8Array, boxes: number[], goal: Uint8Array): Set<number> {
  const boxSet = new Set(boxes);
  const frozen = new Set<number>();
  let changed = true;
  while (changed) {
    changed = false;
    for (const b of boxes) {
      if (goal[b] || frozen.has(b)) continue;
      let stuck = true;
      for (const [dx, dy] of DIRS) {
        const t = b + dy * lv.w + dx;
        if (floor[t] && !boxSet.has(t)) {
          stuck = false;
          break;
        }
        if (floor[t] && boxSet.has(t) && !frozen.has(t)) {
          stuck = false;
          break;
        }
      }
      if (stuck) {
        frozen.add(b);
        changed = true;
      }
    }
  }
  return frozen;
}

// 2x2 block deadlock: a 2x2 of walls/boxes holding a box off its goal.
function hasBlockDeadlock(lv: Level, boxes: number[], goal: Uint8Array): boolean {
  const boxSet = new Set(boxes);
  const solid = (i: number): boolean => lv.wall[i] || boxSet.has(i);
  for (const b of boxes) {
    if (goal[b]) continue;
    const x = b % lv.w;
    const y = (b / lv.w) | 0;
    for (const [ox, oy] of [[-1, -1], [0, -1], [-1, 0], [0, 0]]) {
      const cx = x + ox;
      const cy = y + oy;
      if (cx < 0 || cy < 0 || cx + 1 >= lv.w || cy + 1 >= lv.h) continue;
      const a = cy * lv.w + cx;
      if (solid(a) && solid(a + 1) && solid(a + lv.w) && solid(a + lv.w + 1)) return true;
    }
  }
  return false;
}

interface SearchResult {
  moves: string;
  pushes: number;
  endPlayer: number;
  endBoxes: number[];
}

// Push-optimal A* over one floor mask. `startBoxes`/`startPlayer` give the
// start state; `goals` the target squares; when `requireExit` the goal test
// also needs the player able to walk back to `door` with no push.
function pushAStar(
  lv: Level,
  floor: Uint8Array,
  goals: number[],
  startBoxes: number[],
  startPlayer: number,
  door: number,
  requireExit: boolean,
  expansionCap: number,
): SearchResult | null {
  const goalSet = new Set(goals);
  const distFromGoal = goals.map((g) => bfsDist(lv, floor, g));
  const dead = deadSquares(lv, floor, goals);
  const h0 = matchingHeuristic(startBoxes, goals, distFromGoal);
  if (h0 === Infinity) return null;
  if (startBoxes.some((b) => dead[b] && !lv.goal[b])) return null;

  // Binary heap on (f, -g, insertion counter): f = pushes + heuristic.
  const heap: [number, number, number, number][] = [];
  const less = (a: [number, number, number, number], b: [number, number, number, number]): boolean =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] < b[1] || (a[1] === b[1] && a[2] < b[2])));
  const heapPush = (e: [number, number, number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i]!, heap[p]!)) break;
      [heap[i], heap[p]] = [heap[p]!, heap[i]!];
      i = p;
    }
  };
  const heapPop = (): [number, number, number, number] => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1]!, heap[c]!)) c++;
        if (!less(heap[c]!, heap[i]!)) break;
        [heap[i], heap[c]] = [heap[c]!, heap[i]!];
        i = c;
      }
    }
    return top;
  };

  const keyOf = (p: number, bs: number[]): string => String.fromCharCode(p, ...bs);
  const players: number[] = [startPlayer];
  const boxList: number[][] = [startBoxes.slice().sort((a, b) => a - b)];
  const gScore: number[] = [0];
  const parent: number[] = [-1];
  const trans: string[] = [''];
  const best = new Map<string, number>([[keyOf(startPlayer, boxList[0]!), 0]]);
  let counter = 0;
  heapPush([h0, 0, counter++, 0]);

  const solved = (p: number, bs: number[]): boolean => {
    if (!bs.every((b) => goalSet.has(b))) return false;
    if (!requireExit) return true;
    const reach = reachableSet(lv, floor, new Set(bs), p);
    return reach[door] === 1;
  };

  let expanded = 0;
  while (heap.length > 0 && expanded < expansionCap) {
    const [, negG, , s] = heapPop();
    const g = -negG;
    if (best.get(keyOf(players[s]!, boxList[s]!)) !== s) continue;
    const p = players[s]!;
    const bs = boxList[s]!;
    if (solved(p, bs)) {
      let moves = '';
      for (let t = s; t > 0; t = parent[t]!) moves = trans[t]! + moves;
      return { moves, pushes: g, endPlayer: p, endBoxes: bs.slice() };
    }
    expanded++;
    const boxSet = new Set(bs);
    const reach = reachableSet(lv, floor, boxSet, p);
    for (let bi = 0; bi < bs.length; bi++) {
      const b = bs[bi]!;
      for (const [dx, dy, letter] of DIRS) {
        const t = b + dy * lv.w + dx;
        const stand = b - dy * lv.w - dx;
        if (!floor[t] || boxSet.has(t)) continue;
        if (!reach[stand]) continue;
        const nb = bs.slice();
        nb[bi] = t;
        nb.sort((a, b) => a - b);
        if (nb.some((x) => dead[x] && !lv.goal[x])) continue;
        if (frozenBoxes(lv, floor, nb, lv.goal).size > 0) continue;
        if (hasBlockDeadlock(lv, nb, lv.goal)) continue;
        const h = matchingHeuristic(nb, goals, distFromGoal);
        if (h === Infinity) continue;
        const k = keyOf(b, nb);
        const seen = best.get(k);
        if (seen !== undefined && gScore[seen]! <= g + 1) continue;
        // Walk from the player to the push square, then the push itself.
        const walk = walkPath(lv, floor, boxSet, p, stand);
        if (walk === null) continue;
        const idx = players.length;
        players.push(b);
        boxList.push(nb);
        gScore.push(g + 1);
        parent.push(s);
        trans.push(walk + letter.toUpperCase());
        best.set(k, idx);
        heapPush([g + 1 + h, -(g + 1), counter++, idx]);
      }
    }
  }
  return null;
}

// Solve one room from its entry square; appends the exit walk to the door
// (when the room is not the last one). Returns null when unsolved.
function solveRoom(
  lv: Level,
  room: Room,
  requireExit: boolean,
): SearchResult | null {
  const res = pushAStar(
    lv,
    room.floor,
    room.goals,
    room.boxes,
    room.entry,
    room.door,
    requireExit,
    ROOM_EXPANSIONS,
  );
  if (!res) return null;
  if (!requireExit) return res;
  // Walk back out to the door with the final boxes as obstacles.
  const walk = walkPath(lv, room.floor, new Set(res.endBoxes), res.endPlayer, room.door);
  if (walk === null) return null;
  return { moves: res.moves + walk, pushes: res.pushes, endPlayer: room.door, endBoxes: res.endBoxes };
}

// Full-level push-A* fallback (sum-of-minima heuristic; no exit test).
function solveFull(lv: Level): SearchResult | null {
  const floor = new Uint8Array(lv.w * lv.h);
  for (let i = 0; i < floor.length; i++) if (!lv.wall[i]) floor[i] = 1;
  const goals: number[] = [];
  for (let i = 0; i < floor.length; i++) if (lv.goal[i]) goals.push(i);
  return pushAStar(lv, floor, goals, lv.boxes, lv.player, lv.player, false, FULL_EXPANSIONS);
}

function solveLevel(lv: Level): string {
  const rooms = decompose(lv);
  if (rooms) {
    // Rooms whose door is a goal go last (a box ending on the door would
    // trap the player); otherwise nearest door first, ties by door index.
    const full = new Uint8Array(lv.w * lv.h);
    for (let i = 0; i < full.length; i++) if (!lv.wall[i]) full[i] = 1;
    const d0 = bfsDist(lv, full, lv.player);
    const ordered = rooms.slice().sort((a, b) => {
      const ga = lv.goal[a.door] ? 1 : 0;
      const gb = lv.goal[b.door] ? 1 : 0;
      if (ga !== gb) return ga - gb;
      return (d0[a.entry]! - d0[b.entry]!) || (a.door - b.door);
    });
    let moves = '';
    let player = lv.player;
    let globalBoxes = lv.boxes.slice();
    for (let ri = 0; ri < ordered.length; ri++) {
      const room = ordered[ri]!;
      const last = ri === ordered.length - 1;
      const walk = walkPath(lv, full, new Set(globalBoxes), player, room.entry);
      if (walk === null) {
        const fb = solveFull(lv);
        return fb ? fb.moves : '';
      }
      moves += walk;
      player = room.entry;
      const res = solveRoom(lv, room, !last);
      if (!res) {
        const fb = solveFull(lv);
        return fb ? fb.moves : '';
      }
      moves += res.moves;
      player = res.endPlayer;
      const done = new Set(res.endBoxes);
      globalBoxes = globalBoxes.filter((b) => !room.boxes.includes(b));
      for (const b of done) globalBoxes.push(b);
      globalBoxes.sort((a, b) => a - b);
    }
    return moves;
  }
  const fb = solveFull(lv);
  return fb ? fb.moves : '';
}

for (const id of readdirSync(I).sort()) {
  let moves = '';
  try {
    if (!statSync(join(I, id)).isDirectory()) continue;
    const lv = parseLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
    moves = solveLevel(lv);
  } catch {
    moves = '';
  }
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
