// sokoban-solver-01-run-20261121 entry.
//
// Every level is generated as a one-square-wide hallway (where the player starts) with 1-4 small
// rooms branching off it, each connected to the hallway through exactly one door cell, each room
// holding as many boxes as goals. Because a box pushed out of its room into the hallway can never
// be pushed back, an optimal solution never needs to push a box out of its room: the level
// decomposes into independent sub-puzzles, one per room. We find the rooms with a cut-vertex
// (articulation point) analysis of the floor graph, solve each room to true push-optimality with an
// exhaustive BFS over (player-region, box-set) states, then chain the rooms together with plain
// player-only pathfinding through the boxless hallway. A level that doesn't match this shape (zero
// rooms found) falls back to a joint whole-board search with deadlock pruning.
//
// Determinism: no wall-clock reads, no Math.random, no reliance on object/Map iteration order that
// could vary run to run for the same input -- every data structure here is built by iterating fixed,
// input-derived sequences (sorted arrays, cells in row-major order, directions in a fixed order), so
// two runs on the same input produce byte-identical output.

import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// --------------------------------------------------------------------------------------------------
// Level model
// --------------------------------------------------------------------------------------------------

type Level = {
  w: number;
  h: number;
  wall: Uint8Array;
  goal: Uint8Array;
  box: Uint8Array;
  player: number;
};

/** Parse an XSB level, padding short rows with wall ('#'), exactly as the checker does. */
function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const w = Math.max(...rows.map((r) => r.length));
  const h = rows.length;
  const size = w * h;
  const wall = new Uint8Array(size);
  const goal = new Uint8Array(size);
  const box = new Uint8Array(size);
  let player = -1;
  for (let y = 0; y < h; y++) {
    const row = rows[y]!;
    for (let x = 0; x < w; x++) {
      const ch = row[x] ?? '#';
      const i = y * w + x;
      wall[i] = ch === '#' ? 1 : 0;
      if (ch === '.' || ch === '*' || ch === '+') goal[i] = 1;
      if (ch === '$' || ch === '*') box[i] = 1;
      if (ch === '@' || ch === '+') player = i;
    }
  }
  return { w, h, wall, goal, box, player };
}

// Fixed direction order used everywhere so processing order never depends on iteration order of an
// unordered collection: up, down, left, right.
const DIRS: { letter: string; dx: number; dy: number }[] = [
  { letter: 'u', dx: 0, dy: -1 },
  { letter: 'd', dx: 0, dy: 1 },
  { letter: 'l', dx: -1, dy: 0 },
  { letter: 'r', dx: 1, dy: 0 },
];

function inBounds(x: number, y: number, w: number, h: number): boolean {
  return x >= 0 && x < w && y >= 0 && y < h;
}

/** All floor (non-wall) cell indices, row-major order. */
function floorCells(level: Level): number[] {
  const out: number[] = [];
  for (let i = 0; i < level.w * level.h; i++) if (!level.wall[i]) out.push(i);
  return out;
}

/** Floor-cell adjacency graph (4-directional), neighbors listed in the fixed DIRS order. */
function buildAdjacency(level: Level, floor: number[]): Map<number, number[]> {
  const adj = new Map<number, number[]>();
  for (const i of floor) {
    const x = i % level.w;
    const y = Math.floor(i / level.w);
    const neigh: number[] = [];
    for (const { dx, dy } of DIRS) {
      const nx = x + dx;
      const ny = y + dy;
      if (!inBounds(nx, ny, level.w, level.h)) continue;
      const j = ny * level.w + nx;
      if (!level.wall[j]) neigh.push(j);
    }
    adj.set(i, neigh);
  }
  return adj;
}

// --------------------------------------------------------------------------------------------------
// Step 3: articulation points via Tarjan's DFS-based algorithm (O(V+E)).
// --------------------------------------------------------------------------------------------------

function findArticulationPoints(floor: number[], adj: Map<number, number[]>, root: number): Set<number> {
  const disc = new Map<number, number>();
  const low = new Map<number, number>();
  const parent = new Map<number, number>();
  const aps = new Set<number>();
  let timer = 0;

  // Iterative DFS (explicit stack) to avoid any recursion-depth concerns on larger boards, while
  // reproducing exactly the classic recursive low-link algorithm's behaviour.
  type Frame = { u: number; it: number };
  const stack: Frame[] = [{ u: root, it: 0 }];
  disc.set(root, timer);
  low.set(root, timer);
  timer++;
  let rootChildren = 0;

  while (stack.length > 0) {
    const frame = stack[stack.length - 1]!;
    const u = frame.u;
    const neigh = adj.get(u)!;
    if (frame.it < neigh.length) {
      const v = neigh[frame.it]!;
      frame.it++;
      if (!disc.has(v)) {
        parent.set(v, u);
        if (u === root) rootChildren++;
        disc.set(v, timer);
        low.set(v, timer);
        timer++;
        stack.push({ u: v, it: 0 });
      } else if (v !== parent.get(u)) {
        low.set(u, Math.min(low.get(u)!, disc.get(v)!));
      }
    } else {
      stack.pop();
      const p = parent.get(u);
      if (p !== undefined) {
        low.set(p, Math.min(low.get(p)!, low.get(u)!));
        if (p !== root && low.get(u)! >= disc.get(p)!) aps.add(p);
      }
    }
  }
  if (rootChildren > 1) aps.add(root);
  return aps;
}

// --------------------------------------------------------------------------------------------------
// Step 4: room (and door) detection.
//
// For each articulation point `ap`, removing it from the floor graph splits off a set of cells
// unreachable from the player (`unreached`). If that set, plus `ap` itself when `ap` carries a box
// or goal, holds exactly as many boxes as goals (3 or 4, per the generator), it is a candidate room
// whose boundary cell is `ap`. The hallway is itself a one-wide corridor, so most cells along the
// path from the player to a room are *also* articulation points separating the same room from the
// player (every interior vertex of a path cuts it); those candidates share the exact same box set as
// the true door, just with a larger `unreached` side. The true door is the candidate with the
// *smallest* unreached side for a given box set -- the cut closest to the room itself.
// --------------------------------------------------------------------------------------------------

type Room = {
  door: number; // the floor cell that is the sole connection between the room and the hallway
  approach: number; // the hallway-side floor cell immediately outside the door
  cells: number[]; // every floor cell of the room's own playable area, including the door
  boxes: number[]; // the room's box cells (sorted)
  goals: number[]; // the room's goal cells (sorted)
};

/** BFS over the floor graph from `root`, pretending `removed` does not exist; returns unreached cells. */
function unreachedWithout(floor: number[], adj: Map<number, number[]>, root: number, removed: number): number[] {
  const seen = new Set<number>([root, removed]);
  const stack: number[] = [root];
  while (stack.length > 0) {
    const c = stack.pop()!;
    for (const n of adj.get(c)!) {
      if (!seen.has(n)) {
        seen.add(n);
        stack.push(n);
      }
    }
  }
  const out: number[] = [];
  for (const f of floor) if (!seen.has(f)) out.push(f);
  return out;
}

function findRooms(level: Level, floor: number[], adj: Map<number, number[]>): Room[] {
  const aps = findArticulationPoints(floor, adj, level.player);
  // Sort candidate APs by cell index so the whole detection pass is deterministic and order-free.
  const apList = [...aps].sort((a, b) => a - b);

  type Candidate = { ap: number; unreached: number[]; boxSig: string };
  const candidates: Candidate[] = [];
  for (const ap of apList) {
    const unreached = unreachedWithout(floor, adj, level.player, ap);
    if (unreached.length === 0) continue;
    const boxesIn = unreached.filter((c) => level.box[c]);
    const goalsIn = unreached.filter((c) => level.goal[c]);
    let bcount = boxesIn.length;
    let gcount = goalsIn.length;
    const boxCells = boxesIn.slice();
    if (level.box[ap]) {
      bcount++;
      boxCells.push(ap);
    }
    if (level.goal[ap]) gcount++;
    if (bcount !== gcount || bcount === 0) continue;
    if (bcount !== 3 && bcount !== 4) continue; // rooms always hold 3 or 4 boxes
    boxCells.sort((a, b) => a - b);
    candidates.push({ ap, unreached, boxSig: boxCells.join(',') });
  }

  // Group candidates that describe the same room (identical box set); keep the smallest cut.
  const groups = new Map<string, Candidate[]>();
  for (const c of candidates) {
    const g = groups.get(c.boxSig);
    if (g) g.push(c);
    else groups.set(c.boxSig, [c]);
  }

  const rooms: Room[] = [];
  const sigKeys = [...groups.keys()].sort();
  for (const sig of sigKeys) {
    const group = groups.get(sig)!;
    group.sort((a, b) => (a.unreached.length - b.unreached.length) || (a.ap - b.ap));
    const best = group[0]!;
    const door = best.ap;
    const roomCells = best.unreached.slice();
    roomCells.push(door);
    roomCells.sort((a, b) => a - b);
    const boxes = sig.split(',').map(Number);
    const goals = best.unreached.filter((c) => level.goal[c]);
    if (level.goal[door]) goals.push(door);
    goals.sort((a, b) => a - b);

    const roomCellSet = new Set(roomCells);
    const hallwayNeighbors = adj.get(door)!.filter((n) => !roomCellSet.has(n));
    hallwayNeighbors.sort((a, b) => a - b);
    const approach = hallwayNeighbors[0]; // deterministic: smallest cell index
    if (approach === undefined) continue; // malformed; skip this candidate defensively

    rooms.push({ door, approach, cells: roomCells, boxes, goals });
  }

  // Visit order does not affect the push total (rooms are independent); sort left-to-right by the
  // door's x coordinate, then y, for a readable and fully deterministic order.
  rooms.sort((a, b) => {
    const ax = a.door % level.w;
    const bx = b.door % level.w;
    if (ax !== bx) return ax - bx;
    const ay = Math.floor(a.door / level.w);
    const by = Math.floor(b.door / level.w);
    return ay - by;
  });
  return rooms;
}

// --------------------------------------------------------------------------------------------------
// Plain BFS pathfinding for the player alone over floor cells (used in the boxless hallway, and to
// walk between pushes inside a room once the room's own domain is fixed).
// --------------------------------------------------------------------------------------------------

/** Shortest player-only move string from `from` to `to`, avoiding `blocked` cells (walls are already
 * excluded from `adj`). Returns null if unreachable. */
function bfsPath(
  level: Level,
  adj: Map<number, number[]>,
  from: number,
  to: number,
  blocked: Set<number>,
): string | null {
  if (from === to) return '';
  const prev = new Map<number, { from: number; letter: string }>();
  const seen = new Set<number>([from]);
  const queue: number[] = [from];
  let qi = 0;
  while (qi < queue.length) {
    const cur = queue[qi++]!;
    const cx = cur % level.w;
    const cy = Math.floor(cur / level.w);
    for (const { letter, dx, dy } of DIRS) {
      const nx = cx + dx;
      const ny = cy + dy;
      if (!inBounds(nx, ny, level.w, level.h)) continue;
      const n = ny * level.w + nx;
      if (level.wall[n] || blocked.has(n) || seen.has(n)) continue;
      seen.add(n);
      prev.set(n, { from: cur, letter });
      if (n === to) {
        let moves = '';
        let c = to;
        while (c !== from) {
          const p = prev.get(c)!;
          moves = p.letter + moves;
          c = p.from;
        }
        return moves;
      }
      queue.push(n);
    }
  }
  return null;
}

// --------------------------------------------------------------------------------------------------
// Step 5: exact room solver. A room's state space is tiny (<= ~38 cells, 3-4 boxes), so a plain BFS
// over "macro" states (player's reachable region, box set) finds the true minimum-push solution: a
// BFS layer corresponds to exactly one push, since walking (non-push) steps never change the layer.
// --------------------------------------------------------------------------------------------------

function boxesKeyOf(boxes: number[]): string {
  // `boxes` must already be sorted by cell index: the key is only a valid state identity (and
  // dedupes correctly against other states) when box order is canonical, not insertion order.
  return boxes.join(',');
}

/** Flood-fill the domain minus the current boxes, starting at `start`; returns every reached cell. */
function floodFill(domainAdj: Map<number, number[]>, boxes: Set<number>, start: number): number[] {
  const seen = new Set<number>([start]);
  const stack = [start];
  while (stack.length > 0) {
    const c = stack.pop()!;
    for (const n of domainAdj.get(c)!) {
      if (!boxes.has(n) && !seen.has(n)) {
        seen.add(n);
        stack.push(n);
      }
    }
  }
  return [...seen];
}

/**
 * Solve one room exactly: returns the push-optimal move string that starts with the player standing
 * at `room.approach` (outside the door, in the hallway) and ends with the player back at
 * `room.approach`, having pushed every room box onto a room goal.
 */
function solveRoom(level: Level, room: Room): string {
  const domainCells = [...room.cells, room.approach].sort((a, b) => a - b);
  const domainSet = new Set(domainCells);
  const domainAdj = new Map<number, number[]>();
  for (const c of domainCells) {
    const x = c % level.w;
    const y = Math.floor(c / level.w);
    const neigh: number[] = [];
    for (const { dx, dy } of DIRS) {
      const nx = x + dx;
      const ny = y + dy;
      if (!inBounds(nx, ny, level.w, level.h)) continue;
      const j = ny * level.w + nx;
      if (domainSet.has(j)) neigh.push(j);
    }
    domainAdj.set(c, neigh);
  }

  const initialBoxes = room.boxes.slice().sort((a, b) => a - b);
  const goalSet = new Set(room.goals);
  const goalKey = room.goals.slice().sort((a, b) => a - b).join(',');

  const rep0 = Math.min(...floodFill(domainAdj, new Set(initialBoxes), room.approach));
  const startKey = `${rep0}|${boxesKeyOf(initialBoxes)}`;

  type Action = { boxFrom: number; letter: string; dx: number; dy: number };
  const visited = new Set<string>([startKey]);
  const parentKey = new Map<string, string>();
  const parentAction = new Map<string, Action>();
  const stateBoxes = new Map<string, number[]>([[startKey, initialBoxes]]);

  const queue: string[] = [startKey];
  let qi = 0;
  let goalStateKey: string | null = null;
  if (initialBoxes.join(',') === goalKey) goalStateKey = startKey;

  while (qi < queue.length && goalStateKey === null) {
    const key = queue[qi++]!;
    const rep = Number(key.split('|')[0]);
    const boxes = stateBoxes.get(key)!;
    const boxSet = new Set(boxes);
    const reachable = new Set(floodFill(domainAdj, boxSet, rep));

    for (const b of boxes) {
      const bx = b % level.w;
      const by = Math.floor(b / level.w);
      for (const { letter, dx, dy } of DIRS) {
        const fromX = bx - dx;
        const fromY = by - dy;
        const toX = bx + dx;
        const toY = by + dy;
        if (!inBounds(fromX, fromY, level.w, level.h) || !inBounds(toX, toY, level.w, level.h)) continue;
        const pushFrom = fromY * level.w + fromX;
        const pushTo = toY * level.w + toX;
        if (!domainSet.has(pushFrom) || !reachable.has(pushFrom)) continue;
        if (!domainSet.has(pushTo) || boxSet.has(pushTo)) continue;

        const newBoxes = boxes.map((x) => (x === b ? pushTo : x)).sort((a, c) => a - c);
        const newBoxesKey = boxesKeyOf(newBoxes);
        const newBoxSet = new Set(newBoxes);
        const newRep = Math.min(...floodFill(domainAdj, newBoxSet, b));
        const newKey = `${newRep}|${newBoxesKey}`;
        if (visited.has(newKey)) continue;
        visited.add(newKey);
        parentKey.set(newKey, key);
        parentAction.set(newKey, { boxFrom: b, letter, dx, dy });
        stateBoxes.set(newKey, newBoxes);
        queue.push(newKey);
        if (newBoxesKey === goalKey) {
          goalStateKey = newKey;
          break;
        }
      }
      if (goalStateKey !== null) break;
    }
  }

  if (goalStateKey === null) {
    throw new Error(`room with door ${room.door} has no solution (search exhausted ${visited.size} states)`);
  }

  // Reconstruct the push sequence (oldest first).
  const actions: Action[] = [];
  let k: string | null = goalStateKey;
  while (k !== startKey) {
    actions.push(parentAction.get(k)!);
    k = parentKey.get(k)!;
  }
  actions.reverse();

  // Forward-simulate: walk to each push's starting cell, then push.
  let moves = '';
  let playerAt = room.approach;
  let boxes = new Set(initialBoxes);
  for (const act of actions) {
    const fx = (act.boxFrom % level.w) - act.dx;
    const fy = Math.floor(act.boxFrom / level.w) - act.dy;
    const pushFrom = fy * level.w + fx;
    const walk = bfsPathInDomain(level, domainAdj, playerAt, pushFrom, boxes);
    if (walk === null) throw new Error('internal error: room reconstruction could not walk to push cell');
    moves += walk;
    moves += act.letter;
    boxes.delete(act.boxFrom);
    const pushTo = act.boxFrom + act.dy * level.w + act.dx;
    boxes.add(pushTo);
    playerAt = act.boxFrom;
  }
  // Walk back out to the approach cell.
  const tail = bfsPathInDomain(level, domainAdj, playerAt, room.approach, boxes);
  if (tail === null) throw new Error('internal error: room reconstruction could not walk back to the approach cell');
  moves += tail;
  return moves;
}

/** Shortest player-only move string within a fixed domain adjacency map, avoiding `boxes`. */
function bfsPathInDomain(
  level: Level,
  domainAdj: Map<number, number[]>,
  from: number,
  to: number,
  boxes: Set<number>,
): string | null {
  if (from === to) return '';
  const prev = new Map<number, { from: number; letter: string }>();
  const seen = new Set<number>([from]);
  const queue: number[] = [from];
  let qi = 0;
  while (qi < queue.length) {
    const cur = queue[qi++]!;
    const cx = cur % level.w;
    const cy = Math.floor(cur / level.w);
    for (const { letter, dx, dy } of DIRS) {
      const nx = cx + dx;
      const ny = cy + dy;
      if (!inBounds(nx, ny, level.w, level.h)) continue;
      const n = ny * level.w + nx;
      if (!domainAdj.has(n) || boxes.has(n) || seen.has(n)) continue;
      seen.add(n);
      prev.set(n, { from: cur, letter });
      if (n === to) {
        let moves = '';
        let c = to;
        while (c !== from) {
          const p = prev.get(c)!;
          moves = p.letter + moves;
          c = p.from;
        }
        return moves;
      }
      queue.push(n);
    }
  }
  return null;
}

// --------------------------------------------------------------------------------------------------
// Step 6-7: chain the rooms together, starting from the player's real position, visiting each room's
// approach cell via plain BFS (the hallway never holds a box so this is always a simple walk), then
// solving the room and moving on.
// --------------------------------------------------------------------------------------------------

function solveByRooms(level: Level): string | null {
  const floor = floorCells(level);
  const adj = buildAdjacency(level, floor);
  const rooms = findRooms(level, floor, adj);
  if (rooms.length === 0) return null;

  // Sanity check: every box and every goal on the board must belong to exactly one detected room,
  // and each room must be internally self-sufficient. If not, this level doesn't match the expected
  // generator shape -- bail out to the fallback solver rather than risk an incorrect answer.
  const allBoxes = floor.filter((c) => level.box[c]).sort((a, b) => a - b);
  const allGoals = floor.filter((c) => level.goal[c]).sort((a, b) => a - b);
  const coveredBoxes = rooms.flatMap((r) => r.boxes).sort((a, b) => a - b);
  const coveredGoals = rooms.flatMap((r) => r.goals).sort((a, b) => a - b);
  if (coveredBoxes.join(',') !== allBoxes.join(',')) return null;
  if (coveredGoals.join(',') !== allGoals.join(',')) return null;

  let moves = '';
  let playerAt = level.player;
  const liveBoxes = new Set(allBoxes);

  for (const room of rooms) {
    const toApproach = bfsPath(level, adj, playerAt, room.approach, liveBoxes);
    if (toApproach === null) return null;
    moves += toApproach;
    playerAt = room.approach;

    const roomMoves = solveRoom(level, room);
    moves += roomMoves;
    playerAt = room.approach; // solveRoom always returns the player to the approach cell

    for (const b of room.boxes) liveBoxes.delete(b);
    for (const g of room.goals) liveBoxes.add(g);
  }

  return moves;
}

// --------------------------------------------------------------------------------------------------
// Step 8: fallback for a level that doesn't decompose into rooms -- a joint search over the whole
// board with simple corner-deadlock pruning, capped by a fixed expansion budget (never the clock).
// --------------------------------------------------------------------------------------------------

function solveWhole(level: Level): string {
  const floor = floorCells(level);
  const floorSet = new Set(floor);
  const goals = floor.filter((c) => level.goal[c]);
  const goalSet = new Set(goals);
  const initialBoxes = floor.filter((c) => level.box[c]).sort((a, b) => a - b);

  // A box resting on a non-goal cell that is wedged into a corner (both an orthogonal pair of walls)
  // can never be pushed again in either axis: prune any state containing one.
  const isCornerDeadlock = (b: number): boolean => {
    if (goalSet.has(b)) return false;
    const x = b % level.w;
    const y = Math.floor(b / level.w);
    const up = y > 0 ? level.wall[b - level.w] : 1;
    const down = y < level.h - 1 ? level.wall[b + level.w] : 1;
    const left = x > 0 ? level.wall[b - 1] : 1;
    const right = x < level.w - 1 ? level.wall[b + 1] : 1;
    return !!((up && left) || (up && right) || (down && left) || (down && right));
  };

  const distToGoal = (b: number): number => {
    let best = Infinity;
    const bx = b % level.w;
    const by = Math.floor(b / level.w);
    for (const g of goals) {
      const gx = g % level.w;
      const gy = Math.floor(g / level.w);
      const d = Math.abs(gx - bx) + Math.abs(gy - by);
      if (d < best) best = d;
    }
    return best;
  };
  const heuristic = (boxes: number[]): number => boxes.reduce((a, b) => a + distToGoal(b), 0);

  const key = (p: number, boxes: number[]): string => `${p}|${boxes.join(',')}`;

  type Node = { p: number; boxes: number[]; g: number; f: number; parent: string | null; move: string };
  const start = key(level.player, initialBoxes);
  const nodes = new Map<string, Node>([[start, { p: level.player, boxes: initialBoxes, g: 0, f: heuristic(initialBoxes), parent: null, move: '' }]]);

  // Binary-heap min-priority queue on (f, g descending for tie-break determinism, key).
  const heap: string[] = [start];
  const heapLess = (ak: string, bk: string): boolean => {
    const a = nodes.get(ak)!;
    const b = nodes.get(bk)!;
    if (a.f !== b.f) return a.f < b.f;
    if (a.g !== b.g) return a.g > b.g;
    return ak < bk;
  };
  const heapPush = (k: string): void => {
    heap.push(k);
    let i = heap.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!heapLess(heap[i]!, heap[p]!)) break;
      [heap[i], heap[p]] = [heap[p]!, heap[i]!];
      i = p;
    }
  };
  const heapPop = (): string => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      let i = 0;
      for (;;) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && heapLess(heap[c + 1]!, heap[c]!)) c++;
        if (!heapLess(heap[c]!, heap[i]!)) break;
        [heap[i], heap[c]] = [heap[c]!, heap[i]!];
        i = c;
      }
    }
    return top;
  };

  const EXPANSIONS = 150_000;
  let expanded = 0;
  let goalKey: string | null = null;

  while (heap.length > 0 && expanded < EXPANSIONS) {
    const k = heapPop();
    const node = nodes.get(k)!;
    if (node.boxes.every((b) => goalSet.has(b))) {
      goalKey = k;
      break;
    }
    expanded++;
    const px = node.p % level.w;
    const py = Math.floor(node.p / level.w);
    const boxSet = new Set(node.boxes);
    for (const { letter, dx, dy } of DIRS) {
      const nx = px + dx;
      const ny = py + dy;
      if (!inBounds(nx, ny, level.w, level.h)) continue;
      const np = ny * level.w + nx;
      if (!floorSet.has(np)) continue;
      let newBoxes = node.boxes;
      let move = letter;
      if (boxSet.has(np)) {
        const bx = nx + dx;
        const by = ny + dy;
        if (!inBounds(bx, by, level.w, level.h)) continue;
        const bp = by * level.w + bx;
        if (!floorSet.has(bp) || boxSet.has(bp)) continue;
        if (isCornerDeadlock(bp)) continue;
        newBoxes = node.boxes.map((b) => (b === np ? bp : b)).sort((a, b) => a - b);
        move = letter.toUpperCase();
      }
      const nk = key(np, newBoxes);
      const ng = node.g + 1;
      const existing = nodes.get(nk);
      if (existing !== undefined && existing.g <= ng) continue;
      nodes.set(nk, { p: np, boxes: newBoxes, g: ng, f: ng + heuristic(newBoxes), parent: k, move });
      heapPush(nk);
    }
  }

  if (goalKey === null) return '';
  let moves = '';
  let c: string | null = goalKey;
  while (c !== null) {
    const node = nodes.get(c)!;
    moves = node.move + moves;
    c = node.parent;
  }
  return moves;
}

// --------------------------------------------------------------------------------------------------
// Entry point
// --------------------------------------------------------------------------------------------------

function solve(text: string): string {
  const level = parseLevel(text);
  const roomSolution = solveByRooms(level);
  if (roomSolution !== null) return roomSolution;
  return solveWhole(level);
}

for (const id of readdirSync(I).sort()) {
  const moves = solve(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
