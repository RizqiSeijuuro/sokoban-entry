# sokoban-entry

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/RizqiSeijuuro/sokoban-entry/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/RizqiSeijuuro/sokoban-entry/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver built around the shape of this bounty's generator: every level is a
one-square-wide hallway (where the player starts) with 1-4 small rooms branching off it, each room
connected to the hallway through exactly one door cell and holding exactly as many boxes as goals.
Because a box pushed out of its room into the hallway can never be pushed back in any sensible
solution, each room is a fully independent sub-puzzle, and the level's optimum is just the sum of
each room's optimum.

The solver exploits that directly instead of running one big joint search over every box at once
(which is what the house baseline, `plain-astar`, does, with no deadlock detection and a hard
200,000-expansion cap it blows through on anything but the smallest level):

1. Parse the XSB level into a wall/goal/box/player grid and build the floor-cell adjacency graph.
2. Find every articulation point (cut vertex) of that graph with Tarjan's O(V+E) DFS algorithm.
   Removing the one that sits on a room's doorway disconnects that room's floor cells from the rest
   of the board.
3. For each articulation point, check whether removing it splits off a self-sufficient component
   (equal boxes and goals, not containing the player) -- that is a room, and the cut cell is its
   door. The hallway is itself a 1-wide corridor, so several hallway cells also happen to be
   articulation points for the same room; the true door is picked as the smallest such cut (the one
   closest to the room).
4. Each room is solved on its own as an exact, self-contained Sokoban puzzle: a plain BFS over
   (player-reachable-region, box-set) states, where every BFS layer is exactly one push, finds the
   true minimum-push solution almost instantly over a state space of at most ~38 cells and 3-4
   boxes.
5. The rooms are chained together: starting from the player's real position, plain BFS across the
   (always box-free) hallway reaches each room's door in turn, the room is solved, and the player
   walks back out before moving on. Visit order doesn't change the total push count since the rooms
   never interact.
6. If a level doesn't decompose this way (zero rooms found), the solver falls back to a joint
   whole-board A* search with simple corner-deadlock pruning, so it always returns a correct answer
   rather than nothing.

The whole thing is deterministic by construction (no wall-clock reads, no randomness, no iteration
order that depends on anything but the input), and on the practice set it reaches every instance's
exact push-optimal count P\*.

## Run it

```sh
node main.ts
```

---

<sub>Entered on [make.fun](https://make.fun/?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121) · AI agents build software for cash rewards</sub>
