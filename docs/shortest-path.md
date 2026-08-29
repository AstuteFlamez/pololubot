# Shortest path

The explorer records everywhere it went, dead ends included. This is how that
becomes the shortest route to the goal — and how the robot drives it.

**Source:** `logic/maze_logic.c` (`path_simplify`, `replay_next`),
`src/main.c` (`run_solved`, `run_replay`)
**Tests:** `tests/logic/test_maze_logic.c`, `tests/logic/sim_maze.c`

## There is no graph and no search

Worth saying up front, because the name suggests otherwise: nothing here
builds a graph, and no Dijkstra or flood fill runs anywhere in this firmware.
The shortest route is recovered by **arithmetic on the recorded string**.

That works because of the maze specification's one hard precondition: the maze
is a **tree**. In a tree there is exactly one simple path between any two
nodes, so the shortest route from start to goal is the *only* route from start
to goal that does not double back. Delete every double-back from what the
explorer drove and what remains is, necessarily, the shortest path. No search
required — the explorer already walked it, buried inside its detours.

## The one thing `'B'` can mean

`decide_left_hand()` emits `MOVE_BACK` at a dead end and **nowhere else**.
That is the license to rewrite: a `'B'` in a recorded path says the branch
before it was a lie.

And a dead-end detour returns the robot to the very junction it left. So it
changes nothing about **position** — only **heading**. Heading changes compose
additively mod 360, which turns the whole problem into addition.

## The arithmetic

Working clockwise in degrees (any consistent convention works; this one keeps
every number positive):

```
L = 270    S = 0    R = 90    B = 180
```

Driving `x`, then 180° at the dead end, then `y`, nets
`x + 180 + y (mod 360)`. The four residues are exactly the four moves' deltas,
so there is always **exactly one** replacement, and it leaves the robot on the
same outgoing corridor.

```
         y=L   y=S   y=R
   x=L    S     R     B          L B S = 270+180+0   = 450 = 90  -> R
   x=S    R     B     L          S B S = 0+180+0     = 180       -> B
   x=R    B     L     S          L B L = 270+180+270 = 720 = 0   -> S
```

Nine cases, one table — but written as arithmetic rather than nine hand-typed
branches, because the equivalences are the point. A net 270° right **is** a
90° left; spelling that out as cases invites one of them to be typed wrong.

```c
static int heading_delta(move_t m) {
    switch (m) {
    case MOVE_LEFT:  return 270;
    case MOVE_RIGHT: return 90;
    case MOVE_BACK:  return 180;
    default:         return 0;      // MOVE_STRAIGHT
    }
}

static move_t collapse(move_t x, move_t y) {
    switch ((heading_delta(x) + 180 + heading_delta(y)) % 360) {
    case 0:   return MOVE_STRAIGHT;
    case 90:  return MOVE_RIGHT;
    case 180: return MOVE_BACK;
    default:  return MOVE_LEFT;     // 270 — the only value left
    }
}
```

A `'B'` can also appear as `x` or `y` in an unsimplified path; the same
arithmetic covers it with delta 180.

## The loop

```c
void path_simplify(path_t *p)
{
    bool changed = true;
    while (changed) {
        changed = false;
        for (uint8_t i = 1; i + 1 < p->len; i++) {
            if (p->moves[i] != MOVE_BACK) { continue; }
            p->moves[i - 1] = collapse(p->moves[i - 1], p->moves[i + 1]);
            for (uint8_t k = i; k + 2 < p->len; k++) { p->moves[k] = p->moves[k + 2]; }
            p->len -= 2;
            changed = true;
            break;   // restart the scan
        }
    }
}
```

Each fold rewrites `moves[i-1]`, closes the two-slot gap, and shrinks `len` by
2.

**Why the scan restarts after every fold:** one collapse can create a *new*
triple to its **left**. `"LLBSBL"` needs more than one pass, and a scan that
kept walking forward would step over the triple it just created. The outer
loop repeats until a full scan changes nothing.

**Why the range is `[1, len-2]`:** a `'B'` at position 0 or `len-1` has no
triple around it, so it is left in place. That range is also what makes the
empty, one-move and two-move cases read nothing at all — they come out
untouched, no special-casing needed.

`len` only ever shrinks, so this cannot overflow. A path with no interior
`'B'` comes out byte-for-byte unchanged.

`p->overflow` is neither read nor cleared here: simplifying a truncated path
yields a shorter truncated path, and it is still not a route. Screening that
is the explore loop's job, at the moment the move is dropped.

## A worked example

```
explored:  L L B L L B S
                ^         LBL = 270+180+270 = 720 = 0    -> S
explored:  L S L B S
                  ^       LBS = 270+180+0   = 450 = 90   -> R
solved:    L S R
```

Seven moves become three, and the two dead ends are never entered again.
`run_solved()` puts both strings on one screen so the collapse is visible.

## Replay

```c
move_t replay_next(const path_t *p, uint8_t idx)
{
    return (idx < p->len) ? p->moves[idx] : MOVE_NONE;
}
```

**The boundary is the function.** A path of `len` moves steers exactly `len`
junctions, so `idx == len` is the first arrival with nothing left to say — and
`MOVE_NONE` there is how the replay loop learns the path is spent and *this
arrival must be the goal*. The comparison is strict `<` for that reason. Off
by one and the robot turns left **into** the goal instead of stopping on it.

There is no out-of-range index: every value is defined, including far past the
end.

Dispatch is on **junction arrivals**, never on distance or elapsed time. Tape
stretches and wheels slip, but junction #4 is junction #4 forever.

## What REPLAY does differently

`run_replay()` shares the follow → creep → classify skeleton with
`run_explore()`, and the duplication is **on purpose, not an oversight**: the
replay speed profile lives in that copy only — straights hot at
`SPEED_REPLAY`, then `arrival_brake()` sheds the momentum before the creep —
and the explorer's copy must never feel any of it. The profile itself is data
in `tuning.h` (`SPEED_REPLAY`, `SPEED_ARRIVAL`, `ARRIVAL_BRAKE_MM`), not
branches in the loop.

The other differences:

| | EXPLORE | REPLAY |
|---|---|---|
| move source | `decide_left_hand(j)` | `replay_next(&path_solved, jct_idx)` |
| verdict lane | `explore_arrival_verdict` | `replay_arrival_verdict` — the strict one |
| dead ends | ordinary; a `'B'` gets recorded | a **fault**: a simplified route never enters one |
| arrival brake | not called | called on every arrival |
| lap clock | none | `lap_start()` after the countdown, `lap_stop()` on the goal arrival |
| speed | `SPEED_EXPLORE` 1800 | `SPEED_REPLAY` 2500 |

The lap chain is a straight line and every link is load-bearing:
`lap_start()` as the wheels are about to move (so the countdown is not driving
time) → `lap_stop()` on the goal arrival, which is the **only** route into
`MODE_DONE` → `run_done()` reads `last`, `best` and the was-best flag → the
new-best melody. Break any link and DONE shows 0.00 s while the melody stays
silent. A faulted run exits *without* stopping the clock, so it records no
finish time at all.

## Faults that mean the plan and the world disagree

All three are replay-only, and each says the same thing in different words.

| Fault | Means |
|---|---|
| `"replay:dead end"` | a detour survived `path_simplify` — the route steered into a corridor end it should never have reached |
| `"goal too early"` | the goal patch, with moves still owed |
| `"replay:not goal"` | the path is spent and the classifier confidently read some *other* junction: wrong maze, drifted path, or a miscount upstream |

Distinguish `"replay:not goal"` from `"classify: NONE"` at the same point: the
first is a confident contradiction (go looking for a real disagreement), the
second is an honest refusal (rerun it).

## Edges the code defines and a real run rarely reaches

- **An empty solved path.** `run_replay()` screens `path_solved.len == 0`
  before preflight and puts up "run EXPLORE to the goal first" rather than
  driving.
- **A leading `'B'`.** The scan starts at index 1, so a `'B'` recorded at the
  very first junction survives simplification — correctly, since it is a real
  180 with no preceding move to fold it into. On replay, that arrival
  classifies as `DEAD_END` and faults `"replay:dead end"`. It takes a maze
  whose start corridor dead-ends before reaching any branch, which the
  specification permits but no sensible board does.
- **A trailing `'B'`.** Left in place for the same reason. An explore cannot
  produce one: after backing out of a dead end the robot always arrives at
  another junction, which records another move, and the run only ends on the
  goal patch.
- **An overflowed path.** Never reaches simplification — `run_explore()`
  faults `"path overflow"` at the 65th move (`PATH_MAX_MOVES` is 64).

## Test coverage

The unit suite pins every cell of the substitution table (including the
`'B'`-as-operand cases: `LBR`, `LBB`, `SBB`, `RBB`, `BBB`), the multi-pass
restart (`LLBSBL -> LB`), the untouched cases (empty path, `len` 1, a lone
`'B'`, a `len`-2 edge `'B'` that never folds, and a no-`'B'` path byte for
byte), and the `replay_next` fenceposts at `len-1`, `len` and far past the
end — including that past-the-end is the guard's answer, not the buffer's.

The simulator proves the whole chain on real mazes: it walks the topology,
derives both the raw and the simplified path, checks the simplified path
against a literal worked out by hand from the maze drawing, and then **drives
the derived route back to the goal** through the same replay logic the robot
runs. It does that at symbol level and at sensor level, through both
classifiers, on both test mazes.
