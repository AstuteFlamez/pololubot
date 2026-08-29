# Maze search

How the robot turns five numbers into a junction type, a junction type into a
move, and a sequence of moves into a route. This is `logic/maze_logic.c`, and
it is the half of the firmware that compiles on a laptop.

**Source:** `logic/maze_logic.c`, `logic/maze_logic.h`,
`logic/maze_logic_ref.c`
**Tests:** `tests/logic/` — 119 unit checks + a 44-assertion whole-maze
simulator, `make -C tests/logic`
**Driven by:** the EXPLORE loop in `src/main.c`

## Why this file has no SDK headers

`logic/` is plain C over plain structs: no register access, no timing, no
I/O, no static state. That is deliberate. The same source that
cross-compiles for the robot compiles with plain `gcc` under `tests/logic/`,
which is why the test suite can be exhaustive and still finish before you
have let go of the return key. The hardware side's whole job is to hand these
functions two clean snapshots per junction and execute the moves they hand
back.

The seam type is one struct:

```c
typedef struct { uint16_t s[5]; } sensor_snapshot_t;
// s[0] leftmost .. s[4] rightmost, calibrated 0 (white paper) .. 1000 (black tape)
```

## The maze this code is designed against

The algorithms hold only on a maze built to a specification, stated in full at
the top of `maze_logic.h`. Outside it, the junction signatures stop meaning
what the code assumes they mean. The parts that bite:

| Rule | Value | Why |
|---|---|---|
| **Topology** | a **tree** — no loops anywhere | this is a *precondition*, not a preference. The left-hand rule is only guaranteed to reach the goal in a loop-free maze; one loop and the same rule can orbit forever. Every added corridor must dead-end rather than reconnect two existing ones. |
| Tape | matte black, 19 mm, on white posterboard | 19 mm ≈ 2.5 sensors of dark: wide enough for a position estimate, narrow enough that the outer sensors stay on white while following — which is what lets "an outer sensor went dark" mean "side branch". **Matte**: glossy tape mirror-reflects at some angles and reads as bright paper. |
| Junction spacing | ≥ 150 mm between centers | the detector is blind for `BLIND_MM` after each junction; closer than creep + blind and two junctions merge into one event |
| Angles | true 90°, crossings run full width | the robot turns exactly 90° by gyro and then expects line under its nose. A 75° "90" leaves the new line half a sensor bar away. |
| Lead-in | ≥ 100 mm before the first junction | so calibration and the first blind window both finish with the robot line-locked |
| Goal patch | solid black, ≥ 50×50 mm (75×75 recommended), at a dead-end tip with a ≥ 150 mm straight approach | the goal signature is "still on broad black after creeping 45 mm in", so the patch has to outlast the creep. 50 mm leaves 5 mm of margin; 75 mm leaves 30. |
| Plane | one flat plane, no steps, no curled tape edges | the sensors ride 1–2 mm above the floor, so a lifted edge is terrain |

## Classification

```c
junction_t classify_junction(const sensor_snapshot_t *before,
                             const sensor_snapshot_t *at_center);
```

Nine possible answers:

```
JCT_LEFT_ONLY      <-      the line turns left
JCT_RIGHT_ONLY      ->     the line turns right
JCT_STRAIGHT_LEFT  ^<      straight continues, plus a left branch
JCT_STRAIGHT_RIGHT ^>      straight continues, plus a right branch
JCT_T              <>      left and right, no straight
JCT_CROSS          ^<>     all three ways open
JCT_DEAD_END               the line just stops
JCT_GOAL                   the big black finish patch
JCT_NONE                   cannot tell — a real answer, and a fault
```

### The two witnesses

Neither snapshot alone is sufficient, and each is the *only* witness to
something:

- **A side branch can only appear in `before`.** The branch's tape passes
  under an outer sensor for a few milliseconds while the robot sweeps across
  the junction; the per-sensor **maximum latch** keeps that darkness. Only the
  outermost sensors `s[0]` and `s[4]` carry it — a 19 mm entry line never
  reaches them, and its shoulders do not climb past the threshold.
- **A straight exit can only appear in `at_center`.** After the creep the bar
  sits *past* the junction, so anything dark under the middle of the bar there
  is line that **continues**.
- **GOAL vs CROSS is decided by `at_center` alone.** On approach the two
  blacken the whole bar identically. After the creep, a crossing leaves only
  its straight exit under the bar (center dark, outers white), while the goal
  patch is still solid black everywhere.
- **A dead end is the absence of everything** — paper in `before` *and* paper
  in `at_center`.

### The dark-count bands

`dark(v)` is `v >= JCT_DARK_THRESH` (600). Counting dark sensors in
`at_center` gives three bands:

```
 0-2 dark   (<= CROSS_MAX_DARK)   at most a straight exit -> read on for branches
 3 dark     (the gap)             fits NOTHING            -> JCT_NONE
 4-5 dark   (>= GOAL_MIN_DARK)    the goal patch          -> JCT_GOAL
```

A straight exit is **one** line under the bar: one sensor dead-on, or two when
the creep stops off-center and the line straddles a sensor gap — never three.

**The gap earns its refusal.** A 3-dark bar has two true stories: a goal patch
entered slightly skewed with two sensors hanging past its edge, or a
crossing's exit line plus a dirt-hot shoulder riding over the threshold. Any
named answer is wrong in one of those worlds, and never being wrong about
GOAL vs CROSS vs T is this classifier's one absolute — so the gap gets the
only verdict that is never wrong.

`CROSS_MAX_DARK` is **derived** as `GOAL_MIN_DARK - 2` rather than given its
own knob. Two independent knobs could be tuned until the gap closed, and a
classifier with no gap has no "cannot tell" left — only confident answers,
some of them wrong. The derivation also fails safe: drag `GOAL_MIN_DARK` down
to 3 and the edge starts refusing legitimate 2-dark off-center exits, which
costs stops and reruns, not lies. **A refusal costs one rerun; a misread
GOAL-vs-CROSS costs the maze.**

### The rest of the decision

```c
bool left     = dark(before->s[0]);
bool right    = dark(before->s[4]);
bool straight = dark(at_center->s[1]) || dark(at_center->s[2]) ||
                dark(at_center->s[3]);

if (left && right) return straight ? JCT_CROSS         : JCT_T;
if (left)          return straight ? JCT_STRAIGHT_LEFT : JCT_LEFT_ONLY;
if (right)         return straight ? JCT_STRAIGHT_RIGHT: JCT_RIGHT_ONLY;
return (center_dark == 0) ? JCT_DEAD_END : JCT_NONE;
```

Any of the center three counts as "straight" because a slightly off-center
stop parks the continuing line under `s[1]` or `s[3]` instead of `s[2]`.

The last line is the one to read twice. `DEAD_END` has to earn corroboration
from **both** witnesses. Classify is only reached this way after the loss path
confirmed white, so `before` testifies "the line ended", and `at_center` has
to agree with bare paper. *Any* dark sensor past the vanished line breaks the
agreement: center-dark is a line continuing where no junction detector fired
(a plain corridor never triggers one), outer-dark is tape where the sweep
swore there was nothing — a stray scrap, a neighbouring line's edge, a creep
that drifted off course. Certifying `DEAD_END` on contradicted evidence would
send the robot into a blind 180 on top of whatever is really down there.

`classify_junction` is **pure**: same inputs, same answer, no state carried
between calls.

## The left-hand rule

```c
move_t decide_left_hand(junction_t j);
```

One ranking — **L beats S beats R** — applied to whichever exits the junction
type has, with `B` reserved for a dead end's single legal move.

| Junction | Move |
|---|---|
| `LEFT_ONLY`, `STRAIGHT_LEFT`, `T`, `CROSS` | `MOVE_LEFT` |
| `STRAIGHT_RIGHT` | `MOVE_STRAIGHT` |
| `RIGHT_ONLY` | `MOVE_RIGHT` |
| `DEAD_END` | `MOVE_BACK` |
| `GOAL`, `NONE`, anything else | `MOVE_NONE` |

**Why hugging one wall reaches the goal at all:** the maze is a tree. Keeping
a hand on the wall of a tree traces its complete outline, walking every
corridor at most twice, so the walk must eventually cross every node — the
goal included. The guarantee lives in the maze's shape, not in this switch.

**Why `MOVE_NONE` and not `MOVE_BACK`** for the lost inputs: `MOVE_NONE` is
the one byte `execute_move()` will not perform, so a wiring bug that lets a
goal or a "cannot tell" through stops the robot on the spot with a fault up.
A 180 would be quieter, and that is exactly what is wrong with it — it would
drive a confused robot off the goal patch, or off the table, while looking
perfectly deliberate. Both lost inputs get the *same* refusal so no caller can
come to depend on the difference.

## Recording the path

```c
#define PATH_MAX_MOVES 64

typedef struct {
    char    moves[PATH_MAX_MOVES];   // NOT nul-terminated
    uint8_t len;                     // len is the truth
    bool    overflow;                // latches, never clears
} path_t;
```

Moves are stored as printable chars — `'L'`, `'R'`, `'S'`, `'B'` — so a
recorded path **is** a human-readable string on the OLED.

`path_record()` is the sole writer of `moves` and `len`, which makes its
bounds check the only place a buffer overrun could originate. Recording into a
full path drops the move, sets `overflow`, and changes nothing else. The flag
**latches** — nothing in the module ever clears it — because a route that lost
a move is no longer a route. `run_explore()` faults `"path overflow"` the
moment it sees the flag, which is what keeps an overflowed run from ever
reaching the goal and masquerading as a solve.

No validation of the move byte happens at record time; legality is screened at
execution time, by the arrival verdict.

## Arrival verdicts

Every arrival, exploring or replaying, ends in the same three-way question:
keep driving, declare the run won, or stop with a fault.

```c
arrival_verdict_t explore_arrival_verdict(junction_t j, move_t m);
arrival_verdict_t replay_arrival_verdict(junction_t j, move_t m);
```

These live in `logic/` for a reason worth recording. The firmware's copy of
the decision table used to sit in `main.c`, unreachable from the host tests,
while the simulator carried a second hand transcription of the same table.
Neither derived from the other, nothing compared them, and they had drifted in
**three cells**: the simulator walked on through a `JCT_NONE` the firmware
faulted on, merged two fault cases the firmware kept separate, and had no
overflow stop at all. A decision table transcribed twice is a decision table
that will disagree with itself. One copy, in the layer the tests can reach.

The verdict is a **code, never a message** — `logic/` stays free of strings
and I/O. The firmware maps codes to its 16-column fault screens
(`verdict_fault_text()` in `main.c`) and the simulator maps the same codes to
full-sentence diagnostics.

### The explore lane

```
j == JCT_GOAL              -> ARRIVE_SUCCESS         the arrival IS the answer
j == JCT_NONE              -> ARRIVE_FAULT_CLASSIFY_NONE
m not one of L/S/R/B       -> ARRIVE_FAULT_ILLEGAL_MOVE
otherwise                  -> ARRIVE_PROCEED
```

There is deliberately **no dead-end screen**: an explorer *meets* dead ends,
and turning around at one is how `'B'` enters the path in the first place. The
byte check catches the wiring bug `decide_left_hand` guards against — its
`MOVE_NONE` refusal leaking through to a live junction.

That this lane can only ever return those four codes is a **guarantee callers
may rely on**, stated in the header and not merely kept by the body: with no
spent-path column and no dead-end screen, the three replay-only codes have no
exit from this lane.

### The replay lane

Replay is the strict lane and owns the whole enum. Three columns, and the
**order matters**:

```
1. is the path spent?      m == MOVE_NONE, which replay_next() answers exactly at idx == len
2. what did the classifier say?
3. is the byte a move at all?

path spent:
    j == JCT_NONE          -> ARRIVE_FAULT_CLASSIFY_NONE    a refusal: rerun
    j != JCT_GOAL          -> ARRIVE_FAULT_NOT_GOAL         a confident contradiction: the plan is wrong
    otherwise              -> ARRIVE_SUCCESS
moves remain:
    j == JCT_GOAL          -> ARRIVE_FAULT_GOAL_EARLY
    j == JCT_NONE          -> ARRIVE_FAULT_CLASSIFY_NONE
    j == JCT_DEAD_END      -> ARRIVE_FAULT_DEAD_END         a detour survived path_simplify
    m illegal              -> ARRIVE_FAULT_ILLEGAL_MOVE
    otherwise              -> ARRIVE_PROCEED
```

Path first, then the junction screens, then the byte: the junction screens
describe where the robot **is**, while the byte only describes what it was
about to do — so a contradicted arrival keeps its own fault even when the byte
is rot. That precedence is pinned cell by cell in the tests, because it is the
kind of ordering an "obvious cleanup" reshuffles by accident.

The two path-spent faults are kept **distinct on purpose**: a classifier
refusal is a rerun, while a confident read of some other junction means the
maze, the route, or the junction count is wrong. One merged message would send
a debugger looking for the wrong bug.

## The EXPLORE loop

`run_explore()` in `src/main.c`, one junction per turn:

```
follow -> creep -> classify -> decide -> verdict -> record -> announce -> move
```

Record **before** moving, so a fault mid-turn still leaves an honest record of
what was decided — but **after** the verdict, so a byte the robot cannot
execute never enters the record at all.

The loop's own screens: `on_junction()` draws the classification, the decided
move and the path so far, then `THINK_PAUSE_MS` (400 ms) of visible "thinking"
before the turn. That pause is cosmetic, and it is pure lap time — but EXPLORE
is not timed. The lap timer records **replay runs only**, because an explore
that reached the goal would seed `best` with its cautious stroll and the next
replay would always beat it, for a guaranteed "NEW!" and an unearned melody.

## The reference classifier

`logic/maze_logic_ref.c` is a second, deliberately **crude** classifier with
the same signature: one hard-coded threshold (`REF_DARK` 600), no tuning
knobs, no stated GOAL-vs-CROSS margin, and no refusal band beyond a single
case. That is the point — everything the real classifier adds is visible as
the diff between the two files.

It serves two purposes. It is a **fallback lane**: comment out
`USE_MY_CLASSIFIER` in `include/tuning.h` and EXPLORE runs on it instead, same
decision layer underneath, only the evidence-to-symbol step swapped. And it is
a **cross-check baseline** for differencing when the real classifier
misbehaves on the floor.

`REF_DARK` is file-local on purpose — no header, no knob — because being
readable end to end in one sitting is that file's whole job. The simulator
needs the number anyway, and `tests/logic/Makefile` lifts the literal out of
the source with `sed` and passes it in as a `-D`, with an `#if` in
`sim_maze.c` that **fails the build** if the mirror ever drifts. Before that
line existed, moving `REF_DARK` from 600 to 700 left every test green.

## Test coverage

```sh
make -C tests/logic     # 119 unit checks + a 44-assertion maze simulator
```

`test_maze_logic.c` exercises each function alone against hand-built
snapshots. `sim_maze.c` proves them **together** by walking whole mazes
through the real pipeline — classify → decide → record → simplify → replay —
at two levels of realism:

- **Symbol level** hands `decide_left_hand()` ground-truth junction types
  straight out of a topology table. A failure here is in decide / record /
  simplify / replay, not in sensing.
- **Sensor level** synthesizes realistic `before`/`at_center` snapshots and
  pushes them through a classifier first, exactly as `main.c` does. It runs
  **twice** — once through each classifier — so the unselected lane cannot rot
  unnoticed. Symbol passing while sensor fails localizes the bug to
  classification.

The simulator also carries a **marginal palette**, where the tape has worn and
the contrast sits close to the classification threshold, so the checks prove
the bands hold at their edges and not just in the middle.

Two habits keep the suite honest. The maze is a **table**, not code, so adding
a maze is data: the second maze arrived as one more table and one more suite
call, with the walker, replayer and every assertion reused untouched. And the
walk **derives** the explore and solved paths from the topology and then
compares against a literal worked out by hand from the drawing — a hard-coded
expected string can only prove the code agrees with whoever typed it, while
deriving it cross-checks the topology encoding, the hand walk, and the code at
once.

Every check in both suites has been **observed failing**. The earliest were
written against stubs and were red until `maze_logic.c` was implemented
function by function; each later hardening check was verified by deliberately
breaking the function it covers and confirming it went red. A test written
after the code proves nothing by passing.
