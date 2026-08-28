// maze_logic.c — THE BRAIN: the seven functions the robot thinks with.
//
// This file is pure logic: no SDK includes, no I/O, no timing. Keep it that
// way — it is what lets host_tests/ prove this code on a laptop (unit tests
// AND the whole-maze simulator) before a single wheel turns.
//
// Each comment block below states WHAT the function guarantees — its
// contract, invariants, and the edge cases that bite — and the code answers
// the contract's questions. Read the contract first, then the body.

#include "maze_logic.h"
#include "tuning.h"   // JCT_DARK_THRESH and friends — this robot's numbers

// ---------------------------------------------------------------------------
// classify_junction
//
// Contract: given the two snapshots (see maze_logic.h for exactly when each
// was taken), return the junction type. Must never be wrong about GOAL vs
// CROSS vs T — a wrong answer here poisons everything downstream.
//
// The evidence, snapshot by snapshot:
//   - A side branch can only show up in `before`: the branch's tape passes
//     under the bar while the robot sweeps across the junction, and the
//     per-sensor MAXIMUM latch keeps that darkness even though the branch
//     was under a sensor for just a few milliseconds. The OUTERMOST sensors
//     (s[0] left, s[4] right) carry that evidence — the entry line itself
//     never reaches them, and its shoulders don't climb past the threshold.
//   - A straight exit can only show up in `at_center`: after the creep the
//     bar sits PAST the junction, so anything dark under the middle of the
//     bar there is line that CONTINUES.
//   - GOAL vs CROSS is decided by `at_center` ONLY. On approach both
//     blacken the whole bar identically — but after the creep a crossing
//     leaves just its straight exit under the bar (center dark, outers
//     white), while the goal patch is still solid black everywhere.
//   - A dead end is the absence of everything: the line ended, so the drive
//     layer hands us paper in `before` AND paper in `at_center` — both.
//     Tape anywhere after the creep contradicts "the line ended", and a
//     contradiction is JCT_NONE, never a certified dead end.
//   - Evidence that fits nothing gets the honest answer: JCT_NONE. The
//     robot treats it as a safe stop — guessing is what drives robots off
//     tables. The sharpest case is a dark COUNT that falls in the gap
//     between the crossing and goal signatures — the margin is stated at
//     CROSS_MAX_DARK below.
// ---------------------------------------------------------------------------

// One calibrated reading counts as "dark" at or above this line. Both
// numbers this function classifies with — JCT_DARK_THRESH here and
// GOAL_MIN_DARK below — are tuning.h knobs, because they are properties
// of YOUR tape, YOUR calibration, and YOUR goal patch, not of this
// algorithm. (Why 4-of-5 for the goal is argued at the knob.)
static bool dark(uint16_t v) { return v >= JCT_DARK_THRESH; }

// The most at_center sensors a straight exit can legitimately darken.
// The exit is ONE line under the bar: one sensor dead-on, or two when
// the creep stops off-center and the line straddles a sensor gap —
// never three (tape width vs the bar's sensor pitch; even the
// simulator's worn-tape world keeps a real line's shoulders below
// JCT_DARK_THRESH). Between this and GOAL_MIN_DARK the GOAL-vs-CROSS
// margin is STATED (counts shown for the shipped GOAL_MIN_DARK = 4):
//
//   0–2 dark (<= CROSS_MAX_DARK)  at most a straight exit — read on
//                                 for the branch evidence below
//   3 dark   (the gap)            fits NOTHING             -> JCT_NONE
//   4–5 dark (>= GOAL_MIN_DARK)   the goal patch           -> JCT_GOAL
//
// The gap earns the refusal because a 3-dark bar has TWO true stories:
// a goal patch entered slightly skewed, two sensors hanging past its
// edge — or a crossing's exit line plus a dirt-hot shoulder riding over
// the threshold. Any named answer is wrong in one of those worlds, and
// "never wrong on GOAL vs CROSS vs T" is the contract's one absolute,
// so the gap gets the only verdict that is never wrong: "I can't tell".
//
// Why DERIVED from GOAL_MIN_DARK instead of its own tuning.h knob: the
// gap between the signatures is what keeps this classifier honest, and
// GOAL_MIN_DARK - 2 keeps it exactly one count wide wherever a hardware
// session moves GOAL_MIN_DARK. Two independent knobs could be tuned
// until the gap closed — and a classifier with no gap has no "I can't
// tell" left, only confident answers, some of them wrong. The
// derivation also fails SAFE: drag GOAL_MIN_DARK down to 3 and the
// edge starts refusing legitimate 2-dark off-center exits — more
// stops, more reruns, zero lies. Retuning can trade availability for
// honesty here, never the reverse: a refusal costs one rerun; a
// misread GOAL-vs-CROSS costs the maze.
#define CROSS_MAX_DARK  (GOAL_MIN_DARK - 2)

junction_t classify_junction(const sensor_snapshot_t *before,
                             const sensor_snapshot_t *at_center)
{
    // The dark COUNT first: for GOAL vs CROSS, `before` is useless (on
    // approach both blacken the whole bar identically), so at_center's
    // count is the entire case file — the three bands stated at
    // CROSS_MAX_DARK.
    int center_dark = 0;
    for (int i = 0; i < 5; i++) {
        if (dark(at_center->s[i])) { center_dark++; }
    }
    if (center_dark >= GOAL_MIN_DARK) { return JCT_GOAL; }

    // The gap: more dark than any straight exit can leave, less black
    // than the goal guarantees. Both remaining candidates come with a
    // world where they are WRONG — refuse instead of guessing, and let
    // main.c stop the robot with the fault screen up.
    if (center_dark > CROSS_MAX_DARK) { return JCT_NONE; }

    // Side-branch evidence: the latched sweep maxima at the bar's ends.
    bool left  = dark(before->s[0]);
    bool right = dark(before->s[4]);

    // Straight-exit evidence: line under the middle of the bar after the
    // creep. Any of the center three counts — a slightly off-center stop
    // parks the continuing line under s[1] or s[3] instead of s[2].
    bool straight = dark(at_center->s[1]) || dark(at_center->s[2]) ||
                    dark(at_center->s[3]);

    if (left && right) { return straight ? JCT_CROSS          : JCT_T; }
    if (left)          { return straight ? JCT_STRAIGHT_LEFT  : JCT_LEFT_ONLY; }
    if (right)         { return straight ? JCT_STRAIGHT_RIGHT : JCT_RIGHT_ONLY; }

    // No branches, no goal. The one verdict left is DEAD_END — and the
    // contract makes it earn corroboration from BOTH witnesses: classify
    // is only called this way after the loss path confirmed white, so
    // `before` testifies "the line ended", and at_center has to agree
    // with bare paper. ANY dark sensor past the vanished line breaks the
    // agreement: center-dark is a line continuing where no junction
    // detector fired (a plain corridor never triggers one), outer-dark
    // is tape where the sweep swore there was nothing — a stray scrap, a
    // neighboring line's edge, a creep that drifted off course.
    // Certifying DEAD_END on contradicted evidence would send the robot
    // into a blind 180 on top of whatever is really down there. Refuse.
    return (center_dark == 0) ? JCT_DEAD_END : JCT_NONE;
}

// ---------------------------------------------------------------------------
// decide_left_hand
//
// Contract: the classic left-hand rule. An explorer hugging the left wall
// always takes the leftmost available exit. The whole rule is one ranking —
// L beats S beats R — applied to whichever exits this junction type has,
// and B only when no exit exists at all (a dead end's single legal move).
//
// Why hugging one wall finds the goal AT ALL: the maze is a TREE — no
// loops, MAZE_BUILD.md's one unbreakable layout rule. Keep a hand on the
// wall of a tree and you trace its complete outline, walking every
// corridor at most twice, so the walk must eventually cross every node —
// the goal included. Add a single loop and the same rule can orbit it
// forever: the guarantee lives in the maze's shape, not in this switch.
//
// GOAL and NONE never reach this function (main.c handles both first), but
// the contract demands something safe anyway. The safe answer here is
// MOVE_NONE — refuse to move. It is the one move execute_move() will NOT
// perform (it rejects it as a fault), so a future wiring bug that lets a
// goal or an "I can't tell" leak through stops the robot on the spot with
// the fault screen up. MOVE_BACK would be quieter — and that is exactly
// what's wrong with it: a 180 would drive a confused robot off the goal
// patch (or off the table) while looking perfectly deliberate. Both lost
// inputs get the SAME refusal, so no caller can grow to depend on the
// difference.
// ---------------------------------------------------------------------------
move_t decide_left_hand(junction_t j)
{
    switch (j) {
    // A left exit exists — leftmost by definition, take it.
    case JCT_LEFT_ONLY:
    case JCT_STRAIGHT_LEFT:
    case JCT_T:
    case JCT_CROSS:          return MOVE_LEFT;
    // No left arm: straight outranks right.
    case JCT_STRAIGHT_RIGHT: return MOVE_STRAIGHT;
    // One exit; take it.
    case JCT_RIGHT_ONLY:     return MOVE_RIGHT;
    // No exits at all: turn around is the only legal move.
    case JCT_DEAD_END:       return MOVE_BACK;
    // GOAL, NONE, or garbage: refuse to move (see the block above).
    case JCT_GOAL:
    case JCT_NONE:
    default:                 return MOVE_NONE;
    }
}

// ---------------------------------------------------------------------------
// path_record
//
// Contract: append m to p->moves. The path is a fixed 64-byte buffer with
// no terminator; p->len is the single source of truth.
//
// Invariants (host_tests enforces them):
//   - p->len never exceeds PATH_MAX_MOVES. Never. This function is the
//     only writer, so this bounds check is the only place a buffer overrun
//     could be born — and the place it is made impossible.
//   - Recording into a full path sets p->overflow and changes nothing else.
//   - Once overflow is set it stays set: the latch only ever writes `true`,
//     so no later append can quietly un-poison a path that already dropped
//     a move. A truncated route must never be trusted for replay — the
//     explore loop checks this flag and faults rather than "solving".
// ---------------------------------------------------------------------------
void path_record(path_t *p, move_t m)
{
    if (p->len >= PATH_MAX_MOVES) {
        p->overflow = true;   // the move is LOST — remember that forever
        return;
    }
    p->moves[p->len++] = m;
}

// ---------------------------------------------------------------------------
// path_simplify
//
// Contract: rewrite p in place so it is the shortest route with the same
// start and end. Every 'B' in a recorded path marks a dead-end detour —
// it can mean nothing else, because decide_left_hand emits MOVE_BACK only
// at a dead end — and that is the license to collapse: a 'B' says "that
// branch was a lie", so any triple  (x, 'B', y)  rewrites history into
// the ONE move the robot would have made had it known.
//
// The paper-work behind the collapse: each move changes the robot's compass
// heading by a fixed amount — L is 270° clockwise (= 90° left), S is 0,
// R is 90, B is 180. Driving x, then 180° at the dead end, then y nets
// x + 180 + y (mod 360) of heading change, and whichever single move
// produces that same net change is the whole detour's replacement.
// (Net 270° right IS 90° left — the mod-360 arithmetic below is exactly
// the "careful" the contract warns about.)
//
// Invariants & edge cases (host_tests enforces them):
//   - One collapse can create a NEW triple ("LLBSBL" needs more than one
//     pass). The outer loop repeats until a full scan changes nothing.
//   - 'B' at position 0 or len-1 has no triple around it — the scan range
//     [1, len-2] never even looks at it.
//   - A path with no 'B' comes out byte-for-byte unchanged (no triple ever
//     matches, so nothing is written).
//   - Empty path, len 1, len 2: the scan range is empty — no reads outside
//     [0, len).
//   - Every collapse shrinks len by 2, so the path never grows and
//     overflow cannot happen here.
// ---------------------------------------------------------------------------

// Heading change of one move, in degrees CLOCKWISE (any consistent
// convention works — this one keeps every number positive).
static int heading_delta(move_t m)
{
    switch (m) {
    case MOVE_LEFT:  return 270;
    case MOVE_RIGHT: return 90;
    case MOVE_BACK:  return 180;
    default:         return 0;    // MOVE_STRAIGHT
    }
}

// The single move whose heading change equals x, then 180°, then y.
static move_t collapse(move_t x, move_t y)
{
    switch ((heading_delta(x) + 180 + heading_delta(y)) % 360) {
    case 0:   return MOVE_STRAIGHT;   // e.g. LBL: 270+180+270 = 720 ≡ 0
    case 90:  return MOVE_RIGHT;      // e.g. LBS: 270+180+0   = 450 ≡ 90
    case 180: return MOVE_BACK;       // e.g. SBS: the detour was in-line
    default:  return MOVE_LEFT;       // 270 — the only value left
    }
}

void path_simplify(path_t *p)
{
    bool changed = true;
    while (changed) {
        changed = false;
        for (uint8_t i = 1; i + 1 < p->len; i++) {
            if (p->moves[i] != MOVE_BACK) { continue; }
            // Triple found: fold it into moves[i-1], close the 2-slot gap.
            p->moves[i - 1] = collapse(p->moves[i - 1], p->moves[i + 1]);
            for (uint8_t k = i; k + 2 < p->len; k++) {
                p->moves[k] = p->moves[k + 2];
            }
            p->len -= 2;
            changed = true;
            break;   // restart the scan — the fold may have made a new
                     // triple LEFT of where we stand
        }
    }
}

// ---------------------------------------------------------------------------
// replay_next
//
// Contract: the move for junction number idx (0-based) during a replay run,
// or MOVE_NONE when idx is past the end — that is how the replay loop knows
// the next junction should be the goal.
//
// The boundary IS the function: a path of len moves steers exactly len
// junctions, so idx == len is the first arrival with nothing left to say —
// strict `<`, never `<=`. Off-by-one here and the robot turns left INTO
// the goal instead of stopping on it (host_tests pins idx == len-1, len,
// and far past).
// ---------------------------------------------------------------------------
move_t replay_next(const path_t *p, uint8_t idx)
{
    return (idx < p->len) ? p->moves[idx] : MOVE_NONE;
}

// ---------------------------------------------------------------------------
// replay_arrival_verdict / explore_arrival_verdict
//
// Contract: given what the classifier saw and the move byte the decision
// layer produced for THIS arrival, return the one verdict the run loop must
// act on — proceed, success, or a named fault. Pure: no I/O, no state, no
// memory of previous arrivals. Same inputs, same verdict, forever.
//
// Why these live here and not in the run loops (audit AUDIT-18): the
// firmware's copy sat in main.c, unreachable from host_tests, and the
// simulator carried a second hand transcription of the same table. Neither
// derived from the other, nothing compared them, and they had drifted in
// three cells — the simulator walked on through a JCT_NONE that the
// firmware faulted on, merged two fault cases the firmware had split, and
// had no overflow stop at all. A decision table transcribed twice is a
// decision table that will disagree with itself; the fix is one copy, in
// the layer the tests can reach.
//
// THE THREE COLUMNS, and nothing else:
//   1. Is the path spent?   Encoded in the move byte: replay_next()
//      answers MOVE_NONE exactly at idx == len, which is the loop's only
//      way of learning "the next arrival must BE the goal".
//   2. What did the classifier say?   The junction kind.
//   3. Is the byte a move at all?   L/S/R/B, or data rot.
//
// ORDER MATTERS, and the order is: path first, then the junction screens,
// then the byte. The junction screens describe where the robot IS; the byte
// only describes what it was about to do — so a contradicted arrival keeps
// its own fault even when the byte is rot. That precedence is pinned cell
// by cell in host_tests/test_maze_logic.c, because it is the kind of
// ordering an "obvious cleanup" reshuffles by accident.
//
// The verdict is a CODE. Turning it into words is the caller's job (main.c
// has 16 columns of OLED, sim_maze.c has a terminal), which is what keeps
// this file free of strings, printf, and screens — invariant I1/DS-a.
// ---------------------------------------------------------------------------

// The four bytes the seams can actually execute. Anything else — a
// corrupted path byte, a decision layer that grew a fifth answer,
// MOVE_NONE arriving past its handled boundary — is data rot, not a
// maneuver. Not exported on purpose: legality is one COLUMN of the table
// above, never a second opinion a caller can consult separately (two
// screens in two places is the duplication this whole extraction closes).
static bool move_is_legal(move_t m)
{
    return m == MOVE_STRAIGHT || m == MOVE_LEFT ||
           m == MOVE_RIGHT    || m == MOVE_BACK;
}

arrival_verdict_t replay_arrival_verdict(junction_t j, move_t m)
{
    if (m == MOVE_NONE) {
        // The path is spent, so THIS arrival must be the goal — the exact
        // boundary replay_next's contract pins at idx == len. Two distinct
        // ways to miss it, kept distinct: a classifier REFUSAL ("I can't
        // tell") is a rerun, while a CONFIDENT read of some other junction
        // means the maze, the route, or the junction count is wrong. One
        // merged message would send a debugger looking for the wrong bug.
        if (j == JCT_NONE) { return ARRIVE_FAULT_CLASSIFY_NONE; }
        if (j != JCT_GOAL) { return ARRIVE_FAULT_NOT_GOAL; }
        return ARRIVE_SUCCESS;
    }

    // Moves remain, so this arrival must be an ordinary junction. A
    // classifier verdict that contradicts that expectation is a fault — a
    // replay that guesses is just exploring, badly.
    if (j == JCT_GOAL)     { return ARRIVE_FAULT_GOAL_EARLY; }
    if (j == JCT_NONE)     { return ARRIVE_FAULT_CLASSIFY_NONE; }
    // 'B' is a legal byte, but a SIMPLIFIED route never enters a dead end:
    // meeting one means a detour survived path_simplify.
    if (j == JCT_DEAD_END) { return ARRIVE_FAULT_DEAD_END; }

    if (!move_is_legal(m)) { return ARRIVE_FAULT_ILLEGAL_MOVE; }
    return ARRIVE_PROCEED;
}

arrival_verdict_t explore_arrival_verdict(junction_t j, move_t m)
{
    // The goal ends the run before anything is recorded — the arrival IS
    // the answer, and GOAL is never a move in the path.
    if (j == JCT_GOAL) { return ARRIVE_SUCCESS; }
    // An honest "I can't tell" — stop rather than guess.
    if (j == JCT_NONE) { return ARRIVE_FAULT_CLASSIFY_NONE; }

    // No dead-end screen here, on purpose: an explorer MEETS dead ends, and
    // turning around at one is how 'B' enters the path in the first place.
    // What is left to check is the byte itself — which catches the wiring
    // bug decide_left_hand's contract warns about, its MOVE_NONE refusal
    // leaking through to a live junction.
    if (!move_is_legal(m)) { return ARRIVE_FAULT_ILLEGAL_MOVE; }
    return ARRIVE_PROCEED;
}
