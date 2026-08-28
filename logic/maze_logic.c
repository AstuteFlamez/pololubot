// maze_logic.c — implementation of the maze reasoning declared in
// maze_logic.h: junction classification, the left-hand rule, bounded path
// recording, path simplification, replay, and the arrival verdicts.
//
// Pure logic: no SDK includes, no I/O, no timing, no static state. That is
// what lets tests/logic/ build this exact source with plain gcc and prove it
// — unit tests and a whole-maze simulator — before any of it drives a motor.
//
// The contracts live in maze_logic.h. What follows is why the code is shaped
// the way it is: which evidence each decision rests on, where the numbers
// come from, and what the defensive branches are defending against.

#include "maze_logic.h"
#include "tuning.h"   // JCT_DARK_THRESH and friends — this robot's numbers

// ---------------------------------------------------------------------------
// classify_junction — the evidence and the thresholds
//
// The two snapshots carry different evidence, and neither one is sufficient:
//   - A side branch can only show up in `before`. The branch's tape passes
//     under the bar while the robot sweeps across the junction, and the
//     per-sensor MAXIMUM latch keeps that darkness even though the branch was
//     under a sensor for only a few milliseconds. The OUTERMOST sensors
//     (s[0] left, s[4] right) carry it: a 19 mm entry line never reaches
//     them, and its shoulders do not climb past the threshold.
//   - A straight exit can only show up in `at_center`. After the creep the
//     bar sits PAST the junction, so anything dark under the middle of the
//     bar there is line that CONTINUES.
//   - GOAL vs CROSS is decided by `at_center` alone. On approach the two
//     blacken the whole bar identically; after the creep a crossing leaves
//     only its straight exit under the bar (center dark, outers white), while
//     the goal patch is still solid black everywhere.
//   - A dead end is the absence of everything: paper in `before` AND paper in
//     `at_center`. Tape anywhere after the creep contradicts "the line
//     ended", and a contradiction is JCT_NONE, never a certified dead end.
// ---------------------------------------------------------------------------

// One calibrated reading counts as "dark" at or above this line. Both numbers
// this file classifies with — JCT_DARK_THRESH here and GOAL_MIN_DARK below —
// live in tuning.h, because they are properties of a particular tape,
// calibration and goal patch, not of this algorithm.
static bool dark(uint16_t v) { return v >= JCT_DARK_THRESH; }

// The most at_center sensors a straight exit can legitimately darken. The
// exit is ONE line under the bar: one sensor dead-on, or two when the creep
// stops off-center and the line straddles a sensor gap — never three (19 mm
// of tape against the bar's sensor pitch; even the simulator's worn-tape
// world keeps a real line's shoulders below JCT_DARK_THRESH). Between this
// and GOAL_MIN_DARK the GOAL-vs-CROSS margin is explicit (counts shown for
// the shipped GOAL_MIN_DARK = 4):
//
//   0–2 dark (<= CROSS_MAX_DARK)  at most a straight exit — read on for the
//                                 branch evidence
//   3 dark   (the gap)            fits NOTHING             -> JCT_NONE
//   4–5 dark (>= GOAL_MIN_DARK)   the goal patch           -> JCT_GOAL
//
// The gap earns its refusal because a 3-dark bar has TWO true stories: a goal
// patch entered slightly skewed with two sensors hanging past its edge, or a
// crossing's exit line plus a dirt-hot shoulder riding over the threshold.
// Any named answer is wrong in one of those worlds, and never being wrong
// about GOAL vs CROSS vs T is this classifier's one absolute, so the gap gets
// the only verdict that is never wrong: JCT_NONE.
//
// Derived from GOAL_MIN_DARK rather than given its own tuning.h knob: the gap
// is what keeps the classifier honest, and GOAL_MIN_DARK - 2 holds it exactly
// one count wide wherever a hardware session moves GOAL_MIN_DARK. Two
// independent knobs could be tuned until the gap closed, and a classifier
// with no gap has no "cannot tell" left — only confident answers, some of
// them wrong. The derivation also fails SAFE: drag GOAL_MIN_DARK down to 3
// and the edge starts refusing legitimate 2-dark off-center exits, which
// costs stops and reruns, not lies. Retuning can trade availability for
// honesty here, never the reverse: a refusal costs one rerun, a misread
// GOAL-vs-CROSS costs the maze.
#define CROSS_MAX_DARK  (GOAL_MIN_DARK - 2)

junction_t classify_junction(const sensor_snapshot_t *before,
                             const sensor_snapshot_t *at_center)
{
    // The dark COUNT first: for GOAL vs CROSS, `before` is useless (on
    // approach both blacken the whole bar identically), so at_center's count
    // is the entire case file — the three bands stated at CROSS_MAX_DARK.
    int center_dark = 0;
    for (int i = 0; i < 5; i++) {
        if (dark(at_center->s[i])) { center_dark++; }
    }
    if (center_dark >= GOAL_MIN_DARK) { return JCT_GOAL; }

    // The gap: more dark than any straight exit can leave, less black than
    // the goal guarantees. Both remaining candidates come with a world where
    // they are WRONG — refuse instead of guessing, and let the run loop stop
    // the robot with the fault screen up.
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

    // No branches, no goal. The one verdict left is DEAD_END, and it has to
    // earn corroboration from BOTH witnesses: classify is only reached this
    // way after the line-loss path confirmed white, so `before` testifies
    // "the line ended", and at_center has to agree with bare paper. ANY dark
    // sensor past the vanished line breaks the agreement — center-dark is a
    // line continuing where no junction detector fired (a plain corridor
    // never triggers one), outer-dark is tape where the sweep swore there was
    // nothing: a stray scrap, a neighboring line's edge, a creep that drifted
    // off course. Certifying DEAD_END on contradicted evidence would send the
    // robot into a blind 180 on top of whatever is really down there.
    return (center_dark == 0) ? JCT_DEAD_END : JCT_NONE;
}

// ---------------------------------------------------------------------------
// decide_left_hand — the ranking, and why it terminates
//
// The whole rule is one ranking, L beats S beats R, applied to whichever
// exits the junction type has, with B reserved for a dead end's single legal
// move.
//
// Why hugging one wall reaches the goal at all: the maze is a tree (the
// topology rule in maze_logic.h). Keeping a hand on the wall of a tree traces
// its complete outline, walking every corridor at most twice, so the walk
// must eventually cross every node — the goal included. Add one loop and the
// same rule can orbit it forever; the guarantee lives in the maze's shape,
// not in this switch.
//
// MOVE_NONE rather than MOVE_BACK for the lost inputs: MOVE_NONE is the one
// byte execute_move() will not perform, so a wiring bug that lets a goal or
// a "cannot tell" through stops the robot on the spot with a fault up. A 180
// would be quieter, and that is what is wrong with it — it would drive a
// confused robot off the goal patch, or off the table, while looking
// perfectly deliberate. Both lost inputs get the SAME refusal so that no
// caller can come to depend on the difference.
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

// This bounds check is the only thing standing between a 64-byte buffer and a
// path that keeps growing, because this function is the sole writer of
// p->moves and p->len. The overflow latch only ever writes `true`, so no
// later append can quietly un-poison a path that already dropped a move.
void path_record(path_t *p, move_t m)
{
    if (p->len >= PATH_MAX_MOVES) {
        p->overflow = true;   // the move is LOST — remember that forever
        return;
    }
    p->moves[p->len++] = m;
}

// ---------------------------------------------------------------------------
// path_simplify — the substitution table and why the arithmetic is right
//
// A 'B' can only mean a dead-end detour, because decide_left_hand emits
// MOVE_BACK at nothing else. That is the license to rewrite: a 'B' says the
// branch before it was a lie, so the triple (x, 'B', y) becomes the one move
// the robot would have made had it known.
//
// The detour returns the robot to the very junction it left, so it changes
// nothing about position — only heading. Heading changes compose additively
// mod 360, and the four residues are exactly the four moves' deltas, so there
// is always exactly one replacement and it leaves the robot on the same
// outgoing corridor. Working clockwise in degrees: L = 270, S = 0, R = 90,
// B = 180. Driving x, then 180° at the dead end, then y nets
// x + 180 + y (mod 360), which gives the whole table:
//
//          y=L   y=S   y=R
//    x=L    S     R     B        e.g. L B S = 270+180+0 = 450 ≡ 90 -> R
//    x=S    R     B     L        e.g. S B S = 0+180+0   = 180      -> B
//    x=R    B     L     S        e.g. L B L = 270+180+270 = 720 ≡ 0 -> S
//
// (Net 270° right IS 90° left — that equivalence is the whole reason the
// substitutions are done in arithmetic rather than as nine hand-written
// cases.) A 'B' can also appear as x or y in an unsimplified path; the same
// arithmetic covers it with delta 180.
//
// One collapse can create a NEW triple to its left ("LLBSBL" needs more than
// one pass), so the scan restarts after every fold and the outer loop repeats
// until a full scan changes nothing. The scan range [1, len-2] is what keeps
// a leading or trailing 'B' — which has no triple around it — untouched, and
// what makes the empty, one-move and two-move cases read nothing at all.
// ---------------------------------------------------------------------------

// Heading change of one move, in degrees CLOCKWISE (any consistent convention
// works — this one keeps every number positive).
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
                     // triple LEFT of where the scan stands
        }
    }
}

// The boundary IS the function: idx == len is the first arrival with nothing
// left to say, so the comparison is strict `<`, never `<=`. Off by one here
// and the robot turns left INTO the goal instead of stopping on it.
move_t replay_next(const path_t *p, uint8_t idx)
{
    return (idx < p->len) ? p->moves[idx] : MOVE_NONE;
}

// ---------------------------------------------------------------------------
// replay_arrival_verdict / explore_arrival_verdict
//
// Why these live here instead of in the run loops: the firmware's copy of the
// decision table sat in main.c, unreachable from the host tests, while the
// simulator carried a second hand transcription of the same table. Neither
// derived from the other, nothing compared them, and they had drifted in
// three cells — the simulator walked on through a JCT_NONE the firmware
// faulted on, merged two fault cases the firmware kept separate, and had no
// overflow stop at all. A decision table transcribed twice is a decision
// table that will disagree with itself. One copy, in the layer the tests can
// reach; tests/logic/test_maze_logic.c pins every cell.
//
// THE THREE COLUMNS, and nothing else:
//   1. Is the path spent?  Encoded in the move byte: replay_next() answers
//      MOVE_NONE exactly at idx == len, which is the loop's only way of
//      learning "the next arrival must BE the goal".
//   2. What did the classifier say?  The junction kind.
//   3. Is the byte a move at all?  L/S/R/B, or data rot.
//
// ORDER MATTERS, and the order is: path first, then the junction screens,
// then the byte. The junction screens describe where the robot IS; the byte
// only describes what it was about to do — so a contradicted arrival keeps
// its own fault even when the byte is rot. That precedence is pinned cell by
// cell in the tests, because it is the kind of ordering an "obvious cleanup"
// reshuffles by accident.
// ---------------------------------------------------------------------------

// The four bytes the motion layer can actually execute. Anything else — a
// corrupted path byte, a decision layer that grew a fifth answer, MOVE_NONE
// arriving past its handled boundary — is data rot, not a maneuver. Kept
// file-local on purpose: legality is one COLUMN of the table above, never a
// second opinion a caller can consult separately.
static bool move_is_legal(move_t m)
{
    return m == MOVE_STRAIGHT || m == MOVE_LEFT ||
           m == MOVE_RIGHT    || m == MOVE_BACK;
}

arrival_verdict_t replay_arrival_verdict(junction_t j, move_t m)
{
    if (m == MOVE_NONE) {
        // The path is spent, so THIS arrival must be the goal — the exact
        // boundary replay_next() pins at idx == len. Two distinct ways to
        // miss it, kept distinct: a classifier REFUSAL ("cannot tell") is a
        // rerun, while a CONFIDENT read of some other junction means the
        // maze, the route, or the junction count is wrong. One merged message
        // would send a debugger looking for the wrong bug.
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
    // The goal ends the run before anything is recorded — the arrival IS the
    // answer, and GOAL is never a move in the path.
    if (j == JCT_GOAL) { return ARRIVE_SUCCESS; }
    // An honest "cannot tell" — stop rather than guess.
    if (j == JCT_NONE) { return ARRIVE_FAULT_CLASSIFY_NONE; }

    // No dead-end screen here, on purpose: an explorer MEETS dead ends, and
    // turning around at one is how 'B' enters the path in the first place.
    // What is left to check is the byte itself, which catches the wiring bug
    // decide_left_hand guards against: its MOVE_NONE refusal leaking through
    // to a live junction.
    if (!move_is_legal(m)) { return ARRIVE_FAULT_ILLEGAL_MOVE; }
    return ARRIVE_PROCEED;
}
