// test_maze_logic.c — host-side unit tests for the maze logic.
//
// Each check exercises one function of logic/maze_logic.c on its own:
// classify_junction, decide_left_hand, path_record, path_simplify,
// replay_next, and the two arrival-verdict functions. The sibling binary
// sim_maze.c drives the same code end to end by solving whole mazes.
//
// Reading a failure: every check prints its own name, and a FAIL line
// adds the file and line number. Junction comparisons also print what the
// classifier answered — "wanted STRAIGHT_LEFT, got LEFT_ONLY" names the
// evidence the classifier stopped believing, not just the fixture that
// broke. main()'s last line prints the totals, and the process exits
// nonzero if anything failed.
//
// Every check here has been observed failing. The earlier ones were
// written before the functions they cover existed and were red until
// maze_logic.c was implemented. The later ones were added to finished
// code, where a fresh test that passes proves nothing about the test, so
// each was verified by mutating the function under test until the check
// went red. A check that cannot fail is not a check.
//
// There is deliberately no check COUNT in this header: a hand-maintained
// census is a second source of truth about a number the program already
// computes, and it goes stale the first time someone adds a check without
// editing the comment. Trust the totals the run prints.

#include <stdio.h>
#include <string.h>
#include "maze_logic.h"
#include "tuning.h"   // JCT_DARK_THRESH / GOAL_MIN_DARK — the threshold-edge
                      // tests probe the knobs THEMSELVES, so they must read
                      // the same numbers the classifier reads. No copies.

static int t_pass, t_fail;

#define CHECK(name, cond)                                            \
    do {                                                             \
        if (cond) { t_pass++; printf("PASS  %s\n", (name)); }        \
        else      { t_fail++; printf("FAIL  %s   (%s:%d)\n",         \
                                     (name), __FILE__, __LINE__); }  \
    } while (0)

// The same check for a JUNCTION comparison, with one addition that pays
// for itself the first time the suite goes red: it prints the ANSWER. A
// bare "FAIL  classify: STRAIGHT_LEFT" says only that a fixture broke;
// "wanted STRAIGHT_LEFT, got LEFT_ONLY" says WHICH evidence the
// classifier stopped believing — the straight arm, not the side arm.
// Use it wherever the condition is "the classifier answered X".
//
// `got` and `want` are each evaluated exactly ONCE into locals: a macro
// that evaluates its argument twice works fine until the day someone
// passes it a call with a side effect, and then it lies.
//
// jct_label() is defined further down the file — legal, because a macro
// body is only compiled where it is USED, and every use is inside main().
#define CHECK_JCT(name, got, want)                                    \
    do {                                                              \
        junction_t got_ = (got), want_ = (want);                      \
        if (got_ == want_) { t_pass++; printf("PASS  %s\n", (name)); }\
        else { t_fail++;                                              \
               printf("FAIL  %s   (%s:%d)  wanted %s, got %s\n",      \
                      (name), __FILE__, __LINE__,                     \
                      jct_label(want_), jct_label(got_)); }           \
    } while (0)

// Build a snapshot from five literals, s0 = leftmost sensor.
static sensor_snapshot_t snap(uint16_t s0, uint16_t s1, uint16_t s2,
                              uint16_t s3, uint16_t s4)
{
    sensor_snapshot_t x = { { s0, s1, s2, s3, s4 } };
    return x;
}

// A byte that is not a move and — the load-bearing half — is not
// MOVE_NONE either. It marks buffer space the path does not own.
#define PATH_POISON 'Z'

// Build a path_t straight from a string literal, with the UNUSED tail of
// the buffer filled with POISON instead of zeros.
//
// Why the poison: MOVE_NONE is '\0', and the obvious initialiser `{ { 0 }, 0, false }` zeroes all 64 bytes — so every byte
// past p.len already reads back as MOVE_NONE. That made this file's
// replay fencepost checks ("past the end must answer MOVE_NONE") pass
// whether or not replay_next actually had its bounds guard: delete the
// guard, index the buffer raw, and the zeros hand back exactly the
// sentinel the test was hunting for. Measured, before this change: with
// `return p->moves[idx];` in place of the guarded return, all 117 unit
// checks still passed.
//
// A test that also passes on the broken code is decoration. Poison makes
// the tail say something the fenceposts can distinguish from success, so
// the only way to get MOVE_NONE back is for the guard to exist.
static path_t path_from(const char *s)
{
    path_t p;
    memset(p.moves, PATH_POISON, sizeof p.moves);
    p.len      = 0;
    p.overflow = false;
    while (s[p.len] != '\0' && p.len < PATH_MAX_MOVES) {
        p.moves[p.len] = s[p.len];
        p.len++;
    }
    return p;
}

static bool path_is(const path_t *p, const char *s)
{
    return strlen(s) == p->len && memcmp(p->moves, s, p->len) == 0;
}

// simplify helper: run + compare in one line.
static bool simplifies_to(const char *in, const char *out)
{
    path_t p = path_from(in);
    path_simplify(&p);
    return path_is(&p, out);
}

// -------------------------------------------------- truth-table helpers
// The label tables below exist so that a failing truth-table row names
// ITSELF: "row 17 failed" means counting commas, while "T + rot byte
// wanted FAULT illegal-move" points straight at the cell of the decision
// table that moved.

// The junction's own name, spelled locally on purpose: the shipped
// junction_name() lives in maze_logic_ref.c, and this binary deliberately
// links ONLY maze_logic.c, so a unit failure here can never be the
// reference classifier's fault. A label table is cheap; linking the two
// implementations together to avoid it is not.
static const char *jct_label(junction_t j)
{
    switch (j) {
    case JCT_LEFT_ONLY:      return "LEFT_ONLY";
    case JCT_RIGHT_ONLY:     return "RIGHT_ONLY";
    case JCT_STRAIGHT_LEFT:  return "STRAIGHT_LEFT";
    case JCT_STRAIGHT_RIGHT: return "STRAIGHT_RIGHT";
    case JCT_T:              return "T";
    case JCT_CROSS:          return "CROSS";
    case JCT_DEAD_END:       return "DEAD_END";
    case JCT_GOAL:           return "GOAL";
    case JCT_NONE:           return "NONE";
    }
    return "?";
}

static const char *move_label(move_t m)
{
    switch (m) {
    case MOVE_LEFT:     return "'L'";
    case MOVE_RIGHT:    return "'R'";
    case MOVE_STRAIGHT: return "'S'";
    case MOVE_BACK:     return "'B'";
    case MOVE_NONE:     return "MOVE_NONE";
    default:            return "rot byte";
    }
}

static const char *verdict_label(arrival_verdict_t v)
{
    switch (v) {
    case ARRIVE_PROCEED:             return "PROCEED";
    case ARRIVE_SUCCESS:             return "SUCCESS";
    case ARRIVE_FAULT_CLASSIFY_NONE: return "FAULT classify-NONE";
    case ARRIVE_FAULT_NOT_GOAL:      return "FAULT not-goal";
    case ARRIVE_FAULT_GOAL_EARLY:    return "FAULT goal-too-early";
    case ARRIVE_FAULT_DEAD_END:      return "FAULT dead-end";
    case ARRIVE_FAULT_ILLEGAL_MOVE:  return "FAULT illegal-move";
    }
    return "?";
}

// One cell of the table: what the two seams see, and what they must decide.
typedef struct {
    junction_t        j;
    move_t            m;
    arrival_verdict_t want;
} verdict_row_t;

// A byte that is not one of the four moves — the path buffer holding rot,
// or a decision table that grew a fifth answer. 'X' is arbitrary; what
// matters is that it is none of L/S/R/B and not MOVE_NONE (which means
// "path spent", a different column of the table entirely).
#define ROT_BYTE 'X'

int main(void)
{
    // ------------------------------------------------ classify: anchors
    // The two ends of the classifier's range: a full-width crossing, and
    // the absence of any line at all.
    {
        // Sweep saw the whole bar dark; after the creep the center is
        // still dark but the outers are back on white.
        sensor_snapshot_t before = snap(900, 900, 900, 900, 900);
        sensor_snapshot_t center = snap(30, 30, 900, 30, 30);
        CHECK_JCT("classify: CROSS",
                  classify_junction(&before, &center), JCT_CROSS);
    }
    {
        // The line just stopped: both snapshots are white everywhere.
        sensor_snapshot_t before = snap(30, 30, 30, 30, 30);
        sensor_snapshot_t center = snap(30, 30, 30, 30, 30);
        CHECK_JCT("classify: DEAD_END",
                  classify_junction(&before, &center), JCT_DEAD_END);
    }

    // ------------------------------------------------ simplify: anchors
    // The two shortest folds, by the heading arithmetic path_simplify
    // documents: LBL nets 270+180+270 = 720 ≡ 0 degrees and collapses to
    // S; LBS nets 270+180+0 = 450 ≡ 90 and collapses to R.
    CHECK("simplify: LBL -> S", simplifies_to("LBL", "S"));
    CHECK("simplify: LBS -> R", simplifies_to("LBS", "R"));

    // -------------------------------------------------- record: capacity
    // The buffer holds exactly PATH_MAX_MOVES moves and latches overflow
    // on the first move that does not fit.
    {
        path_t p = { { 0 }, 0, false };
        for (int i = 0; i < PATH_MAX_MOVES; i++) {
            path_record(&p, MOVE_STRAIGHT);
        }
        bool full_ok = (p.len == PATH_MAX_MOVES) && !p.overflow;
        path_record(&p, MOVE_LEFT);
        CHECK("record: 64 fit, 65th overflows",
              full_ok && p.len == PATH_MAX_MOVES && p.overflow);
    }

    // ---------------------------------------------------- replay: basics
    // replay_next hands back the recorded moves in order, then reports
    // the path spent.
    {
        path_t p = path_from("LRS");
        CHECK("replay: walks LRS then ends",
              replay_next(&p, 0) == MOVE_LEFT &&
              replay_next(&p, 1) == MOVE_RIGHT &&
              replay_next(&p, 2) == MOVE_STRAIGHT &&
              replay_next(&p, 3) == MOVE_NONE);
    }

    // -----------------------------------------------------------------
    // The sections below cover the rest of the surface: a realistic
    // classify snapshot per junction type, the threshold edges, the
    // GOAL-vs-CROSS split, the decide ranking, the string algebra of
    // path_simplify, the overflow latch, the replay fenceposts, and the
    // two arrival-verdict truth tables.
    //
    // Everything here stays at the single-function level. The end-to-end
    // walk — classify → decide → record → simplify → replay over a whole
    // maze — is sim_maze.c's job.

    // ------------------------------------------------ decide: the ranking
    // The left-hand rule is a RANKING, not a lookup table: take the
    // leftmost exit that exists — L beats S beats R — and B only when no
    // exit exists at all. Every case below is that one sentence applied
    // to one junction's exit set.
    CHECK("decide: LEFT_ONLY -> L",
          decide_left_hand(JCT_LEFT_ONLY) == MOVE_LEFT);
    CHECK("decide: STRAIGHT_LEFT -> L",
          decide_left_hand(JCT_STRAIGHT_LEFT) == MOVE_LEFT);
    CHECK("decide: T -> L",
          decide_left_hand(JCT_T) == MOVE_LEFT);
    CHECK("decide: CROSS -> L",
          decide_left_hand(JCT_CROSS) == MOVE_LEFT);
    CHECK("decide: STRAIGHT_RIGHT -> S",   // no left arm: S is leftmost
          decide_left_hand(JCT_STRAIGHT_RIGHT) == MOVE_STRAIGHT);
    CHECK("decide: RIGHT_ONLY -> R",       // one exit: take it
          decide_left_hand(JCT_RIGHT_ONLY) == MOVE_RIGHT);
    CHECK("decide: DEAD_END -> B",         // exactly one legal move
          decide_left_hand(JCT_DEAD_END) == MOVE_BACK);
    {
        // GOAL and NONE never reach decide (main.c handles both first),
        // but the contract requires "something safe anyway" — without
        // naming it. So this test pins the PROPERTY, not one answer: a
        // confused robot may retreat over tape it already knows ('B') or
        // refuse to move (MOVE_NONE) — what it must never do is guess a
        // forward move (L/S/R) onto floor that may hold no line, and the
        // two "I'm lost" inputs must not get different answers.
        // maze_logic.c picks MOVE_NONE and argues, at the function, why
        // refusal beats retreat.
        move_t g = decide_left_hand(JCT_GOAL);
        move_t n = decide_left_hand(JCT_NONE);
        CHECK("decide: GOAL/NONE safe, consistent, never a forward guess",
              g == n && (g == MOVE_BACK || g == MOVE_NONE));
    }

    // --------------------------------------------- replay: the boundary
    {
        // idx == len is the exact moment the replay loop learns "the NEXT
        // junction is the goal". A fencepost here and the robot turns
        // INTO the goal instead of stopping on it — see the replay_next
        // contract in maze_logic.c.
        path_t p = path_from("LSR");
        CHECK("replay: idx == len -> MOVE_NONE",
              replay_next(&p, p.len) == MOVE_NONE);
    }

    // -----------------------------------------------------------------
    // The checks above prove the logic CAN solve a maze; the ones below
    // prove it cannot be fooled. Two themes run through every fixture:
    //
    //   BOUNDARIES — every threshold comparison has an exact edge, and
    //   a test pins which side each edge lands on. An off-by-one in a
    //   comparison operator is invisible on clean tape and fatal on the
    //   afternoon a reading lands exactly on the knob.
    //
    //   HONESTY — evidence that fits no junction signature must come
    //   back JCT_NONE. A classifier that always names SOME type passes
    //   every clean-input test ever written, then confidently turns
    //   left on a goal patch it misread as a crossing. Two fixtures
    //   below are built so that such a classifier MUST fail here first.

    // ---------------------------- classify: one fixture per junction type
    // Snapshot values come from the simulator's sensor model (sim_maze.c
    // documents the physics): tape dead-on 830, latched shoulder peak
    // 520, fresh shoulder 480, latched paper peak 150, fresh paper 60.
    // Not 0/1000 caricatures, on purpose: the maze build guarantees only
    // "paper well under ~200, tape well over ~700", so a classifier that
    // needs cartoon contrast would pass a cartoon test and fail on the
    // real board.
    {
        // ← corner: the sweep latched the left arm (s0,s1) and the entry
        // line (s2); the right side saw only its own following-time
        // peaks. After the creep: bare paper — nothing continues.
        sensor_snapshot_t before = snap(830, 830, 830, 520, 150);
        sensor_snapshot_t center = snap(60, 60, 60, 60, 60);
        CHECK_JCT("classify: LEFT_ONLY (model values)",
                  classify_junction(&before, &center), JCT_LEFT_ONLY);
    }
    {
        // → corner: the mirror image.
        sensor_snapshot_t before = snap(150, 520, 830, 830, 830);
        sensor_snapshot_t center = snap(60, 60, 60, 60, 60);
        CHECK_JCT("classify: RIGHT_ONLY (model values)",
                  classify_junction(&before, &center), JCT_RIGHT_ONLY);
    }
    {
        // ↑← : a left arm in the sweep AND line still under the middle
        // of the bar after the creep — the straight exit, riding its
        // half-lit shoulders.
        sensor_snapshot_t before = snap(830, 830, 830, 520, 150);
        sensor_snapshot_t center = snap(60, 480, 830, 480, 60);
        CHECK_JCT("classify: STRAIGHT_LEFT (model values)",
                  classify_junction(&before, &center), JCT_STRAIGHT_LEFT);
    }
    {
        sensor_snapshot_t before = snap(150, 520, 830, 830, 830);
        sensor_snapshot_t center = snap(60, 480, 830, 480, 60);
        CHECK_JCT("classify: STRAIGHT_RIGHT (model values)",
                  classify_junction(&before, &center), JCT_STRAIGHT_RIGHT);
    }
    {
        // ←→ from below: both arms latched the whole bar dark on the
        // way in; past the junction, paper — nothing continues.
        sensor_snapshot_t before = snap(830, 830, 830, 830, 830);
        sensor_snapshot_t center = snap(60, 60, 60, 60, 60);
        CHECK_JCT("classify: T (model values)",
                  classify_junction(&before, &center), JCT_T);
    }
    {
        // ↑←→ : the same approach as the T — only at_center can split
        // them, and here the straight exit is still under the bar.
        sensor_snapshot_t before = snap(830, 830, 830, 830, 830);
        sensor_snapshot_t center = snap(60, 480, 830, 480, 60);
        CHECK_JCT("classify: CROSS (model values)",
                  classify_junction(&before, &center), JCT_CROSS);
    }
    {
        // Dead end: the loss path confirmed white before classify was
        // ever called, so `before` holds only the sweep's dust-and-grain
        // peaks and at_center is fresh paper.
        sensor_snapshot_t before = snap(150, 150, 150, 150, 150);
        sensor_snapshot_t center = snap(60, 60, 60, 60, 60);
        CHECK_JCT("classify: DEAD_END (model values)",
                  classify_junction(&before, &center), JCT_DEAD_END);
    }
    {
        // Goal: the approach is indistinguishable from a CROSS — whole
        // bar dark — but after the creep the bar is STILL on solid
        // black. This fixture and the CROSS fixture above are the
        // GOAL-vs-CROSS margin pair (at_center all-dark vs center-only-
        // dark): at_center is the only witness there is.
        sensor_snapshot_t before = snap(830, 830, 830, 830, 830);
        sensor_snapshot_t center = snap(830, 830, 830, 830, 830);
        CHECK_JCT("classify: GOAL (model values)",
                  classify_junction(&before, &center), JCT_GOAL);
    }

    // -------------------------------- classify: the threshold's edges
    // dark() is `v >= JCT_DARK_THRESH` (maze_logic.c) and the knob is
    // 600 (tuning.h), so the TRUE boundary pair is 599 light / 600 dark.
    // A slack pair at 590/610 is pinned alongside it, and the two catch
    // different regressions: the slack pair survives a small tuning nudge
    // and keeps asserting "the line is NEAR 600", while the exact pair
    // (spelled with the macro, so it follows the knob) is the only one
    // that catches a `>=` quietly becoming `>`. The fixture is the
    // CROSS/T flip — a crossing whose continuing line has faded to the
    // knife edge — i.e. exactly the comparison the contract's "never
    // wrong on GOAL vs CROSS vs T" clause governs.
    {
        sensor_snapshot_t before = snap(830, 830, 830, 830, 830);
        sensor_snapshot_t c610   = snap(60, 480, 610, 480, 60);
        sensor_snapshot_t c590   = snap(60, 480, 590, 480, 60);
        CHECK_JCT("classify: center 610 -> CROSS (slack pair, dark side)",
                  classify_junction(&before, &c610), JCT_CROSS);
        CHECK_JCT("classify: center 590 -> T (slack pair, light side)",
                  classify_junction(&before, &c590), JCT_T);

        sensor_snapshot_t c_on  = snap(60, 480, JCT_DARK_THRESH,     480, 60);
        sensor_snapshot_t c_off = snap(60, 480, JCT_DARK_THRESH - 1, 480, 60);
        CHECK_JCT("classify: center ==thresh -> CROSS (>= is the contract)",
                  classify_junction(&before, &c_on), JCT_CROSS);
        CHECK_JCT("classify: center thresh-1 -> T (first light value)",
                  classify_junction(&before, &c_off), JCT_T);
    }
    // The same edge on an OUTER sensor — where side-branch evidence
    // lives. At exactly the threshold the latched arm counts as a
    // branch; one count below, the "arm" evaporates…
    {
        sensor_snapshot_t on     = snap(JCT_DARK_THRESH,     830, 830, 520, 150);
        sensor_snapshot_t off    = snap(JCT_DARK_THRESH - 1, 830, 830, 520, 150);
        sensor_snapshot_t center = snap(60, 480, 830, 480, 60);
        CHECK_JCT("classify: s0 ==thresh -> STRAIGHT_LEFT",
                  classify_junction(&on, &center), JCT_STRAIGHT_LEFT);
        // …and what remains fits NOTHING: the junction detector fired
        // and the line continues past the junction, yet no arm cleared
        // the threshold. A plain corridor never fires the detector, so
        // "straight on, no branches" is not a junction signature at all —
        // naming one anyway would be a guess. The classifier already
        // answers JCT_NONE here; this pin makes sure no later change ever
        // loses that honesty.
        CHECK_JCT("classify: s0 thresh-1 -> JCT_NONE (no signature fits)",
                  classify_junction(&off, &center), JCT_NONE);
    }

    // --------------------- classify: the GOAL-vs-CROSS margin, counted
    // On approach a goal patch and a 4-way crossing paint the bar
    // identically black, so at_center's DARK-SENSOR COUNT is the entire
    // case file. The signatures, stated (this is the margin tuning.h's
    // GOAL_MIN_DARK comment promises):
    //   1–2 dark = a crossing's straight exit (one sensor, or two when
    //              the creep stops off-center)          -> CROSS
    //   4–5 dark = the goal patch, GOAL_MIN_DARK = 4 tolerating one
    //              sensor on a dusty patch edge          -> GOAL
    //   3 dark   = MORE line than any crossing leaves, LESS black than
    //              the goal guarantees: the gap between the signatures.
    {
        sensor_snapshot_t before = snap(830, 830, 830, 830, 830);
        sensor_snapshot_t c2 = snap(60, 830, 830, 480, 60);
        CHECK_JCT("classify: 2-dark at_center -> CROSS (off-center stop)",
                  classify_junction(&before, &c2), JCT_CROSS);

        sensor_snapshot_t c4 = snap(830, 830, 830, 830, 150);
        CHECK_JCT("classify: 4-dark at_center -> GOAL (GOAL_MIN_DARK margin)",
                  classify_junction(&before, &c4), JCT_GOAL);

        // The 3-dark row is DESIGNED to be undecidable: it could be the
        // goal patch with the bar skewed so two sensors hang past the
        // edge, or a crossing whose exit line plus a dirt-darkened
        // shoulder cover three sensors. Answering CROSS in the first
        // world turns the robot left OFF the goal patch mid-victory;
        // answering GOAL in the second world ends a replay one junction
        // early. "Never wrong on GOAL vs CROSS vs T" — the contract's
        // one absolute — leaves exactly one honest verdict: refuse to
        // name it. A classifier with no "I can't tell" in its
        // vocabulary CANNOT pass this test; that is the whole point of
        // JCT_NONE existing.
        sensor_snapshot_t c3 = snap(830, 830, 640, 480, 150);
        CHECK_JCT("classify: 3-dark at_center is the gap -> JCT_NONE",
                  classify_junction(&before, &c3), JCT_NONE);
    }

    // -------------------- classify: evidence that contradicts itself
    {
        // The dead-end signature is "the absence of everything": the
        // line ended, the loss path confirmed white, and classify
        // receives paper in BOTH snapshots (the contract block says
        // exactly this). Here the sweep saw nothing — yet after the
        // creep there is solid tape under an outer sensor. Something IS
        // down there (a stray scrap, a neighboring run's edge, a creep
        // that drifted off the board), and certifying DEAD_END would
        // send the robot into a blind 180 on top of it. Paper-plus-
        // paper is a dead end; paper-plus-TAPE fits no signature.
        sensor_snapshot_t before = snap(150, 150, 150, 150, 150);
        sensor_snapshot_t center = snap(60, 60, 60, 60, 830);
        CHECK_JCT("classify: tape past a vanished line -> JCT_NONE",
                  classify_junction(&before, &center), JCT_NONE);
    }

    // --------------------------------------- simplify: string algebra
    // Every expected answer below is DERIVED, not remembered. The rule
    // (maze_logic.c's collapse): a detour  x B y  nets  x + 180 + y
    // degrees of clockwise heading change (L=270, S=0, R=90, B=180,
    // all mod 360), and the replacement is the single move with the
    // same net:
    //   RBL:  90 + 180 + 270 = 540 ≡ 180  -> B   (in right, out left:
    //         the robot leaves facing back the way it came)
    //   SBS:   0 + 180 +   0 = 180        -> B   (the spur was in-line,
    //         so the whole detour was a reversal)
    //   RBR:  90 + 180 +  90 = 360 ≡ 0    -> S
    //   SBL:   0 + 180 + 270 = 450 ≡ 90   -> R
    CHECK("simplify: RBL -> B", simplifies_to("RBL", "B"));
    CHECK("simplify: SBS -> B", simplifies_to("SBS", "B"));
    CHECK("simplify: RBR -> S", simplifies_to("RBR", "S"));
    CHECK("simplify: SBL -> R", simplifies_to("SBL", "R"));

    // The multi-pass monster, worked by hand:
    //   L L B S B L
    //   -> the first triple L·B·S folds to R (270+180+0 ≡ 90):  L R B L
    //   -> the fold EXPOSED a new triple R·B·L (90+180+270 ≡ 180):  L B
    //   -> 'B' is now the last move: no triple surrounds it. Done.
    // One scan is not enough — the repeat-until-stable outer loop is
    // the thing under test here.
    CHECK("simplify: LLBSBL -> LB (multi-pass)",
          simplifies_to("LLBSBL", "LB"));

    {
        // A path with no 'B' gives simplify nothing to say. "Unchanged"
        // is asserted across the WHOLE 64-byte buffer plus len plus the
        // overflow latch — not just the visible prefix — because
        // "simplify didn't touch it" and "simplify rewrote it into the
        // same string" are different claims about the code.
        path_t p = path_from("LSRRSL");
        path_t orig = p;
        path_simplify(&p);
        CHECK("simplify: no-B path untouched byte-for-byte",
              memcmp(p.moves, orig.moves, sizeof p.moves) == 0 &&
              p.len == orig.len && p.overflow == orig.overflow);
    }

    // Bounds: the triple scan runs i over [1, len-2], which is EMPTY
    // for len 0, 1, and 2 — these calls must return without reading a
    // single byte out of range, and a 'B' at either edge of the path
    // has no (x, B, y) around it, so it must survive untouched.
    CHECK("simplify: empty path stays empty", simplifies_to("", ""));
    CHECK("simplify: lone B survives",        simplifies_to("B", "B"));
    CHECK("simplify: len-1 no-op",            simplifies_to("L", "L"));
    CHECK("simplify: len-2 edge B never folds",
          simplifies_to("SB", "SB") && simplifies_to("BS", "BS"));

    // ----------------------------------- record: the latch stays down
    {
        // Overflow is a LATCH, not a status flag: once a move has been
        // dropped the recorded route is a lie, and no amount of later
        // success un-lies it. The explore loop trusts this bit to
        // refuse to "solve" a truncated path — if recording move #66
        // (or #67, #68…) could clear it, that safety check would be
        // theater.
        path_t p = { { 0 }, 0, false };
        for (int i = 0; i < PATH_MAX_MOVES; i++) {
            path_record(&p, MOVE_STRAIGHT);
        }
        path_record(&p, MOVE_LEFT);        // move #65: dropped, latch set
        bool latched = p.overflow;
        path_record(&p, MOVE_RIGHT);       // keep recording anyway…
        path_record(&p, MOVE_BACK);
        CHECK("record: overflow stays latched, path stays intact",
              latched && p.overflow && p.len == PATH_MAX_MOVES &&
              p.moves[PATH_MAX_MOVES - 1] == MOVE_STRAIGHT);
    }

    // ---------------------------------- replay: the fencepost family
    {
        // len-1 is the LAST real move; len is the first arrival with
        // nothing left to say (the replay loop's "the NEXT junction is
        // the goal" moment — already pinned above, re-checked here so
        // the family reads as one story); far past the end must stay
        // MOVE_NONE forever, because a replay loop that keeps asking
        // after a missed goal must keep hearing "stop", never stale
        // buffer bytes. 255 is uint8_t's ceiling — as far past as the
        // index type can express.
        path_t p = path_from("LSR");
        CHECK("replay: len-1 / len / far-past fenceposts",
              replay_next(&p, p.len - 1) == MOVE_RIGHT &&
              replay_next(&p, p.len)     == MOVE_NONE  &&
              replay_next(&p, 255)       == MOVE_NONE);
    }

    // -----------------------------------------------------------------
    // THE ARRIVAL VERDICT, as a truth table.
    //
    // Every arrival — explore or replay — ends in the same three-way
    // question: keep driving, declare the run won, or stop with a fault.
    // That question was once answered TWICE in hand-written `if` chains,
    // once in main.c (which this binary cannot reach) and once in
    // sim_maze.c in a different vocabulary, and the two copies had
    // already drifted apart in three cells. The answer now lives in ONE
    // pure function per lane, which means it lives where a table can pin
    // every cell — including the cells the robot has never reached.
    //
    // Read each row as a sentence: at THIS junction, holding THIS move
    // byte, the seam must do THIS. Three columns decide everything:
    // is the path spent (m == MOVE_NONE), what did the classifier say,
    // and is the byte a move at all.
    //
    // The unreachable cells are here on purpose. "Unreachable" is a claim
    // about today's callers, not a property of the function, and the day
    // a wiring bug makes one reachable is the day its behavior wants to
    // be already pinned rather than discovered on the floor at speed.
    {
        // REPLAY — the strict lane. The path is the plan; any arrival
        // that contradicts the plan is a fault, because a replay that
        // guesses is just exploring, badly.
        static const verdict_row_t replay_rows[] = {
            // --- Column 1: the path is SPENT (replay_next hit idx==len).
            // The contract says this arrival must BE the goal. Missing it
            // splits two ways on purpose: a refusal ("I can't tell") is a
            // rerun; a CONFIDENT read of some other junction means the
            // maze, the route, or the junction count is wrong.
            { JCT_GOAL,           MOVE_NONE, ARRIVE_SUCCESS             },
            { JCT_NONE,           MOVE_NONE, ARRIVE_FAULT_CLASSIFY_NONE },
            { JCT_LEFT_ONLY,      MOVE_NONE, ARRIVE_FAULT_NOT_GOAL      },
            { JCT_RIGHT_ONLY,     MOVE_NONE, ARRIVE_FAULT_NOT_GOAL      },
            { JCT_STRAIGHT_LEFT,  MOVE_NONE, ARRIVE_FAULT_NOT_GOAL      },
            { JCT_STRAIGHT_RIGHT, MOVE_NONE, ARRIVE_FAULT_NOT_GOAL      },
            { JCT_T,              MOVE_NONE, ARRIVE_FAULT_NOT_GOAL      },
            { JCT_CROSS,          MOVE_NONE, ARRIVE_FAULT_NOT_GOAL      },
            { JCT_DEAD_END,       MOVE_NONE, ARRIVE_FAULT_NOT_GOAL      },

            // --- Column 2: moves REMAIN, and the byte is a real move.
            // An ordinary junction is what the plan expects: proceed.
            { JCT_LEFT_ONLY,      MOVE_LEFT,     ARRIVE_PROCEED },
            { JCT_RIGHT_ONLY,     MOVE_RIGHT,    ARRIVE_PROCEED },
            { JCT_STRAIGHT_LEFT,  MOVE_STRAIGHT, ARRIVE_PROCEED },
            { JCT_STRAIGHT_RIGHT, MOVE_STRAIGHT, ARRIVE_PROCEED },
            { JCT_T,              MOVE_LEFT,     ARRIVE_PROCEED },
            // All four letters sweep one junction, so no letter can quietly
            // fall out of the legal set. 'B' belongs here: simplify leaves a
            // 'B' whenever a detour nets 180° (RBL -> B), so a replay really
            // can be told to turn around at a live junction.
            { JCT_CROSS,          MOVE_LEFT,     ARRIVE_PROCEED },
            { JCT_CROSS,          MOVE_RIGHT,    ARRIVE_PROCEED },
            { JCT_CROSS,          MOVE_STRAIGHT, ARRIVE_PROCEED },
            { JCT_CROSS,          MOVE_BACK,     ARRIVE_PROCEED },
            // The three arrivals that contradict "an ordinary junction".
            // DEAD_END is the load-bearing one: 'B' is a perfectly legal
            // byte, but a SIMPLIFIED route never enters a dead end — so
            // meeting one means a detour survived simplify.
            { JCT_DEAD_END,       MOVE_BACK,     ARRIVE_FAULT_DEAD_END      },
            { JCT_GOAL,           MOVE_LEFT,     ARRIVE_FAULT_GOAL_EARLY    },
            { JCT_NONE,           MOVE_LEFT,     ARRIVE_FAULT_CLASSIFY_NONE },

            // --- Column 3: moves remain, but the byte is NOT a move.
            // Unreachable today (path_solved only ever holds what
            // decide_left_hand recorded), which is precisely why it is
            // pinned: this is the cell a corrupted path buffer lands in.
            { JCT_LEFT_ONLY,      ROT_BYTE, ARRIVE_FAULT_ILLEGAL_MOVE },
            { JCT_RIGHT_ONLY,     ROT_BYTE, ARRIVE_FAULT_ILLEGAL_MOVE },
            { JCT_STRAIGHT_LEFT,  ROT_BYTE, ARRIVE_FAULT_ILLEGAL_MOVE },
            { JCT_STRAIGHT_RIGHT, ROT_BYTE, ARRIVE_FAULT_ILLEGAL_MOVE },
            { JCT_T,              ROT_BYTE, ARRIVE_FAULT_ILLEGAL_MOVE },
            { JCT_CROSS,          ROT_BYTE, ARRIVE_FAULT_ILLEGAL_MOVE },
            // PRECEDENCE: the junction screens run BEFORE the byte is
            // judged, so a contradicted arrival keeps its own fault even
            // when the byte is rot. The screens describe where the robot
            // IS; the byte only describes what it was about to do.
            { JCT_DEAD_END,       ROT_BYTE, ARRIVE_FAULT_DEAD_END      },
            { JCT_GOAL,           ROT_BYTE, ARRIVE_FAULT_GOAL_EARLY    },
            { JCT_NONE,           ROT_BYTE, ARRIVE_FAULT_CLASSIFY_NONE },
        };
        for (int i = 0; i < (int)(sizeof replay_rows / sizeof replay_rows[0]); i++) {
            char nm[96];
            snprintf(nm, sizeof nm, "replay verdict: %s + %s -> %s",
                     jct_label(replay_rows[i].j),
                     move_label(replay_rows[i].m),
                     verdict_label(replay_rows[i].want));
            CHECK(nm, replay_arrival_verdict(replay_rows[i].j,
                                             replay_rows[i].m) ==
                      replay_rows[i].want);
        }
    }

    {
        // EXPLORE — the permissive twin. Same three-way question, two
        // deliberate differences from replay, both of them the DEFINITION
        // of exploring:
        //   - a DEAD_END is an ordinary arrival (turning around there is
        //     how 'B' gets recorded at all), not a fault;
        //   - there is no plan to contradict, so no "path spent" column —
        //     the explorer stops when it finds the GOAL, not when it runs
        //     out of moves.
        static const verdict_row_t explore_rows[] = {
            // The seven live junctions, each paired with the move
            // decide_left_hand actually returns for it.
            { JCT_LEFT_ONLY,      MOVE_LEFT,     ARRIVE_PROCEED },
            { JCT_RIGHT_ONLY,     MOVE_RIGHT,    ARRIVE_PROCEED },
            { JCT_STRAIGHT_LEFT,  MOVE_LEFT,     ARRIVE_PROCEED },
            { JCT_STRAIGHT_RIGHT, MOVE_STRAIGHT, ARRIVE_PROCEED },
            { JCT_T,              MOVE_LEFT,     ARRIVE_PROCEED },
            { JCT_CROSS,          MOVE_LEFT,     ARRIVE_PROCEED },
            { JCT_DEAD_END,       MOVE_BACK,     ARRIVE_PROCEED },

            // The two arrivals that END the explore loop. decide_left_hand
            // answers MOVE_NONE at both (it refuses to move when lost),
            // so these rows are the pairing the firmware really produces.
            { JCT_GOAL,           MOVE_NONE, ARRIVE_SUCCESS             },
            { JCT_NONE,           MOVE_NONE, ARRIVE_FAULT_CLASSIFY_NONE },

            // decide_left_hand's refusal LEAKING to a live junction: the
            // wiring bug its contract warns about. MOVE_NONE is not a
            // move, so the explorer stops instead of driving on a byte
            // that means "I refuse".
            { JCT_LEFT_ONLY,      MOVE_NONE, ARRIVE_FAULT_ILLEGAL_MOVE },

            // Rot bytes at live junctions — including DEAD_END, which the
            // explorer screens ONLY for legality (contrast the replay
            // table's DEAD_END rows: same junction, opposite verdict).
            { JCT_T,        ROT_BYTE, ARRIVE_FAULT_ILLEGAL_MOVE },
            { JCT_CROSS,    ROT_BYTE, ARRIVE_FAULT_ILLEGAL_MOVE },
            { JCT_DEAD_END, ROT_BYTE, ARRIVE_FAULT_ILLEGAL_MOVE },

            // PRECEDENCE again: what the classifier saw outranks the byte.
            // A goal patch is still a win even if the move byte is rot,
            // and a refusal is still a refusal.
            { JCT_GOAL,     ROT_BYTE, ARRIVE_SUCCESS             },
            { JCT_NONE,     ROT_BYTE, ARRIVE_FAULT_CLASSIFY_NONE },

            // The same precedence with a PERFECTLY LEGAL byte — the
            // harder half of the rule. Rot is easy to outrank; here the
            // byte names a maneuver the robot really could execute, and
            // the junction screen must still win. It is also the cell
            // where the two lanes part company on identical inputs:
            // replay reads {GOAL, 'L'} as "goal too early" because moves
            // are still owed, while an explorer owes nobody anything and
            // reads it as the win. Unreachable today — decide_left_hand
            // answers MOVE_NONE at GOAL and NONE — and pinned for exactly
            // the reason the replay table pins its own unreachable cells:
            // an untested cell is where two twins drift apart.
            { JCT_GOAL,     MOVE_LEFT, ARRIVE_SUCCESS             },
            { JCT_NONE,     MOVE_LEFT, ARRIVE_FAULT_CLASSIFY_NONE },
        };
        for (int i = 0; i < (int)(sizeof explore_rows / sizeof explore_rows[0]); i++) {
            char nm[96];
            snprintf(nm, sizeof nm, "explore verdict: %s + %s -> %s",
                     jct_label(explore_rows[i].j),
                     move_label(explore_rows[i].m),
                     verdict_label(explore_rows[i].want));
            CHECK(nm, explore_arrival_verdict(explore_rows[i].j,
                                              explore_rows[i].m) ==
                      explore_rows[i].want);
        }
    }

    {
        // The two halves of the explore seam, COUPLED. The rows above pin
        // the verdict against move bytes typed by hand; these pin it
        // against the byte decide_left_hand really hands over, for every
        // junction the enum can hold. They are the assertions that would
        // catch a future decide_left_hand answering, say, MOVE_STRAIGHT
        // at a dead end: the pairing would still be "legal", but these
        // checks read the whole seam as one sentence — an explorer stops
        // ONLY on the goal patch and on "I can't tell".
        //
        // One CHECK PER JUNCTION, deliberately, instead of latching one
        // boolean across the sweep: a single verdict for nine junctions
        // reports THAT the seam broke and then refuses to say WHERE,
        // whereas a named check makes the failing line hand over the
        // case. The loop count is fixed by the enum, so the number of
        // checks stays deterministic.
        for (int j = JCT_LEFT_ONLY; j <= JCT_NONE; j++) {
            junction_t        jt = (junction_t)j;
            arrival_verdict_t v  = explore_arrival_verdict(jt,
                                                           decide_left_hand(jt));
            arrival_verdict_t want = (jt == JCT_GOAL) ? ARRIVE_SUCCESS
                                   : (jt == JCT_NONE) ? ARRIVE_FAULT_CLASSIFY_NONE
                                                      : ARRIVE_PROCEED;
            char nm[128];
            snprintf(nm, sizeof nm,
                     "explore seam coupled at %s: decide_left_hand's "
                     "answer -> %s", jct_label(jt), verdict_label(want));
            CHECK(nm, v == want);
        }
    }

    // -----------------------------------------------------------------
    // THE CHECKS A MUTATION TEST ASKED FOR.
    //
    // The suite was measured the only way a test suite can honestly be
    // measured: maze_logic.c was broken on purpose, one line at a time,
    // and the tests re-run. Six mutations went in and four of them never
    // turned the suite red. Line coverage said nothing useful — every
    // mutated line was executed — because the question that matters is
    // how many WRONG versions of the code the tests would catch.
    //
    // Every block below kills one of those survivors and names the
    // mutation it kills. They are also a record of how coverage lies: in
    // all four cases the fixtures ran the mutated line and read the
    // mutated value, and still agreed with the answer, because every
    // fixture in the file happened to make the difference invisible.
    // Where a blind spot had an obvious mirror image that no mutation
    // happened to try, the twin is pinned too; a bug does not have to be
    // discovered to be worth closing.

    // ------------------- mutation 1: the outer sensors are NOT
    // interchangeable. `right = dark(before->s[4])` rewritten to `s[3]`,
    // and the suite stayed green.
    //
    // Why nothing caught it: EVERY fixture in the project set s3 and s4
    // to the same value. The simulator's model darkens a side's two
    // sensors as a pair (that is what an arm's tape does to the bar),
    // and every snap() literal here followed the model. Two sensors can
    // only be told apart by a fixture that puts them on OPPOSITE sides
    // of the threshold — so that is what these two build.
    //
    // The physical story is real, not a contrivance: the right arm's
    // tape crosses under the outermost sensor during the sweep, while
    // s3 — between the arm and the entry line — peaks one count short
    // of the knob (narrow arm, skewed approach, a latch that caught the
    // shoulder a millisecond late).
    {
        sensor_snapshot_t center = snap(60, 480, 830, 480, 60);

        sensor_snapshot_t s4_dark  = snap(150, 520, 830,
                                          JCT_DARK_THRESH - 1,
                                          JCT_DARK_THRESH);
        CHECK_JCT("classify: s4 ==thresh with s3 light -> STRAIGHT_RIGHT",
                  classify_junction(&s4_dark, &center), JCT_STRAIGHT_RIGHT);

        // The mirror: the same two counts, swapped between the sensors.
        // Now NO arm cleared the threshold, so the evidence fits nothing
        // and the honest answer is a refusal — while a classifier that
        // read s3 for branch evidence would confidently call this a
        // right branch. One fixture pair, both directions of the bug.
        sensor_snapshot_t s4_light = snap(150, 520, 830,
                                          JCT_DARK_THRESH,
                                          JCT_DARK_THRESH - 1);
        CHECK_JCT("classify: s4 thresh-1 with s3 dark -> JCT_NONE",
                  classify_junction(&s4_light, &center), JCT_NONE);
    }
    {
        // The same trap on the LEFT, never mutated but wide open: the s0
        // edge pair above holds s1 dark at 830 on both sides of its
        // boundary, so a classifier reading s[1] for left-branch
        // evidence sails straight through it. Here the left arm is s0
        // alone, and s1 is the one count below the knob.
        sensor_snapshot_t center = snap(60, 480, 830, 480, 60);

        sensor_snapshot_t s0_dark  = snap(JCT_DARK_THRESH,
                                          JCT_DARK_THRESH - 1,
                                          830, 520, 150);
        CHECK_JCT("classify: s0 ==thresh with s1 light -> STRAIGHT_LEFT",
                  classify_junction(&s0_dark, &center), JCT_STRAIGHT_LEFT);

        sensor_snapshot_t s0_light = snap(JCT_DARK_THRESH - 1,
                                          JCT_DARK_THRESH,
                                          830, 520, 150);
        CHECK_JCT("classify: s0 thresh-1 with s1 dark -> JCT_NONE",
                  classify_junction(&s0_light, &center), JCT_NONE);
    }

    // ------------------- mutation 2: straight evidence is THREE sensors,
    // and the suite only ever used the middle one. The
    // `|| dark(at_center->s[3])` arm deleted, and the suite stayed green.
    //
    // Why nothing caught it: every straight fixture in the file put the
    // continuing line dead under s2, so the other two arms of that OR
    // never decided anything. They exist for the off-center stop — the
    // creep ends a few millimetres to one side and the exit line parks
    // under s1 or s3 instead, with its shoulders on the neighbours.
    // Delete an arm and the robot stops seeing the straight exit on
    // exactly the arrivals where the creep drifted: it reads a crossing
    // as a T and turns into the wall.
    {
        // Creep stopped LEFT of the line: exit under s3, shoulders on
        // s2 and s4, paper on the far side. Left arm in the sweep.
        sensor_snapshot_t before = snap(830, 830, 830, 520, 150);
        sensor_snapshot_t s3_only = snap(60, 60, 480, 830, 480);
        CHECK_JCT("classify: straight seen by s3 alone -> STRAIGHT_LEFT",
                  classify_junction(&before, &s3_only), JCT_STRAIGHT_LEFT);
    }
    {
        // Creep stopped RIGHT of the line: exit under s1. Right arm in
        // the sweep, so the answer must be STRAIGHT_RIGHT — the s1 arm
        // of the OR is the only thing keeping it from RIGHT_ONLY.
        sensor_snapshot_t before = snap(150, 520, 830, 830, 830);
        sensor_snapshot_t s1_only = snap(480, 830, 480, 60, 60);
        CHECK_JCT("classify: straight seen by s1 alone -> STRAIGHT_RIGHT",
                  classify_junction(&before, &s1_only), JCT_STRAIGHT_RIGHT);
    }

    // ------------------- mutation 3: every arm of the collapse switch.
    // `default: return MOVE_LEFT` rewritten to MOVE_RIGHT, and the suite
    // stayed green.
    //
    // Why nothing caught it: no assertion in the suite ever made
    // collapse() answer MOVE_LEFT. The six triples pinned above all land on
    // the 0°, 90° and 180° arms; the 270° arm — the one the switch
    // spells as `default`, with the comment "the only value left" — is
    // reached by exactly four (x, y) pairs, and none of them had ever
    // been written down. A switch arm no test reaches is a switch arm
    // anyone can retype.
    //
    // The collapse is a 4x4 TABLE. A detour x·B·y nets
    // heading_delta(x) + 180 + heading_delta(y) (mod 360), with L = 270,
    // S = 0, R = 90, B = 180, and the replacement is the single move
    // with that same net. Sixteen cells; six are pinned above (LBL, LBS,
    // RBL, SBS, RBR, SBL). Here are the other ten. The net is spelled
    // into every check name, so a failure reports the arithmetic rather
    // than a puzzle.
    //
    // The rows with a SECOND 'B' need a stance, and the stance is
    // narrower than it looks. What is TRUE: "BB" never appears in a
    // RECORDED path. decide_left_hand emits MOVE_BACK only at a dead
    // end, and the move recorded after backing out of one belongs to the
    // next junction — which, having two or more arms, is not a dead end.
    // What is FALSE — and was written here until it was measured — is
    // that these cells therefore cannot be reached by a real run.
    // path_simplify feeds its OWN INTERMEDIATES back through collapse():
    // simplify("LBRBL") folds its LEFTMOST triple, L·B·R, first (270 +
    // 180 + 90 = 540 = 180 = B), leaving "BBL" in the buffer, and the
    // very next pass calls
    // collapse('B','L') — the default arm, from a path an explore could
    // genuinely record. So these rows are load-bearing coverage of a
    // reachable code path, not a courtesy to an unreachable one. (They
    // would deserve pinning either way, for the reason the arrival-
    // verdict table pins its unreachable cells: "unreachable" is a claim
    // about today's callers, not a property of the function. The heading
    // arithmetic is TOTAL — 'B' is a 180° move like any other — so the
    // fold is already defined, and a test is how a definition stops
    // being an accident.)
    {
        static const struct {
            const char *in;    // the recorded triple
            const char *out;   // what path_simplify must leave behind
            const char *net;   // the arithmetic, worked
        } collapse_rows[] = {
            { "LBR", "B", "270+180+90 = 540 = 180"  },
            { "LBB", "L", "270+180+180 = 630 = 270" },   // default arm
            { "SBR", "L", "0+180+90 = 270"          },   // default arm
            { "SBB", "S", "0+180+180 = 360 = 0"     },
            { "RBS", "L", "90+180+0 = 270"          },   // default arm
            { "RBB", "R", "90+180+180 = 450 = 90"   },
            { "BBL", "L", "180+180+270 = 630 = 270" },   // default arm
            { "BBS", "S", "180+180+0 = 360 = 0"     },
            { "BBR", "R", "180+180+90 = 450 = 90"   },
            { "BBB", "B", "180+180+180 = 540 = 180" },
        };
        for (int i = 0; i < (int)(sizeof collapse_rows /
                                  sizeof collapse_rows[0]); i++) {
            char nm[96];
            snprintf(nm, sizeof nm, "collapse: %s -> %s  (%s)",
                     collapse_rows[i].in, collapse_rows[i].out,
                     collapse_rows[i].net);
            CHECK(nm, simplifies_to(collapse_rows[i].in,
                                    collapse_rows[i].out));
        }
    }

    // ------------------- mutation 4: the fixture itself, pinned.
    // replay_next's bounds guard replaced by a raw `return p->moves[idx];`,
    // and the suite stayed green. path_from() poisons the tail of the
    // buffer, which is what turns this file's three replay fencepost
    // blocks (the basic replay walk, the idx == len boundary, and the
    // fencepost family) from decoration into tests. That property lives
    // in a helper, and a helper is exactly the kind of code someone
    // "tidies" back to a plain zero-fill — at which point three checks
    // elsewhere go silently vacuous and nothing turns red. So the poison
    // is asserted HERE, where a tidy-up gets caught by name.
    {
        path_t p = path_from("LSR");
        CHECK("replay: the fixture's tail really is poison",
              PATH_POISON != MOVE_NONE &&
              p.moves[p.len]                == PATH_POISON &&
              p.moves[PATH_MAX_MOVES - 1]   == PATH_POISON);

        // And the fencepost itself, re-asked over a poisoned buffer:
        // every one of these answers now comes from replay_next's
        // `idx < p->len` guard, because the bytes underneath say 'Z'.
        CHECK("replay: past-the-end is the guard's answer, not the buffer's",
              replay_next(&p, p.len)                  == MOVE_NONE &&
              replay_next(&p, (uint8_t)(p.len + 1))   == MOVE_NONE &&
              replay_next(&p, PATH_MAX_MOVES - 1)     == MOVE_NONE);
    }

    printf("\n%d passed, %d failed\n", t_pass, t_fail);
    return t_fail != 0;
}
