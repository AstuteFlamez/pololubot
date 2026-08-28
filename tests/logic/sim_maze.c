// sim_maze.c — the maze as DATA: a table-driven whole-maze simulator.
//
// test_maze_logic.c proves each function of logic/maze_logic.c alone. This
// file proves them TOGETHER, by walking a whole maze through the same
// pipeline the robot runs on the floor:
//
//     classify → decide → record → simplify → replay
//
// at two levels of realism:
//
//   SYMBOL level — the walk hands decide_left_hand() ground-truth junction
//     types straight out of the topology table. This isolates the DECISION
//     chain: if this level fails, the bug is in decide/record/simplify/
//     replay, not in sensing.
//   SENSOR level — the walk synthesizes realistic before/at_center sensor
//     snapshots for each junction (the model is documented below) and
//     pushes them through a classifier first, exactly as main.c does. If
//     SYMBOL passes and SENSOR fails, the bug is in classification. The
//     sensor level runs TWICE, once through classify_junction() and once
//     through classify_junction_ref(). The firmware selects one of them
//     with USE_MY_CLASSIFIER; proving both on every run keeps the
//     unselected one runnable instead of letting it rot unnoticed.
//
// Reading a failure: every check name carries the maze and the level that
// produced it — "[harder] sensor replay(derived) reaches the goal" — so
// the name alone says which maze, which level and which stage broke. A
// walk that gives up first prints a line naming the node and heading
// where it stopped, and a misclassification prints what the classifier
// said next to what the topology says. The last line prints the totals,
// and the process exits nonzero if anything failed.
//
// The maze is a TABLE rather than code so that adding a maze is data, not
// new logic. The second maze below arrived as one more table and one more
// run_maze_suite() call, with the walker, the replayer and every
// assertion reused untouched.
//
// The walk DERIVES the explore and solved paths from the topology instead
// of asserting the literal "LLBLLBS", then compares the result against a
// literal worked out by hand from the maze drawing. A hard-coded expected
// string can only prove the code agrees with whoever typed it; deriving
// it cross-checks three things at once — the topology encoding, the hand
// walk, and the code. If any one of them is wrong, they cannot all agree.
//
// Both mazes are transcribed from their drawings as DESIGNED. No physical
// board has been taped and measured, so the tables and their expected
// paths are provisional: when a board exists, any drift between it and
// these tables must be reconciled INTO the tables first. A simulator that
// walks a different maze than the robot does is worse than no simulator.
//
// Pure host C on purpose: maze_logic.h, tuning.h (host-safe — nothing but
// #defines) and libc. No SDK.

#include <stdio.h>
#include <string.h>
#include "maze_logic.h"
#include "tuning.h"     // JCT_DARK_THRESH — the sensor model straddles it

static int t_pass, t_fail;

#define CHECK(name, cond)                                            \
    do {                                                             \
        if (cond) { t_pass++; printf("PASS  %s\n", (name)); }        \
        else      { t_fail++; printf("FAIL  %s   (%s:%d)\n",         \
                                     (name), __FILE__, __LINE__); }  \
    } while (0)

// Junction names for FAILURE REPORTS. maze_logic_ref.c already ships
// junction_name(), and the walks below keep using it — but it answers
// "???" for JCT_NONE. That is the right word on a 16-column OLED, where
// an unclassifiable arrival really is a shrug; it is the wrong word in a
// test report, where JCT_NONE is a specific and often EXPECTED verdict
// (the refusal gap), and "wanted ???" reads like the test is the confused
// one. Spelling the enum here costs nine lines and buys a diagnosis.
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

// The junction flavour of CHECK: same bookkeeping, but a failure prints
// what the classifier actually said. "wanted STRAIGHT_LEFT, got NONE"
// names the lost evidence; a bare FAIL line only names the fixture. Both
// arguments are evaluated exactly once — a macro that evaluates twice
// works until someone hands it a call with a side effect. This file and
// test_maze_logic.c each carry their own copy, the same way each carries
// its own path_from(): the two binaries are deliberately independent
// (the Makefile links them against different sources), and a shared test
// header would be one more thing to break.
#define CHECK_JCT(name, got, want)                                    \
    do {                                                              \
        junction_t got_ = (got), want_ = (want);                      \
        if (got_ == want_) { t_pass++; printf("PASS  %s\n", (name)); }\
        else { t_fail++;                                              \
               printf("FAIL  %s   (%s:%d)  wanted %s, got %s\n",      \
                      (name), __FILE__, __LINE__,                     \
                      jct_label(want_), jct_label(got_)); }           \
    } while (0)

// A byte that is not a move and — the load-bearing half — is not
// MOVE_NONE either. It marks buffer space the path does not own.
#define PATH_POISON 'Z'

// Build a path_t straight from a string literal (same helper shape as
// test_maze_logic.c — each binary stays self-contained), with the UNUSED
// tail of the buffer filled with POISON instead of zeros.
//
// Why the poison (the same defect one file over): MOVE_NONE is '\0', so
// the obvious initialiser `{ { 0 }, 0, false }` pre-loads all 64
// bytes with the exact sentinel replay is supposed to have to EARN. Delete
// the bounds guard in replay_next(), let it index the buffer raw, and a
// literal-built path hands back MOVE_NONE past its end anyway — the replay
// walk below sees "path spent", finds itself on the goal, and reports
// success. Measured before this change, with `return p->moves[idx];`
// substituted for the guarded return: the derived-path replays failed (they
// carry path_simplify's stale tail bytes, which is luck, not coverage) while
// `[starter] sym replay("LSR")` and `[harder] sym replay("SRLR")` — the two
// checks built from THIS helper — stayed green.
//
// Take the poison out and those two checks go back to passing on a
// replay_next with no boundary at all. With it in, the tail answers 'Z',
// replay_arrival_verdict calls that data rot, and the walk fails by name.
// A test that also passes on the broken code is decoration.
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

// The recorded moves are NOT nul-terminated (len is the truth), so print
// with an explicit length.
static void show_path(const char *label, const path_t *p)
{
    printf("  %-28s \"%.*s\"%s\n", label, (int)p->len, p->moves,
           p->overflow ? "  [OVERFLOW]" : "");
}

// ---------------------------------------------------------------------------
// SENSOR MODEL — how a junction turns into numbers.
//
// Calibrated scale: 0 = bright paper, 1000 = deep black tape. The model
// refuses 0/1000 caricatures on purpose: the maze build guarantees only
// "paper well under ~200, tape well over ~700". A classifier that only
// works on cartoon numbers would pass here and fail on the floor.
//
//   SIM_TAPE          — matte tape dead-on under a sensor. Past the
//                       ">700" line, comfortably above JCT_DARK_THRESH.
//   SIM_SHOULDER      — a sensor half-covered by the 19 mm line's edge
//                       (the ~40 mm bar puts ~2.5 sensors' worth on tape,
//                       so the line's neighbors ride its shoulders). MUST
//                       read "not dark": it is line, not a side branch.
//   SIM_PAPER         — clean posterboard, one fresh reading.
//   SIM_PAPER_PEAK    — paper's max over a whole latched sweep: dust and
//                       print grain peak higher than any single reading.
//   SIM_SHOULDER_PEAK — the shoulder's max over a latched sweep, same idea.
//
// The peaks exist because `before` is a per-sensor MAXIMUM latched across
// the junction sweep, while `at_center` is one fresh read (maze_logic.h
// documents when each is taken) — a max is always ≥ a sample, so the
// latched lanes sit a notch above their fresh cousins.
#define SIM_TAPE          830
#define SIM_SHOULDER      480
#define SIM_PAPER         60
#define SIM_PAPER_PEAK    150
#define SIM_SHOULDER_PEAK 520

// The model is only meaningful while JCT_DARK_THRESH sits in the gap it
// straddles. A hardware tuning session can move that knob, so if it ever
// leaves the gap, fail the BUILD rather than silently prove nothing. (The
// reference classifier's fixed 600 lives in the same gap — see
// maze_logic_ref.c.)
#if SIM_TAPE < JCT_DARK_THRESH
#error "sensor model: SIM_TAPE would no longer read as dark - revisit the model"
#endif
#if SIM_SHOULDER_PEAK >= JCT_DARK_THRESH
#error "sensor model: line shoulders would read as side branches - revisit the model"
#endif

// ---------------------------------------------------------------------------
// TOPOLOGY TABLES — the maze as data.
//
// The table stores ABSOLUTE compass arms per node. The robot's left,
// straight and right are computed from its current heading — the same
// arithmetic as turning a paper map to face the direction of travel.
// That is also why junction TYPE is not stored in the table: the same
// piece of tape is a T from below but a STRAIGHT+RIGHT from the west, so
// type must be a FUNCTION of (topology, approach heading), never a label.

enum { NORTH, EAST, SOUTH, WEST };
#define DIR_COUNT 4
static const char DIR_LETTER[DIR_COUNT] = { 'N', 'E', 'S', 'W' };

static int dir_left(int h)  { return (h + 3) % DIR_COUNT; }
static int dir_right(int h) { return (h + 1) % DIR_COUNT; }
static int dir_back(int h)  { return (h + 2) % DIR_COUNT; }

typedef struct {
    const char *name;           // the node's name on the maze drawing, so
                                // failure messages point at the diagram
    int8_t      nbr[DIR_COUNT]; // neighbor node per compass arm, -1 = no tape
    bool        is_goal;        // the ≥50×50 mm solid patch lives here
} sim_node_t;

typedef struct {
    const char       *name;
    const sim_node_t *nodes;
    uint8_t           start;           // node the robot is placed on at GO
    uint8_t           start_heading;   // compass direction it faces at GO
    const char       *explore_oracle;  // the hand-worked answers off the
    const char       *simplify_oracle; // drawing: the cross-check literals
} sim_maze_t;

// The starter maze, transcribed arm-for-arm from its drawing. That
// drawing is the PROVISIONAL oracle: it describes the maze as designed,
// and the physical maze has not been built and measured yet. When it is,
// any as-built drift is reconciled INTO this table first — a simulator
// that walks a different maze than the robot does is worse than no
// simulator:
//
//         X2─────F─────■ GOAL
//                 │
//         X1─────A
//                 │
//    S────────────C          (start; robot facing east →)
//
enum { N_S, N_C, N_A, N_F, N_X1, N_X2, N_G, STARTER_NODE_COUNT };

static const sim_node_t starter_nodes[STARTER_NODE_COUNT] = {
    //            N     E     S     W
    [N_S]  = { "S",    { -1,  N_C,  -1,  -1   }, false },
    [N_C]  = { "C",    { N_A, -1,   -1,  N_S  }, false },
    [N_A]  = { "A",    { N_F, -1,   N_C, N_X1 }, false },
    [N_F]  = { "F",    { -1,  N_G,  N_A, N_X2 }, false },
    [N_X1] = { "X1",   { -1,  N_A,  -1,  -1   }, false },
    [N_X2] = { "X2",   { -1,  N_F,  -1,  -1   }, false },
    [N_G]  = { "GOAL", { -1,  -1,   -1,  N_F  }, true  },
};

static const sim_maze_t starter_maze = {
    .name            = "starter",
    .nodes           = starter_nodes,
    .start           = N_S,
    .start_heading   = EAST,
    .explore_oracle  = "LLBLLBS",
    .simplify_oracle = "LSR",
};

// What junction is this, arriving at `node` while facing `heading`?
// Derived fresh from the table every time (see the type-is-a-function
// note above). The exit-combination → type map is the maze-side mirror of
// what any classifier must reconstruct from photons.
static junction_t junction_at(const sim_maze_t *mz, int node, int heading)
{
    const sim_node_t *nd = &mz->nodes[node];
    if (nd->is_goal) { return JCT_GOAL; }
    bool l = nd->nbr[dir_left(heading)]  >= 0;
    bool s = nd->nbr[heading]            >= 0;
    bool r = nd->nbr[dir_right(heading)] >= 0;
    if (l && r) { return s ? JCT_CROSS          : JCT_T; }
    if (l)      { return s ? JCT_STRAIGHT_LEFT  : JCT_LEFT_ONLY; }
    if (r)      { return s ? JCT_STRAIGHT_RIGHT : JCT_RIGHT_ONLY; }
    if (s) {
        // Straight-only is not a junction — the detector never fires on a
        // plain corridor. Reaching this means the TABLE is malformed.
        return JCT_NONE;
    }
    return JCT_DEAD_END;
}

// Build the two snapshots the drive layer would hand a classifier at this
// junction. The physical story (maze_logic.h documents the timing):
//   `before`    — per-sensor max latched while the bar sweeps the junction.
//                 A full-width side arm drags its side's two sensors to
//                 tape-dark for a few ms and the latch keeps that; sensors
//                 that saw no arm keep only their following-time peaks.
//                 The entry line itself keeps s2 dark the whole way in.
//   `at_center` — one fresh read after the CREEP_MM roll: the bar is PAST
//                 the junction, so it sees only what continues — the
//                 straight exit's line (center dark, shoulders half-lit),
//                 the goal patch (everything dark), or bare paper.
static void synth_snapshots(junction_t truth, bool l, bool s, bool r,
                            sensor_snapshot_t *before,
                            sensor_snapshot_t *at_center)
{
    if (truth == JCT_GOAL) {
        // A straight ≥150 mm approach into a ≥75 mm patch: the bar is on
        // solid black before the creep AND still on it after. The patch is
        // sized for exactly that — 75 mm of patch against the 45 mm creep
        // leaves 30 mm of margin.
        for (int i = 0; i < 5; i++) { before->s[i]    = SIM_TAPE; }
        for (int i = 0; i < 5; i++) { at_center->s[i] = SIM_TAPE; }
        return;
    }
    if (truth == JCT_DEAD_END) {
        // The honest dead end never fires the junction detector — the
        // line just ends, the loss path confirms whiteness, and the drive
        // fills `before` with what the bar sees: paper everywhere. The
        // post-creep read is paper too.
        for (int i = 0; i < 5; i++) { before->s[i]    = SIM_PAPER_PEAK; }
        for (int i = 0; i < 5; i++) { at_center->s[i] = SIM_PAPER; }
        return;
    }

    // An ordinary junction. `before`: entry line under s2 all the way in;
    // each side pair goes tape-dark only if that arm exists.
    before->s[2] = SIM_TAPE;
    if (l) { before->s[0] = SIM_TAPE;       before->s[1] = SIM_TAPE; }
    else   { before->s[0] = SIM_PAPER_PEAK; before->s[1] = SIM_SHOULDER_PEAK; }
    if (r) { before->s[4] = SIM_TAPE;       before->s[3] = SIM_TAPE; }
    else   { before->s[4] = SIM_PAPER_PEAK; before->s[3] = SIM_SHOULDER_PEAK; }

    // `at_center`: only the straight exit can put line under the bar here
    // — that asymmetry is the entire reason two snapshots exist.
    if (s) {
        at_center->s[0] = SIM_PAPER;
        at_center->s[1] = SIM_SHOULDER;
        at_center->s[2] = SIM_TAPE;
        at_center->s[3] = SIM_SHOULDER;
        at_center->s[4] = SIM_PAPER;
    } else {
        for (int i = 0; i < 5; i++) { at_center->s[i] = SIM_PAPER; }
    }
}

// ---------------------------------------------------------------------------
// THE WALKS. `clf == NULL` selects the SYMBOL level (ground truth straight
// into the decision chain); otherwise every arrival goes photons-first
// through `clf`, exactly like main.c's classify seam.

typedef junction_t (*classifier_fn)(const sensor_snapshot_t *,
                                    const sensor_snapshot_t *);

// A correct left-hand walk of the starter tree takes 8 arcs. The cap means
// a broken brain FAILS the test instead of hanging it — the stub decide
// answers 'B' everywhere and would bounce between two nodes forever.
#define SIM_MAX_STEPS 64

// Classify one arrival, at whichever level, and cross-check the sensor
// path against ground truth: a classifier that disagrees with the maze is
// a fault, never something to paper over (main.c stops on it too).
static junction_t arrive(const sim_maze_t *mz, int node, int h,
                         classifier_fn clf, bool *ok)
{
    junction_t truth = junction_at(mz, node, h);
    *ok = true;
    if (clf == NULL) { return truth; }

    const sim_node_t *nd = &mz->nodes[node];
    sensor_snapshot_t before, at_center;
    synth_snapshots(truth,
                    nd->nbr[dir_left(h)]  >= 0,
                    nd->nbr[h]            >= 0,
                    nd->nbr[dir_right(h)] >= 0,
                    &before, &at_center);
    junction_t j = clf(&before, &at_center);
    if (j != truth) {
        printf("  sim(%s): at %s (heading %c) classified %s, topology says %s\n",
               mz->name, nd->name, DIR_LETTER[h],
               junction_name(j), junction_name(truth));
        *ok = false;
    }
    return j;
}

// The exploring walk: drive an arc, classify the arrival, ask the
// left-hand rule, ask the arrival verdict, record the answer, turn,
// repeat — the exact loop the explore seam runs on the robot. Returns
// true iff the goal was reached; the recorded route is in *out.
//
// "Exact" is literal, not aspirational: the verdict comes from the SAME
// explore_arrival_verdict() the firmware calls, so a policy change cannot
// land on one side only. This walk used to re-implement the screens by
// hand and had already drifted from the firmware in three cells. The cure
// for that is DELETING the second copy, not re-synchronizing it by hand.
static bool explore_maze(const sim_maze_t *mz, classifier_fn clf, path_t *out)
{
    int node = mz->start;
    int h    = mz->start_heading;

    for (int step = 0; step < SIM_MAX_STEPS; step++) {
        int next = mz->nodes[node].nbr[h];
        if (next < 0) {
            printf("  sim(%s): drove off the tape leaving %s heading %c\n",
                   mz->name, mz->nodes[node].name, DIR_LETTER[h]);
            return false;
        }
        node = next;

        // The simulator's own check, and it stays UPSTREAM of the
        // verdict: the robot cannot compare its classifier against maze
        // truth, but the simulator can, and catching a misread before
        // the decision layer sees it is the whole point of the sensor
        // level. The verdict below judges the same arrival the firmware
        // would judge.
        bool ok;
        junction_t j = arrive(mz, node, h, clf, &ok);
        if (!ok) { return false; }

        move_t m = decide_left_hand(j);
        arrival_verdict_t v = explore_arrival_verdict(j, m);
        if (v == ARRIVE_SUCCESS) { return true; }  // GOAL ends the run and
                                                   // is never recorded
        if (v != ARRIVE_PROCEED) {
            // Same codes as main.c, different voice: the robot has 16
            // OLED columns, while a failing host run has a terminal and
            // should read like a bug report. Keeping the wording out of
            // logic/ is what lets one pure function serve both.
            switch (v) {
            case ARRIVE_FAULT_CLASSIFY_NONE:
                printf("  sim(%s): JCT_NONE at %s — stopping, not guessing\n",
                       mz->name, mz->nodes[node].name);
                break;
            case ARRIVE_FAULT_ILLEGAL_MOVE:
                printf("  sim(%s): decide returned 0x%02x at %s (%s) — not a move\n",
                       mz->name, (unsigned)(unsigned char)m,
                       mz->nodes[node].name, junction_name(j));
                break;
            default:
                // The replay-only codes cannot come back from the explore
                // twin. Print the number rather than guessing a sentence:
                // an unexpected verdict is a bug in the twin itself.
                printf("  sim(%s): explore verdict %d at %s (%s) — not an "
                       "explore verdict\n", mz->name, (int)v,
                       mz->nodes[node].name, junction_name(j));
                break;
            }
            return false;
        }

        path_record(out, m);
        if (out->overflow) {
            // Junction #65 didn't fit, so the route is now a lie and can
            // never be replayed — main.c stops the run here too. The
            // simulator got away without this check only because
            // SIM_MAX_STEPS happens to equal PATH_MAX_MOVES; move either
            // constant and the missing stop becomes a silent wrong answer.
            printf("  sim(%s): path overflow at %s — a truncated route can "
                   "never be replayed\n", mz->name, mz->nodes[node].name);
            return false;
        }

        // The verdict already certified the byte, so the default is
        // unreachable belt (main.c's execute_move keeps the same one for
        // the same reason: a switch over a char must stay total).
        switch (m) {
        case MOVE_LEFT:     h = dir_left(h);  break;
        case MOVE_RIGHT:    h = dir_right(h); break;
        case MOVE_BACK:     h = dir_back(h);  break;
        case MOVE_STRAIGHT: break;
        default:
            printf("  sim(%s): 0x%02x passed the legality screen at %s — the "
                   "verdict itself is broken\n", mz->name,
                   (unsigned)(unsigned char)m, mz->nodes[node].name);
            return false;
        }
    }
    printf("  sim(%s): %d arcs and never reached the goal\n",
           mz->name, SIM_MAX_STEPS);
    return false;
}

// The replay walk: junction-by-junction, ARRIVAL-indexed — junction #idx
// is junction #idx no matter how the tape stretches or the wheels slip.
// Boundary contract under test (maze_logic.c's replay_next block):
// MOVE_NONE appears exactly when the path is exhausted, and the arrival
// that sees MOVE_NONE must BE the goal. Both fenceposts are caught here:
// reaching the goal with moves left over, and running out of moves while
// standing on an ordinary junction. Judging them is replay_arrival_verdict()
// — the firmware's own policy, called from the firmware's own copy, so
// "the simulator agrees with the robot" is a fact about the code rather
// than a promise about two hand transcriptions.
static bool replay_maze(const sim_maze_t *mz, classifier_fn clf,
                        const path_t *path)
{
    int node    = mz->start;
    int h       = mz->start_heading;
    uint8_t idx = 0;

    for (int step = 0; step < SIM_MAX_STEPS; step++) {
        int next = mz->nodes[node].nbr[h];
        if (next < 0) {
            printf("  sim(%s): replay drove off the tape leaving %s heading %c\n",
                   mz->name, mz->nodes[node].name, DIR_LETTER[h]);
            return false;
        }
        node = next;

        bool ok;
        junction_t j = arrive(mz, node, h, clf, &ok);
        if (!ok) { return false; }  // classifier vs expected arrival = fault,
                                    // and it stays UPSTREAM of the verdict

        move_t m = replay_next(path, idx);
        arrival_verdict_t v = replay_arrival_verdict(j, m);
        if (v == ARRIVE_SUCCESS) { return true; }   // path spent ON the goal
        if (v != ARRIVE_PROCEED) {
            // The firmware's five fault cells, in the simulator's voice.
            // Two of them arrived with the shared verdict function: this
            // walk used to keep driving through a JCT_NONE with moves
            // left, and used to merge the refusal case into "moves ran
            // out, not the goal". The firmware always split them, and now
            // there is only one table to split.
            switch (v) {
            case ARRIVE_FAULT_NOT_GOAL:
                printf("  sim(%s): moves ran out at %s (%s), not the goal\n",
                       mz->name, mz->nodes[node].name, junction_name(j));
                break;
            case ARRIVE_FAULT_CLASSIFY_NONE:
                printf("  sim(%s): JCT_NONE at %s — stopping, not guessing\n",
                       mz->name, mz->nodes[node].name);
                break;
            case ARRIVE_FAULT_GOAL_EARLY:
                printf("  sim(%s): reached the goal with %d moves left\n",
                       mz->name, path->len - idx);
                break;
            case ARRIVE_FAULT_DEAD_END:
                printf("  sim(%s): replay entered dead end %s — a detour survived simplify\n",
                       mz->name, mz->nodes[node].name);
                break;
            case ARRIVE_FAULT_ILLEGAL_MOVE:
                printf("  sim(%s): replay_next returned 0x%02x at %s — not a move\n",
                       mz->name, (unsigned)(unsigned char)m,
                       mz->nodes[node].name);
                break;
            default:
                printf("  sim(%s): replay verdict %d at %s (%s) — unexpected\n",
                       mz->name, (int)v, mz->nodes[node].name,
                       junction_name(j));
                break;
            }
            return false;
        }

        // Certified by the verdict's legality column — the default is the
        // same unreachable belt main.c's execute_move keeps.
        switch (m) {
        case MOVE_LEFT:     h = dir_left(h);  break;
        case MOVE_RIGHT:    h = dir_right(h); break;
        case MOVE_BACK:     h = dir_back(h);  break;
        case MOVE_STRAIGHT: break;
        default:
            printf("  sim(%s): 0x%02x passed the legality screen at %s — the "
                   "verdict itself is broken\n", mz->name,
                   (unsigned)(unsigned char)m, mz->nodes[node].name);
            return false;
        }
        idx++;
    }
    printf("  sim(%s): replay ran %d arcs without finishing\n",
           mz->name, SIM_MAX_STEPS);
    return false;
}

// ---------------------------------------------------------------------------
// One maze, the whole story. The second maze reruns every check below via
// its own table and one more call in main() — nothing in this function
// knows or cares which maze it is proving.
static void run_maze_suite(const sim_maze_t *mz)
{
    char nm[96];
    printf("== %s maze (oracle: %s -> %s) ==\n",
           mz->name, mz->explore_oracle, mz->simplify_oracle);

    // ---------------- SYMBOL level: the decision chain in isolation.
    path_t explored = { { 0 }, 0, false };
    bool reached = explore_maze(mz, NULL, &explored);
    show_path("sym derived explore:", &explored);

    snprintf(nm, sizeof nm, "[%s] sym explore reaches the goal", mz->name);
    CHECK(nm, reached);
    // The cross-check: the topology-derived string must equal the
    // hand-worked literal off the drawing. If the table were encoded
    // wrong, the two could not both be right.
    snprintf(nm, sizeof nm, "[%s] sym explore derives \"%s\"",
             mz->name, mz->explore_oracle);
    CHECK(nm, reached && path_is(&explored, mz->explore_oracle));

    // Chained: simplify what the walk itself recorded...
    path_t solved = explored;
    path_simplify(&solved);
    show_path("sym derived simplified:", &solved);
    snprintf(nm, sizeof nm, "[%s] sym simplify(derived) == \"%s\"",
             mz->name, mz->simplify_oracle);
    CHECK(nm, path_is(&solved, mz->simplify_oracle));

    // ...and independent: simplify the oracle literal, so this stage still
    // means something while the explore stage is broken (triage which
    // station to blame).
    {
        path_t lit = path_from(mz->explore_oracle);
        path_simplify(&lit);
        snprintf(nm, sizeof nm, "[%s] sym simplify(\"%s\") == \"%s\"",
                 mz->name, mz->explore_oracle, mz->simplify_oracle);
        CHECK(nm, path_is(&lit, mz->simplify_oracle));
    }

    // Chained: replay the derived-and-simplified route over the same
    // topology it came from — the full circle.
    snprintf(nm, sizeof nm, "[%s] sym replay(derived) reaches the goal",
             mz->name);
    CHECK(nm, replay_maze(mz, NULL, &solved));

    // Independent: replay the oracle literal.
    {
        path_t lit = path_from(mz->simplify_oracle);
        snprintf(nm, sizeof nm, "[%s] sym replay(\"%s\") reaches the goal",
                 mz->name, mz->simplify_oracle);
        CHECK(nm, replay_maze(mz, NULL, &lit));
    }

    // ---------------- SENSOR level, classify_junction: the full chain —
    // photons → classify_junction → decide → record → simplify → replay.
    {
        path_t exp2 = { { 0 }, 0, false };
        bool r2 = explore_maze(mz, classify_junction, &exp2);
        show_path("sensor derived explore:", &exp2);
        snprintf(nm, sizeof nm, "[%s] sensor explore derives \"%s\"",
                 mz->name, mz->explore_oracle);
        CHECK(nm, r2 && path_is(&exp2, mz->explore_oracle));

        path_t sol2 = exp2;
        path_simplify(&sol2);
        snprintf(nm, sizeof nm, "[%s] sensor simplify(derived) == \"%s\"",
                 mz->name, mz->simplify_oracle);
        CHECK(nm, path_is(&sol2, mz->simplify_oracle));

        snprintf(nm, sizeof nm, "[%s] sensor replay(derived) reaches the goal",
                 mz->name);
        CHECK(nm, replay_maze(mz, classify_junction, &sol2));
    }

    // ---------------- SENSOR level, the reference classifier: the floor.
    // The robot must stay solvable with USE_MY_CLASSIFIER commented out,
    // so proving that path on every run keeps it from rotting.
    {
        path_t exp3 = { { 0 }, 0, false };
        bool r3 = explore_maze(mz, classify_junction_ref, &exp3);
        snprintf(nm, sizeof nm, "[%s] reference-classifier explore derives \"%s\"",
                 mz->name, mz->explore_oracle);
        CHECK(nm, r3 && path_is(&exp3, mz->explore_oracle));

        path_t sol3 = exp3;
        path_simplify(&sol3);
        snprintf(nm, sizeof nm, "[%s] reference-classifier simplify(derived) == \"%s\"",
                 mz->name, mz->simplify_oracle);
        CHECK(nm, path_is(&sol3, mz->simplify_oracle));

        snprintf(nm, sizeof nm, "[%s] reference-classifier replay(derived) reaches the goal",
                 mz->name);
        CHECK(nm, replay_maze(mz, classify_junction_ref, &sol3));
    }
}

// ---------------------------------------------------------------------------
// THE HARDER MAZE — two boards, six junctions, five dead ends, one
// 4-way. Transcribed arm-for-arm from its drawing, with the same
// provisional-oracle caveat as the starter table above: as designed, not
// as built, and the as-built measurements reconcile into here the day
// they exist:
//
//                 F─────X5
//                 │
//         X4─────D       E─────■ GOAL
//                 │       │
//                 B──────C─────X3
//                 │       │
//         X1─────A       X2
//                 │
//                 S  (start; robot facing north ↑)
//
// C is the only true 4-way crossing in either maze — the junction the
// GOAL-vs-CROSS margin exists for. The layout is still a TREE, which is
// the left-hand rule's entire warranty (see decide_left_hand's contract).
// Fourteen junction events before the goal: run_maze_suite() replays its
// full 12-check suite over this table exactly as it did the starter's.
enum { H_S, H_A, H_B, H_C, H_D, H_E, H_F,
       H_X1, H_X2, H_X3, H_X4, H_X5, H_G, HARDER_NODE_COUNT };

static const sim_node_t harder_nodes[HARDER_NODE_COUNT] = {
    //              N      E      S     W
    [H_S]  = { "S",    { H_A,  -1,    -1,   -1   }, false },
    [H_A]  = { "A",    { H_B,  -1,    H_S,  H_X1 }, false },
    [H_B]  = { "B",    { H_D,  H_C,   H_A,  -1   }, false },
    [H_C]  = { "C",    { H_E,  H_X3,  H_X2, H_B  }, false },  // the only 4-way
    [H_D]  = { "D",    { H_F,  -1,    H_B,  H_X4 }, false },
    [H_E]  = { "E",    { -1,   H_G,   H_C,  -1   }, false },
    [H_F]  = { "F",    { -1,   H_X5,  H_D,  -1   }, false },
    [H_X1] = { "X1",   { -1,   H_A,   -1,   -1   }, false },
    [H_X2] = { "X2",   { H_C,  -1,    -1,   -1   }, false },
    [H_X3] = { "X3",   { -1,   -1,    -1,   H_C  }, false },
    [H_X4] = { "X4",   { -1,   H_D,   -1,   -1   }, false },
    [H_X5] = { "X5",   { -1,   -1,    -1,   H_F  }, false },
    [H_G]  = { "GOAL", { -1,   -1,    -1,   H_E  }, true  },
};

static const sim_maze_t harder_maze = {
    .name            = "harder",
    .nodes           = harder_nodes,
    .start           = H_S,
    .start_heading   = NORTH,
    .explore_oracle  = "LBLSLBLRBLSLLR",  // the 14-move hand walk
    .simplify_oracle = "SRLR",
};

// ---------------------------------------------------------------------------
// THE MARGINAL WORLD — the sensor model after a bad week.
//
// The clean palette above is the maze on build day. This one is the same
// maze after wear: matte tape gone shiny and dusty reads LIGHTER, while
// scuffed paper and a dirt-caked half-covered shoulder sensor read
// DARKER. Every reading drifts TOWARD the threshold, which is the
// direction that kills classifiers. The drift stops short of crossing it
// — tape that truly reads light is failed hardware and no classifier can
// save it — but it leaves about 40 counts of margin where the clean
// palette had about 230. Anything a classifier was getting away with on
// clean tape shows up here.
//
//   SIM_TAPE_WORN — the worst still-legible "dark": worn tape, at or
//                   above both classifiers' thresholds.
//   SIM_SCUFF     — the worst still-legible "light": grimy paper or a
//                   dirt-hot shoulder, still below both.
//
// "Both" is load-bearing: classify_junction reads JCT_DARK_THRESH from
// tuning.h, while classify_junction_ref reads its own fixed REF_DARK.
// REF_DARK is deliberately file-local to maze_logic_ref.c, so SIM_REF_DARK
// mirrors it here and the guards below re-check the whole palette against
// both thresholds. Same fail-the-build treatment as the clean palette's
// guards above: a model that quietly stopped straddling one threshold
// would keep passing while proving nothing.
#define SIM_TAPE_WORN 640
#define SIM_SCUFF     560
#define SIM_REF_DARK  600   // mirror of maze_logic_ref.c's REF_DARK,
                            // verified against the source below

// THE MIRROR, MECHANIZED. The line above used to be a hand copy that
// nothing compared to anything: moving REF_DARK from 600 to 700 in
// maze_logic_ref.c left the whole suite green, because the preprocessor
// never saw the real value — only this typed duplicate of it. The
// Makefile now extracts the literal straight out of maze_logic_ref.c and
// passes it in as REF_DARK_FROM_SOURCE, so the two numbers meet at
// compile time and any drift fails the BUILD.
//
// The general shape of the problem is worth carrying past this file: when
// a constant cannot be shared (no header, and maze_logic.h is a frozen
// interface), the choice is between a comment asking a human to remember
// and a build step making the machine check. Both cost about six lines.
// Only one of them still works in six months.
//
// One #if/#elif chain, not two independent guards, and the reason is a
// preprocessor detail: an UNDEFINED macro evaluates to 0 inside `#if`, so
// a hand build with no -D would trip the "is not defined" belt and then
// trip the mismatch test as well (600 != 0), printing a second error that
// blames a drifted mirror when nothing has drifted. `#elif` asks the
// second question only once the first has an answer, so each build
// failure names its own cause.
#ifndef REF_DARK_FROM_SOURCE
#error "REF_DARK_FROM_SOURCE is not defined: build this file through tests/logic/Makefile, which extracts REF_DARK from logic/maze_logic_ref.c so the SIM_REF_DARK mirror below can be verified instead of merely trusted."
#elif SIM_REF_DARK != REF_DARK_FROM_SOURCE
#error "SIM_REF_DARK no longer matches REF_DARK in logic/maze_logic_ref.c. That mirror is not decoration: the marginal palette below must straddle BOTH thresholds - classify_junction reads JCT_DARK_THRESH, classify_junction_ref reads REF_DARK - and the two guards under this line prove worn tape still reads dark and scuff still reads light for BOTH. Update the mirror to the new value, then re-read those guards and the reference band-edge checks at the bottom of this file: moving REF_DARK can take the reference classifier out of the model band without changing a single test that only walks clean tape."
#endif

#if SIM_TAPE_WORN < JCT_DARK_THRESH || SIM_TAPE_WORN < SIM_REF_DARK
#error "marginal model: worn tape would no longer read dark - revisit the drift model"
#endif
#if SIM_SCUFF >= JCT_DARK_THRESH || SIM_SCUFF >= SIM_REF_DARK
#error "marginal model: scuff would read as line - revisit the drift model"
#endif

// Build the two snapshots for one junction of the marginal world. Same
// physics as synth_snapshots, drift palette. DEAD_END is deliberately NOT
// modeled: a dead end's `before` comes from the drive layer's loss path,
// which only fires after every sensor drops under LINE_LOST_THRESH (300)
// — paper grimy enough to read 560 breaks loss DETECTION long before it
// breaks classification, and that failure belongs to a recalibration
// session, not to this suite.
static void marginal_snapshots(junction_t truth, bool l, bool s, bool r,
                               sensor_snapshot_t *before,
                               sensor_snapshot_t *at_center)
{
    if (truth == JCT_GOAL) {
        for (int i = 0; i < 5; i++) { before->s[i]    = SIM_TAPE_WORN; }
        for (int i = 0; i < 5; i++) { at_center->s[i] = SIM_TAPE_WORN; }
        return;
    }
    before->s[2] = SIM_TAPE_WORN;
    if (l) { before->s[0] = SIM_TAPE_WORN; before->s[1] = SIM_TAPE_WORN; }
    else   { before->s[0] = SIM_SCUFF;     before->s[1] = SIM_SCUFF;     }
    if (r) { before->s[4] = SIM_TAPE_WORN; before->s[3] = SIM_TAPE_WORN; }
    else   { before->s[4] = SIM_SCUFF;     before->s[3] = SIM_SCUFF;     }
    if (s) {
        at_center->s[0] = SIM_SCUFF;
        at_center->s[1] = SIM_SCUFF;
        at_center->s[2] = SIM_TAPE_WORN;
        at_center->s[3] = SIM_SCUFF;
        at_center->s[4] = SIM_SCUFF;
    } else {
        for (int i = 0; i < 5; i++) { at_center->s[i] = SIM_SCUFF; }
    }
}

// The marginal run: every junction type the "never wrong on GOAL vs CROSS
// vs T" clause covers, in the worst legible world — classify_junction
// only. classify_junction_ref is exempt BY CONTRACT, not by omission:
// maze_logic_ref.c's own header bills it as "deliberately crude: one
// hard-coded threshold, no tuning.h knobs, no margin logic, no refusal
// gap beyond one case". Its job is to be the clean-model floor, which the
// whole-maze walks above hold it to; asserting margin behavior against a
// file that disclaims having any would test the test, not the code.
static void run_marginal_suite(void)
{
    printf("== marginal world (drift palette %d dark / %d light, thresh %d) ==\n",
           SIM_TAPE_WORN, SIM_SCUFF, JCT_DARK_THRESH);

    // First: worn and scuffed, but UNAMBIGUOUS. Every reading sits on its
    // correct side of the threshold, so the verdicts must not move an
    // inch — drift alone is never an excuse to misread a junction.
    static const struct {
        const char *name;
        junction_t  truth;
        bool        l, s, r;
    } cases[] = {
        { "[marginal] LEFT_ONLY still LEFT_ONLY",   JCT_LEFT_ONLY,      true,  false, false },
        { "[marginal] RIGHT_ONLY still RIGHT_ONLY", JCT_RIGHT_ONLY,     false, false, true  },
        { "[marginal] STR+LEFT still STR+LEFT",     JCT_STRAIGHT_LEFT,  true,  true,  false },
        { "[marginal] STR+RIGHT still STR+RIGHT",   JCT_STRAIGHT_RIGHT, false, true,  true  },
        { "[marginal] T still T",                   JCT_T,              true,  false, true  },
        { "[marginal] CROSS still CROSS",           JCT_CROSS,          true,  true,  true  },
        { "[marginal] GOAL still GOAL",             JCT_GOAL,           false, false, false },
    };
    for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        sensor_snapshot_t before, at_center;
        marginal_snapshots(cases[i].truth, cases[i].l, cases[i].s,
                           cases[i].r, &before, &at_center);
        CHECK_JCT(cases[i].name,
                  classify_junction(&before, &at_center), cases[i].truth);
    }

    // Then: the GOAL↔CROSS confusion surface, walked one dark sensor at
    // a time. `before` is all worn tape on every rung — on approach a
    // goal and a crossing are IDENTICAL, so at_center's dark count is
    // the only evidence in the case file:
    //   1–2 dark = a crossing's straight exit  (rung 1 = the loop above)
    //   4–5 dark = the goal patch              (rung 5 = the loop above)
    //   3 dark   = the gap between the signatures — see the last rung.
    sensor_snapshot_t worn_before;
    for (int i = 0; i < 5; i++) { worn_before.s[i] = SIM_TAPE_WORN; }

    {
        // Rung 2: exit line under s1–s2 (off-center stop) — still nothing
        // like a goal patch. Must stay CROSS.
        sensor_snapshot_t c = { { SIM_SCUFF, SIM_TAPE_WORN, SIM_TAPE_WORN,
                                  SIM_SCUFF, SIM_SCUFF } };
        CHECK_JCT("[marginal] 2-dark at_center stays CROSS",
                  classify_junction(&worn_before, &c), JCT_CROSS);
    }
    {
        // Rung 4: the goal patch with one sensor riding a dusty patch
        // edge. GOAL_MIN_DARK = 4 exists precisely so this stays GOAL.
        sensor_snapshot_t c = { { SIM_TAPE_WORN, SIM_TAPE_WORN,
                                  SIM_TAPE_WORN, SIM_TAPE_WORN,
                                  SIM_SCUFF } };
        CHECK_JCT("[marginal] 4-dark at_center stays GOAL",
                  classify_junction(&worn_before, &c), JCT_GOAL);
    }
    {
        // Rung 3 — the fixture that forces "I can't tell". These exact
        // bytes have TWO true stories:
        //   (a) the goal patch, arrived slightly skewed, both outer
        //       sensors hanging past its edge onto scuffed board;
        //   (b) a crossing whose straight exit darkens s2 while BOTH
        //       shoulders ride packed dirt up into the worn-tape band.
        // Same evidence, different ground truths: any classifier that
        // names GOAL or CROSS here is WRONG in the other world, and
        // "never wrong on GOAL vs CROSS vs T" is the contract's one
        // absolute. (T is just as wrong — both stories have line under
        // the bar past the junction.) The only honest verdict is
        // JCT_NONE: main.c stops on it and shows the fault, instead of
        // turning left off the goal patch at full confidence.
        sensor_snapshot_t c = { { SIM_SCUFF, SIM_TAPE_WORN, SIM_TAPE_WORN,
                                  SIM_TAPE_WORN, SIM_SCUFF } };
        CHECK_JCT("[marginal] 3-dark at_center is the gap -> JCT_NONE",
                  classify_junction(&worn_before, &c), JCT_NONE);
    }
}

// ---------------------------------------------------------------------------
// THE REFERENCE CLASSIFIER, UP CLOSE.
//
// Everything above proves classify_junction_ref the way the ROBOT uses it
// — whole-maze walks over the clean palette. That is a coarse net, and it
// was measured: moving REF_DARK from 600 to 700 in maze_logic_ref.c (a
// 100-count drift in that classifier's only threshold) left every one of
// the simulator's checks green. The clean palette straddles both numbers
// — tape 830 dark, shoulder peak 520 light — so no junction in either
// maze changed its answer.
//
// A threshold nobody pins is a threshold that can drift until the day it
// lands on a real reading. The two blocks below pin it from different
// sides: where the band edge IS, and whether the two classifiers still
// read the same photons the same way.
static void run_ref_lane_suite(void)
{
    printf("== reference classifier up close (classify_junction_ref) ==\n");

    // ---- (1) THE BAND EDGE. maze_logic_ref.c contains exactly one
    // number — REF_DARK, hard-coded at 600, deliberately: no knobs and no
    // margin logic, so the whole classifier reads in one sitting and
    // diffs cleanly against the real one. Nothing pinned WHERE that
    // number sits, so this pair straddles it on the outermost left
    // sensor: at 600 the latched arm counts as a branch, one count lower
    // the arm evaporates and the evidence fits nothing at all — that
    // classifier's single refusal case.
    //
    // The literals 600/599 are on purpose. The reference classifier
    // exports no knob to spell them with (that IS the file's contract),
    // so a test must name the number. If the number ever moves, the
    // build-time mirror guard above fails FIRST and points at
    // maze_logic_ref.c, and then these two lines say what the move
    // costs.
    {
        sensor_snapshot_t center = { { 60, 480, 830, 480, 60 } };
        sensor_snapshot_t on     = { { 600, 830, 830, 150, 150 } };
        sensor_snapshot_t off    = { { 599, 830, 830, 150, 150 } };
        CHECK_JCT("[ref] s0 == 600 (REF_DARK, dark side) -> STRAIGHT_LEFT",
                  classify_junction_ref(&on, &center), JCT_STRAIGHT_LEFT);
        CHECK_JCT("[ref] s0 == 599 (one count light) -> JCT_NONE",
                  classify_junction_ref(&off, &center), JCT_NONE);
    }

    // ---- (2) HEAD TO HEAD: the same photons into both classifiers, one
    // check per junction type, over the CLEAN palette.
    //
    // Each is checked against the TOPOLOGY's own answer rather than just
    // against each other: two classifiers that agree can still be wrong
    // together, and "they match" would be a green light for that. Both
    // matching the truth makes the agreement a corollary — and the
    // failure line prints both answers, so a disagreement names which
    // lane moved.
    //
    // CLEAN ONLY, and the exclusion is the point. On the MARGINAL
    // palette (run_marginal_suite, above) the two classifiers diverge BY
    // CONTRACT: the 3-dark rung is the refusal gap, where classify_junction
    // answers JCT_NONE because the same bytes have two true stories, while
    // classify_junction_ref — with no "I can't tell" in its vocabulary —
    // confidently answers CROSS. That divergence is the entire value the
    // real classifier adds over the baseline. An agreement assertion over
    // there would not catch a bug; it would demand that classify_junction
    // get DUMBER to keep the test green, which is how a test suite starts
    // steering the design the wrong way.
    static const struct {
        junction_t truth;
        bool       l, s, r;
    } clean_cases[] = {
        { JCT_LEFT_ONLY,      true,  false, false },
        { JCT_RIGHT_ONLY,     false, false, true  },
        { JCT_STRAIGHT_LEFT,  true,  true,  false },
        { JCT_STRAIGHT_RIGHT, false, true,  true  },
        { JCT_T,              true,  false, true  },
        { JCT_CROSS,          true,  true,  true  },
        { JCT_DEAD_END,       false, false, false },
        { JCT_GOAL,           false, false, false },
    };
    for (size_t i = 0; i < sizeof clean_cases / sizeof clean_cases[0]; i++) {
        sensor_snapshot_t before, at_center;
        synth_snapshots(clean_cases[i].truth, clean_cases[i].l,
                        clean_cases[i].s, clean_cases[i].r,
                        &before, &at_center);
        junction_t mine = classify_junction(&before, &at_center);
        junction_t ref  = classify_junction_ref(&before, &at_center);

        char nm[96];
        snprintf(nm, sizeof nm,
                 "[head-to-head] %s: primary and reference agree (clean)",
                 jct_label(clean_cases[i].truth));
        if (mine != clean_cases[i].truth || ref != clean_cases[i].truth) {
            printf("  head-to-head at %s: primary said %s, reference said %s\n",
                   jct_label(clean_cases[i].truth), jct_label(mine),
                   jct_label(ref));
        }
        CHECK(nm, mine == clean_cases[i].truth &&
                  ref  == clean_cases[i].truth);
    }
}

int main(void)
{
    run_maze_suite(&starter_maze);
    run_maze_suite(&harder_maze);
    run_marginal_suite();           // the same walks on worn tape
    run_ref_lane_suite();           // the reference classifier's threshold

    printf("\n%d passed, %d failed\n", t_pass, t_fail);
    return t_fail != 0;
}
