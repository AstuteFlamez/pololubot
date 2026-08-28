// maze_logic.h — the PURE side of the seam.
//
// Everything declared here is plain C over plain structs: no registers, no
// SDK, no timing, no I/O. That is a design decision, not an accident — it
// means this exact code compiles on the robot (arm-none-eabi-gcc) AND on
// your laptop (plain gcc), where host_tests/ can prove it correct before
// a single wheel turns. See host_tests/Makefile.
//
// The hardware side's whole job is to hand these functions two clean
// sensor snapshots and then obey the moves they return.
#ifndef MAZE_LOGIC_H
#define MAZE_LOGIC_H

#include <stdint.h>
#include <stdbool.h>

// One frozen reading of the 5 line sensors, calibrated 0..1000.
//   s[0] = leftmost sensor … s[4] = rightmost sensor.
//   0 = bright white paper, 1000 = deep black tape.
typedef struct {
    uint16_t s[5];
} sensor_snapshot_t;

// What kind of junction the robot is sitting on.
// Named from the robot's point of view, arriving along the line.
typedef enum {
    JCT_LEFT_ONLY,       // ← only a left branch (the line turns left)
    JCT_RIGHT_ONLY,      // → only a right branch
    JCT_STRAIGHT_LEFT,   // ↑← straight continues, plus a left branch
    JCT_STRAIGHT_RIGHT,  // ↑→ straight continues, plus a right branch
    JCT_T,               // ←→ left and right, no straight (a T from below)
    JCT_CROSS,           // ↑←→ all three ways open (4-way crossing)
    JCT_DEAD_END,        // the line just stops
    JCT_GOAL,            // the big black finish patch
    JCT_NONE             // "I can't tell" — the robot treats this as a fault
} junction_t;

// A move the robot can make at a junction. Stored as printable chars so a
// recorded path IS a human-readable string on the OLED: "LSRBL…".
typedef char move_t;
#define MOVE_LEFT     'L'   // turn 90° left, then follow
#define MOVE_RIGHT    'R'   // turn 90° right, then follow
#define MOVE_STRAIGHT 'S'   // no turn, follow through
#define MOVE_BACK     'B'   // turn 180° (dead end), go back
#define MOVE_NONE     '\0'  // returned by replay_next() past the end

#define PATH_MAX_MOVES 64

// The recorded route. NOT nul-terminated — `len` is the truth.
// `overflow` latches true if a caller ever tried to record move #65;
// a path that overflowed can't be trusted for replay.
typedef struct {
    char    moves[PATH_MAX_MOVES];
    uint8_t len;
    bool    overflow;
} path_t;

// ---------------------------------------------------------------------------
// The five functions below are THE BRAIN (logic/maze_logic.c).
// host_tests/ exercises all of them. Full contracts live as the comment
// block on each function in maze_logic.c — read the contract first,
// then the body.
// ---------------------------------------------------------------------------

// Decide what junction the robot is on, from two snapshots:
//   before    — per-sensor MAXIMUM latched while the sensor bar swept across
//               the junction (so a side branch shows as a dark outer sensor,
//               even if it was under the bar for only a few milliseconds).
//   at_center — a fresh reading taken after creeping forward CREEP_MM, i.e.
//               with the sensor bar PAST the junction line. This is the one
//               that knows whether a straight exit exists (center dark), or
//               nothing exists (all white), or you're on the goal patch.
junction_t classify_junction(const sensor_snapshot_t *before,
                             const sensor_snapshot_t *at_center);

// The left-hand rule: which move does a wall-hugging explorer make here?
move_t decide_left_hand(junction_t j);

// Append one move to the path — bounded, never overruns, latches overflow.
void path_record(path_t *p, move_t m);

// Collapse detours in place: every "x B y" triple is really one simpler
// move. Repeat until no B remains between two other moves.
void path_simplify(path_t *p);

// Replay support: the move to make at junction number `idx` (0-based),
// or MOVE_NONE if idx is past the end of the path.
move_t replay_next(const path_t *p, uint8_t idx);

// ---------------------------------------------------------------------------
// THE ARRIVAL VERDICT (logic/maze_logic.c) — the run loops' shared judgment.
//
// Every arrival, exploring or replaying, ends in the same three-way
// question: keep driving, declare the run won, or stop with a fault. The
// answer is a function of three facts and nothing else — is the path spent
// (the move byte is MOVE_NONE), what did the classifier say, and is the
// byte a move at all — so it belongs HERE, with the rest of the thinking,
// not inlined as an `if` chain in a run loop the tests cannot reach.
//
// It used to be inlined, twice: main.c screened arrivals one way and
// host_tests/sim_maze.c re-implemented the same screens in its own
// vocabulary. Two hand transcriptions of one decision table drift, and
// they had — in three cells (audit AUDIT-18). One function, two callers,
// one truth table (host_tests/test_maze_logic.c pins every cell).
//
// The verdict is a CODE, never a message: logic/ stays I/O-free, so the
// firmware maps codes to its 16-column fault screens and the simulator
// maps the same codes to full-sentence diagnostics. The fault STRINGS
// stay where the display is.
// ---------------------------------------------------------------------------

typedef enum {
    ARRIVE_PROCEED,              // ordinary junction — obey the move byte
    ARRIVE_SUCCESS,              // this arrival ENDS the run (the goal patch)
    ARRIVE_FAULT_CLASSIFY_NONE,  // the classifier refused: "I can't tell"
    ARRIVE_FAULT_NOT_GOAL,       // path spent, but this is some other junction
    ARRIVE_FAULT_GOAL_EARLY,     // the goal, with moves still owed
    ARRIVE_FAULT_DEAD_END,       // replay entered a dead end (simplify missed)
    ARRIVE_FAULT_ILLEGAL_MOVE    // the byte to execute is not one of L/S/R/B
} arrival_verdict_t;

// Judge one REPLAY arrival. `m` is what replay_next() answered for this
// arrival index — MOVE_NONE means the path is spent, which is exactly how
// the loop learns "this arrival must BE the goal".
// RETURNS: any of the seven codes above — replay is the strict lane, so
// it owns the whole enum, including the three no explorer can produce
// (NOT_GOAL, GOAL_EARLY, DEAD_END). Each of those three means the same
// thing in different words: the plan and the world disagree.
arrival_verdict_t replay_arrival_verdict(junction_t j, move_t m);

// Judge one EXPLORE arrival. `m` is what decide_left_hand() answered for
// this junction. Same three-way question, two deliberate differences: a
// dead end is an ordinary arrival here (turning around is how 'B' gets
// recorded), and there is no plan to contradict — the explorer stops on
// the goal patch, not on a spent path.
// RETURNS: ONLY ARRIVE_PROCEED, ARRIVE_SUCCESS, ARRIVE_FAULT_CLASSIFY_NONE
// or ARRIVE_FAULT_ILLEGAL_MOVE. That narrower set is a GUARANTEE, not an
// accident of today's body: with no spent-path column and no dead-end
// screen, the three replay-only codes have no exit from this lane. Both
// callers lean on it — the firmware maps this lane's faults to two OLED
// strings, and host_tests/sim_maze.c's explore walk prints the raw code
// for anything else rather than inventing a sentence for it. Widen this
// set and you owe every caller a new arm; that is why the promise lives
// in the contract and not only in the bodies that keep it.
arrival_verdict_t explore_arrival_verdict(junction_t j, move_t m);

// ---------------------------------------------------------------------------
// The FALLBACK LANE (logic/maze_logic_ref.c) — so the robot always runs.
// ---------------------------------------------------------------------------

// Reference classifier: deliberately minimal, fixed thresholds. The
// as-shipped firmware used it so the "maze walker" worked on day one;
// it stays compiled in forever as the fallback — comment out
// USE_MY_CLASSIFIER in tuning.h and this one drives explore instead
// (same decision layer, same seams) — and as a comparison baseline for
// debugging the real classifier by differencing.
junction_t classify_junction_ref(const sensor_snapshot_t *before,
                                 const sensor_snapshot_t *at_center);

// Short display name for a junction type ("CROSS", "DEAD END", …).
const char *junction_name(junction_t j);

#endif // MAZE_LOGIC_H
