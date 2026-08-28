// maze_logic.h — maze reasoning: interface and type contracts.
//
// Everything declared here is plain C over plain structs: no SDK headers, no
// register access, no timing, no I/O. That is a deliberate split. The same
// sources compile for the robot with arm-none-eabi-gcc and for a host with
// plain gcc, so tests/logic/ can exercise junction classification, the
// left-hand rule, path recording, simplification and replay — unit tests and
// a whole-maze simulator — with no hardware in the loop.
//
// The hardware side's job is to hand these functions two clean sensor
// snapshots per junction and then execute the moves they hand back.
#ifndef MAZE_LOGIC_H
#define MAZE_LOGIC_H

#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// THE MAZE THIS CODE IS DESIGNED AGAINST
//
// The algorithms below hold only on a maze built to this specification. It is
// their domain of validity: outside it the junction signatures stop meaning
// what the code assumes they mean.
//
// Robot geometry the numbers come from
//   Sensor bar ~40 mm wide, mounted ahead of the axle. Robot ~98 mm across.
//   After detecting a junction the robot creeps ~45 mm (CREEP_MM in
//   include/tuning.h) to put its axle over the junction center.
//
// Surface
//   Line       matte black tape, 19 mm (one tape width), on white
//              posterboard. MATTE, not glossy: the sensors measure reflected
//              IR, and glossy tape mirror-reflects at some angles, reading as
//              bright paper. 19 mm is about 2.5 sensors' worth of dark — wide
//              enough for a position estimate, narrow enough that the outer
//              sensors stay on white while following, which is what lets
//              "an outer sensor went dark" mean "side branch".
//   Plane      one flat plane: no steps where two boards meet, no curled tape
//              edges. The sensors ride 1-2 mm above the floor, so a lifted
//              edge is terrain.
//
// Layout
//   Segment    ~200 mm, 150-250 acceptable. After the creep and the turn the
//              follower needs runway to re-center before the next event.
//   Junction   >= 150 mm between junction centers. The detector is blind for
//   spacing    BLIND_MM of encoder-measured travel after each junction; two
//              junctions closer together than creep + blind distance merge
//              into a single event.
//   Angles     true 90°, and tape crossings run full width through each
//              other. The robot turns exactly 90° by gyro and then expects
//              line under its nose; a 75° "90" leaves the new line half a
//              sensor bar away.
//   Lead-in    >= 100 mm of straight line before the first junction, so
//              calibration and the first blind window both finish with the
//              robot solidly line-locked.
//   Goal patch solid black square, >= 50x50 mm (75x75 recommended), at a
//              dead-end tip with a >= 150 mm straight approach. The goal
//              signature is "still on broad black after creeping 45 mm in",
//              so the patch has to outlast the creep: 50 mm leaves 5 mm of
//              margin, 75 mm leaves 30. The straight approach is what puts
//              the sensor bar onto the patch square-on.
//   Topology   the maze must be a TREE — no loops anywhere. This is a
//              precondition of the algorithm, not a preference: the
//              left-hand rule is only guaranteed to reach the goal in a
//              loop-free maze. One loop and the same rule can orbit it
//              forever. Every added corridor must dead-end rather than
//              reconnect two existing corridors.
// ---------------------------------------------------------------------------

// One frozen reading of the 5 line sensors, calibrated 0..1000.
//   s[0] = leftmost sensor … s[4] = rightmost sensor.
//   0 = bright white paper, 1000 = deep black tape.
// Calibrated means a calibration pass has already mapped this robot's raw
// sensor decay times onto that scale; the logic here never sees raw counts.
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
    JCT_NONE             // "cannot tell" — the robot treats this as a fault
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
// a path that overflowed cannot be trusted for replay.
typedef struct {
    char    moves[PATH_MAX_MOVES];
    uint8_t len;
    bool    overflow;
} path_t;

// ---------------------------------------------------------------------------
// Navigation (logic/maze_logic.c)
// ---------------------------------------------------------------------------

// Classify the junction the robot is standing on.
//
// `before`    per-sensor MAXIMUM, latched while the sensor bar swept across
//             the junction. A side branch can only appear here: its tape is
//             under an outer sensor for a few milliseconds and the latch
//             keeps that darkness. The outermost sensors s[0] and s[4] carry
//             the evidence; the entry line itself never reaches them.
// `at_center` a fresh reading taken after creeping CREEP_MM forward, with the
//             bar PAST the junction line. Only this snapshot can say whether
//             a straight exit continues (center dark), whether nothing
//             continues (all white), or whether the robot is standing on the
//             goal patch (still broad black).
// Both are calibrated 0..1000 with s[0] leftmost, per sensor_snapshot_t.
// Neither pointer may be NULL; neither is modified.
//
// RETURNS one junction_t. JCT_NONE is a real answer rather than an error
// path: the evidence fits no signature, or contradicts one, and the run loops
// stop with a fault instead of guessing. The function will not certify GOAL,
// CROSS, T or DEAD_END on contradicted evidence — that guarantee is what the
// refusal band in maze_logic.c buys. Pure: same inputs, same answer, no state
// carried between calls.
junction_t classify_junction(const sensor_snapshot_t *before,
                             const sensor_snapshot_t *at_center);

// The left-hand rule: the move a wall-hugging explorer makes at a junction of
// type `j`. Takes the leftmost available exit, ranking L over S over R, and
// answers MOVE_BACK at a dead end (its only legal move).
//
// PRECONDITION: the maze is a tree — see the specification above. That, not
// this function, is what guarantees the walk reaches the goal.
// RETURNS MOVE_LEFT / MOVE_RIGHT / MOVE_STRAIGHT for the six junction types
// with exits, MOVE_BACK for JCT_DEAD_END, and MOVE_NONE for JCT_GOAL,
// JCT_NONE and any out-of-range value. MOVE_NONE is the one byte the motion
// layer refuses to execute, so a wiring bug that lets a goal or a refusal
// reach this function stops the robot rather than moving it blind. Pure.
move_t decide_left_hand(junction_t j);

// Append move `m` to path `p`. The buffer is fixed at PATH_MAX_MOVES bytes
// and is not nul-terminated: p->len is the single source of truth for how
// much of it is real. `p` must point at an initialized path_t (all-zero is a
// valid empty path). No validation of `m` happens here — any byte can be
// recorded; legality is screened at execution time.
//
// FAILURE MODE: recording into a full path (p->len == PATH_MAX_MOVES) drops
// the move, sets p->overflow, and changes nothing else. The flag LATCHES —
// nothing in this module ever clears it — because a route that lost a move is
// no longer a route, and the explore loop is expected to fault on the flag
// instead of reporting a solve. This function is the only writer of
// p->moves/p->len, so it is the only place a buffer overrun could originate.
void path_record(path_t *p, move_t m);

// Collapse dead-end detours in place, leaving the shortest route with the
// same start and end. Every 'B' in a recorded path marks a dead-end detour,
// so each (x, 'B', y) triple is rewritten as the single move with the same
// net heading change, repeatedly, until no 'B' has a move on both sides of
// it. See maze_logic.c for the substitution table and why it is correct.
//
// p->len shrinks by 2 per collapse and never grows, so this cannot overflow.
// A path with no interior 'B' comes out byte-for-byte unchanged. A 'B' at
// position 0 or len-1 has no triple around it and is left in place. Empty,
// one-move and two-move paths are untouched. p->overflow is neither read nor
// cleared: simplifying a truncated path yields a shorter truncated path, and
// it is still not a route.
void path_simplify(path_t *p);

// The move to make at junction number `idx` (0-based) of a replay run.
//
// RETURNS p->moves[idx] while idx < p->len, and MOVE_NONE for idx >= p->len,
// including far past the end — there is no out-of-range index, every value is
// defined. A path of len moves steers exactly len junctions, so idx == len is
// the first arrival with nothing left to say, and MOVE_NONE there is how the
// replay loop learns the path is spent and this arrival must BE the goal.
// The comparison is strict `<` for that reason. Does not modify `p`.
move_t replay_next(const path_t *p, uint8_t idx);

// ---------------------------------------------------------------------------
// Arrival verdicts (logic/maze_logic.c)
//
// Every arrival, exploring or replaying, ends in the same three-way question:
// keep driving, declare the run won, or stop with a fault. The answer depends
// on three facts and nothing else — whether the path is spent (the move byte
// is MOVE_NONE), what the classifier said, and whether the byte is a move at
// all — so it lives here with the rest of the reasoning rather than as an
// `if` chain inside a run loop the tests cannot reach.
//
// The verdict is a CODE, never a message. This layer stays free of strings
// and I/O: the firmware maps codes to its 16-column fault screens and the
// simulator maps the same codes to full-sentence diagnostics.
// ---------------------------------------------------------------------------

typedef enum {
    ARRIVE_PROCEED,              // ordinary junction — obey the move byte
    ARRIVE_SUCCESS,              // this arrival ENDS the run (the goal patch)
    ARRIVE_FAULT_CLASSIFY_NONE,  // the classifier refused: "cannot tell"
    ARRIVE_FAULT_NOT_GOAL,       // path spent, but this is some other junction
    ARRIVE_FAULT_GOAL_EARLY,     // the goal, with moves still owed
    ARRIVE_FAULT_DEAD_END,       // replay entered a dead end (simplify missed)
    ARRIVE_FAULT_ILLEGAL_MOVE    // the byte to execute is not one of L/S/R/B
} arrival_verdict_t;

// Judge one REPLAY arrival. `j` is the classifier's verdict for this arrival
// and `m` is what replay_next() answered for this arrival index — MOVE_NONE
// means the path is spent, which is exactly how the loop learns "this arrival
// must BE the goal".
// RETURNS any of the seven codes above. Replay is the strict lane, so it owns
// the whole enum, including the three no explorer can produce (NOT_GOAL,
// GOAL_EARLY, DEAD_END); each of those says the same thing in different
// words, that the plan and the world disagree. Pure.
arrival_verdict_t replay_arrival_verdict(junction_t j, move_t m);

// Judge one EXPLORE arrival. `m` is what decide_left_hand() answered for
// junction `j`. Same three-way question, with two deliberate differences: a
// dead end is an ordinary arrival here (turning around is how 'B' gets
// recorded), and there is no plan to contradict — the explorer stops on the
// goal patch, not on a spent path.
// RETURNS ONLY ARRIVE_PROCEED, ARRIVE_SUCCESS, ARRIVE_FAULT_CLASSIFY_NONE or
// ARRIVE_FAULT_ILLEGAL_MOVE. That narrower set is a guarantee callers may
// rely on, not an accident of the current body: with no spent-path column and
// no dead-end screen, the three replay-only codes have no exit from this
// lane. The firmware maps this lane's faults to two OLED strings, and the
// simulator's explore walk prints the raw code for anything else rather than
// inventing a sentence for it. Widening the set means giving every caller a
// new arm, which is why the promise is stated here and not only kept by the
// body. Pure.
arrival_verdict_t explore_arrival_verdict(junction_t j, move_t m);

// ---------------------------------------------------------------------------
// Reference lane (logic/maze_logic_ref.c)
// ---------------------------------------------------------------------------

// A second, deliberately simpler junction classifier: one fixed threshold, no
// tuning knobs, no refusal margin. Same signature and same contract on its
// inputs as classify_junction(), and it returns the same junction_t vocabulary
// — but it is coarser and will read marginal evidence differently. It serves
// two purposes: a fallback (comment out USE_MY_CLASSIFIER in include/tuning.h
// and explore runs on this one instead, same decision layer underneath), and a
// cross-check baseline for differencing against classify_junction() when the
// real classifier misbehaves.
junction_t classify_junction_ref(const sensor_snapshot_t *before,
                                 const sensor_snapshot_t *at_center);

// Short display name for a junction type ("CROSS", "DEAD END", …). Returns a
// pointer to a string literal, valid forever; "???" for an unknown value.
const char *junction_name(junction_t j);

#endif // MAZE_LOGIC_H
