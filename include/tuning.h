// tuning.h — every tunable constant in the firmware, in one place.
//
// Each number that would make one 3pi+ 2040 behave differently from the
// next one is here. The .c files are machinery; this is the cockpit, and a
// robot that is too slow, too twitchy or too timid is almost always fixed
// by a number in this file rather than by code.
//
// Each knob states what it controls, its units, the range that is safe (or
// what going outside it costs), and — where one exists — the signature a
// telemetry capture shows when the knob is set wrong. Those signatures
// name rows and events from the CSV format specified in src/telemetry.h:
// the 'F'/'B'/'C'/'T'/'K' tick rows, the 'J'/'A' snapshots, the event rows,
// and the k=v knob line each dump carries.
//
// Record every change in the tuning log at the end of this file.
#ifndef TUNING_H
#define TUNING_H

// ------------------------------------------------------------ robot identity
// Shown on the boot splash.
#define ROBOT_NAME "robo"

// -------------------------------------------------------------- drive speeds
// All speeds are PWM levels out of 6000 (= 100% duty at 20.8 kHz).
// SPEED_HARD_CAP is enforced inside hw_motors.c: no code path in this
// project can drive faster than it, on purpose.
#define SPEED_HARD_CAP   5000   // absolute ceiling — not a lap-time knob

#define SPEED_EXPLORE    1800   // mapping pace: slow enough that no junction slips past
#define SPEED_REPLAY     2500   // pace for replaying a path already solved
#define SPEED_SPEEDRUN   4000   // the fast-lap target SPEED_REPLAY is stepped toward
#define SPEED_CREEP      1200   // rolling the sensors across a junction
#define SPEED_TURN_MAX   2500   // cap for in-place gyro turns
#define SPEED_CALIBRATE  1000   // in-place spin while sweeping sensors over the line

// Raising SPEED_REPLAY is a staged exercise, not a single edit: step it
// 2500 -> 3000 -> 3500 toward SPEED_SPEEDRUN, one change per flash, two
// clean laps per step. The check at each step is a COUNT, not an
// impression: JCT_DETECT events in the dump must equal the junctions in
// the maze. A junction missed at speed does not crash — the branch line
// flashes under an outer sensor and the robot sails on, politely wrong —
// so the flight recorder, not the eye, is the judge.

// --------------------------------------------------- replay arrival braking
// The speed wall that appears once the replay base is stepped up. This is
// DERIVED from the numbers in this file, not observed: raise the replay
// base toward 4000 and junctions that classify correctly at 2500 should
// start classifying WRONG. It is not a threshold problem (the sensors are
// fine) and not a detection problem (even at 4000 ~ 1 m/s the 2 ms tick
// reads every ~2 mm, and a branch line is ~19 mm of tape — nine looks,
// plus drive.c's edge latch). It is momentum: creep_to_center measures its
// CREEP_MM from wherever it starts, but a chassis needs real DISTANCE to
// shed speed. Enter the creep at 4000-pace and the robot is still braking
// when the budget runs out — it stops PAST the junction, the at_center
// read lands on the wrong patch of floor, and the classifier honestly
// reports what the mis-aimed photo shows. Dump signature: 'C' tick `a`
// advancing in big steps, then CREEP_END followed by CLASSIFY errors that
// are absent at 2500.
//
// The fix is a slowdown window, not threshold surgery: the moment
// drive_until_junction returns — motors still running at base, which is
// exactly what drive.h promises — replay brakes to SPEED_ARRIVAL, and only
// then creeps. Straights stay hot; every arrival is COMMANDED down to the
// pace the creep is designed around.

// Creep-entry speed, PWM out of 6000. 2500 is the flat replay pace every
// piece of arithmetic in this file is written around; no hardware has
// signed it off (see the note at the end of the tuning log). The window
// only exists when the run's base > SPEED_ARRIVAL, so at SPEED_REPLAY 2500
// these knobs change nothing at all — a stepped base (3000, 3500) is what
// arms them. Dump signature for a hot entry: the 'C' rows between the 'J'
// snapshot and CREEP_START still show large per-tick progress in `a`. The
// fix is to lower this toward SPEED_EXPLORE (1800), the pace explore takes
// every junction at. Lowering it is free under the distance bound below:
// the brake drags in proportion to the excess over the COMMAND (physics at
// ARRIVAL_BRAKE_MS), so a lower command sheds MORE speed in the SAME
// ARRIVAL_BRAKE_MM of floor — this knob buys braking authority without
// moving the stop.
#define SPEED_ARRIVAL    2500

// How FAR the window may roll, in encoder-measured millimetres. Safe
// range 0..25, enforced immediately below. The bound is DISTANCE because
// the window's real cost is POSITION: creep_to_center measures its
// CREEP_MM from wherever the window ends, so every window millimetre
// pushes the deciding at_center read that much further past the detect
// edge. Do that arithmetic before trusting any number here. At
// SPEED_ARRIVAL 2500 (~0.62 m/s by the 4000 ~ 1 m/s rule above) the
// chassis covers ~6 mm every 10 ms, so a wall-time window is a distance
// nobody chose: the retired 80 ms default travelled ~50 mm — more than the
// creep's entire 45 mm budget — landing the at_center read ~95 mm past the
// detect edge, out the FAR side of the recommended 75 mm goal patch, whose
// margin past the creep is only 75 - 45 = 30 mm. Every stepped-base goal
// arrival would misread and fault "replay:not goal". Budget at THIS
// default: 20 mm window + 45 mm creep = 65 mm, stopping 10 mm inside the
// patch. Tune with the geometry, not the stopwatch: overshoot shows in a
// dump as GOAL misreads on the patch and 'A' snapshot rows reading white
// just past a plain junction, and the answer to that is lowering THIS
// knob; 'C' rows that still enter hot say lower SPEED_ARRIVAL instead.
#define ARRIVAL_BRAKE_MM  20

// The 25 mm ceiling above, enforced: the far edge of the goal patch is
// arithmetic, not advice.
#if ARRIVAL_BRAKE_MM > 25
#error "ARRIVAL_BRAKE_MM > 25 rolls the deciding at_center read out the far side of the recommended 75 mm goal patch (window + 45 mm creep vs the patch's 45 + 30): every stepped-base goal arrival would misread and fault 'replay:not goal'. Shed speed by lowering SPEED_ARRIVAL instead."
#endif

// The window's TIMEOUT, not its length — ARRIVAL_BRAKE_MM ends the window;
// this clock only closes it if the encoders stop counting (wheel jammed,
// chassis blocked), so the loop can never hang, and on timeout it still
// hands off DRIVE_OK: diagnosing the stall belongs to creep_to_center's
// watchdog right behind it. The braking physics is unchanged by the unit:
// at every off-fraction of the 20.8 kHz PWM cycle these PHASE/ENABLE
// drivers tie the motor terminals together (the "both bottoms closed"
// H-bridge state = braking, not coasting), so a command below the
// chassis's pace drags it toward SPEED_ARRIVAL in proportion to the
// excess, and can never drag it below. Time was simply the wrong UNIT to
// bound the window with, because its cost is measured in millimetres.
// 80 ms is >2x the ~32 ms the 20 mm bound takes at arrival pace, so on a
// healthy run this clock never fires — a dump in which it does fires
// TIMEOUT with the 'C' rows' `a` flat.
#define ARRIVAL_BRAKE_MS  80

// Spacing the window costs: at a stepped base an arrival consumes window
// 20 + creep 45 + post-turn blind 30 = 95 mm of the 150 mm minimum
// junction spacing the maze is built to (55 mm to spare). Pack two
// junctions closer than that and they merge into ONE arrival, whose dump
// signature is a JCT_DETECT count one short of the junction count at
// stepped bases only, with the merged sweep row fattened by both
// junctions' tape.

// Every SPEED_* knob rides under SPEED_HARD_CAP — enforced here at compile
// time because hw_motors.c enforces it at RUN time by CLAMPING: a knob
// above the cap does not crash, it LIES. The wheels get SPEED_HARD_CAP
// while every derived calculation — base/6000 curvature scaling, the
// arrival window's excess-speed story, the stepped-base arithmetic —
// reasons from a speed the motors never deliver. (This #if sits down here,
// after SPEED_ARRIVAL, because the preprocessor treats a not-yet-defined
// name as 0 and would wave a misordered check through.)
#if SPEED_EXPLORE > SPEED_HARD_CAP || SPEED_REPLAY > SPEED_HARD_CAP || \
    SPEED_SPEEDRUN > SPEED_HARD_CAP || SPEED_CREEP > SPEED_HARD_CAP || \
    SPEED_TURN_MAX > SPEED_HARD_CAP || SPEED_CALIBRATE > SPEED_HARD_CAP || \
    SPEED_ARRIVAL > SPEED_HARD_CAP
#error "A SPEED_* knob exceeds SPEED_HARD_CAP: hw_motors.c clamps every command to the cap, so the motors would silently run at SPEED_HARD_CAP while the tuning arithmetic (base/6000 scaling, the arrival window) reasons from a speed they never deliver. Lower the knob - the cap does not move."
#endif

// ---------------------------------------------------------- line-follow PID
// Same shape as Pololu's own line follower (line_follower.py):
//     pid = (p * LINE_KP + d * LINE_KD) * base / 6000
// where p = (line position 0..4000) - 2000 and d = p - last_p.
// pid is added to one wheel and subtracted from the other, then clamped
// to [0, base].
//
// Two normalizations make Pololu's numbers mean here what they meant there
// (they tuned at base = 6000 in a ~3 ms MicroPython loop):
//   * base/6000 — steering is differential drive, so the SAME curvature
//     needs a differential proportional to speed. Without this, running at
//     explore speed (1800) triples the effective P gain and the follower
//     turns into a twitchy bang-bang controller.
//   * d = p - last_p is per-TICK, so KD's strength depends on the tick
//     rate. This C loop runs a fixed CONTROL_PERIOD_US (2 ms) against
//     their ~3 ms, so KD is seeded at 3000 (= 2000 * 3/2) for the same
//     damping.
//
// Dump signature of an under-damped follower: on a straight, `a` on the
// 'F' rows swings sign to sign with growing amplitude while the duty
// columns alternate between 0 and base. Raise LINE_KD ~500 at a time; if
// it still fishtails at twice the seeded KD, lower LINE_KP by 10-20
// instead — that is too much P to damp with any reasonable D.
//
// base/6000 is also what lets the replay base be stepped 2500 -> 3500
// without touching KP: same p, same commanded curvature, at any base.
// What the scaling cannot hide is ACTUATOR LAG. Motors take real time to
// change a wheel's speed, so at 3500 the same correction arrives after
// more millimetres of travel than it did at 1800, and a straight that is
// calm at explore pace can weave at replay pace. That is LINE_KD's fight,
// and the discriminator is a pair of captures over the same straight at
// 1800 and 3500: if the fast one oscillates while the slow one is clean,
// that is actuator-lag physics (KD +500, then re-check the slow run for
// chatter), not a broken follower.
#define LINE_KP  90
#define LINE_KD  3000

// The follower runs at exactly this period, microseconds (the sensor read
// itself takes up to ~1.1 ms, so do not go below ~1500). A FIXED period is
// what makes the D term trustworthy: on a free-running loop the read time
// varies ~7x with how much tape is under the bar, and the damping varies
// with it.
#define CONTROL_PERIOD_US  2000

// ------------------------------------------------------- junction detection
// Calibrated readings are 0 (bright white) .. 1000 (deep black).
//
// JCT_OUTER_THRESH: an outer sensor at or above this is a side branch.
// Too low and a tape seam or a speck of dust latches a junction that is
// not there — dump signature is a JCT_DETECT mid-segment where the maze
// has none, with only one outer sensor briefly over the line in the rows
// just before it; raise toward 700 so a flicker that brief cannot latch.
// Too high and real branches go uncounted (JCT_DETECT count below the
// maze's junction count).
// LINE_LOST_THRESH: all sensors below this means the line is gone.
#define JCT_OUTER_THRESH  600
#define LINE_LOST_THRESH  300

// The classifier's thresholds (logic/maze_logic.c). tuning.h is nothing
// but #defines, which is why the pure logic/ layer may include it: the
// brain gets this robot's numbers without ever touching an SDK header.
//
// One reading counts as "dark" at or above this line. It is a knob, not a
// constant of nature: it encodes THIS tape and THIS calibration (the build
// keeps paper well under ~200 and tape well over ~700, so one number can
// separate them), never anything about the algorithm.
#define JCT_DARK_THRESH   600

// GOAL vs CROSS: how many at_center sensors must read dark to call the
// goal patch. Both blacken the whole bar on approach; after the creep a
// crossing leaves only its straight exit under the bar (1-2 dark sensors),
// while the goal patch still darkens all 5. Requiring 4 of 5 instead of
// all 5 keeps one sensor riding the patch edge through a dusty spot from
// turning GOAL into CROSS — the gap between "1-2" and "4" is the margin.
// Its other edge is DERIVED, not tuned: maze_logic.c's CROSS_MAX_DARK
// (= GOAL_MIN_DARK - 2) keeps the gap exactly one count wide wherever this
// knob moves, and a count in the gap refuses as JCT_NONE — the full
// argument lives at that #define. tests/logic/sim_maze.c's marginal-world
// suite exercises the analysis with near-threshold readings.
// Safe range 3..5, enforced below. Dump signature of a wrong setting: a
// CLASSIFY row reading GOAL at a plain crossing, or NONE at either.
#define GOAL_MIN_DARK  4

// The margin story above only holds while this knob stays where the bar's
// own geometry can honor it — enforced at compile time.
#if GOAL_MIN_DARK < 3 || GOAL_MIN_DARK > 5
#error "GOAL_MIN_DARK must stay in 3..5: below 3 the derived CROSS_MAX_DARK (GOAL_MIN_DARK - 2) drops under the 1-2 dark sensors an ordinary crossing's exit legitimately leaves, so crossings misread as GOAL or refuse; above 5 it asks for more dark sensors than the bar has, and the goal patch can never classify."
#endif

// ------------------------------------------- chassis geometry and distances
// Wheels: 3.56 encoder counts per mm (~358.3 counts/rev on a 32 mm wheel).
// Stored x100 so the code stays in integers. This constant converts every
// distance knob below into counts, and the dump carries it as
// counts_per_mm_x100 so a capture can be checked against them.
#define ENC_COUNTS_PER_MM_X100  356

// If a wheel counts BACKWARD when the robot drives forward (check on the
// DIAG screen — spin each wheel forward by hand), flip its sign here.
#define ENC_SIGN_LEFT   (+1)
#define ENC_SIGN_RIGHT  (+1)

// How far to roll forward after detecting a junction, in millimetres, so
// the wheel axle (the robot's turning center) sits on the junction center.
// The sensor bar rides ahead of the axle; this closes that gap, and it is
// specific to this chassis — after a 90 degree turn the robot should end
// up centered on the new line. Dump check: the 'A' snapshot row right
// after CREEP_END is the settled read this distance aims. Its center
// sensors reading white means the creep stopped short of, or rolled past,
// the junction center; the 'C' rows' `a` right before it says which,
// against CREEP_MM x counts_per_mm_x100/100.
#define CREEP_MM  45

// How long the creep waits after its stop before taking the at_center
// read, milliseconds. A stop is not instant stillness: the chassis pitches
// on its tires, and the sensor bar — cantilevered ahead of the axle —
// sweeps millimetres of floor while the body rocks. The at_center read is
// THE deciding classifier input; taken mid-rock it can straddle a tape
// edge and hand the brain a smeared photo, so the creep stops, outwaits
// the rock, and only then reads. Same physics as BT_SETTLE_MS; the creep's
// stop is gentler (it arrives at SPEED_CREEP, not out of a mid-correction
// loss), so its default sits lower. Too low and 'A' rows disagree with the
// 'J' row above them for no reason the sensor trace explains; too high and
// every junction costs the extra dwell.
#define CREEP_SETTLE_MS  60

// Post-turn blind window, in DISTANCE (encoder-measured millimetres), not
// time: right after a turn the outer sensors may still see pieces of the
// junction just handled, so junction detection stays off — and the
// follower steers from the CENTER THREE sensors only — until the bar has
// rolled this far onto clean line. Distance, because the same window must
// work at explore AND replay speed. Dump signature of too short a window:
// `a` pegged at +/-2000 on the 'B' rows or the first 'F' rows after a
// turn, with all three center sensors white, while the preceding TURN_END
// residual (`a`, deg x10) was small — a large residual means the turn, not
// this knob, is the root cause. Audit the distance at BLIND_END's `a`
// (counts travelled blind) against BLIND_MM x counts_per_mm_x100/100.
#define BLIND_MM  30

// --------------------------------------------------------- line-loss recovery
// The whole bar going white for LOST_CONFIRM_MS means the line is gone.
// Which KIND of gone is decided by how hard the follower was correcting
// just before (max |p| over its recent history):
//   * recent max |p| <= DEADEND_P_MAX — the robot was driving straight and
//     the line simply stopped: an honest dead end, reported to the
//     classifier.
//   * recent max |p| >  DEADEND_P_MAX — the robot was mid-correction and
//     drove OFF the tape: back-track (retrace the recorded encoder history
//     in reverse, slowly) until a center sensor sees tape again, then
//     resume.
// DEADEND_P_MAX is the coin-flip line, so keep it clear of the typical
// straight-line |p| this robot runs at (read that off the 'F' rows on a
// clean straight). Dump signature of a badly placed threshold: DEADEND
// events whose `a` sits close to DEADEND_P_MAX — that verdict was a coin
// flip, not a clear read.
#define LOST_CONFIRM_MS     20
#define DEADEND_P_MAX       600
#define CENTER_LOST_MAX_MS  250   // center gone this long while slamming = lost too

// The follower calls the CENTER of the bar "lost" when all three middle
// sensors read below CENTER_LOST_THRESH — from that tick on it steers from
// memory, not measurement. The back-track calls the line "found" when a
// middle sensor reads at or above LINE_FOUND_THRESH.
//
// ORDERING RULE:  CENTER_LOST_THRESH <= LINE_FOUND_THRESH.  Always.
// These two thresholds are a handoff: the back-track finds the line, the
// follower takes it from there. If "found" could fire on a reading the
// follower still calls "lost" (found threshold below lost threshold),
// there is a dead zone — a graze in between ends the retrace, the follower
// resumes already lost, and a real line turns into a false dead end. Equal
// (or found higher) means every refind is one the follower will accept.
// Corollary: tape that calibrates below LINE_FOUND_THRESH can never be
// refound after a loss, which is why the build requires tape reading >700
// calibrated.
#define CENTER_LOST_THRESH  700
#define LINE_FOUND_THRESH   700   // a center sensor at/above this ends the back-track

// The ORDERING RULE, enforced at compile time: a knob turn must not be
// able to silently reopen the dead zone. The build refuses instead.
#if CENTER_LOST_THRESH > LINE_FOUND_THRESH
#error "CENTER_LOST_THRESH must be <= LINE_FOUND_THRESH: 'found' below 'lost' reopens the dead zone - a 600-699 graze ends the retrace with a line the follower still disowns, and the resume cascades into a false DEADEND on open floor (the ORDERING RULE above tells the whole story)."
#endif

#define BACKTRACK_SPEED     1200  // duty cap while retracing (slow on purpose)
#define BACKTRACK_KP        60    // duty per encoder-count of retrace error
#define BACKTRACK_TICK_X    2     // consume history at 1/2 recording speed
#define BACKTRACK_TIMEOUT_MS 6000 // retrace gives up here: BT_FAIL with a ~= this

// Ending the retrace on a SINGLE dark sample would let a glare flash or a
// tape seam "find" a line that is not there. One sample is an opinion;
// this many consecutive samples are a measurement. At the back-track tick
// rate (CONTROL_PERIOD_US x BACKTRACK_TICK_X = 4 ms) three ticks = 12 ms
// of continuous tape — a one-sample artifact cannot fake that, and the
// extra distance retraced while confirming is well under a millimetre.
// Too high and genuine refinds are retraced straight past.
#define BT_FOUND_CONFIRM_TICKS  3

// How long the chassis needs after hw_motors_stop() before a sensor read
// (or the follower's first p) can be trusted, milliseconds. A stop is not
// instant stillness: the body pitches on its tires and rocks for a few
// hundredths of a second, and the sensor bar — cantilevered out ahead of
// the axle — sweeps millimetres across the floor while it does. A read
// taken mid-rock can straddle the tape edge. 80 ms outlasts the rock at
// recovery speeds; heavier cells or worn tires rock longer and want more.
#define BT_SETTLE_MS  80

// The settle before the retrace MEASURES anything. backtrack_recover's
// first act after stopping is to read the encoders — those two numbers
// become the chase targets the whole retrace steers by, so they must come
// from a chassis that has genuinely stopped: every millimetre coasted
// AFTER that read is a millimetre the retrace will chase back as phantom
// error it never recorded. This is also the hottest stop in the recovery —
// the robot arrives out of a full-pace swerve at base speed,
// mid-correction, not out of a BACKTRACK_SPEED crawl — so it outwaits more
// momentum than BT_SETTLE_MS does. Raise it for heavier cells or a hotter
// base; the cost is dead time at every loss. Dump signature of too short a
// settle: 'K' rows whose retrace errors start large and never converge,
// ending in BT_FAIL.
#define BT_PREROLL_SETTLE_MS  120

// The retrace refinds the line while REVERSING, and momentum keeps
// carrying the robot backward through the stop — so when the settle check
// afterwards sees bare floor, the tape it just found lies AHEAD of the
// sensor bar, never behind it. The fix-up is a slow FORWARD creep toward
// the find point, bounded in distance AND time (every motor loop keeps a
// timeout and the any-button stop). Its ticks are 'C' rows between
// BT_FOUND and RESUME.
#define RESUME_NUDGE_MM          20   // beyond any plausible stop-coast
#define RESUME_NUDGE_TIMEOUT_MS  800  // stalled-wheel escape hatch

// The recovery budget: how many back-track parachute pulls one follow
// segment gets before the drive gives up and faults (DRIVE_LOST). The
// budget is about REPEATED failure — a follower that keeps falling off the
// same stretch of tape is broken, and recovery must not turn that into an
// infinite lawnmower pattern. Raising this hides a follower problem rather
// than fixing one: repeated LOSS -> BT_FOUND cycles on the same segment
// mean the recovery is working and the follower is too hot, so fix
// LINE_KP/LINE_KD first.
#define RECOVERIES_MAX  3

// ...but a SUCCESSFUL recovery followed by a long stretch of clean
// following is not repeated failure — it is the parachute working. Each
// RECOVERY_DECAY_MM of travel earns one budget credit back (never past a
// full budget), so four one-off recoveries spread across a long segment do
// not add up to a fault. The decay is distance-earned, which is what
// bounds it: a genuine lawnmower loop loses the line again within a few
// centimetres, earns nothing back, and still hits the budget. Be
// clear-eyed about timescales, though: this arithmetic is the only guard
// that acts QUICKLY on a cycling recovery. The run watchdog counts
// follow-time only (recovery time is credited back — see the resume in
// drive.c) and a loss cycle accrues just ~80+ ms of it, so DRIVE_TIMEOUT
// would end the churn eventually — after minutes, not seconds. The
// any-button stop always stands. Rely on the budget; treat the watchdog as
// the slow fence behind it.
#define RECOVERY_DECAY_MM  200

// Grace distance after a recovery resume, millimetres, before "all white"
// may again be judged a hard loss. The resume re-arms with an empty |p|
// history, so without this gate a single white read at the resume point
// would fault as a dead end with a recent-max-|p| of zero — dump signature
// is a DEADEND with `a` = 0 immediately after a RESUME. Distance, not
// time, for the same reason as BLIND_MM: it must mean the same thing at
// every speed. Must be measured from a FRESH encoder baseline taken at the
// resume itself (drive.c explains the trap).
#define RESUME_GRACE_MM          15

// Straightness servo gain while creeping: duty correction per encoder
// count of left-minus-right disagreement. Shared by the junction creep,
// the recovery nudge, and the arrival window — one gain, one meaning. It
// is the `b` column on 'C' rows, and it is unbounded, so too high a gain
// prints 'C' duty columns past the +/-5000 hard cap while the wheels are
// silently clamped.
#define CREEP_BAL_GAIN  8

// Pause at each junction after the decision is shown, milliseconds. Purely
// cosmetic — long enough for a human (and a camera) to see the robot
// "think" — and it is pure lap time, so shrink it when chasing a fast lap.
#define THINK_PAUSE_MS  400

// ---------------------------------------------------------------- gyro turns
// PD controller on gyro angle, straight from Pololu's gyro_turn.py
// (kp=140, kd=4, +/-3 degrees, settled for 250 ms).
//
// Dump signature of an undersized TURN_KP: TURN_END with a large residual
// (|a| > 50, i.e. > 5 degrees) or `b` near TURN_TIMEOUT_MS, with the 'T'
// rows' angle error stalling short of zero; a full timeout shows as
// TURN_START with no TURN_END, then TIMEOUT. Before raising TURN_KP, run
// the sag-versus-gain check documented at BATT_WARN_MV — weak turns from
// tired cells paint the same picture, and raising the gain to fight sag
// makes the next fresh-cell run overshoot and oscillate.
#define TURN_KP          140
#define TURN_KD          4
#define TURN_TOL_DEG     3
#define TURN_SETTLE_MS   250
#define TURN_TIMEOUT_MS  2500

// ----------------------------------------------------------------- watchdogs
// If the robot line-follows RUN_WATCHDOG_MS without finding a junction, it
// has almost certainly escaped the maze: stop and say so rather than
// touring the room. Dump signature: ~10 s of 'F' rows with no JCT_DETECT,
// ending TIMEOUT [phase=F] then FAULT.
// CREEP_TIMEOUT_MS bounds one creep. Dump signature when it fires:
// CREEP_START, 'C' rows whose `a` (progress, counts) stops climbing, then
// TIMEOUT [phase=C] — a jammed wheel, a blocked chassis, or an encoder
// sign flipped in ENC_SIGN_LEFT/RIGHT, not a knob that wants raising.
#define RUN_WATCHDOG_MS  10000
#define CREEP_TIMEOUT_MS 1500

// ------------------------------------------------------------------- battery
// 4x NiMH AAA: 4.8 V nominal. Below ~1.1 V/cell the motors get weak and
// any PID tuning done above that turns into fiction; below ~1.0 V/cell the
// readings are junk.
//
// Sag versus gain — same symptom, two causes, and one of them is free to
// fix. Before touching TURN_KP, read batt_mv= from the dump-facts header
// line. That number is RESTING voltage (the motors were stopped for the
// walk back to the bench), so the loaded voltage mid-turn was lower still:
// a batt_mv already down near BATT_WARN_MV means the turns ran on sagging
// cells and the fix is fresh batteries. The converse does NOT hold. A
// healthy resting number (4600+) does not clear the batteries, because
// resting voltage says nothing about internal resistance, and a worn pack
// is exactly the one that rests high and then collapses the moment both
// motors pull turn current — weak turns from the FIRST turn onward, which
// is the same picture an undersized TURN_KP paints. A high resting number
// only takes the easy verdict off the table. The real discriminator is the
// trend: sag gets worse lap over lap on one build (compare TURN_END
// residuals early and late in the same dump), while an undersized gain is
// exactly as wrong on a full charge as on an empty one.
#define BATT_WARN_MV    4400   // nag on screen
#define BATT_REFUSE_MV  4000   // refuse to start a run; a refusal logs a bare FAULT

// ----------------------------------------------------------- calibration gate
// After calibration, every sensor must have seen at least this much span
// between its white and black readings, or the calibration is rejected. A
// sensor with span < 300 was probably never over the tape (or the tape is
// glossy). The dump carries the window as cal_min=/cal_max= and this knob
// as cal_min_span, and plot_telemetry.py --summary prints the per-sensor
// spans with any that fall below it flagged.
#define CAL_MIN_SPAN  300

// ---------------------------------------------------------- classifier lane
// classify_junction() passes the full host suite and both simulator lanes,
// so the firmware runs on it. The reference classifier stays in the binary
// as the fallback — comment this out to go back to it.
#define USE_MY_CLASSIFIER

// ------------------------------------------------------------------ tuning log
// date / what / why. Read it with the honesty note at the end beside it.
//
// 2026-08-10 / LINE_KD 2000->3000, pid now scaled by base/6000, fixed 2 ms
//   tick / the C loop is faster than the MicroPython loop these gains came
//   from, and explore speed is 1/3 of the speed they were tuned at — the
//   old combination over-steers ~3x and under-damps ~3x, which slams the
//   robot at perpendicular junctions and loses the line.
// 2026-08-10 / blind window is now BLIND_MM of encoder distance, center-3
//   steering while blind / the old 200 ms time window followed junction
//   remnants with the outer sensors at full authority right after a turn.
// 2026-08-10 / added the line-loss recovery block above / a lost line was
//   indistinguishable from a dead end, so the robot would 180 in open
//   floor and wander. It now retraces its own encoder history.
// 2026-08-10 / CENTER_LOST_THRESH hoisted here (was a literal 700 in two
//   places in drive.c), LINE_FOUND_THRESH 600->700 / the back-track
//   declared "found" at 600 while the follower still called the center
//   lost below 700 — a 600-699 graze ends the retrace with a line the
//   follower immediately disowns, and the resume cascades into a false
//   DEADEND on open floor. found >= lost closes the dead zone (see the
//   ORDERING RULE above).
// 2026-08-10 / LOST_CONFIRM_MS 10->20; added BT_FOUND_CONFIRM_TICKS,
//   RESUME_NUDGE_MM, RESUME_NUDGE_TIMEOUT_MS, RESUME_GRACE_MM,
//   CREEP_BAL_GAIN / recovery hardening: 10 ms is thin enough for a glare
//   patch to fake a hard loss mid-straight; a single dark sample could end
//   the retrace on a flash of glare; stop-momentum leaves the bar short of
//   the refound tape (which lies AHEAD — the robot was reversing, so the
//   fix-up creeps FORWARD, never further back); and the resume needs its
//   own encoder baseline before the loss logic may fire again.
//   CREEP_BAL_GAIN names the creep straightness gain that was a literal 8.
// 2026-08-10 / added RECOVERIES_MAX (names the literal 3) and
//   RECOVERY_DECAY_MM / the recovery counter never decayed within a
//   segment, so four spread-out SUCCESSFUL recoveries faulted the same as
//   a tight lawnmower loop. Clean travel now earns credits back; the decay
//   is distance-earned so a genuine loop cannot exploit it — the
//   budget-plus-decay arithmetic is the bound that acts on a useful
//   timescale (the watchdog counts follow-time only and would take
//   minutes; the any-button stop always stands).
// 2026-08-15 / USE_MY_CLASSIFIER on / classify_junction passes the full
//   host suite and the whole-maze simulator at sensor level, so the
//   firmware ships thinking with it. The reference classifier stays
//   compiled into the binary and the simulator proves that lane on every
//   run — comment the flag back out and the simulated robot still solves
//   the maze.
// 2026-08-15 / ROBOT_NAME "robo" / the name picked for the boot splash.
// 2026-08-15 / GOAL_MIN_DARK hoisted here from maze_logic.c (value 4,
//   unchanged) / every number a tuner might reach for lives in tuning.h;
//   GOAL-vs-CROSS is the one verdict a dusty patch edge could flip, so its
//   margin knob belongs next to JCT_DARK_THRESH, findable, not buried in
//   logic/.
// 2026-08-16 / added SPEED_ARRIVAL (2500) + ARRIVAL_BRAKE_MS (80): the
//   arrival window / at a hot replay base the robot enters the creep with
//   momentum, stops past the junction, and classification degrades only at
//   speed. Replay now brakes to SPEED_ARRIVAL the moment
//   drive_until_junction returns, before creep_to_center takes the encoder
//   baseline it measures CREEP_MM from. Defaults are deliberately a no-op
//   at today's SPEED_REPLAY = 2500 — the window only runs when the base is
//   stepped past SPEED_ARRIVAL — so nothing already exercised at 2500
//   changes until a stepped base asks for it.
// 2026-08-16 / added ARRIVAL_BRAKE_MM (20); ARRIVAL_BRAKE_MS demoted from
//   window length to encoder-stall timeout (value 80 unchanged) / review
//   refuted the 80 ms wall-time window with this file's own numbers: at
//   arrival pace (~0.62 m/s) 80 ms is ~50 mm of travel — more than
//   CREEP_MM itself — so the at_center read landed ~95 mm past the detect
//   edge, out the far side of the recommended 75 mm goal patch (45 mm
//   creep + 30 mm margin): every stepped-base goal arrival would fault
//   "replay:not goal". The window's cost is position, so the bound is now
//   distance: 20 mm caps the drift at 65 mm total, 10 mm inside the patch,
//   and the clock survives only as the cannot-hang bound for a stalled
//   encoder.
// 2026-08-16 / CREEP_SETTLE_MS hoisted here (was a literal sleep_ms(60) in
//   creep_to_center; value 60, behavior identical) / the dwell before the
//   at_center read is a knob a tuner may genuinely need — a robot that
//   rocks longer (worn tires, heavier cells) needs a longer settle, and a
//   fast lap trades dwell for time — so it belongs in the cockpit with a
//   name, not buried in drive.c as a bare number.
// 2026-08-18 / BT_PREROLL_SETTLE_MS hoisted here (was a literal
//   sleep_ms(120) at the top of backtrack_recover; value 120, behavior
//   identical) / the pre-retrace settle is correctness-bearing recovery
//   tuning: it decides whether the retrace's encoder chase targets are
//   read from a stopped chassis or a rolling one, and anyone who needs to
//   move it must find it in the cockpit, not buried in drive.c.
//
// HONESTY NOTE — none of the failure stories in this log was OBSERVED.
// Several entries above narrate a robot misbehaving; nobody watched any of
// it happen. Every one is a CODE TRACE: the failure was derived from these
// constants and the control law, the way a fault is derived from a
// schematic. No entry in this log rests on a hardware observation, so any
// clause anywhere above that describes what the robot does or did is
// reasoning. The present tense is not a loophole either — the
// USE_MY_CLASSIFIER entry's claim about the fallback lane is a fact about
// the whole-maze SIMULATOR, which re-proves that lane on every host run,
// not about a robot on a floor. "Validated" anywhere in this log means
// "green in the host suite and the whole-maze simulator", never "seen on a
// floor". When a real run finally happens, the honest record of it is a
// NEW dated entry quoting what the telemetry showed, never an edit to the
// entries above.

#endif // TUNING_H
