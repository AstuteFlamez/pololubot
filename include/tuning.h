// tuning.h — THE one file where your robot's personality lives.
//
// Every number that makes YOUR robot behave differently from the robot next
// to it is in this file. The .c files are machinery; this is the cockpit.
// When something is too slow, too twitchy, too timid — the fix is almost
// always here, not in the code.
//
// These are YOUR numbers. When you change one, write down why (a note at
// the end of this file is fine). Sign your work:
//
//   Tuned by: ________________   Date: ________
//
#ifndef TUNING_H
#define TUNING_H

// ---------------------------------------------------------------- identity
// Shown on the boot splash. A robot with a name gets debugged with more
// patience than "the robot".
#define ROBOT_NAME "robo"

// ------------------------------------------------------------------ speeds
// All speeds are PWM levels out of 6000 (= 100% duty at 20.8 kHz — the
// same scale as Day 6). SPEED_HARD_CAP is enforced inside hw_motors.c:
// no code path in this project can drive faster than it, on purpose.
#define SPEED_HARD_CAP   5000   // absolute ceiling — do not raise on Day 21 at 11pm

#define SPEED_EXPLORE    1800   // sized so no junction can slip past — V6's census is the stage that checks
#define SPEED_REPLAY     2500   // Day 20: replaying a path you trust
#define SPEED_SPEEDRUN   4000   // Day 21: the number on the scoreboard
#define SPEED_CREEP      1200   // rolling the sensors across a junction
#define SPEED_TURN_MAX   2500   // cap for in-place gyro turns
#define SPEED_CALIBRATE  1000   // in-place spin while sweeping sensors over the line

// Day 21 makes SPEED_REPLAY a LADDER, not a number: V7 steps it
// 2500 → 3000 → 3500, one knob per flash, two clean laps per rung — and
// SPEED_SPEEDRUN is the gold rung it is climbing toward. Copy it into
// SPEED_REPLAY only behind V7's gates, never at 11pm. The pass gate per
// rung is a COUNT, not a feeling: JCT_DETECT events in the dump ==
// junctions in the maze. A junction missed at speed doesn't crash — the
// branch line flashes under an outer sensor and the robot sails on,
// politely wrong — so the flight recorder, not the driver's eye, is the
// judge.

// ------------------------------------------------ Day 21: arrival window
// The speed wall the ladder V7 exists to find — DERIVED from the numbers
// in this file, not yet watched happen: raise the replay base toward 4000
// and junctions should start classifying WRONG — the same junctions
// that classify correctly at 2500. Not a threshold problem (the sensors
// are fine) and not a detection problem (even at 4000 ≈ 1 m/s the 2 ms
// tick reads every ~2 mm, and a branch line is ~19 mm of tape — nine
// looks, plus drive.c's edge latch). It is momentum: creep_to_center
// measures its CREEP_MM from wherever it starts, but a chassis needs
// real DISTANCE to shed speed. Enter the creep at 4000-pace and the
// robot is still braking when the budget runs out — it stops PAST the
// junction, the at_center read lands on the wrong patch of floor, and
// the classifier honestly reports what the mis-aimed photo shows.
// Dump signature: C-tick `a` advancing in big steps, then CREEP_END →
// CLASSIFY errors that are absent at 2500.
//
// The fix is a slowdown window, not threshold surgery: the moment
// drive_until_junction returns — motors still running at base, which is
// exactly what drive.h promises — replay brakes to SPEED_ARRIVAL, and
// only then creeps. Straights stay hot; every arrival is COMMANDED
// down to the pace the creep is designed around (how fully 20 mm
// sheds it is V7's question — the contingency sits below).

// The creep-entry speed the ladder STARTS from — not one it has
// blessed. 2500 is Day 20's flat replay pace, the arrival speed every
// piece of arithmetic in this file is written around; no hardware has
// signed it off (no ladder stage has run — see the erratum at the end
// of the tuning log). The window only exists when base >
// SPEED_ARRIVAL, so at SPEED_REPLAY 2500 these knobs change nothing at
// all — V7's first rungs (3000, 3500) are what arm it. If hot entries
// show in the C rows up there, lower this toward SPEED_EXPLORE (1800)
// on V7's arrival-dynamics step: 1800 is the pace explore takes every
// junction at, by design. That turn is
// free under the distance bound below: the brake drags in proportion
// to the excess over the COMMAND (physics at ARRIVAL_BRAKE_MS), so a
// lower command sheds MORE speed in the SAME ARRIVAL_BRAKE_MM of
// floor — this knob buys braking authority without moving the stop.
#define SPEED_ARRIVAL    2500

// How FAR the window may roll, in encoder-measured millimetres. The
// bound is DISTANCE because the window's real cost is POSITION:
// creep_to_center measures its CREEP_MM from wherever the window ends,
// so every window millimetre pushes the deciding at_center read that
// much further past the detect edge. Do that arithmetic before
// trusting any number here. At SPEED_ARRIVAL 2500 (~0.62 m/s by the
// 4000 ≈ 1 m/s rule above) the chassis covers ~6 mm every 10 ms, so a
// wall-time window is a distance nobody chose: the retired 80 ms
// default traveled ~50 mm — more than the creep's entire 45 mm budget
// — landing the at_center read ~95 mm past the detect edge, out the
// FAR side of MAZE_BUILD.md's recommended 75 mm goal patch, whose
// margin past the creep is only 75 − 45 = 30 mm. Every stepped-base
// goal arrival would misread and fault "replay:not goal". Budget at
// THIS default: 20 mm window + 45 mm creep = 65 mm, stopping 10 mm
// inside the patch. Tune with the geometry, not the stopwatch: if V7
// shows overshoot (GOAL misreads on the patch, at_center rows on
// white past a plain junction), THIS knob goes DOWN; if the C rows
// show the creep still entering hot, lower SPEED_ARRIVAL instead —
// and never stretch this past ~25 mm, because 45 + 30 IS the far edge
// of the patch.
#define ARRIVAL_BRAKE_MM  20

// The "never stretch this past ~25 mm" above, enforced: the far edge of
// the goal patch is arithmetic, not advice.
#if ARRIVAL_BRAKE_MM > 25
#error "ARRIVAL_BRAKE_MM > 25 rolls the deciding at_center read out the far side of MAZE_BUILD.md's 75 mm goal patch (window + 45 mm creep vs the patch's 45 + 30): every stepped-base goal arrival would misread and fault 'replay:not goal'. Shed speed by lowering SPEED_ARRIVAL instead."
#endif

// The window's TIMEOUT, not its length — ARRIVAL_BRAKE_MM ends the
// window; this clock only closes it if the encoders stop counting
// (wheel jammed, chassis blocked), so the loop can never hang, and
// on timeout it still hands off DRIVE_OK: the stall diagnosis belongs
// to creep_to_center's watchdog right behind it. The braking physics
// is unchanged — Day 6's stretch question, answered: at every
// off-fraction of the 20.8 kHz PWM cycle these PHASE/ENABLE drivers
// tie the motor terminals together (the "both bottoms closed" H-bridge
// state = braking, not coasting), so a command below the chassis's
// pace drags it toward SPEED_ARRIVAL in proportion to the excess, and
// can never drag it below — but time was the wrong UNIT to bound the
// window with, because its cost is measured in millimetres. 80 ms is
// >2x the ~32 ms the 20 mm bound takes at arrival pace, so on a
// healthy run this clock never fires. V7 still reads the window in the
// dump: its ticks are the C rows between the 'J' snapshot and
// CREEP_START, wearing SPEED_ARRIVAL-sized duty columns.
#define ARRIVAL_BRAKE_MS  80

// Spacing rent the window pays: at a stepped base an arrival now
// consumes window 20 + creep 45 + post-turn blind 30 = 95 mm of
// MAZE_BUILD.md's ≥150 mm junction-spacing minimum (55 mm to spare) —
// pack two junctions closer than that and they merge into ONE arrival,
// whose dump signature is a JCT_DETECT count one short of the junction
// count at stepped bases only, with the merged sweep row fattened by
// both junctions' tape.

// Every SPEED_* knob rides under SPEED_HARD_CAP — enforced here at
// compile time because hw_motors.c enforces it at RUN time by CLAMPING:
// a knob above the cap doesn't crash, it LIES. The wheels get
// SPEED_HARD_CAP while every derived arithmetic — base/6000 curvature
// scaling, the arrival window's excess-speed story, the ladder's rung
// math — reasons from a speed the motors never deliver. (This #if sits
// down here, after SPEED_ARRIVAL, because the preprocessor treats a
// not-yet-defined name as 0 and would wave a misordered check through.)
#if SPEED_EXPLORE > SPEED_HARD_CAP || SPEED_REPLAY > SPEED_HARD_CAP || \
    SPEED_SPEEDRUN > SPEED_HARD_CAP || SPEED_CREEP > SPEED_HARD_CAP || \
    SPEED_TURN_MAX > SPEED_HARD_CAP || SPEED_CALIBRATE > SPEED_HARD_CAP || \
    SPEED_ARRIVAL > SPEED_HARD_CAP
#error "A SPEED_* knob exceeds SPEED_HARD_CAP: hw_motors.c clamps every command to the cap, so the motors would silently run at SPEED_HARD_CAP while the tuning arithmetic (base/6000 scaling, the arrival window) reasons from a speed they never deliver. Lower the knob - the cap never moves, least of all on Day 21 at 11pm."
#endif

// -------------------------------------------------------- line-follow PID
// Same shape as Pololu's own line follower (line_follower.py):
//     pid = (p * LINE_KP + d * LINE_KD) * base / 6000
// where p = (line position 0..4000) - 2000 and d = p - last_p.
// pid is added to one wheel and subtracted from the other, then clamped
// to [0, base].
//
// Two normalizations make Pololu's numbers mean here what they meant
// there (they tuned at base = 6000 in a ~3 ms MicroPython loop):
//   * base/6000 — steering is differential drive, so the SAME curvature
//     needs a differential proportional to speed. Without this, running
//     at explore speed (1800) triples the effective P gain and the
//     follower turns into a twitchy bang-bang controller.
//   * d = p - last_p is per-TICK, so KD's strength depends on the tick
//     rate. Our C loop runs a fixed CONTROL_PERIOD_US (2 ms) vs their
//     ~3 ms, so KD is seeded at 3000 (= 2000 * 3/2) to give the same
//     damping. Symptom check: fishtailing = raise KD or lower KP.
//
// Day 21 corollary: base/6000 is also what lets V7 step the replay base
// 2500→3500 without touching KP — same p, same commanded curvature, at
// any base. What the scaling cannot hide is ACTUATOR LAG: motors take
// real time to change a wheel's speed, so at 3500 the same correction
// arrives after more millimetres of travel than it did at 1800, and a
// straight that is calm at explore pace can weave at replay pace. That
// is LINE_KD's fight, and the ladder validates it with data: same
// straight at 1800 vs 3500 — if high speed oscillates while low is
// clean, that's actuator-lag physics (KD +500, then re-check low speed
// for chatter), not a broken follower.
#define LINE_KP  90
#define LINE_KD  3000

// The follower runs at exactly this period (the sensor read itself takes
// up to ~1.1 ms, so don't go below ~1500). A FIXED period is what makes
// the D term trustworthy: on a free-running loop the read time varies
// ~7x with how much tape is under the bar, and the damping varies with it.
#define CONTROL_PERIOD_US  2000

// -------------------------------------------------- junction detection
// Calibrated readings are 0 (bright white) .. 1000 (deep black).
#define JCT_OUTER_THRESH  600  // an outer sensor above this = a side branch
#define LINE_LOST_THRESH  300  // all sensors below this = the line is gone

// The classifier's thresholds (logic/maze_logic.c). tuning.h is nothing
// but #defines, which is why the pure logic/ layer may include it: the
// brain gets this robot's numbers without ever touching an SDK header.
//
// One reading counts as "dark" at or above this line. It is a knob, not
// a constant of nature: it encodes THIS tape and THIS calibration (the
// build QC keeps paper well under ~200 and tape well over ~700, so one
// number can separate them), never anything about the algorithm.
#define JCT_DARK_THRESH   600

// GOAL vs CROSS: how many at_center sensors must read dark to call the
// goal patch. Both blacken the whole bar on approach; after the creep a
// crossing leaves only its straight exit under the bar (1–2 dark
// sensors), while the goal patch still darkens all 5. Requiring 4 of 5
// instead of all 5 keeps one sensor riding the patch edge through a
// dusty spot from turning GOAL into CROSS — the gap between "1–2" and
// "4" is the margin. Its other edge is DERIVED, not tuned:
// maze_logic.c's CROSS_MAX_DARK (= GOAL_MIN_DARK - 2) keeps the gap
// exactly one count wide wherever this knob moves, and a count in the
// gap refuses as JCT_NONE — the full argument lives at that #define.
// sim_maze.c's marginal-world suite proves the analysis with
// near-threshold readings.
#define GOAL_MIN_DARK  4

// The margin story above only holds while this knob stays where the
// bar's own geometry can honor it — enforced at compile time.
#if GOAL_MIN_DARK < 3 || GOAL_MIN_DARK > 5
#error "GOAL_MIN_DARK must stay in 3..5: below 3 the derived CROSS_MAX_DARK (GOAL_MIN_DARK - 2) drops under the 1-2 dark sensors an ordinary crossing's exit legitimately leaves, so crossings misread as GOAL or refuse; above 5 it asks for more dark sensors than the bar has, and the goal patch can never classify."
#endif

// ---------------------------------------------------------------- geometry
// Wheels: 3.56 encoder counts per mm (Day 10 math: ~358.3 counts/rev on a
// 32 mm wheel). Stored ×100 so the code stays in integers.
#define ENC_COUNTS_PER_MM_X100  356

// If a wheel counts BACKWARD when the robot drives forward (check on the
// DIAG screen — spin each wheel forward by hand), flip its sign here.
#define ENC_SIGN_LEFT   (+1)
#define ENC_SIGN_RIGHT  (+1)

// How far to roll forward after detecting a junction, so the wheel axle
// (the robot's turning center) sits on the junction center. The sensor bar
// rides ahead of the axle — this closes that gap. Tune on YOUR maze: after
// a 90° turn the robot should be centered on the new line.
#define CREEP_MM  45

// How long the creep waits after its stop before taking the at_center
// read. A stop is not instant stillness: the chassis pitches on its
// tires, and the sensor bar — cantilevered ahead of the axle — sweeps
// millimetres of floor while the body rocks. The at_center read is THE
// deciding input (the classifier's "money read"); taken mid-rock it can
// straddle a tape edge and hand the brain a smeared photo, so the creep
// stops, outwaits the rock, and only then reads. Same physics as
// BT_SETTLE_MS; the creep's stop is gentler (it arrives at SPEED_CREEP,
// not out of a mid-correction loss), so its default sits lower.
#define CREEP_SETTLE_MS  60

// Post-turn blind window, in DISTANCE (encoder-measured), not time: right
// after a turn the outer sensors may still see pieces of the junction we
// just handled, so junction detection stays off — and the follower steers
// from the CENTER THREE sensors only — until the bar has rolled this far
// onto clean line. Distance, because the same window must work at explore
// AND replay speed.
#define BLIND_MM  30

// ------------------------------------------------- line-loss recovery
// The whole bar going white for LOST_CONFIRM_MS means the line is gone.
// Which KIND of gone is decided by how hard the follower was correcting
// just before (max |p| over its recent history):
//   * recent max |p| <= DEADEND_P_MAX — we were driving straight and the
//     line simply stopped: an honest dead end, report it to the classifier.
//   * recent max |p| >  DEADEND_P_MAX — we were mid-correction and drove
//     OFF the tape: back-track (retrace the recorded encoder history in
//     reverse, slowly) until a center sensor sees tape again, then resume.
#define LOST_CONFIRM_MS     20
#define DEADEND_P_MAX       600
#define CENTER_LOST_MAX_MS  250   // center gone this long while slamming = lost too

// The follower calls the CENTER of the bar "lost" when all three middle
// sensors read below CENTER_LOST_THRESH — from that tick on it steers
// from memory, not measurement. The back-track calls the line "found"
// when a middle sensor reads at or above LINE_FOUND_THRESH.
//
// ORDERING RULE:  CENTER_LOST_THRESH <= LINE_FOUND_THRESH.  Always.
// These two thresholds are a handoff: the back-track finds the line, the
// follower takes it from there. If "found" could fire on a reading the
// follower still calls "lost" (found threshold below lost threshold),
// there is a dead zone — a graze in between ends the retrace, the
// follower resumes already lost, and a real line turns into a false dead
// end. Equal (or found higher) means every refind is one the follower
// will accept. Corollary: V1's "tape reads >700 calibrated" gate is
// load-bearing — tape that calibrates below LINE_FOUND_THRESH can never
// be refound after a loss.
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
#define BACKTRACK_TIMEOUT_MS 6000

// Ending the retrace on a SINGLE dark sample would let a glare flash or a
// tape seam "find" a line that isn't there. One sample is an opinion;
// this many consecutive samples are a measurement. At the back-track tick
// rate (CONTROL_PERIOD_US x BACKTRACK_TICK_X = 4 ms) three ticks = 12 ms
// of continuous tape — a one-sample artifact can't fake that, and the
// extra distance retraced while confirming is well under a millimeter.
#define BT_FOUND_CONFIRM_TICKS  3

// How long the chassis needs after hw_motors_stop() before a sensor
// read (or the follower's first p) can be trusted. A stop is not
// instant stillness: the body pitches on its tires and rocks for a few
// hundredths of a second, and the sensor bar — cantilevered out ahead
// of the axle — sweeps millimetres across the floor while it does. A
// read taken mid-rock can straddle the tape edge. 80 ms outlasts the
// rock at recovery speeds; V5 may tune it.
#define BT_SETTLE_MS  80

// The settle before the retrace MEASURES anything. backtrack_recover's
// first act after stopping is to read the encoders — those two numbers
// become the chase targets the whole retrace steers by, so they must
// come from a chassis that has genuinely stopped: every millimetre
// coasted AFTER that read is a millimetre the retrace will chase back
// as phantom error it never recorded. This is also the hottest stop in
// the recovery — the robot arrives out of a full-pace swerve at base
// speed, mid-correction, not out of a BACKTRACK_SPEED crawl — so it
// outwaits more momentum than BT_SETTLE_MS does. V5 may tune it
// (heavier cells rock longer; a hotter base coasts further).
#define BT_PREROLL_SETTLE_MS  120

// The retrace refinds the line while REVERSING, and momentum keeps
// carrying the robot backward through the stop — so when the settle
// check afterwards sees bare floor, the tape it just found lies AHEAD of
// the sensor bar, never behind it. The fix-up is a slow FORWARD creep
// toward the find point, bounded in distance AND time (every motor loop
// keeps a timeout and the any-button stop).
#define RESUME_NUDGE_MM          20   // beyond any plausible stop-coast
#define RESUME_NUDGE_TIMEOUT_MS  800  // stalled-wheel escape hatch

// The recovery budget: how many back-track parachute pulls one follow
// segment gets before the drive gives up and faults (DRIVE_LOST). The
// budget is about REPEATED failure — a follower that keeps falling off
// the same stretch of tape is broken, and recovery must not turn that
// into an infinite lawnmower pattern.
#define RECOVERIES_MAX  3

// ...but a SUCCESSFUL recovery followed by a long stretch of clean
// following is not repeated failure — it's the parachute working. Each
// RECOVERY_DECAY_MM of travel earns one budget credit back (never past
// a full budget), so four one-off recoveries spread across a long
// segment don't add up to a fault. The decay is distance-earned, which
// is what bounds it: a genuine lawnmower loop loses the line again
// within a few centimetres, earns nothing back, and still hits the
// budget. Be clear-eyed about timescales, though: this arithmetic is
// the only guard that acts QUICKLY on a cycling recovery. The run
// watchdog counts follow-time only (recovery time is credited back —
// see the resume in drive.c) and a loss cycle accrues just ~80+ ms of
// it, so DRIVE_TIMEOUT would end the churn eventually — after minutes,
// not seconds. The any-button stop always stands. Rely on the budget;
// treat the watchdog as the slow fence behind it.
#define RECOVERY_DECAY_MM  200

// Grace distance after a recovery resume before "all white" may again be
// judged a hard loss. The resume re-arms with an empty |p| history, so
// without this gate a single white read at the resume point would fault
// as a dead end with a recent-max-|p| of zero. Distance, not time, for
// the same reason as BLIND_MM: it must mean the same thing at every
// speed. Must be measured from a FRESH encoder baseline taken at the
// resume itself (drive.c explains the trap).
#define RESUME_GRACE_MM          15

// Straightness servo gain while creeping: duty correction per encoder
// count of left-minus-right disagreement (Day 10's balance trick).
// Shared by the junction creep, the recovery nudge, and the Day-21
// arrival window — one gain, one meaning.
#define CREEP_BAL_GAIN  8

// Pause at each junction after the decision is shown. Long enough for a
// human (and the camera) to see the robot "think". Shrink it on Day 21.
#define THINK_PAUSE_MS  400

// ------------------------------------------------------------- gyro turns
// PD controller on gyro angle, straight from Pololu's gyro_turn.py
// (kp=140, kd=4, ±3°, settled for 250 ms). If the robot stalls just short
// of the target and times out, raise TURN_KP a little.
#define TURN_KP          140
#define TURN_KD          4
#define TURN_TOL_DEG     3
#define TURN_SETTLE_MS   250
#define TURN_TIMEOUT_MS  2500

// -------------------------------------------------------------- watchdogs
// If the robot line-follows this long without finding a junction, it has
// almost certainly escaped the maze. Stop and say so instead of touring
// the living room.
#define RUN_WATCHDOG_MS  10000
#define CREEP_TIMEOUT_MS 1500

// ---------------------------------------------------------------- battery
// 4× NiMH AAA: 4.8 V nominal. Below ~1.1 V/cell the motors get weak and
// your PID tuning turns into fiction; below ~1.0 V/cell readings are junk.
#define BATT_WARN_MV    4400   // nag on screen
#define BATT_REFUSE_MV  4000   // refuse to start a run

// -------------------------------------------------------- calibration gate
// After calibration, every sensor must have seen at least this much span
// between its white and black readings, or CAL is rejected. A sensor with
// span < 300 was probably never over the tape (or the tape is glossy).
#define CAL_MIN_SPAN  300

// ------------------------------------------------------------- the brain
// classify_junction() passes the full host suite and both simulator
// lanes, so the firmware runs on it. The reference classifier stays in
// the binary as the fallback — comment this out to go back.
#define USE_MY_CLASSIFIER

// ------------------------------------------------------------- tuning log
// date / what / why:
//
// 2026-08-10 / [agent-default] LINE_KD 2000->3000, pid now scaled by base/6000, fixed 2 ms
//   tick / the C loop is faster than the MicroPython loop these gains came
//   from, and explore speed is 1/3 of the speed they were tuned at — the
//   old combination over-steered ~3x and under-damped ~3x, which is why
//   the robot slammed at perpendicular junctions and lost the line.
//   (Course-maintainer fix in GIVEN plumbing — see branch capstone-drive-fixes.)
// 2026-08-10 / [agent-default] blind window is now BLIND_MM of encoder distance, center-3
//   steering while blind / the old 200 ms time window followed junction
//   remnants with the outer sensors at full authority right after a turn.
// 2026-08-10 / [agent-default] added line-loss recovery block above / a lost line used to
//   be indistinguishable from a dead end — the robot would 180 in open
//   floor and wander. Now it retraces its own encoder history.
// 2026-08-10 / CENTER_LOST_THRESH hoisted here (was a literal 700 in two
//   places in drive.c), LINE_FOUND_THRESH 600->700 / the back-track
//   declared "found" at 600 while the follower still called the center
//   lost below 700 — a 600-699 graze ended the retrace with a line the
//   follower immediately disowned, and the resume cascaded into a false
//   DEADEND on open floor. found >= lost closes the dead zone (see the
//   ORDERING RULE above). [agent]
// 2026-08-10 / LOST_CONFIRM_MS 10->20; added BT_FOUND_CONFIRM_TICKS,
//   RESUME_NUDGE_MM, RESUME_NUDGE_TIMEOUT_MS, RESUME_GRACE_MM,
//   CREEP_BAL_GAIN / recovery hardening: 10 ms was thin enough for a
//   glare patch to fake a hard loss mid-straight; a single dark sample
//   could end the retrace on a flash of glare; stop-momentum left the
//   bar short of the refound tape (which lies AHEAD — the robot was
//   reversing, so the fix-up creeps FORWARD, never further back); and
//   the resume needs its own encoder baseline before the loss logic may
//   fire again. CREEP_BAL_GAIN names the creep straightness gain that
//   was a literal 8. [agent]
// 2026-08-10 / added RECOVERIES_MAX (names the literal 3) and
//   RECOVERY_DECAY_MM / the recovery counter never decayed within a
//   segment, so four spread-out SUCCESSFUL recoveries faulted the same
//   as a tight lawnmower loop. Clean travel now earns credits back;
//   the decay is distance-earned so a genuine loop can't exploit it —
//   the budget-plus-decay arithmetic is the bound that acts on a
//   useful timescale (the watchdog counts follow-time only and would
//   take minutes; the any-button stop always stands). [agent]
// 2026-08-15 / USE_MY_CLASSIFIER on / classify_junction now passes the
//   full host suite and the whole-maze simulator at sensor level, so the
//   firmware ships thinking with it. The reference classifier stays
//   compiled into the binary and the simulator proves that lane on every
//   run — comment the flag back out and the robot still solves the maze.
//   [agent]
// 2026-08-15 / ROBOT_NAME "robo" / the name Bilal picked for the boot
//   splash — a robot with a name gets debugged with more patience. [agent]
// 2026-08-15 / GOAL_MIN_DARK hoisted here from maze_logic.c (value 4,
//   unchanged) / every number a tuner might reach for lives in tuning.h;
//   GOAL-vs-CROSS is the one verdict a dusty patch edge could flip, so
//   its margin knob belongs next to JCT_DARK_THRESH, findable, not
//   buried in logic/. [agent]
// 2026-08-16 / added SPEED_ARRIVAL (2500) + ARRIVAL_BRAKE_MS (80): the
//   Day-21 arrival window / at a hot replay base the robot enters the
//   creep with momentum, stops past the junction, and classification
//   degrades only at speed (the ladder's V7 wall). Replay now brakes to
//   SPEED_ARRIVAL the moment drive_until_junction returns, before
//   creep_to_center takes the encoder baseline it measures CREEP_MM
//   from. Defaults are deliberately a no-op at today's SPEED_REPLAY =
//   2500 — the window only runs when the base is stepped past
//   SPEED_ARRIVAL (V7's 3000/3500 rungs), so nothing validated on Day
//   20 changes until the ladder itself asks for it. [agent]
// 2026-08-16 / added ARRIVAL_BRAKE_MM (20); ARRIVAL_BRAKE_MS demoted from
//   window length to encoder-stall timeout (value 80 unchanged) / review
//   refuted the 80 ms wall-time window with the repo's own numbers: at
//   arrival pace (~0.62 m/s) 80 ms is ~50 mm of travel — more than
//   CREEP_MM itself — so the at_center read landed ~95 mm past the
//   detect edge, out the far side of the recommended 75 mm goal patch
//   (45 mm creep + 30 mm margin): every stepped-base goal arrival would
//   fault "replay:not goal", and V7's two-clean-laps gate could not pass
//   at the shipped default. The window's cost is position, so the bound
//   is now distance: 20 mm caps the drift at 65 mm total, 10 mm inside
//   the patch, and the clock survives only as the can't-hang bound for
//   a stalled encoder. [agent]
// 2026-08-16 / CREEP_SETTLE_MS hoisted here (was a literal sleep_ms(60)
//   in creep_to_center; value 60, behavior identical) / the dwell before
//   the at_center money read is a knob a tuner may genuinely need — a
//   robot that rocks longer (worn tires, heavier cells) needs a longer
//   settle, and Day 21 trades dwell for lap time — so it belongs in the
//   cockpit with a name, not buried in drive.c as a bare number. [agent]
// 2026-08-18 / BT_PREROLL_SETTLE_MS hoisted here (was a literal
//   sleep_ms(120) at the top of backtrack_recover; value 120, behavior
//   identical) / the pre-retrace settle is correctness-bearing recovery
//   tuning: it decides whether the retrace's encoder chase targets are
//   read from a stopped chassis or a rolling one, and a V5 session that
//   needs to move it must find it in the cockpit, not buried in
//   drive.c. [agent]
// 2026-08-18 / [erratum] the failure stories above were REASONED, never
//   OBSERVED / a tuning log is a record of what was decided when, so the
//   entry bodies above stay byte-frozen — but six of them narrate a
//   robot misbehaving, and a reader is owed the fact that nobody watched
//   any of them happen: "[agent-default] LINE_KD 2000->3000" ("the robot
//   slammed at perpendicular junctions and lost the line");
//   "[agent-default] blind window is now BLIND_MM" ("the old 200 ms time
//   window followed junction remnants"); "[agent-default] added
//   line-loss recovery block above" ("the robot would 180 in open floor
//   and wander"); "CENTER_LOST_THRESH hoisted here" (a 600-699 graze
//   that "ended the retrace ... and the resume cascaded into a false
//   DEADEND on open floor"); "LOST_CONFIRM_MS 10->20" ("stop-momentum
//   left the bar short of the refound tape"); and "added SPEED_ARRIVAL
//   (2500) + ARRIVAL_BRAKE_MS (80)" ("the robot enters the creep with
//   momentum, stops past the junction"). Every one of those is a CODE
//   TRACE: the failure derived from these constants and the control law,
//   the way you derive a fault from a schematic. Do not read the count
//   as a whitelist — NO entry in this log rests on a hardware
//   observation, so any clause anywhere above that describes what the
//   robot does or did is reasoning, listed here or not. The present
//   tense is not a loophole either: the 2026-08-15 USE_MY_CLASSIFIER
//   entry's "comment the flag back out and the robot still solves the
//   maze" is a fact about the whole-maze SIMULATOR, which re-proves the
//   fallback lane on every host run — not about a robot on a floor.
//   No stage of
//   capstone/VALIDATION_LADDER.md has been run, capstone/validation/
//   holds no CSVs, and therefore "validated" anywhere in this log means
//   "green in the host suite and the whole-maze simulator" — never "seen
//   on a floor". Read the entries with this note beside them; when a
//   ladder stage finally runs, the honest record of it is a NEW dated
//   entry quoting what the telemetry showed, never an edit to the ones
//   above. [agent]
//

#endif // TUNING_H
