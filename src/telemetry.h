// telemetry.h — black-box flight recorder for the drive layer.
//
// A maze failure happens in 50 ms, twenty feet from the bench, with no USB
// cable attached. Printing from the control loop is not an alternative:
// USB CDC writes can stall for milliseconds, and a millisecond hole in a
// 2 ms control law is a visible wobble (the same reason ui.h never flushes
// the display mid-maneuver). So every control tick appends one 16-byte
// record to a RAM ring buffer — about a microsecond, no I/O, no allocation
// — and the ring holds the last ~16 seconds (8192 records at 500 Hz). Back
// on the bench with USB attached, the LOG DUMP menu entry prints the ring
// as CSV; tools/capture_log.sh automates the capture and
// tools/plot_telemetry.py turns the CSV into plots.
//
// The ring is SRAM: cutting power destroys it, and telemetry_reset() runs
// at the start of every run, so one run yields at most one dump.
//
// Two loops bend the one-record-per-tick rule on purpose. The creep and
// gyro-turn loops free-run (no fixed tick — each pass takes as long as its
// sensor work takes); the creep still records every pass, but the turn
// loop records a 'T' row only every ~16 ms, so a long spin cannot flush 16
// seconds of history out of the ring.
//
// Concurrency: the ring is written from exactly ONE place (whichever
// maneuver loop is active — they never overlap) and only dumped with the
// motors stopped. No locks needed, on purpose.
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// CSV DUMP FORMAT
//
// This block is the format's only specification: telemetry_dump() writes
// it, tools/plot_telemetry.py reads it, tests/telemetry/check_decode.py
// pins it. Keep all three in step.
//
// Frame: exactly three '#' header lines, the column-header row, one row per
// record, then '# end'. The three-line count is part of the contract.
//
//   1. Format version:  "# 3pi2040 telemetry v1"
//   2. The knob line — the build that produced the dump, as k=v tokens
//      copied from include/tuning.h: speeds (explore, replay, arrival,
//      creep, turnmax, bt_speed), gains (kp, kd, turn_kp), thresholds
//      (outer_thresh, lost_thresh, dark_thresh, goal_min_dark,
//      deadend_p_max, cal_min_span), geometry and time (blind_mm, creep_mm,
//      brake_mm, brake_ms, think_ms, counts_per_mm_x100, period_us). A
//      shared CSV has to name its own build: the robot may be several
//      flashes ahead by the time anyone reads the file. However long the
//      list grows, it stays ONE '#' line.
//   3. The dump-facts line — what is true of THIS dump: records=, wrapped=,
//      and, when main.c supplied them, batt_mv= (battery millivolts) and
//      cal_min=/cal_max= (five comma-separated values each: the line
//      calibration window, max-min per sensor being the span judged against
//      cal_min_span).
//
// The header describes DUMP time, not run time. batt_mv is measured with
// the motors long stopped, so the loaded voltage during the run was lower.
// The calibration window is likewise read at dump time; it is the window
// that scaled the rows below only if nothing recalibrated between the run
// and the dump (the ring is cleared at run start, the calibration arrays
// are not). Recalibrate in between and the header honestly reports the NEW
// window over rows still carrying the old one's scaling.
//
// Column row:  t_ms,rec,s0,s1,s2,s3,s4,a,b,left,right
//
//   t_ms    Robot clock in milliseconds, stored as uint16 — it WRAPS at
//           65536. plot_telemetry.py unwraps it; raw values must not be
//           differenced by hand across a wrap.
//   rec     One letter for a tick row, a name for an event row.
//   s0..s4  Calibrated line sensors, 0 (white paper) .. 1000 (black tape);
//           s0 leftmost, s4 rightmost. Stored in the ring divided by 4 and
//           re-expanded x4 on dump, so values land on multiples of 4 — an
//           848 here and an 850 on the display are the same reading.
//           Populated on ticks; 0 on events.
//   a, b    Meaning depends on rec — see the two tables below.
//   left,   The duty drive.c COMMANDED for this tick, on the +/-6000 PWM
//   right   scale, recorded BEFORE hw_motors.c clamps it to the +/-5000
//           SPEED_HARD_CAP. F/B rows are already bounded to [0, base] and T
//           rows to SPEED_TURN_MAX, so those cannot exceed the cap. C rows
//           can: they log SPEED_CREEP +/- bal (or SPEED_ARRIVAL +/- bal in
//           the arrival window) and the straightness term bal is unbounded,
//           so a large left/right count disagreement prints a duty the
//           wheels never received. A C row past +/-5000 is a real signal —
//           the balance servo is fighting something — not a decode error.
//           Populated on ticks; 0 on events.
//
// Tick rows — one per control tick, with two exceptions: T rows are
// decimated to ~60 Hz, and J/A are one-shot snapshots rather than phases
// the robot spends time in.
//
//   rec  phase                      a                         b
//   ---  -------------------------  ------------------------  ----------------
//   F    line follow                position error p          PID steer command
//                                   (-2000..+2000, 0 = mid)
//   B    post-turn blind window     as F, from the center-3   as F
//                                   sensor estimate
//   C    creep                      progress, encoder counts  balance correction
//   T    gyro turn (~60 Hz)         angle error, deg x10      commanded speed
//   K    backtrack                  left encoder retrace err  right retrace err
//   J    snapshot at JCT_DETECT     latched s0                latched s4
//   A    snapshot at CREEP_END      0                         0
//
//   C is emitted by three callers sharing one letter: the junction roll-in,
//   the recovery's forward nudge (so C ticks legitimately appear between
//   BT_FOUND and RESUME), and the replay arrival window (its C ticks sit
//   between the J snapshot and CREEP_START, wearing SPEED_ARRIVAL-sized
//   duty columns). Which target a C row's `a` climbs toward depends on
//   which emitter: CREEP_MM, RESUME_NUDGE_MM, or ARRIVAL_BRAKE_MM.
//   J and A are the classifier's two actual inputs; every other row is what
//   the bar saw on the way. J's s0/s4 are the edge-latched values the
//   classifier receives, not this instant's raw read, and its left/right
//   are the follow command still running. A is the settled at_center read,
//   taken stopped, so a, b, left and right are all 0.
//
// Event rows — one per occurrence, rec is the name.
//
//   RUN_START     a = run mode (run_mode_t in src/feedback.h: 0 DIAG,
//                     1 CALIBRATE, 2 EXPLORE, 3 SOLVED, 4 REPLAY, 5 DONE,
//                     6 SPARE, 7 LOGDUMP; a maze run is always 2 or 4)
//                 b = THIS run's base speed. A stepped replay base lives
//                     here, not on the knob line.
//   FOLLOW_START  a = base speed
//   BLIND_END     a = encoder counts travelled while blind
//   JCT_DETECT    a = latched s0, b = latched s4 (the trigger)
//   DEADEND       a = recent max |p| (small = honest dead end)
//   LOSS          a = recent max |p|, b = ms the center was lost
//   BT_START      a = history entries available to retrace
//   BT_FOUND      a = ms spent, b = entries consumed
//   BT_FAIL       a = ms spent, b = entries consumed. Three causes: history
//                     ran out, timed out, or the refound line did not
//                     survive the settle re-check — that last one trails
//                     its own BT_FOUND (a find demoted, not a decode error;
//                     see bt_found_finish in drive.c).
//   CREEP_START   -
//   CREEP_END     -
//   TURN_START    a = target delta, deg x10
//   TURN_END      a = residual error, deg x10, b = ms taken
//   CLASSIFY      a = junction_t in logic/maze_logic.h: 0 LEFT_ONLY,
//                     1 RIGHT_ONLY, 2 STRAIGHT_LEFT, 3 STRAIGHT_RIGHT, 4 T,
//                     5 CROSS, 6 DEAD_END, 7 GOAL, 8 NONE
//   ABORT         a = the tick kind number the abort happened in
//   TIMEOUT       a = the tick kind number
//   FAULT         -
//   RESUME        a = the segment's recovery count at this resume
//                 b = suppressed span ahead of the resume, in mm: how far
//                     junction detection stays holstered while the bar
//                     re-covers ground this segment already cleared
//                     (drive.c's resume protocol). 0 means the refind
//                     landed at or past the suppression bound — the LATER
//                     of the arm point and the departure point,
//                     max(arm_enc, ontape_enc) — so nothing was suppressed.
//                     Lose the line behind the arm point and that bound IS
//                     the arm point. The span leaves no other trace:
//                     suppressed detection looks exactly like open corridor.
//
// Two facts that decide most triage:
//
//   * A dead end produces an A row but NO J row. J is emitted only at
//     JCT_DETECT; the honest-dead-end path fills `before` inside the follow
//     loop and returns without a detect, then still creeps to center. The
//     missing J is by design, not a dropped record.
//   * Follow rows carry no odometry. On F/B rows left/right are commanded
//     duty and a/b are the error and the steer — nothing in a follow row
//     says how far the robot has travelled, and ticks x speed slips exactly
//     when the wheels do. Audit a distance knob (BLIND_MM, CREEP_MM,
//     ARRIVAL_BRAKE_MM, RESUME_NUDGE_MM) at the rows that do carry encoder
//     numbers instead: BLIND_END's a (counts travelled blind — compare
//     against BLIND_MM x counts_per_mm_x100/100), the C rows' a (creep
//     progress in counts), the K rows' per-wheel retrace errors, and
//     RESUME's b (suppressed span, already in mm).
//
// '# end' is the completeness receipt. The firmware prints it last,
// unconditionally, so a CSV without it means the CAPTURE was cut off
// (interrupted too early, cable pulled) and the tail of the run — usually
// the part worth reading — may be missing. plot_telemetry.py warns and
// still parses what survived: a truncated dump is evidence that must say so.
//
// Kind and event NUMBERS ARE APPEND-ONLY. Every CSV ever dumped encodes
// them, so renumbering an existing kind silently re-labels history. Old
// dumps simply lack newer k=v tokens and decoders treat every token as
// optional: the format only ever grows, inside the same three-line frame.
// ---------------------------------------------------------------------------

// ---- tick records (a/b meanings in the tick table above)
#define TEL_TICK_FOLLOW 1  // 'F' row: line follow
#define TEL_TICK_BLIND  2  // 'B' row: post-turn blind window
#define TEL_TICK_CREEP  3  // 'C' row: creep (three emitters share it)
#define TEL_TICK_TURN   4  // 'T' row: gyro turn, decimated to ~60 Hz
#define TEL_TICK_BT     5  // 'K' row: backtrack
// Two ONE-SHOT snapshot kinds — not phases, single records. They exist
// because the classifier's two actual inputs were the one thing the log
// never contained: the F/C ticks around a junction show what the bar saw
// tick by tick, but not the edge-latched/settled values the brain is
// actually handed. These are tick records (not events) so they keep their
// five sensor columns in every downstream tool.
#define TEL_TICK_JCT_SNAP    6  // 'J' row: the edge-latched `before` at JCT_DETECT
#define TEL_TICK_CENTER_SNAP 7  // 'A' row: the at_center read at CREEP_END

// ---- event records (a/b meanings in the event table above)
#define EV_RUN_START    16
#define EV_FOLLOW_START 17
#define EV_BLIND_END    18
#define EV_JCT_DETECT   19
#define EV_DEADEND      20
#define EV_LOSS         21
#define EV_BT_START     22
#define EV_BT_FOUND     23
#define EV_BT_FAIL      24
#define EV_CREEP_START  25
#define EV_CREEP_END    26
#define EV_TURN_START   27
#define EV_TURN_END     28
#define EV_CLASSIFY     29
#define EV_ABORT        30
#define EV_TIMEOUT      31
#define EV_FAULT        32
#define EV_RESUME       33
// Numbering is append-only (see the format block). New kinds go at the end,
// and the dump tables plus range checks in telemetry.c — and the decoder
// tables in tools/plot_telemetry.py — must grow in the same commit.

void telemetry_init(void);   // once at boot
void telemetry_reset(void);  // at each run start: empty the ring

// One control tick. line = calibrated 0..1000 (stored /4 — 4-count
// resolution is plenty for pictures). left/right = commanded motor duty.
void telemetry_tick(uint8_t phase, const uint16_t line[5],
                    int16_t a, int16_t b, int16_t left, int16_t right);

// One event record, appended to the same ring against the same clock.
// ev = an EV_* kind; a/b as listed in the event table above (0 where the
// table shows "-"). The five sensor columns are stored as 0: an event is a
// moment, not a reading.
void telemetry_event(uint8_t ev, int16_t a, int16_t b);

uint32_t telemetry_count(void);  // records currently held (caps at ring size)

// Dump-time context, handed in by main.c just before telemetry_dump().
// This is a hand-off rather than a lookup because telemetry OBSERVES: it
// never reaches into hw_battery/hw_line itself.
// batt_mv = battery millivolts read at dump time (resting — the motors have
// been stopped for the walk back to the bench, so the loaded voltage during
// the run was LOWER than this number). cal_min/cal_max = the line
// calibration window as it stands AT DUMP TIME; see the format block for
// when that is not the window the rows were scaled by.
// Optional: without a call, the dump prints its original header shape, so
// old callers and old CSVs stay valid.
void telemetry_set_dump_context(uint16_t batt_mv,
                                const uint16_t cal_min[5],
                                const uint16_t cal_max[5]);

// Print the whole ring as CSV over USB stdio, in the format specified
// above. Takes seconds — only ever call it with the motors stopped.
void telemetry_dump(void);

#endif
