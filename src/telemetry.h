// telemetry.h — black-box flight recorder for the drive layer.
//
// WHY: a maze failure happens in 50 ms, twenty feet from the nearest
// printf, with USB unplugged. Printing from a control loop is not an
// option anyway — USB CDC writes can stall for milliseconds, and a
// millisecond hole in a 2 ms control law is a wobble you can SEE (same
// reason ui.h bans flushes mid-maneuver). So instead: every control tick
// appends one 16-byte record to a RAM ring buffer — about a microsecond,
// no I/O, no allocation — and the ring remembers the last ~16 seconds of
// truth (8192 records at 500 Hz). Back on the bench, USB in, the
// LOG DUMP menu entry prints the ring as CSV; capstone/plot_telemetry.py
// turns the CSV into pictures.
//
// Concurrency: the ring is written from exactly ONE place (whichever
// maneuver loop is active — they never overlap) and only dumped with the
// motors stopped. No locks needed, on purpose.
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

// ---- tick records (one per control tick; a,b meaning depends on phase)
#define TEL_TICK_FOLLOW 1  // line follow:  a = p (pos-2000), b = pid steer
#define TEL_TICK_BLIND  2  // post-turn blind window: same, center-3 estimate
#define TEL_TICK_CREEP  3  // creep:        a = progress (enc counts), b = balance
#define TEL_TICK_TURN   4  // gyro turn:    a = angle err deg*10, b = cmd speed
#define TEL_TICK_BT     5  // backtrack:    a = left enc err, b = right enc err
// Two ONE-SHOT snapshot kinds — not phases, single records. They exist
// because the classifier's two actual inputs were the one thing the log
// never contained: the F/C ticks around a junction show what the bar saw
// tick by tick, but not the edge-latched/settled values the brain is
// actually handed. These are tick records (not events) so they keep
// their five sensor columns in every downstream tool.
#define TEL_TICK_JCT_SNAP    6  // 'J': the edge-latched `before` at JCT_DETECT:
                                //   a = latched s0, b = latched s4,
                                //   left/right = last follow command (still rolling)
#define TEL_TICK_CENTER_SNAP 7  // 'A': the at_center money read at CREEP_END:
                                //   a = b = left = right = 0 (robot stopped)

// ---- event records (a,b noted per event)
#define EV_RUN_START    16 // a = run_mode_t, b = base speed
#define EV_FOLLOW_START 17 // a = base speed
#define EV_BLIND_END    18 // a = encoder counts travelled while blind
#define EV_JCT_DETECT   19 // a = latched s0, b = latched s4 (the trigger)
#define EV_DEADEND      20 // a = recent max |p| (small = honest dead end)
#define EV_LOSS         21 // a = recent max |p|, b = ms center was lost
#define EV_BT_START     22 // a = history entries available to retrace
#define EV_BT_FOUND     23 // a = ms spent, b = entries consumed
#define EV_BT_FAIL      24 // a = ms spent, b = history entries consumed.
                           //   Three causes: history ran out; timed out;
                           //   or the refound line didn't survive the
                           //   settle re-check — that one trails its own
                           //   BT_FOUND (a find demoted, not a decode
                           //   error; see bt_found_finish in drive.c)
#define EV_CREEP_START  25
#define EV_CREEP_END    26
#define EV_TURN_START   27 // a = target delta deg*10
#define EV_TURN_END     28 // a = residual err deg*10, b = ms taken
#define EV_CLASSIFY     29 // a = junction_t value
#define EV_ABORT        30 // a = tick phase the abort happened in
#define EV_TIMEOUT      31 // a = tick phase
#define EV_FAULT        32
#define EV_RESUME       33 // recovery over, follower back in control:
                           //   a = the segment's recovery count at this resume
                           //   b = suppressed span ahead of the resume, mm:
                           //   how far junction detection stays holstered
                           //   while the bar re-covers ground this segment
                           //   already cleared (drive.c's resume protocol).
                           //   0 = the refind landed at/past the suppression
                           //   bound — the LATER of the arm point and the
                           //   departure point, max(arm_enc, ontape_enc) —
                           //   so nothing was suppressed. (Lose the line
                           //   behind the arm point and that bound IS the
                           //   arm point.) The span leaves no other
                           //   trace in the dump: suppressed detect looks
                           //   exactly like plain corridor.
// NUMBERING IS APPEND-ONLY. Every CSV ever dumped encodes these values;
// renumbering an existing kind silently re-labels history. New kinds go
// at the end, and the dump tables + range checks in telemetry.c (and the
// decoder tables in plot_telemetry.py) must grow in the same commit.

void telemetry_init(void);   // once at boot
void telemetry_reset(void);  // at each run start: empty the ring

// One control tick. line = calibrated 0..1000 (stored /4 — 4-count
// resolution is plenty for pictures). left/right = commanded motor duty.
void telemetry_tick(uint8_t phase, const uint16_t line[5],
                    int16_t a, int16_t b, int16_t left, int16_t right);

void telemetry_event(uint8_t ev, int16_t a, int16_t b);

uint32_t telemetry_count(void);  // records currently held (caps at ring size)

// Dump-time context, handed in by main.c just before telemetry_dump().
// The layering rule (DS-a) is why this is a hand-off and not a lookup:
// telemetry OBSERVES — it never reaches into hw_battery/hw_line itself.
// batt_mv = battery millivolts read at dump time (resting — motors have
// been stopped for the walk back to the laptop, so the loaded voltage
// during the run was LOWER than this number). cal_min/cal_max = the line
// calibration window as it stands AT DUMP TIME — the window that scaled
// this dump's sensor columns only if nothing recalibrated between the
// run and the dump (the ring is cleared at run start, the cal arrays
// are not).
// Optional: without a call, the dump prints its original header shape,
// so old callers and old CSVs stay valid (I3).
void telemetry_set_dump_context(uint16_t batt_mv,
                                const uint16_t cal_min[5],
                                const uint16_t cal_max[5]);

// Print the whole ring as CSV over USB stdio (see format spec in
// TELEMETRY.md). Takes seconds — only ever call it with motors stopped.
void telemetry_dump(void);

#endif
