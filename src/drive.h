// drive.h — maneuvers: the layer that turns "follow until something
// interesting" into clean sensor snapshots for the brain, and turns the
// brain's answers back into motion.
//
// This is the HARDWARE side of the seam. It hands logic/ two clean
// snapshots and executes the moves; it never decides anything itself.
// Also by design: NO ui calls in this file (see ui.h rule 3), and every
// loop polls the buttons — a human can always stop the robot.
#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>
#include "maze_logic.h"     // sensor_snapshot_t — the shared seam type

typedef enum {
    DRIVE_OK,        // maneuver finished
    DRIVE_ABORT,     // a button was pressed — caller must bail out cleanly
    DRIVE_TIMEOUT,   // watchdog fired — robot is lost/stuck, caller faults
    DRIVE_LOST,      // drove off the line and back-tracking couldn't refind
                     // it — robot is stopped somewhere on the floor, caller
                     // faults (recovered losses are handled internally and
                     // never reach the caller)
} drive_status_t;

void drive_init(void);

// PID line-follow at `base` speed until a junction announces itself
// (outer sensor over dark tape, or the whole line disappears while the
// robot was driving straight — a dead end).
// On DRIVE_OK, *before holds the trigger-moment snapshot with the outer
// sensors edge-latched (see drive.c for why). Assume the motors are
// STILL RUNNING — call drive_stop(), arrival_brake(), or
// creep_to_center() promptly. (The one exception is not a licence to
// dawdle: on the first tick after a recovery resume the retrace has
// left the chassis stopped and this tick's follow command has not been
// issued yet, so a junction detected on exactly that tick returns
// DRIVE_OK with the robot stationary. Every caller-visible guarantee
// still holds — the snapshot is valid and the next call commands its
// own speed — but "motors running" is the contract to CODE against,
// not a fact to measure.)
// If the line disappears while the follower was CORRECTING hard, that's
// not a dead end, it's a swerve off the tape. Recovery, all invisible to
// the caller: stop, retrace the per-tick encoder history in reverse
// (slowly) until a STREAK of consecutive ticks confirms tape under a
// center sensor (one dark sample is an opinion, not a measurement);
// stop, let the chassis settle, and re-check — if the stop's coast
// carried the bar past the find, nudge FORWARD to the tape (the refound
// line lies ahead: the robot was reversing); then resume following from
// a FRESH encoder baseline. Each segment carries a recovery budget that
// clean travel earns back (tuning.h's RECOVERY_DECAY_MM story). Only
// when a recovery fails outright — or the budget is spent — does it
// return DRIVE_LOST.
drive_status_t drive_until_junction(sensor_snapshot_t *before,
                                    int32_t base, uint32_t timeout_ms);

// Day 21's slowdown window. drive_until_junction hands back a robot
// still moving at base speed, and creep_to_center measures its CREEP_MM
// from wherever it begins — at a hot base the chassis is still braking
// when that budget runs out, so the deciding at_center read lands past
// the junction (the ladder's V7 wall). This is the fix: called on the
// arrival itself, it commands SPEED_ARRIVAL and holds it while steering
// straight on encoders (the bar is over junction tape — line data there
// is geometry, not guidance) and latching per-sensor maxima into *sweep
// exactly as the creep does, so the classifier's evidence has no blind
// spot. The window is bounded by DISTANCE: it ends after
// ARRIVAL_BRAKE_MM of encoder-measured travel, because every millimetre
// it rolls moves the creep's start line — and the at_center read — that
// much further past the junction (the goal-patch arithmetic that sizes
// the bound lives in tuning.h). ARRIVAL_BRAKE_MS is the wall-time
// TIMEOUT on that bound: if the encoders stop counting, the clock
// closes the window instead, so the loop cannot hang. Either bound
// exits with DRIVE_OK — a stalled chassis is diagnosed by
// creep_to_center's own watchdog right behind, never here, so this
// function never needs DRIVE_TIMEOUT; and it never hunts the line, so
// it cannot get LOST. A no-op (immediate DRIVE_OK, motors untouched)
// when base <= SPEED_ARRIVAL. Returns ONLY DRIVE_OK or DRIVE_ABORT:
// the buttons are polled every tick (abort = robot stopped). On
// DRIVE_OK the motors are STILL RUNNING at (or under) SPEED_ARRIVAL:
// call creep_to_center promptly, same rule as drive_until_junction.
drive_status_t arrival_brake(sensor_snapshot_t *sweep, int32_t base);

// Roll straight CREEP_MM (encoder-counted) so the wheel axle reaches the
// junction center. While rolling, latch per-sensor maxima into *sweep
// (merging with what drive_until_junction saw), then stop and take the
// deciding *at_center reading.
drive_status_t creep_to_center(sensor_snapshot_t *sweep,
                               sensor_snapshot_t *at_center);

// In-place gyro turns (PD on integrated angle, gyro_turn.py constants).
// DRIVE_TIMEOUT means the turn never settled — treat as a fault.
drive_status_t turn_left_90(void);
drive_status_t turn_right_90(void);
drive_status_t turn_around_180(void);

void drive_stop(void);

#endif
