// drive.h — maneuvers: the layer that turns "follow the line until
// something interesting happens" into clean sensor snapshots for the
// maze logic, and turns the logic's answers back into motion.
//
// This is the hardware side of the seam. It hands logic/ two snapshots
// per junction and executes the moves; it decides nothing itself.
// Two standing rules in the implementation: no ui calls anywhere in
// drive.c (an OLED flush is a multi-millisecond SPI transfer, and a
// control loop cannot afford that hole in its timing), and every loop
// polls the buttons, so a human can always stop the robot.
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

// Claim the motor, encoder, and line-sensor hardware. Call once, before
// any other function here. Leaves the motors stopped. The IMU is NOT
// initialized here — only the turns need it, so hw_imu_init() stays the
// caller's call and its success/failure stays the caller's to report.
void drive_init(void);

// PD line-follow at `base` speed until a junction announces itself: an
// outer sensor over dark tape, or the whole line disappearing while the
// robot was driving straight (a dead end).
//
// base       raw PWM counts against the 6000-count wrap, forward only
//            (SPEED_EXPLORE / SPEED_REPLAY sized). Neither wheel is ever
//            commanded above it, so the robot cannot outrun the speed
//            asked for while following.
// timeout_ms wall-clock watchdog for the whole segment. Time spent
//            inside an internal recovery does not count against it.
// before     filled on DRIVE_OK only: the trigger-moment snapshot, with
//            the two outer sensors edge-latched (a branch can flash past
//            in one sample at speed; drive.c has the latch reasoning).
//            Untouched on every other status.
//
// Blocking. On DRIVE_OK assume the motors are STILL RUNNING — call
// drive_stop(), arrival_brake(), or creep_to_center() promptly. (The one
// exception is not a licence to dawdle: on the first tick after a
// recovery resume the retrace has left the chassis stopped and that
// tick's follow command has not been issued yet, so a junction detected
// on exactly that tick returns DRIVE_OK with the robot stationary. Every
// caller-visible guarantee still holds — the snapshot is valid and the
// next call commands its own speed — but "motors running" is the
// contract to code against, not a fact to measure.) DRIVE_ABORT,
// DRIVE_TIMEOUT, and DRIVE_LOST all leave the robot stopped.
//
// If the line disappears while the follower was CORRECTING hard, that is
// not a dead end, it is a swerve off the tape. Recovery is internal and
// invisible to the caller: stop, retrace the per-tick encoder history in
// reverse (slowly) until a STREAK of consecutive ticks confirms tape
// under a center sensor (one dark sample is an opinion, not a
// measurement); stop, let the chassis settle, and re-check — if the
// stop's coast carried the bar past the find, nudge FORWARD onto the
// tape (the refound line lies ahead, because the robot was reversing);
// then resume following from a fresh encoder baseline. Each segment
// carries a recovery budget that clean travel earns back (see
// RECOVERIES_MAX and RECOVERY_DECAY_MM in include/tuning.h). Only when a
// recovery fails outright — or the budget is spent — does DRIVE_LOST
// reach the caller.
drive_status_t drive_until_junction(sensor_snapshot_t *before,
                                    int32_t base, uint32_t timeout_ms);

// Shed excess speed between the junction detect and the creep, so the
// creep starts from the pace it was tuned for.
//
// drive_until_junction hands back a robot still moving at `base`, and
// creep_to_center measures its CREEP_MM from wherever it begins. Out of
// a hot base the chassis is still braking when that budget runs out, so
// the deciding at_center read lands past the junction. Called on the
// arrival itself, this commands SPEED_ARRIVAL and holds it while
// steering straight on encoders (the bar is over junction tape — line
// data there is geometry, not guidance) and latching per-sensor maxima
// into *sweep exactly as the creep does, so the classifier's evidence
// has no blind spot.
//
// sweep  in/out: the snapshot drive_until_junction filled. Per-sensor
//        maxima seen during the window are merged into it; nothing is
//        cleared. Callers pass the same struct on to creep_to_center.
// base   the speed the follow leg was running at. A no-op — immediate
//        DRIVE_OK, motors untouched — when base <= SPEED_ARRIVAL.
//
// The window is bounded by DISTANCE: it ends after ARRIVAL_BRAKE_MM of
// encoder-measured travel, because every millimetre rolled here moves
// the creep's start line — and with it the at_center read — that much
// further past the junction (the goal-patch arithmetic that sizes the
// bound is written up on ARRIVAL_BRAKE_MM in include/tuning.h).
// ARRIVAL_BRAKE_MS is the wall-time TIMEOUT on that bound: if the
// encoders stop counting, the clock closes the window instead, so the
// loop cannot hang. Either bound exits DRIVE_OK — a stalled chassis is
// diagnosed by creep_to_center's own watchdog right behind, never here,
// which is why this function never needs DRIVE_TIMEOUT; and it never
// hunts the line, so it cannot get LOST.
//
// Blocking. Returns ONLY DRIVE_OK or DRIVE_ABORT (buttons are polled
// every tick; abort leaves the robot stopped). On DRIVE_OK the motors
// are STILL RUNNING at roughly SPEED_ARRIVAL — call creep_to_center
// promptly, same rule as drive_until_junction.
drive_status_t arrival_brake(sensor_snapshot_t *sweep, int32_t base);

// Roll straight CREEP_MM (encoder-counted) so the wheel axle reaches the
// junction center, then stop, let the chassis settle, and take the
// deciding read.
//
// sweep      in/out: the snapshot from the detect (and, at a stepped
//            base, from arrival_brake). Per-sensor maxima seen while the
//            bar crosses the junction are merged into it.
// at_center  out: the settled reading taken with the bar past the
//            junction line. Written on DRIVE_OK only.
//
// Blocking; entry assumes the motors are running from the detect.
// Returns DRIVE_OK with the robot STOPPED and settled on the junction
// center — the state the turns expect. DRIVE_TIMEOUT (CREEP_TIMEOUT_MS,
// robot stopped) means the distance never arrived: a stalled wheel or an
// inverted encoder sign. DRIVE_ABORT also leaves the robot stopped.
drive_status_t creep_to_center(sensor_snapshot_t *sweep,
                               sensor_snapshot_t *at_center);

// In-place gyro turns: PD on the integrated gyro angle, relative to
// wherever the robot is pointing when the call is made (+90 left, -90
// right, +180 about-face). Blocking; entry assumes the robot is stopped
// on a junction center, and every exit leaves it stopped.
//
// DRIVE_OK means the body angle held within TURN_TOL_DEG for
// TURN_SETTLE_MS. DRIVE_TIMEOUT means the turn never settled — treat it
// as a fault, not something to retry. DRIVE_ABORT on a button.
// hw_imu_init() must have succeeded; these read no line data.
drive_status_t turn_left_90(void);
drive_status_t turn_right_90(void);
drive_status_t turn_around_180(void);

// Zero duty on both motors; returns immediately. Requires drive_init()
// to have run. Safe to call from any state after that, mid-maneuver
// included — it is the fault paths' first move.
void drive_stop(void);

#endif
