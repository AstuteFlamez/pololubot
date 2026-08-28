// hw_motors.h — H-bridge motor driver: PWM slice 7 + two direction pins.
#ifndef HW_MOTORS_H
#define HW_MOTORS_H

#include <stdint.h>

// Claim GP10/GP11 as direction outputs and GP14/GP15 as PWM slice 7, set
// the slice to a 6000-count wrap at full system clock (20.8 kHz), and enable
// it with both channels at zero duty. Must run before hw_motors_set(); the
// motors are left stopped.
void hw_motors_init(void);

// Command both motors. Speeds are raw PWM counts against the 6000-count
// wrap, so the representable range is -6000..+6000, but every command is
// clamped here to ±SPEED_HARD_CAP (tuning.h) before it reaches the hardware
// — this is the single choke point where that ceiling is enforced.
// Positive drives the wheel forward, negative reverse. Returns immediately;
// the PWM keeps running at the commanded duty until the next call.
void hw_motors_set(int32_t left, int32_t right);

// Command zero duty on both motors — identical to hw_motors_set(0, 0).
// These are PHASE/ENABLE drivers, so zero duty ties the motor terminals
// together: the wheels brake, they do not coast.
void hw_motors_stop(void);

#endif
