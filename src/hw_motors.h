// hw_motors.h — H-bridge motor driver: PWM slice 7 + direction pins.
// You built this on Day 6 — this is the same idea, hardened.
#ifndef HW_MOTORS_H
#define HW_MOTORS_H

#include <stdint.h>

void hw_motors_init(void);

// speed range: -6000..+6000 conceptually, but everything is clamped to
// ±SPEED_HARD_CAP (tuning.h) inside. Positive = forward.
void hw_motors_set(int32_t left, int32_t right);

void hw_motors_stop(void);

#endif
