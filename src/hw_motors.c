// hw_motors.c — motors: PWM slice 7 (GP14 right = chan A, GP15 left =
// chan B) + direction pins GP10 (right) / GP11 (left), HIGH = reverse.
//
// Wrap 5999 with 125 MHz / 1 → 20.8 kHz, just above hearing, same as
// Pololu's own driver and your Day 6 build.
//
// SAFETY: SPEED_HARD_CAP is enforced HERE, at the last gate before the
// hardware. Every clever speed calculation upstream — PID overshoot,
// integer bug, 11pm Day-21 bravado — hits this clamp. One choke point,
// no exceptions. That is what "always a stop path" looks like in code.

#include "hw_motors.h"
#include "tuning.h"
#include "pins.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define MOTOR_SLICE 7
#define PWM_WRAP    6000

void hw_motors_init(void)
{
    gpio_init(PIN_MOTOR_RIGHT_DIR);
    gpio_init(PIN_MOTOR_LEFT_DIR);
    gpio_set_dir(PIN_MOTOR_RIGHT_DIR, GPIO_OUT);
    gpio_set_dir(PIN_MOTOR_LEFT_DIR, GPIO_OUT);

    gpio_set_function(PIN_MOTOR_RIGHT_PWM, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_LEFT_PWM, GPIO_FUNC_PWM);

    pwm_set_clkdiv_int_frac(MOTOR_SLICE, 1, 0);      // full 125 MHz
    pwm_set_wrap(MOTOR_SLICE, PWM_WRAP - 1);         // → 20.8 kHz
    pwm_set_both_levels(MOTOR_SLICE, 0, 0);
    pwm_set_enabled(MOTOR_SLICE, true);
}

static int32_t clamp_cap(int32_t s)
{
    if (s >  SPEED_HARD_CAP) { return  SPEED_HARD_CAP; }
    if (s < -SPEED_HARD_CAP) { return -SPEED_HARD_CAP; }
    return s;
}

void hw_motors_set(int32_t left, int32_t right)
{
    left  = clamp_cap(left);
    right = clamp_cap(right);

    // Direction pins: HIGH = reverse (pins.h). Duty is always positive.
    gpio_put(PIN_MOTOR_LEFT_DIR,  left  < 0);
    gpio_put(PIN_MOTOR_RIGHT_DIR, right < 0);

    uint16_t l = (uint16_t)(left  < 0 ? -left  : left);
    uint16_t r = (uint16_t)(right < 0 ? -right : right);

    // chan A = GP14 (right), chan B = GP15 (left) — even/odd pin rule.
    pwm_set_both_levels(MOTOR_SLICE, r, l);
}

void hw_motors_stop(void)
{
    hw_motors_set(0, 0);
}
