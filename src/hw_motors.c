// hw_motors.c — motors: PWM slice 7 (GP14 right = channel A, GP15 left =
// channel B) plus direction pins GP10 (right) and GP11 (left), HIGH =
// reverse.
//
// Slice arithmetic: GP14 and GP15 land on the same slice because the RP2040
// maps GPIO n to slice (n/2) % 8 and channel A/B by even/odd — 14 and 15 are
// an even/odd pair, so one slice drives both wheels and both channels share
// the wrap and clock divider. Clock divider 1 keeps the full 125 MHz system
// clock; a wrap of 5999 gives 125e6 / 6000 = 20.8 kHz, above the audible
// range so the motors do not whine, and matching Pololu's own driver.
//
// Safety: SPEED_HARD_CAP is enforced here, at the last gate before the
// hardware. Every speed calculation upstream — PID overshoot, an integer
// overflow, a mistyped constant — funnels through this clamp. One choke
// point, no exceptions, so there is always a bounded stop path.
//
// The drivers take PHASE (the direction pin) and ENABLE (the PWM pin), so
// this code never drives the two legs of a half-bridge itself and there is
// no shoot-through to interlock against. It does mean zero duty is a BRAKE,
// not a coast: during the off-fraction of every PWM cycle the driver ties
// the motor terminals together, so hw_motors_stop() holds the wheels rather
// than letting them roll.
//
// Direction and duty are two separate writes, so a reversal applies the old
// duty in the new direction for the few instructions between them. At
// 20.8 kHz that is a fraction of one PWM period and the gearbox never
// sees it.

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

    pwm_set_clkdiv_int_frac(MOTOR_SLICE, 1, 0);      // no division: 125 MHz
    pwm_set_wrap(MOTOR_SLICE, PWM_WRAP - 1);         // 6000 counts → 20.8 kHz
    pwm_set_both_levels(MOTOR_SLICE, 0, 0);
    pwm_set_enabled(MOTOR_SLICE, true);
}

// Saturate a speed command to ±SPEED_HARD_CAP.
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

    // Sign lives on the direction pin; the PWM level is always a magnitude.
    gpio_put(PIN_MOTOR_LEFT_DIR,  left  < 0);
    gpio_put(PIN_MOTOR_RIGHT_DIR, right < 0);

    uint16_t l = (uint16_t)(left  < 0 ? -left  : left);
    uint16_t r = (uint16_t)(right < 0 ? -right : right);

    // Channel A is GP14 (right), channel B is GP15 (left) — the even/odd
    // pin rule, so the argument order here is (right, left).
    pwm_set_both_levels(MOTOR_SLICE, r, l);
}

void hw_motors_stop(void)
{
    hw_motors_set(0, 0);
}
