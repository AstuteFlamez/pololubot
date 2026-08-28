// hw_line.c — parallel bit-banged RC-decay read of GP18–GP22.
//
// How an RC-decay sensor works (Day 12): each sensor is a phototransistor
// on a small capacitor. Drive the pin HIGH to charge it, float the pin,
// and time how long the pin stays high. Lots of reflected IR (white paper)
// bleeds the charge fast → short decay. Black tape reflects little →
// long decay. TIME IS THE MEASUREMENT.
//
// Why parallel? Five sequential 1 ms reads = 5 ms per sample — a robot at
// replay speed moves ~3 mm in that time and junction edges smear. Charging
// all five together and polling all five in one loop costs the same as
// ONE sensor: bounded at ~1.1 ms worst case. (Pololu goes one step further
// and does this in PIO hardware — qtr_sensor_counter.pio, worth reading
// on Day 22 — but a tight CPU loop with the µs timer is plenty at our
// speeds, and it's code you can single-step.)
//
// GP26 discipline: the line emitter control pin IS the battery ADC pin
// (pins.h — read the why there). Rule: this file turns emitters on for
// the duration of a read and releases the pin after; hw_battery.c
// reconfigures it for ADC on every battery read. Each module re-claims
// the pin every time and never assumes the other left it alone.

#include "hw_line.h"
#include "hw_millis.h"
#include "tuning.h"
#include "pins.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"

#define LINE_TIMEOUT_US 1024   // same full-scale as Pololu's PIO version
#define CHARGE_US       32

// GP18..GP22 → bit 0..4 of the polled mask. Sensor index 0 (leftmost)
// is GP22 = bit 4, so sensor index = 4 - bit.
#define LINE_PIN_BASE PIN_LINE_5   // GP18
#define LINE_MASK     0x1Fu

uint16_t hw_line_cal_min[5];
uint16_t hw_line_cal_max[5];

void hw_line_init(void)
{
    hw_line_cal_reset();
}

static void emitters(bool on)
{
    if (on) {
        gpio_init(PIN_LINE_EMITTER);          // reclaim from ADC if needed
        gpio_put(PIN_LINE_EMITTER, 1);
        gpio_set_dir(PIN_LINE_EMITTER, GPIO_OUT);
    } else {
        gpio_init(PIN_LINE_EMITTER);          // input = emitters off
    }
}

void hw_line_read_raw(uint16_t out[5])
{
    emitters(true);

    // Charge phase: all five pins driven high together.
    for (int i = 0; i < 5; i++) {
        uint pin = LINE_PIN_BASE + i;
        gpio_init(pin);
        gpio_disable_pulls(pin);              // pulls would skew the decay
        gpio_put(pin, 1);
        gpio_set_dir(pin, GPIO_OUT);
    }
    busy_wait_us_32(CHARGE_US);

    // Release phase: flip all five to inputs, then poll with timestamps.
    for (int i = 0; i < 5; i++) {
        gpio_set_dir(LINE_PIN_BASE + i, GPIO_IN);
    }

    for (int i = 0; i < 5; i++) { out[i] = LINE_TIMEOUT_US; }

    uint32_t t0 = hw_micros();
    uint32_t pending = LINE_MASK;             // bits still high
    while (pending) {
        uint32_t elapsed = hw_micros() - t0;
        if (elapsed >= LINE_TIMEOUT_US) { break; }   // rest score full-dark

        uint32_t pins = (sio_hw->gpio_in >> LINE_PIN_BASE) & LINE_MASK;
        uint32_t fell = pending & ~pins;      // just went low → decayed NOW
        while (fell) {
            int bit = __builtin_ctz(fell);
            fell &= fell - 1;
            out[4 - bit] = (uint16_t)elapsed; // GP18=bit0 is the RIGHTMOST
        }
        pending &= pins;
    }

    emitters(false);   // save power, free GP26 for the battery ADC
}

void hw_line_cal_reset(void)
{
    for (int i = 0; i < 5; i++) {
        hw_line_cal_min[i] = LINE_TIMEOUT_US + 1;   // impossible-high
        hw_line_cal_max[i] = 0;
    }
}

void hw_line_cal_update(void)
{
    uint16_t raw[5];
    hw_line_read_raw(raw);
    for (int i = 0; i < 5; i++) {
        if (raw[i] < hw_line_cal_min[i]) { hw_line_cal_min[i] = raw[i]; }
        if (raw[i] > hw_line_cal_max[i]) { hw_line_cal_max[i] = raw[i]; }
    }
}

bool hw_line_cal_ok(void)
{
    for (int i = 0; i < 5; i++) {
        if (hw_line_cal_max[i] < hw_line_cal_min[i] + CAL_MIN_SPAN) {
            return false;
        }
    }
    return true;
}

void hw_line_read_calibrated(uint16_t out[5])
{
    uint16_t raw[5];
    hw_line_read_raw(raw);
    for (int i = 0; i < 5; i++) {
        uint16_t lo = hw_line_cal_min[i], hi = hw_line_cal_max[i];
        if (hi <= lo) { out[i] = 0; continue; }        // uncalibrated sensor
        if (raw[i] <= lo)      { out[i] = 0; }
        else if (raw[i] >= hi) { out[i] = 1000; }
        else {
            out[i] = (uint16_t)((uint32_t)(raw[i] - lo) * 1000u / (hi - lo));
        }
    }
}
