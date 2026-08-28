// hw_line.c — parallel bit-banged RC-decay read of GP18–GP22.
//
// Each sensor is a phototransistor across a small capacitor. Drive the pin
// high to charge the capacitor, float the pin, and time how long the pin
// still reads high: reflected IR makes the phototransistor conduct and bleed
// the charge away, so a white surface decays fast and black tape decays
// slowly. The decay time is the measurement — there is no ADC involved.
//
// Why read all five in parallel: five sequential reads would cost up to
// 5 ms, and at replay speed the robot covers ~3 mm in that time, which
// smears the junction edges the reading exists to locate. Charging all five
// together and polling them in one loop costs the same as a single sensor —
// bounded at ~1.1 ms worst case — and yields five samples of the same
// instant of travel rather than five samples from five different places on
// the course.
//
// Pololu's own driver pushes this further and counts the decay in PIO
// hardware (third_party/pololu_3pi_2040_robot/qtr_sensor_counter.pio), which
// frees the CPU entirely. A tight CPU loop against the µs timer is
// sufficient at these speeds and is far easier to single-step.
//
// GP26 discipline: the line-emitter control pin IS the battery ADC pin
// (pins.h has the full trap). The rule this file keeps is that it turns the
// emitters on for the duration of a read and releases the pin afterwards,
// while hw_battery.c reconfigures the same pin for ADC on every battery
// read. Each module re-claims the pin every time and neither assumes the
// other left it alone, so the two are safe in any order — as long as they do
// not overlap.

#include "hw_line.h"
#include "hw_millis.h"
#include "tuning.h"
#include "pins.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"

// Full-scale decay time, and the cap on how long a read may block. 1024 µs
// is the same full scale Pololu's PIO implementation uses, so raw readings
// from the two are directly comparable.
#define LINE_TIMEOUT_US 1024
// How long the pins are driven high to charge the sensor capacitors before
// the decay is timed. Paid once for the whole bar, not once per sensor.
#define CHARGE_US       32

// GP18..GP22 are contiguous, so one shift of the GPIO input register puts
// them in bits 0..4 of the polled mask. GP18 (bit 0) is the RIGHTMOST
// sensor and GP22 (bit 4) the leftmost, while the output array runs left
// to right — hence sensor index = 4 - bit.
#define LINE_PIN_BASE PIN_LINE_5   // GP18
#define LINE_MASK     0x1Fu

uint16_t hw_line_cal_min[5];
uint16_t hw_line_cal_max[5];

void hw_line_init(void)
{
    hw_line_cal_reset();
}

// Switch the down-facing IR emitters. Driving GP26 high turns them on;
// leaving it an input turns them off and hands the pad back for battery ADC
// use. The pin is re-initialised on every call because hw_battery.c may have
// switched it to the ADC function since the last read.
static void emitters(bool on)
{
    if (on) {
        gpio_init(PIN_LINE_EMITTER);          // reclaim from ADC if needed
        // Set the output latch before enabling the driver, so the pad never
        // presents a low pulse to the emitters on the way up.
        gpio_put(PIN_LINE_EMITTER, 1);
        gpio_set_dir(PIN_LINE_EMITTER, GPIO_OUT);
    } else {
        gpio_init(PIN_LINE_EMITTER);          // back to input = emitters off
    }
}

void hw_line_read_raw(uint16_t out[5])
{
    emitters(true);

    // Charge phase: all five pins driven high together, one 32 µs wait for
    // the whole bar.
    for (int i = 0; i < 5; i++) {
        uint pin = LINE_PIN_BASE + i;
        gpio_init(pin);
        // A pull-up would keep feeding the capacitor and a pull-down would
        // drain it: either one biases the decay time this function measures.
        gpio_disable_pulls(pin);
        gpio_put(pin, 1);
        gpio_set_dir(pin, GPIO_OUT);
    }
    busy_wait_us_32(CHARGE_US);

    // Release phase: flip all five to inputs as close together as possible,
    // so they share one t0, then poll until each one falls.
    for (int i = 0; i < 5; i++) {
        gpio_set_dir(LINE_PIN_BASE + i, GPIO_IN);
    }

    // Default to full scale: a sensor that never falls keeps this value.
    for (int i = 0; i < 5; i++) { out[i] = LINE_TIMEOUT_US; }

    uint32_t t0 = hw_micros();
    uint32_t pending = LINE_MASK;             // bits still high
    while (pending) {
        uint32_t elapsed = hw_micros() - t0;
        if (elapsed >= LINE_TIMEOUT_US) { break; }   // stragglers stay at full scale

        uint32_t pins = (sio_hw->gpio_in >> LINE_PIN_BASE) & LINE_MASK;
        // Bits that were high last pass and are low now decayed during this
        // interval; they all get the same timestamp, which is the polling
        // loop's resolution.
        uint32_t fell = pending & ~pins;
        while (fell) {
            int bit = __builtin_ctz(fell);
            fell &= fell - 1;
            out[4 - bit] = (uint16_t)elapsed; // GP18 = bit 0 = rightmost
        }
        pending &= pins;
    }

    emitters(false);   // save power and hand GP26 back to the battery ADC
}

void hw_line_cal_reset(void)
{
    for (int i = 0; i < 5; i++) {
        // Start each window inverted (min above any possible reading, max
        // below) so the first update replaces both ends rather than
        // widening from a bogus seed.
        hw_line_cal_min[i] = LINE_TIMEOUT_US + 1;
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
        // Empty or inverted window: the sensor was never calibrated, so
        // there is no honest scale. Report white rather than divide by zero.
        if (hi <= lo) { out[i] = 0; continue; }
        if (raw[i] <= lo)      { out[i] = 0; }
        else if (raw[i] >= hi) { out[i] = 1000; }
        else {
            out[i] = (uint16_t)((uint32_t)(raw[i] - lo) * 1000u / (hi - lo));
        }
    }
}
