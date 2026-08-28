// hw_line.h — the 5 down-facing RC-decay line sensors, read in parallel.
#ifndef HW_LINE_H
#define HW_LINE_H

#include <stdint.h>
#include <stdbool.h>

// Clear the calibration window. No pins are touched — hw_line_read_raw()
// configures everything it needs on each call.
void hw_line_init(void);

// One blocking parallel read of all five sensors. out[i] is the raw decay
// time in µs, 0..1024, where 1024 means the sensor never decayed inside the
// timeout (very dark). Indexing runs left to right across the robot:
// out[0] = leftmost sensor (GP22) … out[4] = rightmost (GP18).
// Blocks for the length of the slowest sensor, ~0.1 ms over white paper up
// to the 1.024 ms timeout over black tape.
// Side effects on shared pins: switches the GP26 IR emitters on for the
// duration and releases GP26 back to a plain input on the way out (see
// pins.h — GP26 is also the battery ADC pin), and leaves GP18–GP22 as
// inputs with their pulls disabled.
void hw_line_read_raw(uint16_t out[5]);

// Reset the per-sensor min/max calibration window to an empty state.
void hw_line_cal_reset(void);

// Take one raw read and widen each sensor's min/max window to include it.
// Same blocking cost and pin side effects as hw_line_read_raw(). Call it
// repeatedly while the sensor bar sweeps across BOTH white surface and black
// tape — a window that has only seen one of the two is not a calibration.
void hw_line_cal_update(void);

// True when every sensor's window spans at least CAL_MIN_SPAN µs between its
// white and black extremes. A narrower span means that sensor never really
// saw the tape (or the tape is glossy enough to reflect), and scaling
// against it would amplify noise into full-range readings.
bool hw_line_cal_ok(void);

// Blocking read scaled through the calibration window: 0 = at or below the
// sensor's white extreme, 1000 = at or above its black extreme, linear in
// between. A sensor whose window is empty or inverted reads 0. Same indexing,
// blocking cost and pin side effects as hw_line_read_raw().
void hw_line_read_calibrated(uint16_t out[5]);

// The live calibration window, indexed like the read functions, in raw µs.
// Exposed for the DIAG screen and the telemetry dump; treat as read-only.
extern uint16_t hw_line_cal_min[5];
extern uint16_t hw_line_cal_max[5];

#endif
