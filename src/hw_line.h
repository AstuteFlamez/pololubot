// hw_line.h — the 5 down-facing RC-decay line sensors, read in parallel.
// You built this on Day 12 — this is the same idea, hardened.
#ifndef HW_LINE_H
#define HW_LINE_H

#include <stdint.h>
#include <stdbool.h>

void hw_line_init(void);

// One blocking parallel read of all 5 sensors, raw decay time in µs,
// 0..1024 (1024 = never decayed = very dark). Takes ~0.1–1.1 ms depending
// on surface. out[0] = leftmost sensor (GP22) … out[4] = rightmost (GP18).
void hw_line_read_raw(uint16_t out[5]);

// Calibration: fold one raw read into the per-sensor min/max window.
// Call repeatedly while sweeping the bar across white paper AND black tape.
void hw_line_cal_reset(void);
void hw_line_cal_update(void);

// True if every sensor saw a span of at least CAL_MIN_SPAN between its
// white and black extremes — anything less means that sensor never really
// saw the tape (or the tape is glossy) and calibrated values would be noise.
bool hw_line_cal_ok(void);

// Calibrated read: 0 (white, at/below cal min) .. 1000 (black, at/above
// cal max), linearly scaled between. Same indexing as hw_line_read_raw.
void hw_line_read_calibrated(uint16_t out[5]);

// For the DIAG screen.
extern uint16_t hw_line_cal_min[5];
extern uint16_t hw_line_cal_max[5];

#endif
