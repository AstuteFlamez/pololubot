// lap_timer.h — stopwatch for one replay run, plus the best time of the
// session.
//
// All times are milliseconds, measured with hw_millis(). The state lives
// in RAM only, so a power cycle wipes the best time — by design: every
// session starts with something to beat. One lap at a time: main.c starts
// the clock when a replay's wheels are about to move and stops it on the
// goal arrival, so a faulted run leaves no finish time behind.
#ifndef LAP_TIMER_H
#define LAP_TIMER_H

#include <stdint.h>
#include <stdbool.h>

// Start the clock. A lap already in progress is discarded.
void     lap_start(void);

// Stop the clock and latch the result: returns this lap's duration in ms,
// updates the session best, and sets the was-best flag. Returns 0 and
// changes nothing if no lap is running.
uint32_t lap_stop(void);

// Elapsed ms of the lap in progress; 0 when no lap is running.
uint32_t lap_current_ms(void);

// Most recent finished lap, in ms (0 = none finished this session).
uint32_t lap_last_ms(void);

// Fastest finished lap of this session, in ms (0 = none yet).
uint32_t lap_best_ms(void);

// True if the most recent finished lap was a new session best. Latched by
// lap_stop(); the DONE screen only reads it.
bool     lap_last_was_best(void);

#endif
