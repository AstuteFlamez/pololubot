// lap_timer.h — the scoreboard. Best time lives in RAM (a power cycle
// wipes it — by design: every session starts with something to beat).
#ifndef LAP_TIMER_H
#define LAP_TIMER_H

#include <stdint.h>
#include <stdbool.h>

void     lap_start(void);
uint32_t lap_stop(void);        // returns this lap's ms; updates best
uint32_t lap_current_ms(void);  // running time while a lap is live
uint32_t lap_last_ms(void);     // most recent finished lap (0 = none yet)
uint32_t lap_best_ms(void);     // best this session (0 = none yet)
bool     lap_last_was_best(void);

#endif
