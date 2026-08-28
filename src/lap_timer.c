// lap_timer.c — stopwatch over hw_millis. Nothing clever; the drama it
// creates on Day 21 is the point.

#include "lap_timer.h"
#include "hw_millis.h"

static uint32_t start_ms;
static uint32_t last_ms_v;
static uint32_t best_ms_v;
static bool running;
static bool last_was_best;

void lap_start(void)
{
    start_ms = hw_millis();
    running = true;
}

uint32_t lap_stop(void)
{
    if (!running) { return 0; }
    running = false;
    last_ms_v = hw_millis() - start_ms;
    last_was_best = (best_ms_v == 0 || last_ms_v < best_ms_v);
    if (last_was_best) { best_ms_v = last_ms_v; }
    return last_ms_v;
}

uint32_t lap_current_ms(void) { return running ? hw_millis() - start_ms : 0; }
uint32_t lap_last_ms(void)    { return last_ms_v; }
uint32_t lap_best_ms(void)    { return best_ms_v; }
bool     lap_last_was_best(void) { return last_was_best; }
