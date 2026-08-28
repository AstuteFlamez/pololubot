// hw_millis.c — the 1 MHz TIMER peripheral, read directly.
//
// The RP2040 has one 64-bit microsecond counter (datasheet §4.6). Reading
// 64 bits on a 32-bit CPU takes two loads, and the counter can carry
// between them — TIMERAWH/TIMERAWL must be read with the classic
// "read-high, read-low, re-read-high" loop. You met this trap on Day 4.

#include "hw_millis.h"
#include "hardware/timer.h"
#include "hardware/structs/timer.h"

uint32_t hw_micros(void)
{
    return timer_hw->timerawl;   // single atomic 32-bit read
}

uint64_t hw_micros64(void)
{
    uint32_t hi, lo;
    do {
        hi = timer_hw->timerawh;
        lo = timer_hw->timerawl;
    } while (timer_hw->timerawh != hi);   // carry slipped in? try again
    return ((uint64_t)hi << 32) | lo;
}

uint32_t hw_millis(void)
{
    return (uint32_t)(hw_micros64() / 1000);
}
