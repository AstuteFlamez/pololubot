// hw_millis.c — the 1 MHz TIMER peripheral, read directly.
//
// The RP2040 has one free-running 64-bit microsecond counter (datasheet
// §4.6). Reading it takes two 32-bit loads on a 32-bit CPU, and the counter
// can carry from the low word into the high word between them — a naive
// read can return a value an hour off. TIMERAWH/TIMERAWL are therefore read
// with the classic read-high, read-low, re-read-high loop.
//
// TIMERAWH/TIMERAWL are the raw aliases: unlike TIMEHR/TIMELR they have no
// latching side effect, so reading them from an interrupt cannot corrupt a
// read in progress in main.

#include "hw_millis.h"
#include "hardware/timer.h"
#include "hardware/structs/timer.h"

uint32_t hw_micros(void)
{
    return timer_hw->timerawl;   // one load, no tearing possible
}

uint64_t hw_micros64(void)
{
    uint32_t hi, lo;
    // Retry until the high word is unchanged across the low-word read; if it
    // moved, a carry landed between the two loads and the pair is garbage.
    do {
        hi = timer_hw->timerawh;
        lo = timer_hw->timerawl;
    } while (timer_hw->timerawh != hi);
    return ((uint64_t)hi << 32) | lo;
}

uint32_t hw_millis(void)
{
    return (uint32_t)(hw_micros64() / 1000);
}
