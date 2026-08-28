// hw_millis.h — time, read straight from the TIMER peripheral.
// You built this on Day 4 — this is the same idea, hardened.
#ifndef HW_MILLIS_H
#define HW_MILLIS_H

#include <stdint.h>

// Microseconds since boot, low 32 bits. Wraps every ~71.6 minutes — always
// compare with subtraction ((uint32_t)(now - then)), never with '<'.
uint32_t hw_micros(void);

// Full 64-bit microseconds (never wraps in your lifetime).
uint64_t hw_micros64(void);

// Milliseconds since boot.
uint32_t hw_millis(void);

#endif
