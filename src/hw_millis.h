// hw_millis.h — time, read straight from the RP2040 TIMER peripheral.
#ifndef HW_MILLIS_H
#define HW_MILLIS_H

#include <stdint.h>

// Microseconds since boot, low 32 bits of the free-running timer.
// Wraps back to 0 every 2^32 µs (~71.6 minutes), so elapsed time must
// always be computed as unsigned subtraction — (uint32_t)(now - then) is
// correct across the wrap, (now < then) is not. Single 32-bit register
// read; safe from an interrupt handler.
uint32_t hw_micros(void);

// Microseconds since boot, all 64 bits. Does not wrap in any practical
// runtime. Costs two or three register reads (see the retry loop in the .c).
uint64_t hw_micros64(void);

// Milliseconds since boot. Derived from hw_micros64(), so it survives the
// 32-bit microsecond wrap, but it truncates to 32 bits and therefore wraps
// after ~49.7 days; compare with unsigned subtraction as above. Costs a
// 64-bit division — prefer hw_micros() in tight loops.
uint32_t hw_millis(void);

#endif
