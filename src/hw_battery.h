// hw_battery.h — battery voltage via ADC0 (GP26, ÷11 divider).
#ifndef HW_BATTERY_H
#define HW_BATTERY_H

#include <stdint.h>
#include <stdbool.h>

// Pack voltage in millivolts, roughly 4800–5600 on a freshly charged NiMH
// set. One blocking conversion, tens of microseconds.
// Side effect on a shared pin: this switches GP26 to the ADC function and
// LEAVES it there — GP26 is also the line-sensor emitter control (pins.h).
// hw_line.c re-claims the pin on every sensor read, so calling the two in
// sequence from the main loop is fine; what must not happen is a battery
// read in the middle of a line read.
uint16_t hw_battery_mv(void);

// Threshold checks against tuning.h. Each takes its own fresh reading, with
// the same GP26 side effect as hw_battery_mv().
bool hw_battery_low(void);       // below BATT_WARN_MV: warn, keep running
bool hw_battery_critical(void);  // below BATT_REFUSE_MV: refuse to start a run

#endif
