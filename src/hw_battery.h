// hw_battery.h — battery voltage via ADC0 (GP26, ÷11 divider).
// You built this on Day 11 — this is the same idea, hardened.
#ifndef HW_BATTERY_H
#define HW_BATTERY_H

#include <stdint.h>
#include <stdbool.h>

// Battery voltage in millivolts (~4800–5600 on fresh NiMH).
// NOTE: reconfigures GP26 for ADC — never call mid line-sensor read
// (hw_line reclaims the pin on every read, so ordinary sequential use
// from the main loop is fine).
uint16_t hw_battery_mv(void);

// Convenience checks against tuning.h thresholds.
bool hw_battery_low(void);       // < BATT_WARN_MV  → nag
bool hw_battery_critical(void);  // < BATT_REFUSE_MV → refuse to run

#endif
