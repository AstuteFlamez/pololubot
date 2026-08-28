// hw_battery.c — VBAT/11 on ADC input 0.
//
// The battery feeds a resistor divider that lands VBAT/11 on GP26 — the
// SAME pin that switches the line-sensor emitters (schematic economy;
// pins.h tells the story). The dance: adc_gpio_init() here hands the pin
// to the ADC; hw_line.c's gpio_init() takes it back for the emitters.
// Both sides re-claim on every use, so order never matters.
//
// Math: 12-bit ADC, 3.3 V reference → mv = raw × 11 × 3300 / 4096.
// Why refuse below 4.0 V (tuning.h)? Below ~1.0 V/NiMH-cell the pack is
// empty and sagging hard under motor load: PWM level no longer means the
// speed the knob was set for, and brownout resets mid-run look exactly like
// software bugs. Don't debug ghosts — charge the batteries.

#include "hw_battery.h"
#include "tuning.h"
#include "pins.h"
#include "hardware/adc.h"

uint16_t hw_battery_mv(void)
{
    if (!(adc_hw->cs & ADC_CS_EN_BITS)) { adc_init(); }
    adc_gpio_init(PIN_LINE_EMITTER);         // GP26 → ADC function
    adc_select_input(ADC_BATTERY);
    uint32_t raw = adc_read();
    return (uint16_t)(raw * (11 * 3300) / 4096);
}

bool hw_battery_low(void)      { return hw_battery_mv() < BATT_WARN_MV; }
bool hw_battery_critical(void) { return hw_battery_mv() < BATT_REFUSE_MV; }
