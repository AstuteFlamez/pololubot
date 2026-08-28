// hw_battery.c — VBAT/11 on ADC input 0.
//
// The pack feeds a resistor divider that lands VBAT/11 on GP26 — the same
// pin that switches the line-sensor emitters (pins.h documents the trap in
// full). The handover is symmetric and stateless: adc_gpio_init() here
// hands the pad to the ADC, hw_line.c's gpio_init() takes it back for the
// emitters, and both sides re-claim on every use, so call order never
// matters as long as the two do not interleave.
//
// Conversion: 12-bit ADC against the 3.3 V reference, divider ratio 11, so
// mv = raw × 11 × 3300 / 4096. The arithmetic is done in uint32_t; the
// largest intermediate (4095 × 36300) is comfortably inside 32 bits.
//
// The refusal threshold matters more than the warning one. Below roughly
// 1.0 V per NiMH cell the pack is empty and sags hard under motor load: a
// PWM level no longer delivers the speed the tuning assumed, and a brownout
// reset mid-run is indistinguishable from a firmware fault. Refusing to
// start is cheaper than debugging that.

#include "hw_battery.h"
#include "tuning.h"
#include "pins.h"
#include "hardware/adc.h"

uint16_t hw_battery_mv(void)
{
    // Idempotent bring-up: the ADC may not have been touched yet, and
    // adc_init() resets the block, so only run it when it is not enabled.
    if (!(adc_hw->cs & ADC_CS_EN_BITS)) { adc_init(); }
    adc_gpio_init(PIN_LINE_EMITTER);         // re-claim GP26 from hw_line.c
    adc_select_input(ADC_BATTERY);
    uint32_t raw = adc_read();
    return (uint16_t)(raw * (11 * 3300) / 4096);
}

bool hw_battery_low(void)      { return hw_battery_mv() < BATT_WARN_MV; }
bool hw_battery_critical(void) { return hw_battery_mv() < BATT_REFUSE_MV; }
