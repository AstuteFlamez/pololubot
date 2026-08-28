// hw_buttons.c — reading buttons that don't own their pins.
//
// Button A shares GP25 with the yellow LED; button C shares GP0 with the
// OLED's D/C line (pins.h: "when a conflict finally bites you, open the
// schematic"). The trick — same one Pololu's driver uses — is the pad's
// OUTPUT-ENABLE OVERRIDE: force the output driver off for a microsecond,
// let the pin float, read the level, restore. The display never notices
// because we only poll between display transfers, never during.
//
// Press = LOW on both (pull-up topology).

#include "hw_buttons.h"
#include "hw_millis.h"
#include "pins.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define DEBOUNCE_MS   20
#define LONGPRESS_MS  600

static bool raw_a(void)
{
    gpio_set_oeover(PIN_LED, GPIO_OVERRIDE_LOW);   // stop driving GP25
    busy_wait_us_32(1);
    bool pressed = !gpio_get(PIN_LED);
    gpio_set_oeover(PIN_LED, GPIO_OVERRIDE_NORMAL);
    return pressed;
}

static bool raw_c(void)
{
    gpio_set_oeover(PIN_BUTTON_C, GPIO_OVERRIDE_LOW);  // stop driving GP0
    gpio_pull_up(PIN_BUTTON_C);
    busy_wait_us_32(1);
    bool pressed = !gpio_get(PIN_BUTTON_C);
    gpio_set_oeover(PIN_BUTTON_C, GPIO_OVERRIDE_NORMAL);
    return pressed;
}

void hw_buttons_init(void)
{
    // Make sure both pads are inputs to the SIO input path (gpio_get needs
    // the input buffer enabled). Function stays whatever the display set.
    gpio_set_input_enabled(PIN_LED, true);
    gpio_set_input_enabled(PIN_BUTTON_C, true);
}

// --- debounced event state (C gets short/long, A gets press) ------------
static bool c_down, a_down;
static uint32_t c_edge_ms, a_edge_ms;
static bool c_long_fired;

button_event_t hw_buttons_poll(void)
{
    uint32_t now = hw_millis();

    // ---- button C: short vs long ----
    bool c = raw_c();
    if (c != c_down && now - c_edge_ms >= DEBOUNCE_MS) {
        c_down = c;
        c_edge_ms = now;
        if (c) {
            c_long_fired = false;              // press begins
        } else if (!c_long_fired) {
            return BTN_C_SHORT;                // released before long fired
        }
    }
    if (c_down && !c_long_fired && now - c_edge_ms >= LONGPRESS_MS) {
        c_long_fired = true;                   // fire once, while held
        return BTN_C_LONG;
    }

    // ---- button A: press edge ----
    bool a = raw_a();
    if (a != a_down && now - a_edge_ms >= DEBOUNCE_MS) {
        a_down = a;
        a_edge_ms = now;
        if (a) { return BTN_A_PRESS; }
    }

    return BTN_NONE;
}

bool hw_buttons_any(void)
{
    return raw_a() || raw_c();
}

void hw_buttons_wait_release(void)
{
    while (hw_buttons_any()) { sleep_ms(10); }
    sleep_ms(50);          // let the contacts finish rattling
    c_down = a_down = false;
    c_long_fired = false;
}
