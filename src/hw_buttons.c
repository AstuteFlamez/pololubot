// hw_buttons.c — reading buttons that do not own their pins.
//
// Button A shares GP25 with the yellow LED; button C shares GP0 with the
// OLED's D/C line (pins.h). A pad that is being driven cannot be read, so
// both are sampled through the pad's OUTPUT-ENABLE OVERRIDE: force the
// output driver off for a microsecond, let the pin float to whatever the
// button and its pull-up say, read the level, restore the override. This is
// the same technique Pololu's own driver uses. The display never notices
// because polling only happens between display transfers, never inside one.
//
// Both buttons read LOW when pressed — they short a pulled-up line to ground.
//
// The debounce is a per-button state machine rather than a delay: an edge is
// only accepted once DEBOUNCE_MS has passed since the last accepted edge on
// that button, which rejects contact bounce without ever blocking the
// caller. Button C additionally splits into short and long: the long event
// fires while the button is still held, once, and suppresses the short event
// that would otherwise follow on release.

#include "hw_buttons.h"
#include "hw_millis.h"
#include "pins.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define DEBOUNCE_MS   20    // longer than switch bounce, shorter than a tap
#define LONGPRESS_MS  600   // deliberate hold, not a slow tap

// Sample button A on GP25 without disturbing the yellow LED that shares it.
static bool raw_a(void)
{
    gpio_set_oeover(PIN_LED, GPIO_OVERRIDE_LOW);   // stop driving GP25
    busy_wait_us_32(1);                            // let the pad settle
    bool pressed = !gpio_get(PIN_LED);
    gpio_set_oeover(PIN_LED, GPIO_OVERRIDE_NORMAL);
    return pressed;
}

// Sample button C on GP0 without disturbing the OLED D/C line that shares
// it. Unlike GP25, this pad needs the internal pull-up enabled to give the
// floated pin a defined high level for the button to pull away from. The
// pull-up is left enabled afterwards, matching Pololu's own driver; it is
// weak enough that the display's driver still wins the pad.
static bool raw_c(void)
{
    gpio_set_oeover(PIN_BUTTON_C, GPIO_OVERRIDE_LOW);  // stop driving GP0
    gpio_pull_up(PIN_BUTTON_C);
    busy_wait_us_32(1);                                // let the pad settle
    bool pressed = !gpio_get(PIN_BUTTON_C);
    gpio_set_oeover(PIN_BUTTON_C, GPIO_OVERRIDE_NORMAL);
    return pressed;
}

void hw_buttons_init(void)
{
    // gpio_get() reads through the pad's input buffer, which is disabled on
    // an output-only pad. Enable it on both; the pin FUNCTION is left alone,
    // so the LED and the display keep whatever they configured.
    gpio_set_input_enabled(PIN_LED, true);
    gpio_set_input_enabled(PIN_BUTTON_C, true);
}

// --- debounced event state (C gets short/long, A gets press) -------------
static bool c_down, a_down;
static uint32_t c_edge_ms, a_edge_ms;
static bool c_long_fired;

button_event_t hw_buttons_poll(void)
{
    uint32_t now = hw_millis();

    // ---- button C: short vs long ----
    bool c = raw_c();
    // Accept a level change only once the previous accepted edge is
    // DEBOUNCE_MS old; anything faster is contact bounce.
    if (c != c_down && now - c_edge_ms >= DEBOUNCE_MS) {
        c_down = c;
        c_edge_ms = now;
        if (c) {
            c_long_fired = false;              // a fresh press begins
        } else if (!c_long_fired) {
            // Released before the hold matured, so this was a tap. If the
            // long event already fired, the release is swallowed instead —
            // one press must never produce two events.
            return BTN_C_SHORT;
        }
    }
    // The long press is reported while the button is still down, so the
    // operator gets feedback at the moment the threshold is crossed rather
    // than on release. c_edge_ms is the press instant here, because the
    // branch above only runs on accepted edges.
    if (c_down && !c_long_fired && now - c_edge_ms >= LONGPRESS_MS) {
        c_long_fired = true;                   // once per press
        return BTN_C_LONG;
    }

    // ---- button A: press edge only ----
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
    // Drop the debounce state as well, so the release that just happened
    // cannot be reported as an event by the next poll.
    c_down = a_down = false;
    c_long_fired = false;
}
