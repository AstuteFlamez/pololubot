// hw_buttons.h — buttons A and C, debounced, with short/long press events.
// You built debouncing on Day 3 — this is the same idea, hardened.
//
// Button B is the BOOTSEL pin; we leave it alone so it stays a guaranteed
// recovery path (hold B + RESET → RPI-RP2 drive, always).
#ifndef HW_BUTTONS_H
#define HW_BUTTONS_H

#include <stdbool.h>

typedef enum {
    BTN_NONE = 0,
    BTN_C_SHORT,   // C tapped (< long threshold): menu next
    BTN_C_LONG,    // C held ≥ 600 ms: fires ONCE while still held (GO)
    BTN_A_PRESS,   // A pressed: also GO
} button_event_t;

void hw_buttons_init(void);

// Poll from the idle/menu loop (~every 10 ms). Debounced edge events.
button_event_t hw_buttons_poll(void);

// Raw "is anything pressed right now" — the always-stop check used inside
// every motor loop. Cheap (~2 µs) and unconditional.
bool hw_buttons_any(void);

// Block until every button is released (plus a settle delay), so the press
// that STARTED a run doesn't also instantly STOP it.
void hw_buttons_wait_release(void);

#endif
