// hw_buttons.h — buttons A and C, debounced, with short/long press events.
//
// Button B is the BOOTSEL pin and is deliberately left untouched, so
// holding B while pressing RESET always drops the board into the RPI-RP2
// bootloader — a recovery path no firmware bug can take away.
#ifndef HW_BUTTONS_H
#define HW_BUTTONS_H

#include <stdbool.h>

typedef enum {
    BTN_NONE = 0,
    BTN_C_SHORT,   // C pressed and released before the long-press threshold
    BTN_C_LONG,    // C held 600 ms: delivered once, while the button is down
    BTN_A_PRESS,   // A pressed (delivered on the press edge, not the release)
} button_event_t;

// Enable the input buffers on GP25 and GP0 so their levels can be sampled.
// Deliberately does not set a pin function or direction: both pads belong to
// the LED and the display (pins.h), and this module only borrows them.
void hw_buttons_init(void);

// Return at most one debounced event per call, BTN_NONE when nothing has
// happened. Poll it steadily from an idle or menu loop, roughly every 10 ms:
// the 20 ms debounce and the 600 ms long-press threshold are both measured
// against hw_millis() rather than call counts, but events are edge-detected,
// so a caller that stops polling misses presses rather than queuing them.
// C is checked before A, so a simultaneous press reports C first.
button_event_t hw_buttons_poll(void);

// True while any button is physically down, undebounced and with no event
// bookkeeping. This is the abort check every motor loop runs on every
// iteration: a finger on any button stops the robot. Costs a couple of
// microseconds and is safe to call at any rate.
bool hw_buttons_any(void);

// Block until no button is down, then wait out the contact bounce, then
// clear the debounce state so no stale edge is reported afterwards. Call it
// after acting on a press, so that the press which STARTED a run is not
// immediately read back by hw_buttons_any() as the abort that stops it.
void hw_buttons_wait_release(void);

#endif
