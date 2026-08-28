// ui.h — the ONLY module allowed to touch the OLED.
//
// Rules of the screen:
//   1. All display calls in the whole project go through this file. If
//      you're typing display_* anywhere else, stop.
//   2. Drawing (ui_text etc.) only writes the in-RAM frame buffer — cheap.
//      Pushing that buffer over SPI (ui_flush) is the expensive part, so
//      it is throttled to ≤10 Hz.
//   3. NEVER flush during a maneuver. drive.c contains zero ui calls by
//      design: an SPI transfer mid-PID-loop is a multi-millisecond hole in
//      your control law, and you WILL see it as a wobble.
#ifndef UI_H
#define UI_H

#include <stdint.h>
#include <stdbool.h>

void ui_init(void);

void ui_clear(void);

// 8×8 font: 16 columns × 8 rows. printf-style.
void ui_text(int col, int row, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

// 8×16 font (big): 16 columns × 4 rows. Used for the path string + lap time.
void ui_text_big(int col, int row, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

// Live 5-bar sensor graph, values 0..vmax, drawn with its top-left at
// (x,y), each bar 8 px wide, `height` px tall.
void ui_bars(int x, int y, int height, const uint16_t v[5], uint16_t vmax);

// Push the frame buffer to the OLED — but at most every 100 ms.
// Returns true if it actually flushed.
bool ui_flush(void);

// Push unconditionally (menu transitions, fault screens).
void ui_flush_now(void);

#endif
