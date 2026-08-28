// ui.h — the only module that touches the OLED.
//
// Drawing (ui_text, ui_text_big, ui_bars) writes an in-RAM frame buffer
// and is cheap. Pushing that buffer to the panel over SPI is the
// expensive part: the transfer takes several milliseconds, so ui_flush()
// throttles pushes to at most one per 100 ms.
//
// Never push a frame during a maneuver. The control loops run on a 2 ms
// tick, and an OLED transfer — like a USB write — stalls long enough to
// blow that budget; the resulting hole in the control law shows up as a
// visible wobble. drive.c therefore contains no ui calls at all, and the
// run loops in main.c draw only with the wheels stopped.
#ifndef UI_H
#define UI_H

#include <stdint.h>
#include <stdbool.h>

// Bring up the panel, select the 8x8 font and show a cleared screen.
// Blocks for the init sequence. Call once, before any other ui call.
void ui_init(void);

// Blank the frame buffer. RAM only: the panel keeps showing the last
// pushed frame until the next flush.
void ui_clear(void);

// printf-style text in the 8x8 font on a 16 column x 8 row grid of
// character cells (col 0..15, row 0..7). Formatted output is truncated
// at 31 characters. Frame buffer only — nothing appears until a flush.
void ui_text(int col, int row, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

// Same, in the 8x16 font: 16 columns x 4 rows (col 0..15, row 0..3).
// Used for the path string and the lap time.
void ui_text_big(int col, int row, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

// Five-bar sensor graph with its top-left corner at pixel (x, y). Each
// bar is 8 px wide on a 12 px pitch and at most `height` px tall; v[i]
// is scaled against vmax and clipped to `height`. vmax == 0 is treated
// as 1, so the scaling cannot divide by zero. Frame buffer only.
void ui_bars(int x, int y, int height, const uint16_t v[5], uint16_t vmax);

// Push the frame buffer to the OLED, but only if something was drawn
// since the last push and at least 100 ms have elapsed since it. Returns
// true when the push actually happened, and blocks for the transfer when
// it does.
bool ui_flush(void);

// Push unconditionally and restart the throttle window (menu
// transitions, fault and result screens). Blocks for the transfer, so
// the caller must have the motors stopped.
void ui_flush_now(void);

#endif
