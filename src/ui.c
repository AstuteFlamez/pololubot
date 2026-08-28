// ui.c — throttled wrapper over the vendor display driver in
// third_party/pololu_3pi_2040_robot, which owns the frame buffer, the
// fonts and the SPI push to the SH1106 panel. This file adds only the
// flush policy: a dirty flag so an unchanged frame is never pushed, and
// a minimum interval between pushes. The reason for the policy, and the
// rule against flushing mid-maneuver, are in ui.h.

#include "ui.h"
#include "hw_millis.h"
#include <stdio.h>
#include <stdarg.h>
#include "display.h"          // vendor lib: frame buffer + fonts + SPI push

#define FLUSH_MIN_MS 100      // at most 10 pushes per second

static uint32_t last_flush_ms;
static bool dirty;

void ui_init(void)
{
    display_init();
    display_set_font(font_8x8);
    ui_clear();
    ui_flush_now();
}

void ui_clear(void)
{
    display_fill(0);
    dirty = true;
}

// Format into a fixed 32-byte buffer and draw at pixel (px, py). One
// screen line is 16 characters, so 31 usable bytes is deliberate slack;
// vsnprintf truncates rather than overruns if a caller asks for more.
static void vtext(const uint8_t *font, int px, int py,
                  const char *fmt, va_list ap)
{
    char buf[32];
    vsnprintf(buf, sizeof buf, fmt, ap);
    display_set_font(font);
    display_text(buf, px, py, COLOR_WHITE_ON_BLACK);
    dirty = true;
}

void ui_text(int col, int row, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vtext(font_8x8, col * 8, row * 8, fmt, ap);
    va_end(ap);
}

void ui_text_big(int col, int row, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vtext(font_8x16, col * 8, row * 16, fmt, ap);
    va_end(ap);
}

void ui_bars(int x, int y, int height, const uint16_t v[5], uint16_t vmax)
{
    if (vmax == 0) { vmax = 1; }
    for (int i = 0; i < 5; i++) {
        int h = (int)((uint32_t)v[i] * (uint32_t)height / vmax);
        if (h > height) { h = height; }
        int bx = x + i * 12;   // 8 px bar + 4 px gap
        // Clear the whole slot first: bars shrink as well as grow, and
        // the frame buffer still holds the previous frame's taller bar.
        display_fill_rect(bx, y, 8, height, COLOR_BLACK);
        // Bars grow upward from the baseline, so the origin moves up.
        display_fill_rect(bx, y + height - h, 8, h, COLOR_WHITE);
    }
    dirty = true;
}

bool ui_flush(void)
{
    uint32_t now = hw_millis();
    if (!dirty || now - last_flush_ms < FLUSH_MIN_MS) { return false; }
    last_flush_ms = now;
    dirty = false;
    display_show();
    return true;
}

// Both flushes stamp last_flush_ms and clear `dirty`, so an unconditional
// push also restarts the throttle window for the throttled one.
void ui_flush_now(void)
{
    last_flush_ms = hw_millis();
    dirty = false;
    display_show();
}
