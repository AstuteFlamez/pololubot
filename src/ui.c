// ui.c — throttled wrapper over Pololu's display driver (the one library
// piece this capstone takes as-is: writing an SH1106 driver teaches you
// less than a maze does).

#include "ui.h"
#include "hw_millis.h"
#include <stdio.h>
#include <stdarg.h>
#include "display.h"          // Pololu lib: frame buffer + fonts + SPI push

#define FLUSH_MIN_MS 100      // ≤10 Hz, always

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
        int bx = x + i * 12;
        display_fill_rect(bx, y, 8, height, COLOR_BLACK);         // clear slot
        display_fill_rect(bx, y + height - h, 8, h, COLOR_WHITE); // the bar
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

void ui_flush_now(void)
{
    last_flush_ms = hw_millis();
    dirty = false;
    display_show();
}
