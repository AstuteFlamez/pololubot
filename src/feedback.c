// feedback.c — buzzer (PWM on the buzzer pin), RGB LEDs (vendor APA102
// driver) and the status/path drawing that goes with them (via ui.c).
// The contracts for everything here are in feedback.h.

#include "feedback.h"
#include "ui.h"
#include "hw_millis.h"
#include "pins.h"
#include "tuning.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "rgb_leds.h"        // vendor lib (APA102 over spi0, shared w/ OLED)

// PWM slices are pin/2, and the odd pin in a pair is channel B.
#define BUZZER_SLICE 3       // PIN_BUZZER = GP7 → slice 3, channel B
#define RGB_BRIGHTNESS 3     // 0..31 — dim is plenty indoors

// The state color legend: from twenty feet away in a maze the LEDs are
// the only readout of what the firmware thinks it is doing, and nothing
// can be printed from a running control loop. One entry per mode, indexed
// by run_mode_t, so the table must grow with the enum.
static const rgb_color state_color[MODE_COUNT] = {
    [MODE_DIAG]      = {  0,  0, 40 },   // blue    — "on the bench"
    [MODE_CALIBRATE] = { 40, 30,  0 },   // yellow  — "learning the floor"
    [MODE_EXPLORE]   = { 40, 10,  0 },   // orange  — "hunting"
    [MODE_SOLVED]    = {  0, 40, 20 },   // teal    — "I know the way"
    [MODE_REPLAY]    = { 40,  0, 40 },   // magenta — "executing"
    [MODE_DONE]      = {  0, 40,  0 },   // green   — "victory lap"
    [MODE_SPARE]     = { 40, 40, 40 },   // white   — unused slot
    [MODE_LOGDUMP]   = { 10, 10, 10 },   // dim white — "talking to USB"
};

void feedback_init(void)
{
    gpio_set_function(PIN_BUZZER, GPIO_FUNC_PWM);
    pwm_set_clkdiv_int_frac(BUZZER_SLICE, 125, 0);   // 125 MHz/125 = 1 MHz tick
    pwm_set_wrap(BUZZER_SLICE, 1000);
    pwm_set_chan_level(BUZZER_SLICE, PWM_CHAN_B, 0); // silent
    pwm_set_enabled(BUZZER_SLICE, true);

    rgb_leds_init();
}

static void buzzer_tone(uint16_t hz)
{
    if (hz == 0) {
        pwm_set_chan_level(BUZZER_SLICE, PWM_CHAN_B, 0);
        return;
    }
    // The slice ticks at 1 MHz, so wrap is the period in µs and the
    // frequency is 1e6/hz counts. 50% duty is the loudest square wave.
    uint16_t wrap = (uint16_t)(1000000u / hz - 1);
    pwm_set_wrap(BUZZER_SLICE, wrap);
    pwm_set_chan_level(BUZZER_SLICE, PWM_CHAN_B, (uint16_t)((wrap + 1) / 2));
}

void feedback_beep(uint16_t hz, uint16_t ms)
{
    buzzer_tone(hz);
    sleep_ms(ms);
    buzzer_tone(0);
}

void feedback_play_melody(const note_t *m)
{
    // {0, ms} is a rest and {0, 0} is the terminator, so both fields must
    // be zero before the loop stops.
    for (; m->hz != 0 || m->ms != 0; m++) {
        buzzer_tone(m->hz);
        sleep_ms(m->ms);
    }
    buzzer_tone(0);
}

void feedback_rgb_all(uint8_t r, uint8_t g, uint8_t b)
{
    rgb_color c = { r, g, b };
    rgb_color six[6] = { c, c, c, c, c, c };
    rgb_leds_write(six, 6, RGB_BRIGHTNESS);
}

// ---------------------------------------------------------------- hooks

void on_state_change(run_mode_t m)
{
    rgb_color c = state_color[m];
    feedback_rgb_all(c.red, c.green, c.blue);
}

// Draw the path as 8x16 text: two rows of 16 characters, so the last 32
// moves are visible. Older moves scroll off the front.
static void draw_path(const path_t *p)
{
    char row[17];
    int start = (p->len > 32) ? p->len - 32 : 0;   // show the tail
    for (int r = 0; r < 2; r++) {
        int n = 0;
        for (int i = start + r * 16; i < p->len && n < 16; i++, n++) {
            row[n] = p->moves[i];
        }
        row[n] = '\0';
        ui_text_big(0, 2 + r, "%-16s", row);
    }
    if (p->overflow) { ui_text(13, 1, "OVF"); }
}

void on_junction(junction_t j, move_t decided, const path_t *path_so_far)
{
    ui_text(0, 1, "%-9s -> %c  ", junction_name(j),
            decided ? decided : '?');
    draw_path(path_so_far);
    ui_flush_now();               // stopped at the junction, so a push is legal
    feedback_beep(NOTE_A5, 30);   // one tick per junction, countable by ear
}

void on_goal(void)
{
    on_state_change(MODE_SOLVED);
    ui_text(0, 1, "GOAL!           ");
    ui_flush_now();
    feedback_play_melody(melody_goal);
}

void on_fault(const char *msg)
{
    feedback_rgb_all(60, 0, 0);
    ui_clear();
    ui_text_big(0, 0, "FAULT");
    ui_text(0, 3, "%-16s", msg);
    ui_text(0, 6, "any button: menu");
    ui_flush_now();
    for (int i = 0; i < 3; i++) {         // three low groans
        feedback_beep(NOTE_C4, 150);
        sleep_ms(80);
    }
}

void on_countdown(int n)
{
    ui_clear();
    ui_text_big(7, 1, "%d", n);
    ui_flush_now();
    // Pitch rises as n falls, so the countdown can be followed without
    // watching the screen: 523 Hz, 623 Hz, 723 Hz.
    feedback_beep((uint16_t)(NOTE_C5 + (3 - n) * 100), 120);
}
