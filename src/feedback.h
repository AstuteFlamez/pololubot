// feedback.h — everything the robot does to be understood from across a
// room: buzzer tones, RGB colors, and the status and path lines on the
// OLED.
//
// The run loops in main.c call the on_* hooks and never drive the buzzer
// or the LEDs directly, so the whole presentation can be changed in two
// places: the mode color table in feedback.c and the note tables in
// melodies.h.
//
// Every sound here BLOCKS for its full duration (sleep_ms per note), and
// several hooks flush the OLED as well. They are therefore only ever
// called with the motors stopped — junction pauses, goal, faults, the
// countdown. Calling one from inside a control loop would stall the loop
// for as long as the sound lasts.
#ifndef FEEDBACK_H
#define FEEDBACK_H

#include <stdint.h>
#include "maze_logic.h"
#include "melodies.h"

// The firmware's top-level modes: main.c's mode machine and menu are
// built on these, and feedback.c indexes its color table by them, so
// every value below MODE_COUNT needs a table entry.
//
// The numeric values are a wire contract. EV_RUN_START stores the mode
// in the telemetry ring, so every CSV ever dumped encodes them and the
// decoders read them back by number. Append new modes at the end; never
// reorder or renumber the existing ones.
typedef enum {
    MODE_DIAG,       // live sensor, encoder, gyro and battery dashboard
    MODE_CALIBRATE,  // gyro bias, then the line-sensor sweep
    MODE_EXPLORE,    // solve the maze by the left-hand rule, recording
    MODE_SOLVED,     // goal reached — show the raw and simplified paths
    MODE_REPLAY,     // drive the simplified path against the lap clock
    MODE_DONE,       // replay finished — last and best lap on screen
    MODE_SPARE,      // unused menu slot
    MODE_LOGDUMP,    // dump the telemetry ring over USB as CSV
    MODE_COUNT
} run_mode_t;

// Configure the buzzer PWM slice and the RGB LEDs. Call once at boot,
// before any other feedback call. Leaves the buzzer silent.
void feedback_init(void);

// ---- hooks called by main.c -------------------------------------------

// Light the LEDs in the color for mode `m` (must be < MODE_COUNT).
// Returns immediately.
void on_state_change(run_mode_t m);

// Junction pause: draw the classification, the decided move and the path
// so far, push the screen, and tick the buzzer once. Blocks ~30 ms.
// `decided` may be 0, which draws as '?'. Only the last 32 moves of the
// path fit on screen, and an overflowed path is marked OVF.
void on_junction(junction_t j, move_t decided,
                 const path_t *path_so_far);

// Goal reached: repaints the LEDs in the MODE_SOLVED color, puts GOAL! on
// the status line, then plays the goal melody. Blocks ~1.1 s.
void on_goal(void);

// Fault screen: red LEDs, a full-screen FAULT page carrying `msg` (16
// columns, clipped past that), then three low beeps. Blocks ~0.7 s. Does
// not wait for a button — the caller decides how the screen is dismissed.
void on_fault(const char *msg);

// One countdown step, n = 3, 2, 1: the digit on screen plus a beep whose
// pitch rises as n falls. Blocks 120 ms.
void on_countdown(int n);

// ---- lower-level pieces the hooks are built from ----------------------

// Square wave at `hz` for `ms` milliseconds, then silence. hz == 0 makes
// it a rest. Blocks for the whole duration.
void feedback_beep(uint16_t hz, uint16_t ms);

// Play a note table (melodies.h) up to its {0, 0} terminator, then leave
// the buzzer silent. Blocks for the sum of the note durations.
void feedback_play_melody(const note_t *m);

// Set all six RGB LEDs to one color, 0..255 per channel, scaled by the
// fixed brightness in feedback.c. Returns after a short SPI write.
void feedback_rgb_all(uint8_t r, uint8_t g, uint8_t b);

#endif
