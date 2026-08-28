// feedback.h — the hook layer: everything the robot does to be UNDERSTOOD.
//
// Sounds, RGB colors, and status drawing all live behind these hooks, so
// the run logic in main.c reads like a story and the "show" is swappable.
// The defaults in feedback.c work and ship; the swap points are the mode
// color table (feedback.c) and melodies.h — change either and nothing in
// main.c or drive.c needs to know. That is the whole point of a hook
// layer, and it is the cheapest place in the codebase to make the robot
// yours.
//
// NOTE: melodies and beeps here are BLOCKING and are only ever called with
// the motors stopped (junction pauses, goal, faults). Keep it that way.
#ifndef FEEDBACK_H
#define FEEDBACK_H

#include <stdint.h>
#include "maze_logic.h"
#include "melodies.h"

// The whole firmware is one mode machine (Day 7 grown up).
typedef enum {
    MODE_DIAG,       // live sensor/battery/encoder dashboard
    MODE_CALIBRATE,  // gyro bias + line sensor sweep
    MODE_EXPLORE,    // walk the maze (walker as shipped; brain on Day 19)
    MODE_SOLVED,     // goal found — path on screen (simplify on Day 20)
    MODE_REPLAY,     // run the simplified path (Day 20)
    MODE_DONE,       // replay finished — lap times on screen
    MODE_SPARE,      // yours. Rename it. Do something weird with it.
    MODE_LOGDUMP,    // dump the telemetry ring over USB
    MODE_COUNT
} run_mode_t;

void feedback_init(void);

// ---- hooks called by main.c -------------------------------------------
void on_state_change(run_mode_t m);                  // RGB color + header
void on_junction(junction_t j, move_t decided,       // beep + junction line
                 const path_t *path_so_far);         //   + live path string
void on_goal(void);                                  // fanfare + green
void on_fault(const char *msg);                      // red + dirge + message
void on_countdown(int n);                            // 3..2..1 beeps

// ---- lower-level pieces the hooks are built from ----------------------
void feedback_beep(uint16_t hz, uint16_t ms);        // blocking beep
void feedback_play_melody(const note_t *m);          // blocking melody
void feedback_rgb_all(uint8_t r, uint8_t g, uint8_t b);

#endif
