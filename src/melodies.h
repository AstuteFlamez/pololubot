// melodies.h — note tables for the buzzer.
//
// A melody is an array of note_t entries, each a {frequency_hz,
// duration_ms} pair, terminated by {0, 0}. A note with hz == 0 and
// ms > 0 is a rest. feedback_play_melody() walks the table in order,
// holding each note for its duration (blocking, one sleep_ms per note)
// and leaving the buzzer silent at the end. Durations include no gap
// between notes: repeat a pitch and it sounds as one long note unless a
// short REST separates the two.
#ifndef MELODIES_H
#define MELODIES_H

#include <stdint.h>

typedef struct {
    uint16_t hz;
    uint16_t ms;
} note_t;

// Equal-temperament pitches, octaves 4–6 (A4 = 440 Hz).
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_E6  1319
#define NOTE_G6  1568

#define REST(ms) { 0, (ms) }
#define MELODY_END { 0, 0 }

// Played when the explorer lands on the goal patch.
static const note_t melody_goal[] = {
    { NOTE_C5, 120 }, { NOTE_E5, 120 }, { NOTE_G5, 120 }, { NOTE_C6, 200 },
    REST(60),
    { NOTE_G5, 100 }, { NOTE_C6, 350 },
    MELODY_END
};

// Played when a replay run sets a new best lap time.
static const note_t melody_new_best[] = {
    { NOTE_C5, 90 }, { NOTE_C5, 90 }, REST(40),
    { NOTE_E5, 90 }, { NOTE_G5, 90 }, { NOTE_C6, 140 },
    { NOTE_E6, 140 }, { NOTE_G6, 320 },
    MELODY_END
};

// Further melodies go here and are played by passing them to
// feedback_play_melody():
// static const note_t melody_mine[] = { ... , MELODY_END };

#endif // MELODIES_H
