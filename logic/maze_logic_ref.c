// maze_logic_ref.c — a second, deliberately simpler junction classifier, plus
// junction_name().
//
// This is an intentionally crude implementation of the same job
// classify_junction() does, kept in the build as a cross-check. It reads one
// hard-coded threshold, has no tuning.h knobs, no stated GOAL-vs-CROSS margin
// and no refusal band beyond a single case, which is the point: everything
// the real classifier adds is visible as the diff between the two files.
//
// It also serves as a fallback lane. Comment out USE_MY_CLASSIFIER in
// include/tuning.h and explore runs on this classifier instead — same
// decision layer, same interfaces, only the evidence-to-symbol step swapped —
// so a classifier misbehaving on the floor can be checked against a control
// group with one #define. The simulator re-proves this lane on every
// `make -C tests/logic`, so it cannot quietly rot.

#include "maze_logic.h"

// Fixed on purpose — crude is this file's job. The threshold is file-local
// (no header, no knob), and tests/logic/sim_maze.c mirrors it as
// SIM_REF_DARK: the Makefile extracts the number straight out of this line
// and passes it in, and the simulator refuses to compile if the two disagree.
// Changing this value therefore breaks the build rather than silently moving
// the fallback lane out of the band the simulator's marginal-tape palette
// straddles.
#define REF_DARK 600

static bool dark(uint16_t v) { return v >= REF_DARK; }

junction_t classify_junction_ref(const sensor_snapshot_t *before,
                                 const sensor_snapshot_t *at_center)
{
    bool left     = dark(before->s[0]);
    bool right    = dark(before->s[4]);
    bool straight = dark(at_center->s[1]) || dark(at_center->s[2]) ||
                    dark(at_center->s[3]);

    // Goal patch: after creeping forward the bar is STILL on solid black.
    int center_dark = 0;
    for (int i = 0; i < 5; i++) {
        if (dark(at_center->s[i])) { center_dark++; }
    }
    if (center_dark >= 4) { return JCT_GOAL; }

    if (left && right) { return straight ? JCT_CROSS : JCT_T; }
    if (left)          { return straight ? JCT_STRAIGHT_LEFT : JCT_LEFT_ONLY; }
    if (right)         { return straight ? JCT_STRAIGHT_RIGHT : JCT_RIGHT_ONLY; }
    return straight ? JCT_NONE : JCT_DEAD_END;
}

const char *junction_name(junction_t j)
{
    switch (j) {
    case JCT_LEFT_ONLY:      return "LEFT";
    case JCT_RIGHT_ONLY:     return "RIGHT";
    case JCT_STRAIGHT_LEFT:  return "STR+LEFT";
    case JCT_STRAIGHT_RIGHT: return "STR+RIGHT";
    case JCT_T:              return "T";
    case JCT_CROSS:          return "CROSS";
    case JCT_DEAD_END:       return "DEAD END";
    case JCT_GOAL:           return "GOAL";
    default:                 return "???";
    }
}
