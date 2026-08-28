// maze_logic_ref.c — the reference junction classifier: the fallback lane.
//
// Why this exists: the project's first firmware was a "maze walker" that
// bounced off dead ends on a crude reference classifier, written so the
// plumbing could be exercised separately from the brain. No session ever
// ran on it — the brain landed first (day19/README.md:42–46 tells the
// same history) — so that scaffolding job lapsed unused. The classifier
// stays compiled into the binary anyway, for two better reasons:
//
//   * FALLBACK — comment out USE_MY_CLASSIFIER in tuning.h and explore
//     runs on this classifier instead: same decision layer, same seams,
//     only the evidence→symbol step swapped. If the real classifier
//     ever misbehaves on the floor, one #define gives you a control
//     group — and the simulator re-proves this lane on every
//     `make -C host_tests`, so it can never quietly rot.
//   * BASELINE — it is deliberately crude: one hard-coded threshold, no
//     tuning.h knobs, no margin logic, no refusal gap beyond one case.
//     Read it freely, side by side with classify_junction(): everything
//     the real classifier adds (calibrated knobs, the stated
//     GOAL-vs-CROSS margin, honest JCT_NONE refusals) is visible as the
//     diff between these two files.

#include "maze_logic.h"

#define REF_DARK 600  // fixed on purpose — crude is this file's job (above)

static bool dark(uint16_t v) { return v >= REF_DARK; }

junction_t classify_junction_ref(const sensor_snapshot_t *before,
                                 const sensor_snapshot_t *at_center)
{
    bool left     = dark(before->s[0]);
    bool right    = dark(before->s[4]);
    bool straight = dark(at_center->s[1]) || dark(at_center->s[2]) ||
                    dark(at_center->s[3]);

    // Goal patch: after creeping forward we are STILL on solid black.
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
