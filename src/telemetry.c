// telemetry.c — the ring behind telemetry.h (the WHY of recording at all
// lives in that header). 8192 records × 16 bytes = 128 KB of the RP2040's
// 264 KB SRAM, spent deliberately: nothing else in this firmware wants
// that RAM, and history you didn't record is the one debug tool you can't
// add after the crash.

#include <stdio.h>

#include "telemetry.h"
#include "hw_millis.h"
#include "tuning.h"

#define TEL_RING 8192   // power of two, so wrap is a mask, not a divide

typedef struct __attribute__((packed)) {
    uint16_t t_ms;      // low 16 bits of hw_millis() — wraps at 65.536 s;
                        // fine, the host-side parser unwraps
    uint8_t  kind;      // TEL_TICK_* or EV_*
    uint8_t  s[5];      // calibrated line / 4 (0..250)
    int16_t  a, b, left, right;
} tel_rec_t;

static tel_rec_t ring[TEL_RING];   // .bss — costs boot-time zeroing, not flash
static uint32_t head;              // next write slot
static uint32_t total;             // records ever written since reset

void telemetry_init(void)
{
    telemetry_reset();
}

void telemetry_reset(void)
{
    // Stale array contents are unreachable once total is 0 — no memset.
    head = 0;
    total = 0;
}

// Runs inside the 2 ms control loop: no printf, no float, no allocation.
// Honesty note: hw_millis() is NOT free — it divides a 64-bit microsecond
// count by 1000, and the M0+ has no 64-bit divide instruction, so that's
// a software-divide library call. It costs on the order of a microsecond,
// which the 2 ms budget absorbs without noticing — the point is that the
// cost is bounded and constant, unlike I/O, which can stall for
// milliseconds waiting on a host.
void telemetry_tick(uint8_t phase, const uint16_t line[5],
                    int16_t a, int16_t b, int16_t left, int16_t right)
{
    tel_rec_t *r = &ring[head];
    r->t_ms = (uint16_t)hw_millis();
    r->kind = phase;
    for (int i = 0; i < 5; i++) { r->s[i] = (uint8_t)(line[i] >> 2); }
    r->a = a;
    r->b = b;
    r->left = left;
    r->right = right;
    head = (head + 1u) & (TEL_RING - 1u);
    total++;
}

void telemetry_event(uint8_t ev, int16_t a, int16_t b)
{
    tel_rec_t *r = &ring[head];
    r->t_ms = (uint16_t)hw_millis();
    r->kind = ev;
    for (int i = 0; i < 5; i++) { r->s[i] = 0; }
    r->a = a;
    r->b = b;
    r->left = 0;
    r->right = 0;
    head = (head + 1u) & (TEL_RING - 1u);
    total++;
}

uint32_t telemetry_count(void)
{
    return (total < TEL_RING) ? total : TEL_RING;
}

// Dump-time context (telemetry_set_dump_context) — kept in statics so the
// dump signature stays stable and a dump WITHOUT context still prints its
// original header shape. Zero-initialized .bss; have_dump_ctx is the
// "was I told anything?" latch, because 0 mV is a lie, not a default.
static uint16_t dump_batt_mv;
static uint16_t dump_cal_min[5], dump_cal_max[5];
static int      have_dump_ctx;

void telemetry_set_dump_context(uint16_t batt_mv,
                                const uint16_t cal_min[5],
                                const uint16_t cal_max[5])
{
    dump_batt_mv = batt_mv;
    for (int i = 0; i < 5; i++) {
        dump_cal_min[i] = cal_min[i];
        dump_cal_max[i] = cal_max[i];
    }
    have_dump_ctx = 1;
}

// Indexed by kind - EV_RUN_START; must track the EV_* list in telemetry.h.
static const char *const ev_names[] = {
    "RUN_START", "FOLLOW_START", "BLIND_END", "JCT_DETECT", "DEADEND",
    "LOSS", "BT_START", "BT_FOUND", "BT_FAIL", "CREEP_START", "CREEP_END",
    "TURN_START", "TURN_END", "CLASSIFY", "ABORT", "TIMEOUT", "FAULT",
    "RESUME",
};

void telemetry_dump(void)
{
    uint32_t n = telemetry_count();
    int wrapped = (total > TEL_RING);
    // Once wrapped, head points at the oldest surviving record.
    uint32_t start = wrapped ? head : 0;

    printf("# 3pi2040 telemetry v1\n");
    // The knob line: every tuning number that changes how a ROW of this
    // dump is READ — speeds (the saturation and duty baselines), the
    // thresholds that decided what counted as a junction, a goal, or a
    // loss, and the geometry that turns counts into millimetres. Not all
    // of tuning.h: the bar for adding a token is "would a reader interpret
    // some row differently if this value were different?". A shared CSV
    // must name its own build — the robot that produced it may be three
    // flashes ahead by the time anyone reads the file. However long it
    // grows, it stays ONE '#' line: the 3-line header frame is contract
    // (I3), and every decoder counts on it.
    printf("# period_us=%d kp=%d kd=%d explore=%d creep=%d turnmax=%d"
           " outer_thresh=%d lost_thresh=%d blind_mm=%d deadend_p_max=%d"
           " bt_speed=%d counts_per_mm_x100=%d"
           " replay=%d arrival=%d brake_mm=%d brake_ms=%d turn_kp=%d"
           " dark_thresh=%d goal_min_dark=%d creep_mm=%d think_ms=%d"
           " cal_min_span=%d\n",
           CONTROL_PERIOD_US, LINE_KP, LINE_KD, SPEED_EXPLORE, SPEED_CREEP,
           SPEED_TURN_MAX, JCT_OUTER_THRESH, LINE_LOST_THRESH, BLIND_MM,
           DEADEND_P_MAX, BACKTRACK_SPEED, ENC_COUNTS_PER_MM_X100,
           SPEED_REPLAY, SPEED_ARRIVAL, ARRIVAL_BRAKE_MM, ARRIVAL_BRAKE_MS,
           TURN_KP, JCT_DARK_THRESH, GOAL_MIN_DARK, CREEP_MM, THINK_PAUSE_MS,
           CAL_MIN_SPAN);
    // The dump-facts line: what is true of THIS dump, as opposed to the
    // build (line 2 above) — how much history survived, whether the ring
    // wrapped, and, when main.c handed them in, the battery at dump time
    // and the calibration window the sensor columns were scaled by.
    if (have_dump_ctx) {
        printf("# records=%lu wrapped=%d batt_mv=%u"
               " cal_min=%u,%u,%u,%u,%u cal_max=%u,%u,%u,%u,%u\n",
               (unsigned long)n, wrapped, dump_batt_mv,
               dump_cal_min[0], dump_cal_min[1], dump_cal_min[2],
               dump_cal_min[3], dump_cal_min[4],
               dump_cal_max[0], dump_cal_max[1], dump_cal_max[2],
               dump_cal_max[3], dump_cal_max[4]);
    } else {
        printf("# records=%lu wrapped=%d\n", (unsigned long)n, wrapped);
    }
    printf("t_ms,rec,s0,s1,s2,s3,s4,a,b,left,right\n");

    for (uint32_t i = 0; i < n; i++) {
        const tel_rec_t *r = &ring[(start + i) & (TEL_RING - 1u)];
        char letter[2] = { 0, 0 };
        char unk[8];
        const char *rec;
        if (r->kind >= TEL_TICK_FOLLOW && r->kind <= TEL_TICK_CENTER_SNAP) {
            letter[0] = "?FBCTKJA"[r->kind];
            rec = letter;
        } else if (r->kind >= EV_RUN_START && r->kind <= EV_RESUME) {
            rec = ev_names[r->kind - EV_RUN_START];
        } else {
            snprintf(unk, sizeof unk, "UNK%u", (unsigned)r->kind);
            rec = unk;
        }
        // s stored /4 → re-expand to ~0..1000 (events stored their s as 0).
        printf("%u,%s,%u,%u,%u,%u,%u,%d,%d,%d,%d\n",
               (unsigned)r->t_ms, rec,
               r->s[0] * 4u, r->s[1] * 4u, r->s[2] * 4u,
               r->s[3] * 4u, r->s[4] * 4u,
               r->a, r->b, r->left, r->right);
    }
    printf("# end\n");
}
