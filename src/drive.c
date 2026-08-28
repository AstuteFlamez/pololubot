// drive.c — the motion layer: line following, junction detection, the
// arrival window, the junction creep, gyro turns, and line-loss recovery.
//
// Four rules hold everywhere in this file:
//   1. Every loop checks hw_buttons_any() and returns DRIVE_ABORT. A
//      finger on any button always stops the robot, no matter how wrong
//      the surrounding code is.
//   2. Every loop has a timeout. A maneuver that cannot finish says so
//      instead of grinding its gears forever.
//   3. No ui calls, ever. An OLED flush is a multi-millisecond SPI
//      transfer; a control loop cannot afford that hole in its timing,
//      and the hole is visible as a wobble.
//   4. No printf either — telemetry_tick() writes 16 bytes to RAM and
//      that is the only reporting a control loop is allowed to do.
//
// The follower runs on a FIXED tick (CONTROL_PERIOD_US). The raw sensor
// read takes anywhere from ~0.15 ms (all white) to ~1.1 ms (deep black),
// so a free-running loop speeds up and slows down ~7x with the floor
// under it — and since the D term is a per-tick difference, the damping
// would breathe with it. Fixing the period is what makes LINE_KD a
// constant of the robot instead of a constant of the floor.
//
// Layout, roughly the order a segment walks through the phases: the tick
// clock, the follower (steering law and post-turn blind window),
// backtrack recovery, the loss state machine, the junction approach that
// drives all of those, the arrival brake, the creep, and the gyro turns.
// Each phase opens with what it assumes on entry and what it leaves
// behind on exit.

#include "drive.h"
#include "hw_motors.h"
#include "hw_encoders.h"
#include "hw_imu.h"
#include "hw_line.h"
#include "hw_buttons.h"
#include "hw_millis.h"
#include "telemetry.h"
#include "tuning.h"
#include <math.h>
#include "pico/stdlib.h"

void drive_init(void)
{
    hw_motors_init();
    hw_encoders_init();
    hw_line_init();
}

void drive_stop(void)
{
    hw_motors_stop();
}

// Squeeze a control value into a telemetry column. Telemetry only —
// nothing in a control path is ever clamped through here.
static int16_t sat16(int32_t v)
{
    if (v >  32000) { return  32000; }
    if (v < -32000) { return -32000; }
    return (int16_t)v;
}

// mm → encoder counts, staying in integer math (ENC_COUNTS_PER_MM_X100
// carries the ×100). Every distance-measured window in this file — blind,
// grace, decay, nudge, creep, arrival — goes through this one conversion.
#define MM_TO_COUNTS(mm) ((mm) * ENC_COUNTS_PER_MM_X100 / 100)

// ------------------------------------------------------------- tick clock
// One deadline, advanced by a fixed step. If a tick ever overruns its
// period (it shouldn't: worst-case read + math is ~1.2 ms), resync
// instead of sprinting to catch up — a late tick is noise, a burst of
// back-to-back ticks is a D-term lie.
static uint32_t next_tick_us;

static void tick_begin(void) { next_tick_us = hw_micros(); }

static void tick_wait(uint32_t period_us)
{
    next_tick_us += period_us;
    if ((int32_t)(hw_micros() - next_tick_us) > 0) {
        next_tick_us = hw_micros();
        return;
    }
    while ((int32_t)(hw_micros() - next_tick_us) < 0) { }
}

// ---------------------------------------------------------------- follower
// The steering law every driving phase shares. Entry: follow_reset() has
// run for this segment, and the caller reads the five calibrated sensors
// once per fixed tick. Exit: this tick's command has been issued to the
// motors and recorded in last_cmd_left/right; the follower keeps no
// state the caller has to unwind.
//
// The shape is the stock Pololu weighted-average follower: position
// 0..4000 across the bar, error p = pos - 2000, command
// pid = p*LINE_KP + d*LINE_KD, and that command steers by SLOWING one
// wheel — each side clamped to [0, base]. Because neither wheel is ever
// commanded above `base`, the robot cannot outrun the speed the caller
// asked for while following, which is what keeps junction detection
// honest.
//
// The derivative is a plain difference of the RAW error between adjacent
// ticks: no filter, no averaging. That is only defensible because the
// tick period is fixed (see the file header) — d is then proportional to
// the true rate of change and LINE_KD keeps one meaning. Two departures
// from the stock controller, both because its gains were tuned at base
// speed 6000 in a ~3 ms MicroPython loop (LINE_KP and LINE_KD in
// include/tuning.h carry that history):
//   * the command is scaled by base/6000 — same numbers, same CURVATURE
//     at any base speed;
//   * d is skipped on the first tick after a reset. Differencing against
//     a made-up last_p of 0 is a full-scale derivative kick, and it would
//     land at exactly the worst moment: the first sample after a turn.
static int32_t last_p;
static bool    have_p;           // last_p holds A value (suppresses first-tick D)
static bool    have_measured_p;  // that value traces back to a REAL sensor read
                                 // — the center-lost branch writes synthetic
                                 // last_p values, so have_p alone can't tell a
                                 // memory from a fabrication

// Recent MEASURED |p| history (synthetic center-lost samples excluded),
// 64 samples ≈ 128 ms at the control rate. This is the jury for the
// dead-end-vs-swerve verdict: a line that ends while the follower was
// driving straight is a dead end; a line that "ends" while it was sawing
// at the wheel is a robot leaving the road.
static int16_t recent_p[64];
static uint8_t recent_i;

// What the last follow tick commanded. A junction detect returns with the
// motors still running (by design), so the 'J' snapshot records the
// command they are still executing at that instant.
static int16_t last_cmd_left, last_cmd_right;

// Per-tick encoder deltas, the robot's own breadcrumb trail (~2 s at the
// control rate). Backtracking replays it newest-first.
#define BT_RING 1024
static int8_t   bt_dl[BT_RING], bt_dr[BT_RING];
static uint16_t bt_head, bt_count;
static int32_t  bt_last_l, bt_last_r;

static void follow_reset(void)
{
    last_p = 0;
    have_p = false;
    have_measured_p = false;
    last_cmd_left = last_cmd_right = 0;
    for (int i = 0; i < 64; i++) { recent_p[i] = 0; }
    recent_i = 0;
    bt_head = bt_count = 0;
    bt_last_l = hw_encoder_left();
    bt_last_r = hw_encoder_right();
}

// Append this tick's encoder movement to the breadcrumb ring. The ±127
// clamp is the cost of storing a delta per wheel in one byte: at the
// control rate a tick's real travel is a handful of counts, so nothing
// mechanical can reach the clamp — only an encoder discontinuity can,
// and a clamped breadcrumb still retraces in the right direction.
static void bt_record(void)
{
    int32_t l = hw_encoder_left(), r = hw_encoder_right();
    int32_t dl = l - bt_last_l, dr = r - bt_last_r;
    bt_last_l = l;
    bt_last_r = r;
    if (dl >  127) { dl =  127; }  if (dl < -127) { dl = -127; }
    if (dr >  127) { dr =  127; }  if (dr < -127) { dr = -127; }
    bt_dl[bt_head] = (int8_t)dl;
    bt_dr[bt_head] = (int8_t)dr;
    bt_head = (uint16_t)((bt_head + 1) % BT_RING);
    if (bt_count < BT_RING) { bt_count++; }
}

// Largest |p| in the measured history — how hard the follower was
// correcting recently, which is what separates a dead end from a swerve.
static int32_t recent_abs_max(void)
{
    int32_t m = 0;
    for (int i = 0; i < 64; i++) {
        int32_t v = recent_p[i] < 0 ? -recent_p[i] : recent_p[i];
        if (v > m) { m = v; }
    }
    return m;
}

static void follow_tick(const uint16_t line[5], int32_t base, bool blind)
{
    int32_t pos;
    if (blind) {
        // Post-turn blind window: pieces of the junction just handled may
        // still sit under the OUTER sensors, and they would drag the
        // weighted average hard sideways. Steer from the center three
        // only until the bar has rolled BLIND_MM onto clean line.
        uint32_t sum = 0, weighted = 0;
        for (int i = 1; i <= 3; i++) {
            sum      += line[i];
            weighted += (uint32_t)line[i] * (uint32_t)(i * 1000);
        }
        // A weighted average divides by the light under the bar, and the
        // "position" of nothing is a random number: near-zero sum turns
        // single-count dust flicker into full-range position swings. 100
        // counts across three sensors (a tenth of ONE sensor's scale) is
        // the floor under which the read is noise — steer dead ahead
        // instead. The all-5 branch below needs no such floor: it only
        // runs when a center sensor is >= CENTER_LOST_THRESH, so its sum
        // is guaranteed real signal.
        if (sum >= 100) {
            pos = (int32_t)(weighted / sum);
            have_measured_p = true;      // a real read, center-3 scale
        } else {
            pos = 2000;
        }
    } else if (line[1] < CENTER_LOST_THRESH && line[2] < CENTER_LOST_THRESH &&
               line[3] < CENTER_LOST_THRESH) {
        // Center lost (overshoot on a correction): steer hard back toward
        // where the line WAS. The sign of the last error remembers that —
        // but only if that sign traces back to a real measurement. This
        // branch itself writes SYNTHETIC last_p values (p becomes ±2000
        // below), and have_p cannot tell those from the real thing: after
        // a reset, one pass through here would launder a made-up 0 into
        // last_p and the next tick would slam full-scale toward a coin
        // flip. So the gate is have_MEASURED_p — hold straight, every
        // tick, until the follower has actually seen the line once.
        pos = !have_measured_p ? 2000 : (last_p < 0) ? 0 : 4000;
    } else {
        uint32_t sum = 0, weighted = 0;
        for (int i = 0; i < 5; i++) {
            sum      += line[i];
            weighted += (uint32_t)line[i] * (uint32_t)(i * 1000);
        }
        pos = sum ? (int32_t)(weighted / sum) : 2000;
        have_measured_p = true;
        recent_p[recent_i++ & 63] = (int16_t)(pos - 2000);   // measured only
    }

    int32_t p = pos - 2000;
    int32_t d = have_p ? p - last_p : 0;
    last_p = p;
    have_p = true;

    // p*KP + d*KD can reach ~12M; the multiply by base needs 64 bits.
    int32_t pid =
        (int32_t)(((int64_t)(p * LINE_KP + d * LINE_KD) * base) / 6000);

    // Saturation is what turns a signed correction into "slow one wheel":
    // the low clamp stops a large pid from commanding a wheel backwards
    // (an in-place pivot mid-segment), the high clamp stops the outer
    // wheel from being driven past the speed the caller asked for. Both
    // sides are held to [0, base], so the pair can only ever be a
    // differential slowdown around `base`.
    int32_t left  = base + pid;
    int32_t right = base - pid;
    if (left  < 0) { left  = 0; }  if (left  > base) { left  = base; }
    if (right < 0) { right = 0; }  if (right > base) { right = base; }
    hw_motors_set(left, right);
    last_cmd_left  = (int16_t)left;
    last_cmd_right = (int16_t)right;

    telemetry_tick(blind ? TEL_TICK_BLIND : TEL_TICK_FOLLOW, line,
                   (int16_t)p, sat16(pid), (int16_t)left, (int16_t)right);
}

// -------------------------------------------------------------- backtrack
// Recovery from a swerve off the tape. Entry: the follow loop has ruled
// the line lost while the follower was correcting hard, and bt_dl/bt_dr
// hold the per-tick encoder deltas of the arc that got the robot there.
// Exit: DRIVE_OK with the robot stopped ON tape and settled (the follow
// loop's resume protocol takes it from there), DRIVE_LOST with the robot
// stopped somewhere on the floor, or DRIVE_ABORT. Every path stops the
// motors; none of them touch the follower's own state.
//
// The robot's own encoder history knows the arc it took; replay it
// newest-first as a RETREATING target and P-chase the target — closed
// loop on odometry, not on duty, so stiction and battery sag can't bend
// the path. Consuming one recorded tick per BACKTRACK_TICK_X control
// periods retraces at 1/BACKTRACK_TICK_X of the recorded speed:
// deliberately slow, this is a recovery, not a maneuver.
static bool bt_step(int32_t tl, int32_t tr, uint16_t line[5])
{
    int32_t el = tl - hw_encoder_left();
    int32_t er = tr - hw_encoder_right();
    int32_t cl = el * BACKTRACK_KP;
    int32_t cr = er * BACKTRACK_KP;
    if (cl >  BACKTRACK_SPEED) { cl =  BACKTRACK_SPEED; }
    if (cl < -BACKTRACK_SPEED) { cl = -BACKTRACK_SPEED; }
    if (cr >  BACKTRACK_SPEED) { cr =  BACKTRACK_SPEED; }
    if (cr < -BACKTRACK_SPEED) { cr = -BACKTRACK_SPEED; }
    hw_motors_set(cl, cr);

    hw_line_read_calibrated(line);
    telemetry_tick(TEL_TICK_BT, line, sat16(el), sat16(er),
                   sat16(cl), sat16(cr));

    // "Tape under the center THIS tick" — one sample, so it's a vote,
    // not a verdict. The caller counts BT_FOUND_CONFIRM_TICKS
    // consecutive votes before it believes them.
    return line[1] >= LINE_FOUND_THRESH || line[2] >= LINE_FOUND_THRESH ||
           line[3] >= LINE_FOUND_THRESH;
}

// The retrace confirmed tape under the bar — but the robot was REVERSING
// when it did, and reverse momentum keeps carrying it through the stop.
// So after the chassis settles, look again. If the center still sees
// tape: done. If it sees bare floor, the tape just confirmed is AHEAD of
// the bar (the robot backed over it) — creep FORWARD toward the find
// point until the center sees it again. Never further backward: reverse
// is the one direction guaranteed to be wrong here.
// Bounded in distance (RESUME_NUDGE_MM) and time (RESUME_NUDGE_TIMEOUT_MS),
// with the any-button stop, like every other motor loop in this file.
static drive_status_t bt_settle_verify(void)
{
    hw_motors_stop();
    sleep_ms(BT_SETTLE_MS);             // let the rocking die before reading

    uint16_t line[5];
    hw_line_read_calibrated(line);
    if (line[1] >= CENTER_LOST_THRESH || line[2] >= CENTER_LOST_THRESH ||
        line[3] >= CENTER_LOST_THRESH) {
        return DRIVE_OK;                // settled ON the tape — resume as-is
    }

    const int32_t limit = MM_TO_COUNTS(RESUME_NUDGE_MM);
    int32_t  l0 = hw_encoder_left();
    int32_t  r0 = hw_encoder_right();
    uint32_t t0 = hw_millis();
    int      streak = 0;

    tick_begin();
    for (;;) {
        tick_wait(CONTROL_PERIOD_US);
        if (hw_buttons_any()) {
            drive_stop();
            telemetry_event(EV_ABORT, TEL_TICK_CREEP, 0);
            return DRIVE_ABORT;
        }
        if (hw_millis() - t0 > RESUME_NUDGE_TIMEOUT_MS) { break; }

        int32_t dl = hw_encoder_left() - l0;
        int32_t dr = hw_encoder_right() - r0;
        int32_t progress = (dl + dr) / 2;
        if (progress >= limit) { break; }   // tape wasn't where it must be

        hw_line_read_calibrated(line);
        bool on_tape = line[1] >= LINE_FOUND_THRESH ||
                       line[2] >= LINE_FOUND_THRESH ||
                       line[3] >= LINE_FOUND_THRESH;
        streak = on_tape ? streak + 1 : 0;
        if (streak >= BT_FOUND_CONFIRM_TICKS) {
            hw_motors_stop();
            sleep_ms(BT_SETTLE_MS);     // same rock, same wait
            return DRIVE_OK;
        }

        // Same straightness servo as the junction creep: feed the
        // left/right count difference back as a small steering trim.
        int32_t bal = (dl - dr) * CREEP_BAL_GAIN;
        hw_motors_set(SPEED_CREEP - bal, SPEED_CREEP + bal);
        telemetry_tick(TEL_TICK_CREEP, line, sat16(progress), sat16(bal),
                       sat16(SPEED_CREEP - bal), sat16(SPEED_CREEP + bal));
    }

    // The nudge ran out of room or time without seeing tape: this
    // recovery has honestly failed. Stopped robot, caller faults.
    drive_stop();
    return DRIVE_LOST;
}

// Shared tail of both refind paths: log the find, then settle-verify the
// stop position. A verify that comes up empty demotes the find to a
// BT_FAIL — better an honest fault than a resume onto bare floor.
static drive_status_t bt_found_finish(uint32_t t0, uint16_t consumed)
{
    telemetry_event(EV_BT_FOUND, (int16_t)(hw_millis() - t0),
                    (int16_t)consumed);
    drive_status_t v = bt_settle_verify();
    if (v == DRIVE_LOST) {
        telemetry_event(EV_BT_FAIL, (int16_t)(hw_millis() - t0),
                        (int16_t)consumed);
    }
    return v;
}

static drive_status_t backtrack_recover(void)
{
    hw_motors_stop();
    sleep_ms(BT_PREROLL_SETTLE_MS);     // kill momentum before measuring
    telemetry_event(EV_BT_START, (int16_t)bt_count, 0);

    int32_t  tl = hw_encoder_left();
    int32_t  tr = hw_encoder_right();
    uint32_t t0 = hw_millis();
    uint16_t remaining = bt_count, consumed = 0;
    uint16_t idx = bt_head;             // one past the newest entry
    uint16_t line[5];
    int      streak = 0;                // consecutive tape-seen ticks

    tick_begin();
    while (remaining > 0) {
        tick_wait(CONTROL_PERIOD_US * BACKTRACK_TICK_X);
        if (hw_buttons_any()) {
            drive_stop();
            telemetry_event(EV_ABORT, TEL_TICK_BT, 0);
            return DRIVE_ABORT;
        }
        if (hw_millis() - t0 > BACKTRACK_TIMEOUT_MS) { break; }

        idx = (uint16_t)((idx + BT_RING - 1) % BT_RING);
        tl -= bt_dl[idx];
        tr -= bt_dr[idx];
        remaining--;
        consumed++;

        streak = bt_step(tl, tr, line) ? streak + 1 : 0;
        if (streak >= BT_FOUND_CONFIRM_TICKS) {
            return bt_found_finish(t0, consumed);
        }
    }

    // History spent (or timed out) with the target still retreating ahead
    // of the robot — drain the residual chase briefly; the tape may be
    // one stiction-lag short of the bar. Why a drain exists at all: the
    // P-chase always trails its target (stiction holds a wheel until the
    // error grows enough duty to break it), so when the last breadcrumb
    // is consumed the TARGET is final but the chassis may still be shy of
    // it — and the segment-start tape could be sitting in exactly that
    // last unclosed gap. Why 500 ms: the gap is at most a few mm and the
    // chase authority (BACKTRACK_KP capped at BACKTRACK_SPEED) closes
    // that in a few hundred ms — a residual still open after half a
    // second is not stiction lag but a wall or a lifted wheel, and those
    // belong to the BT_FAIL path below, not to more blind reversing. The
    // master timeout (t0/BACKTRACK_TIMEOUT_MS) and the any-button stop
    // hold through the drain like everywhere else.
    uint32_t drain0 = hw_millis();
    while (hw_millis() - t0 <= BACKTRACK_TIMEOUT_MS &&
           hw_millis() - drain0 < 500) {
        tick_wait(CONTROL_PERIOD_US * BACKTRACK_TICK_X);
        if (hw_buttons_any()) {
            drive_stop();
            telemetry_event(EV_ABORT, TEL_TICK_BT, 0);
            return DRIVE_ABORT;
        }
        streak = bt_step(tl, tr, line) ? streak + 1 : 0;
        if (streak >= BT_FOUND_CONFIRM_TICKS) {
            return bt_found_finish(t0, consumed);
        }
    }

    hw_motors_stop();
    telemetry_event(EV_BT_FAIL, (int16_t)(hw_millis() - t0), (int16_t)consumed);
    return DRIVE_LOST;
}

// ------------------------------------------------------ loss state machine
// Deciding whether the line is lost — and which KIND of lost — takes real
// state: two debounced conditions, a post-resume grace distance, and a
// decaying recovery budget. Giving that state a name and three small
// functions keeps the follow loop below readable as a narrative instead
// of a thicket of flags. Pure bookkeeping: nothing here touches a motor.
typedef enum {
    LOSS_NONE,        // line in hand (or nothing confirmed yet): keep driving
    LOSS_LINE_ENDED,  // hard loss while driving straight: an honest dead end
    LOSS_SWERVED,     // the robot left the tape; the tape did not end: recover
} loss_verdict_t;

typedef struct {
    bool     aw, cl;              // debounce latches: all-white / center-lost
    uint32_t aw_since, cl_since;  // when each condition began
    int      recoveries;          // budget used; decays with clean travel
    int32_t  resume_enc;          // grace baseline — a FRESH read at resume
    int32_t  decay_enc;           // clean-travel meter for the budget decay
    int32_t  ontape_enc;          // DEPARTURE POINT: last enc where the
                                  // center still held the tape — the resume
                                  // protocol suppresses junction detection
                                  // until the bar is back past it
} loss_monitor_t;

static void loss_monitor_init(loss_monitor_t *m, int32_t enc_now)
{
    m->aw = m->cl = false;
    m->aw_since = m->cl_since = 0;
    m->recoveries = 0;
    // Both baselines start at the segment start, which makes the grace
    // distance (RESUME_GRACE_MM, 15 mm) expire before the detector is
    // even armed (BLIND_MM, 30 mm): the gate only ever delays anything
    // AFTER a resume.
    m->resume_enc = enc_now;
    m->decay_enc  = enc_now;
    m->ontape_enc = enc_now;
}

static void loss_monitor_note_resume(loss_monitor_t *m, int32_t enc_now)
{
    m->aw = m->cl = false;
    m->resume_enc = enc_now;      // the grace distance restarts here...
    m->decay_enc  = enc_now;      // ...and so does the clean-travel meter
    m->ontape_enc = enc_now;      // ...and the departure bookkeeping
}

// One tick of loss bookkeeping, then the verdict for this instant.
// `armed` is the caller's blind-window flag: inside the blind window
// there is no such thing as a loss verdict.
static loss_verdict_t loss_monitor_tick(loss_monitor_t *m,
                                        const uint16_t line[5],
                                        uint32_t now, int32_t enc_now,
                                        bool armed)
{
    // Budget decay: every RECOVERY_DECAY_MM of travel with the line in
    // hand earns one recovery credit back. Without it, four one-off (and
    // successful) recoveries spread over a long segment would fault the
    // same as a tight lawnmower loop — the budget would be measuring
    // lifetime bad luck, not repeated failure. The decay is paid for in
    // distance, which is what bounds it: a genuine lawnmower loop loses
    // the line again within a few centimetres and earns nothing back.
    if (m->recoveries > 0 &&
        enc_now - m->decay_enc >= MM_TO_COUNTS(RECOVERY_DECAY_MM)) {
        m->recoveries--;
        m->decay_enc = enc_now;   // each credit is paid for separately
    }

    bool all_white = true;
    for (int i = 0; i < 5; i++) {
        if (line[i] >= LINE_LOST_THRESH) { all_white = false; break; }
    }
    if (all_white) { if (!m->aw) { m->aw = true; m->aw_since = now; } }
    else           { m->aw = false; }

    bool center_lost = line[1] < CENTER_LOST_THRESH &&
                       line[2] < CENTER_LOST_THRESH &&
                       line[3] < CENTER_LOST_THRESH;
    if (center_lost) { if (!m->cl) { m->cl = true; m->cl_since = now; } }
    else             { m->cl = false; }

    // Departure bookkeeping for the resume protocol: remember the last
    // position where the center still HELD the tape. Everything the
    // robot drives past this point is off-road arc, not swept tape —
    // which is exactly why a post-recovery resume may not trust any
    // junction evidence until the bar is back past here (the resume
    // protocol in drive_until_junction carries the full argument).
    if (!center_lost) { m->ontape_enc = enc_now; }

    // The grace gate: after a resume, the hard-loss verdict stays
    // holstered until the robot has covered RESUME_GRACE_MM. The resume
    // re-arms with a wiped |p| history, so a white read in the first
    // ticks would reach the jury below with rmax = 0 — an automatic
    // (and false) "honest dead end".
    bool grace_over = enc_now - m->resume_enc >= MM_TO_COUNTS(RESUME_GRACE_MM);
    bool lost_hard  = armed && grace_over && m->aw &&
                      now - m->aw_since >= LOST_CONFIRM_MS;
    bool stuck_slam = armed && m->cl && now - m->cl_since >= CENTER_LOST_MAX_MS;

    if (!lost_hard && !stuck_slam) { return LOSS_NONE; }

    // The verdict. Recent measured |p| is the jury (see recent_p above):
    // a line that ends while the follower was driving straight is a dead
    // end; a line that "ends" while it was sawing at the wheel is a robot
    // leaving the road. A center-lost slam that never resolved
    // (stuck_slam) is never a dead end — the outer sensors may still be
    // seeing tape.
    if (lost_hard && recent_abs_max() <= DEADEND_P_MAX) {
        return LOSS_LINE_ENDED;
    }
    return LOSS_SWERVED;
}

// ------------------------------------------------------- junction approach
// The phase that drives one maze segment. Entry: the robot sits on or
// just past a junction, pointing down the new segment, with the motors
// stopped or rolling — it commands its own speed from the first tick.
// Exit: DRIVE_OK with *before holding the junction evidence and the
// motors STILL RUNNING (drive.h has the caller contract, including the
// one stationary exception); DRIVE_ABORT, DRIVE_TIMEOUT, or DRIVE_LOST
// with the robot stopped. Recoveries happen inside and are invisible to
// the caller.
//
// Three mechanisms share the one tick loop: the post-turn blind window,
// the edge latch that catches a branch flashing past in a single sample,
// and the loss machine with its backtrack and resume protocol.
drive_status_t drive_until_junction(sensor_snapshot_t *before,
                                    int32_t base, uint32_t timeout_ms)
{
    // Why the blind window: right after a turn the sensor bar may still
    // be hovering over pieces of the junction just handled. Triggering
    // on those would classify the same junction twice. It is measured in
    // encoder DISTANCE (BLIND_MM), not time, so the same window works at
    // explore and replay speed.
    const int32_t blind_counts = MM_TO_COUNTS(BLIND_MM);

    uint32_t t0 = hw_millis();
    uint16_t line[5];
    uint16_t prev0 = 0, prev4 = 0;   // previous outer readings (edge latch)

    follow_reset();
    telemetry_event(EV_FOLLOW_START, sat16(base), 0);

    int32_t enc0 = (hw_encoder_left() + hw_encoder_right()) / 2;
    bool    was_armed = false;

    // Where the ORIGINAL blind window ends. Remembered separately because
    // every recovery resume rewrites enc0 (back-dated to keep the follower
    // armed) — and the resume protocol needs the original arming point as
    // the floor of its detect-suppression bound.
    const int32_t arm_enc = enc0 + blind_counts;
    // Post-resume junction-detect suppression. Inert until a recovery
    // resume raises it (the resume protocol below computes the bound and
    // argues why it can never mask a real junction).
    int32_t suppress_until  = enc0;
    bool    was_suppressed  = false;

    loss_monitor_t loss;
    loss_monitor_init(&loss, enc0);

    tick_begin();
    for (;;) {
        tick_wait(CONTROL_PERIOD_US);

        if (hw_buttons_any()) {
            drive_stop();
            telemetry_event(EV_ABORT, TEL_TICK_FOLLOW, 0);
            return DRIVE_ABORT;
        }
        uint32_t now = hw_millis();
        if (now - t0 > timeout_ms) {
            drive_stop();
            telemetry_event(EV_TIMEOUT, TEL_TICK_FOLLOW, 0);
            return DRIVE_TIMEOUT;
        }

        hw_line_read_calibrated(line);
        bt_record();                     // breadcrumbs, one per tick

        int32_t enc_now = (hw_encoder_left() + hw_encoder_right()) / 2;
        int32_t dist = enc_now - enc0;
        bool armed = dist >= blind_counts;
        if (armed && !was_armed) {
            was_armed = true;
            have_p = false;   // estimate switches center-3 → all-5: no D spike
            // The edge latch below ORs each outer sensor with its previous
            // sample — but the "previous sample" was taken INSIDE the blind
            // window, where the outers may still be over remnants of the
            // junction just handled. Blind exists precisely to ignore
            // those readings; letting one leak through the latch on the
            // first armed tick would shrink the blind margin by a tick and
            // could re-detect the old junction. Start the latch clean.
            prev0 = prev4 = 0;
            telemetry_event(EV_BLIND_END, sat16(dist), 0);
        }

        // Post-resume suppression window (set by the resume protocol
        // below): after a recovery the bar may be re-covering ground this
        // segment already cleared, so junction detection stays holstered
        // until it is back past the departure point. Same one-tick leak
        // as the arming edge above: flush the latch when the window
        // lifts, so a sample taken inside it can't trigger a detect one
        // tick past the boundary.
        bool suppressed = enc_now < suppress_until;
        if (was_suppressed && !suppressed) { prev0 = prev4 = 0; }
        was_suppressed = suppressed;

        // EDGE LATCH: a branch line is ~19 mm of tape passing under an
        // outer sensor. At replay speed that can be just 1–2 samples. By
        // OR-ing each outer sensor with its previous sample, a branch that
        // flashed by for a single read still makes it into the snapshot.
        uint16_t l0 = line[0] > prev0 ? line[0] : prev0;
        uint16_t l4 = line[4] > prev4 ? line[4] : prev4;

        if (armed && !suppressed &&
            (l0 >= JCT_OUTER_THRESH || l4 >= JCT_OUTER_THRESH)) {
            for (int i = 0; i < 5; i++) { before->s[i] = line[i]; }
            before->s[0] = l0;
            before->s[4] = l4;
            telemetry_event(EV_JCT_DETECT, (int16_t)l0, (int16_t)l4);
            // One-shot 'J' snapshot: the exact `before` evidence the
            // classifier will be handed, recorded NOW. It must be logged
            // here and not later: the caller reuses this same struct as
            // the creep's sweep accumulator, so by CLASSIFY time it no
            // longer holds what the detect saw. left/right carry the
            // follow command the motors are still executing.
            telemetry_tick(TEL_TICK_JCT_SNAP, before->s,
                           (int16_t)l0, (int16_t)l4,
                           last_cmd_left, last_cmd_right);
            return DRIVE_OK;            // motors still running — by design
        }

        // ---- line-loss bookkeeping + verdict ----------------------------
        loss_verdict_t verdict =
            loss_monitor_tick(&loss, line, now, enc_now, armed);

        if (verdict == LOSS_LINE_ENDED) {
            // Driving straight and the line simply stopped: an honest
            // dead end. `before` gets the RAW reading, unlike the detect
            // path above which hands over the edge-latched outers: a
            // latched outer sample from an earlier tick would put branch
            // evidence into the one snapshot whose meaning is that there
            // is none.
            for (int i = 0; i < 5; i++) { before->s[i] = line[i]; }
            telemetry_event(EV_DEADEND, sat16(recent_abs_max()), 0);
            return DRIVE_OK;            // classifier will see: nothing
        }

        if (verdict == LOSS_SWERVED) {
            // Mid-correction: the robot left the tape, the tape didn't
            // end. Retrace the breadcrumbs — within budget: if the
            // follower keeps falling off, recovery must not become an
            // infinite lawnmower pattern.
            telemetry_event(EV_LOSS, sat16(recent_abs_max()),
                            (int16_t)(loss.cl ? now - loss.cl_since : 0));
            if (++loss.recoveries > RECOVERIES_MAX) {
                drive_stop();
                return DRIVE_LOST;
            }
            uint32_t bt_begin = hw_millis();
            drive_status_t r = backtrack_recover();
            if (r != DRIVE_OK) { return r; }

            // ---- the resume protocol ------------------------------------
            // Back on the tape, stopped, pointing roughly down the line —
            // but WHERE on the tape? The retrace stops at the first
            // confirmed refind, which can be anywhere along the recorded
            // arc: usually just behind the swerve, but after a bad kink
            // it can be all the way back over the junction this segment
            // STARTED from — already classified, recorded, and turned at.
            // Resume ARMED for steering and for the loss machinery, but
            // junction detection stays HOLSTERED until the bar is back
            // past everything this segment already cleared:
            //
            //     suppress_until = max(arm_enc, departure point)
            //
            // (departure point = the last tick the center still held the
            // tape — the loss monitor's ontape_enc). A detect inside that
            // stretch could only re-announce evidence already dealt with:
            // the caller would classify the same junction AGAIN and
            // record a phantom move, and path_raw would stop describing
            // the maze. Why the bound can't mask a REAL next junction:
            // everything up to arm_enc is the original blind window,
            // where the >=150 mm junction-spacing minimum the maze is
            // built to already guarantees no next junction lives (worst
            // case spends ARRIVAL_BRAKE_MM 20 + CREEP_MM 45 + BLIND_MM 30
            // = 95 mm of the 150); everything from arm_enc to the
            // departure point was swept ON the tape with detection armed
            // and SILENT — the maze itself testifying there is no
            // junction there. The next branch therefore lies strictly
            // beyond the departure point, where detection re-arms — and
            // the 55 mm of unspent spacing absorbs the few millimetres
            // of closed-loop retrace slack many times over. The bound is
            // recorded state, not a new knob. (Computed BEFORE
            // note_resume resets the bookkeeping below.)
            suppress_until = loss.ontape_enc > arm_enc
                           ? loss.ontape_enc : arm_enc;
            // Recovery time must not eat the segment watchdog: push the
            // deadline out by however long the retrace took.
            t0 += hw_millis() - bt_begin;
            follow_reset();
            // The grace baseline must be a FRESH encoder read: enc0 is
            // about to be back-dated by blind_counts (that's how the
            // resume arms instantly), so measuring the grace from IT
            // would hand the gate 30 pre-paid millimetres and silently
            // no-op it.
            int32_t enc_resume = (hw_encoder_left() + hw_encoder_right()) / 2;
            loss_monitor_note_resume(&loss, enc_resume);
            enc0 = enc_resume - blind_counts;
            // was_armed = TRUE, not false: if it lagged behind `armed`,
            // the arming block would run once more and log a BLIND_END
            // the robot never earned — a resume is not a blind-window
            // exit and must not dress up as one in the flight log.
            // EV_RESUME is the honest marker.
            was_armed = true;
            prev0 = prev4 = 0;
            // The second EV_RESUME field is the suppressed span still
            // ahead of this resume, in mm (0 = the refind landed at or
            // past the bound computed above, max(arm_enc, ontape_enc) —
            // the LATER of the arm point and the departure point, so a
            // line lost behind the arm point makes that bound the arm
            // point, not the departure point; either way nothing is left
            // to suppress). The suppression itself is invisible in a
            // telemetry dump — detection staying quiet looks exactly like
            // open corridor — so this number is the only way to see after
            // the fact how much re-covered tape the resume holstered
            // detection for. Millimetres because that is the unit all the
            // spacing arithmetic (BLIND_MM 30 + CREEP_MM 45 against the
            // 150 mm minimum) is done in; encoder counts would demand a
            // counts_per_mm conversion by hand in the middle of triage.
            telemetry_event(EV_RESUME, (int16_t)loss.recoveries,
                            sat16((suppress_until > enc_resume
                                   ? suppress_until - enc_resume : 0)
                                  * 100 / ENC_COUNTS_PER_MM_X100));
            tick_begin();
            continue;
        }

        follow_tick(line, base, !armed);
        prev0 = line[0];
        prev4 = line[4];
    }
}

// ---------------------------------------------------------- arrival brake
// The speed-shedding window between the junction detect and the creep.
// Entry: drive_until_junction has just returned DRIVE_OK, the motors are
// still running at `base`, and the bar is somewhere over junction tape.
// Exit: DRIVE_OK with the motors still running at roughly SPEED_ARRIVAL
// (the last straightness trim is still applied) and *sweep extended with
// everything the bar saw on the way; or DRIVE_ABORT with the robot
// stopped. It cannot return DRIVE_TIMEOUT or DRIVE_LOST — it measures,
// it never diagnoses. Caller contract in drive.h.
//
// The one design fact that belongs next to the code: braking here is
// nothing but COMMANDING the lower speed. These PHASE/ENABLE drivers
// spend the off-fraction of every 20.8 kHz PWM cycle with both bottom
// H-bridge transistors closed — motor terminals tied together, which is
// braking rather than coasting — so a command below the chassis's
// current pace drags it toward SPEED_ARRIVAL in proportion to the
// excess, and can never drag it below. Self-limiting: this loop needs no
// speed measurement, no ramp table, and has no way to stall the robot
// dead short of a junction it still has to cross.
drive_status_t arrival_brake(sensor_snapshot_t *sweep, int32_t base)
{
    // The window exists to shed EXCESS speed. If the straights already
    // ran at (or under) the trusted arrival pace there is nothing to
    // shed: return with the motors untouched. At the shipped SPEED_REPLAY
    // the two are equal, so the arrival path is exactly what it was
    // before this window existed — only a base stepped past
    // SPEED_ARRIVAL ever feels this function run.
    if (base <= SPEED_ARRIVAL) { return DRIVE_OK; }

    // Brake FIRST, then loop. The whole point of an arrival-triggered
    // window is that no more hot millimetres pass between the detect
    // edge and the speed drop — not even one control tick's worth.
    hw_motors_set(SPEED_ARRIVAL, SPEED_ARRIVAL);

    // The window is bounded by DISTANCE, not duration, because its real
    // cost is position: creep_to_center measures its CREEP_MM from
    // wherever this loop ends, so every millimetre rolled here shifts
    // the deciding at_center read that much further past the detect
    // edge — and the goal patch has only 30 mm to give past the creep
    // (that budget arithmetic is written up on ARRIVAL_BRAKE_MM in
    // include/tuning.h). Encoders cap the cost no matter how the shed
    // itself goes. ARRIVAL_BRAKE_MS survives as the TIMEOUT: if the
    // encoders stop counting (wheel jammed, chassis blocked) the clock
    // closes the window instead, so this loop cannot hang — and a
    // stalled chassis is creep_to_center's watchdog's diagnosis to make,
    // not this function's.
    const int32_t target = MM_TO_COUNTS(ARRIVAL_BRAKE_MM);

    int32_t  l0 = hw_encoder_left();
    int32_t  r0 = hw_encoder_right();
    uint32_t t0 = hw_millis();
    uint16_t line[5];

    tick_begin();
    for (;;) {
        tick_wait(CONTROL_PERIOD_US);
        if (hw_buttons_any()) {
            drive_stop();
            telemetry_event(EV_ABORT, TEL_TICK_CREEP, 0);
            return DRIVE_ABORT;
        }

        int32_t dl = hw_encoder_left() - l0;
        int32_t dr = hw_encoder_right() - r0;
        if ((dl + dr) / 2 >= target)              { break; }  // window done
        if (hw_millis() - t0 >= ARRIVAL_BRAKE_MS) { break; }  // stall bail-out

        // Steer straight on ENCODERS, not the line: the bar is somewhere
        // over the junction it just detected, and a weighted average over
        // branch tape is geometry, not guidance. Same servo, same gain as
        // the creep.
        int32_t bal = (dl - dr) * CREEP_BAL_GAIN;
        hw_motors_set(SPEED_ARRIVAL - bal, SPEED_ARRIVAL + bal);

        // The bar crosses junction tape for the whole window — that is
        // classifier evidence, and the creep's sweep must not open with
        // a blind spot. Latch maxima exactly as creep_to_center does.
        hw_line_read_calibrated(line);
        for (int i = 0; i < 5; i++) {
            if (line[i] > sweep->s[i]) { sweep->s[i] = line[i]; }
        }
        // Same C row as the creep and the nudge. These ticks sit between
        // the 'J' snapshot and EV_CREEP_START, wearing SPEED_ARRIVAL-
        // sized duty columns: that is how the window shows up in a
        // telemetry dump, and their absence is how the no-op at a trusted
        // base shows up.
        telemetry_tick(TEL_TICK_CREEP, line, sat16((dl + dr) / 2),
                       sat16(bal), sat16(SPEED_ARRIVAL - bal),
                       sat16(SPEED_ARRIVAL + bal));
    }

    // Window closed, motors still commanded around SPEED_ARRIVAL —
    // creep_to_center takes over from a rolling start, the same handoff
    // it gets straight from the detect at an unstepped base. Buttons were
    // polled every tick above; the distance bound ends the window, the
    // time bound refuses to let it hang, and both close it with DRIVE_OK.
    return DRIVE_OK;
}

// ------------------------------------------------------------------- creep
// Entry: a junction has just been detected (and, at a stepped base, the
// arrival window has already shed the excess), the motors are running,
// and *sweep holds the evidence gathered so far. Exit: DRIVE_OK with the
// robot STOPPED and settled with its axle on the junction center — the
// state a turn expects — *sweep extended and *at_center written; or
// DRIVE_TIMEOUT / DRIVE_ABORT, robot stopped, at_center untouched.
//
// Why creep at all: the sensor bar rides ahead of the wheel axle. When
// the bar detects a junction, the axle — the point the robot pivots
// around — is still CREEP_MM short of it. Turning now would cut the
// corner and miss the new line. So roll forward a measured distance,
// sweeping the sensors across the junction (free evidence for the
// classifier), and only then stop and decide.
drive_status_t creep_to_center(sensor_snapshot_t *sweep,
                               sensor_snapshot_t *at_center)
{
    const int32_t target = MM_TO_COUNTS(CREEP_MM);

    int32_t l0 = hw_encoder_left();
    int32_t r0 = hw_encoder_right();
    uint32_t t0 = hw_millis();
    uint16_t line[5];

    telemetry_event(EV_CREEP_START, 0, 0);

    for (;;) {
        if (hw_buttons_any()) {
            drive_stop();
            telemetry_event(EV_ABORT, TEL_TICK_CREEP, 0);
            return DRIVE_ABORT;
        }
        if (hw_millis() - t0 > CREEP_TIMEOUT_MS) {
            drive_stop();
            telemetry_event(EV_TIMEOUT, TEL_TICK_CREEP, 0);
            return DRIVE_TIMEOUT;      // wheel stalled or encoder sign wrong
        }

        int32_t dl = hw_encoder_left() - l0;
        int32_t dr = hw_encoder_right() - r0;
        if ((dl + dr) / 2 >= target) { break; }

        // Keep it straight: feed the left/right count difference back as a
        // small steering correction. No line data in this loop — the bar
        // is over the junction, where a weighted average measures tape
        // geometry, not where the robot ought to point.
        int32_t bal = (dl - dr) * CREEP_BAL_GAIN;
        hw_motors_set(SPEED_CREEP - bal, SPEED_CREEP + bal);

        // The sweep IS the measurement: latch every sensor's darkest
        // moment while the bar crosses the junction.
        hw_line_read_calibrated(line);
        for (int i = 0; i < 5; i++) {
            if (line[i] > sweep->s[i]) { sweep->s[i] = line[i]; }
        }
        telemetry_tick(TEL_TICK_CREEP, line, sat16((dl + dr) / 2),
                       sat16(bal), sat16(SPEED_CREEP - bal),
                       sat16(SPEED_CREEP + bal));
    }

    hw_motors_stop();
    sleep_ms(CREEP_SETTLE_MS);          // outwait the rock, then decide
    hw_line_read_calibrated(at_center->s);
    telemetry_event(EV_CREEP_END, 0, 0);
    // One-shot 'A' snapshot: the at_center read itself — the other half
    // of the classifier's evidence. The robot is stopped and settled, so
    // every non-sensor column is honestly zero. Triage note: an honest
    // dead end skips JCT_DETECT (its path fills `before` in the follow
    // loop and returns without one) but still creeps through here — so a
    // DEADEND classification has an 'A' row and NO 'J' row. That gap is
    // by design, not a dropped record.
    telemetry_tick(TEL_TICK_CENTER_SNAP, at_center->s, 0, 0, 0, 0);
    return DRIVE_OK;
}

// ------------------------------------------------------------- gyro turns
// Entry: the robot is stopped with its axle on the junction center, and
// hw_imu_init() has succeeded. Exit: DRIVE_OK with the robot stopped,
// having rotated by the requested angle and held it; DRIVE_TIMEOUT or
// DRIVE_ABORT, also stopped. No line data is read here.
//
// PD on the integrated gyro angle (TURN_KP, TURN_KD, settling on
// TURN_TOL_DEG for TURN_SETTLE_MS — all in include/tuning.h). Gyro
// rather than encoders because encoders measure WHEEL rotation: on a
// dusty poster board the wheels slip during in-place turns and the error
// accumulates junction after junction. The gyro measures the BODY's
// rotation directly — slip-proof. And the turn is RELATIVE (target =
// current + delta), so drift while the robot sits still between
// junctions never enters the picture.
static const uint16_t no_line[5];       // zero-filled: a turn's telemetry
                                        // rows carry no sensor data

static drive_status_t turn_relative(float delta_deg)
{
    hw_imu_update();
    float target = hw_imu_angle_deg() + delta_deg;

    uint32_t t0 = hw_millis();
    uint32_t last_far = t0;
    uint32_t last_rec = 0;

    telemetry_event(EV_TURN_START, sat16((int32_t)(delta_deg * 10.0f)), 0);

    for (;;) {
        if (hw_buttons_any()) {
            drive_stop();
            telemetry_event(EV_ABORT, TEL_TICK_TURN, 0);
            return DRIVE_ABORT;
        }
        uint32_t now = hw_millis();
        if (now - t0 > TURN_TIMEOUT_MS) {
            drive_stop();
            telemetry_event(EV_TIMEOUT, TEL_TICK_TURN, 0);
            return DRIVE_TIMEOUT;
        }

        hw_imu_update();
        float err = target - hw_imu_angle_deg();

        // "Settled" = inside ±TURN_TOL_DEG continuously for TURN_SETTLE_MS.
        // A single in-tolerance sample isn't enough — the robot swings
        // through the target with momentum, and what the next phase needs
        // is a chassis STOPPED there.
        if (fabsf(err) > TURN_TOL_DEG) {
            last_far = now;
        } else if (now - last_far >= TURN_SETTLE_MS) {
            break;
        }

        // The D term reads the gyro's RATE directly rather than
        // differencing the angle, so this loop needs no fixed period the
        // way the line follower does.
        int32_t speed = (int32_t)(err * TURN_KP - hw_imu_rate_dps() * TURN_KD);
        if (speed >  SPEED_TURN_MAX) { speed =  SPEED_TURN_MAX; }
        if (speed < -SPEED_TURN_MAX) { speed = -SPEED_TURN_MAX; }

        // +err → need CCW (left): left wheel back, right wheel forward.
        hw_motors_set(-speed, speed);

        if (now - last_rec >= 16) {     // ~60 Hz is plenty for a picture
            last_rec = now;
            telemetry_tick(TEL_TICK_TURN, no_line,
                           sat16((int32_t)(err * 10.0f)), sat16(speed),
                           sat16(-speed), sat16(speed));
        }
    }

    hw_motors_stop();
    hw_imu_update();
    telemetry_event(EV_TURN_END,
                    sat16((int32_t)((target - hw_imu_angle_deg()) * 10.0f)),
                    (int16_t)(hw_millis() - t0));
    return DRIVE_OK;
}

drive_status_t turn_left_90(void)     { return turn_relative(+90.0f); }
drive_status_t turn_right_90(void)    { return turn_relative(-90.0f); }
drive_status_t turn_around_180(void)  { return turn_relative(+180.0f); }
