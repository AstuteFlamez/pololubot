// main.c — application shell: mode machine, button menu, run loops.
//
// This firmware is a maze SOLVER: it calibrates, explores with the
// left-hand rule (follow → creep → classify → decide → record → move),
// collapses the recorded detours into the shortest route, and replays
// that route junction-by-junction to the goal. The decision logic lives
// in logic/maze_logic.c — pure, host-tested C — and is called from the
// run loops below; this file only orchestrates I/O, sequencing, and the
// screens.
//
// Buttons, in every mode:
//   C short tap   → next menu item
//   C long press  → GO (run the selected mode)     A → also GO
//   ANY button during a run → motors off, back to the menu. Always.

#include <stdio.h>
#include "pico/stdlib.h"

#include "tuning.h"
#include "maze_logic.h"

#include "hw_millis.h"
#include "hw_motors.h"
#include "hw_encoders.h"
#include "hw_imu.h"
#include "hw_line.h"
#include "hw_battery.h"
#include "hw_buttons.h"
#include "telemetry.h"
#include "ui.h"
#include "drive.h"
#include "feedback.h"
#include "lap_timer.h"

// USE_MY_CLASSIFIER (tuning.h) selects which junction classifier runs.
// The reference classifier stays compiled in as a permanent fallback:
// comment the flag out and everything below is unaffected, because the
// flag swaps the classifier ONLY, never the decision layer. The host
// simulator exercises both classifiers on every run.
#ifdef USE_MY_CLASSIFIER
#define CLASSIFY classify_junction
#else
#define CLASSIFY classify_junction_ref
#endif

static bool imu_ok;
static bool gyro_calibrated;

// The two routes: path_raw is what the explorer actually drove, detours
// and all; path_solved is its simplified twin — the one REPLAY runs.
static path_t path_raw;
static path_t path_solved;

// ---------------------------------------------------------------- helpers

static void show_battery_line(int row)
{
    unsigned mv = hw_battery_mv();
    ui_text(0, row, "bat %u.%02uV %-4s", mv / 1000, (mv % 1000) / 10,
            mv < BATT_REFUSE_MV ? "DEAD" : mv < BATT_WARN_MV ? "LOW" : "ok");
}

// Wait (after a fault or an info screen) for a fresh button press.
static void wait_for_button(void)
{
    hw_buttons_wait_release();
    while (!hw_buttons_any()) { sleep_ms(10); }
    hw_buttons_wait_release();
}

// Stop, log, show the reason, wait for acknowledgement.
//
// The fault string reaches the OLED and nowhere else: EV_FAULT carries no
// code (a = b = 0), so triage means matching what the screen said against
// the shape the ring recorded. Every string this firmware can show, and the
// dump signature that goes with it:
//
//   FIRST, the preflight phantom. The three preflight refusals below call
//   fault() before either runner resets the ring, so a refusal appends a
//   lone FAULT row onto whatever the PREVIOUS run left behind — a run whose
//   own story may have ended cleanly. A trailing FAULT with no fresh
//   RUN_START opening a new run is a refusal at the menu, not a crash mid
//   run, and the last RUN_START in the dump may not even be the mode that
//   got refused. Do not debug the previous run for a crash it never had.
//
//   "battery dead"        preflight. Cross-check batt_mv in the dump-facts
//                         header: at or below BATT_REFUSE_MV earns it.
//   "run CALIBRATE 1st"   preflight. cal_min/cal_max spans near zero mean
//                         no sweep has run since boot.
//   "no gyro cal"         preflight. Nothing in the ring measures gyro bias;
//                         the screen is the only witness.
//   "lost: 10s no jct"    RUN_START, then RUN_WATCHDOG_MS of F rows with no
//                         JCT_DETECT, ending TIMEOUT [phase=F]. Drove off the
//                         maze, or a junction never read as one.
//   "line LOST"           recovery exhausted, in one of three shapes.
//                         Budget spent: a final LOSS with no BT_START after
//                         it. Retrace failed: LOSS, BT_START, K rows,
//                         BT_FAIL. Demoted find: BT_FOUND (possibly with
//                         nudge C rows) then BT_FAIL — what it refound did
//                         not survive the settle re-check.
//   "creep stalled"       CREEP_START, then C rows whose a (progress in
//                         counts) stops climbing, then TIMEOUT [phase=C].
//                         Jammed wheel, blocked chassis, or a flipped
//                         encoder sign.
//   "classify: NONE"      CLASSIFY [NONE]. The J/A snapshot pair just above
//                         it is the exact evidence the classifier refused.
//                         A dead-end-shaped arrival has an A row and no J.
//   "path overflow"       the 65th recorded move (PATH_MAX_MOVES is 64):
//                         a long explore ending CLASSIFY then FAULT with no
//                         TURN_START between them.
//   "illegal move"        CLASSIFY reporting a confident junction, then
//                         FAULT with no TURN_START — the move byte itself
//                         was rot. Same immediate shape as path overflow,
//                         since both checks run inside the arrival verdict,
//                         before the think pause. Tell them apart by the
//                         junction count: overflow needs 64 moves behind it,
//                         an illegal move can fire at junction one.
//   "turn timeout"        TURN_START with no TURN_END, T rows whose angle
//                         error (a, deg x10) stalls short of zero, then
//                         TIMEOUT [phase=T]. Check batt_mv for sag before
//                         reaching for TURN_KP.
//   "replay:not goal"     path spent, but the last CLASSIFY reads a
//                         confident non-GOAL junction. Wrong maze, drifted
//                         path, or a miscount upstream.
//   "goal too early"      CLASSIFY [GOAL] with moves still remaining.
//   "replay:dead end"     DEADEND (an honest line end, so no J row), creep,
//                         CLASSIFY [DEAD_END]. The solved path steered into
//                         a corridor end it should never have reached.
static void fault(const char *msg)
{
    drive_stop();
    telemetry_event(EV_FAULT, 0, 0);
    on_fault(msg);
    wait_for_button();
}

// Everything that must be true before wheels are allowed to move.
static bool preflight(void)
{
    if (hw_battery_critical()) {
        fault("battery dead");
        return false;
    }
    if (!hw_line_cal_ok()) {
        fault("run CALIBRATE 1st");
        return false;
    }
    if (!imu_ok || !gyro_calibrated) {
        fault("no gyro cal");
        return false;
    }
    return true;
}

static void countdown(void)
{
    hw_buttons_wait_release();     // the GO press must not become a STOP
    for (int n = 3; n >= 1; n--) {
        on_countdown(n);
        sleep_ms(400);
    }
    ui_clear();
    ui_flush_now();
}

// OLED text for one arrival verdict. The judging itself is pure and
// host-tested (the arrival-verdict block in logic/maze_logic.c); only
// this mapping lives here, because a 16-column OLED string is I/O and
// logic/ owns no screens.
//
// ARRIVE_PROCEED and ARRIVE_SUCCESS are not faults and never reach here,
// but the switch must stay total: "illegal move" is the least-wrong text
// for a verdict code this layer cannot act on.
static const char *verdict_fault_text(arrival_verdict_t v)
{
    switch (v) {
    case ARRIVE_FAULT_CLASSIFY_NONE: return "classify: NONE";
    case ARRIVE_FAULT_NOT_GOAL:      return "replay:not goal";
    case ARRIVE_FAULT_GOAL_EARLY:    return "goal too early";
    case ARRIVE_FAULT_DEAD_END:      return "replay:dead end";
    case ARRIVE_FAULT_ILLEGAL_MOVE:
    default:                         return "illegal move";
    }
}

// Turn a decided move into motion. MOVE_STRAIGHT costs nothing (the
// follower is already pointed the right way); turns can fault. Both run
// loops screen the move byte through the arrival verdict first, so the
// default below is unreachable in practice — it is kept because a switch
// over a char must stay total, and reporting a timeout is the safest
// answer if a future caller skips that screening.
static drive_status_t execute_move(move_t m)
{
    switch (m) {
    case MOVE_STRAIGHT: return DRIVE_OK;
    case MOVE_LEFT:     return turn_left_90();
    case MOVE_RIGHT:    return turn_right_90();
    case MOVE_BACK:     return turn_around_180();
    default:            return DRIVE_TIMEOUT;
    }
}

// ------------------------------------------------------------------- DIAG
// Bench dashboard: live line sensors, calibration state, encoder counts,
// gyro angle and battery, redrawn until any button is pressed. Motors
// stay off, so flushing the screen inside the loop is safe.
static void run_diag(void)
{
    on_state_change(MODE_DIAG);
    hw_buttons_wait_release();

    uint16_t raw[5], cal[5];
    while (!hw_buttons_any()) {
        hw_line_read_raw(raw);
        hw_line_read_calibrated(cal);
        hw_imu_update();

        ui_clear();
        ui_text(0, 0, "DIAG  cal:%s", hw_line_cal_ok() ? "OK" : "--");
        ui_bars(0, 10, 22, cal, 1000);
        ui_text(9, 2, "raw mid");
        ui_text(9, 3, "%4u", raw[2]);
        ui_text(0, 4, "enc L%7d", (int)hw_encoder_left());
        ui_text(0, 5, "    R%7d", (int)hw_encoder_right());
        ui_text(0, 6, "gyro %6.1f%s", (double)hw_imu_angle_deg(),
                imu_ok ? "" : " !!");
        show_battery_line(7);
        ui_flush();                 // throttled to 10 Hz inside
        sleep_ms(20);
    }
    hw_buttons_wait_release();
}

// -------------------------------------------------------------- CALIBRATE
// Two calibrations, back to back:
//   1. Gyro bias — robot must sit STILL (that's why it's first, before
//      any spinning has the chassis rocking).
//   2. Line sensors — spin in place so the bar sweeps white→black→white,
//      teaching every sensor its own extremes. Place the robot ON the
//      line before starting.
static void run_calibrate(void)
{
    on_state_change(MODE_CALIBRATE);
    hw_buttons_wait_release();   // the GO hold must not abort the sweep

    ui_clear();
    ui_text_big(0, 0, "CAL: gyro");
    ui_text(0, 3, "hold still...");
    ui_flush_now();
    sleep_ms(400);                  // let the GO-press wobble die out
    if (imu_ok) {
        hw_imu_calibrate();
        gyro_calibrated = true;
    }

    ui_clear();
    ui_text_big(0, 0, "CAL: line");
    ui_text(0, 3, "sweeping...");
    ui_flush_now();

    hw_line_cal_reset();
    // Right 0.5 s, left 1.0 s, right 0.5 s — ends facing where it started.
    // These durations are the other half of SPEED_CALIBRATE (tuning.h):
    // an in-place spin sweeps angle = speed x time, and the sweep must
    // carry the whole bar across the line and back in each direction.
    // Turn that knob and these times must scale inversely with it, or
    // the outer sensors never reach the tape — which the span screen
    // after the sweep catches (a sensor that missed shows < CAL_MIN_SPAN).
    const struct { int32_t dir; uint32_t ms; } phase[3] =
        { { +1, 500 }, { -1, 1000 }, { +1, 500 } };
    for (int ph = 0; ph < 3; ph++) {
        uint32_t t0 = hw_millis();
        hw_motors_set(phase[ph].dir * SPEED_CALIBRATE,
                      -phase[ph].dir * SPEED_CALIBRATE);
        while (hw_millis() - t0 < phase[ph].ms) {
            if (hw_buttons_any()) { drive_stop(); return; }
            hw_line_cal_update();
        }
    }
    drive_stop();

    ui_clear();
    ui_text_big(0, 0, hw_line_cal_ok() ? "CAL OK" : "CAL BAD");
    ui_text(0, 2, "spans (min 300):");
    for (int i = 0; i < 5; i++) {
        int span = (int)hw_line_cal_max[i] - (int)hw_line_cal_min[i];
        if (span < 0) { span = 0; }
        ui_text(i * 3, 3, "%3d", span > 999 ? 999 : span);
    }
    if (!hw_line_cal_ok()) {
        ui_text(0, 5, "on the line? matte");
        ui_text(0, 6, "tape? try again.");
    }
    ui_flush_now();
    feedback_beep(hw_line_cal_ok() ? NOTE_C6 : NOTE_C4, 150);
    // Hold the screen until a button press. These five spans are the
    // evidence that the sweep worked (every span >= CAL_MIN_SPAN), and a
    // timed flash was gone before one number could be read, let alone
    // five.
    wait_for_button();
}

// ---------------------------------------------------------------- EXPLORE
// One junction per loop turn: follow → creep → classify → announce →
// decide → move. Returns the mode to enter next.
static run_mode_t run_explore(void)
{
    if (!preflight()) { return MODE_DIAG; }
    on_state_change(MODE_EXPLORE);
    countdown();

    telemetry_reset();
    telemetry_event(EV_RUN_START, MODE_EXPLORE, SPEED_EXPLORE);

    path_raw.len = 0;
    path_raw.overflow = false;
    // No lap clock here, on purpose: lap_timer records REPLAY runs only.
    // An explore that reached the goal would seed `best` with its
    // cautious stroll, and the next replay — faster than exploring by
    // construction — would always beat it, for a guaranteed "NEW!" and an
    // unearned new-best melody. run_replay owns lap_start/lap_stop (chain
    // documented there); an explore's duration is recoverable from the
    // telemetry timestamps.

    for (;;) {
        sensor_snapshot_t before, at_center;

        // RUN_WATCHDOG_MS without a junction = the robot left the maze.
        drive_status_t st =
            drive_until_junction(&before, SPEED_EXPLORE, RUN_WATCHDOG_MS);
        if (st == DRIVE_ABORT)   { drive_stop(); return MODE_DIAG; }
        if (st == DRIVE_TIMEOUT) { fault("lost: 10s no jct"); return MODE_DIAG; }
        if (st == DRIVE_LOST)    { fault("line LOST"); return MODE_DIAG; }

        st = creep_to_center(&before, &at_center);
        if (st == DRIVE_ABORT)   { return MODE_DIAG; }
        if (st == DRIVE_TIMEOUT) { fault("creep stalled");  return MODE_DIAG; }

        junction_t j = CLASSIFY(&before, &at_center);
        telemetry_event(EV_CLASSIFY, (int16_t)j, 0);

        // The decision chain: the left-hand rule picks the move, the
        // arrival verdict judges what to do with it, the path records
        // it, the feedback hooks announce it — and only then does the
        // robot act. Both logic halves are pure and host-tested; this
        // loop only obeys. Record BEFORE moving, so a fault mid-turn
        // still leaves an honest record of what was decided — but AFTER
        // the verdict, so a byte the robot cannot execute never enters
        // the record at all.
        move_t m = decide_left_hand(j);
        arrival_verdict_t v = explore_arrival_verdict(j, m);
        if (v == ARRIVE_SUCCESS) {   // standing on the goal patch
            drive_stop();
            on_goal();
            return MODE_SOLVED;
        }
        if (v != ARRIVE_PROCEED) {
            fault(verdict_fault_text(v));
            return MODE_DIAG;
        }

        path_record(&path_raw, m);
        if (path_raw.overflow) {
            // Junction #65 didn't fit. A truncated route can never be
            // replayed, so the latch must surface NOW — stopping here is
            // what keeps an overflowed run from ever reaching the goal
            // and masquerading as a solve.
            fault("path overflow");
            return MODE_DIAG;
        }
        on_junction(j, m, &path_raw);
        sleep_ms(THINK_PAUSE_MS);

        st = execute_move(m);
        if (st == DRIVE_ABORT)   { return MODE_DIAG; }
        if (st == DRIVE_TIMEOUT) { fault("turn timeout"); return MODE_DIAG; }
    }
}

// A recorded path is NOT a C string — no terminator, `len` is the truth
// (see path_t). This makes one, so the UI can print it.
static void path_to_string(const path_t *p, char out[PATH_MAX_MOVES + 1])
{
    for (int i = 0; i < p->len; i++) { out[i] = p->moves[i]; }
    out[p->len] = '\0';
}

// ----------------------------------------------------------------- SOLVED
// The goal was found; path_raw holds the route with all its detours.
// This mode copies it, simplifies the copy, and shows BOTH strings on one
// screen, so the collapse ("LLBLLBS" into "LSR") is visible. path_solved
// is what REPLAY runs; path_raw is kept untouched as the honest record of
// what the explorer actually drove.
static run_mode_t run_solved(void)
{
    on_state_change(MODE_SOLVED);

    path_solved = path_raw;
    path_simplify(&path_solved);

    // The OLED is 16 columns wide; the length counters stay honest even
    // when a long route's tail is clipped from the screen.
    char buf[PATH_MAX_MOVES + 1];
    ui_clear();
    ui_text(0, 0, "explored (%u):", path_raw.len);
    path_to_string(&path_raw, buf);
    ui_text_big(0, 1, "%.16s", buf);                              // px 16..31
    ui_text(0, 4, "%.16s", path_raw.len > 16 ? buf + 16 : "");    // px 32..39

    ui_text(0, 5, "solved (%u):", path_solved.len);               // px 40..47
    path_to_string(&path_solved, buf);
    ui_text_big(0, 3, "%.16s", buf);                              // px 48..63

    ui_flush_now();
    wait_for_button();
    return MODE_DIAG;
}

// ----------------------------------------------------------------- REPLAY
// Drive path_solved to the goal against the lap clock. Needs a solved
// path (EXPLORE must have reached the goal first) and a passing
// preflight; returns the mode to enter next.
static run_mode_t run_replay(void)
{
    on_state_change(MODE_REPLAY);

    if (path_solved.len == 0) {
        ui_clear();
        ui_text_big(0, 0, "REPLAY");
        ui_text(0, 3, "no solved path.");
        ui_text(0, 4, "run EXPLORE to");
        ui_text(0, 5, "the goal first.");
        ui_flush_now();
        wait_for_button();
        return MODE_DIAG;
    }

    if (!preflight()) { return MODE_DIAG; }
    countdown();

    telemetry_reset();
    telemetry_event(EV_RUN_START, MODE_REPLAY, SPEED_REPLAY);

    // The lap clock starts HERE — after the countdown, as the wheels are
    // about to move, so the countdown never counts as driving time. The
    // chain the lap display depends on: this lap_start → lap_stop on the
    // goal arrival below (the ONLY route into MODE_DONE) → run_done reads
    // last/best → the new-best melody. Break any link and DONE shows
    // 0.00 s while the melody stays silent.
    lap_start();

    // The same follow → creep → classify skeleton as run_explore, but the
    // decision at each junction comes from replay_next(&path_solved,
    // jct_idx) instead of the left-hand rule. Dispatch is on JUNCTION
    // ARRIVALS, never on distance or time: tape stretches and wheels
    // slip, but junction #4 is junction #4 forever.
    // The skeleton is duplicated from run_explore ON PURPOSE, not shared:
    // the replay speed profile lives in THIS copy only — straights hot at
    // SPEED_REPLAY, then arrival_brake sheds the momentum before the
    // creep — and the explorer's copy must never feel any of it. The
    // profile itself is data in tuning.h (SPEED_REPLAY, SPEED_ARRIVAL,
    // ARRIVAL_BRAKE_MM), not branches in this loop.
    uint8_t jct_idx = 0;   // how many junctions this run has arrived at
    for (;;) {
        sensor_snapshot_t before, at_center;

        drive_status_t st =
            drive_until_junction(&before, SPEED_REPLAY, RUN_WATCHDOG_MS);
        if (st == DRIVE_ABORT)   { drive_stop(); return MODE_DIAG; }
        if (st == DRIVE_TIMEOUT) { fault("lost: 10s no jct"); return MODE_DIAG; }
        if (st == DRIVE_LOST)    { fault("line LOST"); return MODE_DIAG; }

        // The slowdown window, triggered by the ARRIVAL itself (the
        // DRIVE_OK edge above) — never by guessed distance or elapsed
        // time. The motors are still running at base speed (drive.h's
        // contract): shed that momentum to SPEED_ARRIVAL now, BEFORE
        // creep_to_center takes the encoder baseline it measures CREEP_MM
        // from. Physics and knobs: tuning.h.
        st = arrival_brake(&before, SPEED_REPLAY);
        if (st == DRIVE_ABORT)   { return MODE_DIAG; }

        st = creep_to_center(&before, &at_center);
        if (st == DRIVE_ABORT)   { return MODE_DIAG; }
        if (st == DRIVE_TIMEOUT) { fault("creep stalled");  return MODE_DIAG; }

        junction_t j = CLASSIFY(&before, &at_center);
        telemetry_event(EV_CLASSIFY, (int16_t)j, 0);

        // The plan for this arrival, and the verdict on it. The whole
        // decision table — path spent or not, what the classifier saw,
        // whether the byte is a move — lives in logic/maze_logic.c, with
        // every cell pinned by the tests in tests/logic; this loop's job
        // is to obey it and put the right screen up.
        move_t m = replay_next(&path_solved, jct_idx);
        arrival_verdict_t v = replay_arrival_verdict(j, m);
        if (v == ARRIVE_SUCCESS) {
            // The clock stops on the arrival itself — before the melody
            // and the screen, for the same reason it started after the
            // countdown. lap_stop() latches last, best and the was-best
            // verdict in one place; run_done() only ever READS them. A
            // faulted run exits below WITHOUT stopping the clock, so it
            // records no finish time at all.
            lap_stop();
            drive_stop();
            on_goal();
            return MODE_DONE;
        }
        if (v != ARRIVE_PROCEED) {
            fault(verdict_fault_text(v));
            return MODE_DIAG;
        }

        st = execute_move(m);
        if (st == DRIVE_ABORT)   { return MODE_DIAG; }
        if (st == DRIVE_TIMEOUT) { fault("turn timeout"); return MODE_DIAG; }
        jct_idx++;
    }
}

// ------------------------------------------------------------------- DONE
// The read-only end of the lap chain (GO → lap_start, goal → lap_stop,
// here → display). Every value below was latched by lap_stop() at the
// goal; this screen only reports them, and the melody fires only on a
// genuine new best.
static run_mode_t run_done(void)
{
    on_state_change(MODE_DONE);
    uint32_t last = lap_last_ms(), best = lap_best_ms();
    ui_clear();
    ui_text_big(0, 0, "%2u.%02us", (unsigned)(last / 1000),
                (unsigned)((last % 1000) / 10));
    ui_text(0, 4, "best %2u.%02us %s", (unsigned)(best / 1000),
            (unsigned)((best % 1000) / 10),
            lap_last_was_best() ? "NEW!" : "");
    ui_flush_now();
    if (lap_last_was_best()) { feedback_play_melody(melody_new_best); }
    wait_for_button();
    return MODE_DIAG;
}

// ------------------------------------------------------------------ SPARE
// Unused slot in the menu: puts a screen up and returns. Kept so a new
// mode can be added without touching the menu machinery.
static run_mode_t run_spare(void)
{
    on_state_change(MODE_SPARE);
    ui_clear();
    ui_text_big(0, 0, "SPARE");
    ui_text(0, 3, "unused menu slot");
    ui_text(0, 4, "for a new mode.");
    ui_flush_now();
    wait_for_button();
    return MODE_DIAG;
}

// --------------------------------------------------------------- LOG DUMP
// Replay the flight recorder over USB (see telemetry.h). The dump takes
// seconds and only makes sense with a terminal attached — the screen says
// so instead of leaving a silent robot.
static void run_logdump(void)
{
    on_state_change(MODE_LOGDUMP);
    ui_clear();
    ui_text_big(0, 0, "LOG DUMP");
    ui_text(0, 3, "%u recs", (unsigned)telemetry_count());
    ui_text(0, 4, "USB terminal on?");
    ui_text(0, 5, "dumping...");
    ui_flush_now();
    // Hand the dump its context: telemetry.c only observes, and this file
    // is the one that knows the hardware. Battery is read NOW — resting,
    // motors long stopped — so the loaded voltage during the run was
    // lower still: a low number here is damning, not borderline. The
    // calibration window is read NOW too, and that is a real caveat: the
    // ring is only cleared at a run start, so this is the window that
    // scaled the rows ONLY if nothing recalibrated in between. Run
    // CALIBRATE between the run and the dump and the header advertises
    // the NEW window over rows the OLD one scaled. Both the ring and the
    // window survive between a run and its dump only because the robot
    // is never powered off in between.
    telemetry_set_dump_context(hw_battery_mv(),
                               hw_line_cal_min, hw_line_cal_max);
    telemetry_dump();
    ui_text(0, 6, "done");
    ui_flush_now();
    wait_for_button();
}

// ------------------------------------------------------------------- menu

static const struct { run_mode_t mode; const char *name; } menu[] = {
    { MODE_DIAG,      "DIAG"      },
    { MODE_CALIBRATE, "CALIBRATE" },
    { MODE_EXPLORE,   "EXPLORE"   },
    { MODE_REPLAY,    "REPLAY"    },
    { MODE_SPARE,     "SPARE"     },
    { MODE_LOGDUMP,   "LOG DUMP"  },
};
#define MENU_COUNT ((int)(sizeof menu / sizeof menu[0]))

static void draw_menu(int sel)
{
    ui_clear();
    ui_text_big(0, 0, "%-9s", menu[sel].name);
    ui_text(13, 0, "%d/%d", sel + 1, MENU_COUNT);
    ui_text(0, 3, "C tap : next");
    ui_text(0, 4, "C hold: GO");
    show_battery_line(6);
    ui_text(0, 7, "cal:%s best:%u.%us", hw_line_cal_ok() ? "OK" : "--",
            (unsigned)(lap_best_ms() / 1000),
            (unsigned)((lap_best_ms() % 1000) / 100));
    ui_flush_now();
}

static void launch(run_mode_t m)
{
    run_mode_t next = m;
    do {
        switch (next) {
        case MODE_DIAG:      run_diag();            next = MODE_COUNT; break;
        case MODE_CALIBRATE: run_calibrate();       next = MODE_COUNT; break;
        case MODE_EXPLORE:   next = run_explore();  break;
        case MODE_SOLVED:    next = run_solved();   break;
        case MODE_REPLAY:    next = run_replay();   break;
        case MODE_DONE:      next = run_done();     break;
        case MODE_SPARE:     run_spare();           next = MODE_COUNT; break;
        case MODE_LOGDUMP:   run_logdump();         next = MODE_COUNT; break;
        default:             next = MODE_COUNT;     break;
        }
        // Chained states (EXPLORE→SOLVED, REPLAY→DONE) keep running;
        // MODE_DIAG from a runner means "back to menu".
        if (next == MODE_DIAG) { next = MODE_COUNT; }
    } while (next != MODE_COUNT);
    drive_stop();
}

int main(void)
{
    stdio_init_all();

    ui_init();
    feedback_init();
    hw_buttons_init();
    drive_init();          // motors + encoders + line sensors
    telemetry_init();
    imu_ok = hw_imu_init();

    // Boot splash: robot name, battery state, IMU presence.
    ui_clear();
    ui_text_big(0, 0, "%.16s", ROBOT_NAME);
    ui_text(0, 2, "line maze solver");
    show_battery_line(4);
    ui_text(0, 5, "imu: %s", imu_ok ? "ok" : "MISSING");
    ui_text(0, 7, "hold C to begin");
    ui_flush_now();
    feedback_beep(NOTE_C5, 60);
    feedback_beep(NOTE_G5, 90);

    int sel = 0;
    uint32_t last_redraw = 0;
    draw_menu(sel);

    for (;;) {
        button_event_t ev = hw_buttons_poll();
        if (ev == BTN_C_SHORT) {
            sel = (sel + 1) % MENU_COUNT;
            draw_menu(sel);
        } else if (ev == BTN_C_LONG || ev == BTN_A_PRESS) {
            launch(menu[sel].mode);
            hw_buttons_wait_release();
            draw_menu(sel);
        }
        // Battery + cal status refresh while idling.
        if (hw_millis() - last_redraw > 1000) {
            last_redraw = hw_millis();
            draw_menu(sel);
        }
        sleep_ms(10);
    }
}
