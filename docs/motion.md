# Motion

Everything between "the sensors say X" and "the wheels do Y". This layer
decides nothing about the maze: it hands `logic/` two clean sensor snapshots
per junction and executes whatever move comes back.

**Source:** `src/drive.c` (1051 lines), `src/drive.h` (the caller contract),
`src/hw_motors.c`
**Consumers:** the EXPLORE and REPLAY loops in `src/main.c`

## Four rules that hold everywhere in `drive.c`

1. **Every loop checks `hw_buttons_any()`** and returns `DRIVE_ABORT`. A
   finger on any button always stops the robot, no matter how wrong the
   surrounding code is.
2. **Every loop has a timeout.** A maneuver that cannot finish says so
   instead of grinding its gears forever.
3. **No `ui` calls, ever.** An OLED flush is a multi-millisecond SPI
   transfer; a control loop cannot afford that hole in its timing, and the
   hole is visible as a wobble.
4. **No `printf` either.** `telemetry_tick()` writes 16 bytes to RAM, and
   that is the only reporting a control loop is allowed to do.

## The motor driver

`src/hw_motors.c`. PWM slice 7 drives both wheels — GP14 (right) is channel
A, GP15 (left) is channel B, because the RP2040 maps GPIO *n* to slice
`(n/2) % 8` with A/B by even/odd, and 14/15 are an even/odd pair. Clock
divider 1 keeps the full 125 MHz; a 6000-count wrap gives **20.8 kHz**, above
the audible range so the motors do not whine.

Direction lives on GP10 (right) and GP11 (left), HIGH = reverse. The PWM
level is always a magnitude.

Two facts with consequences:

**`SPEED_HARD_CAP` (5000) is clamped here and only here.** Every speed
calculation upstream — PD overshoot, an integer overflow, a mistyped constant
— funnels through one choke point at the last gate before the hardware, so
there is always a bounded stop path. `tuning.h` also checks every `SPEED_*`
knob against it at compile time.

**Zero duty is a BRAKE, not a coast.** These are PHASE/ENABLE drivers: during
the off-fraction of every PWM cycle the driver ties the motor terminals
together. `hw_motors_stop()` therefore *holds* the wheels. This is what makes
the arrival brake work by simply commanding a lower speed.

## One segment, end to end

```
  drive_until_junction(&before, base, RUN_WATCHDOG_MS)
      PD follow at `base`, junction detector armed after BLIND_MM
      returns DRIVE_OK with the motors STILL RUNNING
                    |
  arrival_brake(&before, base)          [replay only; no-op at base <= 2500]
      command SPEED_ARRIVAL, roll ARRIVAL_BRAKE_MM, keep latching maxima
                    |
  creep_to_center(&before, &at_center)
      roll CREEP_MM on encoders, latch maxima, stop, settle, read at_center
      returns DRIVE_OK with the robot STOPPED on the junction center
                    |
  CLASSIFY(&before, &at_center)  ->  a junction_t          [logic/]
  decide / replay_next           ->  a move_t              [logic/]
                    |
  execute_move(m)   turn_left_90 / turn_right_90 / turn_around_180 / nothing
```

The two snapshots are the seam. `before` is the **sweep**: per-sensor maxima
latched while the bar crossed the junction. `at_center` is the **settled
read**, taken stopped, with the bar past the junction line. Why each carries
the evidence it does is [maze-search.md](maze-search.md).

## The line follower

The steering law every driving phase shares, in `follow_tick()`.

```
pos  = weighted average of the five calibrated sensors, 0..4000
p    = pos - 2000                        (0 = centered)
d    = p - last_p                        (plain difference, no filter)
pid  = (p*LINE_KP + d*LINE_KD) * base / 6000
left  = clamp(base + pid, 0, base)
right = clamp(base - pid, 0, base)
```

`LINE_KP` 90, `LINE_KD` 3000.

**Saturation is what turns a signed correction into "slow one wheel."** The
low clamp stops a large `pid` from commanding a wheel *backwards* (an
in-place pivot mid-segment); the high clamp stops the outer wheel from being
driven past the speed the caller asked for. Both sides held to `[0, base]`
means the pair can only ever be a differential slowdown around `base` — so
**the robot cannot outrun the speed it was given while following**, which is
what keeps junction detection honest.

**The `base/6000` scaling** is why the same two gains work at explore speed
and replay speed: steering is differential drive, so scaling the command with
the base speed produces the same *curvature* at any pace. The gains
themselves came from the stock Pololu follower, tuned at base 6000 in a ~3 ms
MicroPython loop, which is the history the tuning log records.

### The fixed tick

`CONTROL_PERIOD_US` = 2000 → **500 Hz**, held by one deadline advanced in
fixed steps.

The raw sensor read takes anywhere from ~0.15 ms (all white) to ~1.1 ms (deep
black), so a free-running loop would speed up and slow down about 7× with the
floor under it. Since `d` is a per-tick difference, the damping would breathe
with the floor. Fixing the period is what makes `LINE_KD` a constant of the
robot instead of a constant of the floor.

If a tick ever overruns its period (it should not — worst-case read plus math
is ~1.2 ms), the clock **resyncs** rather than sprinting to catch up: a late
tick is noise, but a burst of back-to-back ticks is a D-term lie.

### Three estimator branches

| Branch | When | What it does |
|---|---|---|
| **blind** | inside the post-turn `BLIND_MM` window | weighted average of the **center three** sensors only. Remnants of the junction just handled may still sit under the outers, and they would drag the average hard sideways. A near-zero sum (< 100 across three sensors) steers dead ahead instead — the "position" of nothing is a random number. |
| **center lost** | `s1,s2,s3` all below `CENTER_LOST_THRESH` | steer hard back toward where the line *was*, using the sign of the last error. Gated on `have_measured_p`, not `have_p`: this branch writes **synthetic** ±2000 values, and without the stricter gate one pass through it would launder a made-up 0 into `last_p` and the next tick would slam full-scale toward a coin flip. |
| **normal** | otherwise | weighted average of all five, and the only branch that appends to the `recent_p` history |

The D term is skipped on the first tick after a reset (`have_p == false`).
Differencing against a made-up `last_p` of 0 is a full-scale derivative kick,
and it would land at exactly the worst moment — the first sample after a
turn. The same suppression fires when the estimate switches from center-3 to
all-5 at the end of the blind window, so the scale change does not read as a
spike.

## Junction detection

Two independent triggers, both inside the follow loop.

### The edge latch

A branch line is ~19 mm of tape passing under an outer sensor. At replay
speed that can be **one or two samples**. So each outer sensor is OR-ed with
its own previous sample:

```c
l0 = max(line[0], prev0);
l4 = max(line[4], prev4);
if (armed && !suppressed && (l0 >= JCT_OUTER_THRESH || l4 >= JCT_OUTER_THRESH))
        -> junction
```

A branch that flashed by for a single read still makes it into the snapshot.
The latch is flushed to zero at three boundaries — when the blind window
lifts, when the post-resume suppression lifts, and at a resume — so a sample
taken inside a window where readings are not trusted cannot leak across it
one tick later.

### The blind window

`BLIND_MM` = 30 mm of **encoder distance** (not time, so the same window
works at any speed) after each segment starts. Right after a turn the sensor
bar may still be hovering over pieces of the junction just handled;
triggering on those would classify the same junction twice.

This is the other half of the maze's ≥150 mm junction-spacing rule: worst
case a junction consumes `ARRIVAL_BRAKE_MM` 20 + `CREEP_MM` 45 + `BLIND_MM`
30 = 95 mm of the 150.

### The honest dead end

The second trigger is the line **disappearing** — and telling that apart from
driving off the tape is the interesting part. See the loss machine below. A
dead end returns `DRIVE_OK` with `before` holding the **raw** reading, not
the edge-latched outers: a latched outer sample from an earlier tick would
put branch evidence into the one snapshot whose entire meaning is that there
is none.

## The loss machine

A small state machine (`loss_monitor_t`) answers one question per tick:
keep driving, honest dead end, or swerved off the road?

```
all five below LINE_LOST_THRESH, held LOST_CONFIRM_MS   -> a hard loss
        and recent max |p| <= DEADEND_P_MAX             -> LOSS_LINE_ENDED
        and recent max |p| >  DEADEND_P_MAX             -> LOSS_SWERVED
center lost, held CENTER_LOST_MAX_MS                    -> LOSS_SWERVED
```

**The jury is `recent_p`** — the last 64 *measured* |p| samples, about 128 ms.
A line that ends while the follower was driving straight is a dead end; a
line that "ends" while the follower was sawing at the wheel is a robot
leaving the road. Synthetic center-lost samples are excluded from that
history on purpose. A center-lost slam that never resolves is never called a
dead end: the outer sensors may still be seeing tape.

| Constant | Value | Role |
|---|---|---|
| `LOST_CONFIRM_MS` | 20 | debounce; 10 ms was thin enough for a glare patch to fake a loss |
| `DEADEND_P_MAX` | 600 | the dead-end / swerve verdict line |
| `CENTER_LOST_MAX_MS` | 250 | center gone this long while slamming is also lost |
| `RESUME_GRACE_MM` | 15 | after a resume, the hard-loss verdict stays holstered this far |
| `RECOVERIES_MAX` | 3 | recovery budget per segment |
| `RECOVERY_DECAY_MM` | 200 | clean travel earns one credit back |

The **grace gate** exists because a resume re-arms with a wiped |p| history: a
white read in the first ticks would reach the jury with `rmax = 0` — an
automatic, and false, "honest dead end".

The **budget decay** exists because the budget should measure *repeated
failure*, not lifetime bad luck. Four one-off successful recoveries spread
over a long segment would otherwise fault the same as a tight lawnmower loop.
The decay is paid for in distance, which is what bounds it: a genuine
lawnmower loop loses the line again within a few centimetres and earns
nothing back.

## Backtrack recovery

When the verdict is `LOSS_SWERVED`, the robot retraces its own path.

`drive.c` records the **per-tick encoder delta of each wheel** into a
1024-entry ring (~2 s of history) on every follow tick. Recovery replays it
newest-first as a *retreating target* and P-chases it — closed loop on
odometry, not on duty, so stiction and battery sag cannot bend the retraced
path. One recorded tick is consumed per `BACKTRACK_TICK_X` (2) control
periods, so the retrace runs at half the recorded speed. This is a recovery,
not a maneuver.

```
stop, settle BT_PREROLL_SETTLE_MS (120 ms)     the hottest stop in the sequence:
                                               entered mid-correction, and the
                                               first act after stopping is to read
                                               the encoders that become the targets
replay breadcrumbs backwards, P-chase          BACKTRACK_KP 60, capped BACKTRACK_SPEED 1200
tape under a center sensor for
  BT_FOUND_CONFIRM_TICKS (3) in a row          one dark sample is an opinion, not a measurement
history spent? drain the residual chase 500 ms the chase always trails its target
stop, settle BT_SETTLE_MS (80 ms), re-check
  still on tape                    -> resume
  bare floor                       -> nudge FORWARD up to RESUME_NUDGE_MM
  nudge finds nothing              -> DRIVE_LOST
```

The **forward** nudge is the non-obvious step. The retrace confirmed tape
while the robot was *reversing*, and reverse momentum keeps carrying it
through the stop — so the tape just confirmed is **ahead** of the bar. Never
further backward: reverse is the one direction guaranteed to be wrong there.
The nudge is bounded in distance (`RESUME_NUDGE_MM` 20) and time
(`RESUME_NUDGE_TIMEOUT_MS` 800), and it uses the same straightness servo as
the creep.

A find that does not survive the settle re-check is **demoted** to a failure —
better an honest fault than a resume onto bare floor. In a dump that shows as
`BT_FOUND` followed by `BT_FAIL`.

### The resume protocol

Back on the tape, stopped, pointing roughly down the line — but *where* on the
tape? The retrace stops at the first confirmed refind, which can be anywhere
along the recorded arc: usually just behind the swerve, but after a bad kink
it can be all the way back over the junction this segment **started** from,
already classified, recorded and turned at.

So the resume re-arms steering and the loss machinery immediately, but
junction detection stays holstered until:

```
suppress_until = max(arm_enc, departure point)
```

where the departure point is the last tick the center still held the tape.
A detect inside that stretch could only re-announce evidence already dealt
with — the caller would classify the same junction *again* and record a
phantom move, and the recorded path would stop describing the maze.

The bound cannot mask a real next junction, and the argument is worth
keeping: everything up to `arm_enc` is the original blind window, where the
maze's ≥150 mm junction spacing already guarantees nothing lives; everything
from `arm_enc` to the departure point was swept **on** the tape with detection
armed and silent — the maze itself testifying there is no junction there. The
next branch therefore lies strictly beyond the departure point, and the 55 mm
of unspent spacing absorbs the few millimetres of retrace slack many times
over.

Three more bookkeeping details, each guarding a specific lie:

- The segment watchdog deadline is pushed out by however long the retrace
  took, so recovery time does not eat the segment's budget.
- The grace baseline is a **fresh** encoder read, because `enc0` is about to
  be back-dated by `blind_counts` (that is how the resume arms instantly), and
  measuring grace from it would hand the gate 30 pre-paid millimetres.
- `was_armed` stays **true**, so the arming block does not run again and log a
  `BLIND_END` the robot never earned. `EV_RESUME` is the honest marker, and
  its second field carries the suppressed span in mm — the only trace
  suppression leaves, since detection staying quiet looks exactly like open
  corridor.

## The arrival brake

Replay only. `drive_until_junction` hands back a robot still moving at `base`,
and `creep_to_center` measures its `CREEP_MM` from wherever it begins. Out of
a hot base the chassis is still braking when that budget runs out, so the
deciding `at_center` read lands past the junction.

```c
if (base <= SPEED_ARRIVAL) { return DRIVE_OK; }   // no-op at today's SPEED_REPLAY
hw_motors_set(SPEED_ARRIVAL, SPEED_ARRIVAL);      // brake FIRST, then loop
```

Braking here is nothing but *commanding* the lower speed — the PHASE/ENABLE
drivers do the rest — so the loop needs no speed measurement, no ramp table,
and has no way to stall the robot dead short of a junction it still has to
cross.

The window is bounded by **distance** (`ARRIVAL_BRAKE_MM` 20), because its
real cost is position. `ARRIVAL_BRAKE_MS` (80) survives only as the *timeout*:
if the encoders stop counting, the clock closes the window instead, so the
loop cannot hang. A stalled chassis is the creep watchdog's diagnosis to make.

The arithmetic that sized it is in the tuning log: at arrival pace (~0.62 m/s)
the retired 80 ms wall-time window travelled ~50 mm — more than the creep's
entire budget — landing the `at_center` read ~95 mm past the detect edge, out
the far side of the recommended 75 mm goal patch. Every stepped-base goal
arrival would have faulted `"replay:not goal"`. Twenty millimetres caps the
total drift at 65 mm, 10 mm inside the patch.

The window keeps latching per-sensor maxima into the sweep, so the
classifier's evidence has no blind spot, and steers straight **on encoders** —
the bar is over junction tape, where a weighted average measures tape
geometry, not where the robot ought to point.

## The creep

The sensor bar rides ahead of the wheel axle. When the bar detects a junction,
the axle — the point the robot pivots around — is still `CREEP_MM` (45 mm)
short of it. Turning now would cut the corner and miss the new line.

So roll forward a measured distance at `SPEED_CREEP` (1200), sweeping the
sensors across the junction (free evidence for the classifier), then stop,
wait `CREEP_SETTLE_MS` (60 ms) for the chassis to stop rocking, and take the
deciding `at_center` read.

Straightness is a small servo on the left/right count difference,
`CREEP_BAL_GAIN` (8) — the same gain used by the arrival brake and the
recovery nudge. No line data steers this loop, for the same reason as the
arrival window.

`CREEP_TIMEOUT_MS` (1500) catches a stalled wheel or an inverted encoder sign.

Exit state is precise and the turns depend on it: **stopped and settled with
the axle on the junction center**.

## Gyro turns

PD on the integrated gyro angle, relative to wherever the robot is pointing
when the call is made. Covered in [sensor-imu.md](sensor-imu.md) — including
why the gyro rather than the encoders, and why "settled" means in tolerance
*continuously* for 250 ms rather than for one sample.

## Speeds

| Constant | Value | Used for |
|---|---|---|
| `SPEED_HARD_CAP` | 5000 | the absolute ceiling, clamped in `hw_motors.c` — not a lap-time knob |
| `SPEED_EXPLORE` | 1800 | mapping pace: slow enough that no junction slips past |
| `SPEED_REPLAY` | 2500 | replaying a solved path |
| `SPEED_ARRIVAL` | 2500 | the pace the creep was tuned for |
| `SPEED_SPEEDRUN` | 4000 | the fast-lap target `SPEED_REPLAY` is stepped toward |
| `SPEED_CREEP` | 1200 | rolling the sensors across a junction |
| `SPEED_TURN_MAX` | 2500 | cap for in-place gyro turns |
| `SPEED_CALIBRATE` | 1000 | the in-place calibration spin |
| `BACKTRACK_SPEED` | 1200 | duty cap while retracing |

`SPEED_REPLAY` and `SPEED_ARRIVAL` are equal today, which makes the arrival
window a deliberate no-op: only a base stepped past 2500 ever feels it run.

If `SPEED_CALIBRATE` is changed, the calibration sweep's three phase
durations (500/1000/500 ms) must scale **inversely** with it — an in-place
spin sweeps angle = speed × time, and the sweep has to carry the whole bar
across the line and back in each direction. A sensor that missed shows up as
a span under `CAL_MIN_SPAN` on the screen right after.

## Status codes

`drive_status_t`, and what each obliges the caller to do:

| Status | State on return | Caller |
|---|---|---|
| `DRIVE_OK` | **motors still running** after `drive_until_junction` and `arrival_brake`; **stopped and settled** after `creep_to_center` and the turns | proceed promptly |
| `DRIVE_ABORT` | stopped | bail out cleanly, back to the menu |
| `DRIVE_TIMEOUT` | stopped | fault — the robot is lost or stuck |
| `DRIVE_LOST` | stopped somewhere on the floor | fault — recovery is exhausted |

Recovered losses never reach the caller. Only a recovery that fails outright,
or a spent budget, becomes `DRIVE_LOST`.

The "motors still running" contract has one narrow exception, documented in
`drive.h`: on the first tick after a recovery resume the retrace has left the
chassis stopped and that tick's follow command has not been issued yet, so a
junction detected on exactly that tick returns `DRIVE_OK` with the robot
stationary. Every caller-visible guarantee still holds — treat "motors
running" as the contract to code against, not a fact to measure.

## Failure modes

| Fault string | Cause | Dump signature |
|---|---|---|
| `"lost: 10s no jct"` | drove off the maze, or a junction never read as one | `RUN_START`, `RUN_WATCHDOG_MS` of `F` rows with no `JCT_DETECT`, `TIMEOUT [phase=F]` |
| `"line LOST"` | budget spent | a final `LOSS` with no `BT_START` after it |
| `"line LOST"` | retrace failed | `LOSS`, `BT_START`, `K` rows, `BT_FAIL` |
| `"line LOST"` | find demoted | `BT_FOUND` (possibly with nudge `C` rows) then `BT_FAIL` |
| `"creep stalled"` | jammed wheel, blocked chassis, flipped encoder sign | `CREEP_START`, `C` rows whose `a` stops climbing, `TIMEOUT [phase=C]` |
| `"turn timeout"` | the turn never settled | `TURN_START` with no `TURN_END`, `T` rows stalling short of zero |

One triage note that saves time: **follow rows carry no odometry.** On `F`/`B`
rows, `left`/`right` are commanded duty and `a`/`b` are the error and the
steer — nothing in a follow row says how far the robot travelled, and ticks ×
speed slips exactly when the wheels do. Audit a distance knob at the rows that
do carry encoder numbers: `BLIND_END`'s `a` (counts travelled blind), the `C`
rows' `a` (progress in counts), the `K` rows' per-wheel retrace errors, and
`RESUME`'s `b` (suppressed span, already in mm).
