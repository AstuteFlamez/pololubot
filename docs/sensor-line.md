# Line sensors

Five down-facing reflectance sensors on a bar ahead of the axle. They are the
only sensor that sees the maze; everything else measures the robot.

**Source:** `src/hw_line.c`, `src/hw_line.h`
**Pins:** GP18–GP22 (sensors), GP26 (IR emitter control)
**Consumers:** `src/drive.c` (steering, junction detection, loss),
`logic/maze_logic.c` (classification)

## The hardware

Each sensor is an IR phototransistor across a small capacitor, with a
down-facing emitter LED beside it. There is no ADC in the path. The
measurement is a **time**:

```
1. drive the pin HIGH for 32 µs        -> charges the capacitor
2. flip the pin to an INPUT            -> it floats, still reading high
3. count µs until the pin reads LOW    -> that count IS the reading
```

Reflected IR makes the phototransistor conduct and bleed the charge away, so
white posterboard decays fast (~100 µs) and black tape decays slowly (up to
the 1024 µs timeout). The full-scale value 1024 matches Pololu's own PIO
implementation, so raw readings from the two are directly comparable.

Indexing runs **left to right across the robot**:

```
   out[0]   out[1]   out[2]   out[3]   out[4]
    GP22     GP21     GP20     GP19     GP18
  leftmost                            rightmost
  (DN1)                                  (DN5)
```

GP18–GP22 are contiguous, so one shift of the GPIO input register lands them
in bits 0..4 — and because GP18 (bit 0) is the *rightmost* sensor while the
array runs left to right, the conversion is `index = 4 - bit`. That single
line is the whole reason the array is not mirrored.

## Why all five are read in parallel

Five sequential reads would cost up to 5 ms. At replay pace the robot covers
about 3 mm in that time, which smears the junction edges the reading exists
to find. Charging all five together and polling them in one loop costs the
same as one sensor — bounded at ~1.1 ms worst case — and yields five samples
of *the same instant of travel* rather than five samples from five different
places on the course.

The poll loop reads the whole GPIO register each pass, finds the bits that
were high last pass and are low now, and stamps them all with the same
elapsed time. Sensors that never fall keep the full-scale 1024.

A pull-up would keep feeding the capacitor and a pull-down would drain it, so
pulls are explicitly disabled on all five pins before the charge phase.

Pololu's driver counts the decay in PIO hardware and frees the CPU entirely
(`third_party/pololu_3pi_2040_robot/qtr_sensor_counter.pio`). A CPU loop
against the µs timer is sufficient at these speeds and far easier to
single-step, which is why this code is the one that ships.

## Calibration

Raw microseconds are a property of this robot, this tape, this room's light,
and this battery charge. The logic layer never sees them.

`hw_line_cal_update()` takes a raw read and widens each sensor's own min/max
window; **CALIBRATE** calls it in a loop while the robot spins in place —
right 0.5 s, left 1.0 s, right 0.5 s, ending where it started — so every
sensor crosses both white paper and black tape. The windows start inverted
(`min = 1025`, `max = 0`) so the first update replaces both ends instead of
widening from a bogus seed.

`hw_line_read_calibrated()` then maps each sensor through its own window:

```
0     at or below that sensor's white extreme
1000  at or above its black extreme
      linear in between; a sensor whose window is empty or inverted reads 0
```

### The calibration gate

`hw_line_cal_ok()` is true only when **every** sensor's window spans at least
`CAL_MIN_SPAN` (300 µs). A narrower span means that sensor never really saw
the tape, and scaling against it amplifies noise into full-range readings.
`preflight()` in `main.c` refuses to explore or replay without it —
`"run CALIBRATE 1st"`.

The usual cause of a narrow span is calibrating over paper only. The second
is glossy tape: the sensors measure *reflected* IR, and glossy tape
mirror-reflects at some angles and reads as bright paper. The maze
specification calls for matte black tape for exactly this reason.

The CALIBRATE screen prints all five spans after the sweep and holds them
until a button press, because five numbers cannot be read off a timed flash.

## The GP26 trap

The pin that switches the down-facing IR emitters **is** the pin the VBAT/11
divider feeds (see `include/pins.h`). A pad has one function at a time.

The discipline both owners keep is *re-claim on every use, assume nothing*:

- `hw_line.c` calls `gpio_init()` on GP26 and drives it high for the length
  of a read, then returns it to a plain input (emitters off).
- `hw_battery.c` calls `adc_gpio_init()` on it before every conversion and
  leaves it in ADC mode.

So call order never matters. What is **not** allowed is interleaving: a
battery read taken in the middle of a line read switches the emitters off and
measures a driven pin instead of the divider. Both are blocking calls from
the main loop, so this cannot happen today — it is a rule for whoever adds
the third caller.

## The numbers that read this sensor

All calibrated, 0..1000, all in `include/tuning.h`:

| Constant | Value | Means |
|---|---|---|
| `JCT_OUTER_THRESH` | 600 | an outer sensor this dark = a side branch is here |
| `JCT_DARK_THRESH` | 600 | one reading counts as "dark" to the classifier |
| `GOAL_MIN_DARK` | 4 | this many dark sensors after the creep = the goal patch |
| `CENTER_LOST_THRESH` | 700 | below this on all of s1..s3, the follower has lost center |
| `LINE_FOUND_THRESH` | 700 | at/above this, a backtrack has refound the line |
| `LINE_LOST_THRESH` | 300 | below this on all five, the bar sees no tape at all |
| `CAL_MIN_SPAN` | 300 raw µs | the calibration gate |

**Ordering rule:** `LINE_FOUND_THRESH >= CENTER_LOST_THRESH`. When "found"
sat at 600 while the follower disowned the center below 700, a 600–699 graze
ended a retrace with a line the follower immediately gave up on, and the
resume cascaded into a false dead end on open floor. Closing that dead zone
is a 2026-08-10 entry in the tuning log.

## Failure modes

| Symptom | Cause | Where it shows |
|---|---|---|
| `"run CALIBRATE 1st"` | a span under 300 µs | CALIBRATE's span row; `cal_min`/`cal_max` in the dump header |
| every sensor reads 0 | window empty or inverted — never calibrated | DIAG bars flat with `cal:--` |
| tape reads as paper | glossy tape, or a lifted/curled edge | narrow span on the sensors that crossed it |
| junction missed at speed | branch tape under an outer sensor for one sample | handled by the edge latch in `drive.c` — see [motion.md](motion.md) |
| drifting readings mid-session | battery sag changing emitter brightness | `batt_mv` in the dump header |

## Where to look

- Live: **DIAG** shows the five calibrated bars, the raw middle sensor, and
  `cal:OK/--`.
- After a run: columns `s0..s4` of every tick row in the CSV dump are these
  five values. They are stored in the ring divided by 4 and re-expanded on
  dump, so they land on multiples of 4 — an 848 in the CSV and an 850 on the
  display are the same reading.
- The two one-shot rows `J` and `A` are the exact snapshots handed to the
  classifier. See [maze-search.md](maze-search.md).
