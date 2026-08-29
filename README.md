# pololubot

Firmware for a Pololu 3pi+ 2040 that solves a taped line maze. It explores
until it finds the goal, works out the shortest route from what it recorded,
and then runs that route back at speed.

Almost everything is written against the RP2040 directly. GPIO, the
microsecond timer, PWM, the encoder interrupts, the ADC, and the I2C traffic
to the IMU are all local code. The only vendor library linked in is Pololu's,
and only for the OLED and the RGB LEDs.

This README is the tour. [`docs/`](docs/) is the detail: one document per
sensor, plus one each for the motion layer, the maze search and the shortest
path — the mechanism, where every constant came from, and the telemetry
signature of each way it fails.

## Status

It works. The robot explores a taped maze it hasn't seen, finds the goal,
simplifies the route, and replays it.

The maze logic also carries 163 checks that build with plain `gcc` and run in
milliseconds, including a sensor-level simulator that solves both test mazes
end to end, so a change to the brain gets caught on a laptop before it ever
reaches the floor. The firmware cross-compiles clean at `-Wall -Wextra`.

## The board

RP2040, dual Cortex-M0+ at 125 MHz, 264 KB SRAM, 16 MB flash. Five reflectance
sensors on the front bar, quadrature encoders on both motors, an LSM6DSO IMU,
a 128x64 OLED, a buzzer, and six addressable RGB LEDs. It ships running
MicroPython; this replaces that entirely.

## What it does

Power on and you get a menu. C taps cycle through modes, C held starts one.
Any button stops the robot immediately, at any point.

**DIAG** shows live sensor bars, encoder counts, gyro rate, battery voltage,
and whether the current calibration is good enough to run on.

**CALIBRATE** takes the gyro bias with the robot held still, then spins it
across the line to learn the min and max each sensor sees. The firmware
refuses to explore on a calibration whose span is too narrow, which is what
usually happens when someone calibrates over paper only.

**EXPLORE** follows the line, stops at every junction, classifies it, and
applies the left-hand rule: take the leftmost available exit, and treat a
dead end as a 180. On a maze with no loops that reaches the goal by
construction rather than by luck. The path grows on the display as a string
of `L`, `R`, `S` and `B` while it runs.

**SOLVED** shows the recorded path next to the simplified one. Simplification
is arithmetic on the string. Every `B` is a dead end, and a dead end returns
the robot to the junction it left, so only the heading matters: add the three
turn angles, take them mod 360, and the answer is the single turn that
replaces them. `LBL` is 270 + 180 + 270 = 720, which is 0, so it becomes `S`.
Nine cases, one table, in `logic/maze_logic.c`.

**REPLAY** runs the simplified route junction by junction, so the dead ends
are never entered again. The lap timer runs, and the best time survives until
power off.

**LOG DUMP** prints the flight recorder over USB as CSV.

If the line disappears mid-correction the robot doesn't just stop. It
retraces its own encoder history backwards until it finds tape again, on a
fixed budget, and picks up where it left off. It only faults once that budget
is spent, so a momentary loss on worn tape costs a little distance instead of
the run.

## How it's put together

The split that matters is between `src/` and `logic/`.

`src/` is hardware. Registers, interrupt handlers, timing, I2C transactions.
It only compiles for the robot.

`logic/` is the maze brain: junction classification, the left-hand rule, path
recording, simplification, replay. It is plain C over plain structs with no
SDK headers anywhere in it, which means the same file that ships on the robot
also compiles on a laptop. That's why the test suite can be exhaustive and
still finish before you've let go of the return key.

Between them, `drive.c` does the motion: a PD line follower on a fixed 500 Hz
tick, gyro-closed-loop turns, and a junction creep that puts the axle over the
junction center before the classifier gets a look. It hands `logic/` two frozen
sensor snapshots per junction and then obeys whatever move comes back.

Roughly, one measurement gets promoted five times:

```
photons          IR bounced off tape or paper
  -> hw_line.c   charge, float, time the RC decay on all five pins at once
raw microseconds 0..1024 per sensor
  -> calibration
calibrated       0..1000 per sensor, scaled to this maze in this light
  -> drive.c     follow the line, latch the outer sensors at a junction
snapshots        two sensor_snapshot_t: the sweep and the settled read
  -> logic/      classify, decide, record, simplify
a route          "LSR"
```

Every tunable number lives in `include/tuning.h`, one file, with the reasoning
next to each constant. The hard speed cap is in `hw_motors.c` and nothing else
is allowed to raise it.

## Build

Once per clone:

```sh
tools/setup.sh
```

That fetches the pinned ARM GNU toolchain 14.2 and pico-sdk 2.2.0 into
`third_party/` (both gitignored) and installs cmake and picotool through
Homebrew. It's written for macOS on Apple Silicon; on anything else, change
the toolchain URL at the top.

Then:

```sh
cmake -S . -B build
make -C build -j8          # build/pololubot.uf2
make -C build flash        # robot on, USB-C plugged in
```

`flash` opens the robot's serial port at 1200 baud, which is the knock that
reboots an RP2040 into its ROM bootloader, then copies the `.uf2` onto the
drive that appears. If the firmware is too broken to answer the knock, hold
button B while pressing RESET and the drive shows up anyway. The bootloader
is in ROM, so the board can't actually be bricked.

## Tests

```sh
make -C tests/logic        # 119 unit checks + a 44-assertion maze simulator
make -C tests/telemetry    # CSV decode contract, python3 stdlib only
```

`tests/logic` compiles `logic/maze_logic.c` with the host compiler and runs it
against hand-built sensor snapshots, then against a simulator that models the
whole maze at sensor level, including a "marginal" palette where the tape has
worn and the contrast is close to the classification threshold.

Both classifiers run in the simulator: the real one, and the deliberately
crude `maze_logic_ref.c` kept as a cross-check. The Makefile pulls that file's
threshold constant out with `sed` and passes it to the simulator as a `-D`, so
the simulator's copy of the number can't silently drift from the source.

Some of these checks were written before the code they cover. The rest were
added afterwards, during hardening, and a test written after the code proves
nothing by passing, so each of those was first shown going red under a
deliberate mutation of the logic.

## Telemetry

Printing from a 2 ms control loop is a good way to make the robot wobble,
because a USB CDC write can block for milliseconds waiting on the host. So
nothing prints during a run. Every control tick appends a 16-byte record to a
fixed 8192-entry ring in SRAM. The append costs about a microsecond, and the
ring works out to roughly the last 16 seconds of history, oldest overwritten
first. It also costs 128 KB of the 264 KB of SRAM on the chip, which is a
lot, and worth it.

After the run, LOG DUMP prints the ring as CSV over the same USB serial port
used for flashing. `tools/capture_log.sh` handles the capture, and
`tools/plot_telemetry.py` turns the result into plots. The full column
contract is documented in `src/telemetry.h`.

Two ways to lose a run: the ring is in SRAM, so powering off between the run
and the dump erases it, and every GO resets the ring, so an explore you wanted
to read is gone the moment you start the replay after it.

## Layout

```
src/              drivers and motion, robot-only
logic/            the maze brain, builds on host and robot both
include/          pins.h (with the shared-pin traps annotated), tuning.h
docs/             layer notes: one per sensor, plus motion, search, path
tests/logic/      host tests and the maze simulator
tests/telemetry/  CSV decode contract and its fixture
tools/            setup, flash, capture, plot
cmake/            toolchain and SDK wiring
third_party/      Pololu's display and LED library
```

## References

- [3pi+ 2040 User's Guide](https://www.pololu.com/docs/0J86) — pinout and schematics
- [RP2040 datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
- [Pico SDK documentation](https://www.raspberrypi.com/documentation/pico-sdk/)
