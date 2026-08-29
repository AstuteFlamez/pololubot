# docs

Reference notes for the pololubot firmware — one document per sensor, one
for motion, one for the maze search, one for the shortest path.

The top-level `README.md` is the tour: what the robot does and how to build
it. These are the layer-by-layer notes underneath it, for someone who has to
change a number, read a telemetry dump, or work out why a run faulted.

## The documents

| Document | Covers | Source |
|---|---|---|
| [sensor-line.md](sensor-line.md) | the five reflectance sensors, RC decay, calibration | `src/hw_line.c` |
| [sensor-encoders.md](sensor-encoders.md) | quadrature wheel encoders, distance | `src/hw_encoders.c` |
| [sensor-imu.md](sensor-imu.md) | LSM6DSO gyro, integrated heading | `src/hw_imu.c` |
| [sensor-battery.md](sensor-battery.md) | pack voltage, the two thresholds | `src/hw_battery.c` |
| [sensor-buttons.md](sensor-buttons.md) | buttons A/C on shared pads, debounce | `src/hw_buttons.c` |
| [motion.md](motion.md) | follower, junction detect, recovery, creep, turns | `src/drive.c`, `src/hw_motors.c` |
| [maze-search.md](maze-search.md) | classification, the left-hand rule, recording | `logic/maze_logic.c` |
| [shortest-path.md](shortest-path.md) | simplification and replay | `logic/maze_logic.c` |

Two files sit under everything and get no document of their own:
`src/hw_millis.c` (the RP2040 microsecond timer, read directly — every
timeout and every `dt` in the tree comes from it) and `include/tuning.h`
(every tunable constant, with the reasoning next to each one and a dated
tuning log at the bottom). Numbers quoted in these documents are the shipped
values; `tuning.h` is the authority if they ever disagree.

## The stack, in one picture

```
                         main.c          menu, run loops, faults
                            |
              +-------------+-------------+
              |                           |
          logic/                        src/
   classify, decide, record,     drive.c   motion, one seam type
   simplify, replay              hw_*.c    registers and interrupts
   (host + robot)                (robot only)
```

The seam between the two halves is two `sensor_snapshot_t` per junction, in
and one `move_t` back out. `logic/` includes no SDK header, so the same
source that ships on the robot compiles with plain `gcc` under `tests/logic/`
— 163 checks, including a sensor-level simulator that solves both test mazes
end to end.

## How one measurement travels

```
photons          IR bounced off tape or paper
  -> hw_line.c   charge, float, time the RC decay on five pins at once
raw microseconds 0..1024 per sensor           sensor-line.md
  -> calibration
calibrated       0..1000 per sensor
  -> drive.c     follow, latch the outer sensors at a junction   motion.md
snapshots        two sensor_snapshot_t: the sweep and the settled read
  -> logic/      classify, decide, record             maze-search.md
a raw route      "LLBLLBS"
  -> logic/      collapse the dead-end detours      shortest-path.md
a solved route   "LSR"
```

## Reading a run after the fact

Nothing prints while the wheels turn. Every control tick appends a 16-byte
record to an 8192-entry SRAM ring (~16 s of history); **LOG DUMP** prints it
as CSV over USB afterwards. The column contract is specified in
`src/telemetry.h` and nowhere else — `tools/plot_telemetry.py` reads it and
`tests/telemetry/check_decode.py` pins it. Each document below ends with the
dump signature of its own failures.
