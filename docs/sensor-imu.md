# IMU (gyro)

An LSM6DSO on the I2C bus. Only the **Z-axis gyro** is used, and only for one
job: turning the robot exactly 90° or 180° in place.

**Source:** `src/hw_imu.c`, `src/hw_imu.h`
**Bus:** i2c0 at 400 kHz, GP4 SDA / GP5 SCL, address `0x6B`
**Consumers:** `src/drive.c` (`turn_left_90`, `turn_right_90`,
`turn_around_180`), the DIAG screen

## Why the gyro and not the encoders

Encoders measure **wheel** rotation. On a dusty posterboard the wheels slip
during an in-place turn, and the error accumulates junction after junction
until the robot turns onto empty floor. The gyro measures the **body's**
rotation directly, which is slip-proof.

Turns are also **relative** — `target = current angle + delta` — so heading
drift accumulated while the robot sits still between junctions never enters
the turn.

## Register map

Cross-checked against the LSM6DSO datasheet and Pololu's driver for this
board:

| Register | Addr | Value written / meaning |
|---|---|---|
| `WHO_AM_I` | `0x0F` | reads `0x6C` |
| `CTRL3_C` | `0x12` | `0x44` = BDU + IF_INC |
| `CTRL2_G` | `0x11` | `0x5C` = ODR 208 Hz, FS ±2000 dps |
| `STATUS` | `0x1E` | bit 1 (GDA) = new gyro sample available |
| `OUTZ_L_G` | `0x26` | little-endian `int16`, low byte first |

At ±2000 dps the sensitivity is **70 mdps/LSB**.

**BDU** (block data update) matters because the Z reading is two bytes
fetched in one burst: without it the chip may refresh the high byte between
the two, producing a value that never existed. **IF_INC** is what makes that
burst walk `0x26 → 0x27` instead of re-reading the same register.

Register reads use a repeated start: write the address with no STOP (holding
the bus), then turn the bus around and read.

## Heading is integrated, not measured

```c
rate_dps  = (raw - bias_raw) * 70.0f / 1000.0f;
angle_deg += rate_dps * dt_us * 1e-6f;
```

Two consequences shape the whole driver.

**1. Any constant offset integrates into unbounded drift.**
`hw_imu_calibrate()` averages 256 samples at 208 Hz — about 1.2 s — with the
robot **dead still**, and subtracts that bias from every later sample. This
is why CALIBRATE does the gyro *first*, before any spinning has the chassis
rocking, and why the screen says "hold still" and then waits 400 ms for the
GO-press wobble to die out.

Calibration cannot remove the bias for good: it moves with temperature. An
integrated heading therefore degrades the longer it runs since the last
`hw_imu_calibrate()` or `hw_imu_reset_angle()`. Turns being relative is what
keeps that from mattering.

**2. `dt` is measured, never assumed.** The caller polls on its own schedule,
so every consumed sample is timestamped with the µs timer and the real
interval is used. Assuming the nominal 208 Hz would silently mis-scale the
angle whenever a sample arrived late or was dropped.

`hw_imu_update()` never blocks: it returns `false` when no sample is ready.
Samples are **not queued** — a caller slower than 208 Hz loses angle rather
than accumulating it, so the turn loop polls it every pass.

## Sign convention

The gyro's Z axis points up out of the board, so **counterclockwise (a left
turn) is positive**, and so is the integrated heading. It is not wrapped:
multiple turns keep accumulating past ±360.

`turn_relative(+90)` is a left turn, `-90` a right turn, `+180` an
about-face. In the turn loop, positive error means "need CCW", which commands
`hw_motors_set(-speed, +speed)` — left wheel back, right wheel forward.

## A missing IMU is not fatal

`hw_imu_init()` returns `false` if the chip does not answer its `WHO_AM_I`.
Every other call then becomes a no-op: the angle stays 0 and
`hw_imu_update()` always returns `false`. The robot still boots, still drives,
still shows DIAG — it just cannot turn by angle. `main.c` reports the result
on the boot splash (`imu: ok` / `imu: MISSING`) rather than halting, and
`preflight()` refuses to start a run with `"no gyro cal"`.

One caveat worth knowing: `hw_imu_calibrate()` waits on the data-ready flag
with **no timeout**, so a bus that dies mid-calibration hangs there. The
presence check at init is what normally keeps that path unreachable.

## The turn control law

In `drive.c`, `turn_relative()` runs PD on the integrated angle:

```c
speed = err * TURN_KP - hw_imu_rate_dps() * TURN_KD;   // clamped to ±SPEED_TURN_MAX
```

| Constant | Value |
|---|---|
| `TURN_KP` | 140 |
| `TURN_KD` | 4 |
| `TURN_TOL_DEG` | 3 |
| `TURN_SETTLE_MS` | 250 |
| `TURN_TIMEOUT_MS` | 2500 |
| `SPEED_TURN_MAX` | 2500 |

The D term reads the gyro's **rate** directly rather than differencing the
angle, so this loop needs no fixed period — unlike the line follower, which
does (see [motion.md](motion.md)).

"Settled" means inside ±`TURN_TOL_DEG` *continuously* for `TURN_SETTLE_MS`.
One in-tolerance sample is not enough: the robot swings through the target
with momentum, and what the next phase needs is a chassis **stopped** there.

## Failure modes

| Symptom | Cause | Where it shows |
|---|---|---|
| `imu: MISSING` on the splash | `WHO_AM_I` did not answer — bus, address, or a dead chip | boot screen; DIAG shows `!!` beside the gyro angle |
| `"no gyro cal"` | IMU absent, or CALIBRATE never run this power cycle | preflight refusal — a bare `FAULT` row with no fresh `RUN_START` |
| `"turn timeout"` | the turn never settled | `TURN_START` with no `TURN_END`, `T` rows whose `a` (deg ×10) stalls short of zero, `TIMEOUT [phase=T]` |
| turns consistently short/long | bias drift, or a stepped `TURN_KP` fighting a sagging pack | check `batt_mv` in the dump header **before** reaching for `TURN_KP` — a sagging pack and a weak gain look identical |
| heading wanders in DIAG | normal integrated drift since the last calibration | watch the rate, not the angle |

## Where to look

- Live: **DIAG** row 6 shows `gyro <angle>` with `!!` appended if the IMU is
  absent.
- After a run: `T` rows carry angle error (deg ×10) in `a` and commanded
  speed in `b`, decimated to ~60 Hz so a long spin cannot flush the ring.
  `TURN_START` carries the target delta, `TURN_END` the residual error and
  the milliseconds taken — a large residual on a `TURN_END` means the turn,
  not the follower, is what left the robot mispointed.
