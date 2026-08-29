# Wheel encoders

Quadrature encoders on both motor shafts. They are how the robot measures
**distance**, and every distance-bounded window in the firmware — blind,
grace, decay, nudge, creep, arrival brake — is counted in encoder counts, not
milliseconds.

**Source:** `src/hw_encoders.c`, `src/hw_encoders.h`
**Pins:** GP8/GP9 (right A/B), GP12/GP13 (left A/B)
**Consumers:** `src/drive.c` (every measured window, the backtrack breadcrumb
trail, the creep's straightness servo)

## The decode

Each channel is a magnetic disc on the motor shaft giving 12 counts per motor
revolution, through a ~29.86:1 gearbox. Decoding **all four edges** of each
quadrature cycle is what turns that into ~358.3 counts per wheel revolution —
enough resolution that a 45 mm creep is a 160-count target rather than a
16-count one.

Every edge on any of the four pins fires one interrupt. The handler reads
both pins of both channels from a single snapshot of the GPIO input register
and indexes a 16-entry table with `(previous AB << 2) | current AB`:

```c
static const int8_t QDEC[16] = {
     0, +1, -1,  0,
    -1,  0,  0, +1,
    +1,  0,  0, -1,
     0, -1, +1,  0,
};
```

The four legal gray-code steps yield ±1. The zeros are two different things:
the no-change cases (indices 0, 5, 10, 15) and the **impossible diagonal
jumps**, which mean an edge was missed or a contact bounced. Those yield 0,
so noise stops the count from advancing instead of corrupting it.

One register read rather than four `gpio_get()` calls is not only cheaper —
it samples both pins of a channel at the same instant, which a sequence of
separate reads would not.

`hw_encoders_init()` seeds `state_left`/`state_right` from the pins' current
levels, so the first real edge decodes against reality instead of a zeroed
history.

## Why interrupts

At speed-run pace the wheels produce an edge roughly every 100 µs, while the
main loop is off doing a line-sensor read that can take 1.1 ms. Polling would
drop counts wholesale — and a dropped count is not noise, it is a permanently
short distance measurement, which shifts every window downstream of it.

The counters are `volatile` because the ISR writes them behind the main
loop's back. Reading them needs no critical section: a 32-bit load is a
single instruction on Cortex-M0+, so it cannot tear.

`hw_encoders_init()` installs the RP2040's **single shared GPIO callback**
via `gpio_set_irq_enabled_with_callback()`. Any other module that wants GPIO
interrupts has to cooperate with this handler rather than register its own.

## Scale and sign

| Constant | Value | Means |
|---|---|---|
| `ENC_COUNTS_PER_MM_X100` | 356 | 3.56 counts per mm (the ×100 keeps the math integer) |
| `ENC_SIGN_LEFT` | +1 | flip if forward motion makes this wheel count down |
| `ENC_SIGN_RIGHT` | +1 | same |

The signs are applied in the accessors, so `hw_encoder_left()` and
`hw_encoder_right()` both count **up** for forward motion no matter how the
encoders are wired. Every mm→counts conversion in `drive.c` goes through one
macro:

```c
#define MM_TO_COUNTS(mm) ((mm) * ENC_COUNTS_PER_MM_X100 / 100)
```

To check a sign: spin each wheel forward by hand and watch the two counts on
the **DIAG** screen. A wrong sign is not subtle — the creep never reaches its
target and faults `"creep stalled"`.

`hw_encoders_reset()` zeroes both counters with separate stores, so an edge
landing between them is credited to one wheel and not the other. That is one
count out of 356 per mm, and no caller cares.

## What the encoders are trusted with

| Use | Window | Why distance and not time |
|---|---|---|
| post-turn blind window | `BLIND_MM` 30 | the same window must work at explore and replay speed |
| junction creep | `CREEP_MM` 45 | it is a geometric offset — the bar rides ahead of the axle |
| arrival brake | `ARRIVAL_BRAKE_MM` 20 | its real cost is *position*: every mm here moves the deciding read |
| resume grace | `RESUME_GRACE_MM` 15 | measured from a fresh baseline after a recovery |
| recovery budget decay | `RECOVERY_DECAY_MM` 200 | clean travel earns credits back; a tight loop earns nothing |
| resume nudge | `RESUME_NUDGE_MM` 20 | bounded search forward for refound tape |
| creep/brake straightness | `CREEP_BAL_GAIN` 8 | left−right count difference fed back as a steering trim |

The backtrack recovery is the encoders' most unusual use: `drive.c` records
the **per-tick encoder delta of each wheel** into a 1024-entry ring (about 2 s
of history at the control rate) and replays it newest-first as a retreating
target, P-chasing it. That is closed loop on odometry rather than on duty, so
stiction and battery sag cannot bend the retraced path. The deltas are stored
one signed byte per wheel; at the control rate a tick's real travel is a
handful of counts, so nothing mechanical can reach the ±127 clamp — only an
encoder discontinuity can, and a clamped breadcrumb still retraces in the
right direction.

## Failure modes

| Symptom | Cause | Where it shows |
|---|---|---|
| `"creep stalled"` | jammed wheel, blocked chassis, or a flipped `ENC_SIGN_*` | `CREEP_START`, then `C` rows whose `a` (progress in counts) stops climbing, then `TIMEOUT [phase=C]` |
| every distance short | a channel not counting (broken wire, missed IRQ install) | one DIAG count frozen while the other moves |
| turns drift junction after junction | not an encoder problem — turns are on the gyro for exactly this reason | see [sensor-imu.md](sensor-imu.md) |

## Where to look

- Live: **DIAG** prints both counts.
- After a run: `C` rows carry creep/brake progress in counts in column `a`
  and the straightness trim in `b`. `K` (backtrack) rows carry the per-wheel
  chase error in `a` and `b`.
