# Battery monitor

Pack voltage through a resistor divider on ADC input 0. It is the cheapest
sensor on the robot and it prevents the most confusing class of bug.

**Source:** `src/hw_battery.c`, `src/hw_battery.h`
**Pin:** GP26 / ADC0 — **the same pad that switches the line-sensor emitters**
**Consumers:** `main.c` (`preflight()`, the menu and DIAG status lines, the
telemetry dump header)

## The measurement

The pack feeds a divider that lands VBAT/11 on GP26. One blocking 12-bit
conversion against the 3.3 V reference, tens of microseconds:

```c
mv = raw * 11 * 3300 / 4096;
```

The arithmetic is done in `uint32_t`; the largest intermediate
(4095 × 36300) is comfortably inside 32 bits.

Bring-up is idempotent — `adc_init()` resets the block, so it only runs when
the ADC is not already enabled — and `adc_gpio_init()` re-claims GP26 from
the line sensors on **every** conversion.

The pack is 4× NiMH AAA, 4.8 V nominal; a freshly charged set reads roughly
4800–5600 mV. Below ~1.1 V/cell the motors get weak and any PID tuning done
above that turns into fiction; below ~1.0 V/cell the readings are junk.

## The two thresholds

| Constant | Value | Behaviour |
|---|---|---|
| `BATT_WARN_MV` | 4400 | nag on screen; the robot still runs |
| `BATT_REFUSE_MV` | 4000 | `preflight()` refuses to start a run |

The refusal threshold is the one that matters. Below roughly 1.0 V per NiMH
cell the pack is empty and sags hard under motor load: a PWM level no longer
delivers the speed the tuning assumed, and a brownout reset mid-run is
indistinguishable from a firmware fault. Refusing to start is cheaper than
debugging that.

## Sag versus gain

This is the diagnostic the sensor exists for. A weak turn and a sagging pack
produce the same symptom — the robot under-rotates, junction after junction —
and only one of the two causes is free to fix.

So the first thing to read after a `"turn timeout"` is `batt_mv` in the dump
header, not `TURN_KP`. A `batt_mv` already down near `BATT_WARN_MV` means the
turns ran on sagging cells and the fix is fresh batteries.

**The converse does not hold.** A healthy resting number (4600+) does not
clear the batteries: resting voltage says nothing about internal resistance,
and a worn pack is exactly the one that rests high and then collapses the
moment both motors pull turn current — weak turns from the *first* turn
onward, which is the same picture an undersized `TURN_KP` paints. A high
resting number only takes the easy verdict off the table.

**The real discriminator is the trend.** Sag gets worse lap over lap on one
build — compare `TURN_END` residuals early and late in the same dump — while
an undersized gain is exactly as wrong on a full charge as on an empty one.

Retuning a gain against a flat battery is how a knob ends up with a reason
that was never true.

## GP26 discipline

The pin that switches the down-facing IR emitters **is** the pin the divider
feeds. See `include/pins.h` for the full trap and
[sensor-line.md](sensor-line.md) for the other side of the handover.

The handover is symmetric and stateless: this file calls `adc_gpio_init()`
before every conversion and leaves the pad in ADC mode; `hw_line.c` calls
`gpio_init()` and takes it back for the emitters on every read. Neither owner
assumes anything about the state it finds the pin in, so **call order never
matters**. What is not allowed is interleaving — a battery read taken in the
middle of a line read would switch the emitters off and measure a driven pin
instead of the divider.

## When it is read

- **The menu and DIAG**, once a second, as `bat 5.12V ok` / `LOW` / `DEAD`.
- **`preflight()`**, before every EXPLORE and REPLAY. A refusal calls
  `fault("battery dead")`.
- **LOG DUMP**, into the dump-facts header line as `batt_mv=`.

Each of `hw_battery_mv()`, `hw_battery_low()` and `hw_battery_critical()`
takes its own fresh reading.

## Reading `batt_mv` in a dump

The header describes **dump time, not run time**. The reading is taken with
the motors long stopped and the pack rested, so the loaded voltage during the
run was *lower still*. That makes a low number in the header damning rather
than borderline: if the pack reads 4300 mV at rest, it was well under that
while the motors pulled turn current.

## The preflight phantom

All three preflight refusals — `"battery dead"`, `"run CALIBRATE 1st"`,
`"no gyro cal"` — call `fault()` **before** either run loop resets the
telemetry ring. So a refusal appends a lone `FAULT` row onto whatever the
*previous* run left behind, a run whose own story may have ended perfectly
cleanly.

A trailing `FAULT` with no fresh `RUN_START` opening a new run is a refusal
at the menu, not a crash mid-run — and the last `RUN_START` in the dump may
not even be the mode that got refused. Do not debug the previous run for a
crash it never had.
