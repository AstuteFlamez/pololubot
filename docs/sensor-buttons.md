# Buttons

Three buttons, two of which the firmware reads, both on pads that already
belong to something else.

**Source:** `src/hw_buttons.c`, `src/hw_buttons.h`
**Pins:** GP25 (button A, shared with the yellow LED), GP0 (button C, shared
with the OLED D/C line), QSPI_SS (button B — BOOTSEL, deliberately untouched)
**Consumers:** `main.c` (the menu), and **every motor loop in `drive.c`**

## Button B is not read on purpose

Button B is the BOOTSEL pin, not a normal GPIO. Leaving it alone means that
holding B while pressing RESET always drops the board into the RPI-RP2
bootloader — a recovery path no firmware bug can take away. The bootloader is
in ROM, so the board cannot be bricked by a bad flash.

## Reading a pad that is being driven

A pad that is being driven cannot be read. Both A and C are sampled through
the pad's **output-enable override**:

```
force the output driver off        (GPIO_OVERRIDE_LOW on OE)
wait 1 µs                          (let the pad settle)
read the level
restore the override               (GPIO_OVERRIDE_NORMAL)
```

This is the same technique Pololu's own driver uses. The display never
notices, because polling only ever happens *between* display transfers, never
inside one.

Two differences between the buttons:

- **A (GP25)** shares the yellow LED, which is active-low and driven. Floating
  the pad is enough — the button's own pull-up defines the high level.
- **C (GP0)** shares the OLED D/C line and needs the internal pull-up
  explicitly enabled to give the floated pin a defined high level for the
  button to pull away from. The pull-up is left enabled afterwards, matching
  Pololu's driver; it is weak enough that the display's driver still wins the
  pad.

`hw_buttons_init()` enables the **input buffers** on both pads (`gpio_get()`
reads through the input buffer, which is disabled on an output-only pad) and
deliberately does not set a pin function or direction — the LED and the
display keep whatever they configured.

**Both buttons read LOW when pressed.**

## The event model

`hw_buttons_poll()` returns at most one debounced event per call:

| Event | Meaning |
|---|---|
| `BTN_NONE` | nothing happened |
| `BTN_C_SHORT` | C pressed and released before the long-press threshold |
| `BTN_C_LONG` | C held 600 ms — delivered **once, while the button is still down** |
| `BTN_A_PRESS` | A pressed (on the press edge, not the release) |

The debounce is a per-button state machine, not a delay: an edge is accepted
only once `DEBOUNCE_MS` (20 ms) has passed since the last accepted edge on
that button. It never blocks the caller. C additionally splits into short and
long: the long event fires at the moment the threshold is crossed —
`LONGPRESS_MS` 600 — so the operator gets feedback while still holding, and
it suppresses the short event that would otherwise follow on release. One
press must never produce two events.

C is checked before A, so a simultaneous press reports C first.

Events are **edge-detected, not queued**. A caller that stops polling misses
presses rather than banking them, which is why the menu loop polls every
10 ms.

In the menu that becomes: **C tap → next mode, C hold → GO**, and A → GO as
well.

## `hw_buttons_any()` — the abort

```c
bool hw_buttons_any(void);   // undebounced, no bookkeeping, a couple of µs
```

This is the one that matters for safety. **Every** motor loop in `drive.c`
calls it on every iteration — the follower, the backtrack, the nudge, the
arrival brake, the creep, the turns — and returns `DRIVE_ABORT` with the
motors stopped. A finger on any button stops the robot, at any point, no
matter how wrong the surrounding code is. It is rule 1 of that file.

The calibration sweep and the DIAG loop poll it too.

## `hw_buttons_wait_release()` — and why it exists

```c
void hw_buttons_wait_release(void);   // blocks until nothing is down, then settles
```

Without it, the press that *starts* a run is immediately read back by
`hw_buttons_any()` as the abort that *stops* it. So it blocks until no button
is down, waits out 50 ms of contact rattle, and then clears the debounce
state so the release cannot be reported as a fresh event by the next poll.

`main.c` calls it at the top of every mode, inside `countdown()` ("the GO
press must not become a STOP"), and in `wait_for_button()` on both sides of
the wait — release, wait for a genuinely new press, release again.

## Failure modes

| Symptom | Cause |
|---|---|
| a run aborts the instant it starts | a missing `hw_buttons_wait_release()` after the GO press |
| one press advances the menu twice | debounce state not cleared, or `DEBOUNCE_MS` too short for the switch |
| C never registers | GP0 pull-up not enabled before the read, or the OLED driving the pad during the sample |
| buttons dead after adding display code | a display transfer that spans the poll — sample between transfers, never inside one |

An abort is recorded in the telemetry ring as an `ABORT` event whose `a` is
the tick kind it happened in (`1` follow, `2` blind, `3` creep, `4` turn,
`5` backtrack), so a dump says exactly which phase the operator stopped.
