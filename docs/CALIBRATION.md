# Calibration and Position Recovery

## Two different kinds of persisted state

The firmware stores two independent concepts in EEPROM.

### Trim calibration

Each axis has its own trim offset:

- ASI
- TURN
- SLIP
- ALT
- BARO
- DG
- BUG
- VSI

The three physical buttons change the selected gauge trim. Calibration is automatically written after a short inactivity delay, and can also be saved immediately with:

```text
CAL:SAVE
```

### Motor-position state

The software also keeps an open-loop estimate of every stepper's current position. These positions are written into a rotating EEPROM journal rather than repeatedly overwriting a single record.

Each record contains:

- format magic,
- version,
- state flags,
- monotonic sequence,
- eight axis positions,
- checksum.

The newest valid record is selected on boot.

## Exact vs estimated

The controller intentionally distinguishes between two confidence states.

### `EXACT/TRACKED`

This means the physical/software relationship has been explicitly trusted and then tracked through commanded motion.

### `ESTIMATED`

This means the controller has a useful step-count estimate, but cannot prove that the physical shaft is still exactly where software believes it is.

Open-loop steppers have no absolute position feedback. Software persistence cannot detect:

- a manually rotated shaft,
- missed steps,
- an unexpected power loss during a step,
- mechanical slip.

The firmware therefore does not claim that a checkpoint magically provides absolute feedback.

## Recommended initial setup

1. Start the controller.
2. Send known reference values from the host.
3. Use D13 to select each gauge.
4. Use D11/D12 to align the physical indication.
5. Wait for automatic trim persistence or send `CAL:SAVE`.
6. Confirm every physical gauge matches the intended value.
7. Stop gauge motion.
8. Send:

```text
POS:TRUST
```

The controller can now treat the software-to-physical relationship as trusted.

## Normal shutdown

Before cutting power, send:

```text
PARK
```

or:

```text
SHUTDOWN
```

Wait until the controller prints:

```text
PARKED. Position saved CLEAN. Safe to power off.
```

Then remove power.

## Unexpected power loss

When the controller is active, a previously clean position record is invalidated before physical movement begins. While stationary, periodic checkpoints can reduce how much position history is lost if power disappears.

On the next boot, the newest valid record is restored. If the last state cannot be treated as fully authoritative, position confidence remains `ESTIMATED`.

Use:

```text
POS:STATUS
```

to inspect the current state.

If the physical gauges still match the expected simulator values, stop motion and send:

```text
POS:TRUST
```

If they do not match, recalibrate instead of trusting the stored estimate.

## `CAL:ZERO` is not a sensor

`CAL:ZERO` changes the firmware coordinate system by declaring the current software positions to be zero. It does not discover the physical angle.

After `CAL:ZERO`, visually verify the physical state before using `POS:TRUST`.

## Heading axes

DG and BUG can rotate through 360°, so a mechanical hard-stop home is not appropriate for those axes. Their normal operation uses continuous software tracking, circular filtering, and nearest-equivalent target selection.

With zero additional hardware, absolute recovery after arbitrary physical movement is fundamentally impossible. The current design chooses explicit confidence reporting rather than hiding that limitation.

## EEPROM write strategy

The firmware reduces EEPROM wear in several ways:

- trim saves are delayed until adjustments stop,
- position checkpoints are rate limited,
- position snapshots are written only while motors are stationary,
- position records rotate across the available journal area,
- `EEPROM.put()` benefits from AVR update semantics for unchanged bytes.
