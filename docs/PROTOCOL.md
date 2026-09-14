# Serial Protocol

The controller uses newline-terminated ASCII at **115200 baud**.

## Telemetry line

A line can contain one or more space-separated tokens. Example:

```text
IAS:123.4 T:-1.8 S:0.7 ALT:5420 BARO:29.92 HDG:278 BUG:300 VSI:-450
```

A host may send only the values that changed, or send a complete frame.

| Token | Value | Unit / meaning |
| --- | --- | --- |
| `IAS:<value>` | float | indicated airspeed in knots |
| `T:<value>` | float | turn input used by the TURN mapping |
| `S:<value>` | float | slip angle/input |
| `ALT:<value>` | float | altitude in feet |
| `BARO:<value>` | float | barometric setting in inHg |
| `HDG:<value>` | float | heading in degrees; normalized to 0-360 |
| `BUG:<value>` | float | heading-bug angle in degrees; normalized to 0-360 |
| `VSI:<value>` | float | vertical speed in ft/min |
| `VVI:<value>` | float | alias for `VSI:` |

## Calibration commands

### `CAL:ZERO`

Rebases all current software motor positions to zero. This is a software coordinate operation, not physical sensing. The position confidence is deliberately made conservative afterward until the relationship is visually verified.

### `CAL:SLIPCENTER`

Sets the current SLIP motor position as the slip trim reference.

### `CAL:SAVE`

Immediately stores trim calibration values to EEPROM.

### `CAL:LOAD`

Reloads trim calibration values from EEPROM.

### `TEST:BUTTONS`

Starts a 10-second non-blocking button diagnostic. Motor servicing continues while the test is active.

## Position and shutdown commands

### `PARK`

Requests park targets for all enabled gauges. When the motors have reached target and settled, the controller stores a clean position snapshot and prints:

```text
PARKED. Position saved CLEAN. Safe to power off.
```

### `SHUTDOWN`

Alias of `PARK`.

### `RESUME`

Leaves explicit park mode and resumes normal target processing.

### `POS:STATUS`

Prints:

- current position-confidence state,
- persisted-state status,
- current software position of every enabled axis.

Position confidence is reported as either:

```text
EXACT/TRACKED
```

or:

```text
ESTIMATED
```

### `POS:SAVE`

Writes a stationary clean position snapshot. The command is rejected while motors are still moving.

### `POS:TRUST`

Marks the current physical/software relationship as trusted and stores it. Use this only after visually checking the instrument positions. The command is rejected while motors are moving.

## BARO service commands

### `BAROSTEP:<steps>`

Moves the BARO target by the requested relative number of steps and enters manual BARO override mode.

Example:

```text
BAROSTEP:100
```

### `BAROSET:<hPa>`

Moves BARO to a target derived from a pressure value in hPa and enters manual BARO override mode.

Example:

```text
BAROSET:1013.25
```

Manual BARO override remains active until a normal `BARO:<inHg>` telemetry token arrives.

## Parser behavior

The serial implementation is designed not to dominate stepper servicing:

- fixed 221-byte RX buffer,
- no Arduino `String` allocation,
- overflowed lines are discarded,
- maximum of 48 serial bytes processed per main-loop pass,
- CR and LF line termination supported.

## Host integration

The firmware does not require a specific simulator plugin. Any program capable of obtaining simulator telemetry and writing the protocol above to a serial port can drive the controller.

For X-Plane 12, this repository includes `xplane/PI_CockpitGaugeController.py`, an XPPython3 bridge that reads the required X-Plane DataRefs and emits complete frames at 20 Hz by default. See [X-Plane integration](XPLANE.md).

A host adapter should:

1. Open the Arduino serial port at 115200 baud.
2. Send newline-terminated frames.
3. Prefer a stable update cadence rather than unbounded serial flooding.
4. Continuously read/drain controller output if the serial port remains open.
5. Send `RESUME` after reconnecting if the controller may have been parked.
6. Send `PARK` before an intentional controller power-down when possible.
