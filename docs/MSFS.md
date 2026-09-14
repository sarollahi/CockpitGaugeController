# Microsoft Flight Simulator integration

CockpitGaugeController includes an external Python bridge for **Microsoft Flight Simulator 2020 and Microsoft Flight Simulator 2024**.

The bridge uses Microsoft's **SimConnect** API to read simulator variables and translates them into the same serial protocol used by the Arduino firmware and the X-Plane bridge.

```text
Microsoft Flight Simulator 2020 / 2024
        |
        | SimConnect
        v
msfs/cockpit_gauge_bridge.py
        |
        | USB serial @ 115200
        v
Arduino Mega 2560
        |
        v
physical gauges
```

No Arduino firmware changes are required when switching between X-Plane and MSFS.

## Why SimConnect

SimConnect is Microsoft's supported API for external applications and hardware integrations. The MSFS 2024 SDK explicitly supports legacy MSFS 2020 SimConnect clients, so this bridge uses the MSFS 2020-compatible API surface required by `pysimconnect` and avoids 2024-only calls.

Official documentation:

- MSFS 2024 SimConnect SDK: https://docs.flightsimulator.com/msfs2024/retail/programming-apis/simconnect/simconnect-sdk/
- Simulation variables: https://docs.flightsimulator.com/msfs2024/retail/programming-apis/simvars/simulation-variables/

## Requirements

- Windows 10 or Windows 11
- Microsoft Flight Simulator 2020 or Microsoft Flight Simulator 2024
- 64-bit Python 3
- Arduino Mega running CockpitGaugeController firmware
- USB serial connection to the Mega

Python dependencies are listed in `msfs/requirements.txt`:

```text
pysimconnect==0.2.6
pyserial>=3.5,<4
```

`pysimconnect` is an MIT-licensed Python wrapper for SimConnect. It is not vendored into this repository.

## Installation

Open PowerShell or Command Prompt in the repository and create a virtual environment:

```powershell
cd msfs
py -m venv .venv
.\.venv\Scripts\activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Start a flight in MSFS, connect the Arduino Mega, then run:

```powershell
python cockpit_gauge_bridge.py
```

The bridge waits for both sides independently:

- if MSFS is not running yet, it retries SimConnect;
- if the Arduino is not connected yet, it retries serial discovery;
- if either connection disappears, it attempts to reconnect without restarting the process.

## SimVar mapping

The bridge translates the following MSFS SimVars into the controller protocol:

| Controller token | MSFS SimVar | Requested unit | Conversion |
| --- | --- | --- | --- |
| `IAS` | `AIRSPEED INDICATED` | knots | direct |
| `T` | `TURN INDICATOR RATE` | radians/second | converted to degrees/second |
| `S` | `TURN COORDINATOR BALL` | native -127..127 position | normalized to configurable slip input |
| `ALT` | `INDICATED ALTITUDE` | feet | direct |
| `BARO` | `KOHLSMAN SETTING HG:1` | inHg | direct |
| `HDG` | `PLANE HEADING DEGREES GYRO` | degrees | normalized to 0..360 |
| `BUG` | `AUTOPILOT HEADING LOCK DIR` | degrees | normalized to 0..360 |
| `VSI` | `VERTICAL SPEED` | feet/minute | direct |

The bridge sends complete frames at 20 Hz by default:

```text
IAS:123.40 T:-1.800 S:0.700 ALT:5420.0 BARO:29.920 HDG:278.00 BUG:300.00 VSI:-450.0
```

## Turn/slip normalization

MSFS and X-Plane expose the slip/ball information differently.

The firmware protocol expects `S` as a compact signed slip input and applies its own mechanism gain. MSFS exposes `TURN COORDINATOR BALL` as an instrument position from approximately `-127` to `127`. The bridge therefore maps full ball deflection to `+/-4.0` protocol degrees by default:

```text
-127 -> -4.0
   0 ->  0.0
+127 -> +4.0
```

This value can be changed with `slip_full_scale_input_deg` in `config.json` without modifying firmware.

If the physical turn or slip indication moves in the opposite direction from MSFS, use `turn_invert` or `slip_invert` rather than rewiring the motor.

## Configuration

`msfs/config.json` contains all normal runtime settings:

```json
{
  "serial_port": "auto",
  "baud_rate": 115200,
  "update_hz": 20.0,
  "simconnect_reconnect_interval_s": 2.0,
  "serial_reconnect_interval_s": 2.0,
  "arduino_reset_delay_s": 2.0,
  "send_resume_on_connect": true,
  "park_on_exit": true,
  "park_wait_timeout_s": 10.0,
  "log_controller_messages": false,
  "turn_invert": false,
  "slip_invert": false,
  "slip_full_scale_input_deg": 4.0,
  "simconnect_dll": "auto"
}
```

### `serial_port`

`"auto"` searches for Arduino/Mega-like USB serial devices. If more than one plausible device is connected, specify the COM port explicitly:

```json
"serial_port": "COM5"
```

### `simconnect_dll`

Leave this as `"auto"` for normal use. `pysimconnect` supplies a compatible SimConnect DLL. An explicit DLL path can be supplied for testing or SDK-specific setups:

```json
"simconnect_dll": "C:\\path\\to\\SimConnect.dll"
```

### `park_on_exit`

When enabled, Ctrl+C or a normal process shutdown sends `PARK` to the Arduino and waits for the controller's clean-state confirmation before closing the serial connection.

This cannot help if the bridge process or PC is terminated abruptly, but it improves normal shutdown behavior.

## Serial auto-detection

The bridge ranks USB serial ports using:

- official Arduino vendor IDs,
- `Arduino` text in USB metadata,
- `Mega 2560` descriptors,
- common ACM/USB modem descriptors,
- CH340/WCH descriptors used by many compatible boards.

If no unique candidate can be identified, it deliberately does not guess. Configure the COM port explicitly instead.

## MSFS 2024 compatibility

Microsoft documents that SimConnect modules built against the legacy MSFS 2020 SDK continue to work in MSFS 2024, without access to new 2024-only features. This bridge only needs established aircraft SimVars and therefore intentionally stays on that compatible subset.

The Python `pysimconnect` wrapper itself predates MSFS 2024, so the bridge does not claim use of 2024-specific APIs. The compatibility comes from Microsoft's SimConnect backward-compatibility layer.

## Troubleshooting

### `Failed to open SimConnect`

Start MSFS and enter an active flight before launching the bridge. The bridge will keep retrying if MSFS is not yet available.

### `No unique Arduino serial port found`

Set `serial_port` in `msfs/config.json` to the Mega's Windows COM port, for example `COM5`.

### Gauges do not move after serial connection

Check that:

1. the Arduino firmware is running at 115200 baud;
2. MSFS is in an active flight;
3. the bridge logs both `Connected to Microsoft Flight Simulator through SimConnect` and `Connected to controller`;
4. another application such as Arduino Serial Monitor is not holding the COM port.

### Turn/slip direction is wrong

Set one or both of:

```json
"turn_invert": true,
"slip_invert": true
```

### Slip travel is too small or too large

Adjust:

```json
"slip_full_scale_input_deg": 4.0
```

The firmware still applies its own `SLIP_GAIN`, so this is simulator-side normalization rather than motor calibration.

### Controller messages are needed for debugging

Enable:

```json
"log_controller_messages": true
```

The bridge continuously drains controller output even when logging is disabled, preventing firmware diagnostic output from filling the Arduino USB serial transmit buffer.

## Design boundary

The MSFS bridge owns simulator-specific concerns:

- SimConnect lifecycle,
- SimVar subscription,
- MSFS-specific unit conversion,
- turn/slip normalization,
- Windows serial discovery and reconnect.

The Arduino owns hardware-specific concerns:

- stepper timing,
- gauge scaling,
- filtering,
- calibration,
- EEPROM persistence,
- park/recovery logic.

Keeping that boundary means MSFS and X-Plane can drive the exact same hardware firmware.
