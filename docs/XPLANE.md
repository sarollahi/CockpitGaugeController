# X-Plane integration

CockpitGaugeController includes a native **X-Plane 12 bridge for XPPython3**. It does not require XPLDirect.

The integration keeps the Arduino firmware simulator-independent:

```text
X-Plane 12
    |
    | standard X-Plane DataRefs
    v
XPPython3
    |
    | PI_CockpitGaugeController.py
    v
USB serial @ 115200 baud
    |
    v
Arduino Mega 2560
    |
    v
8 physical gauge axes
```

## Requirements

- X-Plane 12
- XPPython3 4.x
- CockpitGaugeController firmware on an Arduino Mega 2560
- `pyserial` (the plugin can request installation through XPPython3 automatically)

XPPython3 itself is **not** bundled with this repository. Install the current version from its official documentation:

- https://xppython3.readthedocs.io/en/latest/usage/installation_plugin.html

## Install the bridge

After XPPython3 has created its `PythonPlugins` directory, copy these repository items:

```text
xplane/PI_CockpitGaugeController.py
xplane/CockpitGaugeController/
```

into:

```text
<X-Plane>/Resources/plugins/PythonPlugins/
```

The final layout should be:

```text
<X-Plane>/
└── Resources/
    └── plugins/
        ├── XPPython3/
        └── PythonPlugins/
            ├── PI_CockpitGaugeController.py
            └── CockpitGaugeController/
                └── config.json
```

Restart X-Plane or reload XPPython3 plugins.

If `pyserial` is not installed in XPPython3's Python environment, the bridge asks XPPython3 to install `pyserial>=3.5`. After installation finishes, reload XPPython3 plugins or restart X-Plane.

## DataRefs

The bridge reads the following standard cockpit DataRefs:

| Firmware token | X-Plane DataRef | Unit sent to firmware |
| --- | --- | --- |
| `IAS` | `sim/cockpit2/gauges/indicators/airspeed_kts_pilot` | knots |
| `T` | `sim/cockpit2/gauges/indicators/turn_rate_roll_deg_pilot` | X-Plane turn indicator value |
| `S` | `sim/cockpit2/gauges/indicators/slip_deg` | degrees |
| `ALT` | `sim/cockpit2/gauges/indicators/altitude_ft_pilot` | feet |
| `BARO` | `sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot` | inHg |
| `HDG` | `sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot` | degrees |
| `BUG` | `sim/cockpit2/autopilot/heading_dial_deg_mag_pilot` | degrees |
| `VSI` | `sim/cockpit2/gauges/indicators/vvi_fpm_pilot` | ft/min |

At the default 20 Hz update rate, the bridge sends a complete frame such as:

```text
IAS:123.40 T:-1.800 S:0.700 ALT:5420.0 BARO:29.920 HDG:278.00 BUG:300.00 VSI:-450.0
```

The Arduino remains responsible for filtering, mapping values to mechanism geometry, trim, wrap handling, parking, and position recovery.

## Serial auto-detection

`config.json` defaults to:

```json
"serial_port": "auto"
```

Auto-detection prioritizes:

1. USB devices with Arduino/Genuino vendor IDs.
2. Ports whose USB description contains `Arduino` or `Mega 2560`.
3. Common Arduino-style `usbmodem` / `ttyACM` devices.
4. A single unambiguous USB serial device as a conservative fallback.

If more than one candidate exists or your clone is not recognized, set the port explicitly.

Examples:

**macOS**

```json
"serial_port": "/dev/cu.usbmodem1101"
```

**Linux**

```json
"serial_port": "/dev/ttyACM0"
```

**Windows**

```json
"serial_port": "COM5"
```

## Configuration

`xplane/CockpitGaugeController/config.json` supports:

| Option | Default | Purpose |
| --- | ---: | --- |
| `serial_port` | `"auto"` | Automatic or explicit serial device selection |
| `baud_rate` | `115200` | Must match the firmware |
| `update_hz` | `20.0` | X-Plane telemetry frames per second |
| `reconnect_interval_s` | `2.0` | Retry interval after disconnect/no device |
| `arduino_reset_delay_s` | `2.0` | Delay after opening the Mega's serial port |
| `park_on_plugin_disable` | `true` | Send `PARK` when the X-Plane plugin is disabled |
| `send_resume_on_connect` | `true` | Send `RESUME` after each successful connection |
| `log_controller_messages` | `false` | Mirror Arduino serial output into XPPython3's log |

The bridge clamps unsafe configuration ranges rather than allowing a bad JSON value to create an unbounded callback rate.

## Connection behavior

Opening the serial port may reset an Arduino Mega. The bridge therefore:

1. Opens the selected port.
2. Waits for `arduino_reset_delay_s`.
3. Continuously drains startup/debug output from the controller.
4. Sends `RESUME` once the controller is ready.
5. Starts 20 Hz telemetry streaming.

If USB is unplugged or the serial device disappears, the bridge closes the failed connection and retries at the configured reconnect interval.

Serial writes have a short timeout so a blocked USB connection does not stall X-Plane's flight loop indefinitely.

## Shutdown behavior

With the default configuration, disabling the plugin sends:

```text
PARK
```

before closing the serial port. The Arduino then parks the enabled gauges and records its clean position state independently.

The bridge does not wait synchronously for the gauges to finish parking because blocking X-Plane while physical motors move would be undesirable. Keep controller power on long enough for the firmware to finish parking before removing power.

## Logging and troubleshooting

XPPython3 logs plugin messages to its normal XPPython3 log. Search for:

```text
[CockpitGaugeController]
```

Typical messages include:

```text
Connected to /dev/cu.usbmodem1101 at 115200 baud; waiting 2.0s for controller reset.
No suitable Arduino serial port found...
Could not open serial port ...
```

If `log_controller_messages` is enabled, lines emitted by the Arduino are also logged with a `controller:` prefix.

### No serial port found

Set `serial_port` explicitly in `config.json`. This is preferable to making auto-detection aggressively select an unrelated USB serial device.

### Plugin says pyserial is missing

Allow XPPython3's package installer to finish, then reload XPPython3 plugins or restart X-Plane.

### X-Plane shows the plugin but gauges do not move

1. Confirm the Arduino firmware is running at 115200 baud.
2. Check XPPython3's log for a successful serial connection.
3. Enable `log_controller_messages` temporarily.
4. Confirm that the serial port is not already open in Arduino Serial Monitor or another application.
5. Send `POS:STATUS` manually if you need to inspect the controller's recovery state.
