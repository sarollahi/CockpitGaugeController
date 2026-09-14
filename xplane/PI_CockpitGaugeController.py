"""
Cockpit Gauge Controller - X-Plane bridge for XPPython3.

Reads standard X-Plane cockpit DataRefs and streams them to the Arduino Mega
firmware using CockpitGaugeController's newline-delimited ASCII protocol.

Copyright (C) 2026 sarollahi
SPDX-License-Identifier: GPL-2.0-only
"""

from __future__ import annotations

import json
import os
import time
from typing import Any, Dict, Optional

from XPPython3 import xp

try:
    import serial
    from serial.tools import list_ports
except ModuleNotFoundError:
    serial = None
    list_ports = None


PLUGIN_NAME = "Cockpit Gauge Controller"
PLUGIN_SIGNATURE = "sarollahi.cockpitgaugecontroller.xplane"
PLUGIN_DESCRIPTION = "Streams X-Plane flight-instrument DataRefs to CockpitGaugeController hardware."
LOG_PREFIX = "[CockpitGaugeController]"

DEFAULT_CONFIG: Dict[str, Any] = {
    "serial_port": "auto",
    "baud_rate": 115200,
    "update_hz": 20.0,
    "reconnect_interval_s": 2.0,
    "arduino_reset_delay_s": 2.0,
    "park_on_plugin_disable": True,
    "send_resume_on_connect": True,
    "log_controller_messages": False,
}

DATAREFS = {
    "IAS": "sim/cockpit2/gauges/indicators/airspeed_kts_pilot",
    "T": "sim/cockpit2/gauges/indicators/turn_rate_roll_deg_pilot",
    "S": "sim/cockpit2/gauges/indicators/slip_deg",
    "ALT": "sim/cockpit2/gauges/indicators/altitude_ft_pilot",
    "BARO": "sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot",
    "HDG": "sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot",
    "BUG": "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot",
    "VSI": "sim/cockpit2/gauges/indicators/vvi_fpm_pilot",
}

ARDUINO_VIDS = {0x2341, 0x2A03}


def _log(message: str) -> None:
    xp.log(f"{LOG_PREFIX} {message}")


def _safe_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _safe_bool(value: Any, default: bool) -> bool:
    if isinstance(value, bool):
        return value
    return default


def _format_frame(values: Dict[str, float]) -> str:
    """Create one complete firmware telemetry frame."""
    return (
        f"IAS:{values['IAS']:.2f} "
        f"T:{values['T']:.3f} "
        f"S:{values['S']:.3f} "
        f"ALT:{values['ALT']:.1f} "
        f"BARO:{values['BARO']:.3f} "
        f"HDG:{values['HDG']:.2f} "
        f"BUG:{values['BUG']:.2f} "
        f"VSI:{values['VSI']:.1f}\n"
    )


class PythonInterface:
    def __init__(self):
        self.config = dict(DEFAULT_CONFIG)
        self.datarefs: Dict[str, Any] = {}
        self.serial_connection = None
        self.connected_port: Optional[str] = None
        self.serial_ready_at = 0.0
        self.resume_sent = False
        self.next_connect_attempt = 0.0
        self.flight_loop_registered = False
        self.rx_text_buffer = ""

        self.plugin_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_path = os.path.join(
            self.plugin_dir, "CockpitGaugeController", "config.json"
        )

    def XPluginStart(self):
        self._load_config()
        return PLUGIN_NAME, PLUGIN_SIGNATURE, PLUGIN_DESCRIPTION

    def XPluginEnable(self):
        global serial, list_ports

        if serial is None or list_ports is None:
            _log("pyserial is not installed. Opening XPPython3's package installer.")
            try:
                from XPPython3.utils import xp_pip

                xp_pip.load_requirements(
                    "pyserial>=3.5",
                    start_message=(
                        "Cockpit Gauge Controller requires pyserial.\n"
                        "Installing the missing package..."
                    ),
                    end_message=(
                        "pyserial installation requested.\n"
                        "Reload XPPython3 plugins or restart X-Plane after installation finishes."
                    ),
                )
            except Exception as exc:
                _log(f"Unable to start pyserial installation: {exc}")
            return 0

        if not self._resolve_datarefs():
            return 0

        interval = self._flight_loop_interval()
        xp.registerFlightLoopCallback(self._flight_loop, interval, 0)
        self.flight_loop_registered = True
        self.next_connect_attempt = 0.0
        _log(f"Enabled. Telemetry rate: {1.0 / interval:.1f} Hz.")
        return 1

    def XPluginDisable(self):
        if self.flight_loop_registered:
            xp.unregisterFlightLoopCallback(self._flight_loop, 0)
            self.flight_loop_registered = False

        if self.config.get("park_on_plugin_disable", True):
            self._send_control_command("PARK")
            self._flush_serial_output()

        self._disconnect("plugin disabled")

    def XPluginStop(self):
        self._disconnect("plugin stopped")

    def XPluginReceiveMessage(self, inFromWho, inMessage, inParam):
        pass

    def _load_config(self) -> None:
        self.config = dict(DEFAULT_CONFIG)

        try:
            with open(self.config_path, "r", encoding="utf-8") as config_file:
                loaded = json.load(config_file)
        except FileNotFoundError:
            _log(f"Config not found at {self.config_path}; using defaults.")
            return
        except (OSError, json.JSONDecodeError) as exc:
            _log(f"Could not read config.json ({exc}); using defaults.")
            return

        if not isinstance(loaded, dict):
            _log("config.json must contain a JSON object; using defaults.")
            return

        port = loaded.get("serial_port", self.config["serial_port"])
        if isinstance(port, str) and port.strip():
            self.config["serial_port"] = port.strip()

        baud = loaded.get("baud_rate", self.config["baud_rate"])
        if isinstance(baud, int) and 1200 <= baud <= 2_000_000:
            self.config["baud_rate"] = baud

        update_hz = _safe_float(loaded.get("update_hz"), self.config["update_hz"])
        self.config["update_hz"] = min(max(update_hz, 1.0), 100.0)

        reconnect_s = _safe_float(
            loaded.get("reconnect_interval_s"), self.config["reconnect_interval_s"]
        )
        self.config["reconnect_interval_s"] = min(max(reconnect_s, 0.5), 60.0)

        reset_delay = _safe_float(
            loaded.get("arduino_reset_delay_s"), self.config["arduino_reset_delay_s"]
        )
        self.config["arduino_reset_delay_s"] = min(max(reset_delay, 0.0), 10.0)

        for key in (
            "park_on_plugin_disable",
            "send_resume_on_connect",
            "log_controller_messages",
        ):
            self.config[key] = _safe_bool(loaded.get(key), self.config[key])

    def _resolve_datarefs(self) -> bool:
        self.datarefs.clear()
        missing = []

        for token, dataref_name in DATAREFS.items():
            handle = xp.findDataRef(dataref_name)
            if handle is None:
                missing.append(dataref_name)
            else:
                self.datarefs[token] = handle

        if missing:
            _log("Required X-Plane DataRefs were not found:")
            for name in missing:
                _log(f"  {name}")
            return False

        return True

    def _flight_loop_interval(self) -> float:
        hz = float(self.config.get("update_hz", 20.0))
        return 1.0 / min(max(hz, 1.0), 100.0)

    def _choose_auto_port(self) -> Optional[str]:
        try:
            ports = list(list_ports.comports())
        except Exception as exc:
            _log(f"Unable to enumerate serial ports: {exc}")
            return None

        if not ports:
            return None

        ranked = []
        for port in ports:
            text = " ".join(
                str(value or "")
                for value in (
                    getattr(port, "device", ""),
                    getattr(port, "description", ""),
                    getattr(port, "manufacturer", ""),
                    getattr(port, "product", ""),
                    getattr(port, "hwid", ""),
                )
            ).lower()

            score = 0
            vid = getattr(port, "vid", None)
            if vid in ARDUINO_VIDS:
                score += 100
            if "arduino" in text:
                score += 80
            if "mega 2560" in text or "mega2560" in text:
                score += 70
            if "usbmodem" in text or "ttyacm" in text:
                score += 30
            if "ch340" in text or "wch" in text:
                score += 15

            if score > 0:
                ranked.append((score, str(port.device)))

        if ranked:
            ranked.sort(key=lambda item: (-item[0], item[1]))
            return ranked[0][1]

        usb_ports = [p for p in ports if getattr(p, "vid", None) is not None]
        if len(usb_ports) == 1:
            return str(usb_ports[0].device)

        return None

    def _configured_port(self) -> Optional[str]:
        configured = str(self.config.get("serial_port", "auto")).strip()
        if configured.lower() == "auto":
            return self._choose_auto_port()
        return configured

    def _connect_if_needed(self) -> None:
        if self.serial_connection is not None and self.serial_connection.is_open:
            return

        now = time.monotonic()
        if now < self.next_connect_attempt:
            return

        self.next_connect_attempt = now + float(
            self.config.get("reconnect_interval_s", 2.0)
        )

        port = self._configured_port()
        if not port:
            _log(
                "No suitable Arduino serial port found. "
                "Set serial_port explicitly in config.json if auto-detection cannot identify it."
            )
            return

        try:
            connection = serial.Serial(
                port=port,
                baudrate=int(self.config.get("baud_rate", 115200)),
                timeout=0,
                write_timeout=0.02,
            )
        except (OSError, serial.SerialException) as exc:
            _log(f"Could not open serial port {port}: {exc}")
            return

        self.serial_connection = connection
        self.connected_port = port
        self.serial_ready_at = now + float(
            self.config.get("arduino_reset_delay_s", 2.0)
        )
        self.resume_sent = False
        self.rx_text_buffer = ""
        _log(
            f"Connected to {port} at {self.config.get('baud_rate', 115200)} baud; "
            f"waiting {self.config.get('arduino_reset_delay_s', 2.0):.1f}s for controller reset."
        )

    def _disconnect(self, reason: str) -> None:
        connection = self.serial_connection
        self.serial_connection = None
        previous_port = self.connected_port
        self.connected_port = None
        self.resume_sent = False
        self.rx_text_buffer = ""

        if connection is not None:
            try:
                if connection.is_open:
                    connection.close()
            except Exception:
                pass

        if previous_port:
            _log(f"Disconnected from {previous_port}: {reason}.")

    def _serial_is_ready(self) -> bool:
        return (
            self.serial_connection is not None
            and self.serial_connection.is_open
            and time.monotonic() >= self.serial_ready_at
        )

    def _write_bytes(self, payload: bytes) -> bool:
        if not self._serial_is_ready():
            return False

        try:
            self.serial_connection.write(payload)
            return True
        except (OSError, serial.SerialException, serial.SerialTimeoutException) as exc:
            self._disconnect(f"serial write failed ({exc})")
            return False

    def _send_control_command(self, command: str) -> bool:
        return self._write_bytes((command.rstrip("\r\n") + "\n").encode("ascii"))

    def _flush_serial_output(self) -> None:
        if self.serial_connection is None:
            return
        try:
            self.serial_connection.flush()
        except Exception:
            pass

    def _drain_controller_output(self) -> None:
        connection = self.serial_connection
        if connection is None or not connection.is_open:
            return

        try:
            waiting = int(connection.in_waiting)
            if waiting <= 0:
                return
            chunk = connection.read(min(waiting, 2048))
        except (OSError, serial.SerialException) as exc:
            self._disconnect(f"serial read failed ({exc})")
            return

        if not chunk:
            return

        if not self.config.get("log_controller_messages", False):
            return

        self.rx_text_buffer += chunk.decode("utf-8", errors="replace")
        if len(self.rx_text_buffer) > 8192:
            self.rx_text_buffer = self.rx_text_buffer[-4096:]

        while "\n" in self.rx_text_buffer:
            line, self.rx_text_buffer = self.rx_text_buffer.split("\n", 1)
            line = line.rstrip("\r")
            if line:
                _log(f"controller: {line}")

    def _read_values(self) -> Dict[str, float]:
        return {
            token: float(xp.getDataf(handle))
            for token, handle in self.datarefs.items()
        }

    def _flight_loop(self, elapsedMe, elapsedSim, counter, refcon):
        interval = self._flight_loop_interval()

        self._connect_if_needed()
        self._drain_controller_output()

        if not self._serial_is_ready():
            return interval

        if self.config.get("send_resume_on_connect", True) and not self.resume_sent:
            if self._send_control_command("RESUME"):
                self.resume_sent = True
            else:
                return interval

        try:
            frame = _format_frame(self._read_values()).encode("ascii")
        except Exception as exc:
            _log(f"Unable to read/format X-Plane telemetry: {exc}")
            return interval

        self._write_bytes(frame)
        return interval
