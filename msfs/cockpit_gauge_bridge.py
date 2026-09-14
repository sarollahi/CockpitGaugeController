"""
Cockpit Gauge Controller - Microsoft Flight Simulator bridge.

Runs as an external Windows process, reads MSFS 2020/2024 SimVars through
SimConnect, and streams normalized telemetry to the Arduino Mega firmware over
USB serial using CockpitGaugeController's newline-delimited ASCII protocol.

Copyright (C) 2026 sarollahi
SPDX-License-Identifier: GPL-2.0-only
"""

from __future__ import annotations

import json
import logging
import math
import os
import signal
import time
from pathlib import Path
from typing import Any, Dict, Optional

try:
    import serial
    from serial.tools import list_ports
except ModuleNotFoundError as exc:
    raise SystemExit(
        "pyserial is missing. Run: python -m pip install -r requirements.txt"
    ) from exc

try:
    from simconnect import PERIOD_VISUAL_FRAME, SimConnect
except ModuleNotFoundError as exc:
    raise SystemExit(
        "pysimconnect is missing. Run: python -m pip install -r requirements.txt"
    ) from exc


APP_NAME = "CockpitGaugeController-MSFS"
CONFIG_PATH = Path(__file__).with_name("config.json")
LOG = logging.getLogger(APP_NAME)

DEFAULT_CONFIG: Dict[str, Any] = {
    "serial_port": "auto",
    "baud_rate": 115200,
    "update_hz": 20.0,
    "simconnect_reconnect_interval_s": 2.0,
    "serial_reconnect_interval_s": 2.0,
    "arduino_reset_delay_s": 2.0,
    "send_resume_on_connect": True,
    "park_on_exit": True,
    "park_wait_timeout_s": 10.0,
    "log_controller_messages": False,
    "turn_invert": False,
    "slip_invert": False,
    "slip_full_scale_input_deg": 4.0,
    "simconnect_dll": "auto",
}

SIMVARS = [
    dict(name="AIRSPEED INDICATED", units="knots"),
    dict(name="TURN INDICATOR RATE", units="radians per second"),
    "TURN COORDINATOR BALL",
    dict(name="INDICATED ALTITUDE", units="feet"),
    dict(name="KOHLSMAN SETTING HG:1", units="inHg"),
    dict(name="PLANE HEADING DEGREES GYRO", units="degrees"),
    dict(name="AUTOPILOT HEADING LOCK DIR", units="degrees"),
    dict(name="VERTICAL SPEED", units="feet per minute"),
]

ARDUINO_VIDS = {0x2341, 0x2A03}


class StopFlag:
    def __init__(self) -> None:
        self.requested = False

    def request(self, *_args: object) -> None:
        self.requested = True


class SerialController:
    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = config
        self.connection: Optional[serial.Serial] = None
        self.port: Optional[str] = None
        self.ready_at = 0.0
        self.next_attempt = 0.0
        self.resume_sent = False
        self.rx_buffer = ""

    def _auto_port(self) -> Optional[str]:
        ports = list(list_ports.comports())
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
            if getattr(port, "vid", None) in ARDUINO_VIDS:
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
        value = str(self.config["serial_port"]).strip()
        return self._auto_port() if value.lower() == "auto" else value

    def ensure_connected(self) -> None:
        if self.connection is not None and self.connection.is_open:
            return

        now = time.monotonic()
        if now < self.next_attempt:
            return
        self.next_attempt = now + float(self.config["serial_reconnect_interval_s"])

        port = self._configured_port()
        if not port:
            LOG.warning(
                "No unique Arduino serial port found. Set serial_port in msfs/config.json."
            )
            return

        try:
            connection = serial.Serial(
                port=port,
                baudrate=int(self.config["baud_rate"]),
                timeout=0,
                write_timeout=0.05,
            )
        except (OSError, serial.SerialException) as exc:
            LOG.warning("Could not open serial port %s: %s", port, exc)
            return

        self.connection = connection
        self.port = port
        self.ready_at = now + float(self.config["arduino_reset_delay_s"])
        self.resume_sent = False
        self.rx_buffer = ""
        LOG.info(
            "Connected to controller on %s at %s baud; waiting %.1fs for reset",
            port,
            self.config["baud_rate"],
            self.config["arduino_reset_delay_s"],
        )

    def disconnect(self, reason: str) -> None:
        connection = self.connection
        old_port = self.port
        self.connection = None
        self.port = None
        self.resume_sent = False
        self.rx_buffer = ""

        if connection is not None:
            try:
                if connection.is_open:
                    connection.close()
            except Exception:
                pass

        if old_port:
            LOG.info("Disconnected from %s: %s", old_port, reason)

    def ready(self) -> bool:
        return (
            self.connection is not None
            and self.connection.is_open
            and time.monotonic() >= self.ready_at
        )

    def write_line(self, line: str) -> bool:
        if not self.ready():
            return False
        assert self.connection is not None
        try:
            self.connection.write((line.rstrip("\r\n") + "\n").encode("ascii"))
            return True
        except (OSError, serial.SerialException, serial.SerialTimeoutException) as exc:
            self.disconnect(f"serial write failed ({exc})")
            return False

    def send_resume_if_needed(self) -> bool:
        if not self.config["send_resume_on_connect"] or self.resume_sent:
            return True
        if self.write_line("RESUME"):
            self.resume_sent = True
            return True
        return False

    def drain(self, return_lines: bool = False) -> list[str]:
        connection = self.connection
        if connection is None or not connection.is_open:
            return []

        try:
            waiting = int(connection.in_waiting)
            if waiting <= 0:
                return []
            chunk = connection.read(min(waiting, 4096))
        except (OSError, serial.SerialException) as exc:
            self.disconnect(f"serial read failed ({exc})")
            return []

        if not chunk:
            return []

        self.rx_buffer += chunk.decode("utf-8", errors="replace")
        if len(self.rx_buffer) > 16384:
            self.rx_buffer = self.rx_buffer[-8192:]

        lines: list[str] = []
        while "\n" in self.rx_buffer:
            line, self.rx_buffer = self.rx_buffer.split("\n", 1)
            line = line.rstrip("\r")
            if line:
                if self.config["log_controller_messages"]:
                    LOG.info("controller: %s", line)
                if return_lines:
                    lines.append(line)
        return lines

    def park_and_wait(self) -> None:
        if not self.config["park_on_exit"] or not self.ready():
            return
        if not self.write_line("PARK"):
            return

        timeout = float(self.config["park_wait_timeout_s"])
        LOG.info("PARK sent; waiting up to %.1fs for controller confirmation", timeout)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for line in self.drain(return_lines=True):
                if "Safe to power off" in line:
                    LOG.info("Controller confirmed clean parked state")
                    return
            time.sleep(0.02)
        LOG.warning("Timed out waiting for PARK confirmation")


class MsfsConnection:
    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = config
        self.sc: Optional[SimConnect] = None
        self.datadef = None
        self.next_attempt = 0.0

    def connect_if_needed(self) -> None:
        if self.sc is not None:
            return

        now = time.monotonic()
        if now < self.next_attempt:
            return
        self.next_attempt = now + float(self.config["simconnect_reconnect_interval_s"])

        dll_value = str(self.config.get("simconnect_dll", "auto")).strip()
        kwargs: Dict[str, Any] = {"name": APP_NAME}
        if dll_value and dll_value.lower() != "auto":
            kwargs["dll_path"] = dll_value

        sc = None
        try:
            sc = SimConnect(**kwargs)
            datadef = sc.subscribe_simdata(
                SIMVARS,
                period=PERIOD_VISUAL_FRAME,
                interval=1,
            )
        except Exception as exc:
            LOG.warning("Waiting for Microsoft Flight Simulator / SimConnect: %s", exc)
            if sc is not None:
                try:
                    sc.Close()
                except Exception:
                    pass
            return

        self.sc = sc
        self.datadef = datadef
        LOG.info("Connected to Microsoft Flight Simulator through SimConnect")

    def disconnect(self, reason: str) -> None:
        sc = self.sc
        self.sc = None
        self.datadef = None
        if sc is not None:
            try:
                sc.Close()
            except Exception:
                pass
            LOG.info("SimConnect disconnected: %s", reason)

    def pump(self) -> bool:
        if self.sc is None:
            return False
        try:
            while self.sc.receive(timeout_seconds=0.0):
                pass
            return True
        except Exception as exc:
            self.disconnect(f"receive failed ({exc})")
            return False

    def values(self) -> Optional[Dict[str, float]]:
        if self.datadef is None:
            return None
        data = self.datadef.simdata

        required = (
            "AIRSPEED INDICATED",
            "TURN INDICATOR RATE",
            "TURN COORDINATOR BALL",
            "INDICATED ALTITUDE",
            "KOHLSMAN SETTING HG:1",
            "PLANE HEADING DEGREES GYRO",
            "AUTOPILOT HEADING LOCK DIR",
            "VERTICAL SPEED",
        )
        try:
            values = {name: float(data[name]) for name in required}
        except (KeyError, TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in values.values()):
            return None
        return values


def _load_config() -> Dict[str, Any]:
    config = dict(DEFAULT_CONFIG)
    try:
        loaded = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    except FileNotFoundError:
        LOG.warning("%s not found; using defaults", CONFIG_PATH)
        return config
    except (OSError, json.JSONDecodeError) as exc:
        LOG.warning("Could not read %s (%s); using defaults", CONFIG_PATH, exc)
        return config

    if not isinstance(loaded, dict):
        LOG.warning("config.json must contain a JSON object; using defaults")
        return config

    config.update({key: value for key, value in loaded.items() if key in config})

    config["baud_rate"] = int(config["baud_rate"])
    config["update_hz"] = min(max(float(config["update_hz"]), 1.0), 100.0)
    config["simconnect_reconnect_interval_s"] = min(
        max(float(config["simconnect_reconnect_interval_s"]), 0.5), 60.0
    )
    config["serial_reconnect_interval_s"] = min(
        max(float(config["serial_reconnect_interval_s"]), 0.5), 60.0
    )
    config["arduino_reset_delay_s"] = min(
        max(float(config["arduino_reset_delay_s"]), 0.0), 10.0
    )
    config["park_wait_timeout_s"] = min(
        max(float(config["park_wait_timeout_s"]), 0.0), 60.0
    )
    config["slip_full_scale_input_deg"] = min(
        max(abs(float(config["slip_full_scale_input_deg"])), 0.1), 20.0
    )
    return config


def _clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def _normalize(values: Dict[str, float], config: Dict[str, Any]) -> Dict[str, float]:
    turn_deg_s = math.degrees(values["TURN INDICATOR RATE"])
    if bool(config["turn_invert"]):
        turn_deg_s = -turn_deg_s

    ball = _clamp(values["TURN COORDINATOR BALL"], -127.0, 127.0)
    slip = (ball / 127.0) * float(config["slip_full_scale_input_deg"])
    if bool(config["slip_invert"]):
        slip = -slip

    return {
        "IAS": values["AIRSPEED INDICATED"],
        "T": turn_deg_s,
        "S": slip,
        "ALT": values["INDICATED ALTITUDE"],
        "BARO": values["KOHLSMAN SETTING HG:1"],
        "HDG": values["PLANE HEADING DEGREES GYRO"] % 360.0,
        "BUG": values["AUTOPILOT HEADING LOCK DIR"] % 360.0,
        "VSI": values["VERTICAL SPEED"],
    }


def _frame(values: Dict[str, float]) -> str:
    return (
        f"IAS:{values['IAS']:.2f} "
        f"T:{values['T']:.3f} "
        f"S:{values['S']:.3f} "
        f"ALT:{values['ALT']:.1f} "
        f"BARO:{values['BARO']:.3f} "
        f"HDG:{values['HDG']:.2f} "
        f"BUG:{values['BUG']:.2f} "
        f"VSI:{values['VSI']:.1f}"
    )


def _configure_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


def main() -> int:
    if os.name != "nt":
        LOG.error("The MSFS SimConnect bridge must run on Windows")
        return 2

    config = _load_config()
    stop = StopFlag()
    signal.signal(signal.SIGINT, stop.request)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, stop.request)

    serial_controller = SerialController(config)
    simulator = MsfsConnection(config)
    interval = 1.0 / float(config["update_hz"])
    next_send = time.monotonic()

    LOG.info("Starting MSFS bridge at %.1f Hz", config["update_hz"])

    try:
        while not stop.requested:
            serial_controller.ensure_connected()
            serial_controller.drain()
            simulator.connect_if_needed()

            if simulator.sc is not None:
                simulator.pump()

            now = time.monotonic()
            if now >= next_send:
                missed = max(1, int((now - next_send) / interval) + 1)
                next_send += missed * interval

                if serial_controller.ready() and serial_controller.send_resume_if_needed():
                    raw = simulator.values()
                    if raw is not None:
                        serial_controller.write_line(_frame(_normalize(raw, config)))

            time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        serial_controller.park_and_wait()
        simulator.disconnect("bridge exiting")
        serial_controller.disconnect("bridge exiting")

    return 0


if __name__ == "__main__":
    _configure_logging()
    raise SystemExit(main())
