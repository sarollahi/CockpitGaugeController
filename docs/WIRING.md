# Wiring

This document describes the default pin map used by `CockpitGaugeController.ino`.

## Arduino Mega 2560 pin map

### Stepper axes

Each four-pin group connects to the four logic inputs of the corresponding motor driver board. The firmware internally uses the `AccelStepper::HALF4WIRE` sequence expected by the library.

| Axis | IN1 | IN2 | IN3 | IN4 |
| --- | ---: | ---: | ---: | ---: |
| ASI | D2 | D3 | D4 | D5 |
| TURN | D22 | D23 | D24 | D25 |
| SLIP | D26 | D27 | D28 | D29 |
| VSI | D30 | D31 | D32 | D33 |
| ALT | D34 | D35 | D36 | D37 |
| BARO | D38 | D39 | D40 | D41 |
| DG | D42 | D43 | D44 | D45 |
| BUG | D46 | D47 | D48 | D49 |

Use a ULN2003-type driver or another appropriate driver between the Mega and each stepper motor. Do not connect stepper coils directly to the microcontroller pins.

## Buttons

Buttons are configured as `INPUT_PULLUP`, so each button should connect the input pin to **GND when pressed**.

| Function | Mega pin | Electrical behavior |
| --- | --- | --- |
| Select gauge | D13 | Active LOW |
| CW / positive trim | D11 | Active LOW |
| CCW / negative trim | D12 | Active LOW |

No external pull-up resistor is required for the default configuration.

## TFT

The ST7735 display uses hardware SPI.

| TFT signal | Mega pin |
| --- | --- |
| SCK / CLK | D52 |
| MOSI / SDA | D51 |
| CS | A9 |
| DC / A0 | A8 |
| RESET | A10 |

The exact power pin depends on the display module. Confirm whether the module expects 3.3 V or accepts 5 V logic/power before connecting it.

## Heartbeat output

| Function | Mega pin |
| --- | --- |
| Optional heartbeat LED | D10 |

D10 was unused by the original gauge wiring and is intentionally separate from the Select button on D13. If used, connect an external LED with an appropriate series resistor.

**Important:** `D10` and `A10` are separate pins on the Arduino Mega. `A10` is the TFT reset line; `D10` is the optional heartbeat output.

## Power and grounding

All logic grounds must share a common reference:

- Arduino Mega GND
- stepper-driver GND
- external stepper-supply GND
- TFT GND
- button ground

Use an external supply sized for the number of simultaneously energized stepper motors. Do not assume the Mega's onboard regulator or USB power should supply every motor.
