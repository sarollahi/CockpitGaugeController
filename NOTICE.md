# Attribution and Third-Party Notice

## Mechanical instrument designs

The physical flight-instrument mechanisms used as the basis for this build are based on designs by **Martin Rusk**.

Original project:

- https://github.com/MartinRusk/Sixpack

Relevant 3D-printable designs:

- Airspeed indicator: https://www.printables.com/model/338749-airspeed-indicator-for-flight-simulation
- Altimeter: https://www.printables.com/model/341603-altimeter-for-flight-simulation
- Variometer: https://www.printables.com/model/338636-variometer-for-flight-simulation
- Gyro compass: https://www.printables.com/model/356101-gyro-compass-for-flight-simulation
- Turn coordinator: https://www.printables.com/model/357184-turn-coordinator-for-flight-simulation

The firmware in this repository is a separate implementation and is not a fork of Martin Rusk's Sixpack firmware.

No original Sixpack STL/STEP/3D-model files are redistributed in this repository. Anyone downloading or redistributing those designs should consult the license shown on the corresponding original source page.

Some mechanical scaling choices in the firmware are informed by the physical geometry of the instrument mechanisms used in the build. Those references are identified in code comments where relevant.

## Software dependencies

This project depends on third-party Arduino libraries, including:

- AccelStepper by Mike McCauley / AirSpayce
- Adafruit GFX Library
- Adafruit ST7735 and ST7789 Library

Those libraries are not vendored into this repository and remain subject to their own licenses and copyright notices.
