# Rotorflight

[Rotorflight](https://github.com/rotorflight) is a Flight Control software suite designed for
single-rotor helicopters. It consists of:

- Rotorflight Flight Controller Firmware (this repository)
- Rotorflight Configurator, for flashing and configuring the flight controller
- Rotorflight Blackbox Explorer, for analyzing blackbox flight logs
- Rotorflight Lua Scripts, for configuring the flight controller using a transmitter

Built on Betaflight 4.3, Rotorflight incorporates numerous advanced features specifically
tailored for helicopters. It's important to note that Rotorflight does _not_ support multi-rotor
crafts or airplanes; it's exclusively designed for RC helicopters.

This version of Rotorflight is also known as **Rotorflight 2** or **RF2**.


## Information

Tutorials, documentation, and flight videos can be found on the [Rotorflight website](https://www.rotorflight.org/).


## Features

Rotorflight has many features:

* Many receiver protocols: CRSF, S.BUS, FBUS, F.Port, SRXL2, IBUS, XBUS, EXBUS, GHOST, CPPM
* Support for various telemetry protocols: CSRF, S.Port, FBUS, HoTT, etc.
* ESC telemetry protocols: Hobbywing, Scorpion, Kontronik, Castle, OMP, ZTW, APD, YGE, XDFly, FLYROTOR
* Advanced PID control algorithms for helicopters
* Stabilisation modes (6D)
* Rotor speed governor
* Motorised tail support with Tail Torque Assist (TTA, also known as TALY)
* Remote configuration and tuning with the transmitter
  - With knobs / switches assigned to functions
  - With LUA scripts on EdgeTX, OpenTX and Ethos
* Extra servo/motor outputs for AUX functions
* Fully customisable servo/motor mixer
* Sensors for battery voltage, current, BEC, etc.
* Advanced gyro filtering
  - Dynamic RPM based notch filters
  - Dynamic notch filters based on FFT
  - Dynamic LPF
* High-speed Blackbox logging

Plus lots of features inherited from Betaflight:

* Configuration profiles for changing various tuning parameters
* Rates profiles for changing the stick feel and agility
* Multiple ESC protocols: PWM, DSHOT, Multishot, etc.
* Multi-color RGB LEDs
* GPS support (telemetry & logging only)

And much more...


## Hardware support

The best hardware for Rotorflight is any Flight Controller especially designed for it.
See [What board is suitable?](https://www.rotorflight.org/docs/Tutorial-Quickstart/What-Board)

Otherwise, Rotorflight supports all flight controller boards that are supported by Betaflight 4.3,
assuming that the board has enough suitable I/O pins for connecting all the servos and motors required.

Also, the Betaflight boards are labeled for multi-rotor use - thus the user needs to understand how
these functions can be used for a different purpose with helicopters. Usually this is just about using
the motor outputs for servos, but in some cases a more advanced remapping may be needed.

Rotorflight supports STM32F405, STM32F722, STM32F745 and STM32H743 MCUs from ST.

The support for lesser MCUs like STM32G474 and STM32F411 is EOL and will be removed soon.


## Installation

Download and flash the Rotorflight firmware with the
[Rotorflight Configurator](https://github.com/rotorflight/rotorflight-configurator/releases).

Flashing the Rotorflight firmware with any other flashing tools is not recommended.

Visit the [website](https://www.rotorflight.org/) for more details on setting up and using Rotorflight.


## Contributing

Rotorflight is an open-source community project. Anybody can join in and help to make it better by:

* helping other users on Rotorflight Discord or other online forums
* [reporting](https://github.com/rotorflight?tab=repositories) bugs and issues, and suggesting improvements
* testing new software versions, new features and fixes; and providing feedback
* participating in discussions on new features
* create or update content on the [Website](https://www.rotorflight.org)
* [contributing](https://www.rotorflight.org/docs/Contributing/intro) to the software development - fixing bugs, implementing new features and improvements
* [translating](https://www.rotorflight.org/docs/Contributing/intro#translations) Rotorflight Configurator into a new language, or helping to maintain an existing translation


## Origins

Rotorflight is software that is **open source** and is available free of charge without warranty.

Rotorflight is forked from [Betaflight](https://github.com/betaflight), which in turn is forked from [Cleanflight](https://github.com/cleanflight).
Rotorflight borrows ideas and code also from [HeliFlight3D](https://github.com/heliflight3d/), another Betaflight fork for helicopters.

Big thanks to everyone who has contributed along the journey!


## Contact

Team Rotorflight can be contacted by email at rotorflightfc@gmail.com.
