# Rotorflight

[![Website](https://img.shields.io/badge/Website-rotorflight.org-2ea44f)](https://www.rotorflight.org/)
[![Discord](https://img.shields.io/badge/Discord-Join%20Chat-5865F2?logo=discord&logoColor=white)](https://discord.gg/FyfMF4RwSA)
[![License: GPLv3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)

[Rotorflight](https://github.com/rotorflight) is a Flight Control software suite designed for
single-rotor helicopters. It consists of:

- Rotorflight Flight Controller Firmware (this repository)
- [Rotorflight Configurator](https://github.com/rotorflight/rotorflight-configurator), for flashing and configuring the flight controller
- [Rotorflight Blackbox Explorer](https://github.com/rotorflight/rotorflight-blackbox), for analyzing blackbox flight logs
- [Rotorflight Lua Scripts](https://github.com/rotorflight/rotorflight-lua-scripts), for configuring the flight controller using a transmitter

Built on Betaflight 4.3, Rotorflight incorporates numerous advanced features specifically
tailored for helicopters. It's important to note that Rotorflight does _not_ support multi-rotor
crafts or airplanes; it's exclusively designed for RC helicopters.

This version of Rotorflight is also known as **Rotorflight 2** or **RF2**.


## Information

Tutorials, documentation, and flight videos can be found on the [Rotorflight website](https://www.rotorflight.org/).


## Features

Rotorflight has many features:

* Many receiver protocols: CRSF, S.BUS, FBUS, F.Port, SRXL2, IBUS, IBUS2, XBUS, EXBUS, GHOST, CPPM
* Support for various telemetry protocols: CSRF, S.Port, FBUS, HoTT, etc.
* ESC telemetry protocols: BLHeli32, Hobbywing, Scorpion, Kontronik, Castle, OMP, ZTW, APD, YGE, XDFly, Graupner, FLYROTOR
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
* High-speed Blackbox logging

Plus lots of features inherited from Betaflight:

* Configuration profiles for changing various tuning parameters
* Rates profiles for changing the stick feel and agility
* Multiple ESC protocols: PWM, DSHOT, Multishot, etc.
* Multi-color RGB LEDs

And much more...


## Hardware support

The best hardware for Rotorflight is any Flight Controller especially designed for it.
See [Rotorflight Flight Controllers](https://rotorflight.org/docs/controllers)

Otherwise, Rotorflight supports all flight controller boards that are supported by Betaflight 4.3,
assuming that the board has enough suitable I/O pins for connecting all the servos and motors required.

Also, the Betaflight boards are labeled for multi-rotor use - thus the user needs to understand how
these functions can be used for a different purpose with helicopters. Usually this is just about using
the motor outputs for servos, but in some cases a more advanced remapping may be needed.

Rotorflight supports STM32F405, STM32F722, STM32F745 and STM32H743 MCUs from ST.

The support for lesser MCUs like STM32G474 and STM32F411 is EOL and will be removed soon.


## Safety

Helicopters can be dangerous. Rotorflight is provided free of charge and without
any warranty — you use it entirely at your own risk.

- Always remove the main and tail rotor blades before configuring or testing on the bench.
- Always double check your configuration before flying.
- Keep a safe distance from the helicopter whenever the battery is connected.


## Installation

Download and flash the Rotorflight firmware with the
[Rotorflight Configurator](https://github.com/rotorflight/rotorflight-configurator/releases).

Flashing the Rotorflight firmware with any other flashing tool is strictly not
supported, and will not work.

Visit the [website](https://www.rotorflight.org/) for more details on setting up
and using Rotorflight.


## Support

The primary place for support, questions, and discussion is the
[Rotorflight Discord](https://discord.gg/FyfMF4RwSA). The developers, flight
controller manufacturers, and many experienced pilots are all active there.

There is also a [Rotorflight Facebook Group](https://www.facebook.com/groups/rotorflight)
for hanging out with other Rotorflight pilots.


## Contributing

Rotorflight is an open-source community project. Anybody can join in and help to make it better by:

* helping other users on [Rotorflight Discord](https://discord.gg/FyfMF4RwSA) or other online forums
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


## License

Rotorflight is free software licensed under the GNU General Public License v3.0 (GPLv3).
See the [LICENSE](LICENSE) file for the full license text.


## Contact

Team Rotorflight can be contacted by email at rotorflightfc@gmail.com.

Please note that this email address is **not** for support. For help and questions,
please use the [Support](#support) channels above.
