
# Rotorflight

**Rotorflight** is a _Flight Control_/_FBL_ Software Suite for traditional single-rotor RC helicopters. It is based on Betaflight 4.2, enjoying all the great features of the Betaflight platform, plus many new features added for helicopters. Rotorflight borrows ideas and code also from Heliflight3D, an earlier fork of Betaflight for helicopters.

Rotorflight does **NOT** support multi-rotor 'drones', nor airplanes; it is only for traditinal RC helicopters, including co-axial and tandem helicopters.


## Information

For more information, please see our github space:

 - [Main page](https://github.com/rotorflight)
 - [Rotorflight Wiki](https://github.com/rotorflight/rotorflight/wiki)


## Features

Rotorflight has many features:

* Great GUI application for setting up and configuration
* Advanced PID control, tuned for helicopters
* Main rotor speed governor
* Precompensation between collective/cyclic/yaw
* Advanced gyro filtering for helicopters
  - Dynamic RPM based notch filter banks
  - Advanced dynamic LPF
* Fully customisable servo/motor mixer
* Flexible servo configuration
* Flight Profiles (Banks)
* In-flight settings adjustment from RC channels

Plus lots of features inherited from Betaflight:

* Blackbox Flight logger
* Multiple ESC protocols: DShot, Multishot, Oneshot, and traditional PWM
* Multiple RX telemetry protocols: CSRF, FrSky, HoTT, MSP, etc.
* Telemetry inputs for RSSI, battery voltage, current sensors, etc.
* Multi-color RGB LEDs
* Configurable buzzer sounds
* GPS support

And many more...


## Hardware support

Generally speaking, Rotorflight supports most flight controller hardware that is supported by Betaflight. With a caveat that the flight controller must have enough suitable I/O pins for connecting all the servos and motors required by a collective pitch helicopter.

Also, the FC boards are typically labeled for multi-rotor use - thus the user needs to understand how these functions can be used for a different purpose with helicopters. Usually this is just about using some of the motor outputs for servos, but in some cases a more advanced remapping may be needed.

It is highly recommended to use a STM32F7 or STM32F405 based FC board, as Rotorflight greatly benefits from the latest filtering algorithms and other new features that are all CPU intensive.
An absolute minimum is a STM32F411 based board, but it probably won't be able to run all the new features.


## Installation

Please see the [Releases](https://github.com/rotorflight/rotorflight/wiki/releases) page on the Wiki to download the latest official release.

Also read the installation instruction in the [Wiki](https://github.com/rotorflight/rotorflight/wiki/Installing-Rotorflight-Firmware).


## Contributing

Contributions are welcome and encouraged. You can contribute in many ways:

 - testing Rotorflight with different types of helicopters
 - improving the documentation in the Wiki
 - writing How-To guides
 - provide a new translation for the configurator
 - implement new features or fix bugs
 - reporting bugs
 - new ideas & suggestions
 - helping other users

For reporting Rotorflight issues or bugs, please raise them here:

 - [Feature requests](https://github.com/rotorflight/rotorflight/issues)
 - [Configurator issue tracker](https://github.com/rotorflight/rotorflight-configurator/issues)
 - [Blackbox issue tracker](https://github.com/rotorflight/rotorflight-blackbox/issues)
 - [Firmware issue tracker](https://github.com/rotorflight/rotorflight-firmware/issues)


## Credits

Rotorflight is Free Software. Meaning, it is available free of charge _without warranty_, the source code is available, and it is supported by the users themselves as a community. Rotorflight is under the GPLv3 license.

Rotorflight is forked from Betaflight, which in turn is forked from Cleanflight.
Rotorflight borrows ideas and code also from Heliflight-3D, another Betaflight fork for helis.

So thanks goes to all those whom have contributed along the way.

Origins for Rotorflight:
 - **Petri Mattila** (Dr.Rudder) - author, maintainer
 - **pkaig** - wiki, resource mapping, testing
 - **egon** - wiki, Dutch translation, Lua Scripts, testing
 - **mopatop** - wiki, testing
 - **Mike_PSL** - wiki, testing
 - **mattis** - German translation, testing
 - **Simon Stummer** (simonsummer) - testing

Origins for Heliflight-3D:
 - **James-T1** (author)
 - **Dr.Rudder**
 - **Westie**

And many many others who haven't been mentioned....
