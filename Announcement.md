:regional_indicator_r: :regional_indicator_o: :regional_indicator_t: :regional_indicator_o: :regional_indicator_r: :regional_indicator_f: :regional_indicator_l: :regional_indicator_i: :regional_indicator_g: :regional_indicator_h: :regional_indicator_t:   :two: . :three: . :zero: - :regional_indicator_r: :regional_indicator_c: :two:

We are proud to present **Rotorflight 2.3.0 - Release Candidate 2**.

This RC2 build focuses on final polish for Rotorflight 2.3.0, with targeted new features, reliability improvements, and important bug fixes based on RC1 testing feedback.

## Thank You

### BIG THANKS to everybody who has helped making this release happen!

Rotorflight is a community-driven project, developed entirely by volunteers in their spare time. If you enjoy using Rotorflight and would like to support its continued development, please consider making a donation. Every contribution — however small — helps keep the project alive and allows us to dedicate more time to bringing you new features and improvements.

[Rotorflight Donations](https://opencollective.com/rotorflight)

### BIG THANKS also to everybody who has already donated — your generosity is truly appreciated!

## Downloads
Rotorflight 2.3.0-RC2 can be downloaded here:

- [Rotorflight Configurator](https://github.com/rotorflight/rotorflight-configurator/releases/tag/release/2.3.0-RC2)
- [Rotorflight Blackbox](https://github.com/rotorflight/rotorflight-blackbox/releases/tag/release/2.3.0-RC2)
- [Rotorflight Lua Scripts for EdgeTx and OpenTx](https://github.com/rotorflight/rotorflight-lua-scripts/releases/tag/release/2.3.0-RC2)
- [Rotorflight Lua Scripts for FrSky Ethos](https://github.com/rotorflight/rotorflight-lua-ethos/releases/tag/release/2.3.0-RC2)
- [Rotorflight Lua Suite for FrSky Ethos](https://github.com/rotorflight/rotorflight-lua-ethos-suite/releases/tag/release/2.3.0-RC2)

## NOTE!
This is not yet the _official_ release of 2.3 — it is a _release candidate_. The final release will follow once any remaining issues discovered during community testing have been addressed. We encourage experienced users to test this version and report any problems via our GitHub issue tracker or Discord.

The firmware for 2.3 appears as **4.6.x** in the Firmware Flasher in the Configurator.

## Changes since 4.6.0-RC1
- Fix potential buffer overflows in multiple RX protocols
- Fix 4-way MSP ESC teardown for reliable return to normal run mode
- Update CMS rates menu and stick thresholds
- Add MSP command to retrieve the setpoints
- Add S.PORT sensor input

## 4.6 Firmware Highlights

### Rotorflight Rates
The rates system has been completely redesigned. The new Rotorflight Rates offer more intuitive and precise control feel tuning, giving pilots a dedicated rates model built specifically for helicopters. Existing profiles will continue to work, and we encourage pilots to explore the new system for an improved flying experience.

### Polar Coordinates rate mode
Rates can now be applied in polar coordinates, treating cyclic stick input as a single magnitude-and-direction command rather than two independent axes. This results in a more natural and consistent feel across all stick directions — especially noticeable during combined cyclic inputs.

### Governor refactor for Nitro/I.C. engines
The Governor has received a major overhaul with significant improvements and fixes aimed at nitro and I.C. engine setups. Head speed control is now more robust and responsive, making Rotorflight a better fit than ever for combustion-powered helicopters.

### Battery profile support
Multiple battery profiles can now be configured and switched, making it easy to fly different packs without having to reconfigure voltage and current settings each time.

### ESC Forward Programming
Rotorflight now supports Forward Programming for a wide range of ESCs — AM32, BLHeli S/Bluejay, ZTW, OMP, and XDFly — all accessible directly from the Ethos Suite.

### FlySky IBUS2 Control & Telemetry
Full bidirectional support for FlySky IBUS2 has been added, bringing control and rich telemetry to FlySky users.

### FBUS Master Out & Sensors
FBUS Master Out and FBUS Master Sensors are now supported, further expanding the already comprehensive telemetry capabilities for FrSky/FBUS setups.

### S.Port Master for Sensors
S.Port master is now supported, allowing to connect legacy S.Port sensors.

### New hardware support
This release adds drivers for several new components:
- BMI323 gyro driver
- BMP581 barometer driver
- GYRO_CLK support for external gyro clocking

### Native ELRS telemetry sensors
RPM and temperature are now available as native ELRS telemetry sensors, visible directly on ELRS-equipped transmitters without any additional configuration.

### Important Bug Fixes
- Potential buffer overflows in multiple RX protocols - fixed
- JR Xbus Mode-A — channel corruption fixed
- Failsafe — throttle handling corrected
- APD ESC telemetry — fixed
- MSPv2 over SmartPort — fixed
- Battery presence voltage stability check — fixed
- ACRO TRAINER — fixed a catastrophic failure mode

