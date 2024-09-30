# 4.4.0-20240929

This is a _development snapshot_ of the Rotorflight 2.1 firmware.

## Notes

This version is intended to be used for beta-testing only.
It is not fully working nor stable, and should not be used by end-users.

For more information, please join the Rotorflight Discord chat.

## Downloads

The download locations are:

- [Rotorflight Configurator](https://github.com/rotorflight/rotorflight-configurator/releases/tag/snapshot/2.1.0-20240929)
- [Rotorflight Blackbox](https://github.com/rotorflight/rotorflight-blackbox/releases/tag/snapshot/2.1.0-20240929)
- [LUA Scripts for EdgeTx and OpenTx](https://github.com/rotorflight/rotorflight-lua-scripts/releases/tag/snapshot/2.1.0-20240929)
- [LUA Scripts for FrSky Ethos](https://github.com/rotorflight/rotorflight-lua-ethos/releases/tag/snapshot/2.1.0-20240929)

## Changes from 4.4.0-20240828

- Add ESC telemetry and programming for FLYROTOR
- Add missing ESC fields in BB
- Add missing ESC id in ELRS Custom telemetry
- Add RPM (freq) input edge and pull parameters
- Add MSP_PILOT_CONFIG for Lua integration
- Fix RPM filter update rate calculation
- Fix a buffer overflow in CRSF Device Info
- Improved PID defaults
- Improved Rates defaults
- Improved Cyclic Cross-Coupling
- Improved Rates acceleration and response filter


***

# 4.4.0-20240828

This is a _development snapshot_ of the Rotorflight 2.1 firmware.

## Notes

This version is intended to be used for beta-testing only.
It is not fully working nor stable, and should not be used by end-users.

For more information, please join the Rotorflight Discord chat.

## Downloads

The download locations are:

- [Rotorflight Configurator](https://github.com/rotorflight/rotorflight-configurator/releases/tag/snapshot/2.1.0-20240828)
- [Rotorflight Blackbox](https://github.com/rotorflight/rotorflight-blackbox/releases/tag/snapshot/2.1.0-20240828)
- [LUA Scripts for EdgeTx and OpenTx](https://github.com/rotorflight/rotorflight-lua-scripts/releases/tag/snapshot/2.1.0-20240828)
- [LUA Scripts for FrSky Ethos](https://github.com/rotorflight/rotorflight-lua-ethos/releases/tag/snapshot/2.1.0-20240828)

## Changes from 4.3.0

#### Custom CRSF/ELRS telemetry

A brand new custom telemetry has been implemented for ELRS. It allows
arbitrary number of sensors, not limited by the CRSF protocol.
Currently there are over 100 sensors to choose from, and a maximum of
40 sensors total. With the ELRS configurable telemetry ratio,
the sensors can be updated up to 20 times per second.

#### Swashplate wiggle indications

The swashplate is now indicating the FC readiness by doing a wiggle.
Also, any errors or arming failures are now clearly indicated.
Currently there are three different kind of wiggles:
- FC ready to be armed
- Arming not yet possible
- Armin permanent failure
- Armed succesfully (disabled by default)

#### SBUS2 support

SBUS2 telemetry is now available on the F7 and H7 targets.

#### Tx/Rx pinswap for serial ports

A support for swapping the Tx/Rx pins on serial receivers, ESC telemetry,
and transmitter telemetry UARTs have been added.

#### ESC Forward Programming support

A support for ESC Forward Programming has been added. So far,
three ESC vendors/models are supported:
- Hobbywing Platinum V5
- Scorpion
- YGE (beta firmware required)

#### BB logging for ESC telemetry

Three new BB log options have been added: ESC, BEC and ESC2.
These options are logging the data received from the ESC(s).

#### Other Changes

- Adjustments for acc trims
- Improvements in BB headers
- Improvements in Cyclic Cross-Coupling
- Fix adjfunction initilisation bug
- Better battery voltage filtering
- model_id parameter
- Various CMS fixes


***

# 4.3.0

This is the _first official_ Release of the Rotorflight-2 Firmware.

## Downloads

The download locations for Rotorflight 2.0.0 are:

- [Rotorflight Configurator](https://github.com/rotorflight/rotorflight-configurator/releases/tag/release/2.0.0)
- [Rotorflight Blackbox](https://github.com/rotorflight/rotorflight-blackbox/releases/tag/release/2.0.0)
- [LUA Scripts for EdgeTx and OpenTx](https://github.com/rotorflight/rotorflight-lua-scripts/releases/tag/release/2.0.0)
- [LUA Scripts for FrSky Ethos](https://github.com/rotorflight/rotorflight-lua-ethos/releases/tag/release/2.0.0)


## Notes

1. There is a new website [www.rotorflight.org](https://www.rotorflight.org/) for Rotorflight-2.
   The old Wiki in github is deprecated, and is for Rotorflight-1 only.
   Big thanks to the documentation team for setting this up!

2. Please download and install [Rotorflight Configurator](https://github.com/rotorflight/rotorflight-configurator/releases/) before attempting to flash this firmware.

3. Rotorflight-2 is **NOT** backward compatible with RF1. You **MUST NOT** load your configuration dump from RF1 into RF2.

4. If coming from RF1, please setup your helicopter from scratch for RF2. Follow the instructions on the website!

5. As always, please double check your configuration on the bench before flying!


## Support

The main source of Rotorflight information and instructions is now the [website](https://www.rotorflight.org/).

Rotorflight has a strong presence on the Discord platform - you can join us [here](https://discord.gg/FyfMF4RwSA/).
Discord is the primary location for support, questions and discussions. The developers are all active there,
and so are the manufacturers of RF Flight Controllers. Many pro pilots are also there.
This is a great place to ask for advice or discuss any complicated problems or even new ideas.

There is also a [Rotorflight Facebook Group](https://www.facebook.com/groups/rotorflight) for hanging out with other Rotorflight pilots.


## Changes

RF2 is based on Betaflight 4.3.x, and is rewritten from ground up, with the experience learned from Rotorflight-1.

Lots of things have changed in the two years of development. A full changelog can be found in
[git](https://github.com/rotorflight/rotorflight-firmware/commits/RF-4.3.x/).

### Changes since 4.3.0-RC3

- Improved rescue rate defaults
- Imrpoved baro filtering defaults


***

# 4.3.0-RC3

This is the _third_ Release Candidate of the Rotorflight-2 Firmware.

## Downloads

The official download locations for Rotorflight 2.0.0-RC3 are:

- [Rotorflight Configurator](https://github.com/rotorflight/rotorflight-configurator/releases/tag/release/2.0.0-RC3)
- [Rotorflight Blackbox](https://github.com/rotorflight/rotorflight-blackbox/releases/tag/release/2.0.0-RC3)
- [LUA Scripts for EdgeTx and OpenTx](https://github.com/rotorflight/rotorflight-lua-scripts/releases/tag/release/2.0.0-RC3)
- [LUA Scripts for FrSky Ethos](https://github.com/rotorflight/rotorflight-lua-ethos/releases/tag/release/2.0.0-RC3)


## Notes

1. There is a new website [www.rotorflight.org](https://www.rotorflight.org/) for Rotorflight-2.
   The old Wiki in github is deprecated, and is for Rotorflight-1 only.
   Big thanks to the documentation team for setting this up!

2. Please download and install [Rotorflight Configurator](https://github.com/rotorflight/rotorflight-configurator/releases/tag/release/2.0.0-RC3) before attempting to flash this firmware.

3. Rotorflight-2 is **NOT** backward compatible with RF1. You **MUST NOT** load your configuration dump from RF1 into RF2.

4. If coming from RF1, please setup your helicopter from scratch for RF2. Follow the instructions on the website!

5. As always, please double check your configuration on the bench before flying!


## Support

The main source of Rotorflight information and instructions is now the [website](https://www.rotorflight.org/).

Rotorflight has a strong presence on the Discord platform - you can join us [here](https://discord.gg/FyfMF4RwSA/).
Discord is the primary location for support, questions and discussions. The developers are all active there,
and so are the manufacturers of RF Flight Controllers. Many pro pilots are also there.
This is a great place to ask for advice or discuss any complicated problems or even new ideas.

There is also a [Rotorflight Facebook Group](https://www.facebook.com/groups/876445460825093) for hanging out with other Rotorflight pilots.


## Changes

RF2 is based on Betaflight 4.3.x, and is rewritten from ground up, with the experience learned from Rotorflight-1.

Lots of things have changed in the two years of development. A full changelog can be found in
[git](https://github.com/rotorflight/rotorflight-firmware/commits/RF-4.3.x/).

### Changes since 4.3.0-RC2

- Improved stability in hover
- Extended throttle PWM pulse limits to allow narrow band servos
- Extended governor max throttle range to 0..100%
- Fixed governor passthrough throttle ramp-down in IDLE
- Fixed CMS feature disabling with Spektrum


***

# 4.3.0-RC2

This is the _second_ Release Candidate of the Rotorflight-2 Firmware.

## Downloads

The official download locations for Rotorflight 2.0.0-RC2 are:

- [Rotorflight Configurator](https://github.com/rotorflight/rotorflight-configurator/releases/tag/release/2.0.0-RC2)
- [Rotorflight Blackbox](https://github.com/rotorflight/rotorflight-blackbox/releases/tag/release/2.0.0-RC2)
- [LUA Scripts for EdgeTx and OpenTx](https://github.com/rotorflight/rotorflight-lua-scripts/releases/tag/release/2.0.0-RC2)
- [LUA Scripts for FrSky Ethos](https://github.com/rotorflight/rotorflight-lua-ethos/releases/tag/release/2.0.0-RC2)


## Notes

1. There is a new website [www.rotorflight.org](https://www.rotorflight.org/) for Rotorflight-2.
   The old Wiki in github is deprecated, and is for Rotorflight-1 only.
   Big thanks to the documentation team for setting this up!

1. Please download and install [Rotorflight Configurator](https://github.com/rotorflight/rotorflight-configurator/releases/tag/release/2.0.0-RC2) before attempting to flash this firmware.

1. Rotorflight-2 is **NOT** backward compatible with RF1. You **MUST NOT** load your configuration dump from RF1 into RF2.

1. If coming from RF1, please setup your helicopter from scratch for RF2. Follow the instructions on the website!

1. As always, please double check your configuration on the bench before flying!


## Support

The main source of Rotorflight information and instructions is now the [website](https://www.rotorflight.org/).

Rotorflight has a strong presence on the Discord platform - you can join us [here](https://discord.gg/FyfMF4RwSA/).
Discord is the primary location for support, questions and discussions. The developers are all active there,
and so are the manufacturers of RF Flight Controllers. Many pro pilots are also there.
This is a great place to ask for advice or discuss any complicated problems or even new ideas.

There is also a [Rotorflight Facebook Group](https://www.facebook.com/groups/876445460825093) for hanging out with other Rotorflight pilots.


## Changes

RF2 is based on Betaflight 4.3.x, and is rewritten from ground up, with the experience learned from Rotorflight-1.

Lots of things have changed in the two years of development. A full changelog can be found online later.

### Changes since 4.3.0-RC1

- Introduce default PID D-gains
- Change default motor protocol to PWM
- Simplify RC frame rate calculation
- Reduce default RC smoothing level to minimum
- Add OpenYGE v3 protocol
- Fix a possible underflow bug in OpenYGE
- Fix Kontronik telemetry protocol
- Add github CI workflows
- Add GOV_MODE FrSky telemetry sensor
- Fix TTA headroom calculation in Passthrough mode
- Fix RPM filter error check with DD motors
- Fix scale lights terminology and documentation
- Implement Horizon mode inverted self-leveling


***

# 4.3.0-RC1

This is the first Release Candidate of the Rotorflight 2 Firmware.

## Downloads

The official download locations for Rotorflight 2.0.0-RC1 are:

- [Rotorflight Configurator](https://github.com/rotorflight/rotorflight-configurator/releases/tag/release/2.0.0-RC1)
- [Rotorflight Blackbox](https://github.com/rotorflight/rotorflight-blackbox/releases/tag/release/2.0.0-RC1)
- [LUA Scripts for EdgeTx](https://github.com/rotorflight/rotorflight-lua-scripts/releases/tag/release/2.0.0-RC1)
- [LUA Scripts for Ethos](https://github.com/rotorflight/rotorflight-lua-ethos/releases/tag/release/2.0.0-RC1)


## Notes

1. There is a new website [www.rotorflight.org](https://www.rotorflight.org/) for Rotorflight 2.
   The old Wiki in github is deprecated, and is for Rotorflight-1 only.
   Big thanks to the documentation team for setting this up!

1. Please download and install [Rotorflight Configurator](https://github.com/rotorflight/rotorflight-configurator/releases/tag/release/2.0.0-RC1) before attempting to flash this firmware.

1. Rotorflight 2 is **NOT** backward compatible with RF1. You **MUST NOT** load your configuration dump from RF1 into RF2.

1. If coming from RF1, please setup your helicopter from scratch for RF2. Follow the instructions on the website!

1. As always, please double check your configuration on the bench before flying!


## Support

The main source of Rotorflight information and instructions is now the [website](https://www.rotorflight.org/).

Rotorflight has a strong presence on the Discord platform - you can join us [here](https://discord.gg/FyfMF4RwSA/).
Discord is the primary location for support, questions and discussions. The developers are all active there,
and so are the manufacturers of RF Flight Controllers. Many pro pilots are also there.
This is a great place to ask for advice or discuss any complicated problems or even new ideas.

There is also a [Rotorflight Facebook Group](https://www.facebook.com/groups/876445460825093) for hanging out with other Rotorflight pilots.


## Changes

RF2 is based on Betaflight 4.3.x, and is rewritten from ground up, with the experience learned from Rotorflight-1.

Lots of things have changed in the two years of development. A full changelog can be found online later.

### Changes since 2.0.0-20240218

- Refactor MSP_SERVO_CONFIGURATIONS
- Use internal pull-up on FREQ input
- Use falling edge trigger on FREQ input
- Change default yaw precomp parameters
- Change default governor master gain
- Change default rates response time to 0 (no limit)
- Change default dynamic notch count to 4

