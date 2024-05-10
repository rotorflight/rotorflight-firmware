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

