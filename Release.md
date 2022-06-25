This is Release 4.2.13 of the _Rotorflight Flight Control_ / _FBL_ firmware.

__Please read the notes below before flashing the Rotorflight firmware:__

1. Please read our great [Wiki](https://github.com/rotorflight/rotorflight/wiki) and consider joining our [Discord chat](https://discord.gg/FyfMF4RwSA) too!

2. Start with installing the latest version of [Rotorflight Configurator](https://github.com/rotorflight/rotorflight-configurator/releases)

3. Update all Rotorflight components at the same time: [Configurator](https://github.com/rotorflight/rotorflight-configurator/releases), [BlackBox](https://github.com/rotorflight/rotorflight-blackbox/releases) and [LUA Scripts](https://github.com/rotorflight/rotorflight-lua-scripts/releases).

4. [Backup](https://github.com/rotorflight/rotorflight/wiki/Back-up-and-restore) your current FC configuration. Whether it is still running Betaflight, or an earlier version of Rotorflight, please save the configuration dump. It is likely to be useful later on.

5. Please select "Full chip erase" if upgrading from an earlier Rotorflight _snapshot_ or Betaflight.

6. Please double-check your configuration before attempting to fly!


## Changelog

Please see the [git history ](https://github.com/rotorflight/rotorflight-firmware/commits/master)for a complete list of changes.


### Changes from 4.2.12

- Collective to Pitch (Elevator) precomp added (CLI only)

- Swash phase setting added (CLI only)

- ICM42688P gyro support added

- BMI270 gyro support added

- OSR4 mode added for BMI270

- Extra blackbox headers added

- Tail idle fixed in early spoolup

- iterm_decay=0 bug fixed

- Mixer saturation bug fixed

- Yaw CW/CCW stop gain range extended to 25..250

- Motor PWM rate range extended to 50..480

- RC interpolation disabled

- Absolute control disabled

