This is the first official release of the _Rotorflight Flight Control_ / _FBL_ firmware.

__Please read the notes below before flashing the Rotorflight firmware:__

1. Please read our great [Wiki](https://github.com/rotorflight/rotorflight/wiki) and consider joining our [Discord chat](https://discord.gg/FyfMF4RwSA) too!

2. Start with installing the latest version of [Rotorflight Configurator](https://github.com/rotorflight/rotorflight-configurator/releases)

3. Update all Rotorflight components at the same time: [Configurator](https://github.com/rotorflight/rotorflight-configurator/releases), [BlackBox](https://github.com/rotorflight/rotorflight-blackbox/releases) and [LUA Scripts](https://github.com/rotorflight/rotorflight-lua-scripts/releases).

4. [Backup](https://github.com/rotorflight/rotorflight/wiki/Back-up-and-restore) your current FC configuration. Whether it is still running Betaflight, or an earlier version of Rotorflight, please save the configuration dump. It is likely to be useful later on.

5. Please select "Full chip erase" if upgrading from an earlier Rotorflight _snapshot_ or Betaflight.

6. Please double-check your configuration before attempting to fly!


## Changelog

Please see the [git history ](https://github.com/rotorflight/rotorflight-firmware/commits/master)for a complete list of changes.

### Changes from RC2

- Fix RPM sensor on F4
- Fix I-term windup in PASSTHRU mode
- Fix blackbox throttle with bidir tail motors
- Fix battery voltage requirement in governor
- Fix DFU detection on slow systems
- Change default motor PWM rate to 250Hz

### Changes from RC1

- PID normalization removed
- Support for Winbond W25Q64 flash chip added

