## Snapshot Release

This version is a development snapshot, intended to be used only for beta-testing.
It is not fully working nor stable, and should not be used by end-users.

Rotorflight-2 is based on Betaflight-4.3, and is rewritten from ground up,
with the experience learned from Rotorflight-1.

For more information, please join the Rotorflight Discord chat.


## Changes from 4.3.0-20231120

- Governor throttle jump fixed
- More BB fields enabled by default

## Changes from 4.3.0-20231118

- Flashing STM32H743 with the Configurator fixed

## Changes from 4.3.0-20231018

- Realtime scheduling refactored
- Blackbox buffering refactored
- Gyro and PID loop speed limits added
- Multi-gyro support disabled on F411 and G4
- Attitude telemetry reuse added to CRSF/ELRS
- Gyro drivers updated from BF/master
- ICM42688P AFSR fix integrated
- W25N01G flash chip driver fixed
- rc_arm_throttle parameter added
- gov_handover_throttle parameter added
- Default telemetry sensor list updated
- Default swashplate type changed to CP120
- DFU mode entry fixed with G4
- TTA gain doubled (half your TTA gain!)
