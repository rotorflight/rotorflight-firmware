## Snapshot Release

This version is a development snapshot, intended to be used only for beta-testing.
It is not fully working nor stable, and should not be used by end-users.

Rotorflight-2 is based on Betaflight-4.3, and is rewritten from ground up,
with the experience learned from Rotorflight-1.

For more information, please join the Rotorflight Discord chat.


## Changes from 4.3.0-20230912

- RX subsystem refactored
- RX failsafe refactored
- Dynamic Notch filter adapted for helis
- ESC telemetry implemented for various ESCs
  - Hobbywing Platinum V4
  - Hobbywing Platinum V5
  - Hobbywing FlyFun V5
  - Scorpion UNC
  - Kontronik
  - OMP Hobby
  - ZTW Skyhawk
  - APD Pro/HV
- Servo speed calculation fixed to use ms/60°
- PID Mode 3 fixed to use P-term on error
- PT1 filter cutoff calculation fixed
- MSP over FrSky telemetry speed improved
- RPM calculation accuracy improved
- More CRSF FM reuse options added

