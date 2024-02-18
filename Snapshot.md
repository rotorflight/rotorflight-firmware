## Snapshot Release

This version is a development snapshot, intended to be used only for beta-testing.
It is not fully working nor stable, and should not be used by end-users.

Rotorflight-2 is based on Betaflight-4.3, and is rewritten from ground up,
with the experience learned from Rotorflight-1.

For more information, please join the Rotorflight Discord chat.


## Changes from 4.3.0-20240128

- More CRSF reuse options
- Fix CRSF headspeed reuse with EdgeTx >= 2.9.3
- Fix governor autorotation timeout limits
- Fix battery cell count detection with 10S and 12S
- Add OpenYGE ESC telemetry
- Add lowpass filter to governor feedforward
- Refactor MSP_BATTERY_CONFIG
- Improved defaults
   - Stick deadband and deflection
   - Rescue gains and collective levels
   - Governor filter cutoffs
   - Cross-coupling gain
   - PID error limits
