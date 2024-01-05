## Snapshot Release

This version is a development snapshot, intended to be used only for beta-testing.
It is not fully working nor stable, and should not be used by end-users.

Rotorflight-2 is based on Betaflight-4.3, and is rewritten from ground up,
with the experience learned from Rotorflight-1.

For more information, please join the Rotorflight Discord chat.


## Changes from 4.3.0-20231127

- Update HSI curves
- Increase servo PWM resolution
- Add collective geometry correction
- Change Mixer swashplate trims to RPC
- Fix governor RPM glitch handling
- Fix yaw inversion with stick commands
- Fix a receiver bug with more than 18 channels
- Fix CMS build without OSD
- Enable CMS in Unified targets
- Fix tail motor startup throttle range
- Fix tail motor center trim
- Fix Adjfunc ena channel in CLI dump
- Fix MSC on G474
- Enable caches on G4
- Fix 1k gyro rate on slow MCUs
- Fix TTA with CCW main rotor
- Increase TTA headroom
- Change default PID mode to 3
- Change mixer default scaling to 50%
- Refactor ADC voltage and current sensors
- Add voltages and temperatures to Blackbox
- Add gyro_rate_sync parameter
- Change Rescue Alt.Hold I-term scale /10
- Remove unmaintained OSD targets

