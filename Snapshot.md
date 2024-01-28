## Snapshot Release

This version is a development snapshot, intended to be used only for beta-testing.
It is not fully working nor stable, and should not be used by end-users.

Rotorflight-2 is based on Betaflight-4.3, and is rewritten from ground up,
with the experience learned from Rotorflight-1.

For more information, please join the Rotorflight Discord chat.


## Changes from 4.3.0-20240105

- Fix ESC temperature scale
- Fix Scorpion ESC telemetry current and voltage
- Fix esc_sensor_current_offset
- Fix ADC current sensor config
- Fix rcCommand rounding in MSP
- Fix battery percentage calculation
- Refactor MSP_BATTERY_STATE
- Relax RPM glitch filter thresholds
- Add Realtime scheduling load monitor
- Add Yaw precomp lowpass filter
- Add voltages to CRSF telemetry repurpose options
- Add setpoint acceleration and response time parameters
- Add feature flag for CMS
- Adjust CMS/OSD stick command tresholds
- Protect MSP_SET_RX_CONFIG from ELRS bug
- Require ACC calibration before arming
- Require fast RPM signal in governor and RPM filter
- Require battery voltage in governor mode 2
- Change default servo travel to 70°
- Add TTA DEBUG mode
- Update HSI curves
