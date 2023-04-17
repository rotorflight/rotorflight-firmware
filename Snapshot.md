## Snapshot Release

This version is a development snapshot, intended to be used only for beta-testing.
It is not fully working nor stable, and should not be used by end-users.

Rotorflight-2 is based on Betaflight-4.3, and is rewritten from ground up,
with the experience learned from Rotorflight-1.

For more information, please join the Rotorflight Discord chat.

## Changes from 4.3.0-20230508

- RPM filter refactored
- ESC telemetry refactored
- Gyro filter config sanity checks added
- motor_rpm_lpf default changed to 100Hz
- fterm_filter removed in PID mode 9
- pid_dterm_mode and pid_dterm_mode_yaw enabled
- debug_axis parameter changed to an integer value
- led CLI command fixed to be backwards compatible
- SPI ELRS binding bugfix cherry-picked
