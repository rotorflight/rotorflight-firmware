## Snapshot Release

This is a Rotorflight firmware development snapshot for beta-testing.

Note! This snapshot changes the format of a few MSP messages.

Note! Please update all Rotorflight components at the same time.


## Changes from 20220307

- Disable arming if CPU load is over 75%

- Refactor gyro related MSP messages

- Change PID gain variables to 16bit

- Change PID gain MSP messages to match

- Extend range of the gain parameters
  - PID gains: 250 => 1000
  - Yaw precompensation: 250 => 2500

- Add 'enable_stick_commands' parameter

- Use collective instead of throttle with stick commands




