## Snapshot Release

This version is a development snapshot, intended to be used only for beta-testing.
It is not fully working nor stable, and should not be used by end-users.

Rotorflight-2 is based on Betaflight-4.3, and is rewritten from ground up,
with the experience learned from Rotorflight-1.

For more information, please join the Rotorflight Discord chat.


## Changes from 4.3.0-20230822

- Governor gov_max_throttle parameter added
- Governor tracking_speed fixed in passthrough mode
- RPM filter fixed if only main rotor RPM available
- RPM filter update fixed for lower CPU load
- Cyclic Cross-Coupling refactored
- Collective-to-yaw precomp not used during spoolup
- TTA Collective correction refactored
- TTA not used during spoolup
- H7 TIMUP configuration bug fixed
- BlackBox multi-file logging fixed (works on W25N01G too)
- BlackBox erase refactored - works also while logging
- Boost and Offset terms added to BlackBox
- PID Test Mode 3 added for HSI testing
- Tail center trim scaling changed
- Tail motor thrust law changed to linear
- Many new parameters added to MSP
- Many new parameters added to Adjustment Functions

