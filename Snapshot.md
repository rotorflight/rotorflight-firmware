## Snapshot Release

This version is a development snapshot, intended to be used only for beta-testing.
It is not fully working nor stable, and should not be used by end-users.

Rotorflight-2 is based on Betaflight-4.3, and is rewritten from ground up,
with the experience learned from Rotorflight-1.

For more information, please join the Rotorflight Discord chat.


## Changes from 4.3.0-20230724

- Add linear decay limit to error decay
- Add tail_center_trim parameter
- Add cyclic servo speed equaliser
- Add cyclic crosstalk precomp
- Add setpoint boost (B-term)
- Fix cyclic limit handling in the mixer
- Fix collective rates default from 25° to 12.5°
- Fix PID gain defaults
- Remove PID Mode 9

