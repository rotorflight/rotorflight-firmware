## Snapshot Release

This version is a development snapshot, intended to be used only for beta-testing.
It is not fully working nor stable, and should not be used by end-users.

Rotorflight-2 is based on Betaflight-4.3, and is rewritten from ground up,
with the experience learned from Rotorflight-1.

For more information, please join the Rotorflight Discord chat.

**NOTE!** This Snapshot changes the PID gain scaling.

You *MUST* modify your PID gains as follows:

 - Pitch and Roll I-gain: divide by 2
 - Pitch D-gain: divice by 10

**NOTE!** This Snapshot changes the Collective curve scaling.

You *MUST* check and fix the Collective "rates" curve in the Rates tab.


## Changes from 4.3.0-20230628

- Increase Governer total FF limit to 50%
- Fix Rescue MSP_RESCUE_PROFILE message
- Fix Rescue max_collective parameter handling
- Refactor collective-to-yaw precomp again
- Reorganise Adjustment Functions
- Fix LED strip and ADC on H7
- Change Collective curve (rates) scale
- Change PID gain scaling
