# ESC telemetry wiring auto-detect — design

Same mechanism as [docs/rx-wiring-autodetect-design.md](rx-wiring-autodetect-design.md),
applied to the ESC telemetry port (`escSensorConfig`) instead of the RX
port(s). Read that doc first for the underlying rationale (why this needs a
live re-init rather than just a configurator-side reboot loop, the general
shape of the trial state machine, the UART invert/pin-swap latch bug it
uncovered and fixed). This note only covers what's different here.

## What's different from the RX version

- **No `inverted` bit.** `escSensorInit()` ([esc_sensor.c:4471](../src/main/sensors/esc_sensor.c#L4471))
  always opens the port `SERIAL_NOT_INVERTED` - ESC telemetry protocols don't
  invert. So this is a 4-combo search (`halfDuplex` × `pinSwap`), not 8.
- **No baud confound.** Every protocol here has a baud fixed by the protocol
  itself (115200/19200/38400 depending on which), not a user-configurable or
  negotiated one - CRSF's `crsf_use_negotiated_baud` had no analogue to worry
  about.
- **Graupner requires half-duplex to function at all** (single-wire
  request/response - `validateAndFixEscSensorConfig()` already forces it on
  for exactly this reason), rather than merely ignoring the bit the way
  CRSF/GHST do for RX. Pinned rather than varied, same treatment either way.
- **Better signal-detection primitive.** The RX version used
  `rxIsReceivingSignal()`, a boolean that decays after ~100ms without a fresh
  frame - fine for streaming protocols, but it flickered false between
  frames for anything slower than ~10Hz and produced a real false-negative
  on CRSF during bench testing. ESC telemetry already has a plain monotonic
  `totalFrameCount` counter, incremented by every protocol's own decode path
  only on a structurally/checksum-valid frame. Success is declared once that
  counter has advanced by `ESC_SENSOR_TRIAL_MIN_FRAMES` (2) since the combo
  was applied - a delta check, immune to the "gap between frames" flicker
  entirely, so no debounce timer was needed this time.
- **Some protocols don't have a UART to test at all**: Castle Link (PWM edge
  timing, no serial port), FBUS and SRXL2 ESC telemetry (each reads via its
  own dedicated port function, not `FUNCTION_ESC_SENSOR`). All three are
  clean `REJECTED` cases, mirroring "no serial port assigned."
- **Settle timing split by decode mechanism, not per-protocol guesswork**:
  protocols that go through the shared "rrfsm" (request/response) decode
  engine (HW5, Scorpion, OpenYGE, FLY, Graupner, XDFLY, ZTW, OMPHOBBY - see
  the dispatch switch in `escSensorProcess()`) poll the ESC rather than just
  listening to a continuous stream, so get the longer settle tier. BLHeli32,
  HW4, Kontronik, APD, and Record get the default (faster) tier.

## MSP

One new command, `MSP2_WING_ESC_SENSOR_TRIAL` (`0x5F14` - `0x5F12`/`0x5F13`
are claimed by the RX wiring auto-detect PR's `MSP2_WING_RX_SERIAL_TRIAL`/
`MSP2_WING_RX_INPUT_BACKUP_TRIAL`, deliberately skipped rather than
colliding, since both branches were cut from `master` independently and
each just grabbed the next free slot from its own starting point). Same
start/poll/stop action-byte shape as the RX version and
`MSP2_WING_BOARD_AUTO_ALIGN`. Response layout is one field shorter than the
RX version's (no `inverted` byte):

`U8 state, U8 comboIndex, U8 halfDuplex, U8 pinSwap, U16 elapsedMs`

## Open items pending bench testing

Same category as the RX version: the settle-time tiers are informed
starting points (this time from which decode engine a protocol uses, not a
guess at protocol identity alone), not measurements. Needs real ESCs on the
bench across a few different telemetry protocols before trusting the
timing, same as CRSF needed for the RX version.
