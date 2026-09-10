# RX serial wiring auto-detect — design

Scope: for the **currently selected serial RX protocol**, detect the right
`serialrx_inverted` / `serialrx_halfduplex` / `serialrx_pinswap` combination
by trying each live and watching for signal. Protocol selection itself stays
manual — users know "it's an FBUS receiver," they don't know if their
board's inverter or their wiring guess was right, and that's the part
actually worth automating (8 combos per protocol is tractable).

## Why this needs a firmware change, not just a configurator loop

`rxInit()` only ever runs once, at boot. Every RX_CONFIG field today is
`SET` + `EEPROM_WRITE` + reboot. Cycling 8 combos that way means 8 real
EEPROM writes and 8 full reboot/MSP-reconnect cycles — slow, disruptive,
and needless flash wear for something that's inherently transient (only
the final answer should ever be persisted).

The good news: closing and reopening the RX serial port generically,
without provider cooperation, is already possible with existing plumbing:

- `findSerialPortConfig(FUNCTION_RX_SERIAL)` gives the identifier of
  whatever UART is assigned to the main RX.
- `findSerialPortUsageByIdentifier(identifier)` returns the
  `serialPortUsage_t*` holding the live `serialPort_t*` for that
  identifier, so it can be closed explicitly.
- Each provider's own init (`sbusInit`, `crsfRxInit`, etc., dispatched
  from `serialRxInit()`) just reopens the port itself, building
  `portOptions_e` straight from
  `rxConfig->serialrx_inverted/halfDuplex/pinSwap`. Nothing
  provider-specific needs touching — mutate those three fields in the
  live (RAM-only) `rxConfigMutable()` and re-run the same
  `serialRxInit()` dispatch for the current `serialrx_provider`.

So the mechanism is: **close current port → flip the trial bits in RAM →
re-run the same init switch that already exists → poll
`rxIsReceivingSignal()`**, which the RX task already updates every cycle
regardless of this feature.

## Firmware: state machine (rx.c)

A small non-blocking state machine, ticked from `rxFrameCheck()` every RX
task cycle — not run inside the MSP handler, since a combo can take up to
~2s to settle and MSP processing must stay non-blocking.

```c
typedef enum {
    RX_SERIAL_TRIAL_IDLE,
    RX_SERIAL_TRIAL_RUNNING,
    RX_SERIAL_TRIAL_SUCCESS,
    RX_SERIAL_TRIAL_FAILED,
    RX_SERIAL_TRIAL_REJECTED,   // no serial RX feature/port, already armed, etc.
} rxSerialTrialState_e;

typedef struct rxSerialTrialRuntime_s {
    rxSerialTrialState_e state;
    uint8_t comboIndex;         // 0..7, current (or last, on FAILED) attempt
    uint8_t comboCount;         // actual combo count this run (< 8 for CRSF/GHST, see below)
    uint8_t comboOrder[8];
    timeMs_t comboStartedAt;
    timeMs_t signalSince;       // 0 until rxIsReceivingSignal() first goes true for this combo
    timeMs_t lastPollAt;        // watchdog: configurator must keep polling
    uint8_t savedInverted, savedHalfDuplex, savedPinSwap;         // restore point
    uint8_t savedCrsfUseNegotiatedBaud, savedSbusBaudFast, savedSrxl2BaudFast; // see below
} rxSerialTrialRuntime_t;
```

- **`rxSerialTrialStart()`** — guards: not already running, not armed
  (`ARMING_FLAG(ARMED)` — the trial cycles the live RX connection, which
  is not safe to do while armed), `FEATURE_RX_SERIAL` active and a port
  actually assigned. Snapshots the wiring bits and the baud-negotiation
  fields (see below) as the restore point, builds `comboOrder` (Hamming
  walk, below), applies combo 0, closes+reopens the port via
  `serialRxInit()`, records `comboStartedAt`.
- **`rxSerialTrialTick()`** (called every RX task cycle, cheap early-out
  when idle):
  - If `state != RUNNING`, nothing to do.
  - Watchdog: if `now - lastPollAt > 3000ms`, treat as an abandoned trial
    (configurator crashed / USB unplugged mid-scan) → restore + `IDLE`.
  - If `rxIsReceivingSignal()` has been continuously true for a short
    debounce window → `SUCCESS`, stop touching the port further (leave
    the winning combo live so the user can literally see channels move
    before deciding to save).
  - Else if `now - comboStartedAt > settleMs[provider]` → advance
    `comboIndex`; if that was the last combo, restore original bits,
    reinit, → `FAILED`; else apply next combo, close+reopen, reinit.
- **`rxSerialTrialStop()`** — always restores the snapshotted bits and
  reinits, regardless of current state. Idempotent. This is the only way
  a `SUCCESS` combo becomes "real" from the firmware's point of view — it
  doesn't persist anything itself, ever.

### CRSF baud-negotiation confound

`crsf_use_negotiated_baud` (when enabled) makes `crsfRxInit()` open the
port at `getCrsfCachedBaudrate()` — a value cached from a *previous* CRSF
V3 negotiation — rather than the plain default baud. That cache can be
stale, which means every combo in a trial (including the electrically
correct one) could get opened at the wrong baud: a confound none of the
three wiring bits can fix. `rxSerialTrialStart()` forces
`crsf_use_negotiated_baud`/`sbus_baud_fast`/`srxl2_baud_fast` off for the
duration of the trial and restores them alongside the wiring bits.

### CRSF/GHST don't vary half-duplex

Both protocols hardcode their own bidirectional framing
(`rx/crsf.c`'s port mode, `rx/ghst.c`'s port options both bake in
`SERIAL_BIDIR` unconditionally) and never read `rxConfig->halfDuplex`.
Varying it during a trial can't change the outcome for these two — it
only wastes combos on electrical duplicates and reports a half-duplex
value in the result that had nothing to do with why signal was found.
`rxSerialTrialProtocolIgnoresHalfDuplex()` skips half-duplex variation
for these providers, which is also why `comboCount` can be less than 8 —
the combo-order build filters those combos out rather than trying them.

### Debounce

`rxSignalReceived` already flips `true` on the very next valid frame and
`false` after an internal invalid-frame timeout, so a single stray
checksum-lucky garbage byte is unlikely to read as sustained signal.
Still, `signalSince` requires it held for a short continuous window
before calling a combo a `SUCCESS`, rather than trusting one
instantaneous poll.

### Per-protocol settle timeout

Streaming protocols (S.BUS, CRSF, IBUS, FPort/FBUS, GHST) lock on within
a frame or two. Handshake-based ones (SRXL2, Jeti EXBUS, Spektrum
bind-flavored modes) need materially longer before anything valid shows
up. `rxSerialTrialSettleMs()` uses a default (1000ms) and a longer tier
(2200ms) for the handshake protocols. Worst case (8 combos × slowest
timeout) is well under 20s if every combo fails — acceptable for a
manual "click and wait a few seconds" action, and the scan order below
makes the common case much faster.

### Scan order — walk outward from current settings, not 0..7

Most real-world misconfigurations are "one thing is wrong" (usually
`inverted`, from a board with/without a hardware inverter), not
"everything is wrong." Order the 8 combos by Hamming distance from
whatever's currently configured: distance 0 (current — confirms the
existing setting still isn't the problem, cheap to rule out first), then
the three distance-1 flips, then the three distance-2 flips, then the
distance-3 (all flipped) as last resort. Converges in 1 try for the
common single-bit-wrong case instead of averaging 4.

## New MSP2 command

Defined in `msp_protocol_v2_rotorflight.h`, alongside the other
rotorflight-specific MSP2 extensions:

| Command | ID | Request | Response |
|---|---|---|---|
| `MSP2_RX_SERIAL_TRIAL` | `0x5F0B` | `U8 action` (0 = poll, 1 = start, 2 = stop) | `U8 state`, `U8 comboIndex`, `U8 inverted`, `U8 halfDuplex`, `U8 pinSwap`, `U16 elapsedMs` |

Single command for start/poll/stop (action byte), rather than three
separate commands — every poll response also serves as the watchdog
keep-alive (updates `lastPollAt`).

## Configurator side (rotorflight-configurator)

- A "Detect Wiring" button next to the serial RX protocol select,
  enabled only when the current RX protocol uses `FUNCTION_RX_SERIAL`.
- On click: send `action=1` (start); poll `action=0` on a ~200ms cadence,
  showing "Trying combo N of M…".
- On `SUCCESS`: apply the *returned* `inverted`/`halfDuplex`/`pinSwap`
  into local RX config state — this is the only place the result becomes
  visible, and it flows through the tab's existing dirty-diff/Save/Revert
  machinery exactly like any manually-edited field. Nothing is pushed to
  the FC until the user hits the normal Save/Reboot button. The dialog
  does not auto-close: closing without saving discards the result
  (reverts to the pre-trial values), so an unattended auto-close would
  silently throw away a successful detection — the user has to take an
  explicit action (Save and Reboot, or Close to discard).
- On `FAILED`: offer Retry, touch nothing else.
- Always send `action=2` (stop) on dialog close / component unmount, not
  just on terminal states — covers the user closing the dialog mid-scan.
  The firmware's 3s watchdog is the last-resort backstop if even that
  doesn't arrive (e.g. the whole app closing).
- Guarded client-side against running while armed, on top of the
  firmware's own `ARMING_FLAG(ARMED)` rejection.

This keeps the trial's blast radius genuinely contained: the FC can
wander across up to 8 electrical configs for a few seconds while the
dialog is open, but from the user's (and the rest of the tab's) point of
view nothing is "real" until the ordinary Save button is pressed — same
contract as every other field on this page today.
