# RX serial wiring auto-detect — design

Scope: for the **currently selected serial RX protocol**, detect the right
`serialrx_inverted` / `serialrx_halfduplex` / `serialrx_pinswap` combination
by trying each live and watching for signal. Protocol selection itself stays
manual (see prior discussion — 8 combos per protocol is tractable, ~160 across
all protocols is not, and most of the value is here anyway: users know "it's
an FBUS receiver," they don't know if their board's inverter or their wiring
guess was right).

## Why this needs a firmware change, not just a configurator loop

`rxInit()` ([rx.c:281](../src/main/rx/rx.c#L281)) only ever runs once,
at boot ([init.c:770](../src/main/fc/init.c#L770)). Every RX_CONFIG
field is `SET` + `EEPROM_WRITE` + reboot today
([Receiver.svelte:110-124](../../wingflight-configurator/src/tabs/receiver/Receiver.svelte#L110)).
Cycling 8 combos that way means 8 real EEPROM writes and 8 full reboot/MSP-
reconnect cycles — slow, disruptive, and needless flash wear for something
that's inherently transient (only the final answer should ever be persisted).

The good news: closing and reopening the RX serial port generically, without
provider cooperation, is already possible with existing plumbing:

- `findSerialPortConfig(FUNCTION_RX_SERIAL)` gives the identifier of whatever
  UART is assigned to the main RX.
- `findSerialPortUsageByIdentifier(identifier)` is already public
  ([serial.h:144](../src/main/io/serial.h#L144)) and returns the
  `serialPortUsage_t*` holding the live `serialPort_t*` for that identifier.
- `closeSerialPort()` ([serial.c:445](../src/main/io/serial.c#L445))
  releases it and marks the slot `FUNCTION_NONE` again.
- Each provider's own init (e.g. `sbusInit`,
  [sbus.c:225-234](../src/main/rx/sbus.c#L225)) just reopens the port
  itself, building `portOptions_e` straight from
  `rxConfig->serialrx_inverted/halfDuplex/pinSwap`. Nothing provider-specific
  needs touching — mutate those three fields in the live (RAM-only)
  `rxConfigMutable()` and re-run the existing `serialRxInit()` dispatch
  ([rx.c:172](../src/main/rx/rx.c#L172)) for the current
  `serialrx_provider`.

So the mechanism is: **close current port → flip the trial bits in RAM →
re-run the same init switch that already exists → poll
`rxIsReceivingSignal()`** ([rx.c:391](../src/main/rx/rx.c#L391)),
which is already exposed to MSP as `mainLinkUp` in
`MSP2_WING_RX_INPUT_BACKUP_STATUS` and already polled every 200ms by the
configurator today.

## Firmware: new state machine (rx.c)

A small non-blocking state machine, ticked from the scheduler like any other
periodic task — **not** run inside the MSP handler, since a combo can take
up to ~1s to settle and MSP processing must stay non-blocking.

```c
typedef enum {
    RX_TRIAL_IDLE,
    RX_TRIAL_RUNNING,
    RX_TRIAL_SUCCESS,
    RX_TRIAL_FAILED,
} rxTrialState_e;

typedef struct {
    rxTrialState_e state;
    uint8_t comboIndex;         // 0..7, current attempt
    uint8_t comboOrder[8];      // walk order, see below
    timeMs_t comboStartedAt;
    timeMs_t lastPollAt;        // watchdog: configurator must keep polling
    bool savedInverted, savedHalfDuplex, savedPinSwap; // restore point
} rxSerialTrial_t;
```

- **`rxSerialTrialStart()`** — guards: `FEATURE_RX_SERIAL` active, a serial
  port is actually assigned, no trial already running. Snapshots the three
  current bits as the restore point, builds `comboOrder` (below), sets
  `comboIndex = 0`, applies combo 0, closes+reopens the port, calls
  `serialRxInit()`, records `comboStartedAt`.
- **`rxSerialTrialTick()`** (called every scheduler pass, cheap early-out
  when idle):
  - If `state != RUNNING`, nothing to do.
  - Watchdog: if `now - lastPollAt > 3000ms`, treat as an abandoned trial
    (configurator crashed / USB unplugged mid-scan) → restore + `IDLE`.
  - If `rxIsReceivingSignal()` has been continuously true for the debounce
    window (see below) → `SUCCESS`, stop touching the port further (leave
    the winning combo live so the user can literally see channels move
    before deciding to save).
  - Else if `now - comboStartedAt > settleMs[provider]` → advance
    `comboIndex`; if that was the last of 8, restore original bits,
    reinit, → `FAILED`; else apply next combo, close+reopen, reinit.
- **`rxSerialTrialStop()`** — always restores the snapshotted bits and
  reinits, regardless of current state. Idempotent. This is the only way a
  `SUCCESS` combo becomes "real" from the firmware's point of view — it
  doesn't persist anything itself, ever.

### Debounce

`rxSignalReceived` already flips `true` on the very next valid frame and
`false` after an internal invalid-frame timeout
([rx.c:545-554](../src/main/rx/rx.c#L545)), so a single stray
checksum-lucky garbage byte is already unlikely to read as sustained signal.
Still, require it held for a short continuous window (e.g. 150–250ms) before
calling a combo a `SUCCESS`, rather than trusting one instantaneous poll —
cheap insurance, and easy to tune from bench testing.

### Per-protocol settle timeout

Streaming protocols (S.BUS, CRSF, IBUS, FPort/FBUS, GHST) lock on within a
frame or two — a few hundred ms of budget is generous. Handshake-based ones
(SRXL2, Jeti EXBUS, Spektrum bind-flavored modes) need materially longer
before anything valid shows up. Rather than guess exact numbers here, add a
`settleMs[]` lookup keyed by `serialrx_provider` with a conservative default
(~600ms) and larger values (~1200-1500ms) for the handshake protocols,
then tune from real receivers on the bench. Worst case (8 combos × slowest
timeout) is ~10-12s if every combo fails — acceptable for a manual
"click and wait a few seconds" action, and the scan order below makes the
common case much faster.

### Scan order — walk outward from current settings, not 0..7

Most real-world misconfigurations are "one thing is wrong" (usually
`inverted`, from a board with/without a hardware inverter), not "everything
is wrong." Order the 8 combos by Hamming distance from whatever's currently
configured: distance 0 (current — confirms the existing setting still isn't
the problem, cheap to rule out first), then the three distance-1 flips, then
the three distance-2 flips, then the distance-3 (all flipped) as last
resort. Converges in 1 try for the common single-bit-wrong case instead of
averaging 4.

## New MSP2 commands

Next free IDs after `0x5F11` ([msp_protocol.h:305-322](../src/main/msp/msp_protocol.h#L305)):

| Command | ID | Request | Response |
|---|---|---|---|
| `MSP2_WING_RX_SERIAL_TRIAL_START` | `0x5F12` | *(none — always trials the currently-configured `serialrx_provider`)* | `U8 accepted` (0 = rejected: no serial RX feature/port, or trial already running) |
| `MSP2_WING_RX_SERIAL_TRIAL_STATUS` | `0x5F13` | *(none)* | `U8 state` (idle/running/success/failed), `U8 comboIndex`, `U8 inverted`, `U8 halfDuplex`, `U8 pinSwap`, `U16 elapsedMs` — also **is** the watchdog keep-alive (updates `lastPollAt`) |
| `MSP2_WING_RX_SERIAL_TRIAL_STOP` | `0x5F14` | *(none)* | `U8 ack` — always restores + reinits, idempotent |

Mirrors the existing `MSP2_WING_RX_INPUT_BACKUP_STATUS` handler pattern
([msp.c:1321](../src/main/msp/msp.c#L1321)) closely enough to reuse
its shape.

## Configurator side (wingflight-configurator)

- `ReceiverType.svelte`: a "Detect wiring" button next to the protocol
  select, enabled only when `RX_PROTOCOLS[rxProtoIndex]?.feature ===
  "RX_SERIAL"` (same guard already used for the signaling SubSection at
  [ReceiverType.svelte:65](../../wingflight-configurator/src/tabs/receiver/ReceiverType.svelte#L65)).
- On click: send `TRIAL_START`; if accepted, poll `TRIAL_STATUS` on the same
  ~150-200ms cadence the tab already uses for
  `MSP2_WING_RX_INPUT_BACKUP_STATUS`
  ([Receiver.svelte:89-91](../../wingflight-configurator/src/tabs/receiver/Receiver.svelte#L89)),
  showing "Trying combo N of 8…".
- On `SUCCESS`: send `TRIAL_STOP` (commits nothing on the FC, just cleans up
  the trial and restores original live bits), then apply the *returned*
  `inverted`/`halfDuplex`/`pinSwap` into local `FC.RX_CONFIG` — this is the
  only place the result becomes visible, and it flows through the tab's
  existing dirty-diff/Save/Revert machinery exactly like any manually-edited
  field. Nothing is pushed to the FC until the user hits the normal
  Save/Reboot button. Toast a friendly success message.
- On `FAILED`: send `TRIAL_STOP`, toast "no signal found on this protocol —
  check wiring/provider," touch nothing.
- Always send `TRIAL_STOP` on component unmount / tab navigation, not just
  on terminal states — covers the user clicking away mid-scan. (Firmware's
  3s watchdog is the last-resort backstop if even that doesn't arrive.)

This keeps the trial's blast radius genuinely contained: the FC can wander
across up to 8 electrical configs for a few seconds while the button spins,
but from the user's (and the rest of the tab's) point of view nothing is
"real" until the ordinary Save button is pressed — same contract as every
other field on this page today.

## Open items to settle before/while implementing

1. Exact `settleMs[]` values per provider — needs bench time with real
   receivers, not a guess from reading code.
2. Where the scheduler task hook goes (need to check `fc/tasks.c` task table
   conventions) and confirm ticking a lightweight trial state machine there
   doesn't need its own `TASK_*` slot vs. piggybacking on the existing RX
   task.
3. Whether `SUCCESS` should leave the winning combo *live* (so the user sees
   channel bars move as confirmation before saving) vs. immediately
   restoring and just reporting the numbers — leaning toward "leave it live
   until STOP," since seeing channels twitch is a much stronger confidence
   signal than a text result, but it does mean the FC sits on unsaved wiring
   for slightly longer.
