# ESC Rework Context

Temporary handoff note for ongoing ESC telemetry / programming rework.

## Purpose

This file exists to help work continue cleanly across different systems / agents.

The current goal is to redesign the ESC telemetry and forward-programming stack so it is easier to maintain and gives clients a cleaner MSP API.

## User Goals

- Throw away the old ESC telemetry / programming API shape and rebuild it in a better way.
- Add MSP commands that let the client know when an ESC write has actually completed.
- Keep MSP payloads below 80 bytes.
- Split ESC identity out from parameter payloads.
- Add a separate MSP call for ESC model / name.

## What We Found

The current implementation is structurally tangled:

- `src/main/sensors/esc_sensor.c`
  - Very large file.
  - Mixes live telemetry decoding, parameter transport, forward programming, write scheduling, cached parameter state, and MSP-facing behavior.
- `src/main/msp/msp.c`
  - Exposes legacy ESC MSP calls.
  - Delegates to helper functions, but the real write/read state machines live in `esc_sensor.c`.
- `src/main/io/serial_4way.c`
  - Owns the low-level 4way transport.

Important observations:

- ESC writes are currently async in some paths, but MSP has no explicit completion model.
- The MSP parameter read path is side-effectful.
- Some protocols bundle identity and parameters together.
- There is little or no focused unit test coverage around ESC MSP / programming flows.

## Key Current Code References

- Legacy MSP ESC handling:
  - `src/main/msp/msp.c`
  - `MSP_ESC_PARAMETERS`
  - `MSP_SET_ESC_PARAMETERS`
  - `MSP_SET_4WIF_ESC_FWD_PROG`
- 4way transport:
  - `src/main/io/serial_4way.c`
  - `src/main/io/serial_4way.h`
- ESC telemetry / params / programming state:
  - `src/main/sensors/esc_sensor.c`
  - `src/main/sensors/esc_sensor.h`
- ESC sensor config enum:
  - `src/main/pg/esc_sensor.h`

## Design Direction Agreed So Far

Do not break legacy MSP ESC commands immediately.

Instead:

1. Add a new MSPv2 ESC API beside the legacy commands.
2. Put new logic in a dedicated `msp_esc` module instead of growing `msp.c`.
3. Add a small ESC service layer that exposes stable concepts:
   - ESC info
   - ESC name / model
   - write operation status
4. Use the existing backend code only as a temporary implementation detail.
5. Migrate protocol backends behind cleaner interfaces later.

## Intended New API Shape

Planned MSPv2-style commands:

- ESC list / capabilities
- ESC info
- ESC name
- ESC telemetry
- ESC parameter metadata
- ESC parameter read
- ESC write begin
- ESC write data
- ESC write status
- ESC session close

Not all of these are implemented yet.

## First Implementation Slice

The first slice being started is:

- define a new MSPv2 ESC namespace
- add a dedicated `msp_esc` module
- expose explicit write-status reporting
- expose separate ESC info / name calls

This is intended to create the new migration path without immediately rewriting every backend.

## Implemented In This Slice

The following pieces are now in the tree:

- New MSPv2 ESC command ids in `src/main/msp/msp_protocol_v2_betaflight.h`
  - `MSP2_GET_ESC_INFO`
  - `MSP2_GET_ESC_NAME`
- `MSP2_GET_ESC_WRITE_STATUS`
- `MSP2_GET_ESC_PARAM_DATA`
- `MSP2_SET_ESC_PARAM_BEGIN`
- `MSP2_SET_ESC_PARAM_DATA`
- `MSP2_SET_ESC_PARAM_COMMIT`
- New dedicated module:
  - `src/main/msp/msp_esc.c`
  - `src/main/msp/msp_esc.h`
- `msp.c` now dispatches the new ESC MSPv2 commands through the new module.
- `msp.c` now dispatches new ESC MSPv2 set-commands through the same module on the input path.
- `esc_sensor.h` now exposes:
  - ESC write state / error enums
  - ESC info / capability structs
  - chunked parameter transfer struct
  - write-status getter
  - info / name getters
- `esc_sensor.c` now exposes chunk-safe parameter helpers:
  - read parameter data by offset / length
  - begin staged parameter write
  - write parameter chunk into the staging buffer
  - commit staged parameter write through the existing backend logic
- `esc_sensor.c` now tracks write status for:
  - 4way writes
  - existing async parameter commit paths
  - best-effort completion via payload recache
- A best-effort separate ESC name / model API now exists.
  - It now prefers protocol-backed payload parsing where current firmware data already contains usable identity fields.
  - Current payload-backed protocols:
    - HW5 / Platinum V5
    - Scorpion Tribunus
    - YGE / OpenYGE
    - FlyRotor
    - XDFly
    - OMP
    - ZTW
  - 4way AM32 and BLHeli_S still fall back to generic family-level names for now.
- A separate ESC details API now exists for `version` and `firmware` text.
  - It follows the same split the current Ethos `esc_tools` modules use, so future Lua clients can keep their existing header concepts.
  - Current payload-backed detail parsing covers:
    - HW5 / Platinum V5
    - Scorpion Tribunus
    - YGE / OpenYGE
    - FlyRotor
    - XDFly
    - OMP
    - ZTW
    - AM32
    - BLHeli_S / Bluejay-family 4way payloads
  - BLHeli_S / Bluejay naming is still generic at the name/model API level; the new details call is where firmware/layout text now comes from.
- New parameter transport stays below the payload target.
  - `GET_ESC_PARAM_DATA` replies are `4` bytes of metadata plus up to `64` bytes of data.
  - `SET_ESC_PARAM_DATA` accepts `3` bytes of header plus up to `64` bytes of data.
  - This keeps both directions comfortably below the `80` byte target.
- `GET_ESC_NAME` now uses the same payload layouts the Ethos suite / ESC tools already use for identity display where possible.
- `GET_ESC_DETAILS` uses the same `version` / `firmware` presentation split the Ethos suite currently expects, with small cleanups where the old tool formatting was obviously fragile.

## Validation Notes

- Windows-side targeted object builds succeeded for:
  - `msp_esc.o`
  - `esc_sensor.o`
- `msp.o` validation is currently blocked by an existing Windows/MSYS pthread header issue unrelated to this ESC change.
- WSL SITL builds are also hitting environment/toolchain issues unrelated to this slice.

## Current MSPv2 ESC Chunked Param API

Current command shape:

- `MSP2_GET_ESC_INFO`
  - Optional `escId` request byte.
  - Returns esc id, protocol, signature, flags, capabilities, visible parameter length, max chunk size.
- `MSP2_GET_ESC_NAME`
  - Optional `escId` request byte.
  - Returns esc id, generic flags, vendor/family name, model string.
  - When current firmware payloads contain real identity data, the model string is now parsed from those payloads instead of falling back to a static generic label.
- `MSP2_GET_ESC_DETAILS`
  - Optional `escId` request byte.
  - Returns esc id, generic flags, version string, firmware string.
  - Response shape is `escId`, `flags`, `versionLen`, `version...`, `firmwareLen`, `firmware...`.
  - With the current `31` byte per-string clamp, the full response still stays below the `80` byte target.
- `MSP2_GET_ESC_WRITE_STATUS`
  - No payload.
  - Returns op id, esc id, protocol, signature, state, error.
- `MSP2_GET_ESC_PARAM_DATA`
  - Request: `escId`, `offset`, `length`
  - Response: `escId`, `totalLength`, `offset`, `chunkLength`, `data...`
- `MSP2_SET_ESC_PARAM_BEGIN`
  - Request: `escId`
  - Copies the current parameter buffer into the staging buffer.
  - Rejects if a write is already pending.
- `MSP2_SET_ESC_PARAM_DATA`
  - Request: `escId`, `offset`, `length`, `data...`
  - Applies one chunk into the staged parameter buffer.
- `MSP2_SET_ESC_PARAM_COMMIT`
  - Request: `escId`
  - Commits the staged parameter buffer through the existing backend commit path.
  - Completion is observed through `MSP2_GET_ESC_WRITE_STATUS`.

## Constraints To Keep In Mind

- Prefer MSPv2 for the new API.
- Keep payloads below 80 bytes.
- Avoid putting identity blobs inside parameter payloads.
- Do not regress existing configurator behavior while the new path is being introduced.
- Avoid adding more ESC-specific complexity directly into `msp.c`.

## Suggested Next Steps

1. Add a compact `MSP2_GET_ESC_PARAM_META` or equivalent schema call so new Lua clients do not need to infer page structure from old vendor-specific blobs.
2. Decide whether `GET_ESC_NAME` should promote Bluejay from generic `BLHeli_S` family naming now that the payload-based detail split exists.
3. Add a normalized telemetry-only MSPv2 call so the new client stack can stop reading mixed programming / telemetry surfaces.
4. Start moving one Lua client path onto the new `INFO` / `NAME` / `DETAILS` / `PARAM_DATA` / `WRITE_STATUS` calls to validate the transport choices before wider migration.

## Notes

- This file is temporary and can be replaced by a proper design note later.
- If multiple agents touch this area, prefer extending the new API path instead of adding more behavior to the legacy ESC MSP commands.
