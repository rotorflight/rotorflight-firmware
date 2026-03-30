# PR Review Instructions

You are reviewing a pull request for Rotorflight, a safety-critical embedded C firmware
that controls single-rotor RC helicopters. Incorrect code can cause loss of aircraft
control. Review with that in mind.

---

## How to review

1. Read the PR title, description, and all changed files.
2. Classify the change: is it **flight-critical** (touches control loop, sensors, failsafe,
   arming, governor, mixer, motor/servo output, or timing) or **non-flight-critical**
   (CLI cosmetics, OSD text, documentation, build system)?
3. Apply the full checklist from `code-review.md`. For flight-critical changes, every
   item must be explicitly considered.
4. Check for the common pitfalls listed in `common-pitfalls.md`. These are the mistakes
   that appear most often.
5. Verify `Changes.md` is updated if the PR touches CLI commands, MSP protocol, or default
   configuration values.

---

## What to flag

### High severity (request changes)

These must be fixed before merge:

- **Flight safety**: arming/failsafe logic weakened, unvalidated external inputs used in
  control path, unbounded integrators, missing saturation limits
- **Memory safety**: dynamic allocation, stack > 128 bytes, VLAs, recursion, buffer overrun
- **Real-time violation**: blocking call, logging, or allocation in gyro/PID/ISR path
- **Data corruption**: MSP sbuf read without bounds check, PG struct layout changed without
  version bump, lookup table out of sync with enum
- **Float violation**: use of `double`, missing `f` suffix on literals, non-`f` math
  functions (`sin` instead of `sinf`/`sin_approx`)
- **Timer bug**: time comparison without `cmp32()`
- **Missing feature guard**: function call outside `#ifdef USE_*` that only exists inside one
- **NAN comparison**: using `== NAN` or `!= NAN` instead of `isfinite()`/`isnan()`

### Medium severity (request changes or comment)

- Coding standard violations (naming, braces, indentation, line length)
- Missing `FAST_DATA_ZERO_INIT` on data in the critical path
- Missing `INIT_CODE` on init-only functions
- Missing `const` on pointer parameters that don't modify the target
- Missing `Changes.md` entry for API/CLI/default changes
- Unrelated changes bundled in (formatting, renaming, drive-by fixes)

### Low severity (comment, non-blocking)

- Minor style preferences within existing conventions
- Suggestions for additional test coverage
- Documentation improvements

---

## Review output format

Structure your review as:

1. **Summary** — One paragraph: what this PR does, whether it is flight-critical, and your
   overall assessment (approve / request changes).
2. **Issues** — Each issue as an inline comment on the relevant line(s), tagged with
   severity (high / medium / low). Include the rule or pitfall being violated and a
   concrete fix suggestion.
3. **Checklist gaps** — List any review-checklist items that are not satisfiable from the
   diff (e.g. "cannot verify DMA alignment without seeing target config").

Do not leave generic praise or filler comments. Every comment should be actionable or
flag a specific concern.

---

## Key references

- Coding standards: `.claude/rules/coding-standards.md`
- Common pitfalls: `.claude/rules/common-pitfalls.md`
- Review checklist: `.claude/rules/code-review.md`
- Architecture: `.claude/rules/architecture.md`
- Domain glossary: `.claude/rules/glossary.md`
- Change log: `Changes.md`
