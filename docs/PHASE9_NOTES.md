# Phase 9 — Diagnostic layer depth (2026-10-08)

Four work items: OBD-II depth (data-driven PID registry + service modes +
discovery engine), UDS depth (DID/RID registries, 0x23/0x24/0x2F/0x31,
security-level tracking, shared 0x34/0x36/0x37 engine), ELM depth
(ATSH apply, ATCRA/ATCF/ATCM, parity-list gaps), and formalized
capability gates. Plus the OEM database's first firmware use: a
build-time extraction pipeline.

Evidence labels: **VERIFIED** = compiles / cited from a primary source.
**INFERRED** = not tested on hardware (everything vehicle-facing; the
Bolt EV is CAN-only, so K-line/J1850/SWCAN paths have no test vehicle).

## 1. Capability gates (`capgate.h/.cpp`) — the safety mechanism

Formalized transmit authority as hierarchical grants, enforced in code:

- `READ_ONLY` (0, default) < `ALLOW_TRANSMIT` (1) <
  `ALLOW_DIAGNOSTIC_WRITE` (2) < `ALLOW_ACTUATOR_CONTROL` (3) <
  `ALLOW_FLASH_PROGRAMMING` (4).
- Per-mode defaults (documented in `docs/SAFETY_REVIEW.md` §10):
  M2RET=TRANSMIT, ELM/STN=DIAGNOSTIC_WRITE, UDS=TRANSMIT, MITM=READ_ONLY,
  OBD2=TRANSMIT, PROG=FLASH_PROGRAMMING. MITM has a ceiling of TRANSMIT.
- `capgate::enterMode()` runs on **every** mode entry (manager
  `requestMode` + `begin`); escalation never survives a mode switch.
- `capgate::require()` refuses with a message naming the missing grant;
  `capgate::escalate()` is upward-only, prints a console warning, never
  silent. No downgrade API — switch modes to shed a grant.
- Enforcement points: UDS `sendRaw`/`mmTx` (service→grant map:
  0x11/0x14/0x28/0x2E/0x85/0x86→WRITE, 0x2F→ACTUATOR, 0x31
  startRoutine/stopRoutine→WRITE (requestRoutineResults→TRANSMIT),
  0x34/0x36/0x37→FLASH, else TRANSMIT); ELM/STN `obdRequest` gates OBD
  04/08 (defense-in-depth: ELM/STN already default to WRITE); OBD2 mode
  gates `clear`/`ctl`; MITM `A_MODIFY` rule adds (console + mm) need
  TRANSMIT. Arm-confirm on 0x11/0x2E stays as defense-in-depth *after*
  the gate.
- `cap show | cap grant <0..4>` console; `CMD_CAP_GRANT` (0x28) on mm
  (`[level]`, `[0xFF]`=query).
- Deliberate asymmetry: ELM defaults to DIAGNOSTIC_WRITE (user-driven
  terminal; Torque clears DTCs) while UDS defaults to TRANSMIT (raw
  service access is broader than ELM's OBD subset).

## 2. OBD-II depth

### PID registry (`pidreg.h/.cpp`) — replaces the `decodePid()` switch
- ~50 data-driven entries: service 01 scalars + support masks
  (0100/0120/0140/0160/0180/01A0/01C0), service 06 monitors, service 09
  infos (incl. 0902 VIN ASCII). Service 02 falls back to 01 formulas.
- Decoder kinds: LINEAR (`scaled = raw*num*10^dec/den + off`, integer
  math only — no float in firmware), BITMASK, ASCII, RAW.
- `mode_obd2.cpp`: the `decodePid()` switch is **deleted**; all decoding
  goes through `pidreg::find`/`pidref::decode`, with raw-hex fallback
  for unknown (svc,PID).
- New console: `obd2 dtcpend` (07), `obd2 dtcperm` (0A), `obd2 clear`
  (04, gated), `obd2 vin` / `obd2 info <pid>` (09), `obd2 ff <pid> [frame]`
  (02), `obd2 ctl <tid>` (08, gated), `obd2 pidinfo <svc> <pid>`,
  `obd2 discover`.
- The poll list is now (service,PID) pairs: `obd2 pids 010C,010D,0902`.
  CSV format is `ts_ms,svc,pid,value` (was `ts_ms,01,pid,value`).
- mm: poll-list config format unchanged (service 01 assumed) — pairs are
  a console-only feature for now.

### Discovery engine (`discover.h/.cpp`)
- CAN-only (11/29-bit): baud/format detect (500k/250k/125k × 11/29),
  functional 0100 ECU find, per-ECU PID support masks (0100–01E0),
  service probes 01/02/03/06/07/09/0A — **deliberately never 04/08**.
- VIN via 0902; `applyFilters()` narrows the Phase 6 filter table to
  discovered ECU response IDs (addr slot = nodeAddr — the OEM seam).
- Requires `ALLOW_TRANSMIT`; acquire-on-entry/release-on-exit; callers
  stop their own bus activity first.
- Exposed via `discover start|status|list|apply|cancel` console and mm
  `CMD_DISCOVER_START` (0x25) / `STATUS` (0x26) / `LIST` (0x27 — mm
  namespace only, unrelated to UDS 0x27).

## 3. UDS depth

- **DID registry** (`udsreg`): 33 ISO DIDs with names + security flags
  from the OEM DB; **RID registry**: 5 ISO-example RIDs. `uds didinfo` /
  `uds ridinfo` console lookups. 0x22 responses get registry-assisted
  output (name + decoded value + `[may need 0x27]` flag); 0x31 positives
  show routine names.
- **0x23/0x24/0x2F**: work through the existing raw `sendRaw` path with
  the new gate checks (0x2F needs ACTUATOR_CONTROL). No separate
  sub-protocol handlers were needed.
- **0x31 routine control**: in UDS mode (was already reachable via raw);
  gate maps startRoutine/stopRoutine→DIAGNOSTIC_WRITE,
  requestResults→TRANSMIT.
- **Security-level state tracking**: `secLevel` records the highest
  unlocked 0x27 level from 0x67 positives; reset on session change,
  session timeout, and mode entry/exit. **0x27 remains a stub — no key
  algorithm, ever** (standing policy; the stub notice is intact).
- **Transfer/download shared with PROG**: `xfer.h/.cpp` holds the
  0x34/0x36/0x37 engine (sequence checks, CRC16 per chunk, 0x76 echo
  verify, CRC32 end-to-end, 30s watchdog). PROG was refactored onto it —
  not duplicated — keeping its arm/powerOk/configured interlocks intact.
  UDS mode gets `uds dl <addr> <len> [alfid]` (gated FLASH_PROGRAMMING +
  arm-confirm), `uds ul <hex...> [last]` (0x36 blocks; `last` ends the
  stream), `uds xferdone <fileCrc32Hex>` (CRC32 enforced in code via
  `xfer::finish()` — the review caught a warn-only path; fixed).

## 4. ELM depth

- **ATSH now applies for CAN** (was accepted-but-not-applied): 11-bit
  protos take up to 3 hex digits as the TX arbitration ID; 29-bit take
  up to 8. J1850/K-line keep 3-byte headers (K-line routed per protocol).
- **ATCRA/ATCF/ATCM**: CAN RX filter commands, applied in software via
  the shared `busfilter::idMatch` on the OBD-reply and ATMA paths.
- Parity-list gaps closed (all v1.5-era): ATFI (fast init),
  ATKW0/1 (stored), ATIIA (init address — now feeds K-line target addr),
  ATIB10/48/96 (stored; driver runs 10400 — documented limitation),
  ATSW (stored), ATWM (stored), ATRD (single-frame read), ATAR0/1
  (stored). OBD 04/08 gated on ALLOW_DIAGNOSTIC_WRITE.
- STN inherits everything (shared ELM handler).

## 5. OEM extraction pipeline (`scripts/oem_extract.py`)

First real firmware use of the OEM database — build-time only:

- Reads `generated/json/diagnostics.json` + `generated/json/dids.json`
  and emits `src/m2/oem_gen.h`: per-make physical request/response ID
  patterns (regex-parsed from service prose like `$7E0-$7E7 req /
  $7E8-$7EF resp`) plus a thin make-specific DID pack.
- **Subset policy**: generic SAE PID/DID tables compiled in by default
  (in `pidreg`/`udsreg`); make packs baked per build flag
  (`--make gm` tested → 7 DIDs). The 6.5MB `vehicles.json` is never
  embedded; a "full" make pack = one generated header per make with its
  DID catalog + addressing patterns, selected at build time.
- Discovery consumes the patterns via the `oem::` namespace seam
  (generic row = default).
- DBC/custom-PID engine stays Phase 12; SAE J1939 stays parked (no
  commitment, per decision #5).

## Flash impact

m2 env: 113,988 → **131,948 bytes** (21.7% → 25.2% of 512KB).
Comfortable headroom remains for Phase 12 make packs.

## INFERRED list (Phase 9)

- All discovery behavior (no vehicle tested yet; Bolt EV is CAN-only —
  CAN discovery CAN be tested there eventually).
- Registry decode values against real ECUs.
- ATSH-apply and ATCRA filter behavior on a live bus.
- K-line ATIIA/ATFI against a K-line vehicle.
- xfer engine against a real ECU (logic preserved from PROG's VERIFIED
  compile, but the refactor is new code).

## Deliberately left out / deferred

- SAE J1939 (still parked, no commitment).
- ChatGPT F4 F1-7E framing (architectural, deferred).
- SuperB web GUI pages for discovery/grants (mm commands exist;
  GUI is follow-up work).
- mm poll-list (service,PID) pairs (console-only for now).
- ATIB actual baud switching (stored; driver is 10400-fixed).
- Full make-pack headers (generator ready; no make selected for default).

## Adversarial review fixes (Marcus standing practice, Phase 8+)

A separate reviewer subagent audited the build against the phase
requirements and safety invariants. Verdict: no BLOCKERs. 3 MAJOR
clusters found and fixed before sign-off:

- **M1** — `confirmGate` out-of-bounds reads: `armedData` was 8B but the
  compare loop ran to `len` (up to 11), and the new `uds dl` arm-confirm
  staged into a 9B `cgData` for a 10B payload — UB plus a confirm that
  could never match (fail-closed, but a dead headline feature). Fixed:
  `armedData` sized to 12, `len` clamped in `confirmGate`, `uds dl`
  passes the request payload directly.
- **M2** — `grantFor()` write-as-read: 0x28/0x85/0x86 fell through to
  ALLOW_TRANSMIT (all state-changing), and the 0x31 subfunction split
  had 0x02/0x03 swapped (stopRoutine as TRANSMIT, requestRoutineResults
  as WRITE). Fixed: 0x28/0x85/0x86→DIAGNOSTIC_WRITE; 0x01/0x02→WRITE,
  0x03→TRANSMIT. SAFETY_REVIEW §10.1 table corrected (it had listed 0x85
  under "doesn't change ECU state").
- **M3** — `uds xferdone` finalized with only a printed warning instead
  of the CRC32 interlock. Fixed: `xferdone` now requires
  `<fileCrc32Hex>` and routes through `xfer::finish()` (CRC compare +
  streamDone + 0x37 enforced in code); `uds ul ... last` marks
  end-of-stream on the console path.
- 9 MINORs fixed: ELM 04/08 gate labeled defense-in-depth (vacuous at
  current defaults), mm-path warning visibility note, `secLevel` reset
  on session change, notes doc errors (33 DIDs not 36; correct script
  inputs), `maxBlockLen` now enforced fail-closed in `xfer::feedChunk`,
  `minV`/`maxV` documented as informational.

Invariants re-verified holding: pin-safety `(baud,255)`, acquire/release
+ quiesce, 0x27 stub (no key algorithm), PROG interlocks through the
xfer refactor, no J1939 / no F4 framing, debug.h + main.cpp untouched,
discovery never probes 04/08, mm 0x27 vs UDS 0x27 namespace separation.

## Sim harness note

`pio run -e sim_m2` still fails pre-existing on `SPI.h` (unchanged).
New shared modules are plain C++ and remain syntax-checkable.

## Files changed

New: `src/m2/capgate.h/.cpp`, `pidreg.h/.cpp`, `udsreg.h/.cpp`,
`xfer.h/.cpp`, `discover.h/.cpp`, `scripts/oem_extract.py`,
`src/m2/oem_gen.h` (generated), `docs/PHASE9_NOTES.md`.
Modified: `mode_obd2.cpp/.h`, `mode_uds.cpp/.h`, `mode_prog.cpp`,
`mode_elm.cpp`, `mode_mitm.cpp`, `mode_manager.cpp`, `mm_channel.h/.cpp`,
`docs/SAFETY_REVIEW.md` (§10 added, dual-use → §11).
Untouched: `debug.h`, `main.cpp` (settled), 0x27 stub, sim harness.
