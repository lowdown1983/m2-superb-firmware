# Phase 10 — Gateway mode (CAN0 ↔ SWCAN) (2026-10-08)

A **new separate GATEWAY mode**, not a MITM extension
(docs/ROADMAP_PHASES_6_PLUS.md Appendix A): MITM is transparent
interception on one protocol (never originates); a gateway crosses
bitrates (500k vs 33.33k, ~15:1), controllers (SAM3X native vs
MCP2515/SPI), and frame semantics (GM-LAN), so every crossing needs ID
remapping, buffering with an explicit overflow policy, and its own
safety case. The MITM ordered rule engine is untouched — the gateway has
its own mapping-table data model.

Evidence labels: **VERIFIED** = compiles / cited from a primary source.
**INFERRED** = not tested on hardware (everything vehicle-facing; the
Bolt EV is CAN-only, so gateway forwarding has no test vehicle).

## 1. Mapping tables (`mode_gateway.h/.cpp`)

- `Mapping` = srcBus/srcId/srcMask/srcExt → dstBus/dstId/dstExt,
  action (PASS | MODIFY byte-replace), drop policy (global / drop-oldest
  / drop-newest), plus runtime hits/drops. 8 entries max. First match
  wins (MITM rule-engine convention).
- **Data-driven and versioned**: `tableVersion` (uint16_t) bumps on every
  add/del/clear/suspend; exposed in list replies, stats, and the web UI.
- Tables live in **RAM** (config, like MITM rules — they survive mode
  switches). EEPROM persistence is Phase 12 (vehicle profiles).
- ID matching reuses the shared `busfilter::idMatch()` primitive (one
  implementation). The mapping table itself is NOT routed through the
  shared `busfilter::accept()` — it is a different data model
  (translation, not filtering) per Appendix A.
- MODIFY semantics mirror MITM's byte-replace exactly:
  `data[off] = (data[off] & ~mask) | (val & mask)`; skipped when
  `off >= length` (frame still forwarded).

## 2. Rate adaptation — explicit overflow policy

- Per-direction queues: CAN0→SWCAN 16 entries, SWCAN→CAN0 8 entries
  (the ~15:1 mismatch lives almost entirely in the C→S direction).
- Overflow policy: **drop-oldest** (default) or **drop-newest**,
  global default + per-mapping override. Rationale for the default:
  gateway traffic is overwhelmingly sensor/state streams where the
  freshest frame is the most valuable; a request/response user can
  override to drop-newest per mapping.
- **Never silent**: every policy drop increments the mapping's `drops`,
  the direction's `dropCS/dropSC`, and `busstat::noteDropped()` on the
  destination bus; `dbg::` logs it.
- TX drain cap per `loop()` iteration (default 4, configurable 1–16)
  paces the slow side; refused TX (mailboxes full / bus down) drops +
  counts loudly instead of requeueing (no head-of-line blocking).
- SWCAN ID 0x100 goes out via the high-speed hop (`hsEnter`/`hsExit`,
  GMLAN quirk VERIFIED from M2RET.ino:1052-1067).
- TX queue contents are **dropped (counted) on mode exit** — a mode
  switch must quiesce fast, not flush stale frames onto the bus.
- Self-found during build: `swcan::sendFrame`/`getFrame` already feed
  `busstat` internally — the gateway does NOT count those paths itself
  (would have double-counted every SWCAN frame). Raw due_can paths
  (CAN0) are counted by the gateway (no internal feeds there).

## 3. Safety interlocks

- **PROG-entry auto-suspend**: `ModeManager::enterProg()` calls
  `gateway::suspend()` alongside `mitm::suspendRules()` — a translating
  gateway during a flash is the same brick vector (SAFETY_REVIEW §2).
  Suspend clears the table with an explicit notice and bumps the
  version.
- **Never silently restored**: `suspend()` sets a flag; `loop()`
  refuses to forward while it is set (defense in depth — the table is
  empty anyway). The flag clears only on explicit user action: any
  successful mapping add, or `gateway enable` on the console.
- **Write mappings need arm-confirm**: MODIFY mappings require
  `ALLOW_DIAGNOSTIC_WRITE` (escalate via `cap grant`; the mode default
  is `ALLOW_TRANSMIT`, ceiling `ALLOW_DIAGNOSTIC_WRITE`) **and** the
  repeat-identical-command-within-10 s arm-confirm — the same mechanism
  as UDS `confirmGate`. On the mm channel the first call stages
  (idx `0xFE`); the identical call within 10 s installs. The web UI
  renders an explicit Confirm page that re-posts the identical fields.
  PASS mappings need only `ALLOW_TRANSMIT` (the deliberate add is the
  intent signal).
- Full safety case: `docs/SAFETY_REVIEW.md` §12 (gateway originates
  frames by construction — MITM's "never originate" stealth property
  does NOT transfer), §10.2/§10.3 updated with the gateway row and
  enforcement point.

## 4. Supervision: mm commands 0x29–0x2D + console + web UI

Following the Phase 6/7/9 supervision template (F1 7E framing,
`CMD|0x80` + status replies, LE payloads; new `mm::Ops` fields
appended — the existing positional initializer is unaffected):

| CMD | Request | Reply |
|-----|---------|-------|
| 0x29 GW_MAP_ADD | 21B mapping payload | `[status, idx]` (`0xFE` = staged for arm-confirm) |
| 0x2A GW_MAP_DEL | `[idx]` (0xFF = clear all) | `[status, nMaps]` |
| 0x2B GW_MAP_LIST | `[start]` | `[status, nTotal, nRet, version LE16, entries...]` (22B/entry, 1/reply) |
| 0x2C GW_STATS | `[]` | `[status, fwdCS/SC, dropCS/SC, modCS/SC LE32, version LE16, nMaps, suspended, active]` (31B) |
| 0x2D GW_CFG | write: 10B `[canBaud LE32][swcanSpeed LE32][dropPolicy][drainPerLoop]` / read: `[]` | `[status, applied]` / `[status, 12B blob]` |

Console (SerialUSB): `gateway status | map list | map add <srcBus> <srcId> <srcMask> <srcExt> <dstBus> <dstId> <dstExt> <pass|modify> [modOff modMask modVal] [oldest|newest] | map del <idx|all> | enable | cfg [canbaud N] [swcanspeed N] [policy oldest|newest] [drain N]`.

**SuperB web UI**: new `/gateway` page (stats, mapping table with
delete/clear-all, add form with arm-confirm Confirm page for MODIFY,
config form, suspend notice with re-enable path), following the `/mitm`
page pattern. Routes + nav link added. (Note: the SuperB `mmc`
namespace previously stopped at 0x19 — only the gateway commands were
added; phases 6–9 mm commands remain M2-side only.)

## 5. Mode plumbing

- `modes::Mode::GATEWAY` (index 7); `modeName` → `"gateway"`;
  `modeImplemented` includes it (button cycling + `mode gateway` work).
- `enterGateway()` acquires CAN0 at the staged baud (promiscuous) +
  `swcan::acquire()` at the staged speed; warns (not fails) if SWCAN
  acquire fails — the CAN side still works, SWCAN legs count TX-fail.
- `exitGateway()` drops TX queues (counted), releases CAN0 +
  `swcan::release()` (idempotent). `quiesceBuses()` needed no change
  (native via begun flags, TH8056 via `sleepGpio`).
- `capgate`: DEFAULTS/CEILINGS extended to 8 entries; GATEWAY default
  `ALLOW_TRANSMIT`, ceiling `ALLOW_DIAGNOSTIC_WRITE`.
- `docs/OVERVIEW.md`: GATEWAY row in the mode table and the resource
  table (standing rule: new modes add a row).

## 6. Debug instrumentation

`debug.h` untouched. New `dbg::` events: mode begin/exit, mapping
add/del/clear, suspend/resume, arm-confirm staging, overflow drops,
TX refusals.

## INFERRED list (Phase 10 — no GM-LAN vehicle)

- All forwarding behavior (ID remap, MODIFY byte-replace on the wire).
- The 15:1 overflow policy under real burst load; drop-oldest vs
  drop-newest tuning; drain-per-loop pacing.
- The 0x100 high-speed hop timing on a live GM-LAN bus.
- SWCAN acquire/health behavior in the gateway context (shared with
  the M2RET path, but untested here).
- The Bolt EV cannot test any of this (CAN-only vehicle).

## Deliberately left out / deferred

- **Optional Phase 10b (LIN master stack)**: NOT approved, NOT built.
  CAN↔SWCAN only.
- **SAE J1939**: still parked (candidate for 9 or 12, no commitment).
- **EEPROM persistence** of mapping tables: Phase 12 (vehicle profiles).
- **OEM database seeding** of mapping tables: noted as a future seam
  (a Phase 12 profile could generate mappings from DBC address maps) —
  not built.
- **Loop suppression** for cyclic mappings: none in Phase 10. A→B and
  B→A mappings covering the same IDs can ping-pong; documented as user
  error, not silently fixed.
- **0x27**: still a stub, untouched.
- **ChatGPT F4** (F1 7E framing): still deferred, untouched.
- **Bringup/SPI**: `src/m2/main.cpp` untouched (settled).

## Resource ownership (Phase 10)

- **New claims**: CAN0 (begun at entry, staged baud) + SWCAN via
  `swcan::` (SPI0 sole claimant per INV-SPI0, PIOC16 slot, SWC_M0/M1
  GPIOs — all owned between `swcan::acquire`/`release`, same as the
  M2RET path). No new TC channels, PWM, PIO interrupts, DAC, or power
  rails. `quiesceBuses()` needed no structural change.
- Pin-safety invariant holds: every native-CAN `begin()` still passes
  `(baud, 255)` — verified by grep; the SWCAN path uses `swcan::`
  (drives only 63/64/65/67, never 50/48 — verified Phase 8).
- `docs/SAFETY_REVIEW.md` **updated** (§12 + §10.2/§10.3): the gateway
  originates frames by construction — new transmit authority, new
  safety case.

## Adversarial review fixes (Marcus standing practice, Phase 8+)

A separate reviewer subagent audited the Phase 10 build against the
phase requirements and safety invariants. Verdict: **no BLOCKERs**.
2 MAJORs and 5 MINORs found and fixed before sign-off:

- **M1 (major):** `dropQueues()` reset the tails to 0 but left stale
  heads — a latent queue-index desync that would have transmitted a
  stale entry after any refactor touching `begin()`'s reset. Fixed:
  full head+tail+count resets.
- **M2 (major):** the 0x100 high-speed hop did `hsEnter`/`hsExit` per
  frame (~60 ms of blocking `delay()` per frame), starving CAN0 RX and
  `swcan::poll()` during bursts. Fixed: the drain batches — non-HS
  frames go first, then all HS frames under a single `hsEnter`/`hsExit`
  per `loop()` iteration (bounded, documented reorder of ≤ drainPerLoop).
- **m1:** the raw mm path bypassed the console/web 29-bit ID clamp —
  `payloadToMapping()` now rejects IDs/masks > 0x1FFFFFFF itself.
- **m2:** a MODIFY add on a full table staged an arm-confirm before
  reporting full — capacity is now checked before staging (fail fast).
- **m3:** `CMD_GW_STATS` reply-size comment corrected (30 bytes:
  status + 29-byte blob).
- **m4:** OVERVIEW.md heading "seven modes" → "eight modes".
- **m5:** §12 gate 2 now states the MODIFY grant is evaluated in the
  ambient mode (config works from any mode by design; the arm-confirm
  carries the intent friction there).
- Self-found during build (before review): `swcan::sendFrame`/`getFrame`
  already feed `busstat` internally — the gateway does not double-count
  the SWCAN paths; plus 11-bit ID range validation on mapping add.

## Sim harness note

`pio run -e sim_m2` still fails on `SPI.h` — **pre-existing** (boot fix,
not Phase 10). `mode_gateway.cpp` uses only `Arduino.h`, `due_can.h`
(CAN_FRAME), and module headers; the CAN0 register paths go through
due_can like the other modes. `sim_superb` still builds.

## Flash impact

m2 env: 131,948 → **140,640 bytes** (25.2% → 26.8% of 512KB). The web
UI lives on the SuperB side (no m2 flash cost). Comfortable headroom
remains.

## Files changed

New: `src/m2/mode_gateway.h/.cpp`, `docs/PHASE10_NOTES.md`.
Modified: `src/m2/mode_manager.h/.cpp` (Mode::GATEWAY, enter/exit,
requestMode, loop, console route, enterProg suspend, 6 mm callbacks +
ops registration), `src/m2/mm_channel.h/.cpp` (CMD 0x29–0x2D + Ops +
dispatch), `src/m2/capgate.cpp` (8-entry grant tables),
`src/superb/main.cpp` (`/gateway` page + 4 POST handlers + mmc
constants + routes + nav), `docs/SAFETY_REVIEW.md` (§12, §10.2, §10.3),
`docs/OVERVIEW.md` (mode + resource rows).
Untouched: `debug.h`, `main.cpp`, 0x27 stub, `mode_mitm.cpp` (rule
engine), sim harness.
