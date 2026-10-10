# Changelog — M2/SuperB Firmware

All notable changes to the M2/SuperB seven-mode firmware. Dates are the dates
work landed, per the phase notes and audit reports in `docs/` and
`audit-2026-10-08/`. Evidence labels used throughout the source notes:
**VERIFIED** = compiles and/or sourced from a primary source;
**INFERRED** = hardware behavior assumed but untested (no bench until
2026-10-08; Bolt EV in-vehicle testing from 2026-10-08 is CAN-only).

## [Unreleased]

### Fixed (2026-10-10 — 2026-10-09 firmware audit findings F-02/F-03 + hardening
fixes, applied on Marcus's approval)
- F-02 (Major): DBC physical-value scaling was wrong by a factor of 1000 —
  `dbcsig::decode()` divided milli-units back down to plain physical units,
  destroying fractional precision. `phys1000` is now true milli-units per
  the `SignalVal` contract, with int32 saturation on narrowing
  (`src/m2/dbcsig.cpp`).
- F-03 (Moderate): five CAN IDs occur in more than one embedded DBC and
  `decode()` silently took the first table entry, decoding shadowed
  vehicles' frames under the wrong layout. `scripts/dbc_extract.py` now
  emits a per-message DBC index (`dbc_gen.h` regenerated with the new
  `Msg.dbc` field); `dbcsig::selectDbc(idx)` / `selectedDbc()` scope ID
  lookup to one DBC (`-1` = legacy first-match default); new console
  command `dbc select <idx|name|all>` (`src/m2/mode_manager.cpp`).
- O-1: the ISO-TP reassembly table (`Reasm r[MAX_REASM]`, ~1 KB) lived on
  the stack in `isotp::requestMulti()` — moved to function-static storage
  with an explicit reset each call (single-threaded by firmware contract:
  console paths only) (`src/m2/isotp.cpp`).
- W-2: the K-line tester-present keep-alive ran through `sendOnly()`'s
  200 ms transceiver-echo wait from a `txsched` callback, stalling the main
  loop every period. New `testerPresentFire()` is fire-and-forget
  (0x3E is response-suppressed by definition); echo verification stays in
  `sendOnly()`/`transact()` for real traffic (`src/m2/kline.cpp`).
- W-4: J1850 PWM arbitration now concedes on a stuck bus — if the bus stays
  active past `onUs + tolerance` with no release, the frame reports
  ARB_LOST instead of spinning on the sanity bound (`src/m2/j1850.cpp`).
- W-5: SWCAN `poll()` now clears stale MCP2515 error flags and aborts
  queued TX (CANCTRL ABAT bit, set directly — the vendored MCP2515 lib
  exposes no Abort API) before re-requesting NORMAL, so a bus-off caused
  by a flooding `txsched` slot can't resume transmitting the stale queue
  on recovery (`src/m2/swcan.cpp`).
- C-1: J1850 TX no longer masks interrupts for the whole frame (~34 ms
  blackout that starved CAN RX ISRs and the Bee UART ring). Each
  drive+spin pair is now masked individually (`vpwDriveSpin`/
  `pwmDriveSpin`); arbitration polls run unmasked. Worst-case continuous
  blackout is one driven section (VPW SOF 200 us) (`src/m2/j1850.cpp`).
- Grok audit: `Can0/Can1.begin()` return values are now honored — M2RET
  `applyCanBusConfig()` and `mmSetCanBaud()` only set the begun/applied
  flags on success; a failed begin leaves the bus disabled instead of
  claiming it live (`src/m2/mode_m2ret.cpp`, `src/m2/mode_manager.cpp`).
- Defensive: `mm_channel` logPoll reply clamps `nRec` to the documented
  <=1 contract before framing the reply (`src/m2/mm_channel.cpp`).
- Still open (unchanged): F-07 log-access policy (`/logs`, `/log_dl` have
  no role check) and the raw `0x34/0x36/0x37` console path — both are
  Marcus's policy calls, not decided.

### Added (2026-10-10)
- Bolt EV test-support package (`docs/bolt-ev/`): `BOLT_EV_TEST.md`
  per-mode test plan for the 2023 Chevy Bolt EV (PROG excluded; safety
  boundaries: no contactor commands, no TX while driving),
  `ATTRIBUTION.md`, and vendored `bolt17.json`/`bolt19.json` PID data
  from iternio/ev-obd-pids (Apache 2.0).

### Fixed (2026-10-09 — ChatGPT audit Section 03, applied on Marcus's approval)
- F-03-01 (HIGH): the UDS/PROG transfer engine now enforces the declared
  image length end to end — `xfer::State.declaredLen`, `open()` takes the
  length and rejects zero, `feedChunk()` rejects overrun chunks mid-stream,
  `finish()` requires `bytesRx == declaredLen`; both 0x34 callers updated.
- F-03-02 (HIGH): the SuperB web UDS sender (`udsSendRaw()`) transmitted
  `1+nParams` bytes for a `2+nParams` buffer, so the M2 rejected every
  request and the whole web UDS path was dead; corrected to `2+nParams`.
- F-03-03 (HIGH): `uds::mmTx` copied an unbounded ISO-TP response into
  `rdata[64]` (stack overwrite when the ISO-TP cap exceeded 64). `respCap`
  is now carried through the `udsTx` op signature (`mm_channel.h/.cpp`,
  `mode_manager.cpp`, `mode_uds.h/.cpp`) with a clamp before the copy —
  same pattern as the `mmcTransact` `outCap` fix.
- New invariants proven with an 8-case host harness against the real
  `xfer.cpp`: zero-open rejected, overrun → abort, short finish refused,
  exact-length stream + valid CRC accepted. All envs build clean, zero new
  warnings.
- Open decision (unchanged): the raw console path can send UDS programming
  services `0x34/0x36/0x37` directly with `ALLOW_FLASH_PROGRAMMING`,
  bypassing the managed transfer engine. Recommendation is to refuse and
  force the managed path (fail-closed); a legitimate power-user case for
  keeping raw access was noted. Marcus's call — not decided.

## [2026-10-09] — Post-roadmap audit triage (ChatGPT audits)

ChatGPT's firmware audit reports (in `~/workspace/user/files/`) triaged
against the code and the original opendbc DBCs. All fixes build-verified
(all envs clean, zero new warnings); consolidated zip refreshed.

### Fixed
- Bee-link S2: `pumpBeeRx()` now stops one byte short of `PIPE_CHUNK` so the
  diverter's 2-byte `F1 7E` replay always fits the pipe.
- Bee-link S3: `handleReply()` now requires the exact reply opcode
  (`cmd|0x80`) instead of accepting any reply.
- Bee-link S4: `mm::encode()` total length corrected to `5+payloadLen`
  (was undercounting by one; latent — zero production callers).
- UDS-XFER-001 (MAJOR, order-dependent): the shared transfer engine's CRC32
  table was initialized only by PROG `begin()`, so a UDS download on a fresh
  boot (PROG never entered) verified against a zero-filled table. Fixed
  structurally: `xfer::crc32feed()` now lazily calls the idempotent
  `crcInit()`; verified with the IEEE known-answer vector (`0xCBF43926`)
  against the real `xfer.cpp` with no explicit init.
- DBC extractor byte-order inversion (MAJOR, build tooling): ChatGPT's
  "130 out-of-DLC signals" diagnosis was wrong (@0 is Motorola per the DBC
  standard and cantools 44.x — 0 of 17,771 signals truly overflow), but it
  exposed a real worse bug: our `dbc_extract.py` had the same inversion,
  flipping every byte-order flag in `dbc_gen.h`. Extractor mapping corrected
  and the signal table regenerated (inventory byte-identical, only the flag
  bit flipped).

### Changed
- Bee-link S1 (unescaped `F1 7E` corrupting binary streams) left alone: it is
  the deferred F4 framing issue, architectural by Marcus's decision. Noted a
  possible future direction (gate the RX diverter on `pending.active` IF the
  M2 never sends unsolicited control frames — premise unverified).

## [Phase 12] — 2026-10-08 — Profiles, auto-detect, logging, test harness

Last phase of the approved roadmap (`docs/ROADMAP_PHASES_6_PLUS.md`).
Build-verified (pio clean, zero new warnings). Everything vehicle-facing is
INFERRED except where HARNESS-VALIDATED (new label: exercised against the
loopback harness on real silicon).

### Added
- Vehicle profiles (`flashstore.*`, `profiles.*`): versioned, CRC32-protected
  records in the last two IFLASH1 pages (A/B slots, `0x000FFC00`/`0x000FFE00`),
  save-to-inactive-slot with read-back verify, monotonic save counter,
  explicit saves only. Round-trips the Phase 6/9/10 knobs
  (CAN0/CAN1 baud, SWCAN speed, ISO-TP blob, busfilter entries, gateway
  mappings, capgate ceiling, log mode, retry limit). Console
  `profile save|load|show|clear`; mm `CMD_PROFILE_SAVE/LOAD/INFO`
  (0x31–0x33); web UI on `/phase12`.
- Multi-protocol auto-detect engine: explicitly user-initiated, layered
  transmit gates; console `harness selfcheck | run | status`;
  mm `CMD_HARNESS_RUN` (0x36) / `CMD_HARNESS_STATUS` (0x37).
- Diagnostic session logging: M2 produces → Bee → SuperB persists to SPIFFS.
- Virtual-ECU self-test harness on the CAN0↔CAN1 loopback pair — the first
  real validation path for the INFERRED CAN behaviors.
- DBC/custom-PID signal engine fed by build-time extraction from the OEM
  database's 45 curated DBCs (`oem_extract.py`, `dbc_extract.py`,
  `dbc_gen.h`).
- SLCAN/Lawicel ASCII transport + M2RET modernization commands.

### Changed
- Sim builds stub the EFC out (`#ifdef SIM_M2`): profiles stay RAM-only on
  the host.

## [Phase 11] — 2026-10-08 — SuperB as communications appliance

Build-verified. Adversarial review: 7 findings fixed. M2's only change is
one new mm query (`CMD_GET_VOLTAGE = 0x2E`) for the sleep manager.

### Added
- WebSocket transport (`ws::`, port 81; port 80 stays the web GUI):
  binary frames = the raw byte stream (same bytes as TCP:23),
  text frames = tiny JSON control (`status`/`claim`/`release`/`auth`).
- Multi-client ownership policy: one control owner across TCP:23, BT SPP,
  and WS binary; everyone else is a passive monitor whose TX is dropped
  and counted (never silently). First-TX-wins claim, no silent preemption.
- Web authentication with roles (viewer/operator/admin) mapped to the Phase 9
  capability gates; physical pairing authorization (3 s Button2 hold +
  120 s window); salted SHA-256 passwords; 64-bit session tokens; device
  identity `M2SB-XXXXXX` from eFuse MAC.
- Signed OTA (P-256 ECDSA, verify-then-commit, anti-rollback via NVS).
  Honest limitation: no TLS — the WPA2 passphrase is part of the trust
  boundary. BLE GATT deferred.
- Sleep/wake manager: voltage via M2 `CMD_GET_VOLTAGE`, engine-off +
  inactivity → light sleep, wake on timer/GPIO0/UART0, modem doze.

## [Phase 10] — 2026-10-08 — Gateway mode (CAN0 ↔ SWCAN)

Build-verified. A new separate GATEWAY mode, not a MITM extension: crossing
bitrates (~15:1), controllers, and GM-LAN frame semantics needs ID remapping
and an explicit overflow policy. The MITM rule engine is untouched.

### Added
- Mapping tables (`mode_gateway.h/.cpp`): srcBus/srcId/srcMask → dstBus/dstId,
  PASS | MODIFY byte-replace actions, 8 entries max, first-match-wins,
  versioned (`tableVersion` bumps on every mutation), RAM-resident.
- Rate adaptation: per-direction queues (CAN0→SWCAN 16, SWCAN→CAN0 8),
  overflow policy drop-oldest (default) / drop-newest, never-silent drop
  accounting, TX drain cap per loop iteration, SWCAN 0x100 via the
  high-speed hop (GMLAN quirk VERIFIED from M2RET.ino:1052–1067).
- TX queues dropped (counted) on mode exit — a mode switch must quiesce
  fast, not flush stale frames.

## [Phase 9] — 2026-10-08 — Diagnostic layer depth

Build-verified. The OEM database's first firmware use: a build-time
extraction pipeline.

### Added
- Capability gates (`capgate.h/.cpp`): hierarchical transmit authority
  `READ_ONLY < ALLOW_TRANSMIT < ALLOW_DIAGNOSTIC_WRITE <
  ALLOW_ACTUATOR_CONTROL < ALLOW_FLASH_PROGRAMMING`; per-mode defaults
  (MITM ceiling = TRANSMIT); upward-only escalation, never silent, never
  surviving a mode switch. Documented in SAFETY_REVIEW §10.
- OBD-II PID registry (`pidreg.h/.cpp`): ~50 data-driven entries (service
  01 scalars + support masks, service 06 monitors, service 09 infos incl.
  0902 VIN ASCII); integer-math-only decoders (LINEAR/BITMASK/ASCII/RAW).
- UDS depth: DID/RID registries, services 0x23/0x24/0x2F/0x31, security-level
  tracking, shared 0x34/0x36/0x37 transfer engine.
- Discovery engine: `discover start|status|list|apply|cancel` console;
  mm `CMD_DISCOVER_START` (0x25) / `STATUS` (0x26) / `LIST` (0x27).
- Build-time OEM extraction (`oem_extract.py`): PID/DID tables generated
  from the OEM database at build time.

### Changed
- ELM `ATSH` now applies; ATCRA/ATCF/ATCM parity gaps closed.

## [Phase 8] — 2026-10-08 — J1850 + SWCAN production hardening

Build-verified (both envs green, zero new warnings; sim-shim syntax check).
No J1850/SWCAN vehicle available — INFERRED until tested (Bolt EV is
CAN-only).

### Added
- J1850 hardening (`j1850.h/.cpp` reworked): explicit RX/TX state machines,
  PWM arbitration, 1- and 3-byte header decode, formal idle detection,
  error counters, optional CRC gate on the monitor path, Phase 6 transport
  wiring. Phase 7's `rail9141` integration kept verbatim.
- SWCAN hardening (new `swcan.h/.cpp`): TH8056 state machine (sleep /
  normal / high-speed / high-voltage wake-up) with datasheet transition
  timings, wake detection, listen-only, MCP2515 health monitoring +
  recovery, bus-off detection, hardware filter support, drain-time
  timestamps. Wraps the vendored Macchina SWcan lib; `mode_m2ret.cpp` no
  longer touches the MCP2515 directly.
- Timer/power allocation map: `RESOURCE_OWNERSHIP.md` gains the explicit
  TC6/TC7/PWM6/SPI0/PIOC16/pin-49 allocation.

### Fixed
- M2RET upstream deviations (`mode_m2ret.cpp`): incoming checksum now
  validated (was calculated-but-not-checked); SWCAN control unified through
  the hardened driver (listen-only actually applied, mode-before-setup
  ordering fixed, ISR detached on disable); echo honors the bus bit;
  bus index validated; LIN words handled truthfully (no LIN driver —
  non-zero LIN words reported, not silently "accepted"); `SET_SW_MODE`
  consumes its byte and maps it onto the TH8056 (was: byte fell through
  into the command parser).

## [Phase 7] — 2026-10-08 — K-line / ISO 9141 / KWP2000 stack

Build-verified. No K-line vehicle available — INFERRED until tested
(Bolt EV is CAN-only).

### Added
- K-line driver (`kline.h/.cpp`): 5-baud init, fast init, K/L line
  sequencing, inter-byte timing, checksum, framing, echo handling,
  wake/sleep, ISO 9141 request/response, KWP2000 init + timing + block
  transfers, tester/ECU addressing. Hardware: TJA1027 (U7/U8), USART0/1
  (pins 55–60), OBD-II pin 7 — pin truth from the knowledge pack only.
- Shared rail owner (`rail9141.h/.cpp`): the `J1850 9141 ON` net (pin 49)
  refcounted between the J1850 and K-line drivers; also fixes the Phase 2
  pin-50-before-49 ordering violation.
- ELM integration: ATSP 3/4/5 (+ATTP/ATDP/ATDPN/ATSH/ATMA), OBD-II request
  dispatch over K-line; Phase 6 transport wiring (busfilter KLINE enums,
  busstat K-line bus, tester-present via txsched).
- SAFETY_REVIEW.md §9 (K-line init transmit authority: three-part gate —
  ELM/STN active + user-initiated + no J1850 session).

## [Phase 6] — 2026-10-08 — Transport layer completion

Build-verified (both envs green, zero new warnings; new modules also pass
the host-sim shim syntax check). One shared core in `src/m2/`, modes
consume it — no parallel stacks.

### Added
- ISO-TP configurability (`isotp.h/.cpp`): `isotp::Config` — STmin, block
  size, P2/P2* timers, N_Bs/N_Cr timeouts, extended + mixed addressing,
  manual vs automatic flow-control generation, cancellation API, payloads
  up to 256 bytes. Byte-compatible with existing callers.
- Generic TX scheduler (`txsched.h/.cpp`): ID, bus, payload, period,
  offset, count, start/stop, one-shot, fire callback. The OBD2 poller now
  runs on it.
- Unified bus filter model (`busfilter.h/.cpp`): bus → protocol →
  address → ID → mask → direction → payload mask; one table, one
  `accept()`, consumed by M2RET, ELM/STN, UDS, OBD2, MITM.
- Error/status reporting (`busstat.h/.cpp`): bus-off notification, RX
  overflow + dropped-frame counters, TX fail counters, per-bus tx/rx
  frames+bytes.

## [2026-10-08] — Bring-up hardening (between phases)

Build-verified (`pio run -e m2 -e superb -e sim_m2 -e sim_superb` all
SUCCESS; `pytest sim/tests/smoke.py` 5/5 pass). Marcus-approved.

### Fixed
- Boot-hang saga RESOLVED: root cause was a missing `SPI.begin()` —
  `MCP2515 SPI.transfer()` blocked forever inside
  `swcanEnter()->setupSW()` on the uninitialized peripheral. Fix:
  `SPI.begin()` in `src/m2/main.cpp` `setup()` before
  `modeManager.begin()`. (Earlier confusion explained: SAM CDC drops all
  USB prints unless the host asserts DTR, and the boot banner raced the
  monitor reconnect.)
- ChatGPT audit F1: `sendReply` buffer overflow — guarded.
- ChatGPT audit F2: `progSendRaw` — guarded.
- ChatGPT audit F3: PROG powerOk interlock added.
- ChatGPT audit F5: ISO-TP BS/STmin handling fixed.
- ChatGPT audit F6: MITM uninitialized variables fixed.
- Fresh-eyes audit (88 files / ~25k lines, three findings-only audits:
  pins vs knowledge pack, buffer bounds, resource ownership) — 2 CRITICAL
  + 3 MAJOR, all fixed same night:
  - C1: SuperB `mmcTransact()` stack overflow — new `uint8_t outCap`
    parameter, copy clamps to `min(respLen, outCap)`; all 42 call sites
    pass `sizeof(resp)`; the four `resp[8]` buffers enlarged to `resp[64]`.
  - F-1: V_SENSE ADC read 4x low (10-bit ADC scaled as 12-bit) —
    `analogReadResolution(12)` added in M2 `setup()`.
  - M-1: auto-detect silently skipped its SWCAN stage —
    `swcan::sleepGpio()` now clears `isAcquired`.
  - M-2: GATEWAY/SLCAN had no mm control-channel pump —
    `mm::divertByte` pump added to both loops.
  - M-3: `mmSetFeature(FEAT_J1850, off)` didn't release in STN mode —
    now releases there too.
  - (Two agent-scored MAJORs downgraded to MINOR, fixed anyway:)
    F-2: TJA1051 silent pins 71/74 — new `src/m2/silent_pins.h`,
    driven HIGH on listen-only entry, LOW on exit.
    F-3: `j1850::release()` — drives pin 50 LOW only when the rail is
    confirmed down (`!rail9141::powered()`).
- Collateral sim-shim repairs (pre-existing breakage found during build
  verification): sim shim lacked `SPI.h` and `setListenOnlyMode()`;
  `analogReadResolution` stub added; smoke-test expectations updated for
  the ELM327 default boot mode.
- G1–G3 CAN init fixes (per parent agent notes; detail not in the phase
  notes — date approximate).

### Changed
- Default boot mode changed M2RET → ELM327 (Marcus's request; ELM
  auto-detects, long-press returns to ELM327).
- Unified debug instrumentation (`src/m2/debug.h`): `debug on` /
  `debug off` console command, zero-cost when off; ELM/STN wired, other
  modes carry the framework. Added for the Bolt EV in-vehicle retest.
- ChatGPT audit F4 (F1-7E framing) DEFERRED — architectural; F7/F8 polish
  deferred.
- Adversarial review adopted as a standing practice (2026-10-08): a
  separate reviewer subagent audits each phase against the phase
  requirements and the safety invariants before any zip goes to Marcus.
  Self-review alone is not sign-off. (Phase 11: 7 findings fixed.
  Phase 12's reviewer pass was interrupted before reporting — open.)

## [2026-10-07] — Host-native simulation harness

### Added
- Host-native sim harness (`sim/`, `docs/SIM.md`): both firmwares compile
  **unmodified** as Linux programs (`[env:sim_m2]` / `[env:sim_superb]`,
  PlatformIO native). Bee link = byte pipe via `sim/broker.py`
  (127.0.0.1:14001/14002, logs `sim/traffic.log`); SuperB web GUI on
  :18080, TCP:23 bridge shim on :14023, Bluetooth-SPP shim on :14024.
  Interactive launcher `sim/run.sh`.
- Smoke tests (`sim/tests/smoke.py`): **5/5 pass** on Linux — M2 console
  alive, SuperB HTTP GUI alive, real `F1 7E CMD_GET_STATUS` round-trip
  across the virtual Bee link, web-driven mode switch reaching the M2's
  mode manager, `ATI` over the Bluetooth shim returning `ELM327 v1.5`
  through the full chain.
- Additive only: no `src/m2`, `src/superb`, `lib`, or hardware env
  touched.

### Known limitations (documented, not bugs)
- Virtual CAN is loopback, not real arbitration; baud not enforced;
  J1850/GPIO interrupts not simulated; Wi-Fi presence faked.
  Vehicle-behavior simulation deferred (Marcus's decision).
  The sim mocks the HW layer and cannot catch SAM3X boot faults —
  phase-by-phase sim re-testing declined for that reason.

## [Phase 5] — 2026-10-04 — PROG (programming) mode

Build-verified (both envs green, zero project warnings). No bench —
INFERRED until in-vehicle testing.

### Added
- PROG mode (`src/m2/mode_prog.h/.cpp`): ECU flashing/tuning over UDS
  (ISO 14229-1) on CAN ISO-TP — 0x10 programming session → 0x27 security
  access (stub) → optional 0x31 erase → 0x34 RequestDownload → 0x36
  TransferData (streamed) → 0x37 RequestTransferExit → 0x11 reset, plus
  optional 0x23 read-back verify. State machine
  IDLE → SESSION → UNLOCKED → ERASED? → DOWNLOADING → STREAM_DONE → DONE,
  with loud ABORTED/ERROR terminal states.
- `docs/SAFETY_REVIEW.md`: the safety case, written BEFORE the code —
  MITM brick-vector rule, the three interlocks, voltage limitation, abort
  behavior, seed/key stub policy, dual-use statement.
- mm_channel CMD 0x11–0x19 (PROG supervision + chunked data path).
- SuperB web GUI `/prog` page: safety summary on the page itself, config,
  per-step wizard with confirms, streaming file upload with progress,
  finalize/verify/reset/abort, `/api/prog`.
- `docs/OVERVIEW.md`: the whole seven-mode system in one place.

### Security
- Destructive steps need arm-confirm (console `prog arm`, 120 s window;
  web per-step checkbox); PROG entry auto-suspends the MITM rule table
  and exit never restores it; explicit bus/ECU addressing required;
  `erase`/`download` refuse without power attestation; `0x27` is a stub
  forever — no OEM seed/key algorithm is or will be invented here.

## [Phase 4] — 2026-10-04 — MITM bridge mode + STN command set

Build-verified (both envs green, zero project warnings). No bench —
INFERRED until in-vehicle testing.

### Added
- MITM mode (`src/m2/mode_mitm.h/.cpp`): dual-CAN bridge CAN0↔CAN1 with
  an ordered rule engine (first match wins; actions pass/drop/modify/
  count; max 8 rules), loop suppression, per-rule rate limiting, stats,
  SerialUSB rule console, web supervision. Eager bus acquire at mode
  entry (a bridge that doesn't own both buses is not a bridge).
- STN mode (`src/m2/mode_stn.h/.cpp`): STN11xx-compatible command subset,
  implemented as an extension of the ELM parser (no duplication).
- SuperB web GUI `/mitm` page: rule table, config, live stats + LOG-frame
  view via `/api/mitm`; MITMLOG sniff ring.
- mm_channel CMD 0x0C–0x10 (MITM rule add/del/list, stats, config).

## [Phase 3] — 2026-10-04 — UDS/diagnostics mode + OBD2 poller

Build-verified. No bench — INFERRED until in-vehicle testing.
Standards cited (public, never invented): ISO 14229-1, ISO 15765-2,
SAE J1979, SAE J2012.

### Added
- UDS mode (`src/m2/mode_uds.h/.cpp`): CAN ISO-TP, 11-bit; services 0x10
  session control, 0x11 ECU reset (arm-confirm), 0x14 clear DTC,
  0x19 read DTC, 0x22 read data by identifier, 0x2E write data by
  identifier (arm-confirm), 0x28 communication control, 0x3E tester
  present, 0x85 control DTC setting. Physical (0x7E0/0x7E8) or functional
  (0x7DF) addressing; `7F <svc> <NRC>` decoded from the ISO 14229-1 NRC
  table. Console `uds` terminal + web UDS page.
- OBD2 poller mode: generic Mode 01/09 polling.
- Phase 6 filter/scheduler integration seams; Phase 12 auto-detect hooks.

### Security
- `0x27` security access is a stub: passes requestSeed/sendKey through
  and prints the seed, but no key algorithm is implemented and none ever
  will be — OEM seed/key algorithms are proprietary per-ECU secrets.
  Arm-confirm (10 s) on 0x11 and 0x2E; never automatic.

## [Phase 2] — 2026-10-04 — ELM327 mode + SuperB web GUI

Build-verified. No bench — INFERRED until in-vehicle testing.

### Added
- Mode-switch command channel (the Phase 1 blocker, resolved):
  **in-band binary channel `0xF1 0x7E LEN PAYLOAD XOR` in ALL modes**,
  verified against M2RET's own parser (0x7E is free in GVRET traffic;
  SavvyCAN never emits it; no legitimate AT text contains 0xF1). The
  SuperB filters app→M2 traffic (drops injected `F1 7E`) and diverts
  M2→app replies to the pending web-GUI request. `CMD_SET_MODE`
  dispatches exclusively through `ModeManager::requestMode()` — quiesce
  all buses, then acquire; no backdoor.
- ELM327 mode (`src/m2/mode_elm.h/.cpp`): genuine-protocol implementation
  (real AT response formats, real bus transactions, `NO DATA` on timeout),
  not an emulator. AT subset covering what Torque/FORScan actually use;
  `?` for unknown commands (genuine ELM327 behavior). Reports
  `ELM327 v1.5` (what most adapters and clones report; Torque and generic
  apps accept it — standing guidance is STN mode for FORScan).
- SuperB web GUI: mode switching, per-mode config pages, live status.

### Fixed
- Phase 2 pin-50-before-49 ordering violation (fixed again properly in
  Phase 7 via `rail9141`).

## [Phase 1] — 2026-10-04 — M2RET mode + SuperB wireless bridge (initial build)

Build-verified (both envs green, zero project warnings; third-party
warnings left as-is). No bench — INFERRED until in-vehicle testing.

### Added
- Mode manager (`mode_manager.h/.cpp`): modes {m2ret, elm327, stn, uds,
  mitm, obd2, prog}; safe switching — `quiesceBuses()` (CAN controllers
  to init/off, TH8056 to sleep, J1850 rail off) before the new mode
  acquires resources. Button1 (short = cycle, long >1 s = back to
  default) + SerialUSB console (`help`, `mode list`, `mode <name>`,
  `status`).
- M2RET mode (`mode_m2ret.h/.cpp`): GVRET binary protocol on Serial
  (Bee UART @115200, immutable); commands 0–14 plus 0xE7 binary-enable,
  faithful to M2RET @ ad9d5af (primary source, cloned for reference).
- SuperB bridge: Bluetooth Classic SPP (`M2-SuperB`), WiFi AP+STA (open
  AP `M2-SuperB`), TCP server port 23 (raw byte pipe to Bee UART,
  SavvyCAN GVRET-TCP target), up to 3 clients with documented
  last-writer-wins eviction.
- due_can enable-pin hazard neutralized: every `begin()` passes 255 so
  pins 50/48 (J1850_PWM_VPW, 5 V buck-boost) are never driven
  (`CAN_COMMON::begin(baud, 255)`).

### Known simplifications (Phase 1)
- No EEPROM persistence (RAM-only GVRET settings); no Lawicel ASCII;
  digital/analog inputs reply zeros; device info reports build 1.
- One upstream discrepancy found and documented: M2RET's DIG_INPUTS
  handler writes byte 1 = 6 (copy-paste from GET_CANBUS_PARAMS); this
  implementation follows the documented protocol (byte 1 = 2).
