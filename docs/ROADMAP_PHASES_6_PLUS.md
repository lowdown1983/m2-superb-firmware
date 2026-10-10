# Roadmap — Phases 6+ (APPROVED 2026-10-06)

`~/workspace/m2-superb-firmware/docs/ROADMAP_PHASES_6_PLUS.md`
Status: APPROVED 2026-10-06 by Marcus. Decisions recorded below; nothing started.
Basis: Phases 0–5 complete, all seven modes compile-verified
(`pio run` clean, both envs). Everything hardware-touching stays
**INFERRED** until in-vehicle testing (2026-10-04 decision, unchanged).

This roadmap is ordered by dependency, not by desirability: each phase's
prerequisites are the phases before it. Phase boundaries can be merged or
split at Marcus's call — the ordering is the part that matters.

## Standing rules (carried forward from Phases 0–5)

1. Acquire-on-entry / release-on-exit for every bus resource; `quiesceBuses()`
   on every mode switch. New modes add a row to the OVERVIEW.md resource table.
2. One core, not parallel stacks: new protocol code lands in the shared
   transport layer (`src/m2/`), host modes (ELM/STN/GVRET/SLCAN) consume it.
3. `docs/SAFETY_REVIEW.md` is updated in the same phase as any change that
   touches transmit authority, destructive services, or bridging.
4. Definition of done per phase: clean `pio run` (both envs) +
   `docs/PHASE<N>_NOTES.md` + evidence labels (VERIFIED = compiles/cited,
   INFERRED = untested on hardware).

---

## Phase 6 — Transport layer completion

**Goal:** make ISO-TP a first-class subsystem and add the two shared
services every diagnostic mode needs. Highest leverage: OBD-II, UDS, PROG,
ELM, and the future gateway all sit on this.

Contents:
- [ ] ISO-TP configurability: STmin, block size, P2/P2* timers, N_Bs/N_Cr
      timeouts, extended + mixed addressing, manual vs automatic flow-control
      generation, cancellation/timeout API, payload limits above 64 bytes.
      (`src/m2/isotp.*` — exists today as 11/29-bit normal-addressing only.)
- [ ] Generic TX scheduler: message ID, bus, payload, period, offset, count,
      start/stop conditions. Serves as the OBD poll engine and the general
      periodic-message generator. (`mm_channel` 0x06–0x0B pattern is the
      supervision template.)
- [ ] Unified bus filter model below OBD/UDS: bus → protocol → address → ID →
      mask → direction → payload mask. One implementation consumed by M2RET,
      ELM/STN, UDS, OBD2, MITM — not five copies.
- [ ] Error/status reporting: bus-off notification, RX overflow + dropped-frame
      counters, TX acknowledgements, per-bus statistics (extends the MITM
      stats pattern to all buses).

Prerequisites: none (builds on Phases 1–5 as-is).
Safety: no new transmit authority — scheduler and filters are bounded by the
owning mode's existing authority.

## Phase 7 — K-line / ISO 9141 / KWP2000 stack

**Goal:** the biggest missing protocol. Firmware today: zero K-line support
(the ELM protocol table only knows CAN and J1850; M2RET accepts LIN words
and ignores them).

Contents:
- [ ] 5-baud init, fast init, address/keyword handling, init retries.
- [ ] K/L line sequencing, inter-byte timing, checksum, message framing,
      echo handling, wake/sleep.
- [ ] ISO 9141 request/response, KWP2000 init + timing parameters + block
      transfers, tester/ECU addressing.
- [ ] Wire into the transport layer (Phase 6 filters/scheduler) and the ELM
      protocol table (ATSP 3/4/5/7/8/9/A/B); auto-detect hooks for Phase 12.
- [ ] Ownership: K-line/L-line share a power-control domain with J1850
      (pack constraint `j1850-no-transceiver` family) — explicit power
      sequencing in the driver, not in application code.

Prerequisites: Phase 6 (uses the filter + scheduler services).
Safety: K-line init sequences transmit on the bus by construction; init
requires the mode's transmit authority (same gate as ELM OBD commands).

## Phase 8 — J1850 + SWCAN production hardening

**Goal:** take the two draft drivers to production-grade.

Contents:
- [ ] J1850: symbol-timing state machines (VPW 10.4k / PWM 41.6k),
      arbitration, CRC, framing, header decode, idle detection, error
      detection, TX/RX state machines. (Today: GPIO+micros RX, INFERRED
      timings, frame-level TX/RX + CRC8 only.)
- [ ] SWCAN: TH8056 state machine (normal / high-speed / high-voltage wake /
      sleep), mode-transition timing, wake detection, listen-only, MCP2515
      recovery, bus-off recovery, filtering, timestamps. (Today: vendored
      Macchina lib, basic operation only.)
- [ ] M2RET fixes inherited from upstream: incoming checksum validation
      (currently calculated-but-not-checked), SWCAN consistency across
      commands, echo coverage, bus-count truthfulness, LIN command handling,
      `setSingleWireMode()` no-op.

Prerequisites: Phase 6 (filters, error reporting).
Safety: J1850/SWCAN share timer and power domains (TC7 collision, shared
J1850/9141 power) — the hardened drivers own those resources explicitly.

## Phase 9 — Diagnostic layer depth

**Goal:** turn the working OBD-II poller and UDS console into a real
diagnostic stack.

Contents:
- [ ] OBD-II: data-driven PID registry (PID, mode, request format, byte/bit
      extraction, endianness, signedness, scale, offset, unit, min/max,
      decoder) replacing the `decodePid()` switch; service modes
      02/05/06/07/08/09/0A; vehicle discovery engine (detect bus/protocol →
      find ECUs → addressing → supported services/PIDs → ECU capability
      table → exposed to SuperB).
- [ ] UDS: DID registry, RID registry, 0x23/0x24/0x2F services, routine
      control in UDS mode, security-level state, transfer/download state
      machine shared with PROG. 0x27 stays a stub — no key algorithms, ever
      (standing policy).
- [ ] ELM: filter commands (ATCRA/ATCF/ATCM), CAN header override
      application (ATSH accepted-but-not-applied today), remaining
      high-value AT commands from the parity list.
- [ ] Formalize capability gates: READ_ONLY default; ALLOW_TRANSMIT,
      ALLOW_DIAGNOSTIC_WRITE, ALLOW_ACTUATOR_CONTROL, ALLOW_FLASH_PROGRAMMING
      as explicit, per-mode grants (today: arm-confirm + safe mode, not
      formalized as roles).

Prerequisites: Phases 6, 7 (all transports present).
Safety: SAFETY_REVIEW.md update — the capability gates are the mechanism
that keeps a `0x2E`/`0x31`/flash command from behaving like a read.

## Phase 10 — Gateway mode (CAN ↔ SWCAN)

**Goal:** protocol translation between bus types. A **separate new mode**,
not a MITM extension — see Appendix A. **Scope decision (2026-10-06):
CAN↔SWCAN now; CAN↔LIN reserved as optional Phase 10b** (LIN master stack
does not exist; no current use case — kept as an option, not a commitment).

Contents:
- [ ] Gateway mapping tables: CAN ID ↔ SWCAN ID, payload transforms,
      direction rules. Data-driven, versioned, editable via SuperB web UI.
- [ ] Rate adaptation with explicit overflow policy: 500k CAN → 33.33k
      SWCAN is ~15:1; bursts must drop by policy (oldest/newest) with loss
      counters, never silently.
- [ ] Safety interlocks: gateway auto-suspends on PROG entry (same
      brick-vector reasoning as MITM rule suspension); write mappings need
      arm-confirm; never silently restored.

Optional Phase 10b (not approved, not scheduled): LIN master stack
(master task, schedule table, slave response handling, checksum,
sniffing) + CAN↔LIN mapping. Prerequisite for any LIN gateway work —
no LIN driver exists today (M2RET accepts LIN words and ignores them).

Prerequisites: Phases 6, 8 (hardened SWCAN; transport services).
Safety: a gateway **originates frames by construction** — the MITM
"never originate" stealth property does not transfer. Treat as
ALLOW_TRANSMIT+ authority minimum.

## Phase 11 — SuperB as communications appliance

**Goal:** SuperB owns wireless/UI/updates; vehicle logic stays on the M2
(standing architecture).

Contents:
- [ ] WebSocket transport (raw stream + GVRET) alongside TCP:23.
- [ ] Multi-client policy: one control owner (transmit authority), others
      passive monitors / logging clients. (Today: 3 TCP clients + BT + web
      share one pipe with no ownership concept.)
- [ ] Authentication + roles mapped to the M2 capability gates (Phase 9);
      secure pairing with physical authorization; unique device identity.
- [ ] Signed OTA firmware updates with anti-rollback version policy.
- [ ] Sleep/wake manager: voltage monitoring, engine-off detection,
      inactivity timer, modem sleep, wake on vehicle activity / UART /
      physical button. No sleep-current claims until measured (pack rule).

Deferred (decision 2026-10-06): BLE GATT service. Classic SPP already
serves every current app (Torque, SavvyCAN, FORScan); BLE adds ESP32
complexity with no current app demand. Revisit on demand — nothing in
Phases 6–10 depends on it.

Prerequisites: Phase 9 (capability gates to map roles onto).
Safety: roles are the remote half of the M2 safety gates; OTA signing is
the precondition for any remote update path.

## Phase 12 — Profiles, auto-detect, logging, test harness

**Goal:** the usability and reliability layer.

Contents:
- [ ] Vehicle profiles: bus, bitrate, frame format, addressing, protocol,
      filters, ISO-TP timing, padding, flow control, diagnostic addressing,
      retry limits, transmit permissions, logging settings — versioned and
      CRC-protected in EEPROM.
- [ ] Auto-detection engine: CAN (125k/250k/500k/1M × 11/29-bit,
      listen-only) → K-line (5-baud / fast init) → J1850 (VPW/PWM) →
      SWCAN (normal/high-speed/wake), looking for valid frames, stable
      timing, known arbitration IDs.
- [ ] Diagnostic session logging: timestamp, bus, request/response, decoded
      service, DID/PID, decoded value, raw bytes, ECU address, session.
      Capture formats: native binary, GVRET, CRTD, CSV; triggers,
      pre/post-trigger buffers, rotation, loss counters.
- [ ] Virtual ECU / self-test harness: the M2 talks to itself
      (CAN0↔CAN1 loopback with the harness as simulated ECU — no bench
      needed): scripted OBD/UDS responses, ISO-TP segmentation, negative
      responses, delayed/dropped/malformed frames, bus overload. Automated
      regression without a vehicle.
- [ ] DBC / custom-PID engine: signal extraction (bit numbering, endianness,
      signedness, scaling, multiplexing, enums) driving dashboard, logging,
      and API from one definition. Consumes the OEM database
      (`~/workspace/m2-oem-database/`) as its data source.
- [ ] SLCAN/Lawicel ASCII transport; M2RET modernization commands
      (CAPABILITIES, GET_VERSION, GET_BUSES, GET_BUS_STATUS, SET_BUS_CONFIG,
      SET_FILTER, CLEAR_FILTER, GET_STATS, RESET_STATS, log control,
      SET_SECURITY_LEVEL).

Prerequisites: Phases 6–9 (everything it configures, detects, or tests).
Safety: test harness runs on the loopback pair only — never against a
vehicle bus without explicit mode selection.

---

## Appendix A — The MITM gateway question (analysis)

Marcus asked whether MITM mode could gain CAN0↔SWCAN and CAN0↔LIN bridging,
possibly in a later phase. **Recommendation: yes, later phase — but as a new
GATEWAY mode (Phase 10), not as an MITM extension.** Reasons:

1. **It isn't man-in-the-middle; it's a gateway.** MITM semantics are
   transparent interception on one protocol: pass/drop/modify frames between
   two legs of the *same* bus. CAN0↔SWCAN crosses bitrates (500k vs 33.33k),
   controllers (SAM3X native vs MCP2515/SPI), and frame semantics (GM-LAN).
   There is no transparent forward — every crossing needs ID remapping,
   buffering across a ~15:1 rate mismatch, and an explicit overflow policy.
2. **LIN can't be bridged at all.** LIN is master-scheduled, not CSMA. A CAN
   frame arriving asynchronously cannot be "forwarded" onto LIN; the gateway
   must originate it from a schedule table. CAN↔LIN is a translation
   application with a LIN master stack (which doesn't exist yet), not a
   bridge rule.
3. **The stealth property doesn't transfer.** MITM's safety story is "never
   originates frames" (CAN ACK is physical-layer). A gateway originates
   frames by construction, so it needs the PROG-style interlock treatment,
   not the MITM treatment.
4. **The current rule engine doesn't fit.** Ordered first-match pass/drop/
   modify rules over identical frames assume both legs speak the same
   protocol. Gateway needs mapping tables + schedule tables — a different
   data model. Bolting it onto `mode_mitm.cpp` would corrupt a clean,
   reviewed component.

None of this is a no — GM vehicles (HS-CAN diagnostics ↔ SWCAN body) are a
genuine use case, and CAN↔LIN is useful for retrofit/bench work. It's a
scoping call: new mode, its own safety review, after the transports it
depends on are hardened (Phases 6, 8) and the LIN master exists.

## Appendix B — Deliberately out of scope (not firmware-reproducible)

From the parity analysis §31, agreed: physical size, 100 V surge protection,
sleep current figures, proprietary Bluetooth/security implementations,
proprietary OEM data coverage, exact STN firmware behavior. These are
hardware/proprietary-data differences. The firmware does not claim them.

## Appendix C — Decisions (resolved 2026-10-06)

1. Phase granularity: **keep 7 separate phases** (matches Phases 0–5 pattern:
   one focused deliverable, clean rebuild, notes doc each).
2. Order: **Phase 7 (K-line) before Phase 8 (hardening)** — capability
   unlock (pre-2008 vehicles, ELM ATSP slots) before polish of working drafts.
3. Gateway: **separate mode, CAN↔SWCAN scope**; CAN↔LIN reserved as optional
   Phase 10b (no commitment, no schedule).
4. BLE GATT: **deferred** — Classic SPP covers all current apps; revisit on
   demand.
5. SAE J1939 (2026-10-08): ELM protocol A needs different
   addressing/transport semantics and was out of Phase 7's K-line scope.
   Saved as a candidate for **Phase 9** (diagnostic depth) or **Phase 12**
   (profiles/auto-detect/logging) — no commitment to either yet.
