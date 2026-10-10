# Phase 7 Notes — K-line / ISO 9141 / KWP2000 Stack (2026-10-08)

Multi-mode firmware, `~/workspace/m2-superb-firmware/`. M2 = Arduino SAM3X8E
(Macchina M2), SuperB = ESP32-WROOM-32D. PlatformIO, envs `m2` + `superb`.

**Build status:** both envs compile green, **zero new warnings**
(remaining warnings are pre-existing third-party: MCP2515 lib, Arduino
framework USBCore, platformio.ini `src_filter` deprecation). The two new
shared modules also pass a syntax check against the host-sim shim headers
(`-DSIM_M2`, stubbed — same discipline as Phase 6). No K-line vehicle
exists here — everything hardware-touching is **INFERRED** until tested
against a K-line vehicle (the available 2023 Chevy Bolt EV is CAN-only).

**What this phase adds** (one core, not parallel stacks — everything new
lands in `src/m2/`, ELM/STN consume it):

1. **K-line driver** (`src/m2/kline.h/.cpp`) — 5-baud init, fast init,
   K/L line sequencing, inter-byte timing, checksum, framing, echo
   handling, wake/sleep, ISO 9141 request/response, KWP2000 init +
   timing parameters + block transfers, tester/ECU addressing.
2. **Shared rail owner** (`src/m2/rail9141.h/.cpp`) — the `J1850 9141 ON`
   net (pin 49) is refcounted between the J1850 and K-line drivers
   (pack INV-PWR49); also fixes the Phase 2 pin-50/49 ordering.
3. **ELM integration** — ATSP 3/4/5 (+ATTP/ATDP/ATDPN/ATSH/ATMA),
   OBD-II request dispatch over K-line.
4. **Phase 6 transport wiring** — `busfilter` KLINE enums consumed,
   `busstat` extended with a K-line bus, tester-present keep-alive via
   the `txsched` scheduler.
5. **Safety review update** — `docs/SAFETY_REVIEW.md` §9 (K-line init
   transmit authority); Phase 12 auto-detect hooks left as stubs.

---

## 1. Hardware truth (knowledge pack v1.4.0 — VERIFIED, never the banned specs)

| Net | Arduino pin | SAM3X | USART | OBD-II pin |
|-----|-------------|-------|-------|------------|
| K TX (TJA1027 U7 TXD) | 55 (`LIN_KTX`) | PA11 | USART0 TX = Serial1 TX | — |
| K RX (U7 RXD) | 56 (`LIN_KRX`) | PA10 | USART0 RX = Serial1 RX | — |
| K SLP (U7 SLP_N) | 57 (`LIN_KSLP`) | PB4 | GPIO | — |
| L TX (TJA1027 U8 TXD) | 58 (`LIN_LTX`) | PA13 | USART1 TX = Serial2 TX | — |
| L RX (U8 RXD) | 59 (`LIN_LRX`) | PA12 | USART1 RX = Serial2 RX | — |
| L SLP (U8 SLP_N) | 60 (`LIN_LSLP`) | PB7 | GPIO | — |
| K-line bus | — | — | — | 7 |
| Rail `J1850 9141 ON` | 49 (`PS_J1850_9141`) | PB5 | GPIO | — |
| J1850 mode select | 50 (`J1850_PWM_VPW`) | PB8 | GPIO | — |

Pack sources: `source/m2-pins.yaml` (pins 55–60, VERIFIED via S-VARH /
S-VARCPP / S-DOCS / S-SCH-I), `source/constraints.yaml`
`tja1027-kline` (U7/U8 = TJA1027, 20 kBd max, SLP_N on symbol pin 2) and
`tja1027-specs` (mode timings), `source/conflicts.yaml` C7 + INV-PWR49 +
INV-MODE50 (shared rail rules). TJA1027 max 20 kBd — we run 10400.

## 2. Driver design (`kline`)

- **Acquire/release** owns the whole power sequence (roadmap requirement —
  not left to application code): `acquire()` = rail via `rail9141`
  (refcounted) → transceivers to normal (SLP HIGH — INFERRED polarity
  from the SLP_N naming) → `Serial1`/`Serial2` at 10400 → 50 ms settle
  (INFERRED). `release()` = tester-present stop → UARTs end → SLP LOW
  (sleep, INFERRED) → rail released refcounted → stats presence cleared.
  Both idempotent; `release()` is safe for `quiesceBuses()`.
- **5-baud init** (`initSlow`): 300 ms idle, GPIO bit-bang of the target
  address byte at 5 baud (200 ms/bit — start + 8 LSB-first + stop) on K
  (L mirrors it when `useL`), then back to USART0 @10400 for the keyword
  handshake (`0x55`, KB1, KB2, inverted-KB2, inverted-address). Retries:
  3 full attempts (INFERRED). The GPIO↔USART mux switchback is standard
  Due behavior but **INFERRED** here.
- **Fast init** (`initFast`): 25 ms LOW / 25 ms HIGH wake via GPIO, then
  `StartCommunication` (`C1 33 F1 81` + checksum) at 10400; parses
  KB1/KB2 from the positive response (`... C1 KB1 KB2`).
- **Framing/checksum**: 8-bit sum mod 256 for both 9141 and KWP2000
  (INFERRED standard behavior). `transact()` appends the checksum,
  verifies the transceiver echo byte-for-byte (TJA1027 loops TX→RX,
  INFERRED), waits for the response (P2-based default timeout), collects
  by inter-byte gap (> 2×P1max, INFERRED — K-line has no end-of-frame),
  verifies + strips the response checksum.
- **KWP2000**: `kwpService()` builds `[0x80|len tgt src sid data cs]`;
  `readMemoryByAddress()` = 0x23 with ALFID 0x33 (INFERRED common
  default); `testerPresent()` = 0x3E with response-suppressed
  subfunction (INFERRED 0x00) via `sendOnly()` (no response expected).
  Timing struct P1..P4 with ISO 14230-2 defaults (INFERRED);
  `setTiming()`/`getTiming()`; P4 enforced between TX bytes, P1×2 used
  as the RX frame delimiter.
- **Monitor**: `pollRx()` assembles frames from `Serial1` by inter-byte
  gap (call from the mode loop); `pollFrame()` drains the 4-frame queue
  honoring the shared filter table (Phase 6 pattern).
- **L-line**: wake/secondary use only in Phase 7 (`initSlow(..., useL)`,
  `Serial2` held while acquired). No L-line data sessions.
- **Never fights J1850**: init refuses while `j1850::acquired()` (defense
  in depth; ELM's `busRelease()` already sequences protocol switches).

## 3. Shared rail (`rail9141`)

Two-owner refcount (`J1850`, `KLINE`) over pin 49:

- `powerUp(o)` is idempotent per owner; drives pin 49 HIGH. The caller
  must settle owner-specific pins first (J1850: pin 50 — INV-PWR49).
- `powerDown(o)` drives pin 49 LOW **only when no owner holds the rail**
  (INV-PWR49). The old `j1850::release()` yanked pin 49 LOW
  unconditionally — that would now brown out an active K-line session.
- **Ordering fix**: `j1850::acquire()` previously drove pin 49 HIGH
  *before* settling pin 50, violating INV-PWR49 ("pin 49 HIGH only after
  pin 50 is settled"). Phase 7 reorders: pin 50 first, then
  `rail9141::powerUp(J1850)`. `j1850.h`'s resource-claim comment updated.

## 4. ELM integration (ATSP 3/4/5)

- Protocol table gains `{3, "ISO 9141-2"}`, `{4, "ISO 14230-4 KWP (5 baud
  init)"}`, `{5, "ISO 14230-4 KWP (fast init)"}` with `isKline`/`kInit`
  fields. **A (genuine: SAE J1939) deliberately NOT added** — J1939 needs
  different addressing/transport semantics; out of Phase 7's K-line
  scope. Documented deviation (was in the task's protocol list).
- `busActivate()`: K-line branch acquires the driver and runs the init
  (fast or 5-baud per protocol). **This call site is the transmit-
  authority gate** — it runs only from user-initiated OBD/ATTP/monitor
  activity via `ensureProtocol()`, the same gate as ELM's CAN/J1850 OBD
  commands (safety case: `docs/SAFETY_REVIEW.md` §9). Failed init →
  release + activation fails.
- `obdKline()`: proto 3 → `request9141()` with `hdrKline` (default
  `68 6A F1`, ATSH-overridable, INFERRED); protos 4/5 → `kwpService()`
  with the first request byte as SID. Timeout floor `max(ATST, 1 s)`
  (INFERRED, mirrors the J1850 path).
- `ATSH` routes to `hdrKline` when a K-line protocol is selected
  (ATSP's `protoNum`, else the active protocol); J1850 behavior unchanged.
- `ATMA`/`ATMR`/`ATMT` monitor K-line via `pollRx()`/`pollFrame()`.
- `ATDP`/`ATDPN`/`ATSP`/`ATTP` work through the shared table (no
  per-protocol code).
- **Auto-detect**: K-line is deliberately NOT in `AUTO_ORDER`; a
  Phase-12-hook comment marks where `kline::probe()` (stub, returns
  false) will be called by the Phase 12 auto-detection engine.

## 5. Phase 6 transport wiring

- `busfilter`: the reserved `Bus::KLINE`/`Proto::KLINE` enums are now
  live. `transact()` and `pollFrame()` consult `accept()` when the table
  is non-empty (Phase 6 pattern); `addr` = ECU source-address byte
  (response header byte 2, INFERRED layout) — the Phase 6 OEM-database
  seam, still unpopulated.
- `busstat`: `Bus` gains `KLINE = 2` (`COUNT = 3`); the CAN_SR poll loop
  and SR seeding are guarded to the two native controllers (K-line has
  no CAN_SR). `CMD_BUS_STATS` accepts bus 2; the `busstat` console prints
  a `KLINE:` row. Feeds: TX/RX bytes, echo-mismatch → txFail, queue
  exhaustion → rxOverflow, filter rejects → dropped.
- `txsched`: **tester-present keep-alive** via a callback-only slot
  (`kline::testerPresentStart(ms)` / `testerPresentStop()`); the slot
  fires `0x3E` while acquired and is removed by `release()`. No txsched
  changes were needed (callback slots bypass the CAN presence check).

## 6. Supervision: mm commands 0x23–0x24 + console

Following the Phase 6 supervision template (F1 7E framing, `CMD|0x80` +
status replies, LE payloads; 2 new `mm::Ops` fields appended — the
existing positional initializer is unaffected):

| CMD | Request | Reply |
|-----|---------|-------|
| 0x23 KLINE_STATUS | `[]` | `[status, acquired, initDone, kb1, kb2, target, tester]` |
| 0x24 KLINE_INIT | `[type 1=slow, 2=fast]` | `[status, kb1, kb2]` |

`CMD_KLINE_INIT` is refused unless ELM/STN mode is active (transmit
authority) or the init fails. The SuperB web GUI does not know these
commands yet — documented for a future UI pass (same standing note as
Phase 6).

Console (SerialUSB): `kline status | kline init slow|fast | kline tx
<hex> | kline timing show | kline timing set <p1min|p1max|p2min|p2max|
p3min|p3max|p4min|p4max> <ms> | kline addrs <targetHex> <testerHex> |
kline tp <ms|off> | kline release`. Bus-touching subcommands (`init`,
`tx`) refuse outside ELM/STN mode.

## 7. Debug instrumentation

Phase 7 routes K-line events through the unified `dbg::` mechanism
(one flag, `debug on/off`, no reflash): acquire/release, init attempts
and keyword bytes, echo mismatches, checksum failures, NO DATA,
tester-present arming. `debug.h` itself untouched.

---

## Resource ownership (Phase 7)

- **New claims:** USART0/`Serial1` (pins 55/56) + PB4 SLP (K-line),
  USART1/`Serial2` (pins 58/59) + PB7 SLP (L-line, wake use) — held
  between `kline::acquire()` and `kline::release()`. Lazy, ELM-driven
  (protocols 3/4/5), same discipline as ELM's CAN/J1850 lazy acquire.
- **Shared:** pin 49 rail via `rail9141` (refcounted with J1850);
  pin 50 untouched by the K-line driver (J1850's mode select).
- No new TC channels, PWM, SPI clients, PIO interrupts, DAC, or power
  rails. `quiesceBuses()` now also calls `kline::release()` (idempotent).
- `docs/OVERVIEW.md` resource table gains a K-line column (ELM/STN:
  lazy); `docs/RESOURCE_OWNERSHIP.md` gains the Phase 7 section.
- Pin-safety invariant holds: every CAN `begin()` still passes
  `(baud, 255)` — verified by grep; no new CAN begin() calls added.
- `docs/SAFETY_REVIEW.md` **updated** (§9): K-line init transmit
  authority — what init does on the wire, the three-part gate, the
  shared-rail reasoning, honest limitations.

## INFERRED list (Phase 7 — no K-line vehicle)

- 5-baud bit time (200 ms), Tidle 300 ms, sync/keyword timeouts.
- Fast-init Twup 25/25 ms; `StartCommunication` frame/response layout.
- P1–P4 defaults and their enforcement points.
- SLP_N polarity (HIGH = normal, LOW = sleep — from `_N` naming).
- GPIO↔USART0 mux switchback for bit-banged init/wake.
- TJA1027 TX→RX echo behavior and its byte-exactness.
- Response frame delimiting by inter-byte gap (2×P1max).
- Checksum = 8-bit sum (both protocols).
- `0x23` ALFID 0x33 default + ECU support for ReadMemoryByAddress.
- Tester-present subfunction 0x00 = response-suppressed.
- Interop: keyword values, init retry count (3), rail settle 50 ms.
- The Bolt EV cannot test any of this (CAN-only vehicle).

## Deliberately left out / deferred

- **Protocol A (SAE J1939)**: not added — different addressing/
  transport semantics, out of K-line scope. Would be a Phase 9/12 item.
- **Auto-detect**: `kline::probe()` is a stub; Phase 12 owns it.
- **L-line data sessions**: wake use only; no L-line protocol sessions.
- **KWP2000 0x83 AccessTimingParameter on-the-wire**: timing is stored
  and enforced locally; the 0x83 service exchange is not implemented
  (no vehicle to validate against).
- **0x27**: still a stub everywhere, untouched (not Phase 7 business).
- **ChatGPT F4** (F1 7E framing): still deferred, untouched.
- **Bringup/SPI**: `src/m2/main.cpp` untouched (settled).
- **OEM database**: not consumed (see §5 seam). First use stays
  Phase 9/12.

## Sim harness note

`pio run -e sim_m2` still fails on `SPI.h` — **pre-existing** (boot fix,
not Phase 7). `kline.cpp`/`rail9141.cpp` pass `-DSIM_M2` syntax checks
(stubbed), so Phase 7 does not make the sim worse. `sim_superb` still
builds (verified via the both-envs `pio run`).

## Files changed

New: `src/m2/kline.h/.cpp`, `src/m2/rail9141.h/.cpp`,
`docs/PHASE7_NOTES.md`.
Modified: `src/m2/j1850.h/.cpp` (rail via rail9141; pin-50-before-49
ordering fix), `src/m2/busstat.h/.cpp` (KLINE bus), `src/m2/mode_elm.cpp`
(protocols 3/4/5, obdKline, monitor branch, ATSH routing, auto-detect
hook comment), `src/m2/mode_manager.cpp` (quiesceBuses, mm 0x23/0x24
callbacks, `kline` console, busstat label), `src/m2/mm_channel.h/.cpp`
(CMD 0x23/0x24 + Ops + dispatch), `docs/SAFETY_REVIEW.md` (§9 K-line
authority), `docs/OVERVIEW.md` (resource table, INFERRED inventory, doc
map), `docs/RESOURCE_OWNERSHIP.md` (Phase 7 claims).
