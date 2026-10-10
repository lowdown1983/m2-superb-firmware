# Phase 8 Notes — J1850 + SWCAN Production Hardening (2026-10-08)

Multi-mode firmware, `~/workspace/m2-superb-firmware/`. M2 = Arduino SAM3X8E
(Macchina M2), SuperB = ESP32-WROOM-32D. PlatformIO, envs `m2` + `superb`.

**Build status:** both envs compile green, **zero new warnings**
(remaining warnings are pre-existing third-party: MCP2515 lib, Arduino
framework USBCore, platformio.ini `src_filter` deprecation). The new
`swcan` module and the reworked `j1850` module pass a syntax check
against the host-sim shim headers (`-DSIM_M2`, same discipline as
Phases 6–7). No J1850/SWCAN vehicle exists here — everything
hardware-touching is **INFERRED** until tested against a suitable vehicle
(the available 2023 Chevy Bolt EV is CAN-only: no J1850 (pre-2008 GM) and
no SWCAN (GM-LAN)).

**What this phase does** (one core, not parallel stacks — everything new
lands in `src/m2/`, modes consume it):

1. **J1850 hardening** (`src/m2/j1850.h/.cpp` reworked) — explicit RX/TX
   state machines, PWM arbitration (long-dominates-short), header decode
   (1- and 3-byte), formal idle detection, error counters, optional CRC
   gate on the monitor path, Phase 6 transport wiring. The Phase 7
   `rail9141` integration (pin-50-before-49, refcounted release) is kept
   verbatim.
2. **SWCAN hardening** (new `src/m2/swcan.h/.cpp`) — TH8056 state machine
   (sleep / normal / high-speed / high-voltage wake-up) with datasheet
   mode-transition timings, wake detection, listen-only, MCP2515 health
   monitoring + recovery, bus-off detection, hardware filter support,
   drain-time timestamps, Phase 6 transport wiring. Wraps the vendored
   Macchina SWcan lib; `mode_m2ret.cpp` no longer touches the MCP2515
   directly.
3. **M2RET upstream fixes** (`src/m2/mode_m2ret.cpp`) — incoming checksum
   now validated (was calculated-but-not-checked), SWCAN control unified
   through the hardened driver (listen-only now actually applied,
   mode-before-setup ordering fixed, ISR detached on disable), echo
   honors the bus bit (deviation from upstream's hardcoded 0), bus index
   validated (bus-count truthfulness), LIN words handled truthfully (no
   LIN driver exists — non-zero LIN words are reported, not silently
   "accepted"), `SET_SW_MODE` consumes its byte and maps it onto the
   TH8056 (was: byte fell through into the command parser).
4. **Timer/power allocation map** — `docs/RESOURCE_OWNERSHIP.md` gains a
   Phase 8 section making the TC6/TC7/PWM6/SPI0/PIOC16/pin-49 allocation
   explicit (roadmap requirement).

---

## 1. J1850 hardening

**State machines.** RX was an implicit `inFrame` flag with a loose edge
classifier; it is now an explicit `RxSt { IDLE, IN_FRAME }` machine:
IDLE accepts only an SOF (active 163–239 us VPW / 40–56 us PWM);
IN_FRAME classifies each symbol against its window and aborts to IDLE —
counting `symErr` — on anything out of window. TX was inline bit-bang
code; it is now explicit `TxPh` phases
(WAIT_IDLE → SOF → DATA → EOD → IFS → DONE, with ARB_LOST / BUS_BUSY
exits). Still blocking bit-bang with interrupts off — timing-critical —
but reviewable.

**Arbitration.** VPW keeps its passive-phase arbitration (matafonoff
rule, tolerance 10 us). PWM gains arbitration (Phase 8): per the
retrolib BAE 5030 "long dominates short" rule, after driving the active
pulse the driver samples the bus — if it stays active past our pulse +
tolerance, another node is sending a longer ('0') pulse and we lost.
Losses count `arbLost` (+ `busstat::noteTxFail`, dbg log).

**Header decode.** `decodeHeader(frame, n, hdrLen, h)`: hdrLen 1 or 3.
3-byte (OBD-style): h0 = priority byte, h1 = target, h2 = source — the
source byte feeds the Phase 6 OEM-seam `addr` slot in the filter model
("J1850 node byte"). 1-byte: priority = h0>>5 (INFERRED), source = h0.
`transact()` uses the request's header length; the monitor path uses a
configurable default (`setHeaderLen`, default 3 = OBD-style). The 1-vs-3
auto-decision is deliberately NOT attempted — J1850 networks are
homogeneous per vehicle and guessing wrong corrupts the node byte.

**Idle detection.** `idle()` (instantaneous passive read) and
`waitIdle(timeoutMs)` (continuous passive stretch ≥ EOF/EOD-gap, the
same thresholds TX uses). TX still enforces its own wait, now via the
shared primitive.

**Error detection.** `Counters { symErr, crcErr, arbLost, qOverflow,
glitch }` via `counters()`/`resetCounters()`. Queue-full policy stays
"drop new" (documented) and now counts `qOverflow` (+
`busstat::noteRxOverflow`). Glitch-filter hits count `glitch`.

**CRC gate (monitor).** `setCrcGate(bool)` — when on, `pollFrame()`
drops bad-CRC frames (counted `crcErr`). Default **off**: ATMA shows raw
traffic like upstream tools. `transact()` always validates (unchanged).

**Transport wiring (Phase 6).** The monitor path (`pollFrame()`) and the
transact response path consult `busfilter::accept(Bus::J1850,
J1850_VPW/J1850_PWM, addr=nodeByte, id=headerByte, RX, ...)` when the
table is non-empty; rejects count `busstat::noteDropped(J1850)`.
`noteTx/noteRx/noteTxFail` feed `busstat::Bus::J1850` (new bus index 3).
`setPresent(J1850, …)` on acquire/release. K-line-style CAN_SR polling
does not apply (no CAN_SR on the GPIO driver) — `busstat::poll()` still
only touches indices 0–1. All J1850 events route through `dbg::`.

**Kept verbatim (Phase 7):** pin 50 settled before `rail9141::powerUp`,
release via `rail9141::powerDown` (INV-PWR49/INV-MODE50), TX polarity
assumption, adaptive RX idle sampling, the matafonoff-derived timing
constants and CRC-8 routine.

## 2. SWCAN hardening (`swcan::`)

**TH8056 state machine.** `Mode { SLEEP, NORMAL, HIGHSPEED, HV_WAKEUP }`
driven on pins 64/65 per the pack-verified table (00/11/10/01).
Transitions carry the datasheet settling waits (pack VERIFIED §2.5):
Sleep→active 50 ms, active↔active 30 ms, →sleep 5 ms (INFERRED — no
datasheet value; sleep is the weak-pulldown default). `setMode()` is the
only path that touches the mode pins (plus GPIO-only `sleepGpio()`).

**Wake detection.** `waitWake(timeoutMs)`: enter HV_WAKEUP, poll the INT
pin for bus activity (the tWUF 10–70 us filter is TH8056 hardware),
transition to NORMAL on wake. INFERRED until a real wake.

**Listen-only.** `setListenOnly()` → MCP2515 `setListenOnlyMode()`,
remembered across `recover()`. (This also fixes the M2RET bug below.)

**MCP2515 recovery.** `poll()` (called from `m2ret::loop()` while
acquired): reads EFLG/TEC; three consecutive 0xFF reads (MISO stuck high
= SPI dead, INFERRED heuristic) → `recover()`; EFLG.TXBO → counts
`busstat::noteBusOff(SWCAN)` (new counter fn), logs, re-requests
MODE_NORMAL (the MCP2515 auto-recovers after 128×11 recessive bits per
its datasheet — we make sure we come back in the expected mode);
RX0OVR/RX1OVR → `noteRxOverflow`. `recover()` = Reset + re-Init +
re-apply mode/listen-only/filters. EFLG bit positions are MCP2515
datasheet §REGISTER 9-1 (behavior INFERRED here).

**Filtering.** `setHwFilter(id, ext)` / `setHwMask(mask, val)` program the
MCP2515 RXB0/RXB1 registers (the vendored lib keeps mask/filter
independent). Software gate via `busfilter::accept(SWCAN/SWCAN/…)` on
every drained frame when the table is non-empty (Phase 6 pattern).

**Timestamps.** `getFrame(frame, &tsUs)` stamps drain-time `micros()`
(INFERRED precision). True SOF capture on pin 68 (TC7_B) is deliberately
NOT implemented: TC7 is mutually exclusive with J1850_PWM_RX (TC7_A) per
the timer_rule, and both drivers explicitly abstain from TC7 (see §4).

**Transport wiring.** `sendFrame` → `noteTx/noteTxFail(SWCAN)`;
`getFrame` → `noteRx`, filter rejects → `noteDropped`;
`setPresent(SWCAN, …)` on acquire/release. All events via `dbg::`.

**Ownership move.** The `SWcan` object moves from a `static` in
`mode_m2ret.cpp` into `swcan.cpp`; the PIOC16 ISR trampoline moves with
it. `m2ret::swcanISR()` is removed (nothing else referenced it).
`swcan::sleepGpio()` preserves the quiesceBuses guarantee: GPIO-only,
safe before/after any MCP2515 init. `swcan::reconfigure()` implements
live speed change as release+acquire.

## 3. M2RET upstream fixes

| Fix | Before | After |
|-----|--------|-------|
| Incoming checksum | `checksumCalc()` computed, ignored (faithful to upstream M2RET.ino:1049) | Enforced on BUILD_CAN_FRAME and ECHO_CAN_FRAME: mismatch → frame dropped, never transmitted, dbg-logged |
| SWCAN consistency | `mode()` driven before `setupSW()`; `swcanListenOnly` set but never applied to the MCP2515; disable path left the PIOC16 ISR attached | All paths through `swcan::` (acquire = init→NORMAL→INT; release detaches); listen-only applied via `setListenOnly()` |
| Echo coverage | `emitFrame(buildFrame, BUS_CAN0)` — bus bit parsed then discarded (faithful to upstream M2RET.ino:1258 bug) | Honors the parsed bus bit (`outBus & 1`). **Deviation from upstream**, documented |
| Bus-count truthfulness | `outBus = inByte & 3` — bus 3 silently fell through `sendCanFrame` doing nothing | `outBus > 2` → dropped + dbg log |
| LIN handling | LIN1/LIN2 words accepted and ignored unconditionally | Words parsed; zero = silent no-op, non-zero = dbg-reported "LIN requested but unsupported" (no LIN driver exists; the LIN master stack stays reserved for optional Phase 10b — NOT built here) |
| `SET_SW_MODE` | Mode byte never consumed — fell through and was parsed as the next command (parser desync) | Byte consumed; 0/1/2/3 mapped onto the TH8056 (sleep/high-speed/HV-wake/normal) when SWCAN is acquired; anything else logged and rejected |

GET_NUMBUSES (3) and GET_EXT_BUSES (LIN reported disabled/0) were
already truthful and are unchanged.

## 4. Timer / power allocation (roadmap requirement)

`docs/RESOURCE_OWNERSHIP.md` gains a Phase 8 section with the full
allocation table. Summary of the decisions:

- **TC7**: explicitly unclaimed by *both* drivers. J1850 keeps the
  proven GPIO+micros RX (no timer capture); SWCAN does not implement
  SOF capture on pin 68. The INV-TC7 mutual exclusion therefore holds by
  construction, and a future TC-capture implementation on either side
  must acquire the whole channel.
- **TC6**: unclaimed by J1850 (GPIO RX); RGB-blue keeps TC6_A
  with the standing rule not to touch TC6_CH6 while J1850 is acquired.
- **PWM6**: conceptually owned while J1850-PWM is active; the PWM
  peripheral is never enabled (GPIO bit-bang) and `release()` parks
  J1850N_TX.
- **SPI0**: SWCAN MCP2515 is the sole onboard claimant (INV-SPI0, NPCS3);
  no other firmware SPI0 client exists.
- **PIOC16**: single callback slot, owned between `swcan::acquire` /
  `release`; pin 67 is INPUT (set by the MCP2515 constructor — pack
  rule satisfied, VERIFIED in MCP2515.cpp).
- **Pin 49 rail**: unchanged from Phase 7 (`rail9141` refcount;
  pin-50-before-49; the Phase 8 J1850 rework keeps the integration
  verbatim).
- **CPU-spin bit-bang** (J1850 TX/RX, K-line 5-baud init) uses
  `micros()`/`delay`, not hardware timers — no TC collision by design.
  `txsched` is `millis()`-based.

## 5. Supervision / console

No new mm commands (nothing the phase needs). Console additions on
SerialUSB: `j1850 status` (acquired, proto, queue depth, counters),
`j1850 crcgate on|off`, `j1850 hdrlen 1|3`, `swcan status` (acquired,
mode, speed, listen-only), `swcan wake <ms>`, `swcan recover`. The
`busstat` console now prints all five buses; `CMD_BUS_STATS` (0x21)
accepts bus 0–4.

## 6. Debug instrumentation

`debug.h` untouched. New `dbg::` events: J1850 acquire/release,
arbitration loss, TX bus-busy, response CRC failures, bad host
checksums, bad bus/mode bytes, LIN-unsupported notices; SWCAN acquire/
release/mode transitions/wake waits/register-unreadable/bus-off/
error-warning/recovery.

---

## Resource ownership (Phase 8)

- **No new hardware claims.** No new modes, no new TC channels, PWM,
  SPI clients, PIO interrupts, DAC, or power rails. `quiesceBuses()`
  needed no structural change (verified by reading it):
  `m2ret::swcanSleep()` → `swcan::sleepGpio()` (same GPIO-only
  guarantee), `j1850::release()` unchanged.
- The `SWcan` object and ISR trampoline move into `swcan.cpp`
  (module-internal). Mode-level ownership in `docs/OVERVIEW.md` is
  unchanged (M2RET owns SWCAN; ELM owns the J1850 block lazily) — the
  table is therefore untouched (rule: update only on ownership change).
- Pin-safety invariant holds: every native-CAN `begin()` still passes
  `(baud, 255)` — verified by grep; no new CAN `begin()` calls added.
  The MCP2515/SWCAN path drives only CS (63), INT (67, input), M0 (64),
  M1 (65) — never 50/48.
- `docs/SAFETY_REVIEW.md` **not** updated: Phase 8 hardens existing
  authority (no new transmit capability, no new modes, no bridging).
  The scheduler/filter/stats wiring is bounded by the owning mode's
  existing authority, same reasoning as Phase 6. This paragraph is the
  safety record for the phase.

## INFERRED list (Phase 8 — no J1850/SWCAN vehicle)

- J1850 TX polarity (pin HIGH = bus active); transceiver non-inversion.
- PWM RX windows, EOD/EOF/IFS, arbitration tolerance + the ~1 us
  `digitalRead` sampling assumption in `pwmTransmit`.
- VPW TX_IFS 300 us; 5 ms idle timeouts.
- 1-byte header priority extraction (h0>>5).
- TH8056 mode-transition settle waits (50/30/5 ms applications of the
  §2.5 maxima); SLP behavior; HVWU wake detection via INT.
- MCP2515 EFLG bit positions (datasheet §REGISTER 9-1 — standard, but
  behavior here is untested); 0xFF-reads = dead SPI heuristic;
  auto-recovery after TXBO.
- Drain-time `micros()` SWCAN timestamps (true SOF capture reserved).
- `0x23 ReadMemoryByAddress` ALFID default (carried from Phase 7).
- The Bolt EV cannot test any of this (CAN-only vehicle).

## Deliberately left out / deferred

- **TC6/TC7 timer-capture J1850 RX**: deliberately not built (Phase 2
  decision stands — register-level TC code can't be validated without
  a bench; the GPIO approach is the cited proven one). The allocation
  map reserves the option.
- **SWCAN SOF timestamping** (pin 68, TC7_B): reserved; blocked by the
  TC7 mutual exclusion until a TC7 owner is designated.
- **KWP2000 0x83 on-the-wire** (Phase 7 deferral): untouched.
- **LIN master stack**: reserved for optional Phase 10b — NOT started.
  Phase 8 only makes M2RET's LIN *handling* truthful.
- **0x27**: still a stub, untouched (not Phase 8 business).
- **ChatGPT F4** (F1 7E framing): still deferred, untouched.
- **Bringup/SPI**: `src/m2/main.cpp` untouched (settled).
- **OEM database**: not consumed (first use stays Phase 9/12).
- **HW-055** (live `setupSW(33333)` register capture): still open —
  needs a bench, not a code change.

## Sim harness note

`pio run -e sim_m2` still fails on `SPI.h` — **pre-existing** (boot fix,
not Phase 8). `swcan.cpp`/`j1850.cpp` pass `-DSIM_M2` syntax checks
against the sim shim (`g++ -fsyntax-only -DSIM_M2 -I src/sim_shim`;
SPI/register paths `#ifndef SIM_M2`-guarded, including the
`MCP2515_defs.h` include). `sim_superb` still builds. Phase 8 does not
make the sim worse.

## Adversarial review fixes (Marcus standing practice, Phase 8)

A separate reviewer subagent audited the Phase 8 build before delivery
(verdict on the first pass: FAIL — 4 blockers). All blockers fixed,
minors/nits addressed:

- **M1 (blocker):** `SETUP_EXT_BUSES` re-sent while SWCAN was already
  acquired silently dropped speed/listen-only changes (`acquire()`
  refuses when acquired; SavvyCAN re-sends on every connect). Fixed:
  the acquired path now goes through `swcan::reconfigure()` (which
  skips the release/acquire cycle when the speed is unchanged) +
  `setListenOnly()`.
- **M2 (blocker):** main-loop MCP2515 SPI transactions raced the PIOC16
  ISR's `intHandler()` (the vendored lib has no interrupt guards) —
  corrupted register reads could spuriously trigger `recover()` or a
  phantom bus-off, and a corrupted TX buffer could reach the bus. Fixed:
  every main-context `hw.*` transaction in `swcan.cpp` is now wrapped in
  `noInterrupts()`/`interrupts()` — the arbiter that
  `RESOURCE_OWNERSHIP.md` Phase-1 checklist item 4 requires. (The hazard
  pre-dated Phase 8; Phase 8's per-loop `poll()` widened it, so Phase 8
  fixes it.)
- **M3 (blocker):** the →SLEEP settle comment falsely claimed "no
  datasheet value" while the pack VERIFIED value is Standby→Sleep
  100–500 ms, and 5 ms went the unsafe direction. Fixed: →SLEEP now
  waits 100 ms (the VERIFIED minimum; the exact Normal→Sleep mapping is
  labeled INFERRED), `release()` settles, and the comments cite the pack
  honestly. `sleepGpio()` stays immediate by design (quiesce path; no
  consumer depends on sleep completion — documented in `swcan.h`).
- **M4 (blocker):** the `-DSIM_M2` syntax-check claim was false as
  stated (the unconditional `MCP2515_defs.h` include has no shim).
  Fixed: the include is now `#ifndef SIM_M2`-guarded (every use was
  already in non-SIM branches) and the check reproduces with plain
  `-I src/sim_shim -DSIM_M2`.
- **m2:** `waitIdle()` was dead public API and the notes claimed TX used
  "the shared primitive" — now true: both TX paths call `waitIdle(5)`.
- **m3:** `recover()`'s docstring no longer promises hardware-filter
  re-application (nothing sets them yet); documents the vendored
  `SetRXMask` MODE_CONFIG side effect for future callers.
- **m4:** the GMLAN 0x100 high-speed hop now costs ~60 ms (2× 30 ms
  datasheet-max settles) vs upstream's 2 ms. Kept deliberately: ≤30 ms
  is the datasheet *maximum* (guaranteed-complete bound), not a required
  dwell, and shortening it without a bench would be the unsafe
  direction. A shorter measured wait is future bench work.
- **m5:** sub-2-byte monitor frames are now counted (`symErr`) instead
  of dropped silently.
- **m1 (minor):** the VPW arbitration sample has no blanking interval
  while the PWM path got explicit tolerance reasoning. Examined:
  the VPW sample follows the vehicle-tested matafonoff pattern and the
  pack's "no firmware delay" note for the discrete J1850 circuit; adding
  an unvalidated blanking delay would risk breaking the proven pattern.
  Kept, labeled INFERRED.
- **Nits:** removed dead `swcan::isr()`; removed unused `TxPh ph`;
  fixed `t0` shadowing in `pwmTransmit`; removed redundant
  `(void)outBus`; `CMD_BUS_STATS` comment now names all five buses;
  softened the unverified "not very applicable to M2" quotation;
  reconciled the Phase-1 checklist item 1 with the Phase 8 TC6/TC7
  abstention.

## Files changed

New: `src/m2/swcan.h/.cpp`, `docs/PHASE8_NOTES.md`.
Modified: `src/m2/j1850.h/.cpp` (state machines, PWM arbitration, header
decode, idle detection, counters, CRC gate, transport wiring),
`src/m2/busstat.h/.cpp` (J1850=3, SWCAN=4 buses; `noteBusOff()`),
`src/m2/mode_m2ret.cpp` (+6 upstream fixes, SWCAN via `swcan::`,
drain-time timestamps), `src/m2/mode_m2ret.h` (removed `swcanISR`,
updated comments), `src/m2/mode_manager.cpp` (`CMD_BUS_STATS` bus 0–4,
`busstat` console five buses), `src/m2/mm_channel.h`
(CMD_BUS_STATS comment), `docs/RESOURCE_OWNERSHIP.md` (Phase 8
allocation map).
