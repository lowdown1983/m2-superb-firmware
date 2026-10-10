# Phase 6 Notes — Transport Layer Completion (2026-10-08)

Multi-mode firmware, `~/workspace/m2-superb-firmware/`. M2 = Arduino SAM3X8E
(Macchina M2), SuperB = ESP32-WROOM-32D. PlatformIO, envs `m2` + `superb`.

**Build status:** both envs compile green, **zero new warnings**
(remaining warnings are pre-existing third-party: MCP2515 lib, Arduino
framework USBCore, platformio.ini `src_filter` deprecation). The four new
shared modules also pass a syntax check against the host-sim shim headers
(`-DSIM_M2`). No bench — everything hardware-touching is **INFERRED**
until in-vehicle testing.

**What this phase adds** (one core, not parallel stacks — everything new
lands in `src/m2/`, modes consume it):

1. **ISO-TP configurability** (`src/m2/isotp.h/.cpp`) — `isotp::Config`:
   STmin, block size, P2/P2* timers, N_Bs/N_Cr timeouts, extended + mixed
   addressing, manual vs automatic flow-control generation, cancellation
   API, payloads up to `ISOTP_BUF_MAX` (256) bytes. `Endpoint` is
   byte-compatible (existing callers' aggregate init still compiles).
2. **Generic TX scheduler** (`src/m2/txsched.h/.cpp`) — message ID, bus,
   payload, period, offset, count, start/stop, one-shot, plus an optional
   fire callback. The OBD2 poller now runs on it (callback-only slot).
3. **Unified bus filter model** (`src/m2/busfilter.h/.cpp`) — bus →
   protocol → address → ID → mask → direction → payload mask. One table,
   one `accept()`, one `idMatch()` primitive consumed by M2RET, ELM/STN,
   UDS, OBD2, MITM.
4. **Error/status reporting** (`src/m2/busstat.h/.cpp`) — bus-off
   notification (CAN_SR BOFF polling), RX overflow + dropped-frame
   counters, TX fail counters, per-bus tx/rx frames+bytes. Extends the
   MITM stats pattern to all native buses.

---

## 1. ISO-TP configurability (`isotp::Config`)

```cpp
struct Config {
    AddrType addrType = AUTO;   // NORMAL_11/29, EXTENDED, MIXED_11/29
    uint8_t addrExt = 0;        // AE / target-address byte
    uint16_t nBsMs = 1000;      // N_Bs: TX waits for Flow Control
    uint16_t nCrMs = 1000;      // N_Cr: RX waits for Consecutive Frame
    uint16_t p2Ms = 50;         // P2 (informational; reported for Phase 9)
    uint16_t p2StarMs = 5000;   // P2*: overall RX window when timeoutMs == 0
    uint8_t maxBlockSize = 0;   // BS we advertise in OUR Flow Control
    uint8_t stMin = 0;          // STmin byte we advertise in OUR Flow Control
    bool autoFlowControl = true;// false: manual FC via sendFlowControl()
    uint16_t maxPayload = 64;   // reassembly cap, 8..ISOTP_BUF_MAX (256)
    const volatile bool* cancel = nullptr;  // cooperative cancel flag
};
```

- **F5 folded in, not bolted on.** The 2026-10-08 BS/STmin enforcement
  (honor the peer's FC block size + STmin, including 100–900 µs
  0xF1–0xF9 encodings, re-arm FC after each block) is now the general
  TX path; the N_Bs wait uses `cfg.nBsMs` instead of the hardcoded 1000.
  Our own FC advertises `cfg.maxBlockSize`/`cfg.stMin` (defaults preserve
  the old CTS/BS=0/STmin=0 behavior exactly).
- **Addressing.** `EXTENDED`/`MIXED_11` put the AE byte at `data[0]` and
  shift every PCI field by one, in both directions; RX requires
  `data[0] == addrExt`. `MIXED_29` = 29-bit normal framing (the AE rides
  in the CAN ID, built by the caller into `txId` — ID construction rules
  are OEM-specific and belong to Phase 9's data, not the transport).
  `AUTO` derives 11/29-bit normal from the existing `Endpoint.extended`
  flag, so every current caller behaves identically.
- **N_Cr** is now a real per-stream watchdog: a stalled reassembly slot
  is dropped and counted as RX overflow (previously a stalled stream
  just sat in the slot table until the overall timeout).
- **Oversize RX** (FF length > `maxPayload`, or no free reassembly slot)
  now sends FC-OVFLW when we own FC generation (was: silently ignored).
- **Manual FC**: `isotp::sendFlowControl(ep, fs, bs, stMin, cfg)` sends
  one FC frame with correct addressing; `autoFlowControl=false` skips
  the automatic FC on RX.
- **Cancel**: `cfg.cancel` is polled in every wait loop; TX returns
  `TxResult::CANCELLED`, RX returns whatever was collected so far.
- **Global default**: `isotp::setDefault()/getDefault()` — the default
  used whenever a caller passes `cfg == nullptr`. Initialized to the
  historical behavior, so all existing modes are byte-identical in
  behavior until someone changes it (via `CMD_ISOTP_CFG` or the
  `isotp set` console command). This is what makes the mm supervision
  command meaningful without touching every call site.
- **STPX `x:` wired up.** The Phase 4 "stored, not wired" limitation is
  gone: `STPX ... x:41` now transmits with extended addressing (AE=0x41)
  and the raw-frame RX path requires the AE byte to match. `mode_stn.h`
  subset comment updated.
- `Response.data` grew 64 → `ISOTP_BUF_MAX` (256); `Response.len` is now
  `uint16_t`. Callers only use `.data`/`.len` generically — verified by
  the clean build. mm-channel replies still truncate to the 48-byte
  frame cap (documented, unchanged).

## 2. TX scheduler (`txsched`)

8 slots. `add()` copies the slot and arms `nextAt = now + offsetMs`.
`poll()` (called once per `ModeManager::loop()`) fires due slots:
optional CAN TX (bus 0/1 only — SWCAN reserved for Phase 8) then the
`onFire` callback. Count semantics: `-1` = infinite, `0` rejected at the
console, positive = total fires then auto-deactivate; `periodMs == 0` =
one-shot. A callback that overruns its period resyncs to now (cadence,
not backlog — INFERRED policy).

**Ownership:** the scheduler owns no bus. A TX slot fires only while its
target bus is `busstat::present()` (set by the mode manager wherever it
begins/ends a controller); if the bus is down the frame is skipped and
counted as TX-fail. The mode that configured the slot must hold the bus —
same acquire/release discipline as every other bus user.

**The scheduler IS the OBD poll engine.** `mode_obd2.cpp` no longer has
its own sweep timer: `obd2 start` → protocol auto-detect as before →
on lock, a callback-only slot (`txEn=false`, `onFire=sweepTick`) is
armed with the poll interval. `obd2 stop`, re-config, bus/baud change,
and mode exit remove the slot. `obd2 interval <ms>` live re-arms via
`txsched::setPeriod()`. The sweep itself (blocking isotp
request/response per PID, CSV on the Bee link, mm-divert pumping
between PIDs) is unchanged — only the cadence source moved.

## 3. Unified bus filter model (`busfilter`)

16-entry table. `accept(bus, proto, addr, id, dir, payload, len)`:
true when the table is empty (every mode's behavior unchanged until
filters exist) or any filter fully matches. One filter =
`bus→proto→addr→id→mask→dir→payload(off/mask/val)` plus `invert`.
`mask == 0` = any ID; `addr == 0xFFFFFFFF` = any address;
`payMask == 0` = no payload condition. `idMatch()` is the single
ID/mask primitive.

Consumption map (duplicates removed/delegated):

| Mode | Before | After |
|------|--------|-------|
| M2RET | none (raw stream) | `emitFrame()` consults `accept()` — empty table = identical output |
| ELM monitor | inline `f.id != filterId` | `busfilter::idMatch()` (mask 0x7FF/0x1FFFFFFF by ext) |
| ELM/STN STPX | hand-rolled pair loops | `busfilter::idMatch()` for STCFCPA rx matching |
| MITM rules | inline `((f.id & mask) == (r.id & mask))` | `busfilter::idMatch()` (provably identical semantics) |
| isotp RX | endpoint check only | additionally `accept()` **when the table is non-empty**; drops counted in busstat. FC-wait frames bypass the table (transport control always passes) |

The pair/rule tables themselves stay where they are (application
state); only the match primitive is unified — one implementation, not
five copies.

**Phase 9/12 seam (OEM database):** the `addr` field is the protocol-
address slot the OEM database will fill — ISO-TP target/source address
(AE byte), ECU physical address, J1850 node byte. Phase 6 only plumbs
it (isotp passes the AE byte as `addr` when filtering); Phase 9's
PID/DID registries and discovery engine will populate filters from the
OEM database's diagnostic-access data without a transport refactor.
The OEM database itself is **not** consumed in Phase 6.

## 4. Error/status reporting (`busstat`)

Per native bus (CAN0/CAN1; SWCAN/J1850 reserved for Phase 8):
`txFrames/rxFrames/txBytes/rxBytes/rxOverflow/txFail/busOff/dropped`
(+ `errWarn` tracked, not yet in the mm blob).

- **bus-off notification**: `busstat::poll()` reads `CAN_SR` BOFF
  (and WARN/ERRA) and counts 0→1 transitions, with `dbg::` logging.
  Only for begun controllers (`busstat::setPresent`, set at every
  begin/end site in `mode_manager.cpp`) — a clock-gated CAN peripheral
  must not be touched. Reading CAN_SR mirrors what due_can's own ISR
  already does; BOFF/WARN are live state bits, not clear-on-read.
  INFERRED until a real bus-off is observed.
- **Feeds**: isotp `sendCan`→tx/txFail, `waitFrame`→rx, reassembly
  exhaustion/N_Cr/SN-error→rxOverflow, filter rejects→dropped;
  txsched→tx/txFail; MITM bridge→rx/tx/dropped on every leg and
  policy drop; ELM monitor + STPX RX→rx; M2RET pump→rx/tx/txFail.
- **TX acknowledgements**: `due_can::sendFrame()`'s bool is
  mailbox-acceptance, counted as tx vs txFail. True controller-level
  TXOK is not exposed by due_can 2.1.6 — documented limitation, not
  claimed.

## 5. Supervision: mm commands 0x1A–0x22 + console

Following the 0x06–0x0B supervision template (F1 7E framing,
`CMD|0x80` + status replies, LE payloads; 10 new `mm::Ops` fields
appended — the existing positional initializer is unaffected):

| CMD | Request | Reply |
|-----|---------|-------|
| 0x1A TXSCHED_ADD | `[bus][id LE32][ext][len][data…][periodMs LE32][offsetMs LE32][count LE32]` (20+len B) | `[status, slot]` |
| 0x1B TXSCHED_DEL | `[slot]` | `[status]` |
| 0x1C TXSCHED_LIST | `[start]` | `[status, nTotal, nRet, 25B entry]` (1/reply) |
| 0x1D TXSCHED_CTL | `[slot][start 0/1]` | `[status, active]` |
| 0x1E FILTER_ADD | 20B `[bus][proto][addr LE32][id LE32][mask LE32][dir][payOff][payMask][payVal][invert]` | `[status, idx]` |
| 0x1F FILTER_DEL | `[idx]` (0xFF = clear all) | `[status, nLeft]` |
| 0x20 FILTER_LIST | `[start]` | `[status, nTotal, nRet, 20B entries]` (2/reply) |
| 0x21 BUS_STATS | `[bus 0/1]` | `[status, 32B blob]` |
| 0x22 ISOTP_CFG | write: 15B config / read: `[]` | `[status]` / `[status, 15B blob]` |

Console (SerialUSB): `txsched list|add|del|start|stop|clear`,
`filter list|add|del|clear`, `busstat [reset]`,
`isotp show | isotp set <nbs|ncr|p2|p2star|bs|stmin|autofc|maxpayload|addrtype|addrext> <v>`.
The SuperB web GUI does not know these commands yet — documented for a
future UI pass, not a regression (all pre-existing commands unchanged).

## 6. Debug instrumentation

Phase 6 routes transport events through the unified `dbg::` mechanism
(one flag, `debug on/off`, no reflash): scheduler slot fires, bus-off /
error-warning transitions, RX overflow, TX fail, ISO-TP cancel/N_Cr/
OVFLW events. `debug.h` itself untouched — still compiles and works.

---

## Resource ownership (Phase 6)

- **No new hardware claims.** No new modes, no new TC channels, PWM,
  SPI clients, PIO interrupts, DAC, or power rails. `quiesceBuses()`
  needed no change (verified by reading it).
- `busstat::setPresent()` is called at the existing begin/end sites
  (`modes::canBegin0/1`, `canEnd0/1`, `enter/exitM2RET`,
  `quiesceBuses()`, live `CMD_SET_CAN_BAUD`) — presence tracking only,
  no ownership change. `docs/OVERVIEW.md`'s resource table is therefore
  untouched (rule: update only on ownership change).
- Pin-safety invariant holds: every `begin()` still passes `(baud, 255)`
  — verified by grep; no new begin() calls were added.
- `docs/SAFETY_REVIEW.md` **not** updated: Phase 6 adds no new transmit
  authority. The scheduler and filters are bounded by the owning mode's
  existing authority (scheduler TX requires the bus to be up, i.e. held
  by the configuring mode; filters can only *narrow* what a mode sees).
  This reasoning is the safety record for the phase.

## INFERRED list (Phase 6 — no bench)

- CAN_SR BOFF/WARN polling transitions (never observed on hardware).
- N_Bs/N_Cr/P2/P2* tuning against real ECUs (defaults are the ISO
  15765-2 standard values).
- Extended/mixed addressing frame layouts (standard layout, untested —
  the Bolt EV test vehicle is 11-bit normal, so this waits for a
  vehicle that needs it).
- STmin sub-millisecond (0xF1–0xF9) timing via `delayMicroseconds`.
- Scheduler cadence under load; the no-backlog resync policy.
- Filter-table effects on live traffic (empty-table default = no change).
- STPX `x:` against a real extended-addressing ECU.

## Deliberately left out / deferred

- **ChatGPT F4** (F1 7E framing swallowing ordinary CAN data containing
  `F1 7E`): still deferred as architectural. The unified filter model
  is the natural home for any future fix (a filter/policy that steers
  such frames around the control parser), noted here per the Phase 6
  brief — not implemented.
- **K-line**: Phase 7. `busfilter::Proto::KLINE` / `Bus::KLINE` enum
  values are reserved.
- **J1850/SWCAN stats + filters**: reserved for Phase 8 hardening.
- **ATSH header-override application**, **ATCRA/ATCF/ATCM**: Phase 9
  (ELM depth), per the roadmap.
- **0x27**: still a stub, untouched (not Phase 6 business).
- **Bringup/SPI**: the `SPI.begin()` boot fix and bringup harness are
  untouched (settled).
- **OEM database**: not consumed (see §3 seam). Its first use remains
  Phase 9 (PID/DID registries, discovery) and Phase 12 (DBC engine).

## Sim harness note

`pio run -e sim_m2` currently fails on `SPI.h: No such file or
directory` (src/m2/main.cpp) — **pre-existing**, from the 2026-10-08
`SPI.begin()` boot fix, not from Phase 6 (no Phase 6 file touches
main.cpp or SPI). The four new modules pass a syntax check against the
sim shim headers (`-DSIM_M2`). `sim_superb` still builds. The sim failure
is left for its owner (the boot-fix change); Phase 6 did not regress it.

## Files changed

New: `src/m2/busstat.h/.cpp`, `src/m2/busfilter.h/.cpp`,
`src/m2/txsched.h/.cpp`, `docs/PHASE6_NOTES.md`.
Modified: `src/m2/isotp.h/.cpp` (Config, addressing, timers, FC, cancel,
>64B, default-config API), `src/m2/mode_elm.cpp` (monitor/STPX via
idMatch, STPX `x:` wired, monitor/STPX RX stats), `src/m2/mode_mitm.cpp`
(rule match via idMatch, per-bus stats feed), `src/m2/mode_m2ret.cpp`
(GVRET output filter gate, TX/RX stats), `src/m2/mode_obd2.cpp`
(scheduler-driven sweep), `src/m2/mode_manager.cpp` (10 new mm ops,
`txsched`/`filter`/`busstat`/`isotp` consoles, `txsched::poll()` +
`busstat::poll()` in `loop()`, presence hooks),
`src/m2/mm_channel.h/.cpp` (CMD 0x1A–0x22 + dispatch),
`src/m2/mode_stn.h` (STPX `x:` comment).
