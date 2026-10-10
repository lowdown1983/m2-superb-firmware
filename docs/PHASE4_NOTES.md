# Phase 4 Notes — MITM Bridge Mode + STN Command Set (2026-10-04)

Multi-mode firmware, `~/workspace/m2-superb-firmware/`. M2 = Arduino SAM3X8E
(Macchina M2), SuperB = ESP32-WROOM-32D. PlatformIO, envs `m2` + `superb`.

**Build status:** both envs compile green, **zero project warnings**
(remaining warnings are pre-existing third-party: MCP2515 lib, Arduino
framework USBCore, platformio.ini `src_filter` deprecation). No bench —
everything hardware-touching is **INFERRED** until in-vehicle testing.

**What this phase adds:**

1. **MITM mode** (`src/m2/mode_mitm.h/.cpp`) — dual-CAN bridge CAN0↔CAN1
   with an ordered rule engine, loop suppression, per-rule rate limiting,
   stats, a SerialUSB rule console, and web supervision.
2. **STN mode** (`src/m2/mode_stn.h/.cpp`) — STN11xx-compatible command
   subset, implemented as an *extension* of the ELM parser (no parser
   duplication).
3. **SuperB web GUI** — `/mitm` page (rule table, config, live stats +
   LOG-frame view via `/api/mitm`), MITM + STN buttons on the mode page,
   MITMLOG sniff ring.
4. **mm_channel** — CMD 0x0C–0x10 (MITM rule add/del/list, stats, config)
   under the existing F1 7E framing.

---

## 1. MITM bridge design (`src/m2/mode_mitm.*`)

### 1.1 Architecture

MITM is a **pure software bridge between the two native CAN controllers**.
CAN0↔CAN1, independently configurable bauds (default both 500000, both
staged or live-reconfigured via `mitm` console / web / mm channel). Both
controllers are begun **at mode entry** (`mitm::begin()` → `modes::canBegin0/1`
with the 255 enable-pin override) with promiscuous RX filters (same pattern
as ELM/M2RET). There is no bus activity before entry; there is no idle
"staged but unbegun" state — unlike ELM/UDS, MITM acquires eagerly, because a
bridge that doesn't own both buses at entry is not a bridge.

No SWCAN, no J1850 in Phase 4. Rationale (per the task decision): bridging a
single-wire bus into the rule engine is a separate design — SWCAN shares TC7
with J1850_PWM_RX (collision table row 1) and has its own SOF handling; the
two-native-CAN bridge is the clean, bench-verifiable core.

### 1.2 Rule engine semantics (exact)

- Rule = `{id, mask, dir (a2b/b2a/both), action, modOff/modMask/modVal,
  rateLim (frames/s, 0 = unlimited)}`. Max **8 rules** (fits the frame caps).
- **First matching rule wins.** Match = `((id & mask) == (rule.id & mask))`
  AND (rule.dir == both OR rule.dir == frame direction). ID width is not
  validated — the mask can restrict to 11 or 29 bits as needed.
- Actions:
  - `pass` — forward as-is.
  - `drop` — discard (counts in per-direction dropped).
  - `modify` — `data[modOff] = (data[modOff] & ~modMask) | (modVal &
    modMask)`; the frame is then forwarded. **If `modOff >= frame length`,
    the modification is skipped, the frame is forwarded UNMODIFIED, and the
    modified counter is NOT incremented** (the rule *hit* still counts).
    Offset range is 0–7 (enforced at rule creation).
  - `count` — forward and count (hit counter; useful as a trip-wire that
    doesn't alter traffic).
  - `log` — forward AND emit a copy on the Bee link:
    `MITMLOG,<A2B|B2A>,<id hex>,<ext>,<len>,<data hex>` — sniffed by the
    SuperB into the `/mitm` live view (same pattern as OBD2 CSV).
- Anything matching **no rule** hits the compiled-in default:
  **pass-all-both-directions** (subject to the default-rule rate cap and the
  global direction gate).
- A global direction gate (`mitm dir a2b|b2a|both`, default both) applies
  **before** rules; gated-out frames count as dropped.
- The mm control channel stays live during bridging: incoming Serial bytes
  are diverted through `mm::divertByte()` (new shared helper in
  `mm_channel.h/.cpp`, same semantics as the ELM pump's inline divert).
  Rules/config therefore work from the web UI while the bridge runs.

### 1.3 Loop suppression — why it exists, how it works

Bridging two taps of the **same physical bus** (the common bench/vehicle
setup: Y-cable or two OBD ports on one bus) without loop suppression
ping-pongs every frame forever: A→B forward appears on bus B, is received on
CAN1, matches the bridge again, forwards B→A, repeat — an exponential storm
bounded only by controller TX queues. This is the single most important
correctness property of the mode.

Design: a 16-entry ring of recently forwarded frames `{id, len, data[8],
direction, timestamp}`. Before forwarding, a frame **identical (id + length
+ data)** to one forwarded in the **opposite** direction within `loopWinMs`
(default **5 ms, INFERRED** — see tuning below) is **dropped** (counts as
loop-suppressed). Zero disables it.

Tuning note (INFERRED, no bench): 5 ms is deliberately generous. A genuine
echo of a just-forwarded frame arrives within ~1 frame time (~250 µs at
500k for an 8-byte frame); 5 ms also covers ISR/`loop()` scheduling jitter
on the SAM3X at high bus load. Documented trade-off: two **genuinely
different** ECUs emitting byte-identical frames within 5 ms on **opposite**
buses will also suppress one copy. For diagnostic tapping (same bus both
sides) this is the desired behavior; for gatewaying two **different** buses
with overlapping ID spaces, consider narrowing the window or scoping rules
by direction.

### 1.4 Rate limiter

Optional per-rule cap (frames/s, sliding 1-second window); excess frames are
dropped (counted separately as rate-limited). The default rule's cap comes
from config (`defRateLim`, 0 = unlimited). Tuning is INFERRED — it is a
safety valve for rule-misfire storms, not a tested value.

### 1.5 Stealth — honest definition

**Stealth = the bridge never originates frames locally; it only forwards
what it received.** That is the complete Phase-4 definition, and the flag is
stored/reported as such.

What stealth does NOT do (documented deliberately): CAN ACK is
**physical-layer** — the transceiver/controller ACKs received frames in
hardware (dominant bit in the ACK slot). No firmware on the M2 can suppress
it short of controller listen-only mode, and a forwarding bridge **cannot**
use listen-only mode (it must transmit to forward). Anyone claiming a
firmware "silent bridge" on CAN is misrepresenting the protocol. The
genuine silent control in this firmware is **STCMM 0** (§2.4), which applies
to *monitoring* sessions only — not to bridging.

### 1.6 Stats

Per direction: forwarded, dropped, modified. Plus loop-suppressed and
rate-limited totals, per-bus **load estimate** (`frames/s × 128 nominal
bits/frame ÷ baud`, INFERRED estimate — not a measurement), direction,
stealth flag, rule count. Stats reset on mode entry; **rules persist**
across mode switches (config, like UDS addressing).

### 1.7 Console (SerialUSB)

`mitm help` | `mitm rule add <id> <mask> <dir> <action> [off mask val] [rate=N]`
| `mitm rule del <n>` | `mitm rule list` | `mitm stats` | `mitm clear`
| `mitm stealth on|off` | `mitm dir a2b|b2a|both`.
IDs/masks are hex (`7E8`, `7FF`); modify params: decimal offset 0–7, hex
mask/value. Works even when MITM is not active (staged).

### 1.8 mm_channel protocol (CMD 0x0C–0x10)

Same F1 7E framing, `CMD|0x80` + status replies (`ST_OK/ST_BAD_ARG/ST_ERR/
ST_UNKNOWN_CMD`). Payloads are little-endian (documented in `mm_channel.h`):

| CMD | Request | Reply |
|-----|---------|-------|
| 0x0C RULE_ADD | 15B: `[id LE32][mask LE32][dir][action][modOff][modMask][modVal][rateLim LE16]` | `[status, idx]`; ST_ERR if table full |
| 0x0D RULE_DEL | `[idx]` (0xFF = clear all) | `[status, nRules]` |
| 0x0E RULE_LIST | `[startIdx]` | `[status, nTotal, nRet, entries…]`, 20B/entry (`[idx][id][mask][dir][action][modOff][modMask][modVal][rateLim LE16][hits LE32]`), **≤2 entries/reply** (48-byte frame cap) |
| 0x0F STATS | `[]` | 38B: `[status][fwdAB/BA, dropAB/BA, modAB/BA, loopSupp, rateDrop (LE32 ×8)][loadA][loadB][dir][stealth][nRules]` |
| 0x10 CFG | WRITE 14B: `[baudA LE32][baudB LE32][dir][stealth][loopWinMs LE16][defRateLim LE16]` → `[status, applied]` (live re-init when active, else staged); READ `[]` → 17B: `[status][baudA][baudB][dir][stealth][loopWinMs][defRateLim][active]` |

`mm::Ops` gained five fields (`mitmRuleAdd/Del/List/Stats/Cfg/Get`) appended
at the end; the dispatch is nullptr-safe. A shared `mm::Divert` +
`divertByte()` helper was added for non-line-oriented data planes (MITM's
Serial pump uses it).

---

## 2. STN mode (`src/m2/mode_stn.*` + ELM-core extension)

### 2.1 Design: extension, not duplication

STN mode **is** the ELM core with ST-command parsing enabled. `stn::begin()`
sets the flag (`elm::setStnMode(true)`) then calls `elm::begin()`; `stn::loop/
exit()` delegate to `elm::`. All AT commands keep working (genuine STN chips
are ELM327 supersets — OBDLink FRPM: "fully compatible with the ELM327 AT
command set"). `handleLine()` routes `ST…` lines to `stCommand()` when the
flag is on (`AT` is checked second, so `ATST` etc. still reach the AT parser).
STN state resets in `elm::begin()`; the flag itself is set by `stn::begin()`
and cleared by `stn::exit()`. STN owns the Bee UART data plane exactly like
ELM (SuperB bridging unchanged); the mm control channel is diverted in the
shared pump.

### 2.2 Implemented subset (FRPM-verified)

Sources: **OBDLink Family Reference and Programming Manual (FRPM) Rev E
(2025-01-10)** and **STN11XX datasheet Rev B (2009-12-04)**, cross-checked
against real-hardware community evidence (commit messages from a tool
measured against a real OBDLink MX+).

| Command | Behavior | Evidence |
|---------|----------|----------|
| `STI` | → `STN1170 v4.0.0` | FRPM §8.4 (`STN<device_id> vX.Y.Z`); tools detect STN by the "STN" prefix (measured: real MX+ answers `STN2255 v5.12.4`) |
| `STDI` | → `M2-SuperB r4.0` | FRPM §8.4 (`<device_name> rX.Y`; measured: `OBDLink MX+ r3.1.3`) |
| `STPX h:…,d:…/l:…,t:…,r:…,x:…,f:…` | Custom CAN transmit + raw-frame RX (full arg parsing; `h:` header/CAN ID, `d:` data hex, `l:` length → `DATA>` prompt flow, `t:` timeout, `r:` response count, `x:` extended address, `f:` flags). Segmentation on temporarily per FRPM; responses print as raw frames; `FC RX TIMEOUT` on flow-control timeout (genuine wording). `d:` and `l:` are mutually exclusive (FRPM). | FRPM §8.8; real-HW measured transcripts |
| `STM [n]` / `STMA [n]` | Raw-CAN monitor; STM uses filters as set, STMA forces pass-all; optional frame count → auto-exit back to prompt (not `STOPPED`) | FRPM §8.9 |
| `STMR hh` / `STMT hh` | Monitor with ID filter (STN-spelled ATMR/ATMT behavior) | FRPM §8.9 |
| `STCMM 0/1/2` | CAN monitoring mode: **0 = receive-only, no CAN ACKs (default)**; 1 = normal node; 2 = also receive error frames (answered OK, not supported on SAM3X/due_can). **This is the genuine STN "silent" control** | FRPM §8.8 |
| `STCFCPA txadd,rxadd` / `STCFCPC` | Flow-control address pairs (up to 4, used as the STPX response filter); clear | FRPM §8.8 |
| `STCAF fmt[,tt]` | Addressing format 0/1/2 (stored; format 0 wired through ISO-TP as before) | FRPM §8.8 |
| `STCSEGR 0/1`, `STCSEGT 0/1` | Segmentation toggles (stored; STPX segments per FRPM regardless) | FRPM §8.8 |
| `STCSTM ms` | STmin offset (stored, decimal ms) | FRPM §8.8 |
| `STCTOR fc,cf` | FC/CF timeouts in ms (stored; defaults 75/150 per FRPM) | FRPM §8.8 |

**INFERRED choices inside STPX:** `t:` units are ms; header ≤3 hex digits →
11-bit ID else 29-bit; unset `r:` collects until timeout with a 200 ms
quiet-exit (mirrors the ISO-TP path); default TX ID 0x7DF.

**Deliberate limitations (documented in code):** STPX is **CAN-only** in
Phase 4 (J1850 protocols answer `?`); `x:`/`f:` are accepted and parsed but
the extended-address byte is not wired into the ISO-TP path; `STCSEGR/T`,
`STCSTM`, `STCTOR` are stored, not wired into timing; STCMM 2 answers OK but
error-frame RX is not supported by due_can/SAM3X.

### 2.3 Honest subset boundary — what answers `?`

- `STSL` (standalone) — does not exist in the FRPM; only as the PowerSave
  family prefix (`STSLCS`, `STSLEEP`, …).
- `STCS` (standalone) — same; only the `STCS*` family prefix (`STCMM`,
  `STCFCPA`, …).
- `STWCF` — does not exist in the FRPM.
- `STDP` / `STDPN` — do not exist; `ATDP`/`ATDPN` already cover this via ELM
  compatibility.
- Anything else starting with `ST` that isn't above.

### 2.4 STCMM 0 — genuine silent monitoring

When STN mode is on and `STCMM` is 0 (the default), monitor sessions
(`STM`/`STMA`/`STMR`/`STMT`) put the SAM3X CAN controller into **listen-only
mode** via due_can 2.1.6's `enable_autobaud_listen_mode()` /
`disable_autobaud_listen_mode()` (CAN_MR_ABM bit — verified at
`.pio/libdeps/m2/due_can/src/due_can.cpp:521-533`; due_can's own comment:
"go into listen only mode so we don't clobber the bus"). The controller
transmits nothing: **no ACKs, no error frames**. Restored on every monitor
exit path (user interrupt, count reached). The ACK-suppression is a silicon
feature driven through a verified driver call — but its effect on a real
bus is still INFERRED (no bench). This is the honest counterpart to §1.5:
silent *monitoring* is real; silent *bridging* is impossible.

### 2.5 Monitor refactor (shared with ATMA/MR/MT)

`monitor(filterId, useFilter, maxFrames)` now takes a frame budget: `-1` =
run until interrupted (genuine ELM327 behavior, prints `STOPPED`), `>= 0` =
auto-exit after N frames with a plain `>` prompt (FRPM behavior for
`STM [n]`). `stnMode`/`stCmm` are forward-declared above `monitor()`; the
STN block defines them below. ATMA/MR/MT pass `-1` — behavior unchanged.

### 2.6 ISO-TP extraction (enabler)

`src/m2/isotp.h/.cpp` gained `isotp::transmit(ep, data, len)` returning
`TxResult {OK, SEND_FAIL, FC_TIMEOUT}` — the transmit half extracted from
`requestMulti()` (which now calls the shared `txPayload()`; behavior
preserved). STPX uses the transmit-only path because it prints *raw*
response frames instead of reassembling ISO-TP.

---

## 3. SuperB web GUI

- **Nav:** MITM link added. **Mode page:** STN (value 2) and MITM (value 4)
  buttons enabled; `handleModeSet` allow-list extended to 0–5.
- **`/mitm` page:** bridge status (active/staged), per-direction stats
  table, loop-suppressed / rate-limited / bus-load / direction / stealth
  readouts, rule table (delete per row + clear-all), add-rule form
  (id/mask hex, dir, action, modify params, rate limit), bridge config form
  (baudA/baudB selects, direction, stealth toggle, **experimental**
  loop-suppression window + default rate limit — labeled as such), and a
  live LOG-frame view.
- **`/api/mitm`:** JSON `{ok, fwdAB, fwdBA, dropAB, dropBA, modAB, modBA,
  loop, rate, loadA, loadB, rules:[[idx,id,mask,dir,action,hits]…],
  log:[…]}`; the page polls it every 1 s (same pattern as `/api/obd2`).
- POST routes: `/mitm_rule_add`, `/mitm_rule_del` (idx=255 clears all),
  `/mitm_cfg`. All validate args server-side and report
  `ST_BAD_ARG`/rejection honestly.
- **MITMLOG sniffing:** `pumpBeeRx()` feeds Bee bytes into a 24-line ring
  when the cached M2 mode is 4 (mode-gated like the OBD2 CSV ring). Only
  lines starting with `MITMLOG,` are stored.
- Config form **prefills from the M2** via the new CMD_MITM_CFG read path
  (no guessed state).

---

## 4. Resource ownership (Phase 4)

- **MITM claims CAN0 + CAN1 + UART0/Bee** — begun at mode entry, released
  in `mitm::exit()` via `modes::canEnd0/1()`. `quiesceBuses()` needed **no
  change**: it already handles both native controllers through the
  per-controller begun flags (the Phase 3 pin-safety refactor). Verified by
  reading `quiesceBuses()`, not by changing it.
- **STN claims exactly what ELM claims** (CAN0 lazy or the J1850 block +
  UART0/Bee) — same code path, no new ownership.
- No new TC channels, no PWM, no SPI, no PIO interrupts, no DAC in either
  mode. The TC6/TC7 and PWM6 notes from Phase 2 stand unchanged.
- `docs/RESOURCE_OWNERSHIP.md` gained a "Phase 4" section; the mode
...[truncated 5165 chars]