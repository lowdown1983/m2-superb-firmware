# Phase 3 Notes — UDS/diagnostics mode + OBD2 poller + web GUI extensions

No bench exists, so everything below is **compile-verified only**. Nothing has
been flashed; no bus, button, LED, radio, or web behavior has been observed on
hardware. Evidence labels throughout: **VERIFIED** = compiles and/or sourced
from a primary source; **INFERRED** = assumed but untestable here.

Standards cited (public): **ISO 14229-1** (UDS service/NRC semantics),
**ISO 15765-2** (ISO-TP transport — already the basis of `src/m2/isotp.*`),
**SAE J1979** (OBD-II PID formulas), **SAE J2012** (DTC text encoding). No
standard content is invented; NRC/DTC/PID meanings come from these documents.

## 1. UDS mode (`src/m2/mode_uds.*`)

**Transport:** CAN ISO-TP only, 11-bit (Phase 3 decision). Reuses the Phase 2
`isotp` module unchanged — `requestMulti()` with functional-addressing
multi-ECU reassembly. No J1850 diagnostics in this phase.

**Services** (console: `uds <svcHex> [dataHex...]` on SerialUSB; web: UDS page):
`0x10` session control (01 default / 02 programming / 03 extended),
`0x11` ECU reset (01 hard / 02 keyOffOn / 03 soft) — **arm-confirm**,
`0x14` clear DTC (bare `uds 14` defaults group to `FFFFFF` per ISO 14229-1),
`0x19` read DTC (`19 02 FF` = by status mask; `19 04 …` = snapshot ID),
`0x22` read data by identifier (e.g. `22 F190` = VIN),
`0x2E` write data by identifier — **arm-confirm + warning log**,
`0x27` security access — **stub** (see §3),
`0x28` communication control, `0x3E` tester present, `0x85` control DTC setting.
Any other service byte is sent raw — the console is a generic UDS terminal.

**Arm-confirm (0x11, 0x2E):** first invocation prints a warning and arms for
10 s; repeating the *identical* command executes. Never automatic. The web
GUI enforces its own confirm checkbox for these two services before sending.

**Negative responses:** `7F <svc> <NRC>` is decoded to text from the ISO
14229-1 NRC table (`uds::nrcText()`; the SuperB mirrors it for the web page).
Unlisted NRC values print as "unknown NRC" — never invented.

**Addressing:** physical (default TX `0x7E0` / RX `0x7E8`, configurable) or
functional `0x7DF` (all ECUs, up to 4 responses collected). Console:
`uds bus 0|1`, `uds baud <n>` (allow-listed), `uds tx|rx <hex>`,
`uds func|phys`. Web: same via `/uds_cfg`.

**Session state machine:** a positive `0x10` response adopts the requested
session. Tester-present (`3E 00`) keepalive is off by default; `uds tp on`
/ `uds tp <ms>` / web toggle enables it (default 2000 ms, INFERRED). Without
TP traffic the session drops to default (`0x01`) after 5000 ms (INFERRED,
per ISO 14229 S3_server) with a console note.

**Bus lifecycle:** entering UDS mode touches no bus (lazy acquire on the
first command — the car sees nothing until then). `exit()` releases the CAN
bus before the manager's `quiesceBuses()`. Config commands work any time
(staged); bus/baud changes drop the bus for lazy re-acquire.

## 2. Seed/key stub rationale (§3 of the task)

`0x27` passes requestSeed/sendKey through and prints the seed, but **no key
algorithm is implemented and none ever will be invented here**: OEM
seed/key algorithms are proprietary per-ECU secrets. The stub is an explicit
extension point (`uds 27 01` → read seed; `uds 27 02 <key>` → send a
user-supplied key, e.g. computed offline). Documented in the console help,
the web page, and the code header.

## 3. OBD2 poller mode (`src/m2/mode_obd2.*`)

Standalone SAE J1979 poller — no phone needed. `obd2 start` (console / web /
mm channel) begins auto-detect (CAN 11-bit **500k then 250k**, INFERRED order;
retries every 5 s until an ECU answers `0100`), then sweeps the PID list
every `intervalMs` (default 1000 ms, 100–60000).

**Default PIDs:** `010C` RPM, `010D` speed, `0105` ECT, `010F` IAT,
`0111` TPS, `0142` control-module voltage. Decoders for the common PIDs
(SAE J1979 formulas, integer math — no float on the SAM3X); anything else
falls back to concatenated uppercase hex.

**CSV format** (Bee link → bridged to TCP:23/BT as a text stream):
`ts_ms,mode,pid,value` — e.g. `123456,01,0C,850` or `123457,01,42,12.4`.
`ts_ms` = `millis()`; `mode` is always `01` in this phase; `pid` is 2 hex
digits; `value` is the decoded quantity (one decimal where the formula needs
it) or raw hex when no decoder exists.

**DTC scan:** `obd2 dtc` sends mode `03` and decodes to `P0xxx` text
(SAE J2012). Works without the poller running (lazy bus acquire).

**Bus lifecycle:** same lazy discipline — entering touches no bus; `exit()`
stops the poller and releases the CAN bus.

## 4. Mode-manager changes (Phase 3)

- `modeImplemented()`: UDS + OBD2 now live (button cycles M2RET/ELM/UDS/OBD2).
- **Pin-safety refactor (correctness fix):** the single `canBegun` latch became
  per-controller `can0Begun`/`can1Begun`. Rationale: UDS/OBD2 can use CAN1
  while CAN0 was never begun — the old global flag would have authorized
  `Can0.disable()` on a never-begun controller, driving Arduino 50
  (J1850_PWM_VPW) LOW via due_can's default enable pin. `quiesceBuses()`,
  `exitM2RET()`, `mmSetCanBaud()`, and the `canBegin0/1`/`canEnd0/1`
  primitives are all per-controller now. VERIFIED from due_can 2.1.6 source
  (constructor defaults 50/48; `disable()` drives enablePin when ≠ 255).
- New primitives `modes::canBegin1()`/`canEnd1()` (same 255 rule as CAN0).
- Console routes `uds ...` / `obd2 ...` (config any time; bus ops need the
  mode active).
- mm ops extended: `udsTx/udsCfg/udsStatus`, `obd2Cfg/obd2Ctl/obd2Status`.

## 5. mm control-channel additions (Phase 3)

`CMD_UDS_TX (0x06)`, `CMD_UDS_CFG (0x07)`, `CMD_OBD2_CFG (0x08)`,
`CMD_OBD2_CTL (0x09)`, `CMD_UDS_STATUS (0x0A)`, `CMD_OBD2_STATUS (0x0B)` —
same `F1 7E LEN PAYLOAD XOR` framing; see `mm_channel.h` for exact layouts.
Notes: `UDS_TX` returns `ST_ERR` when UDS mode isn't active (no implicit bus
touch); the first response is returned inline (truncated to fit 48 bytes —
documented) with the total ECU count; `OBD2_CFG` stops a running poller.

## 6. SuperB web GUI (Phase 3)

New pages: **`/uds`** (service picker + params hex, raw response + NRC
decode, session buttons, tester-present toggle, addressing/bus config —
0x11/0x2E gated by a confirm checkbox) and **`/obd2`** (PID list editor,
interval/bus/baud, start/stop, live CSV view auto-refreshed from
`/api/obd2`). The SuperB sniffs Bee-UART lines into a 24-line ring buffer
**only while the M2 reports OBD2 mode** (mode cached from status fetches —
INFERRED timing) and serves them as JSON. Mode page now offers UDS + OBD2.
`/api/obd2` added alongside `/api/status`.

## 7. INFERRED list (needs hardware/vehicle)

- All UDS/OBD2 bus behavior: ISO-TP against real ECUs, session handling,
  tester-present cadence, multi-ECU reassembly, 500k→250k detect order,
  5 s detect retry, 5 s session timeout, 2000 ms default TP interval.
- DTC decode and PID formulas are standard-published but untested here.
- SuperB: CSV sniff gating on the cached mode byte; `/api/obd2` polling
  cadence; NRC mirror table (same source as the M2's).
- Still open from earlier phases: J1850 timings/polarity, `setupSW(33333)`
  live capture (HW-055), CHIPID, SuperB flash size (4 MB assumed), button
  feel, AP password strength.

## 8. Deferred to Phase 5 (programming mode)

Routine control (`0x31`), request/transfer/exit (`0x34`/`0x36`/`0x37`),
and any actual flashing/tuning routine. The `0x10 0x02` programming-session
button exists on the UDS page (it only *requests* the session — harmless
without following it with transfer services), but no download/upload
sequences ship in Phase 3. The Phase 5 safety review (confirm gates, voltage
checks, no-bus-traffic interlocks) applies before any of that lands.

## Files changed/added (Phase 3)

- `src/m2/mode_uds.h/.cpp` — NEW: UDS mode.
- `src/m2/mode_obd2.h/.cpp` — NEW: OBD2 poller.
- `src/m2/mode_manager.h/.cpp` — per-bus begun flags; UDS/OBD2 wiring;
  `canBegin1/canEnd1`; console routing; mm ops.
- `src/m2/mm_channel.h/.cpp` — CMD 0x06–0x0B + dispatch.
- `src/superb/main.cpp` — UDS/OBD2 pages, `/api/obd2`, CSV ring, routes, nav.
- `docs/RESOURCE_OWNERSHIP.md` — Phase 3 claims note (below).
- `docs/PHASE3_NOTES.md` — this file.
- `SETUP.md` — Phase 3 usage section.
