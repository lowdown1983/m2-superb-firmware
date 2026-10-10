# Phase 12 — Profiles, auto-detect, logging, test harness (2026-10-08)

The LAST phase of the approved roadmap (`docs/ROADMAP_PHASES_6_PLUS.md`).
Six work items: (1) versioned, CRC-protected vehicle profiles in
flash-page emulated EEPROM; (2) an explicitly user-initiated
multi-protocol auto-detect engine with layered transmit gates;
(3) diagnostic session logging (M2 produces → Bee → SuperB persists to
SPIFFS); (4) a virtual-ECU self-test harness on the CAN0↔CAN1 loopback
pair — the first real validation path for the INFERRED CAN behaviors;
(5) a DBC/custom-PID signal engine fed by build-time extraction from
the OEM database's 45 curated DBCs; (6) SLCAN/Lawicel ASCII transport +
M2RET modernization commands.

Evidence labels: **VERIFIED** = compiles / cited from a primary source.
**INFERRED** = not tested on hardware. **HARNESS-VALIDATED** (new label,
defined §8) = exercised against the loopback harness on real silicon —
stronger than INFERRED, weaker than VEHICLE-VALIDATED.

## 1. Vehicle profiles (`flashstore.*`, `profiles.*`)

The SAM3X8E has **no real EEPROM** and the Arduino SAM core ships no
EEPROM emulation — so this phase implements the honest equivalent and
documents it instead of pretending:

- **Backing store:** the last two flash pages of IFLASH1 (EFC1):
  slot A at `0x000FFC00`, slot B at `0x000FFE00` (512 bytes each).
  Saves go to the *inactive* slot via EEFC "Erase Page and Write Page"
  (EWP) with IRQs masked (the EFC stalls the bus while FRDY is low);
  the slot becomes active only after a read-back + CRC32 verify.
  A power loss mid-save can only strand the inactive slot — the
  previous record stays valid. Header: magic `PRFS`, format version,
  payload length, monotonic save counter (newest-wins + wear
  accounting), CRC32.
- **Wear:** Atmel rates embedded-flash P/E endurance in the low tens
  of thousands of cycles (INFERRED — datasheet figure not re-verified
  this phase). Mitigations: saves are **explicit only** (`profile
  save`, never background), A/B halves the per-save cost to one page
  cycle, `flashstore::writes()` exposes the counter.
- **Size limit:** 496 bytes payload per record. Firmware must stay
  below `0x000FFC00` (510 KB); we are at ~223 KB — checked at build
  time, not by a linker assertion (documented limitation).
- **Sim builds:** `#ifdef SIM_M2` stubs the EFC out — profiles stay
  RAM-only on the host (the host has no EFC).

One persisted profile round-trips the Phase 6/9/10 knobs: CAN0/CAN1
baud + SWCAN speed (staged via the manager's `mmSetCanBaud`/
`mmSetSwcanSpeed`, i.e. the same applied-vs-staged semantics as the
web GUI), protocol/frame-format/diag-address, the 15-byte ISO-TP
default blob (`CMD_ISOTP_CFG` layout), up to 8 busfilter entries
(20-byte packed form), up to 8 gateway mappings (21-byte form), the
capgate transmit-permission ceiling (applied through the normal
upward-only escalation — never silent, never downward), log mode, and
retry limit. `profile save <name> | load | show | clear` on the
console; `CMD_PROFILE_SAVE/LOAD/INFO` (0x31–0x33) on the mm channel;
web UI on `/phase12`.

Design note: gateway MODIFY mappings need arm-confirm on the mm path;
a profile `load` is itself an explicit user action, but the arm is
the second factor — MODIFY entries stage on the first load and arm on
the second identical load. Documented in `profiles.cpp`, not hidden.

## 2. Auto-detect engine (`autodet.*`)

Ordered probe: **CAN** (125k/250k/500k/1M × 11/29-bit, **listen-only**
via due_can `setListenOnlyMode`) → **K-line** (5-baud/fast init,
reusing the Phase 7 `kline::probe()` stub) → **J1850** (VPW/PWM,
passive observation) → **SWCAN** (normal, passive observation).
Looks for valid frames, known diagnostic IDs (0x7DF, 0x7E8–0x7EF from
the generic SAE row), and reports a confidence score.

**Safety-critical design** (SAFETY_REVIEW.md §14.1, reasoning
summarized here): the engine is **explicitly user-initiated only**
(`autodetect start` / `CMD_AUTODET_START` 0x34 — never on boot, never
background, never from another mode). The whole engine requires the
Phase 9 capgate `ALLOW_TRANSMIT` up front — even the listen-only CAN
sweep. The transmitting stages (K-line init, active J1850/SWCAN
probing) need a **second explicit consent** (`tx` word / `allowTx`
byte, rendered as a separate checkbox on the web UI). The engine
takes over the buses (`quiesceBuses()` first, so no mode loop fights
it), runs blocking (~3 s worst case), then re-enters the current mode
(new `ModeManager::reacquire()` — exit + quiesce + enter, grant
resets like a normal switch).

J1850/SWCAN stages are passive in this phase (power the block, count
valid frames via the monitor paths — no transmit); the K-line stage
calls the Phase 7 stub and reports honestly that there is no
implementation behind it yet.

## 3. Diagnostic session logging (`logrec.*` + SuperB sink)

The M2 has no SD card, so the sane path is the one built: the M2
produces compact **23-byte records** (tsMs, bus, dir, kind
CAN/OBD/UDS/DBC, ecuAddr, svc, decoded value, raw bytes) into a
48-record RAM ring (1,104 B) with **triggers** (kind/svc match),
**pre/post-trigger buffers** (default keep-16, then N post, then
auto-stop), **rotation** (ring overwrite), and **loss counters**
(`produced/dropped/streamed/pending` — overwrites are never silent).

The SuperB **polls** (`CMD_LOG_POLL` 0x2F, 2 records per 48-byte
reply, records consumed on read) and persists to **SPIFFS** with
rotation: two 64 KB ping-pong files (`/logs/log_00.bin`,
`/logs/log_01.bin`) fitting the 192 KB SPIFFS partition the Phase 11
partition table reserved "for future" — this phase is that future.
Poll misses and SPIFFS write failures are counted separately.
Native binary is stored; **GVRET** (0xF1 binary frames, M2RET
`sendFrameToUSB` shape — VERIFIED against `mode_m2ret.cpp`), **CRTD**
(SavvyCAN text), and **CSV** are converted at download time on
`/logs`.

**Privacy:** logs may contain VINs — they stay local by construction
(SPIFFS + web download only; no upload path exists). Stated on the
web UI and in §14.3.

Known record limitations (documented, not hidden): kind-0 records
carry the CAN ID's **low 16 bits** in `svc` and `ecuAddr` is 8-bit —
full 29-bit IDs and 11-bit ECU addresses are truncated. Full-ID
capture is a future refinement.

## 4. Virtual ECU / self-test harness (`vharness.*`, HARNESS mode)

The M2 talks to itself: **CAN0↔CAN1 loopback** with the module as the
scripted ECU. Physical requirement: wire CAN0H↔CAN1H and CAN0L↔CAN1L
(a short twisted pair; termination per the M2 hardware docs —
INFERRED wiring detail).

- `selfcheck()` sends marker frames both directions and requires both
  to arrive. `runSuite()` **refuses** without a passing selfcheck —
  a vehicle bus will not echo our markers, so the suite cannot start
  against one (heuristic gate, documented as such).
- 8 scripted tests: OBD 010C single-frame, 0902 VIN 20-byte ISO-TP
  reassembly, 7F 22 31 negative response, 10 ms-spaced CFs (STmin),
  stalled CF stream → N_Cr timeout (**negative test**: pass = the
  timeout fires), bad sequence number (**negative test**: pass = the
  error is caught), 200-frame burst overload (no lockup), UDS
  22 F190 scripted DID.
- The ECU simulator answers only the scripted test IDs (0x7E0) with
  raw crafted FF/CF sequences (full fault-injection control); it never
  initiates traffic.

**This is the first thing that can actually validate the INFERRED CAN
behaviors** — ISO-TP reassembly, timeouts, and error handling get
their first real (non-simulated) exercise here. Outcomes from a real
loopback run may be labeled **HARNESS-VALIDATED** (stronger than
INFERRED, weaker than VEHICLE-VALIDATED). No loopback run happened in
this phase (no hardware on this VM) — the suite is compile-verified
and ready for the bench.

Console: `harness selfcheck | run [mask-hex] | status`; mm:
`CMD_HARNESS_RUN` 0x36 / `CMD_HARNESS_STATUS` 0x37; web UI on
`/phase12` (operator role; refusal message explains the wiring).

## 5. DBC / custom-PID engine (`scripts/dbc_extract.py`, `dbcsig.*`)

No DBC text parser in firmware (too heavy for the SAM3X). The OEM
database's 45 curated DBCs are consumed **at build time only**, in the
Phase 9 `oem_extract.py` pattern:

- `scripts/dbc_extract.py` (new): parses DBCs with `cantools`
  (auto-installed into `~/.dbc-extract-venv`), extracts messages,
  signals (start bit, length, Motorola/Intel, signedness, factor,
  offset, unit, multiplexer/muxVal, enum value tables), and emits
  `src/m2/dbc_gen.h` — compact `constexpr` tables in flash.
  `--dbc <name>` is repeatable (45 DBCs available); default set is a
  lean starter (Ford/GM/Mazda/Toyota powertrain, ~67 KB flash — the
  full 45-DBC set is ~700 KB and will NOT fit in 512 KB).
- `dbcsig::decode()` extracts signals with correct DBC bit numbering
  (Motorola sawtooth walk from the MSB; Intel linear from the LSB),
  sign extension, integer-only scaling (`phys1000`), one-level
  multiplexing, and enum descriptions — driving console (`dbc list |
  info | decode`), logging (KIND_DBC), and the mm channel
  (`CMD_DBC_INFO` 0x38 / `CMD_DBC_DECODE` 0x39).
- 5 Subaru files are absent from the OEM clone (pre-existing gap);
  the script warns and continues.

## 6. SLCAN + M2RET modernization

- **SLCAN mode** (`mode_slcan.*`, 10th mode): Lawicel ASCII on the Bee
  UART — `O/C/S0-8/t/T/r/R/F/V/N/Z0-1/M/m/P/A`, CR-terminated, `z` =
  ok / BEL = error. RX frames stream as `t/T` lines (optional
  timestamp); TX honors the capgate. Channel opens lazily on `O`
  (acquire-on-entry for the *mode*, lazy acquire for the *channel* —
  documented in the resource table). Follows the existing mode pattern
  (begin/loop/exit + `quiesceBuses()` on switch).
- **M2RET modernization** (`m2ret::console`): `CAPABILITIES`,
  `GET_VERSION`, `GET_BUSES`, `GET_BUS_STATUS`, `SET_BUS_CONFIG`,
  `SET_FILTER`, `CLEAR_FILTER`, `GET_STATS`, `RESET_STATS`, `LOG
  START|STOP|STATUS|CLEAR`, `SET_SECURITY_LEVEL` (alias for the Phase 9
  grant escalation — upward-only, mode ceiling enforced). The
  human-console face of the mm supervision channel; same underlying
  ops.

New mm commands 0x2F–0x39 (log poll/ctl, profile save/load/info,
autodet start/status, harness run/status, DBC info/decode) follow the
F1 7E framing / `CMD|0x80` / LE payload supervision template;
`docs/OVERVIEW.md` resource table gains the SLCAN + HARNESS rows;
`docs/RESOURCE_OWNERSHIP.md` timer/power map gains the Phase 12 rows
(no new timers/IRQs claimed — everything is millis()-based or reuses
existing drivers).

## 7. SuperB side (kept lean)

`src/superb/main.cpp` only: the `p12::` log sink (poll + SPIFFS
rotation + loss counters), `/phase12` (profiles, auto-detect with TX
consent checkbox, harness, DBC info, sink on/off), `/logs` (file
list + four-format downloads), nine routes, one nav addition. New
`src/sim_shim_esp/SPIFFS.h` host shim (backed by `/tmp/m2sb_spiffs/`)
keeps `sim_superb` building — the real ESP32 core's SPIFFS.h shadows
it on hardware. Flash: 85.5% → **87.9%** (+2.4pp ≈ 47 KB). Still
inside budget, but the SuperB has no room left for another
UI-heavy phase — future work there should be download/API shaped, not
page shaped.

## 8. Evidence labels and the INFERRED list

New label: **HARNESS-VALIDATED** = exercised against the loopback
harness on real silicon. Stronger than INFERRED (not a guess about
the code path — the path ran) but weaker than VEHICLE-VALIDATED (the
peer was our own scripted ECU, not a vehicle). The label applies to
CAN behaviors only (ISO-TP, timeouts, framing); nothing else in this
phase qualifies yet.

INFERRED (untested on hardware):
- EFC flash-write sequence (read-back + CRC verify in code, but no
  write exercised on silicon yet); flash endurance figure.
- Auto-detect on every stage (CAN sweep, J1850/SWCAN passive, K-line
  stub); confidence heuristics.
- Log sink at vehicle data rates (poll timing vs ring overwrite).
- Harness suite on a real loopback pair (compile-verified only).
- DBC decode values against real OEM frames (bit-extraction logic is
  the standard algorithm, but no vehicle frame has been decoded).
- SLCAN interop with real Lawicel apps.
- SPIFFS rotation under sustained logging.

VERIFIED: all three envs compile clean; pin-safety `begin(baud,255)`
holds by grep; `src/m2/debug.h` and `src/m2/main.cpp` untouched;
0x27 still a stub; J1939 untouched; sim_superb serves the new pages
and all four download formats (exercised with synthetic records:
CSV/CRTD/GVRET/native byte-verified).

## 9. Deferred (decisions, unchanged)

- **SAE J1939 stays parked** (decision #5, Appendix C) — not
  implemented in Phase 12 either; the decision is Marcus's.
- ChatGPT F4 (F1 7E framing) stays deferred; Phase 10b LIN master
  stays unapproved; 0x27 stays a stub forever; BLE GATT stays
  deferred.
- K-line `probe()` remains the Phase 7 stub — the auto-detect stage
  ordering exists, the implementation does not.
- The OEM database is consumed at build time only, never embedded
  whole (the 6.5 MB `vehicles.json` stays out of flash).

## 10. Build results (2026-10-08)

- `pio run -e m2`: **SUCCESS**, zero warnings from new/changed files
  (MCP2515/USBCore noise is pre-existing). Flash **43.6%**
  (228,656 / 524,288 B) — was 26.8%; the DBC tables (~67 KB) dominate
  the growth. Slim with `python3 scripts/dbc_extract.py --dbc <name>`
  (repeatable) if flash pressure ever matters; firmware stays well
  below the `0x000FFC00` profile slots.
- `pio run -e superb`: **SUCCESS**. Flash **87.9%** (1,728,913 /
  1,966,080 B); RAM 20.2%.
- `pio run -e sim_superb`: **SUCCESS**; live smoke of the sim binary
  (new `/phase12` + `/logs` pages, all four download formats
  byte-verified against synthetic records).
- `pio run -e sim_m2`: still fails pre-existing on `SPI.h`
  (unchanged; `-DSIM_M2` discipline kept where trivial).
- Adversarial review: per AGENTS.md standing practice, a separate
  reviewer pass should audit this phase before the zip goes to Marcus
  (recommended focus: the EFC write sequence, the auto-detect bus
  takeover/reacquire path, the harness refusal gate, and the
  mm-channel reply length accounting).

## 11. Roadmap closeout — what the 7 phases delivered

| Phase | Delivered |
|---|---|
| 6 | Transport completion: ISO-TP configurability, TX scheduler, unified filters, bus stats |
| 7 | K-line / ISO 9141 / KWP2000 stack (+ Phase 12 auto-detect stub hooks) |
| 8 | J1850 + SWCAN production hardening (state machines, arbitration, TH8056, MCP2515 recovery) |
| 9 | Diagnostic depth: PID/DID/RID registries, service modes, discovery engine, capability gates |
| 10 | GATEWAY mode (new, separate): versioned CAN↔SWCAN mapping tables |
| 11 | SuperB as communications appliance: WebSocket, ownership, auth/roles, signed OTA, sleep/wake |
| 12 | Profiles (flash-persisted), auto-detect, session logging, loopback self-test harness, DBC engine, SLCAN, M2RET modernization |

The firmware is now a complete multi-mode diagnostic platform: 10
modes, 7 shared services, versioned persistence, a real regression
path that needs no vehicle, and a SuperB that bridges, serves,
authenticates, updates, sleeps, and logs.

What remains INFERRED/untested: nearly everything vehicle-facing
(CAN behaviors now have the harness path; J1850/SWCAN/K-line have no
bench at all; OTA, sleep/wake, pairing, and the EFC writes are
compile-verified only).

Sensible next step after Phase 12 — **vehicle testing, in this
order**: (1) loopback harness run on the bench (converts the CAN
INFERREDs to HARNESS-VALIDATED, zero vehicle risk); (2) Bolt EV
(CAN-only): auto-detect CAN sweep + discovery + PID poll + a log
capture with SPIFFS download; (3) profile save/load round-trip on the
M2 (first real EFC write — do it with a known-good image on hand);
(4) J1850/SWCAN/K-line need non-Bolt vehicles (GM for SWCAN/VPW,
Ford for PWM, older ISO 9141/KWP2000 for K-line) — all INFERRED until
then; (5) the parked decisions (J1939, F4, 10b LIN, BLE GATT) are
Marcus's call once the core is vehicle-validated.

## 12. Post-phase audit fixes (2026-10-09)

An independent ChatGPT audit of the Phase 12 tree raised three P1s;
each was verified against the code and the original DBC sources before
fixing. Two were real; the third's diagnosis was wrong but exposed a
worse real bug underneath.

**Byte-order inversion (the big one).** The audit's "130 out-of-DLC
signals" claim came from reading the DBC `@` flag as `@0`=Intel. That
is backwards: the DBC standard (Vector) is `@0`=Motorola (big-endian),
`@1`=Intel (little-endian) — VERIFIED with cantools 44.x by decoding an
actual signal (`SteWhlRelInit2_An_Sns 55|16@0` decodes Motorola). Under
the correct convention, 0 of 17,771 signals in the OEM database's DBCs
overflow their message DLC — the DBC sources are fine. BUT our own
`scripts/dbc_extract.py` had the SAME inversion (`bo=="1"` → Motorola),
so every byte-order flag in `dbc_gen.h` was flipped and `dbcsig.cpp`
ran the wrong extraction algorithm on most signals. Fixed the mapping
(`bo=="0"` → Motorola), regenerated `dbc_gen.h` (inventory unchanged:
4 DBCs / 218 msgs / 1637 sigs / 34 enums — only flags changed;
`SteWhlRelInit2_` now carries the Motorola flag, `APM_Counter` `48|8@1`
carries Intel), and corrected the convention note in `dbcsig.h`. Latent
only — DBC decoding was never vehicle-tested — but it would have
produced wrong values on first contact.

**Fail-closed DBC decoder.** `dbcsig::decode()` now skips any signal
whose bit span reaches past the received frame (Intel:
`(startBit+bitLen-1)/8`; Motorola: max byte across the sawtooth walk)
instead of publishing values decoded from missing-bits-read-as-zero.
Per-call skip count via `dbcsig::truncatedSkips()`, surfaced in the
console `dbc decode` output. The multiplexer signal gets the same
treatment (no mux value from a truncated frame).

**`dbc_extract.py` fail-safe.** Missing explicitly-requested `--dbc`
sources (and the default set) now exit 1 instead of warning-and-skipping
(`--dbc all` stays best-effort per its "that has a file" doc); zero
messages/signals emitted is also fatal; the header is written atomically
(temp + rename).

**Scoped SWCAN listen-only in auto-detect.** `swcanListen()` claimed "NO
transmit" but left the MCP2515 in normal mode (link-layer ACKs). It now
sets `swcan::setListenOnly(true)` before `acquire()` and restores false
after `release()` on every exit path, so later M2RET/gateway SWCAN use
is unaffected.

**Auto-detect web timeout.** `handleP12AutodetStart()` now passes an
explicit 8000 ms timeout to `mmcTransact(CMD_AUTODET_START)` (default
800 ms unchanged) — worst case is ~4.2 s of M2-side synchronous sweep
(4 bauds × 2 formats × 350 ms + J1850 2×350 ms + SWCAN 700 ms).

**Web mode allowlist.** `/mode_set` accepts modes 0–9 now (was 0–6);
HARNESS (9) requires an extra `confirm_harness` checkbox with safety
copy stating the CAN0↔CAN1 physical loopback precondition (the M2-side
suite already refuses without a passing loopback self-check).

Deferred, not touched: the capgate passive-scan question, OEM catalog
wiring, and the async auto-detect redesign.
