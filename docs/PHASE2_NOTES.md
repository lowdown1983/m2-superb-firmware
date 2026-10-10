# Phase 2 Notes — ELM327 mode + SuperB web GUI

No bench exists, so everything below is **compile-verified only**. Nothing has
been flashed; no bus, button, LED, radio, or web behavior has been observed on
hardware. Evidence labels throughout: **VERIFIED** = compiles and/or sourced
from a primary source; **INFERRED** = assumed but untestable here.

## 1. Mode-switch command channel (design decision)

**Chosen: in-band binary channel, `0xF1 0x7E LEN PAYLOAD XOR`, in ALL modes.**

The Phase 1 blocker was "no safe in-band channel over GVRET traffic". The
resolution, verified against M2RET's own parser:

- `0xF1` is the GVRET sync byte; the command-byte position only defines 0–14
  and `0xE7` (`M2RET.h` enum `GVRET_PROTOCOL` @ ad9d5af; `CommProtocol.txt`).
  SavvyCAN's connect sequence is `E7 E7 F1 0C F1 06 F1 07 F1 01 F1 09` — it
  never emits `0x7E`, and no legitimate AT text contains `0xF1`.
- Our GVRET parser resyncs to IDLE on unknown command bytes
  (`mode_m2ret.cpp` GET_COMMAND `default`), and `0x7E` enters a
  **length-prefixed** sub-state (`PState::MM_CTRL`), so payload bytes can
  never be mistaken for commands.
- In ELM mode the byte pump diverts `F1 7E` before line assembly
  (`mode_elm.cpp` `pumpByte()`); a stray lone `0xF1` is dropped.
- The SuperB **filters app→M2 traffic**: any `F1 7E` pair from BT/TCP is
  dropped (1-byte hold-back, flushed on disconnect or 50 ms quiet), so a
  connected app can never inject a control frame.
- The SuperB **diverts M2→app traffic**: `F1 7E` replies are extracted and
  routed to the pending web-GUI request, never broadcast to apps.

**Rejected alternative:** a spare Bee-socket GPIO as an attention line. The
schematic text extraction was layout-garbled and no clean, verified-free GPIO
path was established; the in-band channel needs no wiring change and works
on existing hardware.

**Safety invariant:** `CMD_SET_MODE` dispatches exclusively through
`ModeManager::requestMode()` — quiesce all buses, then acquire. There is no
backdoor. `GET_STATUS` is read-only. `CMD_SET_CAN_BAUD` / `CMD_SET_SWCAN_SPEED`
validate against allow-lists and apply live only in M2RET mode (staged
otherwise, reported in the reply). `CMD_SET_FEATURE` gates `FEAT_J1850`
(J1850 rail + ELM protocols 1/2) and `FEAT_SWCAN`.

**Wire format** (`src/m2/mm_channel.h`): requests `F1 7E LEN [CMD ARGS...]
XOR`; replies `F1 7E LEN [CMD|0x80 STATUS DATA...] XOR`. Status: 0=OK,
1=ERR, 2=UNKNOWN_CMD, 3=BAD_ARG. XOR = XOR of payload bytes (GVRET
`checksumCalc` style, M2RET.ino:418-427).

## 2. ELM327 mode (`src/m2/mode_elm.*`)

A genuine-protocol implementation (not an emulator): real AT response
formats, real bus transactions, `NO DATA` on timeout.

**AT subset** (the commands Torque/FORScan actually use):
`ATZ ATI AT@1 ATE0/1 ATL0/1 ATS0/1 ATH0/1 ATSP0–9/A/B/C ATPB ATDP ATDPN ATRV
ATMA ATMR ATMT ATCAF0/1 ATCFC0/1 ATST ATAT0/1/2 ATTP ATBI ATPC ATWS ATAL ATNL
ATSH ATPP ATD ATM` — plus `?` for anything unknown (genuine ELM327 behavior).

Response formats follow genuine ELM327 conventions: `>` prompt, `OK`, `?`,
`NO DATA`, `STOPPED` (any received char aborts `ATMA`; control-channel bytes
are diverted first so a GUI click mid-monitor doesn't kill it). Power-on
defaults match ELM327 (echo on, spaces on, headers/linefeeds off, ST=32).

**Notes/honest deviations:**
- `AT@1` returns `M2-SuperB` (not a clone of someone's ID string).
- `ATRV` returns `12.4V` — INFERRED placeholder; no calibrated ADC scaling
  exists yet (audit open item: current-sense constants).
- `ATRD` returns `?` — not verified against the ELM327 datasheet and unused
  by Torque/FORScan.
- `ATDPN` with auto returns the set value (`0`) or the resolved protocol —
  INFERRED.
- `ATMR`/`ATMT hh` filter by CAN ID (11/29-bit) or J1850 first header byte —
  INFERRED simplification of the genuine header-byte semantics.
- `ATSPB` = CAN 11/125k, `ATSPC` = CAN 11/100k (documented assignment; on
  genuine chips B/C are user-defined).

**Protocols:** 1=J1850 PWM, 2=J1850 VPW, 6/7/8/9=ISO 15765-4 CAN
(11/29-bit, 500/250k), B/C as above. `ATSP0` auto-detects by trying `0100`
in the order **6, 8, 7, 9, 2, 1** — INFERRED (ELM does not specify an order).

**OBD-II:** service 01 PIDs (whatever the ECU answers), 03/07 DTC (raw hex;
decoding is app-side), 04 clear, 09 multi-frame (exercises ISO-TP RX). CAN
uses functional addressing (`0x7DF` / `0x18DB33F1`) and collects up to 4
ECU responses; J1850 uses the 3-byte header (default `68 6A F1`,
`ATSH`-overridable).

**Bus lifecycle:** entering ELM mode touches no bus (lazy acquire on first
protocol use — the car sees nothing until the first ATSP/OBD command).
`exit()` releases CAN0/J1850 before the manager's `quiesceBuses()`.

## 3. J1850 driver (`src/m2/j1850.*`) — INFERRED throughout

**Reference decision:** `matafonoff/J1850-VPW-Arduino-Transceiver-Library`
(Ms-PL, vehicle-tested on a Chrysler Pacifica) was evaluated and **not**
vendored: its interrupt layer is AVR-specific (`PCMSK` registers) and it is
VPW-only. The driver below is M2-specific and cites that library's timings.

**Timing sources (nothing invented):**
- VPW: the matafonoff library — TX SOF 200µs, short 64µs, long 128µs, EOD
  200µs, EOF 280µs; RX windows SOF 163–239µs, short 34–96µs, long 97–163µs,
  EOD 164–239µs; bit rule active-long/passive-short = 1.
- PWM: SparkFun "OBD-II Getting Started" ('1' = 8µs, '0' = 16µs, SOF = 48µs
  @ 41.6kbaud) + retrolib BAE 5030 (fixed bit time, long dominates short).
  PWM EOD/EOF/IFS (200/280/300µs) and all PWM RX windows are INFERRED.
- CRC-8/J1850: the library's `crc()` (init 0xFF, poly 0x1D, complement).

**RX method:** GPIO pin-change interrupts + `micros()` (the proven
matafonoff approach). The TC6/TC7 capture channels are deliberately NOT
used: register-level TC code cannot be validated without a bench, and a
subtle TC bug would be worse than the known interrupt-latency cost. PWM 8µs
symbols sit near the SAM3X interrupt-latency floor — flagged as the
highest-risk INFERRED item; a vehicle test decides whether TC capture is
needed (the pins are already routed for it).

**Polarity:** RX idle level is sampled at `acquire()` (bus idles passive), so
RX adapts to the transceiver. TX polarity (pin HIGH = bus active) is
INFERRED — the bench must confirm the transceiver doesn't invert.

**What the driver does:** `acquire()` powers the rail (`PS_J1850_9141` HIGH),
selects PWM/VPW, parks TX, attaches the RX ISR; `transact()` bit-bangs a
frame (VPW with polled arbitration, interrupts off) and waits for a
CRC-valid response; `release()` is idempotent (safe for `quiesceBuses()`).
Received frames queue 4-deep for `ATMA` monitoring.

## 4. ISO-TP (`src/m2/isotp.*`) — shared module

Blocking `requestMulti()`: single-frame, first-frame + flow-control +
consecutive-frame TX; single/multi-frame RX with FC-CTS after FF; reassembly
keyed by sender CAN ID (functional addressing interleaves ECUs); SN
validation; N_Bs/N_Cr = 1000 ms (standard defaults, INFERRED tuning).
Phase 3 (UDS) reuses this unchanged.

## 5. SuperB web GUI (`src/superb/main.cpp`)

HTTP on port 80 (AP+STA from Phase 1, kept). Hand-written HTML, no
framework. Routes: `/` (status), `/mode` + `/mode_set` (POST, confirm
checkbox — switching quiesces buses; warning shown), `/settings` +
`/settings_set` (CAN0/1 baud, SWCAN speed, J1850/SWCAN toggles),
`/wifi` + `/wifi_set` (AP password, STA SSID/pass → NVS → restart),
`/api/status` (JSON).

**Phase 1 TODOs closed:** AP is now WPA2 (password in NVS, default
`M2SuperB!` — change it on first use); STA credentials come from NVS via
`/wifi` (no more placeholders); empty STA SSID = don't join.

**Bridging kept:** BT SPP + TCP:23 byte pipes, last-writer-wins, oldest
evicted when full — plus the TX injection filter and RX diverter (§1).

## 6. INFERRED list (needs hardware/vehicle)

- Everything in §3 (J1850): timings, TX polarity, RX windows, arbitration,
  rail settle time, interrupt latency for PWM.
- `ATSP0` auto-detect order; `ATDPN` auto value; `ATMR`/`ATMT` filter
  semantics; `ATRV` voltage; multi-ECU quiet period (200 ms).
- SuperB: TX-filter 50 ms flush timing; web-handler 800 ms M2 timeout;
  `M2SuperB!` default AP password strength (change it).
- `mm` SET_CAN_BAUD live-apply in M2RET re-opens filters via
  `setPromiscuousMode()` — compile-verified, behavior untested.
- Still open from earlier: `setupSW(33333)` live capture (HW-055), CHIPID,
  SuperB flash size (4 MB assumed), DS2/LED polarities, button feel.

## 7. Torque connection walkthrough (BT SPP, ELM mode — once flashed)

1. Flash M2 and SuperB; seat SuperB (antenna toward M2 USB).
2. Join WiFi AP `M2-SuperB` (password `M2SuperB!` unless changed); open
   `http://192.168.4.1/` → Status shows M2 mode.
3. Mode page → ELM327 → confirm. (Or press M2 Button1 once.)
4. Phone: pair Bluetooth `M2-SuperB`.
5. Torque → Settings → OBD2 adapter → Bluetooth → `M2-SuperB`. Torque sends
   `ATZ ATE0 ATL0 ATS0 ATH0 ATSP0 0100…`; the M2 auto-detects the protocol
   and answers with real bus data.
6. FORScan over BT SPP works the same way (it speaks ELM AT + CAN-TP).
7. SavvyCAN still uses M2RET mode over TCP:23 or BT (unchanged from Phase 1).

## Files changed/added (Phase 2)

- `src/m2/mm_channel.h/.cpp` — NEW: control-channel codec + dispatch.
- `src/m2/isotp.h/.cpp` — NEW: shared ISO 15765-2.
- `src/m2/j1850.h/.cpp` — NEW: J1850 VPW/PWM driver.
- `src/m2/mode_elm.h/.cpp` — NEW: ELM327 mode.
- `src/m2/mode_manager.h/.cpp` — ELM implemented; mm ops; settings
  (CAN0/1 baud, SWCAN speed, features); `quiesceBuses()` releases J1850;
  button cycles M2RET↔ELM327.
- `src/m2/mode_m2ret.h/.cpp` — `0x7E` divert (`PState::MM_CTRL`);
  `swcanSetSpeed()` for live speed changes.
- `src/superb/main.cpp` — web GUI, WPA2+NVS, TX filter, RX diverter.
- `docs/RESOURCE_OWNERSHIP.md` — Phase 2 J1850 claims (GPIO-IRQ, not TC).
- `docs/PHASE2_NOTES.md` — this file.
- `SETUP.md` — Phase 2 usage section (see below).
