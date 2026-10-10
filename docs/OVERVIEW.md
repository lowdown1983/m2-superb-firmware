# System Overview — M2 + SuperB Multi-Mode Firmware

`~/workspace/m2-superb-firmware/`. M2 = Macchina M2 (Atmel SAM3X8E, Arduino
framework, `atmelsam`), SuperB = ESP32-WROOM-32D (`espressif32`). One
PlatformIO project, two envs (`m2`, `superb`); `pio run` builds both.
**No bench exists** (2026-10-04 decision) — everything hardware-touching is
**INFERRED** until in-vehicle testing. Compile-verified only.

---

## 1. The eight modes (M2)

| Mode | What it does | Protocols | Console |
|------|--------------|-----------|---------|
| **M2RET** (Phase 1) | GVRET passthrough for SavvyCAN (M2RET-compatible) | CAN0, CAN1, SWCAN | GVRET binary |
| **ELM327** (Phase 2) | AT command set for Torque/FORScan-style apps | CAN 11/29-bit ISO-TP, J1850 PWM/VPW, K-line ISO 9141-2/KWP2000 (Phase 7) | AT commands |
| **STN** (Phase 4) | ELM core + STN11xx command subset | Same as ELM | AT + `ST…` |
| **UDS** (Phase 3) | Diagnostics: sessions, DIDs, DTCs, security stub | CAN 11-bit ISO-TP | `uds …` |
| **MITM** (Phase 4) | Dual-CAN bridge with rule engine | CAN0↔CAN1 | `mitm …` |
| **OBD2** (Phase 3) | Standalone PID poller, CSV stream | CAN 11-bit ISO-TP | `obd2 …` |
| **PROG** (Phase 5) | ECU flashing/tuning over UDS | CAN 11-bit ISO-TP | `prog …` |
| **GATEWAY** (Phase 10) | Protocol translation CAN0↔SWCAN via versioned mapping tables | CAN0, SWCAN (GM-LAN) | `gateway …` |
| **SLCAN** (Phase 12) | Lawicel ASCII transport for CAN232-style apps | CAN0 | Lawicel `O/C/S/t/T/r/R/F/V/N/Z/M/m/P/A` on the Bee UART |
| **HARNESS** (Phase 12) | Virtual ECU self-test on the CAN0↔CAN1 loopback pair (no vehicle) | CAN0, CAN1 (loopback) | `harness selfcheck | run [mask] | status` |

Mode switching: Button1 (short = cycle, long = M2RET), SerialUSB console
(`mode <name>`), or web GUI / SuperB. **Every switch runs
`quiesceBuses()` first** — CAN controllers to init/off, SWCAN TH8056 to
sleep, J1850 + K-line released (the shared pin-49 rail drops only when
both are quiesced, Phase 7) — so the vehicle never sees garbage
mid-switch.
`CMD_SET_MODE` dispatches exclusively through this safe path; there is no
backdoor.

PROG is the dangerous one: **read `docs/SAFETY_REVIEW.md` before use.**
Entry auto-suspends the MITM rule table (a bridge rewriting flash bytes is
a brick vector); rules are never silently restored. Destructive steps need
`prog arm` (console) or a per-step web checkbox; erase/download additionally
need a stable-12V+ **attestation** (ATRV is uncalibrated and gates nothing).
Seed/key is a stub — no algorithms, ever.

---

## 2. Resource ownership (who owns what)

Acquire-on-entry / release-on-exit (`docs/RESOURCE_OWNERSHIP.md`, from the
Pass 7 hardware audit). UART0/Bee is never released (immutable link).

| Mode | CAN0 | CAN1 | SWCAN | J1850 block | K-line | UART0/Bee | Notes |
|------|------|------|-------|-------------|--------|-----------|-------|
| M2RET | ✓ entry | ✓ entry | ✓ entry | — | — | ✓ | SPI0 CS3 + PIOC16 slot |
| ELM327 | ✓ lazy | — | — | ✓ lazy | ✓ lazy | ✓ | J1850 = GPIO+micros RX (TC6/TC7 unclaimed); K-line = USART0/Serial1 + SLP pins (Phase 7) |
| STN | ✓ lazy | — | — | ✓ lazy | ✓ lazy | ✓ | Same claims as ELM |
| UDS | ✓ lazy* | ✓ lazy* | — | — | — | ✓ | *the configured bus |
| MITM | ✓ entry | ✓ entry | — | — | — | ✓ | Independent bauds |
| OBD2 | ✓ lazy* | ✓ lazy* | — | — | — | ✓ | *the configured bus |
| PROG | ✓ lazy* | ✓ lazy* | — | — | — | ✓ | *the configured bus; suspends MITM rules + gateway mappings |
| GATEWAY | ✓ entry | — | ✓ entry | — | — | ✓ | CAN0 @ staged baud + SWCAN via swcan:: @ staged speed |
| SLCAN | ✓ lazy* | — | — | — | — | ✓ | *only after `O` (open); `C`/exit closes |
| HARNESS | ✓ entry | ✓ entry | — | — | — | ✓ | Loopback pair ONLY (wire CAN0H↔CAN1H, CAN0L↔CAN1L); refused without selfcheck |

Shared: pin 49 (`PS_J1850_9141`, the `J1850 9141 ON` rail) is refcounted
between the J1850 and K-line drivers via `rail9141` (Phase 7,
INV-PWR49) — pin 49 goes LOW only when both are quiesced.

Known collisions (hardware): J1850 PWM RX (TC7_A) shares TC7 with SWCAN SOF
(TC7_B); J1850 VPW RX (TC6_B) shares TC6 with RGB blue; J1850N_TX owns PWM6
while active; SPI0 needs arbitration (SWCAN CS3); PIOC16 has one callback
slot; DAC0 is muxed with CAN1 RX. The firmware's J1850 driver deliberately
uses GPIO interrupts, **not** timer capture, so TC6/TC7 stay free.

---

## 3. The F1 7E control channel (M2 ↔ SuperB)

Framing both directions, all modes: `0xF1 0x7E LEN PAYLOAD[LEN] XOR`
(`mm_channel.h`). In-band on the Bee UART at the immutable 115200.

- `0x01` set mode · `0x02` status · `0x03` CAN baud · `0x04` SWCAN speed ·
  `0x05` feature toggle
- `0x06`–`0x0B` UDS/OBD2 supervision (Phase 3)
- `0x0C`–`0x10` MITM supervision (Phase 4)
- `0x11`–`0x19` PROG supervision + chunked data path (Phase 5)
- `0x1A`–`0x22` transport supervision: TX scheduler, filters, bus stats, ISO-TP config (Phase 6)
- `0x23`–`0x24` K-line status + init (Phase 7)

Safety properties: apps can never inject control frames (the SuperB drops
any app→M2 `F1 7E` pair; legitimate GVRET/AT traffic never contains it);
the M2 only honors frames on `Serial`; mode switches go through the
safe-switch path only.

---

## 4. SuperB: wireless + web

- **Bluetooth Classic SPP** (`M2-SuperB`): app data pipe (Torque, FORScan…).
- **WiFi AP+STA** (WPA2, NVS credentials): AP for the garage, STA for the
  home network.
- **TCP :23**: GVRET-TCP target for SavvyCAN over WiFi.
- **Web GUI :80**: `/` status · `/mode` · `/settings` · `/wifi` ·
  `/uds` (+ `/api/status`) · `/obd2` (+ `/api/obd2`) · `/mitm`
  (+ `/api/mitm`) · `/prog` (+ `/api/prog`). Hand-written HTML, no
  framework. The `/prog` page carries the safety summary on the page
  itself and a streaming XHR firmware upload (41-byte chunks, CRC16 per
  chunk, CRC32 end-to-end).

---

## 5. Evidence labels (used everywhere)

- **VERIFIED** — compiles cleanly, or a primary-source fact (datasheet,
  Macchina variant/schematic/docs, standard document, vendored library
  source). Code comments cite the source.
- **INFERRED** — hardware behavior that cannot be tested without a bench:
  every bus timing, every ECU interaction, every "it should work" claim.
  INFERRED items are labeled at the point of use, never in aggregate.

---

## 6. INFERRED inventory (all phases, no bench)

Hardware/firmware facts still awaiting physical validation:

- **Bench validation** (the big one): flashing both chips, Bee UART echo,
  DS2 LED polarity — nothing has run on hardware yet.
- **SuperB**: flash size (defaulted 4 MB); PlatformIO upload-via-M2 recipe
  (the Arduino-IDE procedure is transcribed from Macchina's docs; the PIO
  equivalent is unverified).
- **SWCAN**: live `setupSW(33333)` register capture (HW-055); TH8056 mode
  timings.
- **M2RET**: GVRET protocol fidelity vs real SavvyCAN traffic.
- **ELM/STN**: AT/ST command responses against Torque/FORScan/SavvyCAN;
  J1850 PWM/VPW bit timings (GPIO+micros RX); STCMM listen-only ACK
  suppression effect on a real bus.
- **K-line (Phase 7)**: every timing (5-baud bit time, Twup, P1–P4),
  SLP_N polarity, the GPIO↔USART mux switchback for bit-banged init,
  TJA1027 echo behavior, response framing by inter-byte gap, `0x23`
  availability; the Bolt EV is CAN-only so none of this is vehicle-tested.
- **UDS/OBD2**: session handling, tester-present cadence, multi-ECU
  responses, 500k→250k detect order.
- **MITM**: loop-suppression window (5 ms) and rate-limiter tuning on a
  real bus; two-taps-of-one-bus vs two-different-buses behavior.
- **PROG**: every ECU interaction (0x10/0x27/0x31/0x34/0x36/0x37/0x11/
  0x23 sequencing, NRCs, timeouts); `0x23` read-back availability;
  `maxNumberOfBlockLength` parsing; upload throughput and browser
  tolerance for long uploads; arm (120 s), chunk-watchdog (30 s),
  tester-present (2 s) timings.
- **Silicon**: physical CHIPID read; ADC calibration constants.

---

## 7. Document map

| Doc | Covers |
|-----|--------|
| `SETUP.md` | PlatformIO beginner guide: install, build, flash, per-phase usage |
| `docs/SAFETY_REVIEW.md` | **PROG safety case — required reading before flashing** |
| `docs/OVERVIEW.md` | This file: the whole system |
| `docs/RESOURCE_OWNERSHIP.md` | Pass 7 collision table + per-phase claims |
| `docs/PHASE1_NOTES.md` | M2RET mode + wireless bridge |
| `docs/PHASE2_NOTES.md` | F1 7E channel, ELM327, J1850, ISO-TP, web GUI |
| `docs/PHASE3_NOTES.md` | UDS + OBD2 modes |
| `docs/PHASE4_NOTES.md` | MITM bridge + STN subset |
| `docs/PHASE5_NOTES.md` | PROG mode + data path |
| `docs/PHASE6_NOTES.md` | Transport layer: ISO-TP config, TX scheduler, unified filters, bus stats |
| `docs/PHASE7_NOTES.md` | K-line / ISO 9141 / KWP2000 stack + shared rail |

---

## 8. What this project deliberately does not do

- No J1850/SWCAN in UDS/PROG/MITM (CAN-only by design; TC7 collision).
- No invented security material: seed/key stays a stub, forever.
- No target-specific ECU procedures, memory maps, or routine IDs.
- No silent anything: aborts are loud, NRCs are reported, INFERRED is
  labeled, the safety review is impossible to miss.
