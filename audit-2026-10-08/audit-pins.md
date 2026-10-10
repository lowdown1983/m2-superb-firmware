# Pin Audit — M2/SuperB Firmware (2026-10-08)

**Auditor:** Cortana (fresh-eyes subagent). **Findings only — no code edited.**
**Firmware:** `~/workspace/m2-superb-firmware/` (`src/m2/`, `src/superb/`, `lib/SWcan/`)
**Pin truth:** `~/workspace/m2-superb-knowledge/source/` — `m2-pins.yaml`, `superb-pins.yaml`,
`bee-link.yaml`, `conflicts.yaml`, `superb-conflicts.yaml`, `constraints.yaml` (pack v1.4.0).
Pack entries marked VERIFIED are treated as ground truth; INFERRED stays INFERRED.

## 1. Method

1. Read all six pack sources, extracting the VERIFIED pin map (96 M2 pins), the Bee-link
   crossover, strapping rules, and the full conflict/invariant list
   (C1–C12, SC1–SC8, INV-DUECAN-EN, INV-PWR49, INV-MODE50, INV-SILENT, INV-SWC-INT,
   INV-QUIESCE-ORDER, INV-SB-STRAP, INV-SB-ADC2, INV-SB-FLASH-PINS, INV-SB-UART0).
2. Enumerated every pin-touching construct in firmware with ripgrep:
   `pinMode|digitalWrite|digitalRead|analogRead|analogWrite` (all arguments captured),
   `Can0.begin|Can1.begin|Can0.enable|Can1.enable|Can0.disable|Can1.disable`,
   `Serial.begin|Serial1.begin|Serial2.begin` (all ports), `SPI.begin`,
   `attachInterrupt|detachInterrupt`, `TC2|TC0|TC1|PWM->|PWMC_|REG_TC`,
   and every `#define|constexpr` naming a pin (`XBEE_*`, `DS*`, `BUTTON*`, `J1850*`,
   `K_*`, `L_*`, `PS_*`, `SWC*`, `CAN*_CS`, `VSENSE*`, `SUPERB_LED`).
3. Cross-checked each claim against the pack; verified library behavior against the
   **vendored sources actually compiled** (`.pio/libdeps/m2/due_can`,
   `.pio/libdeps/m2/can_common`, `.pio/libdeps/m2/MCP2515`) and the Arduino SAM core
   actually used (`~/.platformio/packages/framework-arduino-sam`).
4. Checked `variants/m2/variant.h` / `variant.cpp` for every raw Arduino number used.

Result: the firmware touches pins **only through named constants** — zero raw numeric
literals in any `pinMode`/`digitalWrite`/`digitalRead`/`analogRead` call. Every named
constant resolves to the pack-VERIFIED Arduino number (details in §2).

## 2. Per-subsystem pin-truth tables

### 2a. Native CAN (due_can) — the 50/48 hazard (pack C4, INV-DUECAN-EN)

| Firmware claim | Pack truth | Verdict |
|---|---|---|
| `Can0.begin(baud,255)` / `Can1.begin(baud,255)` at mode_manager.cpp:734-735, 2098-2099; mode_m2ret.cpp:181; autodet.cpp:39; vharness.cpp:178-179; mode_slcan.cpp:76 | begin() drives Arduino 50 (J1850_PWM_VPW) / 48 (PS_BUCK) HIGH unless enPin=255 | **PASS** — every begin call passes 255 |
| `Can0.disable()/Can1.disable()` guarded by `can0Begun/can1Begun` latches (mode_manager.cpp:696-697, 755-756, 2152, 2169) and `canSafeToDisable` (mode_m2ret.cpp:190) | disable() drives the enable pin LOW unless 255 (due_can.cpp:462) | **PASS** — verified against vendored due_can 2.1.6: `CAN_COMMON::begin(b,e)` sets `enablePin=e` (can_common.cpp:192-196); `init()`/`enable()`/`disable()` all guard `if (enablePin != 255)` (due_can.cpp:267-269, 451, 462) |
| No `Can0.enable()` / `Can1.enable()` calls anywhere | enable() would drive 50/48 HIGH if not neutralized | **PASS** — none exist |
| Pin 48 (PS_BUCK) never touched by firmware | Vestigial hook; Arduino `init()` never configures it as output (variant.cpp:429-431 only disables pull-ups; VERIFIED in framework source) | **PASS** |

### 2b. J1850 (pins 49/50/51/52/53/54)

| Firmware claim | Pack truth | Verdict |
|---|---|---|
| `J1850_PWM_VPW=50`, `J1850_PWM_RX=51`, `J1850_VPW_RX=52`, `J1850P_TX=53`, `J1850N_TX=54` (j1850.h:43-49, via variant.h) | VERIFIED: 50/PB8, 51/PC28, 52/PC26, 53/PC18, 54/PC23 | **PASS** |
| acquire(): pin 50 settled **before** `rail9141::powerUp(J1850)` (j1850.cpp:353-357) | INV-MODE50/INV-PWR49: pin 50 settled before pin 49 HIGH | **PASS** |
| RX via GPIO interrupt + `micros()`; TC6/TC7 timer-capture deliberately unused (j1850.h:27-39); no `TC*`/`PWM*` register writes anywhere in `src/` | C1/C2/C5/C6: TC6/TC7/PWM6 shared with RGB/SWC-SOF; INV-TC7/INV-TC6/INV-PWM6 | **PASS** — conflicts avoided by design; RESOURCE_OWNERSHIP.md records TC6/TC7 unclaimed |
| TX idle parked LOW on release (j1850.cpp:389-391) | Safe idle state | **PASS** (but see Finding F-3) |

### 2c. Shared rail pin 49 (J1850 vs K-line — pack C7, INV-PWR49)

| Firmware claim | Pack truth | Verdict |
|---|---|---|
| `PS_J1850_9141=49`; owned via `rail9141` two-owner refcount (rail9141.cpp); pin 49 HIGH only when first owner powers up, LOW only when **both** release | C7: one shared power net; INV-PWR49 | **PASS** — implementation matches the contract exactly |
| quiesceBuses(): J1850 release → K-line release, both idempotent (mode_manager.cpp:706-712) | INV-QUIESCE-ORDER | **PASS** |

### 2d. K-line / L-line (pins 55–60, TJA1027)

| Firmware claim | Pack truth | Verdict |
|---|---|---|
| `K_TX=55, K_RX=56, K_SLP=57, L_TX=58, L_RX=59, L_SLP=60` (kline.h:42-46) | VERIFIED: 55/PA11, 56/PA10, 57/PB4, 58/PA13, 59/PA12, 60/PB7 | **PASS** |
| SLP driven HIGH=normal / LOW=sleep; polarity labeled INFERRED (kline.h:43,45) | Pack: SLP_N active-LOW INFERRED | **PASS** — honestly labeled |
| 5-baud bit-bang targets K_TX/L_TX; `Serial1.end()`→GPIO→`Serial1.begin()` mux switchback labeled INFERRED (kline.cpp:307-321) | Pack silent on mux behavior | **PASS** — no pin fact invented |
| `initSlow()` refuses if `j1850::acquired()` (kline.cpp:299) | C7 shared-rail arbitration | **PASS** |

### 2e. SWCAN (pins 61–68, TH8056 + MCP2515)

| Firmware claim | Pack truth | Verdict |
|---|---|---|
| `SWC_M0=64, SWC_M1=65`, mode table SLEEP=00/HIGHSPEED=10/HV_WAKEUP=01/NORMAL=11 (swcan.cpp:50-53) | VERIFIED TH8056 table: 00 sleep, 10 high-speed, 01 HV wake-up, 11 normal | **PASS** |
| `SPI0_CS3=63`, `SWC_INT=67` → `SWcan hw(SPI0_CS3, SWC_INT)` (swcan.cpp:23); INT configured INPUT by `MCP2515::MCP2515` ctor (`pinMode(INT_Pin,INPUT)`, MCP2515.cpp:41) | INV-SWC-INT: pin 67 must be INPUT + PIO IRQ, not variant OUTPUT default | **PASS** — verified in vendored lib source |
| `SWC_SOF=68` never touched; no TC7 capture | INV-TC7 (C2) | **PASS** |
| `SPI.begin()` in main.cpp:31 before any MCP2515 use | SPI0 required by MCP2515 | **PASS** |

### 2f. TJA1051 silent pins 71/74 (pack C10, INV-SILENT)

| Firmware claim | Pack truth | Verdict |
|---|---|---|
| `CAN0_CS=71`, `CAN1_CS=74` defined in variant.h:192,197 — **never referenced** in `src/` | INV-SILENT: "A mode entering listen-only must drive the relevant S pin HIGH on entry and restore its prior level on exit" (VERIFIED) | **FAIL** — see Finding F-2 |
| Listen-only implemented via SAM3X `CAN_MR_ABM` bit (`enable_autobaud_listen_mode()`, due_can.cpp:530-533) and MCP2515 listen-only mode (swcan.cpp:104) | Controller-level listen-only suppresses TX and ACKs | Functional goal met; invariant letter not honored |

### 2g. Bee link (M2 Serial ↔ SuperB UART0)

| Firmware claim | Pack truth | Verdict |
|---|---|---|
| M2: `Serial.begin(115200)` in main.cpp:21; never reconfigured (no other `Serial.begin`) | Bee UART = PA8/PA9 = Arduino 0/1 @115200, immutable (bee-link.yaml, VERIFIED) | **PASS** |
| SuperB: `Serial.begin(115200)` (main.cpp:4326), ESP32-Arduino default UART0 pins (GPIO1/GPIO3) | INV-SB-UART0: Bee link is UART0 (GPIO1/GPIO3) only | **PASS** |
| K-line uses `Serial1`/`Serial2` (pins 55-60), never `Serial` | Bee UART untouched by K-line | **PASS** |
| M2 never drives `XBEE_RST`(4) / `XBEE_MULT4`(11); Button2(23), DS2(14)/DS3(15), XBEE_STAT/CTS/RTS untouched | Pack: variant `PIO_OUTPUT_0` entries are inert — `init()` (variant.cpp:429-431) only disables pull-ups, never drives outputs (VERIFIED in source) | **PASS** — SuperB strapping (SC1/INV-SB-STRAP) not disturbed |

### 2h. SuperB GPIOs

| Firmware claim | Pack truth | Verdict |
|---|---|---|
| `SUPERB_LED=26` as OUTPUT (main.cpp:4328) — comment says "INFERRED (Macchina docs)" | superb-pins/superb-conflicts SC6: GPIO26 → LED D1 (active-high), VERIFIED from SCH-01012 | **PASS** on the pin; firmware's evidence label is overcautious (pack has it VERIFIED) |
| `pair_::PIN=0` (GPIO0) as `INPUT_PULLUP`, polled for Button2-hold pairing (main.cpp:4335, 539) | SC1/INV-SB-STRAP: strapping sampled at reset only; post-boot GPIO use is fine | **PASS** |
| No use of GPIO6–11, no ADC, no touch, no GPIO12 drive | INV-SB-FLASH-PINS, INV-SB-ADC2 | **PASS** |

### 2i. Buttons, V_SENSE, misc M2

| Firmware claim | Pack truth | Verdict |
|---|---|---|
| `BUTTON_PIN=22` (Button1), `INPUT_PULLUP`, active-LOW (mode_manager.h:124, mode_manager.cpp:641,978) | VERIFIED: 22/PC25 | **PASS** |
| `VSENSE_PIN=92` (PA6/AD3), `analogRead` (mode_manager.cpp:500-501) | VERIFIED: pin 92, divider 0.18033 (constraints.yaml) | Pin **PASS**; scaling **FAIL** — see Finding F-1 |

## 3. Findings

### F-1 — CRITICAL — V_SENSE voltage reads 4× low: wrong ADC resolution assumption

**Where:** `src/m2/mode_manager.cpp:495-504` (`mmGetVoltageCb`)
```c
constexpr uint8_t VSENSE_PIN = 92;  // PA6/AD3
uint32_t raw = analogRead(VSENSE_PIN);
mV = (uint16_t)((raw * 446932UL) / 100000UL);   // raw * 4.46932
```
The constant 4.46932 mV/LSB is the **12-bit** scaling: 3300 mV / 4095 × (1/0.18033)
= 4.4716. But nothing in the firmware calls `analogReadResolution(12)`, and the
Arduino SAM core default is **10-bit** — verified in the compiled framework source:
`static int _readResolution = 10;`
(`~/.platformio/packages/framework-arduino-sam/cores/arduino/wiring_analog.c:25`),
with `analogRead()` returning `mapResolution(ulValue, 10, _readResolution)` (line 86).

Correct 10-bit scaling would be 3300/1023 × 5.5455 = **17.886 mV/LSB**.

**Consequence:** a 12.6 V battery reads as ~3.1 V. This callback feeds
`mm::CMD_GET_VOLTAGE` (mm_channel.h:196), consumed by the SuperB Phase-11.5
sleep/wake manager (src/superb/main.cpp:630-670), whose engine-off threshold
defaults to 12.9 V — a threshold the reported voltage can **never** reach, so the
SuperB will permanently believe the engine is off (spurious light-sleep / wrong
wake behavior). Pin 92 itself is correct (pack VERIFIED); the fault is the
resolution/scaling mismatch in the read path.

### F-2 — MAJOR — INV-SILENT not honored: TJA1051 S pins (71/74) never driven

**Where:** `variants/m2/variant.h:192` (`CAN0_CS=71`), `:197` (`CAN1_CS=74`) —
defined but referenced nowhere in `src/`. Listen-only paths:
`src/m2/mode_elm.cpp:591` (`Can0.enable_autobaud_listen_mode()`),
`src/m2/mode_m2ret.cpp:172` (same), `src/m2/swcan.cpp:104` (MCP2515 listen-only).

The pack's INV-SILENT (conflicts.yaml, VERIFIED) requires: "A mode entering
listen-only must drive the relevant S pin HIGH on entry and restore its prior
level on exit." The firmware relies entirely on controller-level listen-only
(SAM3X `CAN_MR_ABM`, MCP2515 listen-only mode), which does suppress TX and ACKs —
the functional goal is met — but the explicit, VERIFIED pack invariant is not
implemented. Graded MAJOR (not CRITICAL) because no bus disturbance occurs via
the ABM path; it is a missing defense-in-depth layer and a direct deviation from
a VERIFIED invariant.

### F-3 — MAJOR — `j1850::release()` moves pin 50 while the shared rail may be powered

**Where:** `src/m2/j1850.cpp:391` — `digitalWrite(J1850_PWM_VPW, LOW)` runs
unconditionally in `release()`, even when `rail9141` is still held by K-line
(pin 49 HIGH). Pack C8: "the mode pin must be settled BEFORE the interface is
powered, and must not change while traffic is active." Changing the J1850
mode-select while the shared rail is powered (K-line session active) violates the
strict reading. Low practical risk — J1850 is quiesced before release, so no
traffic is active — but the ordering guarantee is broken for the powered-but-idle
case. Consider restoring pin 50 only when the rail is fully down, or documenting
the powered-idle exception.

### F-4 — MINOR — SuperB LED evidence label understates the pack

**Where:** `src/superb/main.cpp:66` — `SUPERB_LED = 26` commented "INFERRED
SuperB LED (Macchina docs)". The pack (superb-conflicts.yaml SC6) has GPIO26 → LED
D1 **VERIFIED** from SCH-01012. The pin choice is correct; only the label is
overcautious. No behavior change needed.

### F-5 — NOTE — due_can 50/48 hazard fully neutralized (verified, not just claimed)

Every `begin()` passes 255 (§2a), and I verified the neutralization in the vendored
due_can 2.1.6 / can_common sources actually compiled: `begin(b,e)` stores
`enablePin=e`; `init()`/`enable()`/`disable()` all skip pin driving when
`enablePin==255`. The `can0Begun`/`can1Begun` latches and `canSafeToDisable`
(mode_m2ret.cpp:69,183,190) correctly gate every `disable()`. No `enable()` calls
exist. C4/INV-DUECAN-EN: **satisfied**.

### F-6 — NOTE — double-claimed pins: all documented or avoided

- Pin 49 (J1850/K-line): shared via `rail9141` refcount — arbitration present and correct.
- TC6/TC7 (J1850-RX vs RGB/SWC-SOF), PWM6, TC0_B (SWC_M0): **no claimant in firmware**
  — J1850 uses GPIO interrupts, SWCAN never captures SOF, no `analogWrite`/TC/PWM
  register use anywhere in `src/`. Mutual exclusion holds by non-use, as
  RESOURCE_OWNERSHIP.md records.
- SPI0: sole onboard claimant is the MCP2515 (NPCS3=63); no expansion SPI0 client
  exists in firmware (INV-SPI0 satisfied).
- PB15 (pin 72 CAN1RX vs DAC0/analog): due_can configures it via `PIO_PB15A_CANRX1`
  (peripheral A); no DAC/analog use of pin 72 or 104 anywhere.

### F-7 — NOTE — pack-silent areas (no pin facts invented)

- K-line SLP_N polarity, 5-baud GPIO→UART mux switchback, J1850 TX polarity, rail
  settle times: firmware labels all INFERRED — matches pack silence. Correct.
- `SWC_SOF` (68) capture, `XBEE_RTS/CTS`, expansion SPI0 CS0 (Bee pin 17 vs
  SuperB GPIO2 strapping, SC1): unused by firmware; no claims made.

## 4. Verdict summary

| Subsystem | Result |
|---|---|
| due_can 50/48 hazard | PASS (F-5) |
| J1850 pins + acquire ordering | PASS (F-3 is a release-path ordering nit) |
| Pin-49 shared rail refcounting | PASS |
| K-line/L-line pins | PASS |
| SWCAN pins + INT config | PASS |
| Silent pins 71/74 | **FAIL — F-2 (MAJOR)** |
| Bee UART @115200 both sides | PASS |
| SuperB GPIOs/strapping/ADC2/flash pins | PASS (F-4 label nit) |
| Buttons / V_SENSE pin | Pin PASS; **F-1 (CRITICAL)** on ADC scaling |
| variants/m2/ consistency | PASS — all numbers via variant.h defines, match pack |
| Double-claimed pins | All arbitrated or unclaimed (F-6) |

**Bottom line:** the pin *assignments* are correct everywhere — this firmware does not
put any signal on the wrong pin. The two substantive defects are one level up from
the pin map: **F-1**, a 10-bit/12-bit ADC scaling error that breaks the SuperB
sleep/wake voltage input (CRITICAL), and **F-2**, non-implementation of the
VERIFIED INV-SILENT transceiver-pin drive (MAJOR, functionally masked by
controller listen-only). **F-3** is a release-ordering deviation worth a
one-line fix or an explicit documented exception.
