# 2023 Chevy Bolt EV — in-vehicle test support

**Purpose:** everything needed to exercise the M2/SuperB firmware's modes
against the 2023 Bolt EV (the only test vehicle available). Programming mode
is excluded per Marcus's direction.

**Evidence grades:** A = OEM/official doc · B = open-source implementation
with real-vehicle use · C = community PID-list consolidation · INFERRED = synthesis.
Grade C PIDs are starting guesses: send, watch the value move with the car,
then trust. Attribution: `ATTRIBUTION.md`; raw PID JSON: `bolt17.json`, `bolt19.json`.

**Global test conditions:** parked, key ON (READY for powertrain ECUs — the
earlier Car Scanner failure was ECUs asleep without READY). 12V maintainer for
long sessions. Never drive with MITM bridging or gateway MODIFY rules active.

---

## 1. Bus architecture

| Bus | Rate | OBD pins | Notes |
|---|---|---|---|
| High-speed GMLAN (powertrain) | 500 kbps, 11-bit | 6 / 14 | Torque auto-detected ISO15765-4 CAN 11/500 here (Grade A, observed) |
| Low-speed GMLAN (SWCAN) | 33.33 kbps, single-wire | 1 | Body/modules; OVMS drives two SWCAN buses on the Bolt (Grade B) |

- GMLAN family: low 33.33k / mid 95.2k / high 500k (Grade A, ACDelco TechConnect).
- The BCM acts as the gateway between buses (Grade A). The Bolt firewalls
  some GMLAN commands arriving via the OBD port (unlike the Volt) — expect
  some TX to be ignored at the DLC (Grade B, OVMS community).
- SWCAN wake-up: the car may need low-speed traffic to wake modules (Grade B).

## 2. Diagnostic addressing (11-bit, 500k HS bus)

**VEHICLE-VALIDATED 2026-10-10** — Car Scanner ECU scan of the test car
(VIN 1G1FX6S00P4175849, 2023 MY). Five responders:

| Req | Resp | Module (car's name) | What's there |
|---|---|---|---|
| 0x7E0 | 0x7E8 | ECM-EngineControl | Standard OBD2 (already working via Torque) |
| 0x7E1 | 0x7E9 | HPC1-HybridPtCtrl1 | Hybrid powertrain control 1 |
| 0x7E4 | 0x7EC | HPC2-HybridPtCtrl | **High-voltage PIDs live here** (SOC 228334, capacity, charging, temps) |
| 0x7E6 | 0x7EE | BSC2-BrakeSyste | Brake system control |
| 0x7E7 | 0x7EF | BECM-B+EnergyCtrl | 96 cell voltages, section temps |

Corrections vs the community sources: 0x7E4 is **HPC2**, not "BECM" — the
battery PIDs are served by HPC2 on this car; the module literally named BECM
is at 0x7E7. (Addresses match OVMS; only the names were wrong.)

**Not answering:** 0x7E2 (no 0x7EA responder) — the OVMS trans-temp PID
0x1940 is likely dead on this car. Grade: VEHICLE-VALIDATED (this car).

Functional broadcast: 0x7DF. All extended diagnostics are service 0x22
(read DID) with the 4-hex-digit PID = last 4 of the `22xxxx` code.

**Discovery-engine expectation:** auto-detect + discovery should find exactly
these 5 responders (7E8/7E9/7EC/7EE/7EF). Anything else answering is
unexpected — record it.

Other modules seen via Mode 09 (not UDS responders): Transmission control
unit #1 (supplier LGE), BCM, EPS/TDM (column EPS), ABS/EBCM/ESC (Bosch GEN09),
Continuous Damping Control, instrument cluster. Build date ~2023-07-21.

## 3. Extended PIDs that matter (all Grade C unless noted)

**Battery / high-voltage (header 7E4):**

| PID | Name | Equation | Notes |
|---|---|---|---|
| 228334 | SOC | raw·39/99 (OVMS, Grade B) | Primary state of charge |
| 2245F9 | Pack capacity | ((A·256)+B)·0.0032 | kWh; 2019+ BECM firmware |
| 2241A3 | Pack capacity | ((A·256)+B)·0.032 | kWh; 2017 BECM firmware — **try both on the 2023, keep the one that answers** |
| 224531 | Charging status | A==1\|\|2\|\|3 | A==3 → DC fast charging |
| 22434F | Battery temp | A−40 | °C |
| 22801F | Ambient temp | A/2−40 | °C |
| 22432D | HV battery voltage | community | try; unconfirmed on 2023 |
| 224181–224240 | Cell voltages 1–96 | per-cell mV | On **7E7**, not 7E4; range has gaps — walk it and record what answers (Grade B for the range, C for each cell) |

**Powertrain (headers 7E0/7E1/7E2):**

| PID | Name | Equation | Notes |
|---|---|---|---|
| 22000D | Vehicle speed | A | km/h, header 7E0 |
| 221940 | Trans fluid temp | ((A−40)·9/5)+32 | °F, header 7E2 (Grade B, GM family) |
| 221940 | (same) | A−40 | °C variant |
| 222414 | HV current | (Signed(A)·256+B)/20 | A, header 7E1 — Volt-era PID, **verify before trusting** |
| 222885 | HV voltage | ((A·256)+B)/100 | V, header 7E1 — Volt-era PID, **verify before trusting** |

ELM/STN usage: `ATSH7E4` then `22 83 34` (no spaces: `228334`). The M2's
ELM core supports ATSH (Phase 9), so header switching works.

## 4. Broadcast frames for passive monitoring (Grade B, OVMS)

| ID | Content | Decode |
|---|---|---|
| 0x0C9 | Ignition + motor RPM | d[0]&0xC0 = ign ON; RPM = GET16(d,1)>>2 |
| 0x3E9 | Vehicle speed | GET16(d,0)/100, mph |
| 0x1A1 | Throttle | d[7]·100/256, % |
| 0x1F5 | PRNDL | d[4]: 1=P 2=R 3=N 4=D 5=L |
| 0x0D1 | Foot brake | d[4] |
| 0x241 | BCM | OVMS *transmits* here for preheat — RX-only for us |

## 5. Per-mode test plan

### M2RET (SavvyCAN)
1. Connect at 500k/11-bit on CAN0. Expect immediate traffic in READY.
2. Confirm 0x0C9/0x3E9/0x1A1 present; cross-check speed against dash.
3. SWCAN: 33.33k on the SWCAN channel (OBD pin 1). Expect low-speed body traffic.
4. Try `ATSH7E4`-style manual extended-PID query via the console to validate §3.

### ELM327 (Torque / Car Scanner)
Already validated (Torque: 6.9 PIDs/s). Remaining: full instrumented retest
(`debug on`, Car Scanner, READY) — still open. Add custom PIDs from §3
(228334 SOC, 2245F9 capacity) to Torque and confirm plausible values.

### STN (FORScan)
Same as ELM; FORScan is picky about clone heuristics — STN mode is the
standing recommendation. Verify FORScan connects and reads DTCs.

### UDS
Target 0x7E4 (BECM), service 0x22: DID 0x8334 (SOC), 0x45F9 (capacity),
0x434F (batt temp). Read-only services only — 0x27 stays a stub, and PROG
mode (flashing) is excluded. Any NRC 0x7F/0x31 (request out of range) just
means that DID isn't on this firmware — record and move on.

### OBD2 (poller)
Add to the custom PID list: 228334, 2245F9 (and 2241A3 as fallback),
224531, 22434F on header 7E4. Log a charge session if one happens
opportunely — SOC vs capacity over time is the most valuable validation.

### MITM
**Passive monitoring only on the Bolt.** Ordered rules + 5ms
loop-suppression are INFERRED until harness-tested; do not bridge TX on a
live car. Validate: install a PASS rule, confirm frames flow and the car is
unaffected (parked, READY).

### Gateway (CAN↔SWCAN)
The Bolt is the first vehicle where this is testable (both buses present).
Safe starter mapping: PASS 0x3E9 (HS→SWCAN direction rule), observe only.
MODIFY rules stay parked — no payload mutation on someone else's car.

### SLCAN
Lawicel over the Bee/UART path; point SavvyCAN at it, confirm the same
traffic as M2RET mode. Validates the transport, not the car.

### Auto-detect / discovery / logging
Run auto-detect: expect ISO15765-4 CAN 11/500 on the HS bus, SWCAN stage
should now actually run (audit fix M-1). Discovery should find the 7E0–7E7
responders. Log a short session to SPIFFS and download as CSV.

## 6. What NOT to do (safety boundaries)

- **No PROG mode** — excluded by direction; no flashing, no 0x27, no seed/key.
- **No contactor/actuator commands.** Community Volt/Bolt CAN guides document
  7E4 `AE`-service frames that open/close battery contactors. These are
  **never** test targets — high-voltage safety, not our business.
- **No TX while driving.** MITM bridge rules and gateway MODIFY are
  parked-car-only, and even then prefer passive.
- The Bolt's OBD-port firewall means some TX will be silently ignored —
  that's the car protecting itself, not a firmware bug. Record it.

## 7. Open questions to answer with the car

1. Which capacity PID answers on the 2023: 2245F9 or 2241A3? (Ask HPC2 at 0x7E4.)
2. Do the Volt-era 7E1 PIDs (222414/222885) answer, or are they dead? (0x7E1 =
   HPC1 on this car — plausible but unconfirmed.)
3. Which of the 96 cell PIDs (224181–224240 on **0x7E7/BECM**) actually respond?
4. Does the SWCAN stage of auto-detect find the 33.33k bus on pin 1?
5. ~~Any UDS DIDs beyond the OVMS set (try 0x22 on 0x7E2/0x7E7)?~~ — 0x7E2
   does not answer on this car (answered 2026-10-10); 0x7E7/BECM still worth
   probing beyond the cell PIDs.
