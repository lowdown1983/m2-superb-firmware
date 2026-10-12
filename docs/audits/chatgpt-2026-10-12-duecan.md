# Macchina M2 + SuperB Silicon-Facing Firmware Audit

**Audit date:** 2026-10-12  
**Firmware archive audited:** `m2-superb-firmware-complete-20261012-9899bb0.zip`  
**Firmware archive SHA-256:** `090a2d85a1a5f2aaae8b524e8e8c1bfe1f78490c19eacb5489f925ad2e3166ea`

## Scope and evidence boundary

This audit uses the exact October 12 firmware archive above, the supplied OEM database, `Documentation.zip`, and `Bosch-CAN-Bit-Timing-Hartwich-Bassemir.pdf`. The current firmware was extracted under `/mnt/data/audit_current/`; the October 11 archive was used only for a file/test-coverage comparison, not as proof of current behavior.

The audit is restricted to silicon-facing behavior: native CAN timing/peripheral configuration, SWCAN controller/transceiver recovery order, pin mux/ownership, and interrupt safety. The OEM database was used to check whether the affected CAN rates are relevant to the documented vehicle paths; it is not evidence for register behavior.

Evidence labels mean:
- **VERIFIED** — directly shown by current source or the pinned dependency's public source.
- **INFERRED** — expected hardware consequence of the observed configuration/order; not measured on a physical board.

## Verification run

### Native simulation builds

Both native targets compiled from the current archive using `g++`:

```sh
g++ -O0 -g -I src/sim_shim -DSIM_M2 -std=gnu++17 \
  src/m2/*.cpp src/sim_shim/*.cpp \
  -o .pio/build/sim_m2/program

g++ -O0 -g -I src/sim_shim -I src/sim_shim_esp -DSIM_SUPERB -std=gnu++17 \
  src/superb/*.cpp src/sim_shim/*.cpp src/sim_shim_esp/*.cpp \
  -o .pio/build/sim_superb/program
```

**Result: both passed.** The full current host test suite also passed:

```sh
python3 -m pytest sim/tests/ -q --disable-warnings
# 26 passed in 28.70s
```

These are host-shim results. The shims do not emulate the SAM3X CAN timing registers, the physical CAN transceivers, bus propagation delays, SWCAN electrical behavior, or cycle-accurate J1850 timing.

### Target-build limitation

`pio`, `platformio`, `arm-none-eabi-g++`, and `xtensa-esp32-elf-g++` are not installed in this execution environment. I therefore could not independently build/link the SAM3X8E or ESP32 target images. The current archive's changelog reports all four PlatformIO environments built successfully, but that claim was not independently reproduced here.

## Findings

### 1. MAJOR — VERIFIED timing-constraint violation; operational impact INFERRED

**Location in current project:**
- `platformio.ini:52` — `collin80/due_can@2.1.6`
- `src/m2/mode_manager.cpp:739-740` — `Can0.begin(can0BaudSet, 255)` and `Can1.begin(can1BaudSet, 255)`
- `src/m2/mode_manager.h:109-110` — both native CAN defaults are `500000`
- Pinned dependency's public `due_can/src/due_can.h:81,86-89` — timing-table metadata and row for 8 TQ

**Code evidence:** the upstream `due_can` 2.1.6 timing table contains:

```cpp
{8, (2 + 1), (1 + 1), (1 + 1), (2 + 1), 75},
```

The dependency's own structure comment describes `uc_sjw` as `min(uc_phase1, 4)` (`due_can.h:81` in the public source). The row decodes to **TQ=8, Prop=3, Phase_Seg1=2, Phase_Seg2=2, SJW=3**. SJW is therefore longer than both phase buffer segments.

The current dependency’s public `due_can.cpp:83-94` searches TQ values from `CAN_MIN_TQ_NUM` (8) upward and breaks on a zero remainder; `due_can.cpp:97-101` computes the prescaler and indexes the table using the selected TQ. For the SAM3X8E's 84 MHz master clock, the following project-accepted rates select the invalid 8-TQ row:

| Requested bitrate | Selected TQ | BRP | Phase 1 | Phase 2 | SJW | Constraint check |
|---:|---:|---:|---:|---:|---:|---|
| 100 kbit/s | 8 | 105 | 2 tq | 2 tq | 3 tq | FAIL |
| 125 kbit/s | 8 | 84 | 2 tq | 2 tq | 3 tq | FAIL |
| 250 kbit/s | 8 | 42 | 2 tq | 2 tq | 3 tq | FAIL |
| 500 kbit/s | 8 | 21 | 2 tq | 2 tq | 3 tq | FAIL |

The other common allowlisted rates checked in the same model (800 kbit/s and 1 Mbit/s) select valid rows. The firmware defaults both CAN controllers to 500 kbit/s, so the invalid row affects the default native-CAN configuration. The supplied OEM database also identifies 500 kbit/s as the GM high-speed diagnostic CAN path (`work/gm.md:142`).

**Hardware/protocol invariant:** SJW must not exceed either phase buffer segment: `SJW <= min(Phase_Seg1, Phase_Seg2)`. This constraint is explicit in the supplied Bosch paper, *The Configuration of the CAN Bit Timing*, page 2, Table 1. The public `due_can` repository labels its matching library source version 2.1.6; the project pins that version in `platformio.ini:52`.

**Impact:** **INFERRED.** The register configuration violates the timing constraint, reducing valid synchronization tolerance and potentially increasing sporadic receive/transmit or arbitration errors under oscillator and propagation-delay differences. This finding does **not** prove that all CAN traffic fails or that the vehicle connection must fail; a CAN network can appear to work despite a bad timing configuration.

**Suggested fix:** correct the 8-TQ row in a maintained `due_can` fork/package so `uc_sjw` is 2 functional TQ (change `(2 + 1)` to `(1 + 1)` for that field), or regenerate timing rows with a constraint-checked calculation. Apply the correction inside the library's `set_baudrate()`/initialization sequence. Add a test that checks every table row for `SJW <= min(Phase_Seg1, Phase_Seg2)` and confirms the row selected for each supported bitrate. Do not implement this as a post-`begin()` CAN-register override. The **separate October 11 archive** (`m2-superb-firmware-publish-20261011-9a7f204.zip`, `CHANGELOG.md:12-23`) documents a field regression when its experimental override cleared CANEN after full initialization without rebuilding mailbox state. That is historical integration evidence only; the custom override and its test are absent from the current October 12 archive.

**Validation after fix:** target-build the M2 image, read back or inspect the timing register encoding, then test 500 kbit/s and 250 kbit/s on a bench CAN network while monitoring error counters and arbitration. A scope/analyzer test is still required to judge the chosen propagation segment and sample point for the physical transceiver, bus length, and topology.

## Current-source fixes verified from the prior audit

These are **not new findings** because the current source now contains the corrective ordering:

- `src/m2/swcan.cpp:326-333`: the bus-off path calls `hw.setListenOnlyMode(wantListenOnly)` instead of forcing normal mode.
- `src/m2/swcan.cpp:357-369`: `recover()` drives the TH8056 to sleep before resetting/reinitializing the MCP2515.
- `src/m2/swcan.cpp:371-385`: failed recovery clears `isAcquired` and SWCAN-present state while the transceiver remains asleep and the IRQ remains detached; successful recovery reapplies listen-only before restoring the transceiver mode and reattaching the IRQ.
- The native CAN begin call sites inspected pass `255`, avoiding unintended `due_can` enable-pin activity on Arduino pins 50 and 48.

**Execution caveat:** `src/m2/swcan.cpp:348-349` returns `true` directly in the host simulation. The real MCP2515 register-recovery path is therefore source-reviewed but not dynamically exercised by the 26 passing tests. The changelog explicitly notes that no MCP2515 register mocks exist yet.

## Questions / unverified items

1. **Confirm the resolved PlatformIO package bytes.** The project pins `due_can@2.1.6` and the public repository's `library.properties` also says 2.1.6, but the resolved `.pio/libdeps/m2/due_can/src/due_can.h` from the exact target build was not present in the uploaded archive. Confirm its 8-TQ row matches the upstream source before patching.
2. **Physical CAN timing:** what bus length/topology and transceiver/input-output delays should define the propagation segment and sample point? The Bosch paper requires these physical delays and oscillator tolerance to be considered; they cannot be established from host simulation alone.
3. **GM mid-speed rate path:** `work/gm.md:143-149` describes GM GMLAN mid-speed as 95.2 kbit/s and notes a 95.2/125 kbit/s discrepancy. `src/m2/mode_manager.cpp:2124-2128` allowlists 100 and 125 kbit/s, not 95.2; however, `src/m2/mode_m2ret.cpp:185-191` passes a supplied rate through to `begin()` after a 1 Mbit/s cap. It is unclear which path is intended to provide model-specific 95.2 kbit/s support. Confirm the target ECU/model and intended configuration API before filing this as a defect.
4. **SuperB flash size:** `platformio.ini:58-60` explicitly leaves the WROOM-32D flash size unconfirmed and assumes a 4 MB `esp32dev` layout. The physical module's flash size and partition fit need a board-level check; this audit did not establish them.

## Input hashes

- `m2-superb-firmware-complete-20261012-9899bb0.zip`: `090a2d85a1a5f2aaae8b524e8e8c1bfe1f78490c19eacb5489f925ad2e3166ea`
- `m2-oem-database-publish.zip`: `23aebea87d8daf057a4d32bde825b288094e7250531928937c06cc1351e7ba33`
- `Documentation.zip`: `561b814eb21039b1c3a7f28b9c585db398807771e0f45f4eff1893c75a367582`
- `Bosch-CAN-Bit-Timing-Hartwich-Bassemir.pdf`: `89947e364c13ec473d46ddb012e852a4763e5d7fac3b6a484724bce76157e8c3`

---

## Verification notes (Cortana, 2026-10-12)

Audited tree: `m2-superb-firmware-complete-20261012-9899bb0.zip`
(SHA-256 `090a2d85…`, verified matching the created archive).
Current HEAD at verification: `9899bb0` (master).

**MAJOR finding — VERIFIED in the pinned dependency.**
`.pio/libdeps/m2/due_can/src/due_can.h:99`:
`{8, (2+1), (1+1), (1+1), (2+1), 75}` → TQ=8, PROG=3, PH1=2, PH2=2,
**SJW=3**. The struct comment (`due_can.h:89`) documents `uc_sjw` as
`min(uc_phase1, 4)` = 2. SJW=3 exceeds both phase segments — violates
SJW <= min(PH1, PH2) per the Bosch paper. The TQ-selection loop
(`due_can.cpp:94-104`) picks TQ=8 for 100/125/250/500 kbit/s at 84 MHz
(all divide evenly, remainder 0), so the default 500 kbit/s native CAN
uses the bad row. This answers the auditor's question 1: the resolved
package bytes match the upstream source they quoted.

**On the suggested fix — concur, with emphasis.** Do NOT do a
post-`begin()` CANBR override: commit `4504922` tried exactly that on
2026-10-11 and it was a vehicle-tested regression (clearing CANEN
post-init without rebuilding mailbox state killed all CAN TX/RX;
reverted in `9a7f204`, both abandoned per 2026-10-12 baseline decision).
The correct fix is at the source: a maintained due_can fork (or
constraint-checked table regeneration) with the 8-TQ row's SJW
corrected to 2, applied inside the library's init sequence.

**Impact calibration:** the Bolt EV has been communicating on this
timing (Torque 6.9 PIDs/sec, 286-sensor scan on the `9405769` baseline),
so this is a robustness/tolerance defect, not a hard failure — consistent
with the auditor's INFERRED grading. It cannot explain the differential
"no ECU" incidents (the bad row is constant across all builds).

**Prior-audit fixes — confirmed present** in `9899bb0` as the auditor
states (bus-off listen-only, recover ordering, failed-recovery rollback,
`begin(baud, 255)`).

Disposition 2026-10-12: finding accepted. Recommended as a "due_can
timing fork" work item: fork the library, correct the 8-TQ SJW to 2,
add a table-constraint unit test (every row: SJW <= min(PH1, PH2)),
target-build, and validate 500k/250k on a bench CAN with error-counter
monitoring before vehicle use. Priority: after the open field "no ECU"
hunt — the car talks today, this hardens tolerance.
