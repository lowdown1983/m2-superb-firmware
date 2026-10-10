#pragma once
// M2 multi-mode firmware — virtual ECU / self-test harness (Phase 12).
//
// The M2 talks to ITSELF: CAN0 <-> CAN1 loopback with this module as the
// scripted ECU (no bench needed). Automated regression without a vehicle:
// scripted OBD/UDS responses, ISO-TP segmentation, negative responses,
// delayed/dropped/malformed frames, bus overload.
//
// PHYSICAL REQUIREMENT (documented, not assumed): wire CAN0H <-> CAN1H
// and CAN0L <-> CAN1L externally (a short twisted pair). Both controllers
// need termination per the M2 hardware docs — INFERRED wiring detail.
// selfcheck() verifies the loopback BEFORE any test runs.
//
// SAFETY: the harness runs on the loopback pair ONLY. Enforcement:
//  - It is a dedicated HARNESS mode, entered explicitly (`mode harness`
//    / mm CMD_SET_MODE). Entering quiesces every other bus owner.
//  - runSuite() REFUSES unless selfcheck() passed in this session — a
//    vehicle bus will not echo our marker frames, so the suite cannot
//    start against one (heuristic gate, documented as such).
//  - The ECU simulator only answers the scripted test IDs (0x7E0);
//    it never initiates traffic on its own.
//
// This is the FIRST real validation path for the INFERRED CAN behaviors
// (ISO-TP reassembly, N_Cr/N_Bs timeouts, SN error handling). Test
// outcomes may be labeled HARNESS-VALIDATED (vs INFERRED vs
// VEHICLE-VALIDATED) — see docs/PHASE12_NOTES.md.
//
// Evidence: VERIFIED = compiles. HARNESS-VALIDATED = only after a real
// loopback run (not in this phase — no hardware here).

#include <Arduino.h>
#include <stdint.h>

namespace vharness {

// Test IDs (bitmask for runSuite()).
enum : uint16_t {
    T_OBD_SINGLE   = 0x0001,  // OBD 010C single-frame round trip
    T_ISOTP_MULTI  = 0x0002,  // 0902 VIN multi-frame reassembly
    T_NEG_RESP     = 0x0004,  // 7F 22 31 negative response
    T_STMIN_DELAY  = 0x0008,  // CFs spaced 10ms (STmin respected)
    T_DROP_CF      = 0x0010,  // CF stream stalls -> N_Cr timeout (neg test)
    T_BAD_SN       = 0x0020,  // wrong sequence number -> error (neg test)
    T_OVERLOAD     = 0x0040,  // 200-frame burst, no lockup
    T_UDS_DID      = 0x0080,  // UDS 22 F190 scripted DID
    T_ALL          = 0x00FF,
};

struct TestResult {
    uint16_t id;
    bool pass;
    char note[40];
};

struct SuiteResult {
    bool ran = false;
    uint8_t nTests = 0;
    uint8_t nPass = 0;
    TestResult tests[8];
};

// Mode lifecycle (called by ModeManager on HARNESS enter/exit/loop).
void begin();   // acquire CAN0+CAN1 @500k (begin(baud,255)), promiscuous
void loop();    // pump the ECU simulator (answers scripted IDs on CAN1)
void exit();    // release both controllers

// Loopback presence gate: marker frame CAN0->CAN1 and CAN1->CAN0.
// Returns true only if both arrive (heuristic: a vehicle bus will not
// echo our markers). runSuite() refuses without a passing selfcheck.
bool selfcheck();

// Run the scripted suite (blocking). Requires a passing selfcheck this
// session. Console/mm entry point.
bool runSuite(uint16_t mask, SuiteResult& out);

// Last suite result (for mm CMD_HARNESS_STATUS).
const SuiteResult& lastResult();

// Human-readable report on SerialUSB.
void print(const SuiteResult& r);

} // namespace vharness
