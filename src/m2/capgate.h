#pragma once
// M2 multi-mode firmware — capability gates (Phase 9).
//
// Formalized transmit/write authority. READ_ONLY is the default posture;
// ALLOW_TRANSMIT, ALLOW_DIAGNOSTIC_WRITE, ALLOW_ACTUATOR_CONTROL and
// ALLOW_FLASH_PROGRAMMING are explicit, per-mode grants. Grants are
// hierarchical: holding a higher grant includes everything below it.
//
// This is the mechanism (roadmap Phase 9) that keeps a 0x2E/0x31/flash
// command from behaving like a read. It formalizes what used to be
// arm-confirm + safe mode only. Arm-confirm stays as defense-in-depth
// where it already existed (UDS 0x11/0x2E, PROG flows) — the gate is the
// primary mechanism, the arm is the second factor.
//
// Rules:
//  - The mode manager calls enterMode() on EVERY mode entry (and boot).
//    The grant resets to the mode default — escalation NEVER survives a
//    mode switch, and nothing is granted silently.
//  - Escalation is explicit only: the `cap grant <level>` console command
//    or CMD_CAP_GRANT on the mm channel. Both print a warning to the USB
//    console (m5 note: on the mm path the escalating web-GUI party does
//    not see it — the warning is for the USB observer, not the escalator).
//  - require() is the enforcement point. Callers map their action to the
//    minimum grant and refuse (with a message) when it isn't held.
//
// Evidence: VERIFIED = compiles. INFERRED = gate semantics untested
// against a vehicle (no bench); the matrix below is the design record.

#include <Arduino.h>

namespace capgate {

enum class Grant : uint8_t {
    READ_ONLY = 0,              // passive: monitor, decode, log. No TX.
    ALLOW_TRANSMIT = 1,         // diagnostic reads: may put request frames
                                // on the bus (OBD polls, UDS reads, probes)
    ALLOW_DIAGNOSTIC_WRITE = 2, // state-changing diagnostics: UDS 0x2E,
                                // 0x31 startRoutine, 0x11 ECUReset, OBD
                                // services 04 (clear) and 08 (control)
    ALLOW_ACTUATOR_CONTROL = 3, // UDS 0x2F InputOutputControlByIdentifier
    ALLOW_FLASH_PROGRAMMING = 4,// UDS 0x34/0x36/0x37 transfer, PROG flows
};

const char* name(Grant g);

// Mode entry: resets the grant to the mode's default. modeIdx matches
// modes::Mode (M2RET=0, ELM327=1, STN=2, UDS=3, MITM=4, OBD2=5, PROG=6).
void enterMode(uint8_t modeIdx);

Grant current();

// Enforcement: true when current() >= need. On denial prints a refusal
// naming the required grant and the action; also dbg-logs it.
bool require(Grant need, const char* what);

// Explicit escalation (console/mm only). Upward-only: silently ignores a
// request at or below the current grant, prints a warning on real
// escalation, and never grants above the mode's ceiling. Returns the
// resulting grant.
Grant escalate(Grant g);

} // namespace capgate
