// M2 multi-mode firmware — capability gates implementation (Phase 9).
// See capgate.h for the contract and docs/SAFETY_REVIEW.md §10 for the
// safety case.

#include "capgate.h"
#include "debug.h"

namespace capgate {

namespace {

// Per-mode default grants, indexed by modes::Mode
// (M2RET=0, ELM327=1, STN=2, UDS=3, MITM=4, OBD2=5, PROG=6, GATEWAY=7).
// Rationale (full matrix in docs/SAFETY_REVIEW.md §10):
//  - M2RET: raw frame TX is the mode's purpose (upstream GVRET behavior).
//  - ELM/STN: user-driven diagnostic terminals; every transmission is a
//    direct consequence of a typed/app command. Apps (Torque) clear DTCs.
//  - UDS: raw service access — reads by default, writes need escalation.
//  - MITM: stealth posture (never originates). Modify rules escalate.
//  - OBD2: the poller only reads.
//  - PROG: the flashing mode; arm + power-attestation + explicit config
//    remain as the procedural interlocks on top of the grant.
//  - GATEWAY (Phase 10): originates translated frames by construction —
//    ALLOW_TRANSMIT minimum per the roadmap; MODIFY mappings need
//    DIAGNOSTIC_WRITE (the ceiling) + arm-confirm.
const Grant DEFAULTS[10] = {
    Grant::ALLOW_TRANSMIT,         // M2RET
    Grant::ALLOW_DIAGNOSTIC_WRITE,  // ELM327
    Grant::ALLOW_DIAGNOSTIC_WRITE,  // STN
    Grant::ALLOW_TRANSMIT,         // UDS
    Grant::READ_ONLY,              // MITM
    Grant::ALLOW_TRANSMIT,         // OBD2
    Grant::ALLOW_FLASH_PROGRAMMING, // PROG
    Grant::ALLOW_TRANSMIT,         // GATEWAY
    Grant::ALLOW_TRANSMIT,         // SLCAN (Phase 12: TX is its job)
    Grant::ALLOW_TRANSMIT,         // HARNESS (Phase 12: loopback only)
};

// Escalation ceilings: a mode can never escalate above its ceiling, even
// via console/mm. MITM can reach TRANSMIT (modify rules) but never the
// write/flash grants — it is not a diagnostic mode.
const Grant CEILINGS[10] = {
    Grant::ALLOW_TRANSMIT,         // M2RET
    Grant::ALLOW_DIAGNOSTIC_WRITE,  // ELM327
    Grant::ALLOW_DIAGNOSTIC_WRITE,  // STN
    Grant::ALLOW_FLASH_PROGRAMMING, // UDS (xfer needs it; arm-confirm stays)
    Grant::ALLOW_TRANSMIT,         // MITM
    Grant::ALLOW_DIAGNOSTIC_WRITE,  // OBD2 (clear DTCs)
    Grant::ALLOW_FLASH_PROGRAMMING, // PROG
    Grant::ALLOW_DIAGNOSTIC_WRITE,  // GATEWAY (MODIFY mappings)
    Grant::ALLOW_TRANSMIT,         // SLCAN (Phase 12)
    Grant::ALLOW_TRANSMIT,         // HARNESS (Phase 12: never writes/flash)
};

Grant cur = Grant::READ_ONLY;
uint8_t curMode = 0xFF;

} // namespace (anonymous)

const char* name(Grant g)
{
    switch (g) {
        case Grant::READ_ONLY: return "READ_ONLY";
        case Grant::ALLOW_TRANSMIT: return "ALLOW_TRANSMIT";
        case Grant::ALLOW_DIAGNOSTIC_WRITE: return "ALLOW_DIAGNOSTIC_WRITE";
        case Grant::ALLOW_ACTUATOR_CONTROL: return "ALLOW_ACTUATOR_CONTROL";
        case Grant::ALLOW_FLASH_PROGRAMMING: return "ALLOW_FLASH_PROGRAMMING";
        default: return "?";
    }
}

void enterMode(uint8_t modeIdx)
{
    curMode = modeIdx;
    cur = (modeIdx < 10) ? DEFAULTS[modeIdx] : Grant::READ_ONLY;
    dbg::print(F("[cap] mode ")); dbg::print(modeIdx);
    dbg::print(F(" grant -> ")); dbg::println(name(cur));
}

Grant current() { return cur; }

bool require(Grant need, const char* what)
{
    if ((uint8_t)cur >= (uint8_t)need) return true;
    SerialUSB.print(F("REFUSED: '"));
    SerialUSB.print(what);
    SerialUSB.print(F("' needs "));
    SerialUSB.print(name(need));
    SerialUSB.print(F(" (current: "));
    SerialUSB.print(name(cur));
    SerialUSB.println(F("). Escalate with `cap grant <level>` if intended."));
    dbg::print(F("[cap] DENIED ")); dbg::print(what);
    dbg::print(F(" needs ")); dbg::println(name(need));
    return false;
}

Grant escalate(Grant g)
{
    if (curMode >= 8) return cur;
    Grant ceiling = CEILINGS[curMode];
    if ((uint8_t)g > (uint8_t)ceiling) g = ceiling;  // never above ceiling
    if ((uint8_t)g <= (uint8_t)cur) return cur;      // upward only
    SerialUSB.println(F("WARNING: escalating capability grant."));
    SerialUSB.print(F("  "));
    SerialUSB.print(name(cur));
    SerialUSB.print(F(" -> "));
    SerialUSB.println(name(g));
    SerialUSB.println(F("This enables state-changing bus operations."));
    SerialUSB.println(F("The grant resets to the mode default on mode switch."));
    cur = g;
    dbg::print(F("[cap] ESCALATED to ")); dbg::println(name(cur));
    return cur;
}

} // namespace capgate
