#pragma once
// M2 multi-mode firmware — vehicle discovery engine (Phase 9).
//
// Detect bus/protocol -> find ECUs -> addressing -> supported
// services/PIDs -> ECU capability table -> exposed to the SuperB via the
// mm channel (CMD_DISCOVER_START/STATUS/LIST) and the USB console.
//
// Design notes:
//  - CAN only in Phase 9 (the Bolt EV test vehicle is CAN-only; K-line /
//    J1850 / SWCAN discovery is a later phase). 11-bit and 29-bit.
//  - Discovery TRANSMITS (functional 0100 probes), so it requires the
//    ALLOW_TRANSMIT capability gate. It never sends state-changing
//    services: 04 (clear) and 08 (control) are deliberately NOT probed.
//  - Acquire-on-entry / release-on-exit: run() owns the bus for the whole
//    scan. Callers (obd2/uds consoles, mm) must stop their own bus activity
//    first — the entry points do this.
//  - OEM seam: physical addressing guesses come from oem::DIAG_ACCESS
//    (scripts/oem_extract.py); the generic SAE row is the default.
//  - Everything is INFERRED until tested against a real vehicle.
//
// Evidence: VERIFIED = compiles. INFERRED = untested on hardware.

#include <Arduino.h>
#include <stdint.h>

namespace discover {

constexpr uint8_t MAX_ECUS = 8;
constexpr uint8_t PID_MASK_WORDS = 8;  // 0100..01E0 support masks

struct EcuInfo {
    uint32_t rxId = 0;       // response (physical) ID
    uint32_t txId = 0;       // request (physical) ID
    uint8_t nodeAddr = 0;    // ECU address byte (filter-model `addr` seam)
    uint32_t services = 0;   // bitmask: bit n = service n supported
    uint8_t pidSup[PID_MASK_WORDS * 4] = {0};  // 0100/0120/.../01E0 masks
    char vin[18] = {0};
    bool vinValid = false;
};

struct Result {
    bool done = false;
    uint8_t nEcus = 0;
    EcuInfo ecus[MAX_ECUS];
    uint32_t baud = 0;
    bool ext = false;        // 29-bit
    uint8_t bus = 0;
};

// Blocking discovery scan on CAN bus 0/1. Returns true when at least one
// ECU answered. The result is kept for STATUS/LIST queries.
bool run(uint8_t bus, Result& out);

// Last completed (or attempted) scan.
const Result& lastResult();

// Cooperative cancel for a running scan (console/mm).
void cancel();

// Paged ECU list for the mm channel: up to 4 entries of 11 bytes each
// [txId LE32][rxId LE32][nSvcs][vinOk][nodeAddr] into out.
// nTotal = total ECUs in the result; nRet = entries returned.
void listPage(const Result& r, uint8_t startIdx, uint8_t* out,
              uint8_t& nTotal, uint8_t& nRet);

// Human-readable report on SerialUSB.
void print(const Result& r);

// Narrow the shared busfilter table to the discovered ECU response IDs
// (addr slot = nodeAddr — the Phase 6 OEM seam). Returns false when there
// is nothing to apply.
bool applyFilters(const Result& r);

} // namespace discover
