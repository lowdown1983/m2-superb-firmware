#pragma once
// M2 multi-mode firmware — auto-detection engine (Phase 12).
//
// Ordered probe: CAN (125k/250k/500k/1M x 11/29-bit, LISTEN-ONLY) ->
// K-line (5-baud / fast init) -> J1850 (VPW/PWM) -> SWCAN
// (normal/high-speed/wake). Looks for valid frames, stable timing, and
// known arbitration IDs (0x7DF functional, 0x7E8-0x7EF responses).
//
// SAFETY-CRITICAL DESIGN (documented, enforced in code):
//  - The engine is EXPLICITLY USER-INITIATED ONLY: `autodetect start`
//    on the USB console or CMD_AUTODET_START on the mm channel. It NEVER
//    runs on boot, NEVER in the background, and NEVER from any other
//    mode's normal operation.
//  - CAN probing is LISTEN-ONLY (due_can setListenOnlyMode): zero
//    transmit. K-line init and J1850/SWCAN active probing TRANSMIT BY
//    CONSTRUCTION, so those stages additionally require the Phase 9
//    capgate (ALLOW_TRANSMIT) AND a second explicit confirmation
//    (`autodetect start tx` / mm flag byte = 1). Without it, the engine
//    stops after the listen-only stages and reports what it heard.
//  - Passive stages first: J1850 and SWCAN are observed passively
//    (live buses chatter); active probing is the last resort.
//
// Builds on the Phase 9 discover engine (CAN ECU find) — autodet adds
// the baud/format sweep and the multi-protocol ordering around it.
//
// Evidence: VERIFIED = compiles. INFERRED = every detection behavior
// untested on hardware (no multi-protocol test vehicle here).

#include <Arduino.h>
#include <stdint.h>

namespace autodet {

enum : uint8_t {
    PROTO_NONE = 0,
    PROTO_CAN_11 = 1,
    PROTO_CAN_29 = 2,
    PROTO_KLINE = 3,
    PROTO_J1850_VPW = 4,
    PROTO_J1850_PWM = 5,
    PROTO_SWCAN = 6,
};

struct Result {
    bool done = false;
    bool found = false;
    uint8_t proto = PROTO_NONE;
    uint32_t baud = 0;        // CAN baud (0 for non-CAN)
    uint8_t bus = 0;          // CAN bus index used
    uint8_t confidence = 0;   // 0-100 heuristic score
    uint32_t framesSeen = 0;
    char note[48];            // human-readable stage detail
};

// Start a detection run. allowTx = true enables the TRANSMITTING stages
// (K-line init, J1850/SWCAN active probe) — requires capgate
// ALLOW_TRANSMIT and is a separate explicit user decision from starting
// the engine. Returns false when refused (gates) or already running.
// Blocking; pumps the USB console for cancel ('q' + enter is NOT needed —
// the run is short; use the mm cancel path for remote abort).
bool start(bool allowTx);

// Cooperative cancel (mm/console).
void cancel();

// Last completed (or attempted) run.
const Result& lastResult();

// Human-readable report on SerialUSB.
void print(const Result& r);

} // namespace autodet
