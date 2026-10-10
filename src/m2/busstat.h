#pragma once
// M2 multi-mode firmware — per-bus error/status reporting (Phase 6).
//
// One implementation of bus statistics for every bus, extending the MITM
// stats pattern (mode_mitm.cpp) to all buses. Counters are fed by the
// shared choke points (isotp sendCan/waitFrame, txsched, busfilter,
// MITM bridge, ELM monitor, M2RET pump) so modes never hand-roll stats.
//
// Bus-off / error-warning detection: polled from the SAM3X CAN_SR
// register (BOFF/WARN bits — the same register due_can's own ISR reads;
// BOFF/WARN are live state bits, not clear-on-read). Polled only for
// controllers that are currently begun (busstat::setPresent), because a
// clock-gated CAN peripheral must not be touched. INFERRED until a real
// bus-off is observed on hardware.
//
// Evidence: VERIFIED = compiles. INFERRED = untested on hardware.

#include <Arduino.h>

namespace busstat {

enum class Bus : uint8_t {
    CAN_BUS0 = 0,
    CAN_BUS1 = 1,
    KLINE = 2,   // Phase 7: the K-line driver reports here
    J1850 = 3,   // Phase 8: the hardened J1850 driver reports here
    SWCAN = 4,   // Phase 8: the hardened SWCAN driver reports here
    COUNT = 5,
};

struct Stats {
    uint32_t txFrames = 0;
    uint32_t rxFrames = 0;
    uint32_t txBytes = 0;
    uint32_t rxBytes = 0;
    uint32_t rxOverflow = 0;  // frames lost: reassembly-slot / queue exhaustion
    uint32_t txFail = 0;      // sendFrame() refused, or scheduler fired with bus down
    uint32_t busOff = 0;      // CAN_SR BOFF 0->1 transitions (polled)
    uint32_t errWarn = 0;     // CAN_SR WARN/ERRA 0->1 transitions (polled)
    uint32_t dropped = 0;     // frames dropped by filter or policy decisions
};

void noteTx(Bus b, uint8_t nbytes);     // one frame transmitted
void noteRx(Bus b, uint8_t nbytes);     // one frame received
void noteRxOverflow(Bus b);             // RX data lost (no slot / queue full)
void noteTxFail(Bus b);                 // TX refused (mailbox full / bus down)
void noteDropped(Bus b);                // dropped by filter/policy (not an error)
void noteBusOff(Bus b);                 // bus-off event (Phase 8: SWCAN TXBO)

// Bus presence: the mode manager sets this wherever it begins/ends a
// native controller (modes::canBegin0/1, canEnd0/1, enter/exitM2RET,
// quiesceBuses). poll() only touches registers for present buses.
void setPresent(Bus b, bool up);
bool present(Bus b);

// Poll CAN_SR for BOFF/WARN transitions. Call once per main loop
// (ModeManager::loop does this). No-op on the host sim (no registers).
void poll();

const Stats& get(Bus b);
void reset(Bus b);
void resetAll();

// Pack one bus's stats as 8x LE32 (txFrames, rxFrames, txBytes, rxBytes,
// rxOverflow, txFail, busOff, dropped). errWarn is folded into the mm
// reply separately if ever needed. Returns bytes written (32).
uint8_t pack(Bus b, uint8_t* out);

} // namespace busstat
