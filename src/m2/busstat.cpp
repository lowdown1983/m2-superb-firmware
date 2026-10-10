// M2 multi-mode firmware — per-bus error/status reporting (Phase 6).
// See busstat.h for the contract.

#include "busstat.h"
#include "debug.h"
#include <due_can.h>  // CANRaw globals (Can0/Can1); shim on the host sim

namespace busstat {

namespace {
Stats s[(uint8_t)Bus::COUNT];
bool up[(uint8_t)Bus::COUNT] = { false };
#ifndef SIM_M2
uint32_t lastSr[(uint8_t)Bus::COUNT] = { 0 };
#endif

inline bool valid(Bus b) { return (uint8_t)b < (uint8_t)Bus::COUNT; }
} // namespace

void noteTx(Bus b, uint8_t nbytes)
{
    if (!valid(b)) return;
    Stats& st = s[(uint8_t)b];
    st.txFrames++;
    st.txBytes += nbytes;
}

void noteRx(Bus b, uint8_t nbytes)
{
    if (!valid(b)) return;
    Stats& st = s[(uint8_t)b];
    st.rxFrames++;
    st.rxBytes += nbytes;
}

void noteRxOverflow(Bus b)
{
    if (!valid(b)) return;
    s[(uint8_t)b].rxOverflow++;
    dbg::print(F("[busstat] RX overflow on bus "));
    dbg::println((uint32_t)b);
}

void noteTxFail(Bus b)
{
    if (!valid(b)) return;
    s[(uint8_t)b].txFail++;
    dbg::print(F("[busstat] TX fail on bus "));
    dbg::println((uint32_t)b);
}

void noteDropped(Bus b)
{
    if (!valid(b)) return;
    s[(uint8_t)b].dropped++;
}

void noteBusOff(Bus b)
{
    if (!valid(b)) return;
    s[(uint8_t)b].busOff++;
    dbg::print(F("[busstat] BUS-OFF on bus "));
    dbg::println((uint32_t)b);
}

void setPresent(Bus b, bool isUp)
{
    if (!valid(b)) return;
    up[(uint8_t)b] = isUp;
#ifndef SIM_M2
    if (isUp && (uint8_t)b < 2) {
        // Seed the SR latch so a pre-existing BOFF doesn't count as a new
        // transition the moment the bus comes up. K-line (index 2) has no
        // CAN_SR — skip it.
        Can* p = (b == Bus::CAN_BUS0) ? CAN0 : CAN1;
        lastSr[(uint8_t)b] = p->CAN_SR;
    }
#endif
}

bool present(Bus b)
{
    return valid(b) && up[(uint8_t)b];
}

void poll()
{
#ifdef SIM_M2
    // No CAN_SR on the host sim; counters still work, events don't.
    return;
#else
    // SAM3X CAN_SR: BOFF/WARN/ERRA are live state bits (the same register
    // due_can's ISR reads every interrupt). Count 0->1 transitions.
    // INFERRED: never observed on real hardware (no bench).
    // Phase 7/8: only the native CAN controllers (indices 0-1) have a
    // CAN_SR; K-line (2), J1850 (3), SWCAN (4) are skipped — SWCAN bus-off
    // is detected from the MCP2515 EFLG by the swcan driver instead.
    for (uint8_t i = 0; i < 2; i++) {
        if (!up[i]) continue;
        Can* p = (i == 0) ? CAN0 : CAN1;
        uint32_t sr = p->CAN_SR;
        uint32_t was = lastSr[i];
        if ((sr & CAN_SR_BOFF) && !(was & CAN_SR_BOFF)) {
            s[i].busOff++;
            dbg::print(F("[busstat] BUS-OFF on CAN"));
            dbg::println((uint32_t)i);
        }
        if ((sr & (CAN_SR_WARN | CAN_SR_ERRA)) &&
            !(was & (CAN_SR_WARN | CAN_SR_ERRA))) {
            s[i].errWarn++;
            dbg::print(F("[busstat] error-warning on CAN"));
            dbg::println((uint32_t)i);
        }
        lastSr[i] = sr;
    }
#endif
}

const Stats& get(Bus b)
{
    static const Stats empty;
    return valid(b) ? s[(uint8_t)b] : empty;
}

void reset(Bus b)
{
    if (!valid(b)) return;
    s[(uint8_t)b] = Stats();
}

void resetAll()
{
    for (uint8_t i = 0; i < (uint8_t)Bus::COUNT; i++) s[i] = Stats();
}

uint8_t pack(Bus b, uint8_t* out)
{
    if (!valid(b) || out == nullptr) return 0;
    const Stats& st = s[(uint8_t)b];
    uint32_t v[8] = { st.txFrames, st.rxFrames, st.txBytes, st.rxBytes,
                      st.rxOverflow, st.txFail, st.busOff, st.dropped };
    for (uint8_t i = 0; i < 8; i++) {
        out[4 * i + 0] = (uint8_t)(v[i] & 0xFF);
        out[4 * i + 1] = (uint8_t)((v[i] >> 8) & 0xFF);
        out[4 * i + 2] = (uint8_t)((v[i] >> 16) & 0xFF);
        out[4 * i + 3] = (uint8_t)((v[i] >> 24) & 0xFF);
    }
    return 32;
}

} // namespace busstat
