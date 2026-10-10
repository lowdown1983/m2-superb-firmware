#pragma once
// M2 multi-mode firmware — generic TX scheduler (Phase 6).
//
// Periodic / one-shot / counted CAN transmission as a shared service.
// Slots carry: message ID, bus, payload, period, offset, count, and
// start/stop conditions. A slot may also be callback-only (txEn=false),
// in which case the scheduler is purely a timer — the OBD2 poll engine
// uses it exactly this way (its sweep does request/response via isotp).
//
// OWNERSHIP: the scheduler owns no bus. A slot fires only while its
// target bus is present (busstat::present); the mode that configured the
// slot must hold the bus (acquire-on-entry, like every other bus user).
// Scheduler and filters are bounded by the owning mode's existing
// transmit authority — Phase 6 adds no new authority (see SAFETY_REVIEW).
//
// Evidence: VERIFIED = compiles. INFERRED = timing untested on hardware.

#include <Arduino.h>

namespace txsched {

constexpr uint8_t MAX_SLOTS = 8;

struct Slot {
    bool used = false;
    bool active = false;
    bool txEn = true;    // false = callback-only (no CAN frame sent)
    uint8_t bus = 0;     // 0 = Can0, 1 = Can1 (SWCAN reserved: Phase 8)
    uint32_t id = 0;
    bool ext = false;
    uint8_t data[8] = { 0,0,0,0,0,0,0,0 };
    uint8_t len = 0;     // <= 8
    uint32_t periodMs = 0;  // 0 = one-shot (fires once after offsetMs)
    uint32_t offsetMs = 0;  // initial delay before first fire
    int32_t count = -1;     // -1 = infinite, else total fires
    // --- runtime (managed by poll()) ---
    int32_t remaining = -1;
    uint32_t nextAt = 0;
    uint32_t fires = 0;
    void (*onFire)(uint8_t slot) = nullptr;  // optional, called after TX
};

// Add a slot (copies s). Arms nextAt = now + offsetMs. Returns the slot
// index, or 0xFF when the table is full / args invalid.
uint8_t add(const Slot& s);
bool remove(uint8_t idx);
void clear();
bool start(uint8_t idx);   // re-arm a stopped slot (fresh offset)
bool stop(uint8_t idx);    // halt; keeps the slot for later start()
bool setPeriod(uint8_t idx, uint32_t periodMs);  // live re-arm
void poll();               // call once per main loop (ModeManager::loop)
uint8_t count();
const Slot* get(uint8_t idx);

} // namespace txsched
