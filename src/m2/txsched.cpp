// M2 multi-mode firmware — generic TX scheduler (Phase 6).
// See txsched.h for the contract and ownership rules.

#include "txsched.h"
#include "busstat.h"
#include "debug.h"
#include <due_can.h>  // CANRaw globals; shim on the host sim

namespace txsched {

namespace {
Slot slots[MAX_SLOTS];

busstat::Bus toBus(uint8_t b)
{
    return (b == 1) ? busstat::Bus::CAN_BUS1 : busstat::Bus::CAN_BUS0;
}

void fire(Slot& sl, uint8_t idx)
{
    if (sl.txEn) {
        if (busstat::present(toBus(sl.bus))) {
            CANRaw* bus = (sl.bus == 1) ? &Can1 : &Can0;
            CAN_FRAME f;
            f.id = sl.id;
            f.extended = sl.ext ? 1 : 0;
            f.length = sl.len;
            f.rtr = 0;
            for (uint8_t i = 0; i < 8; i++)
                f.data.bytes[i] = (i < sl.len) ? sl.data[i] : 0x00;
            if (bus->sendFrame(f)) {
                busstat::noteTx(toBus(sl.bus), sl.len);
            } else {
                busstat::noteTxFail(toBus(sl.bus));
            }
        } else {
            // Owning mode doesn't hold the bus: skip the frame, count it.
            busstat::noteTxFail(toBus(sl.bus));
        }
    }
    sl.fires++;
    if (dbg::enabled) {
        dbg::print(F("[txsched] slot "));
        dbg::print((uint32_t)idx);
        dbg::print(F(" fired ("));
        dbg::print(sl.fires);
        dbg::println(F(")"));
    }
    if (sl.onFire != nullptr) sl.onFire(idx);
    if (sl.remaining > 0) {
        sl.remaining--;
        if (sl.remaining == 0) { sl.active = false; return; }
    }
    if (sl.periodMs == 0) { sl.active = false; return; }  // one-shot
    sl.nextAt += sl.periodMs;
    // If the callback blocked past several periods, don't fire a burst to
    // catch up — resync to now (INFERRED policy: periodic means cadence,
    // not backlog).
    uint32_t now = millis();
    if ((int32_t)(now - sl.nextAt) > (int32_t)sl.periodMs) sl.nextAt = now + sl.periodMs;
}

} // namespace

uint8_t add(const Slot& s)
{
    if (s.bus > 1 || s.len > 8) return 0xFF;
    for (uint8_t i = 0; i < MAX_SLOTS; i++) {
        if (!slots[i].used) {
            slots[i] = s;
            slots[i].used = true;
            slots[i].active = true;
            slots[i].remaining = s.count;
            slots[i].fires = 0;
            slots[i].nextAt = millis() + s.offsetMs;
            return i;
        }
    }
    return 0xFF;
}

bool remove(uint8_t idx)
{
    if (idx >= MAX_SLOTS || !slots[idx].used) return false;
    slots[idx].used = false;
    slots[idx].active = false;
    return true;
}

void clear()
{
    for (uint8_t i = 0; i < MAX_SLOTS; i++) {
        slots[i].used = false;
        slots[i].active = false;
    }
}

bool start(uint8_t idx)
{
    if (idx >= MAX_SLOTS || !slots[idx].used) return false;
    slots[idx].remaining = slots[idx].count;
    slots[idx].nextAt = millis() + slots[idx].offsetMs;
    slots[idx].active = true;
    return true;
}

bool stop(uint8_t idx)
{
    if (idx >= MAX_SLOTS || !slots[idx].used) return false;
    slots[idx].active = false;
    return true;
}

bool setPeriod(uint8_t idx, uint32_t periodMs)
{
    if (idx >= MAX_SLOTS || !slots[idx].used) return false;
    slots[idx].periodMs = periodMs;
    slots[idx].nextAt = millis() + slots[idx].offsetMs;
    return true;
}

void poll()
{
    uint32_t now = millis();
    for (uint8_t i = 0; i < MAX_SLOTS; i++) {
        Slot& sl = slots[i];
        if (!sl.used || !sl.active) continue;
        if ((int32_t)(now - sl.nextAt) >= 0) fire(sl, i);
    }
}

uint8_t count()
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < MAX_SLOTS; i++) if (slots[i].used) n++;
    return n;
}

const Slot* get(uint8_t idx)
{
    if (idx >= MAX_SLOTS || !slots[idx].used) return nullptr;
    return &slots[idx];
}

} // namespace txsched
