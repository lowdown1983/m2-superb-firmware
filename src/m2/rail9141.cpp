// M2 multi-mode firmware — shared 'J1850 9141 ON' power rail (Phase 7).
// See rail9141.h for the ownership contract (INV-PWR49 / INV-MODE50).

#include "rail9141.h"
#include "debug.h"

namespace rail9141 {

namespace {
bool heldBy[2] = { false, false };

// INFERRED rail/transceiver settle time (carried over from Phase 2 J1850).
constexpr uint16_t RAIL_SETTLE_MS = 50;
} // namespace

void powerUp(Owner o)
{
    uint8_t i = (uint8_t)o;
    if (i > 1) return;
    if (heldBy[i]) return;  // idempotent per owner
    heldBy[i] = true;
    if (!powered()) {
        // First owner brings the rail up. Owner-specific pins (J1850 pin 50)
        // must already be settled — that is the caller's contract.
        pinMode(PS_J1850_9141, OUTPUT);
        digitalWrite(PS_J1850_9141, HIGH);
        delay(RAIL_SETTLE_MS);
    }
    if (dbg::enabled) {
        dbg::print(F("[rail9141] powerUp owner "));
        dbg::println((uint32_t)i);
    }
}

void powerDown(Owner o)
{
    uint8_t i = (uint8_t)o;
    if (i > 1) return;
    if (!heldBy[i]) return;  // idempotent per owner
    heldBy[i] = false;
    if (!heldBy[0] && !heldBy[1]) {
        // INV-PWR49: pin 49 LOW only when BOTH are quiesced.
        pinMode(PS_J1850_9141, OUTPUT);
        digitalWrite(PS_J1850_9141, LOW);
    }
    if (dbg::enabled) {
        dbg::print(F("[rail9141] powerDown owner "));
        dbg::println((uint32_t)i);
    }
}

bool held(Owner o)
{
    uint8_t i = (uint8_t)o;
    return i <= 1 && heldBy[i];
}

bool powered()
{
    return heldBy[0] || heldBy[1];
}

} // namespace rail9141
