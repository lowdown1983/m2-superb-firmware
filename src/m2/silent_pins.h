// silent_pins.h — TJA1051 silent-mode pin helpers.
//
// Audit fix F-2 (2026-10-08): the knowledge pack's INV-SILENT (VERIFIED)
// requires that a mode entering listen-only drive the relevant TJA1051 S
// pin HIGH on entry and restore its prior level on exit. Listen-only was
// previously implemented via the SAM3X ABM bit alone, which does suppress
// TX and ACKs at the controller — this is defense in depth at the
// transceiver, per the pack invariant.
//
// Pin truth (pack m2-pins.yaml / conflicts.yaml, VERIFIED):
//   Arduino 71 = HSC_S (PD3) -> TJA1051 on CAN0 (high-speed bus)
//   Arduino 74 = MSC_S (PD0) -> TJA1051 on CAN1 (medium-speed bus)
//   HIGH = silent (transmitter disabled), LOW = normal.
#pragma once
#include <Arduino.h>

namespace silentpins {

constexpr uint8_t HSC_S_PIN = 71;  // CAN0
constexpr uint8_t MSC_S_PIN = 74;  // CAN1

// Ownership tracking: true while an enter() has no matching exit().
// Lets a scoped user (e.g. auto-detect) add silence without clearing
// another owner's: only exit what you raised.
inline bool& heldFlag(uint8_t which)
{
    static bool h0 = false, h1 = false;
    return (which == 0) ? h0 : h1;
}

// True if the S pin is currently claimed HIGH by an un-exited enter().
// Usage: bool weRaised = !silentpins::held(0);
//        if (weRaised) silentpins::enter(0); ... if (weRaised) silentpins::exit(0);
inline bool held(uint8_t which) { return heldFlag(which); }

// Drives the S pin for `which` (0 = CAN0, 1 = CAN1) HIGH = silent.
// Lazily configures the pin as OUTPUT; the only level ever driven before
// entry is LOW (normal), so restore is deterministic.
inline void enter(uint8_t which)
{
    uint8_t pin = (which == 0) ? HSC_S_PIN : MSC_S_PIN;
    static bool init0 = false, init1 = false;
    bool& init = (which == 0) ? init0 : init1;
    if (!init) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); init = true; }
    digitalWrite(pin, HIGH);
    heldFlag(which) = true;
}

// Restores the S pin for `which` to LOW = normal operation.
inline void exit(uint8_t which)
{
    uint8_t pin = (which == 0) ? HSC_S_PIN : MSC_S_PIN;
    digitalWrite(pin, LOW);
    heldFlag(which) = false;
}

}  // namespace silentpins
