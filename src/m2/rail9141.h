#pragma once
// M2 multi-mode firmware — shared 'J1850 9141 ON' power rail (Phase 7).
//
// The J1850 interface circuit and the ISO9141/K-line transceivers (TJA1027
// U7/U8) share ONE power net, driven by pin 49/PB5 (PS_J1850_9141, HIGH =
// rail on). Knowledge-pack facts (v1.4.0, VERIFIED):
//   conflicts C7:  "The two subsystems cannot be powered or low-powered
//                   independently — pulling pin 49 LOW kills both."
//   INV-PWR49:     "Pin 49 may be driven HIGH only after pin 50 (mode
//                   select) is settled, and may be driven LOW only when
//                   BOTH J1850 and ISO9141/K-line are quiesced."
//   INV-MODE50:    "Pin 50 must not change while J1850 traffic is active,
//                   and must be set before pin 49 is driven HIGH."
//
// This module owns the rail with a two-owner refcount so neither driver
// can brown out the other, and so the old j1850-only behavior (release()
// yanks pin 49 LOW unconditionally) cannot kill an active K-line session:
//
//   powerUp(o):   marks owner o as holding; drives pin 49 HIGH. The CALLER
//                 must have settled its owner-specific pins first (J1850:
//                 pin 50 mode select — INV-PWR49/INV-MODE50). Idempotent
//                 per owner.
//   powerDown(o): releases owner o; pin 49 goes LOW only when NO owner
//                 holds the rail (INV-PWR49). Idempotent per owner.
//
// Evidence: VERIFIED = compiles + pack citations above. INFERRED = rail
// settle timing (50 ms, carried over from the Phase 2 J1850 driver).

#include <Arduino.h>

namespace rail9141 {

enum class Owner : uint8_t { J1850 = 0, KLINE = 1 };

void powerUp(Owner o);
void powerDown(Owner o);   // idempotent per owner
bool held(Owner o);
bool powered();

} // namespace rail9141
