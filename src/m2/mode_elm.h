#pragma once
// M2 multi-mode firmware — ELM327 mode (Phase 2).
//
// A genuine-protocol ELM327 implementation (not an emulator): AT commands are
// answered in real ELM327 response formats (prompt '>', OK, '?', NO DATA,
// STOPPED), and OBD-II requests perform REAL bus transactions — CAN via the
// shared ISO-TP module (isotp.h), J1850 PWM/VPW via j1850.h. Anything the
// bus doesn't answer within the ATST timeout returns NO DATA.
//
// Transport: Serial (Bee UART @115200, immutable) — the SuperB bridges app
// traffic (BT SPP / WiFi TCP) to it. The multimode control channel
// (F1 7E ...) is diverted in the byte pump before line assembly, so the
// SuperB's web GUI can supervise this mode without polluting the AT stream.
//
// AT SUBSET: the commands Torque/FORScan actually use (see mode_elm.cpp).
// Response-format conventions follow the ELM327 datasheet behavior and
// Collin Kidder's ELM327_Emulator (M2RET tree) where the datasheet is
// ambiguous; deviations are marked INFERRED.
//
// Resource claims: CAN0 (for CAN protocols), J1850 block (for protocols
// 1/2, only if FEAT_J1850 enabled). exit() releases before the manager's
// quiesceBuses() runs.

#include <Arduino.h>

namespace elm {

void begin();   // reset ELM state to power-on defaults; enter mode
void loop();    // pump Serial (AT lines + mm-channel divert)
void exit();    // release bus resources (idempotent)

// Feature gate from the mode manager (mm SET_FEATURE / web GUI).
void setJ1850Enabled(bool en);

// Phase 4: STN extension. STN mode reuses this ELM core with ST-command
// parsing enabled (genuine STN chips are ELM327 supersets — FRPM:
// "fully compatible with the ELM327 AT command set"). All AT commands
// keep working; ST commands are dispatched in stCommand() (mode_elm.cpp).
// The STN subset boundary is documented in docs/PHASE4_NOTES.md.
void setStnMode(bool en);

} // namespace elm
