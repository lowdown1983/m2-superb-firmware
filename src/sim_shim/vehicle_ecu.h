#pragma once
// sim_shim/vehicle_ecu.h — scripted virtual ECU for host-native sim.
//
// Driven from CANRaw::sendFrame(): OBD Mode 01 and UDS services are answered
// via injectFrame(). Multi-frame responses (e.g. VIN DID 0xF190) use FF +
// CF after the tester's Flow Control.
//
// Runtime knobs (sim console, intercepted in SerialUSB stdio path — no
// src/m2 changes):
//   vehicle set  rpm|speed|coolant|throttle <value>
//   vehicle ramp rpm|speed|coolant|throttle <from> <to> <steps>
//   vehicle show
// Ramp advances one step each time that signal is read (OBD PID request).
// Purpose: encoding-boundary sweeps, not a car model.
//
// Evidence: INFERRED vehicle values. Paths VERIFIED in sim pytest.

#include "due_can.h"

namespace vehicle_ecu {

void onTx(CANRaw* bus, const CAN_FRAME& tx);
void setEnabled(bool on);
bool enabled();

// Console intercept. Returns true if `line` was a vehicle command (caller
// must NOT feed it to the firmware). Writes a human reply via `reply`
// (may be called zero or more times; each string is one line without '\n').
bool handleConsoleLine(const char* line,
                       void (*reply)(const char* msg, void* ctx),
                       void* ctx);

}  // namespace vehicle_ecu
