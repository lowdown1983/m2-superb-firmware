#pragma once
// sim_shim/vehicle_ecu.h — scripted virtual ECU for host-native sim.
//
// Driven from CANRaw::sendFrame(): OBD Mode 01 and UDS services are answered
// via injectFrame(). Multi-frame responses (e.g. VIN DID 0xF190) use FF +
// CF after the tester's Flow Control (ISO 15765-2):
//   - BS=0  → send all remaining CFs after one CTS FC
//   - BS=N  → send at most N CFs, then wait for the next FC
//   - FS=1/2 (Wait/Overflow) → abort, no further CFs
//   - STmin is parsed and stored but treated as 0 (no PHY timing in sim)
//   - 11-bit (7E0/7DF) and 29-bit (18DA10F1/18DB33F1) FC both accepted
//
// Runtime knobs (sim console, intercepted in SerialUSB stdio path — no
// src/m2 changes):
//   vehicle set  rpm|speed|coolant|throttle <value>
//   vehicle ramp rpm|speed|coolant|throttle <from> <to> <steps>
//   vehicle show
//   can inject <0|1> <idHex> <dataHex>   // RX only (MITM/tests)
//   can tx <0|1> <idHex> <dataHex>       // sendFrame → vehicle_ecu onTx
//   can rx <0|1>                         // drain/print RX queue
// Ramp advances one step each time that signal is read (OBD PID request).
// Purpose: encoding-boundary sweeps, not a car model.
//
// Evidence: INFERRED vehicle values. Paths VERIFIED in sim pytest.

#include "due_can.h"

namespace vehicle_ecu {

void onTx(CANRaw* bus, const CAN_FRAME& tx);
void setEnabled(bool on);
bool enabled();

// Console intercept for `vehicle ...` and `can inject ...`. Returns true
// if consumed (caller must NOT feed the line to the firmware).
bool handleConsoleLine(const char* line,
                       void (*reply)(const char* msg, void* ctx),
                       void* ctx);

}  // namespace vehicle_ecu
