#pragma once
// sim_shim/vehicle_ecu.h — scripted virtual ECU for host-native sim.
//
// Driven from CANRaw::sendFrame(): OBD Mode 01 and UDS services are answered
// via injectFrame(). Multi-frame responses (e.g. VIN DID 0xF190) use FF +
// CF after the tester's Flow Control.
//
// UDS coverage (INFERRED static data):
//   0x10 DiagnosticSessionControl (01/02/03)
//   0x22 ReadDataByIdentifier (F186, F187/F188/F18A/F197, F190 VIN)
//   0x3E TesterPresent
//   0x19 ReadDTCInformation (02 → empty list)
//   0x27 SecurityAccess (seed stub / key reject)
//   other SIDs → NRC 0x11
//
// Evidence: INFERRED vehicle values. Proves firmware request/response paths
// in sim; does NOT prove silicon timing or a real ECU.

#include "due_can.h"

namespace vehicle_ecu {

void onTx(CANRaw* bus, const CAN_FRAME& tx);
void setEnabled(bool on);
bool enabled();

}  // namespace vehicle_ecu
