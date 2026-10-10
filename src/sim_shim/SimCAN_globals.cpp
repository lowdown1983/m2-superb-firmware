// sim_shim/SimCAN_globals.cpp — Can0/Can1 instances + sendFrame bus model.
#include "due_can.h"
#include "vehicle_ecu.h"

CANRaw Can0;
CANRaw Can1;

bool CANRaw::sendFrame(CAN_FRAME& frame)
{
    if (!begun_) return false;
    frame.timestamp = millis();

    // Cross-bridge CAN0↔CAN1 (physical loopback wire for HARNESS).
    // Do NOT echo onto the transmitting controller's own RX queue — that
    // would make ISO-TP see the tester's request as an ECU response.
    if (this == &Can0) {
        if (Can1.begun()) Can1.injectFrame(frame);
    } else if (this == &Can1) {
        if (Can0.begun()) Can0.injectFrame(frame);
    }

    // Scripted vehicle: may injectFrame() a response onto *this.
    vehicle_ecu::onTx(this, frame);
    return true;
}
