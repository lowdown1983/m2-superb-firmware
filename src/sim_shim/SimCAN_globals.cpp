// sim_shim/SimCAN_globals.cpp — Can0/Can1 instances + sendFrame bus model.
#include "due_can.h"
#include "vehicle_ecu.h"

CANRaw Can0;
CANRaw Can1;

bool CANRaw::sendFrame(CAN_FRAME& frame)
{
    if (!begun_) return false;
    frame.timestamp = millis();

    // Dual independent buses (MITM topology). No Can0↔Can1 cross-bridge:
    // that re-injected TX onto the peer controller and made MITM re-forward
    // forever. HARNESS physical loopback is not modeled here — use
    // `can inject` or vehicle_ecu for peer traffic.
    // Do NOT echo onto the transmitting controller's own RX queue (ISO-TP).

    // Scripted vehicle: may injectFrame() a response onto *this.
    vehicle_ecu::onTx(this, frame);
    return true;
}
