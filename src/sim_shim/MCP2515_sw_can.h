#pragma once
// sim_shim/MCP2515_sw_can.h — host-native SWcan stand-in for [env:sim_m2].
//
// Same constructor/method signatures the firmware uses
// (SWcan(cs,intPin); setupSW(speed); mode(m); sendFrame(); GetRXFrame()).
// Backed by a virtual loopback bus like SimCAN (INFERRED — no MCP2515,
// no SPI, no single-wire transceiver in sim).

#include <Arduino.h>
#include <due_can.h>

class SWcan {
public:
    SWcan(uint8_t cs, uint8_t intPin) : cs_(cs), intPin_(intPin),
                                        speed_(0), mode_(0) {}

    void setupSW(unsigned long speed) { speed_ = speed; }  // recorded (INFERRED)
    void mode(byte m) { mode_ = m; }                        // recorded (INFERRED)
    void intHandler() {}  // no-op: no MCP2515 interrupt line in sim (INFERRED)

    bool sendFrame(CAN_FRAME& frame) {  // loopback, see due_can.h note
        frame.timestamp = millis();
        rxq_.push_back(frame);
        return true;
    }
    bool GetRXFrame(CAN_FRAME& frame) {
        if (rxq_.empty()) return false;
        frame = rxq_.front();
        rxq_.pop_front();
        return true;
    }

    // Test hook (deferred vehicle-behavior sim will drive this).
    void injectFrame(const CAN_FRAME& f) { rxq_.push_back(f); }

    unsigned long speed() const { return speed_; }
    byte modeState() const { return mode_; }

private:
    uint8_t cs_, intPin_;
    unsigned long speed_;
    byte mode_;
    std::deque<CAN_FRAME> rxq_;
};
