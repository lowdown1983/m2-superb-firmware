#pragma once
// sim_shim/due_can.h — host-native due_can stand-in for [env:sim_m2].
//
// CAN_FRAME / BytesUnion are copied VERBATIM from collin80/can_common
// (can_common.h, the same dependency the real [env:m2] build uses), so the
// field layout the firmware touches (.id/.extended/.length/.rtr and
// .data.bytes[]) is identical.
//
// Bus model (2026-10-10 Step 2):
//   - CAN0↔CAN1 cross-bridged on sendFrame (HARNESS physical loopback wire).
//   - Same-controller TX is NOT echoed to own RX (avoids ISO-TP self-poison).
//   - vehicle_ecu::onTx() may injectFrame() scripted ECU responses.
// That is NOT a real multi-node vehicle bus (INFERRED timing/electrical).

#include <Arduino.h>

// ---- verbatim from can_common.h (collin80) ----
class BitRef
{
public:
    BitRef& operator=( bool x )
    {
        *byteRef = (*byteRef & ~(1 << bitPos));
        if (x) *byteRef = *byteRef | (1 << bitPos);
        return *this;
    }
    operator bool() const
    {
        if (*byteRef & (1 << bitPos)) return true;
        return false;
    }
public:
    BitRef(uint8_t *ref, int pos)
    {
        byteRef = ref;
        bitPos = pos;
    }
private:
    uint8_t *byteRef;
    int bitPos;
};

typedef union {
    uint64_t uint64;
    uint32_t uint32[2];
    uint16_t uint16[4];
    uint8_t  uint8[8];
    int64_t int64;
    int32_t int32[2];
    int16_t int16[4];
    int8_t  int8[8];
    uint64_t value;
    struct {
        uint32_t low;
        uint32_t high;
    };
    struct {
        uint16_t s0;
        uint16_t s1;
        uint16_t s2;
        uint16_t s3;
    };
    uint8_t bytes[8];
    uint8_t byte[8];
    struct {
        uint8_t bitField[8];
        const bool operator[]( int pos ) const
        {
            if (pos < 0 || pos > 63) return 0;
            int bitFieldIdx = pos / 8;
            return (bitField[bitFieldIdx] >> pos) & 1;
        }
        BitRef operator[]( int pos )
        {
            if (pos < 0 || pos > 63) return BitRef((uint8_t *)&bitField[0], 0);
            uint8_t *ptr = (uint8_t *)&bitField[0];
            return BitRef(ptr + (pos / 8), pos & 7);
        }
    } bit;
} BytesUnion;

class CAN_FRAME
{
public:
    CAN_FRAME() : id(0), fid(0), timestamp(0), rtr(0), priority(0),
                  extended(0), length(0) { memset(data.bytes, 0, 8); }

    BytesUnion data;
    uint32_t id;
    uint32_t fid;
    uint32_t timestamp;
    uint8_t rtr;
    uint8_t priority;
    uint8_t extended;
    uint8_t length;
};
// ---- end verbatim ----

// Virtual CAN controller. Named CANRaw to match real due_can.
//
// Bus model (Step 2, 2026-10-10):
//   - CAN0↔CAN1 are cross-bridged on sendFrame (simulates a physical
//     loopback wire for HARNESS selfcheck). Same-controller TX is NOT
//     echoed onto that controller's RX (echo would poison ISO-TP RX with
//     the tester's own request).
//   - vehicle_ecu::onTx() may injectFrame() a scripted ECU response onto
//     the transmitting controller (real-car topology: request and reply
//     share the same CAN interface).
class CANRaw {
public:
    CANRaw() : begun_(false), baud_(0), listenOnly_(false) {}

    uint32_t begin(uint32_t baudrate, uint8_t /*enPin*/) {
        begun_ = true; baud_ = baudrate; return baudrate;  // INFERRED: instant
    }
    uint32_t begin(uint32_t baudrate) { return begin(baudrate, 255); }
    void disable() { begun_ = false; rxq_.clear(); }
    bool isBegun() const { return begun_; }

    int setRXFilter(uint8_t /*mailbox*/, uint32_t /*id*/, uint32_t /*mask*/,
                    bool /*extended*/) { return 0; }  // accept-all (INFERRED)
    int setRXFilter(uint32_t id, uint32_t mask, bool ext) {
        return setRXFilter(0, id, mask, ext);
    }

    uint16_t available() { return (uint16_t)rxq_.size(); }

    uint32_t read(CAN_FRAME& msg) {
        if (rxq_.empty()) return 0;
        msg = rxq_.front();
        rxq_.pop_front();
        return 1;
    }

    // Implemented out-of-line in SimCAN_globals.cpp (needs Can0/Can1 + ECU).
    bool sendFrame(CAN_FRAME& frame);

    void enable_autobaud_listen_mode() { listenOnly_ = true; }
    void disable_autobaud_listen_mode() { listenOnly_ = false; }
    // Shim gap fixed 2026-10-08: autodet.cpp uses the due_can 2.1.6
    // setListenOnlyMode() API; the shim only had the older autobaud pair.
    void setListenOnlyMode(bool en) { listenOnly_ = en; }

    // Test hook: inject a frame as if it arrived from the vehicle.
    void injectFrame(const CAN_FRAME& f) { rxq_.push_back(f); }

    bool begun() const { return begun_; }

private:
    bool begun_;
    uint32_t baud_;
    bool listenOnly_;
    std::deque<CAN_FRAME> rxq_;
};

extern CANRaw Can0;
extern CANRaw Can1;
