#pragma once
// M2 multi-mode firmware — unified bus filter model (Phase 6).
//
// ONE filter implementation for every mode, sitting below OBD/UDS:
//     bus -> protocol -> address -> ID -> mask -> direction -> payload mask
//
// Modes register what they want to see; accept() answers for any frame.
// Empty table = accept everything (so every mode's behavior is unchanged
// until someone registers a filter). The shared idMatch() primitive is
// also used by the MITM rule engine and the ELM/STN monitor + STCFCPA
// paths, so there is exactly one ID/mask implementation in the firmware.
//
// SEAM FOR PHASE 9/12 (OEM database): the `addr` field is the protocol
// address slot the OEM database will fill — ISO-TP target/source address
// (extended/mixed addressing AE byte), ECU physical address, or J1850
// node byte. Phase 6 only plumbs it; Phase 9's PID/DID registries and
// vehicle discovery engine will populate it from the OEM database's
// diagnostic-access data without a transport refactor.
//
// Evidence: VERIFIED = compiles. INFERRED = filter semantics untested on
// hardware (no bench).

#include <Arduino.h>

namespace busfilter {

enum class Bus : uint8_t {
    CAN_BUS0 = 0,
    CAN_BUS1 = 1,
    SWCAN = 2,   // reserved: SWCAN driver reports via its own path (Phase 8)
    J1850 = 3,   // reserved: same
    KLINE = 4,   // reserved: Phase 7
    ANY = 0xFF,
};

enum class Proto : uint8_t {
    ANY = 0,
    CANBUS = 1,
    SWCAN = 2,
    J1850_VPW = 3,
    J1850_PWM = 4,
    KLINE = 5,
};

enum class Dir : uint8_t {
    RX = 0,
    TX = 1,
    BOTH = 2,
};

// Sentinel: matches any protocol address.
constexpr uint32_t ADDR_ANY = 0xFFFFFFFF;

struct Filter {
    bool used = false;
    Bus bus = Bus::ANY;
    Proto proto = Proto::ANY;
    uint32_t addr = ADDR_ANY;  // protocol address (see seam note above)
    uint32_t id = 0;           // CAN ID (or J1850 header byte in id's low 8)
    uint32_t mask = 0;         // 0 = any ID; else ((id ^ f.id) & mask) == 0
    Dir dir = Dir::BOTH;
    uint8_t payOff = 0;        // payload byte offset for the payload condition
    uint8_t payMask = 0;       // 0 = no payload condition
    uint8_t payVal = 0;        // ((payload[payOff] ^ payVal) & payMask) == 0
    bool invert = false;       // true: frame passes when the rule does NOT match
};

constexpr uint8_t MAX_FILTERS = 16;

// Shared ID/mask primitive — the ONLY id/mask implementation. Used by
// accept(), the MITM rule engine, and the ELM/STN monitor + STCFCPA paths.
inline bool idMatch(uint32_t id, uint32_t rule, uint32_t mask)
{
    return mask == 0 || (((id ^ rule) & mask) == 0);
}

// Full single-filter match (all specified fields must match).
bool matchOne(const Filter& f, Bus bus, Proto proto, uint32_t addr,
              uint32_t id, Dir dir, const uint8_t* pl, uint8_t pln);

// OR over the table: true when the table is empty or any filter matches.
bool accept(Bus bus, Proto proto, uint32_t addr, uint32_t id, Dir dir,
            const uint8_t* pl, uint8_t pln);

// Table management.
int8_t add(const Filter& f);   // >=0 table index, -1 when full
bool remove(uint8_t idx);
void clear();
uint8_t count();
const Filter* get(uint8_t idx);

// Pack one filter as 20 bytes LE:
// [idx][bus][proto][addr LE32][id LE32][mask LE32][dir]
// [payOff][payMask][payVal][invert]. Returns 20, or 0 on bad idx.
uint8_t pack(uint8_t idx, uint8_t* out);

} // namespace busfilter
