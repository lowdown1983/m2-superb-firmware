#pragma once
// M2 multi-mode firmware — vehicle profiles (Phase 12).
//
// ONE persisted vehicle profile: versioned + CRC32-protected in the
// flashstore A/B slots (the SAM3X8E has no real EEPROM — see
// flashstore.h for the honest backing-store documentation).
//
// A profile round-trips the Phase 6/9/10 knobs:
//  - bus config: CAN0/CAN1 baud, SWCAN speed, protocol, frame format
//  - ISO-TP default config (the 15-byte CMD_ISOTP_CFG layout)
//  - busfilter table (up to 8 entries, 20-byte packed form)
//  - gateway mapping table (up to 8 entries, 21-byte packed form)
//  - diagnostic addressing (ECU physical address / AE byte — the OEM seam)
//  - retry limits, transmit-permission ceiling (capgate grant), log mode
//
// Multi-profile directories are a trivial extension (more page pairs);
// Phase 12 persists the single active profile.
//
// Evidence: VERIFIED = compiles. INFERRED = flash-write path untested
// on silicon (flashstore); profile apply semantics untested on a vehicle.

#include <Arduino.h>
#include <stdint.h>

namespace profiles {

constexpr uint8_t PROFILE_VERSION = 1;
constexpr uint8_t MAX_PFILTERS = 8;   // persisted filter entries
constexpr uint8_t MAX_PGWMAPS = 8;    // persisted gateway mappings

// Protocol encoding (matches busfilter::Proto where overlapping).
enum : uint8_t {
    PROTO_CAN = 0,
    PROTO_J1850_VPW = 1,
    PROTO_J1850_PWM = 2,
    PROTO_KLINE = 3,
    PROTO_SWCAN = 4,
};

#pragma pack(push, 1)
struct Profile {
    uint8_t version;            // PROFILE_VERSION
    char name[16];              // NUL-terminated vehicle/profile name
    uint32_t can0Baud;          // LE32
    uint32_t can1Baud;          // LE32
    uint32_t swcanSpeed;        // LE32
    uint8_t proto;              // PROTO_* above
    uint8_t frameFmt;           // bit0 = 29-bit, bit1 = listen-only default
    uint8_t diagAddr;           // ECU physical address / AE byte (OEM seam)
    uint8_t isotpCfg[15];       // packed ISO-TP default (CMD_ISOTP_CFG layout)
    uint8_t nFilters;           // 0..MAX_PFILTERS
    uint8_t filters[MAX_PFILTERS][20];  // busfilter::pack() form
    uint8_t nGwMaps;            // 0..MAX_PGWMAPS
    uint8_t gwMaps[MAX_PGWMAPS][21];    // gateway 21-byte form
    uint8_t txPerm;             // capgate ceiling grant (0..4)
    uint8_t logMode;            // 0=off, 1=on-demand, 2=always
    uint8_t retryLim;           // diagnostic retry limit
    uint8_t reserved;
};
#pragma pack(pop)

// init(): flashstore::init() + load the stored profile into RAM (if any).
void init();
// has(): a valid stored profile exists. name(): its name (or "").
bool has();
const char* name();
// save(): capture current live settings into the RAM profile and persist
// to flash (explicit only — never background). Returns false on bad args
// or flash failure.
bool save(const char* name);
// load(): apply the stored profile to the live settings (staged where a
// mode owns the bus; documented per-field in the .cpp). Returns false when
// no valid profile is stored.
bool load();
// clearProfile(): invalidate the stored profile (writes a tombstone).
void clearProfile();
// show(): human-readable dump on SerialUSB.
void show();
// Accessors for the mm channel.
bool info(char* outName, uint8_t& version, uint32_t& writes);

} // namespace profiles
