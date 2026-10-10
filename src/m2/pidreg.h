#pragma once
// M2 multi-mode firmware — data-driven OBD-II PID registry (Phase 9).
//
// Replaces the decodePid() switch with a table: PID, service mode,
// request size, byte extraction, endianness, signedness, scale, offset,
// unit, min/max, and decoder kind. Formulas per SAE J1979 (public
// standard — cited, never invented); all multi-byte values are
// big-endian (A*256+B), matching J1979.
//
// Integer math only (no float on the M2 path): LINEAR decoders compute
//   scaled = raw * num * 10^dec / den + off
// where `off`, `minV`, `maxV` are in 10^-dec units. dec = 0..3.
//
// SEAM FOR PHASE 12 (DBC engine): this registry is the scalar-PID half;
// the DBC signal-extraction engine (bit numbering, multiplexing, enums)
// will sit alongside it, not inside it.
//
// Evidence: VERIFIED = compiles, formulas cited to SAE J1979.
// INFERRED = untested against real ECUs (no bench).

#include <Arduino.h>
#include <stdint.h>

namespace pidreg {

enum class DecKind : uint8_t {
    LINEAR = 0,   // scaled numeric value + unit
    BITMASK = 1,  // payload bytes as hex (support masks, status words)
    ASCII = 2,    // payload as ASCII text (mode 09)
    RAW = 3,      // hex fallback (no decoder)
};

enum class Extract : uint8_t {
    A = 0,        // first payload byte
    B = 1,        // second payload byte
    AB = 2,       // big-endian 16-bit (A*256+B)
    ABCD = 3,     // big-endian 32-bit
    PAYLOAD = 4,  // whole payload (BITMASK/ASCII/RAW kinds)
};

struct PidEntry {
    uint8_t svc;        // service mode 0x01..0x0A
    uint8_t pid;
    uint8_t nBytes;     // expected payload bytes after the svc/pid echo
    Extract extract;
    bool isSigned;      // raw interpreted signed (AB/ABCD only)
    int32_t num;        // scale numerator
    int32_t den;        // scale denominator (1 = none)
    int32_t off;        // offset, in 10^-dec units
    uint8_t dec;        // decimals 0..3
    // m9 note (review): minV/maxV are informational only — decode()
    // does NOT enforce them (documented, not a range check).
    int32_t minV;       // informational range, 10^-dec units (INT32_MIN = n/a)
    int32_t maxV;       // (INT32_MAX = n/a)
    const char* unit;
    const char* name;
    DecKind kind;
};

// Lookup: exact (svc,pid); service 0x02 falls back to the 0x01 entry
// (freeze-frame PIDs share 01 formulas; the frame byte is the caller's).
const PidEntry* find(uint8_t svc, uint8_t pid);

// Decode payload `d` (bytes AFTER the svc/pid echo, length n) into `out`.
// Returns false when no entry exists (caller uses hex fallback).
bool decode(const PidEntry* e, const uint8_t* d, uint8_t n,
            char* out, size_t cap);

// Registry introspection (console/mm).
uint16_t count();
const PidEntry* at(uint16_t i);

} // namespace pidreg
