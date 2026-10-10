// M2 multi-mode firmware — DBC signal engine (Phase 12). See dbcsig.h.

#include "dbcsig.h"
#include "debug.h"
#include <stdint.h>
#include <string.h>

namespace dbcsig {
namespace {

int8_t activeDbc = -1;  // F-03: -1 = all DBCs (legacy first-match)

// Highest byte index a signal's bit span touches. Used for the
// fail-closed truncated-frame check: a signal whose span reaches past
// the received frame is skipped, never decoded from partial bits.
// Motorola: startBit = MSB, sawtooth walk toward LSB (mirrors
// extractRaw below). Intel: linear from the LSB.
uint8_t spanMaxByte(const dbc::Sig& s)
{
    if (s.bitLen == 0) return (uint8_t)(s.startBit / 8);
    if (s.flags & 1) {  // Motorola
        int pos = (int)s.startBit;
        int mx = pos / 8;
        for (uint8_t i = 1; i < s.bitLen; i++) {
            if (pos % 8 == 0) pos += 15;
            else pos--;
            if (pos < 0 || pos >= 64) break;  // malformed table: stop
            int b = pos / 8;
            if (b > mx) mx = b;
        }
        return (uint8_t)mx;
    }
    // Intel
    return (uint8_t)(((int)s.startBit + (int)s.bitLen - 1) / 8);
}

// Skips counted by the most recent decode() call (reset on entry).
uint16_t truncSkips = 0;

// Raw bit extraction. Motorola: startBit = MSB, sawtooth walk toward
// LSB (bit0 of byte N -> bit7 of byte N+1, i.e. pos += 15 on wrap).
// Intel: startBit = LSB, linear walk up.
uint32_t extractRaw(const dbc::Sig& s, const uint8_t* data, uint8_t dlen)
{
    uint32_t raw = 0;
    if (s.flags & 1) {  // Motorola
        int pos = (int)s.startBit;
        for (uint8_t i = 0; i < s.bitLen; i++) {
            int byte = pos / 8, bit = pos % 8;
            if (byte >= 0 && byte < dlen && (data[byte] & (1u << bit)))
                raw |= (1UL << (s.bitLen - 1 - i));
            if (bit == 0) pos += 15;
            else pos--;
            if (pos < 0 || pos >= 64) break;  // malformed table: stop
        }
    } else {  // Intel
        for (uint8_t i = 0; i < s.bitLen; i++) {
            int pos = (int)s.startBit + i;
            int byte = pos / 8, bit = pos % 8;
            if (byte >= dlen) break;
            if (data[byte] & (1u << bit))
                raw |= (1UL << i);
        }
    }
    // Sign-extend.
    if ((s.flags & 2) && s.bitLen < 32 && (raw & (1UL << (s.bitLen - 1))))
        raw |= 0xFFFFFFFFUL << s.bitLen;
    return raw;
}

int msgIndexById(uint32_t id, bool ext)
{
    for (uint16_t i = 0; i < dbc::MSG_N; i++) {
        // F-03: when a DBC profile is selected, shadowed duplicate IDs
        // from other DBCs are invisible to the lookup.
        if (activeDbc >= 0 && dbc::MSG[i].dbc != (uint8_t)activeDbc) continue;
        if (dbc::MSG[i].id == id && (bool)dbc::MSG[i].ext == ext)
            return (int)i;
    }
    return -1;
}

const char* enumFor(uint16_t sigIdx, int32_t raw)
{
    for (uint16_t i = 0; i < dbc::ENUMVAL_N; i++) {
        if (dbc::ENUMVAL[i].sigIdx == sigIdx && dbc::ENUMVAL[i].val == raw)
            return dbc::ENUMVAL[i].desc;
    }
    return nullptr;
}

} // namespace (anonymous)

// F-03: public DBC-profile selector (declared in dbcsig.h). Lives outside
// the anonymous namespace; msgIndexById() above reads activeDbc.
void selectDbc(int8_t idx)
{
    if (idx < -1 || idx >= (int8_t)dbc::DBC_N) return;  // ignore nonsense
    activeDbc = idx;
}

int8_t selectedDbc() { return activeDbc; }

uint16_t decode(uint32_t id, bool ext, const uint8_t* data, uint8_t len,
                void (*cb)(const SignalVal& v, void* ctx), void* ctx)
{
    if (!data) return 0;  // cb may be null: count-only mode
    truncSkips = 0;  // per-call skip counter (see truncatedSkips())
    int mi = msgIndexById(id, ext);
    if (mi < 0) return 0;
    const dbc::Msg& m = dbc::MSG[mi];

    // First pass: find the multiplexer value (one level only).
    int32_t muxVal = 0;
    bool haveMux = false;
    for (uint16_t k = 0; k < m.nSig; k++) {
        const dbc::Sig& s = dbc::SIG[m.sigBase + k];
        if (s.flags & 4) {
            // Fail closed: a truncated multiplexer yields no mux value.
            if (spanMaxByte(s) < len) {
                muxVal = (int32_t)extractRaw(s, data, len);
                haveMux = true;
            } else {
                truncSkips++;
            }
            break;
        }
    }

    uint16_t n = 0;
    for (uint16_t k = 0; k < m.nSig; k++) {
        const dbc::Sig& s = dbc::SIG[m.sigBase + k];
        if ((s.flags & 4)) continue;  // multiplexer itself: not a value
        if ((s.flags & 8)) {          // multiplexed: mux must match
            if (!haveMux || muxVal != s.muxVal) continue;
        }
        // Fail closed: never publish a value decoded from a frame shorter
        // than the signal's bit span (missing bits would read as zero and
        // produce plausible-but-wrong telemetry).
        if (spanMaxByte(s) >= len) { truncSkips++; continue; }
        uint32_t raw = extractRaw(s, data, len);
        // F-02: phys1000 is physical value * 1000 (milli-units), matching
        // the SignalVal contract and the fixed-point formatter. The old
        // formula divided back down to plain physical units (1000x too
        // small, fractional precision destroyed). Saturate on narrowing:
        // a pathological factor could exceed int32 milli-units.
        int64_t p1000 = (int64_t)(int32_t)raw * (int64_t)s.factor1000
                      + (int64_t)s.offset1000;
        if (p1000 > INT32_MAX) p1000 = INT32_MAX;
        else if (p1000 < INT32_MIN) p1000 = INT32_MIN;
        int64_t phys1000 = p1000;
        if (cb) {
            SignalVal v;
            v.name = s.name;
            v.phys1000 = (int32_t)phys1000;
            v.raw = (int32_t)raw;
            v.unit = s.unit;
            v.en = enumFor(m.sigBase + k, (int32_t)raw);
            cb(v, ctx);
        }
        n++;
    }
    return n;
}

bool find(uint32_t id, bool ext, const char* sigName,
          const dbc::Sig*& sigOut, uint16_t& msgIdxOut)
{
    int mi = msgIndexById(id, ext);
    if (mi < 0 || !sigName) return false;
    const dbc::Msg& m = dbc::MSG[mi];
    for (uint16_t k = 0; k < m.nSig; k++) {
        if (strncmp(dbc::SIG[m.sigBase + k].name, sigName, 16) == 0) {
            sigOut = &dbc::SIG[m.sigBase + k];
            msgIdxOut = (uint16_t)mi;
            return true;
        }
    }
    return false;
}

uint16_t msgCount() { return dbc::MSG_N; }
uint16_t sigCount() { return dbc::SIG_N; }
uint16_t truncatedSkips() { return truncSkips; }
uint16_t enumCount() { return dbc::ENUMVAL_N; }
uint8_t dbcCount() { return dbc::DBC_N; }
const char* dbcName(uint8_t i)
{
    return (i < dbc::DBC_N) ? dbc::DBC_NAMES[i] : "?";
}

bool msgInfo(uint16_t idx, uint32_t& id, uint8_t& ext, uint8_t& dlc,
             uint16_t& nSig)
{
    if (idx >= dbc::MSG_N) return false;
    id = dbc::MSG[idx].id;
    ext = dbc::MSG[idx].ext;
    dlc = dbc::MSG[idx].dlc;
    nSig = dbc::MSG[idx].nSig;
    return true;
}

} // namespace dbcsig
