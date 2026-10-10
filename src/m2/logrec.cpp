// M2 multi-mode firmware — diagnostic session logging (Phase 12).
// See logrec.h.

#include "logrec.h"
#include "debug.h"
#include <string.h>

namespace logrec {
namespace {

uint8_t ring[RING_N * REC_BYTES];
uint8_t head = 0;      // next write slot
uint8_t count = 0;     // buffered records
bool capturing = false;

bool trigOn = false;
uint8_t trigKind = 0xFF;
uint16_t trigSvc = 0xFFFF;
uint8_t preN = 16;     // pre-trigger keep (INFERRED default)
uint16_t postN = 0;    // post-trigger then auto-stop (0 = continuous)
uint16_t postLeft = 0;
bool triggered = false;

uint32_t nProduced = 0;
uint32_t nDropped = 0;
uint32_t nStreamed = 0;

void packRec(uint8_t* dst, uint32_t tsMs, uint8_t bus, uint8_t dir,
             uint8_t kind, uint8_t ecuAddr, uint16_t svc, int32_t val,
             const uint8_t* raw, uint8_t rawLen)
{
    dst[0] = (uint8_t)(tsMs & 0xFF);
    dst[1] = (uint8_t)((tsMs >> 8) & 0xFF);
    dst[2] = (uint8_t)((tsMs >> 16) & 0xFF);
    dst[3] = (uint8_t)((tsMs >> 24) & 0xFF);
    dst[4] = bus;
    dst[5] = dir;
    dst[6] = kind;
    dst[7] = ecuAddr;
    dst[8] = (uint8_t)(svc & 0xFF);
    dst[9] = (uint8_t)((svc >> 8) & 0xFF);
    dst[10] = (uint8_t)(val & 0xFF);
    dst[11] = (uint8_t)((val >> 8) & 0xFF);
    dst[12] = (uint8_t)((val >> 16) & 0xFF);
    dst[13] = (uint8_t)((val >> 24) & 0xFF);
    if (rawLen > 8) rawLen = 8;
    dst[14] = rawLen;
    for (uint8_t i = 0; i < 8; i++)
        dst[15 + i] = (raw && i < rawLen) ? raw[i] : 0;
}

bool trigMatch(uint8_t kind, uint16_t svc)
{
    if (!trigOn) return true;
    if (trigKind != 0xFF && trigKind != kind) return false;
    if (trigSvc != 0xFFFF && trigSvc != svc) return false;
    return true;
}

} // namespace (anonymous)

void start()
{
    capturing = true;
    triggered = false;
    postLeft = postN;
    dbg::println(F("[logrec] capture started"));
}

void stop()
{
    capturing = false;
    dbg::println(F("[logrec] capture stopped"));
}

void clear()
{
    head = 0;
    count = 0;
    nProduced = 0;
    nDropped = 0;
    nStreamed = 0;
    triggered = false;
}

bool active() { return capturing; }

void setTrigger(uint8_t kind, uint16_t svc)
{
    trigOn = true;
    trigKind = kind;
    trigSvc = svc;
}

void clearTrigger() { trigOn = false; }

void setPrePost(uint8_t pre, uint16_t post)
{
    preN = (pre > RING_N) ? RING_N : pre;
    postN = post;
}

void record(uint8_t bus, uint8_t dir, uint8_t kind, uint8_t ecuAddr,
            uint16_t svc, int32_t val, const uint8_t* raw, uint8_t rawLen)
{
    if (!capturing) return;
    if (!trigMatch(kind, svc)) return;

    // Post-trigger accounting: on first match, keep only the last preN.
    if (trigOn && !triggered) {
        triggered = true;
        if (count > preN) {
            // Drop the oldest (count - preN): advance the tail implicitly
            // by reducing count (head stays; oldest overwritten next).
            nDropped += (count - preN);
            count = preN;
        }
        postLeft = postN;
    }
    if (triggered && postN > 0) {
        if (postLeft == 0) {
            capturing = false;  // auto-stop after post window
            dbg::println(F("[logrec] post-trigger window done, stopped"));
            return;
        }
        postLeft--;
    }

    uint8_t* dst = ring + head * REC_BYTES;
    packRec(dst, millis(), bus, dir, kind, ecuAddr, svc, val, raw, rawLen);
    head = (head + 1) % RING_N;
    nProduced++;
    if (count < RING_N) {
        count++;
    } else {
        nDropped++;  // overwrite oldest; never silent
    }
}

uint8_t poll(uint8_t* out)
{
    if (!out || count == 0) return 0;
    uint8_t n = (count >= PER_POLL) ? PER_POLL : count;
    // Oldest first: tail = (head - count + RING_N) % RING_N.
    uint8_t tail = (head + RING_N - count) % RING_N;
    for (uint8_t i = 0; i < n; i++) {
        uint8_t slot = (tail + i) % RING_N;
        memcpy(out + i * REC_BYTES, ring + slot * REC_BYTES, REC_BYTES);
    }
    count -= n;
    nStreamed += n;
    return n;
}

uint32_t produced() { return nProduced; }
uint32_t dropped() { return nDropped; }
uint32_t streamed() { return nStreamed; }
uint16_t pending() { return count; }

void status()
{
    SerialUSB.println(F("--- log capture ---"));
    SerialUSB.print(F("active: "));
    SerialUSB.println(capturing ? F("yes") : F("no"));
    SerialUSB.print(F("produced/dropped/streamed/pending: "));
    SerialUSB.print(nProduced);
    SerialUSB.print(F("/"));
    SerialUSB.print(nDropped);
    SerialUSB.print(F("/"));
    SerialUSB.print(nStreamed);
    SerialUSB.print(F("/"));
    SerialUSB.println(count);
    if (trigOn) {
        SerialUSB.print(F("trigger: kind="));
        SerialUSB.print(trigKind);
        SerialUSB.print(F(" svc=0x"));
        SerialUSB.println(trigSvc, HEX);
    }
}

} // namespace logrec
