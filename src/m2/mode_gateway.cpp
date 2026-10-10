// M2 multi-mode firmware — GATEWAY mode implementation (Phase 10).
// See mode_gateway.h for the contract and docs/SAFETY_REVIEW.md §12
// for the safety case.

#include "mode_gateway.h"
#include "mode_manager.h"  // modes::canBegin0/1, canEnd0/1
#include "mm_channel.h"    // mm::ST_* codes
#include "busfilter.h"     // shared idMatch() primitive
#include "busstat.h"       // per-bus stats feed
#include "capgate.h"       // MODIFY mappings need DIAGNOSTIC_WRITE
#include "swcan.h"         // Phase 8 hardened SWCAN driver
#include "debug.h"
#include <due_can.h>
#include <string.h>        // memcmp, memcpy

namespace gateway {

namespace {

// Audit fix M-2 (2026-10-08): mm control-channel divert. Without this, the
// web GUI's mode-switch button times out in GATEWAY mode — same pattern as
// MITM (mode_mitm.cpp).
mm::Divert divert;

struct Mapping {
    bool used = false;
    uint8_t srcBus = BUS_CAN0;
    uint32_t srcId = 0;
    uint32_t srcMask = 0;
    uint8_t srcExt = 0;
    uint8_t dstBus = BUS_SWCAN;
    uint32_t dstId = 0;
    uint8_t dstExt = 0;
    uint8_t action = ACT_PASS;
    uint8_t modOff = 0, modMask = 0, modVal = 0;
    uint8_t dropPolicy = POL_GLOBAL;
    // runtime:
    uint32_t hits = 0;
    uint32_t drops = 0;
};

struct Stats {
    uint32_t fwdCS = 0, fwdSC = 0;    // frames accepted into a TX queue
    uint32_t dropCS = 0, dropSC = 0;  // frames lost to the overflow policy
    uint32_t modCS = 0, modSC = 0;    // forwarded frames with MODIFY applied
};

struct QEntry {
    CAN_FRAME f;
};

Mapping maps[MAX_MAPS];
uint8_t nMaps = 0;
uint16_t tableVersion = 0;
bool suspendedFlag = false;

Stats stats;

// Per-direction TX queues (the ~15:1 rate adapter).
QEntry qCS[Q_CS];  // CAN0 -> SWCAN
uint8_t qCSHead = 0, qCSTail = 0, qCSCount = 0;
QEntry qSC[Q_SC];  // SWCAN -> CAN0
uint8_t qSCHead = 0, qSCTail = 0, qSCCount = 0;

bool active = false;

// Staged config (applies live when the mode is active).
uint32_t gCanBaud = 500000;     // HS-CAN side (GM diagnostics)
uint32_t gSwcanSpeed = 33333;   // SWCAN side (GM-LAN 33.33k)
uint8_t gDropPolicy = POL_DROP_OLDEST;  // global overflow default
uint8_t gDrainPerLoop = 4;      // TX queue drain cap per loop() iteration

// Arm-confirm state for MODIFY ("write") mappings — the same mechanism
// as UDS confirmGate: stage on first call, execute only when the
// IDENTICAL 21-byte payload is repeated within ARM_MS.
bool armPending = false;
uint32_t armAt = 0;
uint8_t armPayload[21];
constexpr uint32_t ARM_MS = 10000;

void promiscuous(CANRaw* bus)
{
    // Open all RX mailboxes (same pattern as ELM/M2RET/MITM).
    for (int m = 0; m < 7; m++) bus->setRXFilter(m, 0, 0, (m < 3));
}

void bumpVersion() { tableVersion++; }

uint8_t countMaps()
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < MAX_MAPS; i++) if (maps[i].used) n++;
    return n;
}

// Enqueue a translated frame on the destination bus's TX queue.
// Returns true when queued; false when dropped by policy (counted).
bool enqueue(uint8_t dstBus, const CAN_FRAME& f, Mapping& m, bool modified)
{
    uint8_t pol = (m.dropPolicy == POL_GLOBAL) ? gDropPolicy : m.dropPolicy;
    if (dstBus == BUS_SWCAN) {
        if (qCSCount >= Q_CS) {
            // Overflow: explicit policy, never silent.
            if (pol == POL_DROP_OLDEST) {
                qCSHead = (uint8_t)((qCSHead + 1) % Q_CS);
                qCSCount--;
            } else {
                m.drops++; stats.dropCS++;
                busstat::noteDropped(busstat::Bus::SWCAN);
                dbg::println(F("[gw] drop-newest CAN->SWCAN"));
                return false;
            }
            m.drops++; stats.dropCS++;
            busstat::noteDropped(busstat::Bus::SWCAN);
            dbg::println(F("[gw] drop-oldest CAN->SWCAN"));
        }
        qCS[qCSTail].f = f;
        qCSTail = (uint8_t)((qCSTail + 1) % Q_CS);
        qCSCount++;
        stats.fwdCS++;
        if (modified) stats.modCS++;
        return true;
    }
    // dstBus == BUS_CAN0
    if (qSCCount >= Q_SC) {
        if (pol == POL_DROP_OLDEST) {
            qSCHead = (uint8_t)((qSCHead + 1) % Q_SC);
            qSCCount--;
        } else {
            m.drops++; stats.dropSC++;
            busstat::noteDropped(busstat::Bus::CAN_BUS0);
            dbg::println(F("[gw] drop-newest SWCAN->CAN"));
            return false;
        }
        m.drops++; stats.dropSC++;
        busstat::noteDropped(busstat::Bus::CAN_BUS0);
        dbg::println(F("[gw] drop-oldest SWCAN->CAN"));
    }
    qSC[qSCTail].f = f;
    qSCTail = (uint8_t)((qSCTail + 1) % Q_SC);
    qSCCount++;
    stats.fwdSC++;
    if (modified) stats.modSC++;
    return true;
}

void handleRx(uint8_t srcBus, CAN_FRAME& f)
{
    // First matching mapping wins (MITM rule-engine convention).
    for (uint8_t i = 0; i < MAX_MAPS; i++) {
        Mapping& m = maps[i];
        if (!m.used || m.srcBus != srcBus) continue;
        if ((f.extended ? 1 : 0) != m.srcExt) continue;
        if (!busfilter::idMatch(f.id, m.srcId, m.srcMask)) continue;
        m.hits++;
        CAN_FRAME out = f;
        out.id = m.dstId;
        out.extended = (m.dstExt != 0);
        bool modified = false;
        if (m.action == ACT_MODIFY && m.modOff < out.length) {
            out.data.bytes[m.modOff] =
                (uint8_t)((out.data.bytes[m.modOff] & ~m.modMask) |
                          (m.modVal & m.modMask));
            modified = true;
        }
        enqueue(m.dstBus, out, m, modified);
        return;
    }
}

// NOTE: swcan::sendFrame feeds busstat::noteTx / noteTxFail itself —
// do NOT count here or every frame counts twice.
void sendSwcanFrame(CAN_FRAME& f)
{
    bool ok = swcan::sendFrame(f);
    if (!ok) {
        // Refused (mailboxes full / bus down): drop + count loudly.
        // No requeue — head-of-line blocking would stall the queue.
        stats.dropCS++;
        dbg::println(F("[gw] SWCAN TX refused, frame dropped"));
    }
}

void drainToSwcan()
{
    // Batch the 0x100 high-speed hop: the TH8056 Normal<->High-speed
    // transition costs up to 30 ms each way, so doing hsEnter/hsExit per
    // frame would block loop() ~60 ms per 0x100 frame and starve CAN0 RX
    // (review M2). Instead: pop FIFO, send non-HS frames immediately,
    // then send all HS frames under a single hsEnter/hsExit.
    // Ordering note: within one drain, non-HS frames go before HS
    // frames — a bounded, documented reorder of at most drainPerLoop.
    // GMLAN quirk itself VERIFIED (M2RET.ino:1052-1067).
    CAN_FRAME batch[Q_CS];
    CAN_FRAME hsBatch[Q_CS];
    uint8_t n = 0, nHs = 0;
    while (n < gDrainPerLoop && qCSCount > 0) {
        batch[n++] = qCS[qCSHead].f;
        qCSHead = (uint8_t)((qCSHead + 1) % Q_CS);
        qCSCount--;
    }
    for (uint8_t i = 0; i < n; i++) {
        if (batch[i].id == 0x100 && !batch[i].extended) hsBatch[nHs++] = batch[i];
        else sendSwcanFrame(batch[i]);
    }
    if (nHs > 0) {
        swcan::hsEnter();  // no-op when SWCAN not acquired (guarded inside)
        for (uint8_t i = 0; i < nHs; i++) sendSwcanFrame(hsBatch[i]);
        swcan::hsExit();
    }
}

void drainToCan()
{
    for (uint8_t i = 0; i < gDrainPerLoop && qSCCount > 0; i++) {
        CAN_FRAME f = qSC[qSCHead].f;
        qSCHead = (uint8_t)((qSCHead + 1) % Q_SC);
        qSCCount--;
        if (Can0.sendFrame(f)) {
            busstat::noteTx(busstat::Bus::CAN_BUS0, f.length);
        } else {
            stats.dropSC++;
            busstat::noteTxFail(busstat::Bus::CAN_BUS0);
            dbg::println(F("[gw] CAN0 TX refused, frame dropped"));
        }
    }
}

// Drop both TX queues, counting every frame (exit path: a mode switch
// must quiesce fast, not flush stale frames onto the bus).
// Review M1 fix: reset head AND tail (head==tail when count==0) —
// leaving a stale head would transmit a stale entry on the next use.
void dropQueues()
{
    while (qCSCount > 0) {
        qCSHead = (uint8_t)((qCSHead + 1) % Q_CS);
        qCSCount--;
        stats.dropCS++;
        busstat::noteDropped(busstat::Bus::SWCAN);
    }
    while (qSCCount > 0) {
        qSCHead = (uint8_t)((qSCHead + 1) % Q_SC);
        qSCCount--;
        stats.dropSC++;
        busstat::noteDropped(busstat::Bus::CAN_BUS0);
    }
    qCSHead = qCSTail = qCSCount = 0;
    qSCHead = qSCTail = qSCCount = 0;
}

void clearMaps()
{
    for (uint8_t i = 0; i < MAX_MAPS; i++) maps[i] = Mapping();
    nMaps = 0;
    armPending = false;
    bumpVersion();
}

// Validate a 21-byte map payload and fill a Mapping. Returns false on
// bad args.
bool payloadToMapping(const uint8_t* p, Mapping& m)
{
    uint8_t srcBus = p[0];
    uint32_t srcId = (uint32_t)p[1] | ((uint32_t)p[2] << 8) |
                     ((uint32_t)p[3] << 16) | ((uint32_t)p[4] << 24);
    uint32_t srcMask = (uint32_t)p[5] | ((uint32_t)p[6] << 8) |
                       ((uint32_t)p[7] << 16) | ((uint32_t)p[8] << 24);
    uint8_t srcExt = p[9];
    uint8_t dstBus = p[10];
    uint32_t dstId = (uint32_t)p[11] | ((uint32_t)p[12] << 8) |
                     ((uint32_t)p[13] << 16) | ((uint32_t)p[14] << 24);
    uint8_t dstExt = p[15];
    uint8_t action = p[16];
    if (srcBus > BUS_SWCAN || dstBus > BUS_SWCAN) return false;
    if (srcBus == dstBus) return false;  // a mapping must cross buses
    if (srcExt > 1 || dstExt > 1) return false;
    if (action > ACT_MODIFY) return false;
    if (p[17] > 7) return false;  // modOff within a CAN frame
    if (p[20] > POL_GLOBAL) return false;
    // 11-bit IDs must fit in 11 bits (a larger value could never match on
    // RX and would transmit a bogus ID on TX — reject loudly, not silently).
    if (!srcExt && srcId > 0x7FF) return false;
    if (!dstExt && dstId > 0x7FF) return false;
    if (!srcExt && (srcMask & ~0x7FFu)) return false;
    // Review m1: the raw mm path bypasses the console/web 29-bit clamp —
    // enforce it in the shared parser, not the callers.
    if (srcId > 0x1FFFFFFF || srcMask > 0x1FFFFFFF || dstId > 0x1FFFFFFF)
        return false;
    m.used = true;
    m.srcBus = srcBus; m.srcId = srcId; m.srcMask = srcMask; m.srcExt = srcExt;
    m.dstBus = dstBus; m.dstId = dstId; m.dstExt = dstExt;
    m.action = action;
    m.modOff = p[17]; m.modMask = p[18]; m.modVal = p[19];
    m.dropPolicy = p[20];
    m.hits = 0; m.drops = 0;
    return true;
}

// Shared add path for console + mm. On success clears the suspended
// flag (explicit re-enable). *confirm = true when the call was staged
// for arm-confirm (MODIFY) and needs an identical repeat.
uint8_t addFromPayload(const uint8_t* p, uint8_t& idx, bool& confirm)
{
    confirm = false;
    Mapping m;
    if (!payloadToMapping(p, m)) return mm::ST_BAD_ARG;
    // Review m2: fail fast on a full table BEFORE staging an arm-confirm,
    // so a MODIFY add on a full table doesn't ask for a useless repeat.
    if (nMaps >= MAX_MAPS) return mm::ST_ERR;
    if (m.action == ACT_MODIFY) {
        // Write mapping: DIAGNOSTIC_WRITE grant + arm-confirm (same
        // mechanism as UDS confirmGate — repeat identical within 10 s).
        if (!capgate::require(capgate::Grant::ALLOW_DIAGNOSTIC_WRITE,
                              "gateway modify mapping"))
            return mm::ST_ERR;
        uint32_t now = millis();
        if (armPending && (uint32_t)(now - armAt) < ARM_MS &&
            memcmp(armPayload, p, 21) == 0) {
            armPending = false;  // confirmed — fall through and add
        } else {
            armPending = true; armAt = now;
            memcpy(armPayload, p, 21);
            SerialUSB.println(F("WARNING: this mapping FABRICATES payload bytes on the destination bus."));
            SerialUSB.println(F("Repeat the IDENTICAL command within 10 s to install it."));
            dbg::println(F("[gw] modify-map staged, awaiting confirm"));
            confirm = true;
            idx = 0xFE;
            return mm::ST_OK;
        }
    } else {
        if (!capgate::require(capgate::Grant::ALLOW_TRANSMIT,
                              "gateway map add"))
            return mm::ST_ERR;
    }
    for (uint8_t i = 0; i < MAX_MAPS; i++) {
        if (!maps[i].used) {
            maps[i] = m;
            nMaps++;
            bumpVersion();
            suspendedFlag = false;  // explicit user action = re-enable
            idx = i;
            dbg::print(F("[gw] map add idx=")); dbg::print(idx);
            dbg::print(F(" v=")); dbg::println(tableVersion);
            return mm::ST_OK;
        }
    }
    return mm::ST_ERR;  // unreachable
}

void packMap(uint8_t i, uint8_t* out)
{
    const Mapping& m = maps[i];
    out[0] = i;
    out[1] = m.srcBus;
    out[2] = (uint8_t)(m.srcId & 0xFF); out[3] = (uint8_t)((m.srcId >> 8) & 0xFF);
    out[4] = (uint8_t)((m.srcId >> 16) & 0xFF); out[5] = (uint8_t)((m.srcId >> 24) & 0xFF);
    out[6] = (uint8_t)(m.srcMask & 0xFF); out[7] = (uint8_t)((m.srcMask >> 8) & 0xFF);
    out[8] = (uint8_t)((m.srcMask >> 16) & 0xFF); out[9] = (uint8_t)((m.srcMask >> 24) & 0xFF);
    out[10] = m.srcExt;
    out[11] = m.dstBus;
    out[12] = (uint8_t)(m.dstId & 0xFF); out[13] = (uint8_t)((m.dstId >> 8) & 0xFF);
    out[14] = (uint8_t)((m.dstId >> 16) & 0xFF); out[15] = (uint8_t)((m.dstId >> 24) & 0xFF);
    out[16] = m.dstExt;
    out[17] = m.action;
    out[18] = m.modOff; out[19] = m.modMask; out[20] = m.modVal;
    out[21] = m.dropPolicy;
}

void printMap(uint8_t i)
{
    const Mapping& m = maps[i];
    SerialUSB.print(F("  [")); SerialUSB.print(i); SerialUSB.print(F("] "));
    SerialUSB.print(m.srcBus == BUS_CAN0 ? F("can0:") : F("swcan:"));
    SerialUSB.print(F("0x")); SerialUSB.print(m.srcId, HEX);
    if (m.srcMask) { SerialUSB.print(F("/0x")); SerialUSB.print(m.srcMask, HEX); }
    if (m.srcExt) SerialUSB.print(F("/ext"));
    SerialUSB.print(F(" -> "));
    SerialUSB.print(m.dstBus == BUS_CAN0 ? F("can0:") : F("swcan:"));
    SerialUSB.print(F("0x")); SerialUSB.print(m.dstId, HEX);
    if (m.dstExt) SerialUSB.print(F("/ext"));
    SerialUSB.print(m.action == ACT_PASS ? F(" pass") : F(" modify"));
    if (m.action == ACT_MODIFY) {
        SerialUSB.print(F(" off=")); SerialUSB.print(m.modOff);
        SerialUSB.print(F(" mask=0x")); SerialUSB.print(m.modMask, HEX);
        SerialUSB.print(F(" val=0x")); SerialUSB.print(m.modVal, HEX);
    }
    SerialUSB.print(F(" policy="));
    SerialUSB.print(m.dropPolicy == POL_DROP_OLDEST ? F("oldest") :
                     m.dropPolicy == POL_DROP_NEWEST ? F("newest") : F("global"));
    SerialUSB.print(F(" hits=")); SerialUSB.print(m.hits);
    SerialUSB.print(F(" drops=")); SerialUSB.println(m.drops);
}

// Console token helpers.
bool parseBusTok(const char* t, uint8_t& b)
{
    if (!strcasecmp(t, "can0")) { b = BUS_CAN0; return true; }
    if (!strcasecmp(t, "swcan")) { b = BUS_SWCAN; return true; }
    return false;
}
bool parseHexTok(const char* t, uint32_t maxv, uint32_t& v)
{
    char* e; unsigned long x = strtoul(t, &e, 16);
    if (e == t || *e != '\0' || x > maxv) return false;
    v = (uint32_t)x;
    return true;
}
bool parseDecTok(const char* t, uint32_t maxv, uint32_t& v)
{
    char* e; unsigned long x = strtoul(t, &e, 10);
    if (e == t || *e != '\0' || x > maxv) return false;
    v = (uint32_t)x;
    return true;
}
void putU32(uint8_t* p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF); p[3] = (uint8_t)((v >> 24) & 0xFF);
}

// "gateway map add ..." — continues the strtok() sequence started by
// console(). Builds the 21-byte payload and runs the shared add path.
void conMapAdd()
{
    uint8_t p[21];
    uint8_t srcBus, dstBus, action;
    uint32_t srcId, srcMask, srcExt, dstId, dstExt;
    uint32_t modOff = 0, modMask = 0, modVal = 0;
    uint8_t pol = POL_GLOBAL;
    char* t;
    bool ok = true;
    t = strtok(nullptr, " ");
    ok = ok && t && parseBusTok(t, srcBus);
    t = strtok(nullptr, " ");
    ok = ok && t && parseHexTok(t, 0x1FFFFFFF, srcId);
    t = strtok(nullptr, " ");
    ok = ok && t && parseHexTok(t, 0x1FFFFFFF, srcMask);
    t = strtok(nullptr, " ");
    ok = ok && t && parseDecTok(t, 1, srcExt);
    t = strtok(nullptr, " ");
    ok = ok && t && parseBusTok(t, dstBus);
    t = strtok(nullptr, " ");
    ok = ok && t && parseHexTok(t, 0x1FFFFFFF, dstId);
    t = strtok(nullptr, " ");
    ok = ok && t && parseDecTok(t, 1, dstExt);
    t = strtok(nullptr, " ");
    if (!t) ok = false;
    else if (!strcasecmp(t, "pass")) action = ACT_PASS;
    else if (!strcasecmp(t, "modify")) action = ACT_MODIFY;
    else ok = false;
    if (ok && action == ACT_MODIFY) {
        t = strtok(nullptr, " ");
        ok = ok && t && parseHexTok(t, 7, modOff);
        t = strtok(nullptr, " ");
        ok = ok && t && parseHexTok(t, 0xFF, modMask);
        t = strtok(nullptr, " ");
        ok = ok && t && parseHexTok(t, 0xFF, modVal);
    }
    t = strtok(nullptr, " ");
    if (t) {
        if (!strcasecmp(t, "oldest")) pol = POL_DROP_OLDEST;
        else if (!strcasecmp(t, "newest")) pol = POL_DROP_NEWEST;
        else ok = false;
    }
    if (!ok) {
        SerialUSB.println(F("usage: gateway map add <srcBus:can0|swcan> <srcId:hex> <srcMask:hex> <srcExt:0|1>"));
        SerialUSB.println(F("                         <dstBus:can0|swcan> <dstId:hex> <dstExt:0|1> <pass|modify>"));
        SerialUSB.println(F("                         [modOff:hex modMask:hex modVal:hex] [oldest|newest]"));
        return;
    }
    p[0] = srcBus;
    p[1] = (uint8_t)(srcId & 0xFF); p[2] = (uint8_t)((srcId >> 8) & 0xFF);
    p[3] = (uint8_t)((srcId >> 16) & 0xFF); p[4] = (uint8_t)((srcId >> 24) & 0xFF);
    p[5] = (uint8_t)(srcMask & 0xFF); p[6] = (uint8_t)((srcMask >> 8) & 0xFF);
    p[7] = (uint8_t)((srcMask >> 16) & 0xFF); p[8] = (uint8_t)((srcMask >> 24) & 0xFF);
    p[9] = (uint8_t)srcExt;
    p[10] = dstBus;
    p[11] = (uint8_t)(dstId & 0xFF); p[12] = (uint8_t)((dstId >> 8) & 0xFF);
    p[13] = (uint8_t)((dstId >> 16) & 0xFF); p[14] = (uint8_t)((dstId >> 24) & 0xFF);
    p[15] = (uint8_t)dstExt;
    p[16] = action;
    p[17] = (uint8_t)modOff; p[18] = (uint8_t)modMask; p[19] = (uint8_t)modVal;
    p[20] = pol;
    uint8_t idx = 0xFF; bool confirm = false;
    uint8_t st = addFromPayload(p, idx, confirm);
    if (st != mm::ST_OK) { SerialUSB.println(F("[gateway] add refused (see above).")); return; }
    if (confirm) return;  // warning already printed
    SerialUSB.print(F("[gateway] mapping added at ["));
    SerialUSB.print(idx); SerialUSB.println(F("]"));
}

} // namespace (anonymous)

void begin()
{
    // Acquire BEFORE enable (ownership rule): CAN0 at the staged baud,
    // SWCAN at the staged speed. UART0/Bee was begun in setup() and is
    // never released. quiesceBuses() already handles both on switch.
    modes::canBegin0(gCanBaud);
    promiscuous(&Can0);
    if (!swcan::acquire(gSwcanSpeed)) {
        SerialUSB.println(F("[gateway] WARNING: SWCAN acquire failed; SWCAN legs will not forward."));
    }
    stats = Stats();
    qCSHead = qCSTail = qCSCount = 0;
    qSCHead = qSCTail = qSCCount = 0;
    active = true;
    // Mappings persist across mode switches (config, like MITM rules) —
    // but a PROG-entry suspend is never silently lifted.
    if (suspendedFlag) {
        SerialUSB.println(F("[gateway] suspended on PROG entry: add mappings to re-enable."));
    }
    SerialUSB.print(F("[gateway] up: CAN0@"));
    SerialUSB.print(gCanBaud);
    SerialUSB.print(F(" <-> SWCAN@"));
    SerialUSB.print(gSwcanSpeed);
    SerialUSB.print(F(", "));
    SerialUSB.print(nMaps);
    SerialUSB.println(F(" mapping(s)."));
    dbg::println(F("[gw] begin"));
}

void loop()
{
    if (!active) return;
    if (suspendedFlag) return;  // defense in depth: cleared table anyway
    // mm control-channel divert on Serial (supervision stays live; the
    // gateway data plane is CAN<->SWCAN, Serial carries only mm frames).
    while (Serial.available() > 0) {
        uint8_t b = (uint8_t)Serial.read();
        mm::divertByte(divert, b);
    }
    // RX: HS-CAN side.
    while (Can0.available() > 0) {
        CAN_FRAME f;
        Can0.read(f);
        busstat::noteRx(busstat::Bus::CAN_BUS0, f.length);
        handleRx(BUS_CAN0, f);
    }
    // RX: SWCAN side. NOTE: swcan::getFrame feeds busstat::noteRx itself
    // (and the shared busfilter table when non-empty, counting filtered
    // frames as dropped) — do NOT count here or every frame counts twice.
    {
        CAN_FRAME f;
        while (swcan::getFrame(f)) {
            handleRx(BUS_SWCAN, f);
        }
    }
    // TX: drain both queues with the per-loop cap (rate adaptation).
    drainToSwcan();
    drainToCan();
    swcan::poll();  // MCP2515 health + bus-off (Phase 8)
}

void exit()
{
    // Drain-or-drop on exit (documented): queued frames are dropped and
    // counted — a mode switch must quiesce fast, not flush stale frames.
    dropQueues();
    active = false;
    modes::canEnd0();
    swcan::release();
    dbg::println(F("[gw] exit"));
}

uint8_t suspend()
{
    uint8_t was = countMaps();
    clearMaps();  // bumps version, disarms any pending confirm
    suspendedFlag = true;
    SerialUSB.print(F("[gateway] PROG entry: mapping table auto-suspended, "));
    SerialUSB.print(was);
    SerialUSB.println(F(" mapping(s) cleared (NOT restored on exit)."));
    dbg::print(F("[gw] suspended, cleared=")); dbg::println(was);
    return was;
}

bool clearSuspended()
{
    bool was = suspendedFlag;
    suspendedFlag = false;
    return was;
}

bool suspended() { return suspendedFlag; }

uint8_t mmMapAdd(const uint8_t* p, uint8_t& idx)
{
    bool confirm = false;
    return addFromPayload(p, idx, confirm);
}

uint8_t mmMapDel(uint8_t idx, uint8_t& nMapsOut)
{
    if (idx == 0xFF) {
        clearMaps();
        nMapsOut = 0;
        dbg::println(F("[gw] map clear all"));
        return mm::ST_OK;
    }
    if (idx >= MAX_MAPS || !maps[idx].used) return mm::ST_BAD_ARG;
    maps[idx] = Mapping();
    nMaps = countMaps();
    bumpVersion();
    nMapsOut = nMaps;
    dbg::print(F("[gw] map del idx=")); dbg::println(idx);
    return mm::ST_OK;
}

uint8_t mmMapList(uint8_t start, uint8_t* out,
                  uint8_t& nTotal, uint8_t& nRet, uint16_t& version)
{
    nTotal = countMaps();
    nRet = 0;
    version = tableVersion;
    uint8_t seen = 0;
    for (uint8_t i = 0; i < MAX_MAPS && nRet < 1; i++) {
        if (!maps[i].used) continue;
        if (seen++ < start) continue;
        packMap(i, out + nRet * 22);
        nRet++;
    }
    return mm::ST_OK;
}

void mmStats(uint8_t* out)
{
    putU32(out, stats.fwdCS); putU32(out + 4, stats.fwdSC);
    putU32(out + 8, stats.dropCS); putU32(out + 12, stats.dropSC);
    putU32(out + 16, stats.modCS); putU32(out + 20, stats.modSC);
    out[24] = (uint8_t)(tableVersion & 0xFF);
    out[25] = (uint8_t)((tableVersion >> 8) & 0xFF);
    out[26] = nMaps;
    out[27] = suspendedFlag ? 1 : 0;
    out[28] = active ? 1 : 0;
}

uint8_t mmCfg(const uint8_t* p, uint8_t& applied)
{
    uint32_t canBaud = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    uint32_t swSpeed = (uint32_t)p[4] | ((uint32_t)p[5] << 8) |
                       ((uint32_t)p[6] << 16) | ((uint32_t)p[7] << 24);
    uint8_t pol = p[8], drain = p[9];
    if (pol > POL_DROP_NEWEST) return mm::ST_BAD_ARG;
    if (canBaud != 125000 && canBaud != 250000 && canBaud != 500000 &&
        canBaud != 1000000) return mm::ST_BAD_ARG;
    if (swSpeed != 33333 && swSpeed != 50000 && swSpeed != 83333 &&
        swSpeed != 100000) return mm::ST_BAD_ARG;
    if (drain == 0 || drain > 16) return mm::ST_BAD_ARG;
    gCanBaud = canBaud; gSwcanSpeed = swSpeed;
    gDropPolicy = pol; gDrainPerLoop = drain;
    applied = 0;
    if (active && !suspendedFlag) {
        // Live re-init (gateway active): safe — quiesce this mode's own
        // buses only, then re-acquire.
        modes::canEnd0();
        modes::canBegin0(gCanBaud);
        promiscuous(&Can0);
        swcan::reconfigure(gSwcanSpeed);
        applied = 1;
    }
    dbg::println(F("[gw] cfg updated"));
    return mm::ST_OK;
}

void mmCfgGet(uint8_t* out)
{
    putU32(out, gCanBaud); putU32(out + 4, gSwcanSpeed);
    out[8] = gDropPolicy; out[9] = gDrainPerLoop;
    out[10] = active ? 1 : 0; out[11] = suspendedFlag ? 1 : 0;
}

void console(char* line)
{
    while (*line == ' ') line++;
    char* tok = strtok(line, " ");
    if (!tok || !strcasecmp(tok, "status")) {
        SerialUSB.print(F("[gateway] active=")); SerialUSB.print(active ? F("yes") : F("no"));
        SerialUSB.print(F(" suspended=")); SerialUSB.print(suspendedFlag ? F("yes") : F("no"));
        SerialUSB.print(F(" table v")); SerialUSB.print(tableVersion);
        SerialUSB.print(F(" ")); SerialUSB.print(nMaps); SerialUSB.println(F(" mapping(s)"));
        SerialUSB.print(F("  CAN0@")); SerialUSB.print(gCanBaud);
        SerialUSB.print(F(" SWCAN@")); SerialUSB.print(gSwcanSpeed);
        SerialUSB.print(F(" policy=")); SerialUSB.print(gDropPolicy == POL_DROP_OLDEST ? F("drop-oldest") : F("drop-newest"));
        SerialUSB.print(F(" drain/loop=")); SerialUSB.println(gDrainPerLoop);
        SerialUSB.print(F("  fwd C->S=")); SerialUSB.print(stats.fwdCS);
        SerialUSB.print(F(" S->C=")); SerialUSB.print(stats.fwdSC);
        SerialUSB.print(F(" drop C->S=")); SerialUSB.print(stats.dropCS);
        SerialUSB.print(F(" S->C=")); SerialUSB.print(stats.dropSC);
        SerialUSB.print(F(" mod C->S=")); SerialUSB.print(stats.modCS);
        SerialUSB.print(F(" S->C=")); SerialUSB.println(stats.modSC);
        return;
    }
    if (!strcasecmp(tok, "enable")) {
        if (clearSuspended()) SerialUSB.println(F("[gateway] re-enabled: add mappings to forward."));
        else SerialUSB.println(F("[gateway] not suspended."));
        return;
    }
    if (!strcasecmp(tok, "map")) {
        tok = strtok(nullptr, " ");
        if (tok && !strcasecmp(tok, "list")) {
            SerialUSB.print(F("[gateway] table v")); SerialUSB.print(tableVersion);
            SerialUSB.print(F(" ")); SerialUSB.print(nMaps); SerialUSB.println(F(" mapping(s):"));
            for (uint8_t i = 0; i < MAX_MAPS; i++)
                if (maps[i].used) printMap(i);
            return;
        }
        if (tok && !strcasecmp(tok, "del")) {
            tok = strtok(nullptr, " ");
            if (!tok) { SerialUSB.println(F("usage: gateway map del <idx|all>")); return; }
            uint8_t nLeft = 0;
            uint8_t st;
            if (!strcasecmp(tok, "all")) st = mmMapDel(0xFF, nLeft);
            else {
                uint32_t idx;
                if (!parseDecTok(tok, MAX_MAPS - 1, idx)) {
                    SerialUSB.println(F("usage: gateway map del <idx|all>")); return;
                }
                st = mmMapDel((uint8_t)idx, nLeft);
            }
            SerialUSB.print(F("[gateway] del -> "));
            SerialUSB.print(st == mm::ST_OK ? F("ok") : F("ERR"));
            SerialUSB.print(F(", ")); SerialUSB.print(nLeft);
            SerialUSB.println(F(" left"));
            return;
        }
        if (tok && !strcasecmp(tok, "add")) {
            conMapAdd();
            return;
        }
        SerialUSB.println(F("usage: gateway map <list|add|del>"));
        return;
    }
    if (!strcasecmp(tok, "cfg")) {
        // gateway cfg [canbaud N] [swcanspeed N] [policy oldest|newest] [drain N]
        uint32_t canBaud = gCanBaud, swSpeed = gSwcanSpeed;
        uint8_t pol = gDropPolicy, drain = gDrainPerLoop;
        char* t;
        while ((t = strtok(nullptr, " ")) != nullptr) {
            if (!strcasecmp(t, "canbaud")) {
                t = strtok(nullptr, " "); uint32_t v;
                if (!t || !parseDecTok(t, 1000000, v)) goto cfgusage;
                canBaud = v;
            } else if (!strcasecmp(t, "swcanspeed")) {
                t = strtok(nullptr, " "); uint32_t v;
                if (!t || !parseDecTok(t, 100000, v)) goto cfgusage;
                swSpeed = v;
            } else if (!strcasecmp(t, "policy")) {
                t = strtok(nullptr, " ");
                if (!t) goto cfgusage;
                if (!strcasecmp(t, "oldest")) pol = POL_DROP_OLDEST;
                else if (!strcasecmp(t, "newest")) pol = POL_DROP_NEWEST;
                else goto cfgusage;
            } else if (!strcasecmp(t, "drain")) {
                t = strtok(nullptr, " "); uint32_t v;
                if (!t || !parseDecTok(t, 16, v) || v == 0) goto cfgusage;
                drain = (uint8_t)v;
            } else goto cfgusage;
        }
        {
            uint8_t p[10];
            putU32(p, canBaud); putU32(p + 4, swSpeed);
            p[8] = pol; p[9] = drain;
            uint8_t applied = 0;
            uint8_t st = mmCfg(p, applied);
            SerialUSB.print(F("[gateway] cfg -> "));
            SerialUSB.print(st == mm::ST_OK ? F("ok") : F("ERR"));
            SerialUSB.println(applied ? F(" (applied live)") : F(" (staged)"));
        }
        return;
    cfgusage:
        SerialUSB.println(F("usage: gateway cfg [canbaud N] [swcanspeed N] [policy oldest|newest] [drain N]"));
        return;
    }
    SerialUSB.println(F("usage: gateway <status|map list|map add|map del|enable|cfg>"));
}

} // namespace gateway
