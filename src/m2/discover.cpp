// M2 multi-mode firmware — vehicle discovery engine (Phase 9).
// INFERRED throughout (no bench); the Bolt EV is CAN-only so CAN paths
// can eventually be vehicle-tested.

#include "discover.h"
#include "mode_manager.h"  // modes::canBegin0/1, canEnd0/1
#include "mm_channel.h"
#include "isotp.h"
#include "busfilter.h"
#include "busstat.h"
#include "capgate.h"
#include "debug.h"
#include "oem_gen.h"
#include <due_can.h>

namespace discover {

namespace {

Result last;

// Candidate (baud, 29-bit?) combos, most-common first. INFERRED order.
const uint32_t BAUDS[] = { 500000, 250000, 125000 };
constexpr uint8_t N_BAUDS = 3;

// Cooperative cancel for long scans (console/mm can set it).
volatile bool cancelFlag = false;

void busRelease(uint8_t bus)
{
    if (bus == 0) modes::canEnd0(); else modes::canEnd1();
}

bool busAcquire(uint8_t bus, uint32_t baud)
{
    bool ok = (bus == 0) ? modes::canBegin0(baud) : modes::canBegin1(baud);
    if (!ok) return false;
    CANRaw* bp = (bus == 0) ? &Can0 : &Can1;
    for (int m = 0; m < 7; m++) bp->setRXFilter(m, 0, 0, (m < 3));
    return true;
}

isotp::Endpoint funcEndpoint(uint8_t bus, bool ext)
{
    isotp::Endpoint ep;
    ep.bus = (bus == 0) ? &Can0 : &Can1;
    ep.extended = ext;
    ep.txId = ext ? 0x18DB33F1UL : 0x7DF;
    ep.rxAnyId = 0;
    return ep;
}

isotp::Endpoint physEndpoint(uint8_t bus, bool ext, uint32_t txId, uint32_t rxId)
{
    isotp::Endpoint ep;
    ep.bus = (bus == 0) ? &Can0 : &Can1;
    ep.extended = ext;
    ep.txId = txId;
    ep.rxAnyId = rxId;
    return ep;
}

// One service probe. Returns true on any positive response.
bool probe(const isotp::Endpoint& ep, uint8_t svc,
           const uint8_t* extra, uint8_t extraLen, uint16_t timeoutMs = 400)
{
    uint8_t req[8] = { svc };
    for (uint8_t i = 0; i < extraLen && i + 1 < (uint8_t)sizeof(req); i++)
        req[1 + i] = extra[i];
    isotp::Response r;
    uint8_t n = isotp::requestMulti(ep, req, (uint8_t)(1 + extraLen),
                                    &r, 1, timeoutMs, 100, nullptr);
    if (n == 0 || r.len == 0) return false;
    return r.data[0] == (uint8_t)(svc + 0x40);  // positive SID
}

uint32_t txFor(uint32_t rxId, bool ext)
{
    if (!ext) return (rxId >= 8) ? rxId - 8 : 0;  // 0x7E8 -> 0x7E0
    // 29-bit UDS: ECU 0x18DA<ee>F1 <- tester 0x18DAF1<ee>
    uint8_t ee = (uint8_t)((rxId >> 8) & 0xFF);
    uint8_t tt = (uint8_t)(rxId & 0xFF);
    return 0x18DA0000UL | ((uint32_t)tt << 8) | ee;
}

// Read 0100/0120/.../01E0 support masks for one ECU.
void readPidSupport(const isotp::Endpoint& ep, EcuInfo& ecu)
{
    for (uint8_t w = 0; w < PID_MASK_WORDS; w++) {
        uint8_t pid = (uint8_t)(w * 0x20);
        uint8_t req[2] = { 0x01, pid };
        isotp::Response r;
        uint8_t n = isotp::requestMulti(ep, req, 2, &r, 1, 400, 100, nullptr);
        if (n > 0 && r.len >= 6 && r.data[0] == 0x41 && r.data[1] == pid) {
            for (uint8_t i = 0; i < 4 && 2 + i < r.len; i++)
                ecu.pidSup[w * 4 + i] = r.data[2 + i];
        } else {
            break;  // unsupported block -> higher blocks unsupported too
        }
        if (cancelFlag) break;
    }
}

// Service probes. DELIBERATELY never probes 04 (clear DTCs) or 08
// (control) — those change ECU state.
void probeServices(const isotp::Endpoint& ep, EcuInfo& ecu)
{
    const uint8_t svcs[] = { 0x01, 0x02, 0x03, 0x06, 0x07, 0x09, 0x0A };
    for (uint8_t i = 0; i < sizeof(svcs); i++) {
        uint8_t svc = svcs[i];
        bool ok = false;
        if (svc == 0x01) {
            uint8_t ex[1] = { 0x00 };
            ok = probe(ep, svc, ex, 1);
        } else if (svc == 0x09) {
            uint8_t ex[1] = { 0x00 };
            ok = probe(ep, svc, ex, 1);
        } else {
            ok = probe(ep, svc, nullptr, 0);
        }
        if (ok) ecu.services |= (1UL << svc);
        if (cancelFlag) break;
    }
}

// 0902 VIN (17 ASCII bytes over ISO-TP).
void readVin(const isotp::Endpoint& ep, EcuInfo& ecu)
{
    uint8_t req[2] = { 0x09, 0x02 };
    isotp::Response r;
    uint8_t n = isotp::requestMulti(ep, req, 2, &r, 1, 1000, 200, nullptr);
    if (n > 0 && r.len >= 19 && r.data[0] == 0x49 && r.data[1] == 0x02) {
        for (uint8_t i = 0; i < 17; i++) {
            char c = (char)r.data[2 + i];
            ecu.vin[i] = (c >= 32 && c < 127) ? c : '?';
        }
        ecu.vin[17] = '\0';
        ecu.vinValid = true;
    }
}

} // namespace (anonymous)

bool run(uint8_t bus, Result& out)
{
    out = Result();
    if (bus > 1) return false;
    if (!capgate::require(capgate::Grant::ALLOW_TRANSMIT, "discover")) return false;
    cancelFlag = false;
    dbg::println(F("[discover] scan start"));

    // Phase 1: baud/format detect via functional 0100.
    bool locked = false;
    for (uint8_t b = 0; b < N_BAUDS && !locked; b++) {
        for (uint8_t e = 0; e < 2 && !locked; e++) {
            bool ext = (e == 1);
            busRelease(bus);
            if (!busAcquire(bus, BAUDS[b])) continue;
            dbg::print(F("[discover] trying ")); dbg::print(BAUDS[b]);
            dbg::println(ext ? F(" 29-bit") : F(" 11-bit"));
            uint8_t req[2] = { 0x01, 0x00 };
            isotp::Response rs[8];
            uint8_t n = isotp::requestMulti(funcEndpoint(bus, ext), req, 2,
                                            rs, 8, 500, 150, nullptr);
            if (n > 0) {
                locked = true;
                out.baud = BAUDS[b];
                out.ext = ext;
                out.bus = bus;
                // Phase 2: one EcuInfo per responder.
                for (uint8_t i = 0; i < n && out.nEcus < MAX_ECUS; i++) {
                    EcuInfo& ecu = out.ecus[out.nEcus++];
                    ecu.rxId = rs[i].senderId;
                    ecu.txId = txFor(ecu.rxId, ext);
                    ecu.nodeAddr = (uint8_t)(ecu.rxId & 0xFF);
                }
            }
            if (cancelFlag) break;
        }
        if (cancelFlag) break;
    }
    if (!locked) {
        busRelease(bus);
        last = out;
        dbg::println(F("[discover] no ECU answered"));
        return false;
    }

    // Phase 3: per-ECU capability.
    for (uint8_t i = 0; i < out.nEcus; i++) {
        EcuInfo& ecu = out.ecus[i];
        isotp::Endpoint ep = physEndpoint(bus, out.ext, ecu.txId, ecu.rxId);
        dbg::print(F("[discover] ECU 0x")); dbg::println(ecu.rxId, HEX);
        readPidSupport(ep, ecu);
        probeServices(ep, ecu);
        if (ecu.services & (1UL << 0x09)) readVin(ep, ecu);
        if (cancelFlag) break;
    }
    out.done = true;
    busRelease(bus);
    last = out;
    dbg::println(F("[discover] scan complete"));
    return out.nEcus > 0;
}

const Result& lastResult() { return last; }

void cancel() { cancelFlag = true; }

void print(const Result& r)
{
    if (!r.done || r.nEcus == 0) {
        SerialUSB.println(F("No discovery result (no ECU answered)."));
        return;
    }
    SerialUSB.print(F("Discovery: CAN")); SerialUSB.print(r.bus);
    SerialUSB.print(F(" @ ")); SerialUSB.print(r.baud);
    SerialUSB.println(r.ext ? F(" 29-bit") : F(" 11-bit"));
    for (uint8_t i = 0; i < r.nEcus; i++) {
        const EcuInfo& e = r.ecus[i];
        SerialUSB.print(F("  ECU 0x")); SerialUSB.print(String(e.rxId, HEX));
        SerialUSB.print(F(" (TX 0x")); SerialUSB.print(String(e.txId, HEX));
        SerialUSB.print(F(") services:"));
        for (uint8_t s = 1; s <= 10; s++)
            if (e.services & (1UL << s)) { SerialUSB.print(' '); SerialUSB.print(s, HEX); }
        if (e.vinValid) { SerialUSB.print(F(" VIN ")); SerialUSB.print(e.vin); }
        SerialUSB.println();
    }
}

bool applyFilters(const Result& r)
{
    if (!r.done || r.nEcus == 0) return false;
    busfilter::clear();
    for (uint8_t i = 0; i < r.nEcus; i++) {
        const EcuInfo& e = r.ecus[i];
        busfilter::Filter f;
        f.bus = (r.bus == 0) ? busfilter::Bus::CAN_BUS0 : busfilter::Bus::CAN_BUS1;
        f.proto = busfilter::Proto::CANBUS;
        f.addr = e.nodeAddr;  // Phase 6 OEM seam: ECU address byte
        f.id = e.rxId;
        f.mask = r.ext ? 0x1FFFFFFFUL : 0x7FFUL;
        f.dir = busfilter::Dir::RX;
        if (busfilter::add(f) < 0) return false;
    }
    return true;
}

// Paged ECU list for the mm channel: 11B/entry, 4 per reply (48-byte cap):
// [txId LE32][rxId LE32][nSvcs][vinOk][nodeAddr].
void listPage(const Result& r, uint8_t startIdx, uint8_t* out,
              uint8_t& nTotal, uint8_t& nRet)
{
    nTotal = r.done ? r.nEcus : 0;
    nRet = 0;
    for (uint8_t i = startIdx; i < r.nEcus && nRet < 4; i++, nRet++) {
        const EcuInfo& e = r.ecus[i];
        uint8_t* p = out + nRet * 11;
        p[0] = (uint8_t)(e.txId & 0xFF);
        p[1] = (uint8_t)((e.txId >> 8) & 0xFF);
        p[2] = (uint8_t)((e.txId >> 16) & 0xFF);
        p[3] = (uint8_t)((e.txId >> 24) & 0xFF);
        p[4] = (uint8_t)(e.rxId & 0xFF);
        p[5] = (uint8_t)((e.rxId >> 8) & 0xFF);
        p[6] = (uint8_t)((e.rxId >> 16) & 0xFF);
        p[7] = (uint8_t)((e.rxId >> 24) & 0xFF);
        uint8_t nSvcs = 0;
        for (uint8_t s = 1; s <= 10; s++)
            if (e.services & (1UL << s)) nSvcs++;
        p[8] = nSvcs;
        p[9] = e.vinValid ? 1 : 0;
        p[10] = e.nodeAddr;
    }
}

} // namespace discover
