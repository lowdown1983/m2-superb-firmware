// M2 multi-mode firmware — ISO 15765-2 implementation (Phases 2+6).
// See isotp.h for the API contract and sources.
//
// Frame layout (ISO 15765-2, public standard), shown WITHOUT the
// optional address-extension byte; with EXTENDED/MIXED_11 addressing the
// AE byte sits at data[0] and every PCI field below shifts by one:
//   Single Frame:      data[0] = 0x0N (N = payload len), data[1..N]
//   First Frame:       data[0] = 0x10 | (len>>8), data[1] = len&0xFF, data[2..7]
//   Consecutive Frame: data[0] = 0x20 | (SN & 0xF), data[1..7]
//   Flow Control:      data[0] = 0x30 | FS (0=CTS,1=WAIT,2=OVFLW),
//                      data[1] = block size, data[2] = STmin
// Timeouts: N_Bs (TX waits for FC) and N_Cr (RX waits for CF) default to
// 1000 ms (the standard's defaults); both are configurable in Phase 6.
// STmin decode: 0x00-0x7F = 0-127 ms, 0xF1-0xF9 = 100-900 us, reserved = 0.

#include "isotp.h"
#include "busstat.h"
#include "busfilter.h"
#include "debug.h"

namespace isotp {

namespace {
constexpr uint8_t MAX_REASM = 4;   // concurrent senders (functional addressing)

// Global default config (Phase 6): the historical behavior, mutable via
// setDefault(). Used whenever a caller passes cfg == nullptr.
Config g_default;

struct Reasm {
    bool active = false;
    uint32_t sender = 0;
    uint16_t total = 0;
    uint16_t got = 0;
    uint8_t nextSn = 1;
    uint32_t lastCfAt = 0;   // N_Cr watchdog (Phase 6)
    uint8_t buf[ISOTP_BUF_MAX];
};

struct CfgView {
    AddrType addrType = AddrType::NORMAL_11;
    uint8_t addrExt = 0;
    uint16_t nBsMs = 1000;
    uint16_t nCrMs = 1000;
    uint16_t p2StarMs = 5000;
    uint8_t maxBlockSize = 0;
    uint8_t stMin = 0;
    bool autoFlowControl = true;
    uint16_t maxPayload = 64;
    const volatile bool* cancel = nullptr;
};

CfgView resolve(const Endpoint& ep, const Config* cfg)
{
    CfgView v;
    const Config* c = (cfg != nullptr) ? cfg : &g_default;
    v.addrType = c->addrType;
    if (v.addrType == AddrType::AUTO)
        v.addrType = ep.extended ? AddrType::NORMAL_29 : AddrType::NORMAL_11;
    v.addrExt = c->addrExt;
    v.nBsMs = c->nBsMs;
    v.nCrMs = c->nCrMs;
    v.p2StarMs = c->p2StarMs;
    v.maxBlockSize = c->maxBlockSize;
    v.stMin = c->stMin;
    v.autoFlowControl = c->autoFlowControl;
    if (c->maxPayload >= 8 && c->maxPayload <= ISOTP_BUF_MAX)
        v.maxPayload = c->maxPayload;
    v.cancel = c->cancel;
    return v;
}

// Address-extension offset: 1 when the AE byte leads the payload.
inline uint8_t aeOff(AddrType t)
{
    return (t == AddrType::EXTENDED || t == AddrType::MIXED_11) ? 1 : 0;
}

inline bool cancelled(const CfgView& v)
{
    return v.cancel != nullptr && *v.cancel;
}

inline busstat::Bus statBus(const Endpoint& ep)
{
    return (ep.bus == &Can1) ? busstat::Bus::CAN_BUS1 : busstat::Bus::CAN_BUS0;
}

bool sendCan(const Endpoint& ep, const uint8_t* d, uint8_t n)
{
    CAN_FRAME f;
    f.id = ep.txId;
    f.extended = ep.extended ? 1 : 0;
    f.length = 8;
    f.rtr = 0;
    for (uint8_t i = 0; i < 8; i++) f.data.bytes[i] = (i < n) ? d[i] : 0x00;
    bool ok = ep.bus->sendFrame(f);
    if (ok) busstat::noteTx(statBus(ep), n);
    else busstat::noteTxFail(statBus(ep));
    return ok;
}

// Wait for one CAN frame matching the endpoint; INFERRED timeout behavior.
bool waitFrame(const Endpoint& ep, CAN_FRAME& out, uint16_t timeoutMs,
               const CfgView& v)
{
    uint32_t start = millis();
    while ((uint32_t)(millis() - start) < timeoutMs) {
        if (cancelled(v)) return false;
        while (ep.bus->available() > 0) {
            CAN_FRAME f;
            ep.bus->read(f);
            if (ep.rxAnyId != 0 && f.id != ep.rxAnyId) continue;
            if (((bool)f.extended) != ep.extended) continue;
            busstat::noteRx(statBus(ep), f.length);
            out = f;
            return true;
        }
    }
    return false;
}

void drain(CANRaw* bus)
{
    CAN_FRAME f;
    while (bus->available() > 0) bus->read(f);
}

uint32_t stminToUs(uint8_t r)
{
    if (r <= 0x7F) return (uint32_t)r * 1000;
    if (r >= 0xF1 && r <= 0xF9) return (uint32_t)(r - 0xF0) * 100;
    return 0;  // reserved values treated as 0
}

// TX half of requestMulti, extracted for transmit-only use (Phase 4: STPX).
TxResult txPayload(const Endpoint& ep, const uint8_t* req, uint16_t reqLen,
                   const CfgView& v)
{
    if (reqLen == 0 || reqLen > v.maxPayload) return TxResult::SEND_FAIL;
    const uint8_t ao = aeOff(v.addrType);
    const uint8_t sfCap = (uint8_t)(7 - ao);
    const uint8_t ffFirst = (uint8_t)(6 - ao);
    const uint8_t cfCap = (uint8_t)(7 - ao);

    if (reqLen <= sfCap) {
        uint8_t d[8] = { 0,0,0,0,0,0,0,0 };
        if (ao) d[0] = v.addrExt;
        d[ao] = (uint8_t)reqLen;
        for (uint16_t i = 0; i < reqLen; i++) d[ao + 1 + i] = req[i];
        if (!sendCan(ep, d, 8)) return TxResult::SEND_FAIL;
        return TxResult::OK;
    }
    uint8_t d[8] = { 0,0,0,0,0,0,0,0 };
    if (ao) d[0] = v.addrExt;
    d[ao] = (uint8_t)(0x10 | ((reqLen >> 8) & 0x0F));
    d[ao + 1] = (uint8_t)(reqLen & 0xFF);
    for (uint8_t i = 0; i < ffFirst; i++) d[ao + 2 + i] = req[i];
    if (!sendCan(ep, d, 8)) return TxResult::SEND_FAIL;

    // Wait for Flow Control (N_Bs).
    uint8_t bs = 0;
    uint32_t stminUs = 0;
    {
        CAN_FRAME fc;
        bool gotFc = false;
        uint32_t bsStart = millis();
        while ((uint32_t)(millis() - bsStart) < v.nBsMs) {
            if (cancelled(v)) return TxResult::CANCELLED;
            if (!waitFrame(ep, fc, 100, v)) continue;
            uint8_t pci = fc.data.bytes[ao] >> 4;
            if (pci != 0x3) continue;  // not FC
            if (ao && fc.data.bytes[0] != v.addrExt) continue;
            uint8_t fs = fc.data.bytes[ao] & 0x0F;
            if (fs == 1) continue;        // WAIT: keep waiting (N_Bs window)
            if (fs == 2) return TxResult::FC_TIMEOUT;  // OVFLW
            gotFc = true;                 // CTS
            bs = fc.data.bytes[ao + 1];
            stminUs = stminToUs(fc.data.bytes[ao + 2]);
            break;
        }
        if (!gotFc) return TxResult::FC_TIMEOUT;
    }
    // Consecutive frames, SN = 1..F,0,1...
    uint8_t sn = 1;
    uint16_t off = ffFirst;
    while (off < reqLen) {
        uint8_t inBlock = 0;
        while (off < reqLen && (bs == 0 || inBlock < bs)) {
            if (cancelled(v)) return TxResult::CANCELLED;
            uint8_t chunk = (uint8_t)(((uint16_t)(reqLen - off) > cfCap)
                                     ? cfCap : (reqLen - off));
            uint8_t cf[8] = { 0,0,0,0,0,0,0,0 };
            if (ao) cf[0] = v.addrExt;
            cf[ao] = (uint8_t)(0x20 | (sn & 0x0F));
            for (uint8_t i = 0; i < chunk; i++) cf[ao + 1 + i] = req[off + i];
            if (!sendCan(ep, cf, 8)) return TxResult::SEND_FAIL;
            off += chunk;
            sn = (uint8_t)((sn + 1) & 0x0F);
            inBlock++;
            if (off < reqLen && stminUs > 0) {
                if (stminUs >= 1000) delay(stminUs / 1000);
                else delayMicroseconds(stminUs);
            }
        }
        if (off >= reqLen) break;
        // Block complete (BS > 0): wait for the next Flow Control (N_Bs).
        bool nextFc = false;
        uint32_t bsStart2 = millis();
        while ((uint32_t)(millis() - bsStart2) < v.nBsMs) {
            if (cancelled(v)) return TxResult::CANCELLED;
            CAN_FRAME fc2;
            if (!waitFrame(ep, fc2, 100, v)) continue;
            if ((fc2.data.bytes[ao] >> 4) != 0x3) continue;
            if (ao && fc2.data.bytes[0] != v.addrExt) continue;
            uint8_t fs2 = fc2.data.bytes[ao] & 0x0F;
            if (fs2 == 1) continue;                   // WAIT: keep waiting
            if (fs2 == 2) return TxResult::FC_TIMEOUT; // OVFLW
            bs = fc2.data.bytes[ao + 1];
            stminUs = stminToUs(fc2.data.bytes[ao + 2]);
            nextFc = true;                            // CTS
            break;
        }
        if (!nextFc) return TxResult::FC_TIMEOUT;
    }
    return TxResult::OK;
}

} // namespace

void setDefault(const Config& cfg)
{
    g_default = cfg;
}

void getDefault(Config& cfg)
{
    cfg = g_default;
}

TxResult sendFlowControl(const Endpoint& ep, uint8_t fs, uint8_t bs,
                         uint8_t stMin, const Config* cfg)
{
    CfgView v = resolve(ep, cfg);
    const uint8_t ao = aeOff(v.addrType);
    uint8_t d[8] = { 0,0,0,0,0,0,0,0 };
    if (ao) d[0] = v.addrExt;
    d[ao] = (uint8_t)(0x30 | (fs & 0x0F));
    d[ao + 1] = bs;
    d[ao + 2] = stMin;
    return sendCan(ep, d, 8) ? TxResult::OK : TxResult::SEND_FAIL;
}

TxResult transmit(const Endpoint& ep, const uint8_t* req, uint8_t reqLen,
                  const Config* cfg)
{
    drain(ep.bus);
    CfgView v = resolve(ep, cfg);
    return txPayload(ep, req, reqLen, v);
}

uint8_t requestMulti(const Endpoint& ep,
                     const uint8_t* req, uint8_t reqLen,
                     Response* out, uint8_t maxResp,
                     uint16_t timeoutMs, uint16_t quietMs,
                     const Config* cfg)
{
    if (maxResp == 0) return 0;
    CfgView v = resolve(ep, cfg);
    const uint8_t ao = aeOff(v.addrType);
    if (timeoutMs == 0) timeoutMs = v.p2StarMs;  // P2* window (Phase 6)
    drain(ep.bus);

    // ---- TX (shared with transmit()) ----
    TxResult tr = txPayload(ep, req, reqLen, v);
    if (tr != TxResult::OK) {
        if (tr == TxResult::CANCELLED)
            dbg::println(F("[isotp] TX cancelled"));
        return 0;
    }

    // ---- RX: reassemble, keyed by sender ID (functional addressing can
    // interleave several ECUs' FF/CF streams) ----
    Reasm r[MAX_REASM] = {};
    uint8_t nResp = 0;
    uint32_t start = millis();
    uint32_t lastActivity = start;
    const bool filtering = busfilter::count() > 0;
    const busstat::Bus sb = statBus(ep);

    while (nResp < maxResp && (uint32_t)(millis() - start) < timeoutMs) {
        if (cancelled(v)) {
            dbg::println(F("[isotp] RX cancelled"));
            break;  // return what was collected so far
        }
        if (nResp > 0 && quietMs > 0 &&
            (uint32_t)(millis() - lastActivity) >= quietMs) break;

        // N_Cr watchdog: drop stalled reassembly streams (their frames are
        // lost RX data).
        for (uint8_t i = 0; i < MAX_REASM; i++) {
            if (r[i].active &&
                (uint32_t)(millis() - r[i].lastCfAt) >= v.nCrMs) {
                r[i].active = false;
                busstat::noteRxOverflow(sb);
                dbg::println(F("[isotp] N_Cr timeout: stream dropped"));
            }
        }

        CAN_FRAME f;
        if (!waitFrame(ep, f, 25, v)) continue;

        // Unified filter table (Phase 6): consulted here, not in the FC
        // wait path — transport control frames always pass.
        if (filtering) {
            uint32_t faddr = ao ? f.data.bytes[0] : busfilter::ADDR_ANY;
            busfilter::Bus fb = (ep.bus == &Can1) ? busfilter::Bus::CAN_BUS1
                                                  : busfilter::Bus::CAN_BUS0;
            if (!busfilter::accept(fb, busfilter::Proto::CANBUS, faddr, f.id,
                                   busfilter::Dir::RX,
                                   f.data.bytes, f.length)) {
                busstat::noteDropped(sb);
                continue;
            }
        }
        lastActivity = millis();

        uint8_t pci = f.data.bytes[ao] >> 4;
        if (ao && f.data.bytes[0] != v.addrExt) continue;  // not our AE
        if (pci == 0x0) {  // Single Frame
            uint8_t n = f.data.bytes[ao] & 0x0F;
            if (n == 0 || n > (uint8_t)(7 - ao) || nResp >= maxResp) continue;
            out[nResp].senderId = f.id;
            out[nResp].len = n;
            for (uint8_t i = 0; i < n; i++)
                out[nResp].data[i] = f.data.bytes[ao + 1 + i];
            nResp++;
        } else if (pci == 0x1) {  // First Frame -> FC, open reassembly
            uint16_t total = (uint16_t)(((f.data.bytes[ao] & 0x0F) << 8) |
                                        f.data.bytes[ao + 1]);
            if (total < (uint8_t)(8 - ao) || total > v.maxPayload) {
                // Oversize for our cap: tell the sender (OVFLW) when we own
                // FC generation, then ignore the stream.
                if (v.autoFlowControl)
                    sendFlowControl(ep, 2, 0, 0, cfg);
                busstat::noteRxOverflow(sb);
                continue;
            }
            // Find or allocate a slot for this sender.
            Reasm* slot = nullptr;
            for (uint8_t i = 0; i < MAX_REASM; i++) {
                if (r[i].active && r[i].sender == f.id) { slot = &r[i]; break; }
            }
            if (!slot) {
                for (uint8_t i = 0; i < MAX_REASM; i++) {
                    if (!r[i].active) { slot = &r[i]; break; }
                }
            }
            if (!slot) {
                if (v.autoFlowControl)
                    sendFlowControl(ep, 2, 0, 0, cfg);
                busstat::noteRxOverflow(sb);
                continue;
            }
            slot->active = true; slot->sender = f.id;
            slot->total = total; slot->got = 0; slot->nextSn = 1;
            slot->lastCfAt = millis();
            uint8_t first = (total > (uint8_t)(6 - ao)) ? (uint8_t)(6 - ao)
                                                       : (uint8_t)total;
            for (uint8_t i = 0; i < first; i++)
                slot->buf[i] = f.data.bytes[ao + 2 + i];
            slot->got = first;
            // Flow Control: advertise OUR block size / STmin (Phase 6;
            // defaults preserve the old CTS/BS=0/STmin=0 behavior).
            if (v.autoFlowControl)
                sendFlowControl(ep, 0, v.maxBlockSize, v.stMin, cfg);
        } else if (pci == 0x2) {  // Consecutive Frame
            Reasm* slot = nullptr;
            for (uint8_t i = 0; i < MAX_REASM; i++) {
                if (r[i].active && r[i].sender == f.id) { slot = &r[i]; break; }
            }
            if (!slot) continue;
            uint8_t sn = f.data.bytes[ao] & 0x0F;
            if (sn != slot->nextSn) {
                slot->active = false;  // SN error: drop the stream
                busstat::noteRxOverflow(sb);
                continue;
            }
            slot->nextSn = (uint8_t)((slot->nextSn + 1) & 0x0F);
            slot->lastCfAt = millis();
            uint8_t chunk = (uint8_t)(((uint16_t)(slot->total - slot->got) > (uint8_t)(7 - ao))
                                     ? (uint8_t)(7 - ao)
                                     : (slot->total - slot->got));
            for (uint8_t i = 0; i < chunk; i++)
                slot->buf[slot->got + i] = f.data.bytes[ao + 1 + i];
            slot->got += chunk;
            if (slot->got >= slot->total && nResp < maxResp) {
                out[nResp].senderId = slot->sender;
                out[nResp].len = slot->total;
                for (uint16_t i = 0; i < slot->total; i++)
                    out[nResp].data[i] = slot->buf[i];
                nResp++;
                slot->active = false;
            }
        }
        // 0x3 (FC) addressed to us mid-RX is ignored here; TX path handled it.
    }
    return nResp;
}

} // namespace isotp
