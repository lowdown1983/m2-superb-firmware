// M2 multi-mode firmware — mode manager implementation (Phase 1).
//
// See mode_manager.h for the acquire/release contract. Hardware facts used
// here and their evidence:
//
// - Can0/Can1 are due_can 2.1.6 globals (due_can.cpp:1470-1471). Their
//   constructor enable pins default to 50/48 (due_can.h:43-44). On the M2,
//   Arduino 50 = J1850_PWM_VPW and 48 = 5V buck-boost control (variant.cpp
//   comments, lines 229/233) — driving them would corrupt J1850/power state.
//   THEREFORE: every begin() call MUST pass 255 as the enable pin
//   (CAN_COMMON::begin(baud, enPin) sets enablePin then init(); due_can
//   init() skips pin driving when enablePin == 255 — due_can.cpp:267-269).
//   This mirrors M2RET itself (M2RET.ino:320: Can0.begin(speed, 255)).
//   VERIFIED from primary sources (variant + due_can source).
// - CANRaw::disable() clears CAN_MR_CANEN (due_can.cpp:458-463) — the
//   controller goes to init/off state. But it ALSO does
//   `if (enablePin != 255) digitalWrite(enablePin, LOW)` (due_can.cpp:462),
//   so disable() is only safe AFTER a begin(...,255) has set enablePin=255.
//   The manager tracks canBegun to enforce this. VERIFIED from source.
// - SWCAN sleep: SWcan::mode(0) drives SWC_M0 LOW + SWC_M1 LOW = TH8056
//   Sleep (MCP2515_sw_can.cpp:49-52, Macchina Single-Wire-CAN-mcp2515).
//   VERIFIED from primary source. mode(3) = Normal (M0/M1 HIGH, HIGH).
// - J1850 power: PS_J1850_9141 (49/PB5) HIGH = power on, LOW = off
//   (Macchina Docs.md J1850 setup snippet). Quiesce drives it LOW.
// - Button1 = Arduino 22 (variant.h:151), active LOW per Macchina's
//   SuperB passthrough sketch.

#include "mode_manager.h"
#include "debug.h"
#include "mode_m2ret.h"
#include "mode_elm.h"
#include "mode_stn.h"
#include "mode_uds.h"
#include "mode_mitm.h"
#include "mode_obd2.h"
#include "mode_prog.h"
#include "mode_gateway.h"  // Phase 10: CAN0 <-> SWCAN gateway mode
#include "mode_slcan.h"    // Phase 12: Lawicel ASCII transport
#include "vharness.h"      // Phase 12: virtual ECU self-test harness
#include "autodet.h"       // Phase 12: auto-detection engine
#include "profiles.h"      // Phase 12: vehicle profiles
#include "logrec.h"        // Phase 12: diagnostic session logging
#include "dbcsig.h"        // Phase 12: DBC signal engine
#include "mm_channel.h"
#include "j1850.h"
#include "kline.h"  // Phase 7: ISO 9141-2 / KWP2000
#include "swcan.h"  // Phase 8: hardened SWCAN driver
#include "busstat.h"    // Phase 6: per-bus stats + presence tracking
#include "busfilter.h"  // Phase 6: unified filter table
#include "txsched.h"    // Phase 6: TX scheduler
#include "isotp.h"      // Phase 6: default ISO-TP config
#include "capgate.h"    // Phase 9: capability gates
#include "discover.h"   // Phase 9: discovery console/mm
#include <due_can.h>

// due_can CANRaw::disable() is only pin-safe after begin(...,255) on THAT
// controller (see file header: raw disable() would drive Arduino 50/48 =
// J1850_PWM_VPW / 5V buck-boost). Phase 3 made the flags per-controller:
// UDS/OBD2 can use CAN1 while CAN0 was never begun, and a global flag would
// wrongly authorize Can0.disable() on a never-begun controller.
static bool can0Begun = false;
static bool can1Begun = false;

namespace modes {

// Phase 12: global manager accessor (profiles need staged settings).
static ModeManager* gManager = nullptr;
ModeManager* manager() { return gManager; }

const char* modeName(Mode m)
{
    switch (m) {
        case Mode::M2RET:  return "m2ret";
        case Mode::ELM327: return "elm327";
        case Mode::STN:    return "stn";
        case Mode::UDS:    return "uds";
        case Mode::MITM:   return "mitm";
        case Mode::OBD2:   return "obd2";
        case Mode::PROG:   return "prog";
        case Mode::GATEWAY: return "gateway";
        case Mode::SLCAN:  return "slcan";
        case Mode::HARNESS: return "harness";
        default:           return "?";
    }
}

bool modeFromName(const char* name, Mode& out)
{
    for (uint8_t i = 0; i < (uint8_t)Mode::COUNT; i++) {
        Mode m = (Mode)i;
        if (strcasecmp(name, modeName(m)) == 0) { out = m; return true; }
    }
    return false;
}

bool modeImplemented(Mode m)
{
    return m == Mode::M2RET || m == Mode::ELM327 || m == Mode::STN ||
           m == Mode::UDS || m == Mode::MITM || m == Mode::OBD2 ||
           m == Mode::PROG || m == Mode::GATEWAY ||
           m == Mode::SLCAN || m == Mode::HARNESS;  // Phase 5,10; Phase 12
}

// ---- mm control-channel ops (registered in begin()) ----
// The manager instance for the C-style ops table. Set in begin(); the
// firmware creates exactly one ModeManager (main.cpp).
static ModeManager* mmOwner = nullptr;

static bool mmRequestMode(uint8_t idx)
{
    if (mmOwner == nullptr || idx >= (uint8_t)Mode::COUNT) return false;
    // SAFETY: only through the safe-switch path. Never bypasses quiesce.
    return mmOwner->requestMode((Mode)idx);
}
static uint8_t mmCurrentModeCb()
{
    return mmOwner ? (uint8_t)mmOwner->current() : 0;
}
static void mmGetStatusCb(uint8_t& mode, uint32_t& c0, uint32_t& c1,
                          uint32_t& sw, uint8_t& feat)
{
    if (mmOwner) mmOwner->mmGetStatus(mode, c0, c1, sw, feat);
    else { mode = 0; c0 = c1 = 500000; sw = 33333; feat = 0; }
}
static uint8_t mmSetCanBaudCb(uint8_t bus, uint32_t baud, uint8_t& applied)
{
    if (!mmOwner) return mm::ST_ERR;
    return mmOwner->mmSetCanBaud(bus, baud, applied);
}
static uint8_t mmSetSwcanSpeedCb(uint32_t speed, uint8_t& applied)
{
    if (!mmOwner) return mm::ST_ERR;
    return mmOwner->mmSetSwcanSpeed(speed, applied);
}
static uint8_t mmSetFeatureCb(uint8_t feat, uint8_t enable)
{
    if (!mmOwner) return mm::ST_ERR;
    return mmOwner->mmSetFeature(feat, enable);
}

// ---- Phase 3: UDS + OBD2 supervision ops ----
static uint8_t mmUdsTxCb(const uint8_t* svc, uint8_t svcLen,
                         uint8_t* resp, uint8_t respCap,
                         uint8_t* respLen, uint8_t* nResp)
{
    return uds::mmTx(svc, svcLen, resp, respCap, respLen, nResp);
}
static uint8_t mmUdsCfgCb(uint8_t bus, uint32_t baud, uint32_t txId, uint32_t rxId,
                          uint8_t func, uint16_t tpMs)
{
    return uds::mmCfg(bus, baud, txId, rxId, func, tpMs);
}
static void mmUdsStatusCb(uint8_t& session, uint8_t& bus, uint32_t& baud,
                          uint32_t& txId, uint32_t& rxId, uint8_t& func, uint8_t& tpOn)
{
    uds::mmStatus(session, bus, baud, txId, rxId, func, tpOn);
}
static uint8_t mmObd2CfgCb(uint8_t bus, uint32_t baud, uint16_t intervalMs,
                           const uint8_t* pids, uint8_t nPids)
{
    return obd2::mmCfg(bus, baud, intervalMs, pids, nPids);
}
static uint8_t mmObd2CtlCb(uint8_t start, uint8_t& running)
{
    return obd2::mmCtl(start, running);
}
static void mmObd2StatusCb(uint8_t& running, uint8_t& bus,
                           uint16_t& intervalMs, uint8_t& nPids)
{
    obd2::mmStatus(running, bus, intervalMs, nPids);
}

// ---- Phase 4: MITM supervision ops ----
static uint8_t mmMitmRuleAddCb(const uint8_t* p, uint8_t& idx)
{
    return mitm::mmRuleAdd(p, idx);
}
static uint8_t mmMitmRuleDelCb(uint8_t idx, uint8_t& nRules)
{
    return mitm::mmRuleDel(idx, nRules);
}
static uint8_t mmMitmRuleListCb(uint8_t start, uint8_t* out,
                                uint8_t& nTotal, uint8_t& nRet)
{
    return mitm::mmRuleList(start, out, nTotal, nRet);
}
static void mmMitmStatsCb(uint8_t* out)
{
    mitm::mmStats(out);
}
static uint8_t mmMitmCfgCb(const uint8_t* p, uint8_t& applied)
{
    return mitm::mmCfg(p, applied);
}
static void mmMitmCfgGetCb(uint8_t* out)
{
    mitm::mmCfgGet(out);
}

// ---- Phase 5: PROG supervision ops ----
static uint8_t mmProgCfgCb(uint8_t bus, uint32_t baud, uint32_t txId, uint32_t rxId)
{
    return prog::mmCfg(bus, baud, txId, rxId);
}
static uint8_t mmProgTxCb(const uint8_t* svc, uint8_t svcLen,
                          uint8_t* resp, uint8_t* respLen, uint8_t* nResp)
{
    return prog::mmTx(svc, svcLen, resp, respLen, nResp);
}
static uint8_t mmProgDataCb(uint16_t seq, uint8_t flags, const uint8_t* data,
                            uint8_t len, uint16_t crc, uint16_t& ackSeq)
{
    return prog::mmData(seq, flags, data, len, crc, ackSeq);
}
static uint8_t mmProgDoneCb(uint32_t fileCrc32, uint32_t& txCrc, uint8_t& match)
{
    return prog::mmDone(fileCrc32, txCrc, match);
}
static void mmProgAbortCb()
{
    prog::mmAbort();
}
static uint8_t mmProgArmCb(uint8_t powerOk, uint8_t& armed)
{
    return prog::mmArm(powerOk, armed);
}
static void mmProgStatusCb(uint8_t* out)
{
    prog::mmStatus(out);
}
static uint8_t mmProgDownloadCb(uint32_t addr, uint32_t len, uint8_t alfid)
{
    return prog::mmDownload(addr, len, alfid);
}
static uint8_t mmProgVerifyCb(uint32_t addr, uint32_t len, uint8_t alfid,
                              uint8_t& match, uint32_t& rxCrc)
{
    return prog::mmVerify(addr, len, alfid, match, rxCrc);
}

// ---- Phase 6: transport supervision ops ----
// p = [bus][id LE32][ext][len][data..][periodMs LE32][offsetMs LE32][count LE32]
static uint8_t mmTxschedAddCb(const uint8_t* p, uint8_t pLen, uint8_t& slot)
{
    slot = 0xFF;
    if (pLen < 20) return mm::ST_BAD_ARG;
    uint8_t bus = p[0];
    uint32_t id = (uint32_t)p[1] | ((uint32_t)p[2] << 8) |
                  ((uint32_t)p[3] << 16) | ((uint32_t)p[4] << 24);
    uint8_t ext = p[5];
    uint8_t len = p[6];
    if (bus > 1 || ext > 1 || len > 8 || pLen != (uint8_t)(19 + len))
        return mm::ST_BAD_ARG;
    txsched::Slot s;
    s.txEn = true;
    s.bus = bus;
    s.id = id;
    s.ext = (ext != 0);
    s.len = len;
    for (uint8_t i = 0; i < len; i++) s.data[i] = p[7 + i];
    const uint8_t* q = p + 7 + len;
    s.periodMs = (uint32_t)q[0] | ((uint32_t)q[1] << 8) |
                 ((uint32_t)q[2] << 16) | ((uint32_t)q[3] << 24);
    s.offsetMs = (uint32_t)q[4] | ((uint32_t)q[5] << 8) |
                 ((uint32_t)q[6] << 16) | ((uint32_t)q[7] << 24);
    s.count = (int32_t)((uint32_t)q[8] | ((uint32_t)q[9] << 8) |
                        ((uint32_t)q[10] << 16) | ((uint32_t)q[11] << 24));
    slot = txsched::add(s);
    return (slot == 0xFF) ? mm::ST_ERR : mm::ST_OK;
}
static uint8_t mmTxschedDelCb(uint8_t slot)
{
    return txsched::remove(slot) ? mm::ST_OK : mm::ST_BAD_ARG;
}
// out = 25-byte entries (see CMD_TXSCHED_LIST); nRet <= 1.
static uint8_t mmTxschedListCb(uint8_t start, uint8_t* out,
                               uint8_t& nTotal, uint8_t& nRet)
{
    nTotal = 0; nRet = 0;
    for (uint8_t i = 0; i < txsched::MAX_SLOTS; i++) {
        const txsched::Slot* s = txsched::get(i);
        if (s == nullptr) continue;
        if (nTotal >= start && nRet == 0) {
            out[0] = i;
            out[1] = s->bus;
            out[2] = (uint8_t)(s->id & 0xFF);
            out[3] = (uint8_t)((s->id >> 8) & 0xFF);
            out[4] = (uint8_t)((s->id >> 16) & 0xFF);
            out[5] = (uint8_t)((s->id >> 24) & 0xFF);
            out[6] = s->ext ? 1 : 0;
            out[7] = s->len;
            out[8] = (uint8_t)(s->periodMs & 0xFF);
            out[9] = (uint8_t)((s->periodMs >> 8) & 0xFF);
            out[10] = (uint8_t)((s->periodMs >> 16) & 0xFF);
            out[11] = (uint8_t)((s->periodMs >> 24) & 0xFF);
            out[12] = (uint8_t)(s->offsetMs & 0xFF);
            out[13] = (uint8_t)((s->offsetMs >> 8) & 0xFF);
            out[14] = (uint8_t)((s->offsetMs >> 16) & 0xFF);
            out[15] = (uint8_t)((s->offsetMs >> 24) & 0xFF);
            uint32_t c = (uint32_t)s->count;
            out[16] = (uint8_t)(c & 0xFF);
            out[17] = (uint8_t)((c >> 8) & 0xFF);
            out[18] = (uint8_t)((c >> 16) & 0xFF);
            out[19] = (uint8_t)((c >> 24) & 0xFF);
            out[20] = s->active ? 1 : 0;
            out[21] = (uint8_t)(s->fires & 0xFF);
            out[22] = (uint8_t)((s->fires >> 8) & 0xFF);
            out[23] = (uint8_t)((s->fires >> 16) & 0xFF);
            out[24] = (uint8_t)((s->fires >> 24) & 0xFF);
            nRet = 1;
        }
        nTotal++;
    }
    return mm::ST_OK;
}
static uint8_t mmTxschedCtlCb(uint8_t slot, uint8_t start, uint8_t& active)
{
    active = 0;
    if (start > 1) return mm::ST_BAD_ARG;
    bool ok = start ? txsched::start(slot) : txsched::stop(slot);
    if (!ok) return mm::ST_BAD_ARG;
    const txsched::Slot* s = txsched::get(slot);
    active = (s != nullptr && s->active) ? 1 : 0;
    return mm::ST_OK;
}
// p = 20-byte filter payload (see CMD_FILTER_ADD).
static uint8_t mmFilterAddCb(const uint8_t* p, uint8_t& idx)
{
    idx = 0xFF;
    uint8_t bus = p[0], proto = p[1];
    uint8_t dir = p[15];
    if (bus != 0xFF && bus > 4) return mm::ST_BAD_ARG;
    if (proto > 5) return mm::ST_BAD_ARG;
    if (dir > 2) return mm::ST_BAD_ARG;
    busfilter::Filter f;
    f.bus = (busfilter::Bus)bus;
    f.proto = (busfilter::Proto)proto;
    f.addr = (uint32_t)p[2] | ((uint32_t)p[3] << 8) |
             ((uint32_t)p[4] << 16) | ((uint32_t)p[5] << 24);
    f.id = (uint32_t)p[6] | ((uint32_t)p[7] << 8) |
           ((uint32_t)p[8] << 16) | ((uint32_t)p[9] << 24);
    f.mask = (uint32_t)p[10] | ((uint32_t)p[11] << 8) |
             ((uint32_t)p[12] << 16) | ((uint32_t)p[13] << 24);
    f.dir = (busfilter::Dir)dir;
    f.payOff = p[16];
    f.payMask = p[17];
    f.payVal = p[18];
    f.invert = (p[19] != 0);
    int8_t i = busfilter::add(f);
    if (i < 0) return mm::ST_ERR;
    idx = (uint8_t)i;
    return mm::ST_OK;
}
static uint8_t mmFilterDelCb(uint8_t idx, uint8_t& nLeft)
{
    nLeft = 0;
    if (idx == 0xFF) {
        busfilter::clear();
    } else {
        if (!busfilter::remove(idx)) return mm::ST_BAD_ARG;
    }
    nLeft = busfilter::count();
    return mm::ST_OK;
}
// out = 20-byte entries (see CMD_FILTER_LIST); nRet <= 2.
static uint8_t mmFilterListCb(uint8_t start, uint8_t* out,
                              uint8_t& nTotal, uint8_t& nRet)
{
    nTotal = 0; nRet = 0;
    for (uint8_t i = 0; i < busfilter::MAX_FILTERS && nRet < 2; i++) {
        const busfilter::Filter* f = busfilter::get(i);
        if (f == nullptr) continue;
        if (nTotal >= start) {
            if (busfilter::pack(i, out + nRet * 20) == 0) return mm::ST_ERR;
            nRet++;
        }
        nTotal++;
    }
    return mm::ST_OK;
}
static uint8_t mmBusStatsCb(uint8_t bus, uint8_t* out)
{
    if (bus > 4) return mm::ST_BAD_ARG;  // Phase 8: 3 = J1850, 4 = SWCAN
    busstat::pack((busstat::Bus)bus, out);
    return mm::ST_OK;
}
// Phase 7: K-line supervision.
// klineStatus: out = [acquired, initDone, kb1, kb2, target, tester]
//   (CMD_KLINE_STATUS read).
static void mmKlineStatusCb(uint8_t* out)
{
    kline::Status s;
    kline::getStatus(s);
    out[0] = s.acquired ? 1 : 0;
    out[1] = (uint8_t)s.initDone;
    out[2] = s.kb1;
    out[3] = s.kb2;
    out[4] = s.target;
    out[5] = s.tester;
}
// klineInit: type 1 = 5-baud slow init, 2 = fast init. Runs the init
// sequence, which TRANSMITS on the bus by construction — gated on ELM/STN
// mode (the mode's transmit authority; see docs/SAFETY_REVIEW.md).
// kb1/kb2 = keyword bytes from the init.
static uint8_t mmKlineInitCb(uint8_t type, uint8_t& kb1, uint8_t& kb2)
{
    kb1 = 0; kb2 = 0;
    if (mmOwner == nullptr) return mm::ST_ERR;
    Mode m = mmOwner->current();
    if (m != Mode::ELM327 && m != Mode::STN) return mm::ST_ERR;
    if (type != 1 && type != 2) return mm::ST_BAD_ARG;
    kline::release();  // re-init from a clean state (idempotent)
    if (!kline::acquire()) return mm::ST_ERR;
    bool ok = (type == 2) ? kline::initFast() : kline::initSlow();
    if (!ok) { kline::release(); return mm::ST_ERR; }
    kline::Status s;
    kline::getStatus(s);
    kb1 = s.kb1; kb2 = s.kb2;
    return mm::ST_OK;
}
// Phase 9: discovery + capability supervision.
// discoverStart: run a scan on bus 0/1 (mode must be OBD2/UDS — the entry
// points stop their own bus activity first).
static uint8_t mmDiscoverStartCb(uint8_t bus)
{
    if (mmOwner == nullptr) return mm::ST_ERR;
    Mode m = mmOwner->current();
    if (m == Mode::OBD2) return obd2::mmDiscover(bus);
    if (m == Mode::UDS) return uds::mmDiscover(bus);
    return mm::ST_ERR;
}
// discoverStatus: out = [done][nEcus][baud LE32] (CMD_DISCOVER_STATUS read).
static void mmDiscoverStatusCb(uint8_t* out)
{
    const discover::Result& r = discover::lastResult();
    out[0] = r.done ? 1 : 0;
    out[1] = r.nEcus;
    out[2] = (uint8_t)(r.baud & 0xFF);
    out[3] = (uint8_t)((r.baud >> 8) & 0xFF);
    out[4] = (uint8_t)((r.baud >> 16) & 0xFF);
    out[5] = (uint8_t)((r.baud >> 24) & 0xFF);
}
// discoverList: page of ECU entries from startIdx (CMD_DISCOVER_LIST read).
static void mmDiscoverListCb(uint8_t startIdx, uint8_t* out,
                             uint8_t& nTotal, uint8_t& nRet)
{
    discover::listPage(discover::lastResult(), startIdx, out, nTotal, nRet);
}
// capGrant: level 0..4 = escalate (upward-only, warned, never silent),
// 0xFF = query. Returns the new (or current) grant level.
static uint8_t mmCapGrantCb(uint8_t level)
{
    if (level == 0xFF) return (uint8_t)capgate::current();  // query
    if (level > 4) return (uint8_t)capgate::current();     // invalid: no-op
    return (uint8_t)capgate::escalate((capgate::Grant)level);
}
// Phase 10: gateway supervision.
// gwMapAdd: p = 21-byte mapping payload (see CMD_GW_MAP_ADD); idx = new
//   table index (0xFE = staged for arm-confirm on MODIFY mappings).
static uint8_t mmGwMapAddCb(const uint8_t* p, uint8_t& idx)
{
    return gateway::mmMapAdd(p, idx);
}
// gwMapDel: idx = 0xFF clears all; nMaps = remaining count.
static uint8_t mmGwMapDelCb(uint8_t idx, uint8_t& nMaps)
{
    return gateway::mmMapDel(idx, nMaps);
}
// gwMapList: 22-byte entries into out (nRet <= 1 per the 48-byte cap).
static uint8_t mmGwMapListCb(uint8_t start, uint8_t* out,
                             uint8_t& nTotal, uint8_t& nRet,
                             uint16_t& version)
{
    return gateway::mmMapList(start, out, nTotal, nRet, version);
}
// gwStats: out = 29-byte stats blob (see CMD_GW_STATS).
static void mmGwStatsCb(uint8_t* out)
{
    gateway::mmStats(out);
}
// gwCfg: p = 10-byte config payload (see CMD_GW_CFG); applied=1 if live.
static uint8_t mmGwCfgCb(const uint8_t* p, uint8_t& applied)
{
    return gateway::mmCfg(p, applied);
}
// gwCfgGet: out = 12-byte config blob (CMD_GW_CFG read).
static void mmGwCfgGetCb(uint8_t* out)
{
    gateway::mmCfgGet(out);
}
// Phase 11: vehicle supply voltage for the SuperB sleep/wake manager.
// V_SENSE divider R34/R48 -> PA6 / Arduino 92 / AD3, ratio 0.18033
// (knowledge pack v1.4.0, VERIFIED). SAM3X ADC: 12-bit, 3.3V ref.
// mV = raw * 3300 / (4095 * 0.18033) = raw * 4.46932. INFERRED until
// per-unit calibration (resistor tolerance stack ~+/-4.8% worst-case).
static void mmGetVoltageCb(uint16_t& mV)
{
#ifdef SIM_M2
    mV = 12600;  // sim: nominal resting voltage (documented test double)
#else
    constexpr uint8_t VSENSE_PIN = 92;  // PA6/AD3
    uint32_t raw = analogRead(VSENSE_PIN);
    mV = (uint16_t)((raw * 446932UL) / 100000UL);
#endif
}

// ---- Phase 12: mm supervision callbacks ----
static uint8_t mmLogPollCb(uint8_t* out, uint8_t& nRec)
{
    nRec = logrec::poll(out);
    return mm::ST_OK;
}
static uint8_t mmLogCtlCb(uint8_t op)
{
    if (op == 0) logrec::start();
    else if (op == 1) logrec::stop();
    else if (op == 2) logrec::clear();
    else return mm::ST_BAD_ARG;
    return mm::ST_OK;
}
static uint8_t mmProfileSaveCb(const char* name)
{
    return profiles::save(name) ? mm::ST_OK : mm::ST_ERR;
}
static uint8_t mmProfileLoadCb()
{
    return profiles::load() ? mm::ST_OK : mm::ST_ERR;
}
static void mmProfileInfoCb(uint8_t* out)
{
    char name[16];
    uint8_t ver = 0;
    uint32_t writes = 0;
    bool has = profiles::info(name, ver, writes);
    out[0] = has ? 1 : 0;
    out[1] = ver;
    out[2] = (uint8_t)(writes & 0xFF);
    out[3] = (uint8_t)((writes >> 8) & 0xFF);
    out[4] = (uint8_t)((writes >> 16) & 0xFF);
    out[5] = (uint8_t)((writes >> 24) & 0xFF);
    for (uint8_t i = 0; i < 16; i++) out[6 + i] = (uint8_t)name[i];
}
static uint8_t mmAutodetStartCb(uint8_t allowTx)
{
    // Same explicit bus takeover as the console path: quiesce, run
    // (blocking), re-enter the current mode to restore ownership.
    ModeManager* self = modes::manager();
    if (self) self->quiesceBuses();
    bool ok = autodet::start(allowTx != 0);
    if (self) self->reacquire();
    return ok ? mm::ST_OK : mm::ST_ERR;
}
static void mmAutodetStatusCb(uint8_t* out)
{
    const autodet::Result& r = autodet::lastResult();
    out[0] = r.done ? 1 : 0;
    out[1] = r.found ? 1 : 0;
    out[2] = r.proto;
    out[3] = (uint8_t)(r.baud & 0xFF);
    out[4] = (uint8_t)((r.baud >> 8) & 0xFF);
    out[5] = (uint8_t)((r.baud >> 16) & 0xFF);
    out[6] = (uint8_t)((r.baud >> 24) & 0xFF);
    out[7] = r.confidence;
}
static uint8_t mmHarnessRunCb(uint16_t mask, uint8_t& nTests, uint8_t& nPass)
{
    vharness::SuiteResult r;
    if (!vharness::runSuite(mask, r)) return mm::ST_ERR;  // no selfcheck
    nTests = r.nTests;
    nPass = r.nPass;
    return mm::ST_OK;
}
static void mmHarnessStatusCb(uint8_t* out)
{
    const vharness::SuiteResult& r = vharness::lastResult();
    out[0] = r.ran ? 1 : 0;
    out[1] = r.nTests;
    out[2] = r.nPass;
}
static void mmDbcInfoCb(uint8_t* out)
{
    out[0] = dbcsig::dbcCount();
    out[1] = (uint8_t)(dbcsig::msgCount() & 0xFF);
    out[2] = (uint8_t)((dbcsig::msgCount() >> 8) & 0xFF);
    out[3] = (uint8_t)(dbcsig::sigCount() & 0xFF);
    out[4] = (uint8_t)((dbcsig::sigCount() >> 8) & 0xFF);
    out[5] = (uint8_t)(dbcsig::enumCount() & 0xFF);
    out[6] = (uint8_t)((dbcsig::enumCount() >> 8) & 0xFF);
}
static uint8_t mmDbcDecodeCb(uint32_t id, uint8_t ext, const uint8_t* data,
                             uint8_t len)
{
    // Count only on the mm path (detail via the USB console).
    return (uint8_t)dbcsig::decode(id, ext != 0, data, len, nullptr, nullptr);
}

// p = 15-byte ISO-TP config payload (see CMD_ISOTP_CFG write).
static uint8_t mmIsotpCfgCb(const uint8_t* p)
{
    isotp::Config c;
    c.nBsMs = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
    c.nCrMs = (uint16_t)p[2] | ((uint16_t)p[3] << 8);
    c.p2Ms = (uint16_t)p[4] | ((uint16_t)p[5] << 8);
    c.p2StarMs = (uint16_t)p[6] | ((uint16_t)p[7] << 8);
    c.maxBlockSize = p[8];
    c.stMin = p[9];
    if (p[10] > 1) return mm::ST_BAD_ARG;
    c.autoFlowControl = (p[10] != 0);
    c.maxPayload = (uint16_t)p[11] | ((uint16_t)p[12] << 8);
    if (c.maxPayload < 8 || c.maxPayload > isotp::ISOTP_BUF_MAX)
        return mm::ST_BAD_ARG;
    if (p[13] > 5) return mm::ST_BAD_ARG;
    c.addrType = (isotp::AddrType)p[13];
    c.addrExt = p[14];
    isotp::setDefault(c);
    return mm::ST_OK;
}
// out = 15-byte config blob (CMD_ISOTP_CFG read).
static void mmIsotpCfgGetCb(uint8_t* out)
{
    isotp::Config c;
    isotp::getDefault(c);
    out[0] = (uint8_t)(c.nBsMs & 0xFF);
    out[1] = (uint8_t)((c.nBsMs >> 8) & 0xFF);
    out[2] = (uint8_t)(c.nCrMs & 0xFF);
    out[3] = (uint8_t)((c.nCrMs >> 8) & 0xFF);
    out[4] = (uint8_t)(c.p2Ms & 0xFF);
    out[5] = (uint8_t)((c.p2Ms >> 8) & 0xFF);
    out[6] = (uint8_t)(c.p2StarMs & 0xFF);
    out[7] = (uint8_t)((c.p2StarMs >> 8) & 0xFF);
    out[8] = c.maxBlockSize;
    out[9] = c.stMin;
    out[10] = c.autoFlowControl ? 1 : 0;
    out[11] = (uint8_t)(c.maxPayload & 0xFF);
    out[12] = (uint8_t)((c.maxPayload >> 8) & 0xFF);
    out[13] = (uint8_t)c.addrType;
    out[14] = c.addrExt;
}

void ModeManager::begin()
{
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    SerialUSB.println(F("M2-MULTIMODE phase5"));
    SerialUSB.println(F("Type 'help' for console commands."));
    // Register the mm control-channel ops (SuperB web GUI path).
    mmOwner = this;
    gManager = this;  // Phase 12: profiles accessor
    profiles::init();  // Phase 12: load the stored vehicle profile (if any)
    static const mm::Ops ops = {
        mmRequestMode, mmCurrentModeCb, mmGetStatusCb,
        mmSetCanBaudCb, mmSetSwcanSpeedCb, mmSetFeatureCb,
        mmUdsTxCb, mmUdsCfgCb, mmUdsStatusCb,
        mmObd2CfgCb, mmObd2CtlCb, mmObd2StatusCb,
        // Phase 4: MITM supervision.
        mmMitmRuleAddCb, mmMitmRuleDelCb, mmMitmRuleListCb,
        mmMitmStatsCb, mmMitmCfgCb, mmMitmCfgGetCb,
        // Phase 5: PROG supervision.
        mmProgCfgCb, mmProgTxCb, mmProgDataCb, mmProgDoneCb,
        mmProgAbortCb, mmProgArmCb, mmProgStatusCb,
        mmProgDownloadCb, mmProgVerifyCb,
        // Phase 6: transport supervision (scheduler, filters, stats, ISO-TP).
        mmTxschedAddCb, mmTxschedDelCb, mmTxschedListCb, mmTxschedCtlCb,
        mmFilterAddCb, mmFilterDelCb, mmFilterListCb,
        mmBusStatsCb, mmIsotpCfgCb, mmIsotpCfgGetCb,
        // Phase 7: K-line supervision.
        mmKlineStatusCb, mmKlineInitCb,
        // Phase 9: discovery + capability supervision.
        mmDiscoverStartCb, mmDiscoverStatusCb, mmDiscoverListCb,
        mmCapGrantCb,
        // Phase 10: gateway supervision.
        mmGwMapAddCb, mmGwMapDelCb, mmGwMapListCb,
        mmGwStatsCb, mmGwCfgCb, mmGwCfgGetCb,
        // Phase 11: SuperB sleep/wake manager.
        mmGetVoltageCb,
        // Phase 12: profiles / auto-detect / logging / harness / DBC.
        mmLogPollCb, mmLogCtlCb,
        mmProfileSaveCb, mmProfileLoadCb, mmProfileInfoCb,
        mmAutodetStartCb, mmAutodetStatusCb,
        mmHarnessRunCb, mmHarnessStatusCb,
        mmDbcInfoCb, mmDbcDecodeCb
    };
    mm::setOps(&ops);
    quiesceBuses();       // release-everything first (safe-mode-switch rule)
    enterElm();              // default boot mode: ELM327 (2026-10-08: was M2RET;
                             // ELM auto-detects protocol on first AT command)
    currentMode = Mode::ELM327;
    capgate::enterMode((uint8_t)Mode::ELM327);  // Phase 9: boot grant
    SerialUSB.println(F("Booted into ELM327 mode."));
}

void ModeManager::quiesceBuses()
{
    // 1. Native CAN controllers to init/off — per-controller, ONLY if
    //    begin(...,255) has already run on that controller (see file
    //    header: raw disable() would drive Arduino 50/48 =
    //    J1850_PWM_VPW / 5V buck-boost).
    if (can0Begun) Can0.disable();
    if (can1Begun) Can1.disable();
    busstat::setPresent(busstat::Bus::CAN_BUS0, false);  // Phase 6
    busstat::setPresent(busstat::Bus::CAN_BUS1, false);
    // 2. SWCAN to sleep. m2ret::swcanSleep() routes through
    //    swcan::release(): TH8056 transceiver to sleep via GPIO, plus the
    //    MCP2515 to sleep when it was acquired (card quiesce rule) — safe
    //    even if the MCP2515 was never initialized (GPIO-only fallback).
    //    Sleep encoding VERIFIED: mode(0) = M0 LOW, M1 LOW
    //    (MCP2515_sw_can.cpp:49-52, Macchina Single-Wire-CAN-mcp2515).
    //    Note: up to ~100 ms TH8056 settle on this path, only when SWCAN
    //    was actually acquired.
    m2ret::swcanSleep();
    // 3. J1850 rail off + driver released (idempotent: safe even if ELM
    //    never acquired it). Also detaches the RX ISR and parks TX pins.
    //    Phase 2: J1850 is a real resource now (ELM protocols 1/2).
    //    Phase 7: the rail is refcounted with K-line (rail9141) — this
    //    release() drops pin 49 only when K-line isn't holding it
    //    (INV-PWR49).
    j1850::release();
    // 4. K-line released (idempotent; safe even if ELM never acquired it).
    //    Transceivers to sleep, UARTs ended, rail released refcounted.
    //    Phase 7: ELM protocols 3/4/5.
    kline::release();
    // NOTE (ownership rule): TC6/TC7 are NOT claimed by the J1850 driver
    // (GPIO-interrupt RX, not timer capture — see j1850.h). They stay free.
    // PWM6 is conceptually owned while J1850-PWM is active; release() parks
    // J1850N_TX so nothing drives it afterwards.
}

void ModeManager::enterM2RET()
{
    // Acquire BEFORE enable (ownership rule):
    //   CAN0, CAN1, SWCAN (SPI0 CS3 + PIOC16 callback slot), UART0 (Bee link).
    // UART0/Serial was begun in setup() and is never released (immutable link).
    //
    // due_can begin(baud, 255): 255 disables the transceiver-enable pin
    // driving (see file header). M2RET does exactly this (M2RET.ino:320).
    //
    // G1 (2026-10-08): check the return values. begin() returns 0 on init
    // failure; the begun flags must mean "successfully begun", not
    // "attempted", or the resource-safety logic lies about hardware state.
    bool ok0 = Can0.begin(can0BaudSet, 255) != 0;
    bool ok1 = Can1.begin(can1BaudSet, 255) != 0;
    can0Begun = ok0;
    can1Begun = ok1;
    busstat::setPresent(busstat::Bus::CAN_BUS0, ok0);  // Phase 6
    busstat::setPresent(busstat::Bus::CAN_BUS1, ok1);
    if (!ok0 || !ok1) {
        SerialUSB.println(F("[M2RET] WARNING: CAN controller init failed."));
    }
    m2ret::setPromiscuousMode();  // open RX filters (M2RET.ino:398-412)
    m2ret::setCanSpeeds(can0BaudSet, can1BaudSet);  // keep GVRET replies true
    m2ret::swcanSetSpeed(swcanSpeedSet);  // sync staged speed (no HW touch if inactive)
    m2ret::swcanEnter();          // setupSW(speed) + mode(3) + ISR
    m2ret::begin(can0Begun && can1Begun);  // both true here: disable() is pin-safe
}

void ModeManager::exitM2RET()
{
    // Release in reverse (ownership rule): IRQ first, then transceiver to
    // sleep, then controllers off. After this, quiesceBuses() state holds.
    m2ret::swcanExit();  // detachInterrupt + mode(0)
    if (can0Begun) Can0.disable();
    if (can1Begun) Can1.disable();
    busstat::setPresent(busstat::Bus::CAN_BUS0, false);  // Phase 6
    busstat::setPresent(busstat::Bus::CAN_BUS1, false);
}

void ModeManager::enterElm()
{
    // Acquire BEFORE enable (ownership rule): UART0 (Bee link) is already up
    // and never released. CAN0 / J1850 are acquired lazily by the ELM layer
    // on first protocol use (ensureProtocol), so entering the mode touches
    // no bus — the car sees nothing until the first ATSP/OBD command.
    elm::setJ1850Enabled(features & 0x01);
    elm::begin();
}

void ModeManager::exitElm()
{
    elm::exit();  // releases CAN0 / J1850 if held (idempotent)
}

void ModeManager::enterStn()
{
    // STN mode = the ELM core with ST-command parsing enabled (mode_stn.h).
    // Same lazy-acquire discipline as ELM: entering touches no bus.
    stn::begin();
}

void ModeManager::exitStn()
{
    stn::exit();  // releases CAN0 / J1850 if held (idempotent)
}

void ModeManager::enterMitm()
{
    // Acquire BEFORE enable (ownership rule): CAN0 + CAN1 begun at entry
    // (independent bauds, staged via console/mm). UART0/Bee was begun in
    // setup() and is never released. quiesceBuses() already handles both
    // native controllers via the per-controller begun flags.
    mitm::begin();
}

void ModeManager::exitMitm()
{
    mitm::exit();  // releases both CAN controllers (idempotent)
}

void ModeManager::enterUds()
{
    // Acquire BEFORE enable (ownership rule): UART0 (Bee link) is already up
    // and never released. The CAN bus is acquired lazily by the UDS layer on
    // the first command (uds::begin touches no bus — the car sees nothing
    // until the first service request).
    uds::begin();
}

void ModeManager::exitUds()
{
    uds::exit();  // releases the CAN bus if held (idempotent)
}

void ModeManager::enterObd2()
{
    // Same lazy-acquire discipline as ELM/UDS: entering touches no bus.
    // The poller starts only on 'obd2 start' (console / web / mm channel).
    obd2::begin();
}

void ModeManager::exitObd2()
{
    obd2::exit();  // stops poller, releases CAN bus (idempotent)
}

void ModeManager::enterProg()
{
    // Acquire BEFORE enable (ownership rule): UART0/Bee was begun in
    // setup() and is never released; the CAN bus is acquired lazily by
    // the PROG layer on the first command (entering touches no bus).
    //
    // SAFETY (docs/SAFETY_REVIEW.md §2): a bridge that modifies flash
    // payloads is a brick vector. Entering PROG auto-suspends the MITM
    // rule table (cleared, with an explicit notice); exit does NOT
    // restore it — re-adding rules is a deliberate user action.
    // Phase 10: the gateway mapping table is suspended the same way
    // (a translating gateway during a flash is the same brick vector;
    // docs/SAFETY_REVIEW.md §12).
    uint8_t suspended = mitm::suspendRules();
    gateway::suspend();
    prog::begin(suspended);
}

void ModeManager::exitProg()
{
    prog::exit();  // aborts an in-flight transfer, releases CAN (idempotent)
}

void ModeManager::enterGateway()
{
    // Acquire BEFORE enable (ownership rule): CAN0 at the staged baud +
    // SWCAN at the staged speed, via the mode's own begin(). UART0/Bee
    // was begun in setup() and is never released. quiesceBuses() already
    // handles both native CAN (begun flags) and SWCAN (swcan::release()).
    gateway::begin();
}

void ModeManager::exitGateway()
{
    gateway::exit();  // drops TX queues (counted), releases CAN0 + SWCAN
}

// Phase 12: SLCAN mode (Lawicel ASCII over the Bee UART, CAN0).
void ModeManager::enterSlcan()
{
    slcan::begin();
}
void ModeManager::exitSlcan()
{
    slcan::exit();  // closes channel, releases CAN0 (idempotent)
}

// Phase 12: HARNESS mode (virtual ECU on the CAN0<->CAN1 loopback pair).
void ModeManager::enterHarness()
{
    vharness::begin();
}
void ModeManager::exitHarness()
{
    vharness::exit();  // releases CAN0 + CAN1 (idempotent)
}

bool ModeManager::requestMode(Mode target)
{
    if (target == currentMode) {
        SerialUSB.println(F("Already in that mode."));
        return true;
    }
    if (!modeImplemented(target)) {
        SerialUSB.print(F("Mode '"));
        SerialUSB.print(modeName(target));
        SerialUSB.println(F("' not implemented yet. Staying put."));
        return false;
    }
    // Phase 12: reacquire() shares this body (re-enter the current mode
    // after an explicit bus takeover like auto-detect).
    switchTo(target);
    return true;
}

// Phase 12: re-enter the current mode (exit + quiesce + enter). Used after
// an explicit bus takeover (auto-detect) to restore the mode's ownership
// cleanly. The capgate grant still resets (same as a normal switch).
void ModeManager::reacquire()
{
    switchTo(currentMode);
}

// Safe switch body: release everything the old mode owned, then acquire
// for the new mode. This ordering is the whole point of the mode manager.
void ModeManager::switchTo(Mode target)
{
    switch (currentMode) {
        case Mode::M2RET: exitM2RET(); break;
        case Mode::ELM327: exitElm(); break;
        case Mode::STN: exitStn(); break;
        case Mode::UDS: exitUds(); break;
        case Mode::MITM: exitMitm(); break;
        case Mode::OBD2: exitObd2(); break;
        case Mode::PROG: exitProg(); break;
        case Mode::GATEWAY: exitGateway(); break;
        case Mode::SLCAN: exitSlcan(); break;
        case Mode::HARNESS: exitHarness(); break;
        default: break;
    }
    quiesceBuses();
    switch (target) {
        case Mode::M2RET: enterM2RET(); break;
        case Mode::ELM327: enterElm(); break;
        case Mode::STN: enterStn(); break;
        case Mode::UDS: enterUds(); break;
        case Mode::MITM: enterMitm(); break;
        case Mode::OBD2: enterObd2(); break;
        case Mode::PROG: enterProg(); break;
        case Mode::GATEWAY: enterGateway(); break;
        case Mode::SLCAN: enterSlcan(); break;
        case Mode::HARNESS: enterHarness(); break;
        default: break;  // unreachable: guarded by modeImplemented()
    }
    currentMode = target;
    // Phase 9: capability grant resets to the mode default on EVERY mode
    // entry — escalation never survives a mode switch.
    capgate::enterMode((uint8_t)target);
    SerialUSB.print(F("Switched to mode: "));
    SerialUSB.println(modeName(currentMode));
}

void ModeManager::loop()
{
    pollButton();
    pollSerialConsole();
    switch (currentMode) {
        case Mode::M2RET: m2ret::loop(); break;
        case Mode::ELM327: elm::loop(); break;
        case Mode::STN: stn::loop(); break;
        case Mode::UDS: uds::loop(); break;
        case Mode::MITM: mitm::loop(); break;
        case Mode::OBD2: obd2::loop(); break;
        case Mode::PROG: prog::loop(); break;
        case Mode::GATEWAY: gateway::loop(); break;
        case Mode::SLCAN: slcan::loop(); break;
        case Mode::HARNESS: vharness::loop(); break;
        default: break;
    }
    // Phase 6: shared transport services, serviced once per loop for every
    // mode. txsched fires due slots (incl. the OBD2 poll-engine slot);
    // busstat polls CAN_SR for bus-off / error-warning transitions.
    txsched::poll();
    busstat::poll();
}

void ModeManager::pollButton()
{
    // Active LOW. 50 ms debounce; <1 s = short press (cycle), >=1 s = long
    // press (back to M2RET). Timings are INFERRED (no bench to tune feel).
    bool down = (digitalRead(BUTTON_PIN) == LOW);
    uint32_t now = millis();
    if (down && !buttonWasDown) {
        if (now - lastDebounceAt > 50) {
            buttonWasDown = true;
            buttonDownAt = now;
            lastDebounceAt = now;
        }
    } else if (!down && buttonWasDown) {
        if (now - lastDebounceAt > 50) {
            uint32_t held = now - buttonDownAt;
            buttonWasDown = false;
            lastDebounceAt = now;
            if (held >= 1000) {
                SerialUSB.println(F("[button] long press -> ELM327 (default)"));
                requestMode(Mode::ELM327);
            } else {
                // Cycle to next IMPLEMENTED mode (Phase 4: +STN, +MITM).
                Mode next = (Mode)(((uint8_t)currentMode + 1) % (uint8_t)Mode::COUNT);
                uint8_t guard = 0;
                while (!modeImplemented(next) && guard++ < (uint8_t)Mode::COUNT) {
                    next = (Mode)(((uint8_t)next + 1) % (uint8_t)Mode::COUNT);
                }
                if (modeImplemented(next) && next != currentMode) {
                    SerialUSB.print(F("[button] short press -> "));
                    SerialUSB.println(modeName(next));
                    requestMode(next);
                } else {
                    SerialUSB.println(F("[button] short press: no other mode implemented"));
                }
            }
        }
    }
}

void ModeManager::pollSerialConsole()
{
    // Minimal line console on SerialUSB (the USB debug port — NOT the Bee
    // link; the Bee link carries pure GVRET in M2RET mode).
    static char line[64];
    static uint8_t pos = 0;
    while (SerialUSB.available() > 0) {
        char c = (char)SerialUSB.read();
        if (c == '\r' || c == '\n') {
            if (pos > 0) {
                line[pos] = '\0';
                handleConsoleLine(line);
                pos = 0;
            }
        } else if (pos < sizeof(line) - 1) {
            line[pos++] = c;
        }
    }
}

// ---- Phase 6: transport-layer console helpers ----
namespace {
bool p6Hex32(const char* t, uint32_t& out)
{
    if (t == nullptr || *t == '\0') return false;
    char* e = nullptr;
    unsigned long v = strtoul(t, &e, 16);
    if (e == nullptr || *e != '\0' || v > 0xFFFFFFFFUL) return false;
    out = (uint32_t)v;
    return true;
}
bool p6Dec(const char* t, uint32_t& out)
{
    if (t == nullptr || *t == '\0') return false;
    char* e = nullptr;
    unsigned long v = strtoul(t, &e, 10);
    if (e == nullptr || *e != '\0') return false;
    out = (uint32_t)v;
    return true;
}
int p6HexBytes(const char* t, uint8_t* out, uint8_t cap)
{
    size_t L = strlen(t);
    if (L == 0 || (L & 1) || L / 2 > cap) return -1;
    for (size_t i = 0; i < L / 2; i++) {
        char b[3] = { t[2 * i], t[2 * i + 1], '\0' };
        char* e = nullptr;
        long v = strtol(b, &e, 16);
        if (*e != '\0' || v < 0 || v > 0xFF) return -1;
        out[i] = (uint8_t)v;
    }
    return (int)(L / 2);
}
} // namespace

// txsched list | add <bus> <idhex> <periodms> [datahex] [count] [offsetms]
//         del <slot> | start|stop <slot> | clear
static void consoleTxsched(char* line)
{
    while (*line == ' ') line++;
    char* save = nullptr;
    char* tok = strtok_r(line, " \t", &save);
    if (tok == nullptr) {
        SerialUSB.println(F("txsched list|add|del|start|stop|clear"));
        return;
    }
    if (strcasecmp(tok, "list") == 0) {
        SerialUSB.println(F("slot bus id period offset count active fires"));
        for (uint8_t i = 0; i < txsched::MAX_SLOTS; i++) {
            const txsched::Slot* s = txsched::get(i);
            if (s == nullptr) continue;
            SerialUSB.print(i); SerialUSB.print(F(" "));
            SerialUSB.print(s->bus); SerialUSB.print(F(" 0x"));
            SerialUSB.print(s->id, HEX); SerialUSB.print(F(" "));
            SerialUSB.print(s->periodMs); SerialUSB.print(F(" "));
            SerialUSB.print(s->offsetMs); SerialUSB.print(F(" "));
            SerialUSB.print(s->count); SerialUSB.print(F(" "));
            SerialUSB.print(s->active ? F("on ") : F("off "));
            SerialUSB.println(s->fires);
        }
        return;
    }
    if (strcasecmp(tok, "clear") == 0) {
        txsched::clear();
        SerialUSB.println(F("Scheduler cleared."));
        return;
    }
    if (strcasecmp(tok, "add") == 0) {
        // add <bus 0|1> <idhex> <periodms> [datahex] [count] [offsetms]
        char* aBus = strtok_r(nullptr, " \t", &save);
        char* aId = strtok_r(nullptr, " \t", &save);
        char* aPer = strtok_r(nullptr, " \t", &save);
        char* aData = strtok_r(nullptr, " \t", &save);
        char* aCount = strtok_r(nullptr, " \t", &save);
        char* aOff = strtok_r(nullptr, " \t", &save);
        uint32_t bus = 9, id = 0, per = 0;
        int32_t count = -1;
        uint32_t off = 0;
        uint8_t data[8] = { 0 }; int dlen = 0;
        if (!p6Dec(aBus, bus) || bus > 1 || !p6Hex32(aId, id) ||
            !p6Dec(aPer, per)) {
            SerialUSB.println(F("usage: txsched add <bus 0|1> <idhex> <periodms> [datahex] [count] [offsetms]"));
            return;
        }
        if (aData != nullptr) {
            dlen = p6HexBytes(aData, data, 8);
            if (dlen < 0) { SerialUSB.println(F("Bad data hex.")); return; }
        }
        if (aCount != nullptr) {
            uint32_t c = 0;
            bool neg = (aCount[0] == '-');
            if (!p6Dec(neg ? aCount + 1 : aCount, c)) {
                SerialUSB.println(F("Bad count.")); return;
            }
            count = neg ? -(int32_t)c : (int32_t)c;
            if (!neg && count == 0) { SerialUSB.println(F("count 0 = never fires; use -1 for infinite.")); return; }
        }
        if (aOff != nullptr && !p6Dec(aOff, off)) {
            SerialUSB.println(F("Bad offset.")); return;
        }
        txsched::Slot s;
        s.txEn = true;
        s.bus = (uint8_t)bus;
        s.id = id;
        s.ext = (id > 0x7FF);
        s.len = (uint8_t)dlen;
        for (int i = 0; i < dlen; i++) s.data[i] = data[i];
        s.periodMs = per;
        s.offsetMs = off;
        s.count = count;
        uint8_t slot = txsched::add(s);
        if (slot == 0xFF) SerialUSB.println(F("Scheduler full."));
        else { SerialUSB.print(F("Slot ")); SerialUSB.println(slot); }
        return;
    }
    if (strcasecmp(tok, "del") == 0 || strcasecmp(tok, "start") == 0 ||
        strcasecmp(tok, "stop") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        uint32_t slot = 99;
        if (!p6Dec(a, slot) || slot >= txsched::MAX_SLOTS) {
            SerialUSB.println(F("usage: txsched del|start|stop <slot>"));
            return;
        }
        bool ok = false;
        if (strcasecmp(tok, "del") == 0) ok = txsched::remove((uint8_t)slot);
        else if (strcasecmp(tok, "start") == 0) ok = txsched::start((uint8_t)slot);
        else ok = txsched::stop((uint8_t)slot);
        SerialUSB.println(ok ? F("OK") : F("Bad slot."));
        return;
    }
    SerialUSB.println(F("txsched list|add|del|start|stop|clear"));
}

// filter list | add <bus> <proto> <idhex> [maskhex] [dir] [addrhex]
//             [payoff paymask payval] [invert]
//         del <idx>|all | clear
//   bus: 0|1|2|3|4|any(255)   proto: 0..5 (0=any)   dir: rx|tx|both
static void consoleFilter(char* line)
{
    while (*line == ' ') line++;
    char* save = nullptr;
    char* tok = strtok_r(line, " \t", &save);
    if (tok == nullptr) {
        SerialUSB.println(F("filter list|add|del|clear"));
        return;
    }
    if (strcasecmp(tok, "list") == 0) {
        SerialUSB.println(F("idx bus proto addr id mask dir payoff/mask/val inv"));
        for (uint8_t i = 0; i < busfilter::MAX_FILTERS; i++) {
            const busfilter::Filter* f = busfilter::get(i);
            if (f == nullptr) continue;
            SerialUSB.print(i); SerialUSB.print(F(" "));
            SerialUSB.print((uint8_t)f->bus); SerialUSB.print(F(" "));
            SerialUSB.print((uint8_t)f->proto); SerialUSB.print(F(" 0x"));
            SerialUSB.print(f->addr, HEX); SerialUSB.print(F(" 0x"));
            SerialUSB.print(f->id, HEX); SerialUSB.print(F(" 0x"));
            SerialUSB.print(f->mask, HEX); SerialUSB.print(F(" "));
            SerialUSB.print((uint8_t)f->dir); SerialUSB.print(F(" "));
            SerialUSB.print(f->payOff); SerialUSB.print(F("/0x"));
            SerialUSB.print(f->payMask, HEX); SerialUSB.print(F("/0x"));
            SerialUSB.print(f->payVal, HEX); SerialUSB.print(F(" "));
            SerialUSB.println(f->invert ? F("inv") : F("-"));
        }
        return;
    }
    if (strcasecmp(tok, "clear") == 0) {
        busfilter::clear();
        SerialUSB.println(F("Filters cleared."));
        return;
    }
    if (strcasecmp(tok, "del") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        if (a != nullptr && strcasecmp(a, "all") == 0) {
            busfilter::clear();
            SerialUSB.println(F("Filters cleared."));
            return;
        }
        uint32_t idx = 99;
        if (!p6Dec(a, idx) || !busfilter::remove((uint8_t)idx)) {
            SerialUSB.println(F("usage: filter del <idx>|all"));
            return;
        }
        SerialUSB.println(F("OK"));
        return;
    }
    if (strcasecmp(tok, "add") == 0) {
        char* aBus = strtok_r(nullptr, " \t", &save);
        char* aProto = strtok_r(nullptr, " \t", &save);
        char* aId = strtok_r(nullptr, " \t", &save);
        char* aMask = strtok_r(nullptr, " \t", &save);
        char* aDir = strtok_r(nullptr, " \t", &save);
        char* aAddr = strtok_r(nullptr, " \t", &save);
        char* aPayOff = strtok_r(nullptr, " \t", &save);
        char* aPayMask = strtok_r(nullptr, " \t", &save);
        char* aPayVal = strtok_r(nullptr, " \t", &save);
        char* aInv = strtok_r(nullptr, " \t", &save);
        busfilter::Filter f;
        uint32_t bus = 0xFF, proto = 0, id = 0, mask = 0, addr = 0xFFFFFFFF;
        if (aBus == nullptr || aProto == nullptr || aId == nullptr ||
            !p6Hex32(aId, id)) {
            SerialUSB.println(F("usage: filter add <bus> <proto> <idhex> [maskhex] [dir rx|tx|both] [addrhex] [payoff paymaskhex payvalhex] [invert]"));
            return;
        }
        if (strcasecmp(aBus, "any") == 0) bus = 0xFF;
        else if (!p6Dec(aBus, bus) || bus > 4) {
            SerialUSB.println(F("bus: 0|1|2|3|4|any")); return;
        }
        if (!p6Dec(aProto, proto) || proto > 5) {
            SerialUSB.println(F("proto: 0=any 1=CAN 2=SWCAN 3=VPW 4=PWM 5=KLINE")); return;
        }
        if (aMask != nullptr && !p6Hex32(aMask, mask)) {
            SerialUSB.println(F("Bad mask hex.")); return;
        }
        busfilter::Dir dir = busfilter::Dir::BOTH;
        if (aDir != nullptr) {
            if (strcasecmp(aDir, "rx") == 0) dir = busfilter::Dir::RX;
            else if (strcasecmp(aDir, "tx") == 0) dir = busfilter::Dir::TX;
            else if (strcasecmp(aDir, "both") == 0) dir = busfilter::Dir::BOTH;
            else { SerialUSB.println(F("dir: rx|tx|both")); return; }
        }
        if (aAddr != nullptr && !p6Hex32(aAddr, addr)) {
            SerialUSB.println(F("Bad addr hex.")); return;
        }
        uint32_t po = 0, pm = 0, pv = 0;
        if (aPayOff != nullptr) {
            if (aPayMask == nullptr || aPayVal == nullptr ||
                !p6Dec(aPayOff, po) || po > 7 ||
                !p6Hex32(aPayMask, pm) || pm > 0xFF ||
                !p6Hex32(aPayVal, pv) || pv > 0xFF) {
                SerialUSB.println(F("payload: <off 0..7> <maskhex> <valhex>")); return;
            }
        }
        f.bus = (busfilter::Bus)bus;
        f.proto = (busfilter::Proto)proto;
        f.addr = addr;
        f.id = id;
        f.mask = mask;
        f.dir = dir;
        f.payOff = (uint8_t)po;
        f.payMask = (uint8_t)pm;
        f.payVal = (uint8_t)pv;
        f.invert = (aInv != nullptr);
        int8_t idx = busfilter::add(f);
        if (idx < 0) SerialUSB.println(F("Filter table full."));
        else { SerialUSB.print(F("Filter ")); SerialUSB.println(idx); }
        return;
    }
    SerialUSB.println(F("filter list|add|del|clear"));
}

// busstat [reset]
static void consoleBusstat(char* line)
{
    while (*line == ' ') line++;
    if (strcasecmp(line, "reset") == 0) {
        busstat::resetAll();
        SerialUSB.println(F("Bus stats reset."));
        return;
    }
    SerialUSB.println(F("bus: txFrames rxFrames txBytes rxBytes rxOvf txFail busOff dropped"));
    for (uint8_t b = 0; b < (uint8_t)busstat::Bus::COUNT; b++) {
        const busstat::Stats& s = busstat::get((busstat::Bus)b);
        // Phase 8: five buses now (0-1 native CAN, 2 K-line, 3 J1850, 4 SWCAN).
        switch ((busstat::Bus)b) {
            case busstat::Bus::CAN_BUS0: SerialUSB.print(F("CAN0:  ")); break;
            case busstat::Bus::CAN_BUS1: SerialUSB.print(F("CAN1:  ")); break;
            case busstat::Bus::KLINE:    SerialUSB.print(F("KLINE: ")); break;
            case busstat::Bus::J1850:    SerialUSB.print(F("J1850: ")); break;
            case busstat::Bus::SWCAN:    SerialUSB.print(F("SWCAN: ")); break;
            default:                     SerialUSB.print(F("?:     ")); break;
        }
        SerialUSB.print(s.txFrames); SerialUSB.print(F(" "));
        SerialUSB.print(s.rxFrames); SerialUSB.print(F(" "));
        SerialUSB.print(s.txBytes); SerialUSB.print(F(" "));
        SerialUSB.print(s.rxBytes); SerialUSB.print(F(" "));
        SerialUSB.print(s.rxOverflow); SerialUSB.print(F(" "));
        SerialUSB.print(s.txFail); SerialUSB.print(F(" "));
        SerialUSB.print(s.busOff); SerialUSB.print(F(" "));
        SerialUSB.println(s.dropped);
    }
}

// isotp show | isotp set <field> <value>
//   fields: nbs ncr p2 p2star bs stmin autofc maxpayload addrtype addrext
static void consoleIsotp(char* line)
{
    while (*line == ' ') line++;
    char* save = nullptr;
    char* tok = strtok_r(line, " \t", &save);
    isotp::Config c;
    isotp::getDefault(c);
    if (tok == nullptr || strcasecmp(tok, "show") == 0) {
        SerialUSB.print(F("nBs=")); SerialUSB.print(c.nBsMs);
        SerialUSB.print(F(" nCr=")); SerialUSB.print(c.nCrMs);
        SerialUSB.print(F(" p2=")); SerialUSB.print(c.p2Ms);
        SerialUSB.print(F(" p2*=")); SerialUSB.print(c.p2StarMs);
        SerialUSB.println();
        SerialUSB.print(F("bs=")); SerialUSB.print(c.maxBlockSize);
        SerialUSB.print(F(" stMin=0x")); SerialUSB.print(c.stMin, HEX);
        SerialUSB.print(F(" autoFc=")); SerialUSB.print(c.autoFlowControl ? 1 : 0);
        SerialUSB.print(F(" maxPayload=")); SerialUSB.println(c.maxPayload);
        SerialUSB.print(F("addrType=")); SerialUSB.print((uint8_t)c.addrType);
        SerialUSB.print(F(" addrExt=0x")); SerialUSB.println(c.addrExt, HEX);
        return;
    }
    if (strcasecmp(tok, "set") == 0) {
        char* aF = strtok_r(nullptr, " \t", &save);
        char* aV = strtok_r(nullptr, " \t", &save);
        uint32_t v = 0;
        bool hex = false;
        if (aF == nullptr || aV == nullptr) {
            SerialUSB.println(F("usage: isotp set <nbs|ncr|p2|p2star|bs|stmin|autofc|maxpayload|addrtype|addrext> <value>"));
            return;
        }
        // stmin/addrext accept hex.
        if (strcasecmp(aF, "stmin") == 0 || strcasecmp(aF, "addrext") == 0) {
            hex = true;
            if (!p6Hex32(aV, v)) { SerialUSB.println(F("Bad hex value.")); return; }
        } else if (!p6Dec(aV, v)) {
            SerialUSB.println(F("Bad value.")); return;
        }
        bool ok = true;
        if (strcasecmp(aF, "nbs") == 0) c.nBsMs = (uint16_t)v;
        else if (strcasecmp(aF, "ncr") == 0) c.nCrMs = (uint16_t)v;
        else if (strcasecmp(aF, "p2") == 0) c.p2Ms = (uint16_t)v;
        else if (strcasecmp(aF, "p2star") == 0) c.p2StarMs = (uint16_t)v;
        else if (strcasecmp(aF, "bs") == 0 && v <= 255) c.maxBlockSize = (uint8_t)v;
        else if (strcasecmp(aF, "stmin") == 0 && v <= 255) c.stMin = (uint8_t)v;
        else if (strcasecmp(aF, "autofc") == 0 && v <= 1) c.autoFlowControl = (v != 0);
        else if (strcasecmp(aF, "maxpayload") == 0 && v >= 8 && v <= isotp::ISOTP_BUF_MAX)
            c.maxPayload = (uint16_t)v;
        else if (strcasecmp(aF, "addrtype") == 0 && v <= 5)
            c.addrType = (isotp::AddrType)v;
        else if (strcasecmp(aF, "addrext") == 0 && v <= 255) c.addrExt = (uint8_t)v;
        else ok = false;
        (void)hex;
        if (!ok) { SerialUSB.println(F("Bad field or out-of-range value.")); return; }
        isotp::setDefault(c);
        SerialUSB.println(F("OK"));
        return;
    }
    SerialUSB.println(F("isotp show | isotp set <field> <value>"));
}

// ---- Phase 7: K-line console ----
// kline status | kline init slow|fast | kline tx <hex> | kline timing show |
// kline timing set <p1min|p1max|p2min|p2max|p3min|p3max|p4min|p4max> <ms> |
// kline addrs <targetHex> <testerHex> | kline tp <ms|off> | kline release
//
// Bus-touching subcommands (init, tx) are refused unless ELM/STN is the
// active mode — the mode's transmit authority (see docs/SAFETY_REVIEW.md).
// Phase 8: J1850 console.
// j1850 status | j1850 crcgate on|off | j1850 hdrlen 1|3
static void consoleJ1850(char* line)
{
    while (*line == ' ') line++;
    char* save = nullptr;
    char* tok = strtok_r(line, " \t", &save);
    if (tok == nullptr || strcasecmp(tok, "status") == 0) {
        const j1850::Counters& c = j1850::counters();
        SerialUSB.print(F("acquired=")); SerialUSB.print(j1850::acquired() ? 1 : 0);
        SerialUSB.print(F(" proto="));
        SerialUSB.print(j1850::proto() == j1850::Proto::JPWM ? F("PWM") : F("VPW"));
        SerialUSB.print(F(" queue=")); SerialUSB.print(j1850::queueDepth());
        SerialUSB.print(F(" crcgate=")); SerialUSB.print(j1850::crcGate() ? 1 : 0);
        SerialUSB.print(F(" hdrlen=")); SerialUSB.println(j1850::headerLen());
        SerialUSB.print(F("err: sym=")); SerialUSB.print(c.symErr);
        SerialUSB.print(F(" crc=")); SerialUSB.print(c.crcErr);
        SerialUSB.print(F(" arb=")); SerialUSB.print(c.arbLost);
        SerialUSB.print(F(" qovf=")); SerialUSB.print(c.qOverflow);
        SerialUSB.print(F(" glitch=")); SerialUSB.println(c.glitch);
        return;
    }
    if (strcasecmp(tok, "crcgate") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        if (a == nullptr) { SerialUSB.println(F("usage: j1850 crcgate on|off")); return; }
        j1850::setCrcGate(strcasecmp(a, "on") == 0);
        SerialUSB.println(F("OK"));
        return;
    }
    if (strcasecmp(tok, "hdrlen") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        int n = (a != nullptr) ? atoi(a) : 0;
        if (n != 1 && n != 3) { SerialUSB.println(F("usage: j1850 hdrlen 1|3")); return; }
        j1850::setHeaderLen((uint8_t)n);
        SerialUSB.println(F("OK"));
        return;
    }
    SerialUSB.println(F("j1850 status | j1850 crcgate on|off | j1850 hdrlen 1|3"));
}

// Phase 8: SWCAN console.
// swcan status | swcan wake <ms> | swcan recover
static void consoleSwcan(char* line)
{
    while (*line == ' ') line++;
    char* save = nullptr;
    char* tok = strtok_r(line, " \t", &save);
    if (tok == nullptr || strcasecmp(tok, "status") == 0) {
        SerialUSB.print(F("acquired=")); SerialUSB.print(swcan::acquired() ? 1 : 0);
        SerialUSB.print(F(" mode=")); SerialUSB.print((uint32_t)swcan::mode());
        SerialUSB.print(F(" (0 sleep, 1 normal, 2 high-speed, 3 HV-wake)"));
        SerialUSB.print(F(" speed=")); SerialUSB.print(swcan::speed());
        SerialUSB.print(F(" listenOnly=")); SerialUSB.println(swcan::listenOnly() ? 1 : 0);
        return;
    }
    if (strcasecmp(tok, "wake") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        uint32_t ms = (a != nullptr) ? (uint32_t)atol(a) : 2000;
        if (!swcan::acquired()) { SerialUSB.println(F("not acquired")); return; }
        SerialUSB.println(swcan::waitWake(ms) ? F("wake seen") : F("wake timeout"));
        return;
    }
    if (strcasecmp(tok, "recover") == 0) {
        if (!swcan::acquired()) { SerialUSB.println(F("not acquired")); return; }
        SerialUSB.println(swcan::recover() ? F("recover OK") : F("recover FAILED"));
        return;
    }
    SerialUSB.println(F("swcan status | swcan wake <ms> | swcan recover"));
}

static void consoleKline(char* line, Mode cur)
{
    while (*line == ' ') line++;
    char* save = nullptr;
    char* tok = strtok_r(line, " \t", &save);
    if (tok == nullptr || strcasecmp(tok, "status") == 0) {
        kline::Status s;
        kline::getStatus(s);
        SerialUSB.print(F("acquired=")); SerialUSB.print(s.acquired ? 1 : 0);
        SerialUSB.print(F(" init=")); SerialUSB.print((uint8_t)s.initDone);
        SerialUSB.print(F(" kb="));
        SerialUSB.print(s.kb1, HEX); SerialUSB.print(F(" ")); SerialUSB.print(s.kb2, HEX);
        SerialUSB.print(F(" tgt=0x")); SerialUSB.print(s.target, HEX);
        SerialUSB.print(F(" tst=0x")); SerialUSB.println(s.tester, HEX);
        SerialUSB.print(F("p1=")); SerialUSB.print(s.timing.p1Min);
        SerialUSB.print(F("-")); SerialUSB.print(s.timing.p1Max);
        SerialUSB.print(F(" p2=")); SerialUSB.print(s.timing.p2Min);
        SerialUSB.print(F("-")); SerialUSB.print(s.timing.p2Max);
        SerialUSB.print(F(" p3=")); SerialUSB.print(s.timing.p3Min);
        SerialUSB.print(F("-")); SerialUSB.print(s.timing.p3Max);
        SerialUSB.print(F(" p4=")); SerialUSB.print(s.timing.p4Min);
        SerialUSB.print(F("-")); SerialUSB.println(s.timing.p4Max);
        return;
    }
    bool mayTx = (cur == Mode::ELM327 || cur == Mode::STN);
    if (strcasecmp(tok, "init") == 0) {
        if (!mayTx) { SerialUSB.println(F("refused: needs ELM/STN mode (transmit authority)")); return; }
        char* a = strtok_r(nullptr, " \t", &save);
        bool fast = (a != nullptr && strcasecmp(a, "fast") == 0);
        if (a == nullptr || (!fast && strcasecmp(a, "slow") != 0)) {
            SerialUSB.println(F("usage: kline init slow|fast")); return;
        }
        kline::release();  // re-init from a clean state (idempotent)
        if (!kline::acquire()) { SerialUSB.println(F("acquire failed")); return; }
        bool ok = fast ? kline::initFast() : kline::initSlow();
        SerialUSB.println(ok ? F("init OK") : F("init FAILED"));
        return;
    }
    if (strcasecmp(tok, "tx") == 0) {
        if (!mayTx) { SerialUSB.println(F("refused: needs ELM/STN mode (transmit authority)")); return; }
        char* a = strtok_r(nullptr, " \t", &save);
        uint8_t txb[64];
        int n = (a != nullptr) ? p6HexBytes(a, txb, sizeof(txb)) : -1;
        if (n <= 0) { SerialUSB.println(F("usage: kline tx <hexbytes>")); return; }
        if (!kline::acquired()) { SerialUSB.println(F("not acquired (kline init first)")); return; }
        uint8_t rx[80]; uint8_t rxLen = 0;
        if (!kline::transact(txb, (uint8_t)n, rx, sizeof(rx), rxLen, 1000)) {
            SerialUSB.println(F("NO DATA")); return;
        }
        for (uint8_t i = 0; i < rxLen; i++) {
            if (rx[i] < 0x10) SerialUSB.print(F("0"));
            SerialUSB.print(rx[i], HEX);
            SerialUSB.print(F(" "));
        }
        SerialUSB.println();
        return;
    }
    if (strcasecmp(tok, "timing") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        kline::Timing t;
        kline::getTiming(t);
        if (a == nullptr || strcasecmp(a, "show") == 0) {
            SerialUSB.print(F("p1Min=")); SerialUSB.println(t.p1Min);
            SerialUSB.print(F("p1Max=")); SerialUSB.println(t.p1Max);
            SerialUSB.print(F("p2Min=")); SerialUSB.println(t.p2Min);
            SerialUSB.print(F("p2Max=")); SerialUSB.println(t.p2Max);
            SerialUSB.print(F("p3Min=")); SerialUSB.println(t.p3Min);
            SerialUSB.print(F("p3Max=")); SerialUSB.println(t.p3Max);
            SerialUSB.print(F("p4Min=")); SerialUSB.println(t.p4Min);
            SerialUSB.print(F("p4Max=")); SerialUSB.println(t.p4Max);
            return;
        }
        if (strcasecmp(a, "set") == 0) {
            char* aF = strtok_r(nullptr, " \t", &save);
            char* aV = strtok_r(nullptr, " \t", &save);
            uint32_t v = 0;
            if (aF == nullptr || aV == nullptr || !p6Dec(aV, v) || v > 60000) {
                SerialUSB.println(F("usage: kline timing set <p1min|p1max|p2min|p2max|p3min|p3max|p4min|p4max> <ms>"));
                return;
            }
            bool ok = true;
            if (strcasecmp(aF, "p1min") == 0) t.p1Min = (uint16_t)v;
            else if (strcasecmp(aF, "p1max") == 0) t.p1Max = (uint16_t)v;
            else if (strcasecmp(aF, "p2min") == 0) t.p2Min = (uint16_t)v;
            else if (strcasecmp(aF, "p2max") == 0) t.p2Max = (uint16_t)v;
            else if (strcasecmp(aF, "p3min") == 0) t.p3Min = (uint16_t)v;
            else if (strcasecmp(aF, "p3max") == 0) t.p3Max = (uint16_t)v;
            else if (strcasecmp(aF, "p4min") == 0) t.p4Min = (uint16_t)v;
            else if (strcasecmp(aF, "p4max") == 0) t.p4Max = (uint16_t)v;
            else ok = false;
            if (!ok) { SerialUSB.println(F("Bad field.")); return; }
            kline::setTiming(t);
            SerialUSB.println(F("OK"));
            return;
        }
        SerialUSB.println(F("usage: kline timing show | kline timing set <field> <ms>"));
        return;
    }
    if (strcasecmp(tok, "addrs") == 0) {
        char* aT = strtok_r(nullptr, " \t", &save);
        char* aS = strtok_r(nullptr, " \t", &save);
        uint32_t tg = 0, ts = 0;
        if (aT == nullptr || aS == nullptr || !p6Hex32(aT, tg) || !p6Hex32(aS, ts)
            || tg > 255 || ts > 255) {
            SerialUSB.println(F("usage: kline addrs <targetHex> <testerHex>"));
            return;
        }
        kline::setAddrs((uint8_t)tg, (uint8_t)ts);
        SerialUSB.println(F("OK"));
        return;
    }
    if (strcasecmp(tok, "tp") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        if (a == nullptr) { SerialUSB.println(F("usage: kline tp <ms|off>")); return; }
        if (strcasecmp(a, "off") == 0) { kline::testerPresentStop(); SerialUSB.println(F("OK")); return; }
        uint32_t v = 0;
        if (!p6Dec(a, v) || v < 100 || v > 60000) { SerialUSB.println(F("ms in 100..60000")); return; }
        kline::testerPresentStart(v);
        SerialUSB.println(F("OK"));
        return;
    }
    if (strcasecmp(tok, "release") == 0) {
        kline::release();
        SerialUSB.println(F("OK"));
        return;
    }
    SerialUSB.println(F("kline status | kline init slow|fast | kline tx <hex> | kline timing show|set | kline addrs <tgt> <tst> | kline tp <ms|off> | kline release"));
    SerialUSB.println(F("j1850 status | j1850 crcgate on|off | j1850 hdrlen 1|3"));
    SerialUSB.println(F("swcan status | swcan wake <ms> | swcan recover"));
}

// Phase 9: capability-gate console.
static void consoleCap(char* line)
{
    while (*line == ' ') line++;
    char* save = nullptr;
    char* tok = strtok_r(line, " \t", &save);
    if (tok == nullptr || strcasecmp(tok, "show") == 0) {
        SerialUSB.print(F("Capability grant: "));
        SerialUSB.println(capgate::name(capgate::current()));
        SerialUSB.println(F("Levels: 0=READ_ONLY 1=ALLOW_TRANSMIT 2=ALLOW_DIAGNOSTIC_WRITE"));
        SerialUSB.println(F("        3=ALLOW_ACTUATOR_CONTROL 4=ALLOW_FLASH_PROGRAMMING"));
        SerialUSB.println(F("Grants reset to the mode default on every mode switch."));
        return;
    }
    if (strcasecmp(tok, "grant") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        char* e = nullptr;
        long v = a ? strtol(a, &e, 10) : -1;
        if (a == nullptr || *e != '\0' || v < 0 || v > 4) {
            SerialUSB.println(F("usage: cap grant <0..4>"));
            return;
        }
        capgate::Grant g = capgate::escalate((capgate::Grant)v);
        SerialUSB.print(F("Grant now: "));
        SerialUSB.println(capgate::name(g));
        return;
    }
    SerialUSB.println(F("cap show | cap grant <0..4>"));
}

// Phase 9: discovery console. Delegates to the OBD2/UDS entry points,
// which stop their own bus activity before the scan.
static void consoleDiscover(char* line, Mode cur)
{
    while (*line == ' ') line++;
    char* save = nullptr;
    char* tok = strtok_r(line, " \t", &save);
    if (tok == nullptr || strcasecmp(tok, "status") == 0) {
        const discover::Result& r = discover::lastResult();
        SerialUSB.print(F("Discovery: "));
        if (!r.done) SerialUSB.println(F("no scan yet"));
        else {
            SerialUSB.print(r.nEcus); SerialUSB.print(F(" ECU(s), CAN"));
            SerialUSB.print(r.bus); SerialUSB.print(F(" @ ")); SerialUSB.println(r.baud);
        }
        return;
    }
    if (strcasecmp(tok, "list") == 0) { discover::print(discover::lastResult()); return; }
    if (strcasecmp(tok, "cancel") == 0) { discover::cancel(); SerialUSB.println(F("Cancel requested.")); return; }
    if (strcasecmp(tok, "apply") == 0) {
        if (discover::applyFilters(discover::lastResult()))
            SerialUSB.println(F("Filters narrowed to discovered ECUs."));
        else
            SerialUSB.println(F("Nothing to apply (no discovery result)."));
        return;
    }
    if (strcasecmp(tok, "start") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        uint8_t bus = 0;
        if (a) {
            if ((a[0] != '0' && a[0] != '1') || a[1] != '\0') {
                SerialUSB.println(F("usage: discover start [0|1]"));
                return;
            }
            bus = (uint8_t)(a[0] - '0');
        }
        SerialUSB.println(F("Discovering (CAN)..."));
        uint8_t rc;
        // Route through the mode that owns the bus logic.
        if (cur == Mode::OBD2) rc = obd2::mmDiscover(bus);
        else if (cur == Mode::UDS) rc = uds::mmDiscover(bus);
        else {
            SerialUSB.println(F("Switch to OBD2 or UDS mode first."));
            return;
        }
        if (rc == mm::ST_OK) discover::print(discover::lastResult());
        else SerialUSB.println(F("No ECU answered (or refused)."));
        return;
    }
    SerialUSB.println(F("discover start [0|1] | discover status | discover list | discover apply | discover cancel"));
}

// ---- Phase 12: auto-detect console ----
// EXPLICIT USER INITIATION ONLY. The engine takes over the buses:
// quiesce everything first, run (blocking), then re-enter the current
// mode to restore its ownership. K-line/J1850/SWCAN transmitting stages
// need the second explicit word 'tx' (plus the capgate, inside the engine).
static void consoleAutodetect(char* line)
{
    while (*line == ' ') line++;
    char* save = nullptr;
    char* tok = strtok_r(line, " \t", &save);
    if (tok == nullptr || strcasecmp(tok, "status") == 0) {
        autodet::print(autodet::lastResult());
        return;
    }
    if (strcasecmp(tok, "cancel") == 0) {
        autodet::cancel();
        SerialUSB.println(F("Cancel requested."));
        return;
    }
    if (strcasecmp(tok, "start") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        bool allowTx = (a && strcasecmp(a, "tx") == 0);
        SerialUSB.println(F("Auto-detect: EXPLICIT run. Taking over buses;"));
        SerialUSB.println(F("the current mode will be re-entered afterwards."));
        if (allowTx)
            SerialUSB.println(F("TX consent given: K-line/J1850/SWCAN active stages ENABLED."));
        else
            SerialUSB.println(F("No TX consent: CAN listen-only + passive J1850/SWCAN stages only."));
        ModeManager* self = modes::manager();
        if (self) self->quiesceBuses();
        bool ok = autodet::start(allowTx);
        if (self) self->reacquire();
        if (ok) autodet::print(autodet::lastResult());
        else SerialUSB.println(F("Auto-detect refused (capgate or already running)."));
        return;
    }
    SerialUSB.println(F("autodetect start [tx] | autodetect cancel | autodetect status"));
}

// ---- Phase 12: profile console ----
static void consoleProfile(char* line)
{
    while (*line == ' ') line++;
    char* save = nullptr;
    char* tok = strtok_r(line, " \t", &save);
    if (tok == nullptr || strcasecmp(tok, "show") == 0) {
        profiles::show();
        return;
    }
    if (strcasecmp(tok, "save") == 0) {
        char* name = strtok_r(nullptr, " \t", &save);
        if (!name) { SerialUSB.println(F("usage: profile save <name>")); return; }
        SerialUSB.println(profiles::save(name) ? F("Profile saved.") : F("Save FAILED."));
        return;
    }
    if (strcasecmp(tok, "load") == 0) {
        SerialUSB.println(profiles::load() ? F("Profile loaded.") : F("No stored profile."));
        return;
    }
    if (strcasecmp(tok, "clear") == 0) {
        profiles::clearProfile();
        SerialUSB.println(F("Profile cleared."));
        return;
    }
    SerialUSB.println(F("profile save <name> | load | show | clear"));
}

// ---- Phase 12: log console ----
static void consoleLog(char* line)
{
    while (*line == ' ') line++;
    char* save = nullptr;
    char* tok = strtok_r(line, " \t", &save);
    if (tok == nullptr || strcasecmp(tok, "status") == 0) {
        logrec::status();
        return;
    }
    if (strcasecmp(tok, "start") == 0) { logrec::start(); SerialUSB.println(F("OK")); return; }
    if (strcasecmp(tok, "stop") == 0) { logrec::stop(); SerialUSB.println(F("OK")); return; }
    if (strcasecmp(tok, "clear") == 0) { logrec::clear(); SerialUSB.println(F("OK")); return; }
    if (strcasecmp(tok, "trig") == 0) {
        char* k = strtok_r(nullptr, " \t", &save);
        char* s = strtok_r(nullptr, " \t", &save);
        if (!k) { SerialUSB.println(F("usage: log trig <kind 0..3|off> [svc-hex]")); return; }
        if (strcasecmp(k, "off") == 0) { logrec::clearTrigger(); SerialUSB.println(F("Trigger off.")); return; }
        uint16_t svc = s ? (uint16_t)strtoul(s, nullptr, 16) : 0xFFFF;
        logrec::setTrigger((uint8_t)atoi(k), svc);
        SerialUSB.println(F("Trigger set."));
        return;
    }
    if (strcasecmp(tok, "arm") == 0) {
        char* p = strtok_r(nullptr, " \t", &save);
        char* q = strtok_r(nullptr, " \t", &save);
        logrec::setPrePost(p ? (uint8_t)atoi(p) : 16, q ? (uint16_t)atoi(q) : 0);
        logrec::start();
        SerialUSB.println(F("Armed."));
        return;
    }
    SerialUSB.println(F("log start | stop | status | clear | trig <kind|off> [svc] | arm <pre> <post>"));
}

// ---- Phase 12: harness console ----
static void consoleHarness(char* line)
{
    while (*line == ' ') line++;
    char* save = nullptr;
    char* tok = strtok_r(line, " \t", &save);
    if (tok == nullptr || strcasecmp(tok, "status") == 0) {
        vharness::print(vharness::lastResult());
        return;
    }
    if (strcasecmp(tok, "selfcheck") == 0) {
        SerialUSB.println(vharness::selfcheck() ? F("LOOPBACK OK") : F("LOOPBACK FAILED"));
        return;
    }
    if (strcasecmp(tok, "run") == 0) {
        char* m = strtok_r(nullptr, " \t", &save);
        uint16_t mask = m ? (uint16_t)strtoul(m, nullptr, 16) : vharness::T_ALL;
        vharness::SuiteResult r;
        if (!vharness::runSuite(mask, r)) {
            SerialUSB.println(F("REFUSED: selfcheck has not passed (wire CAN0<->CAN1 first)."));
            return;
        }
        vharness::print(r);
        return;
    }
    SerialUSB.println(F("harness selfcheck | run [mask-hex] | status"));
}

// ---- Phase 12: DBC console ----
static void dbcDecodeCb(const dbcsig::SignalVal& v, void* ctx)
{
    (void)ctx;
    SerialUSB.print(F("  "));
    SerialUSB.print(v.name);
    SerialUSB.print(F("="));
    // Print phys1000 as fixed-point.
    int32_t p = v.phys1000;
    SerialUSB.print(p / 1000);
    SerialUSB.print(F("."));
    int32_t frac = p % 1000;
    if (frac < 0) frac = -frac;
    if (frac < 100) SerialUSB.print(F("0"));
    if (frac < 10) SerialUSB.print(F("0"));
    SerialUSB.print(frac);
    SerialUSB.print(F(" "));
    SerialUSB.print(v.unit);
    SerialUSB.print(F(" (raw "));
    SerialUSB.print(v.raw);
    SerialUSB.print(F(")"));
    if (v.en) { SerialUSB.print(F(" [")); SerialUSB.print(v.en); SerialUSB.print(F("]")); }
    SerialUSB.println();
}

static void consoleDbc(char* line)
{
    while (*line == ' ') line++;
    char* save = nullptr;
    char* tok = strtok_r(line, " \t", &save);
    if (tok == nullptr || strcasecmp(tok, "list") == 0) {
        SerialUSB.print(F("DBCs: "));
        SerialUSB.print(dbcsig::dbcCount());
        SerialUSB.print(F(" msgs: "));
        SerialUSB.print(dbcsig::msgCount());
        SerialUSB.print(F(" sigs: "));
        SerialUSB.print(dbcsig::sigCount());
        SerialUSB.print(F(" enums: "));
        SerialUSB.println(dbcsig::enumCount());
        for (uint8_t i = 0; i < dbcsig::dbcCount(); i++) {
            SerialUSB.print(F("  ["));
            SerialUSB.print(i);
            SerialUSB.print(F("] "));
            SerialUSB.println(dbcsig::dbcName(i));
        }
        return;
    }
    if (strcasecmp(tok, "info") == 0) {
        char* n = strtok_r(nullptr, " \t", &save);
        uint16_t idx = n ? (uint16_t)atoi(n) : 0;
        uint32_t id; uint8_t ext, dlc; uint16_t nSig;
        if (!dbcsig::msgInfo(idx, id, ext, dlc, nSig)) {
            SerialUSB.println(F("no such message"));
            return;
        }
        SerialUSB.print(F("msg"));
        SerialUSB.print(idx);
        SerialUSB.print(F(" id=0x"));
        SerialUSB.print(id, HEX);
        SerialUSB.print(ext ? F(" ext") : F(" std"));
        SerialUSB.print(F(" dlc="));
        SerialUSB.print(dlc);
        SerialUSB.print(F(" sigs="));
        SerialUSB.println(nSig);
        return;
    }
    if (strcasecmp(tok, "decode") == 0) {
        // decode <id-hex> <b0> <b1> ... <b7> — decode a raw frame on the spot.
        char* ids = strtok_r(nullptr, " \t", &save);
        if (!ids) { SerialUSB.println(F("usage: dbc decode <id-hex> <byte-hex...>")); return; }
        uint32_t id = strtoul(ids, nullptr, 16);
        uint8_t data[8] = {0};
        uint8_t len = 0;
        char* b;
        while ((b = strtok_r(nullptr, " \t", &save)) && len < 8)
            data[len++] = (uint8_t)strtoul(b, nullptr, 16);
        bool ext = (id > 0x7FF);
        uint16_t n = dbcsig::decode(id, ext, data, len, dbcDecodeCb, nullptr);
        SerialUSB.print(n);
        SerialUSB.println(F(" signal(s)"));
        uint16_t skipped = dbcsig::truncatedSkips();
        if (skipped) {
            SerialUSB.print(F("  ("));
            SerialUSB.print(skipped);
            SerialUSB.println(F(" skipped: frame shorter than signal span)"));
        }
        return;
    }
    if (strcasecmp(tok, "select") == 0) {
        // F-03: dbc select <idx|name|all> — scope ID lookup to one DBC so
        // shadowed duplicate CAN IDs decode under the right layout.
        char* a = strtok_r(nullptr, " \t", &save);
        if (!a) {
            int8_t cur = dbcsig::selectedDbc();
            SerialUSB.print(F("active DBC: "));
            if (cur < 0) SerialUSB.println(F("all (first-match)"));
            else SerialUSB.println(dbcsig::dbcName((uint8_t)cur));
            return;
        }
        if (strcasecmp(a, "all") == 0) {
            dbcsig::selectDbc(-1);
            SerialUSB.println(F("active DBC: all (first-match)"));
            return;
        }
        int8_t idx = -2;
        char* end = nullptr;
        long v = strtol(a, &end, 10);
        if (end && *end == '\0' && v >= 0 && v < dbcsig::dbcCount()) idx = (int8_t)v;
        else {
            for (uint8_t i = 0; i < dbcsig::dbcCount(); i++)
                if (strcasecmp(a, dbcsig::dbcName(i)) == 0) { idx = (int8_t)i; break; }
        }
        if (idx < 0) { SerialUSB.println(F("no such DBC (try: dbc list)")); return; }
        dbcsig::selectDbc(idx);
        SerialUSB.print(F("active DBC: "));
        SerialUSB.println(dbcsig::dbcName((uint8_t)idx));
        return;
    }
    SerialUSB.println(F("dbc list | info <n> | decode <id-hex> <byte-hex...> | select <idx|name|all>"));
}

void ModeManager::handleConsoleLine(char* line)
{
    // commands: help | mode list | mode <name> | status | uds ... | obd2 ...
    while (*line == ' ') line++;
    if (strcasecmp(line, "help") == 0) {
        SerialUSB.println(F("Commands: help | mode list | mode <name> | status |"));
        SerialUSB.println(F("          uds ... | obd2 ... | mitm ... (rule|stats|clear|stealth|dir)"));
        SerialUSB.println(F("          prog ... (ECU flashing — read docs/SAFETY_REVIEW.md first)"));
        SerialUSB.println(F("          txsched ... (Phase 6 TX scheduler)"));
        SerialUSB.println(F("          filter ... (Phase 6 unified bus filters)"));
        SerialUSB.println(F("          busstat [reset] (Phase 6 per-bus stats)"));
        SerialUSB.println(F("          isotp show | isotp set <field> <val> (Phase 6 transport cfg)"));
        SerialUSB.println(F("          cap show | cap grant <0..4> (Phase 9 capability gates)"));
        SerialUSB.println(F("          discover start|status|list|apply|cancel (Phase 9, OBD2/UDS modes)"));
        SerialUSB.println(F("          debug on | debug off  (bus-level logging, all modes)"));
        SerialUSB.println(F("          autodetect start [tx] | autodetect cancel|status (Phase 12,"));
        SerialUSB.println(F("            explicit only; CAN listen-only; K-line/J1850/SWCAN need 'tx')"));
        SerialUSB.println(F("          profile save <name> | load | show | clear (Phase 12)"));
        SerialUSB.println(F("          log start|stop|status|clear|trig <kind> <svc>|arm <pre> <post> (Phase 12)"));
        SerialUSB.println(F("          harness run [mask] | selfcheck | status (Phase 12, HARNESS mode)"));
        SerialUSB.println(F("          dbc list | info <n> | decode <id> | select <idx|name|all> (Phase 12 DBC signals)"));
        SerialUSB.println(F("          m2ret <CMD> (Phase 12 modernization; M2RET mode)"));
        SerialUSB.println(F("          slcan status (Phase 12; SLCAN mode)"));
        return;
    }
    if (strcasecmp(line, "status") == 0) {
        printStatus();
        return;
    }
    // Unified bus debug toggle (all modes).
    if (strncasecmp(line, "debug", 5) == 0 && (line[5] == ' ' || line[5] == '\0')) {
        const char* arg = line + 5;
        while (*arg == ' ') arg++;
        if (strcasecmp(arg, "on") == 0) {
            dbg::enabled = true;
            SerialUSB.println(F("Bus debug ON."));
        } else if (strcasecmp(arg, "off") == 0) {
            dbg::enabled = false;
            SerialUSB.println(F("Bus debug OFF."));
        } else {
            SerialUSB.print(F("Bus debug is "));
            SerialUSB.println(dbg::enabled ? F("ON") : F("OFF"));
        }
        return;
    }
    // Route "uds ..." / "obd2 ..." / "mitm ..." to the mode consoles
    // (config works any time; bus-touching commands are refused unless the
    // mode is active).
    if (strncasecmp(line, "uds", 3) == 0 && (line[3] == ' ' || line[3] == '\0')) {
        uds::console(line + 3);
        return;
    }
    if (strncasecmp(line, "obd2", 4) == 0 && (line[4] == ' ' || line[4] == '\0')) {
        obd2::console(line + 4);
        return;
    }
    // Route "mitm ..." to the MITM rule console (config works any time;
    // the bridge itself runs only in MITM mode).
    if (strncasecmp(line, "mitm", 4) == 0 && (line[4] == ' ' || line[4] == '\0')) {
        mitm::console(line + 4);
        return;
    }
    // Route "prog ..." to the PROG console (config works any time;
    // bus-touching commands are refused unless PROG is the active mode).
    if (strncasecmp(line, "prog", 4) == 0 && (line[4] == ' ' || line[4] == '\0')) {
        prog::console(line + 4);
        return;
    }
    // Phase 10: route "gateway ..." to the gateway console (config works
    // any time; forwarding runs only in GATEWAY mode).
    if (strncasecmp(line, "gateway", 7) == 0 && (line[7] == ' ' || line[7] == '\0')) {
        gateway::console(line + 7);
        return;
    }
    // Phase 6: transport-layer consoles (mode-independent shared services).
    if (strncasecmp(line, "txsched", 7) == 0 && (line[7] == ' ' || line[7] == '\0')) {
        consoleTxsched(line + 7);
        return;
    }
    if (strncasecmp(line, "filter", 6) == 0 && (line[6] == ' ' || line[6] == '\0')) {
        consoleFilter(line + 6);
        return;
    }
    if (strncasecmp(line, "busstat", 7) == 0 && (line[7] == ' ' || line[7] == '\0')) {
        consoleBusstat(line + 7);
        return;
    }
    if (strncasecmp(line, "isotp", 5) == 0 && (line[5] == ' ' || line[5] == '\0')) {
        consoleIsotp(line + 5);
        return;
    }
    // Phase 7: K-line console (bus-touching subcommands gated inside).
    if (strncasecmp(line, "kline", 5) == 0 && (line[5] == ' ' || line[5] == '\0')) {
        consoleKline(line + 5, currentMode);
        return;
    }
    // Phase 9: capability-gate console.
    if (strncasecmp(line, "cap", 3) == 0 && (line[3] == ' ' || line[3] == '\0')) {
        consoleCap(line + 3);
        return;
    }
    // Phase 9: discovery console (delegates to the OBD2/UDS entry points,
    // which stop their own bus activity first).
    if (strncasecmp(line, "discover", 8) == 0 && (line[8] == ' ' || line[8] == '\0')) {
        consoleDiscover(line + 8, currentMode);
        return;
    }
    // Phase 8: J1850 + SWCAN consoles (read-only except crcgate/hdrlen;
    // wake/recover are diagnostic actions on the already-acquired bus).
    if (strncasecmp(line, "j1850", 5) == 0 && (line[5] == ' ' || line[5] == '\0')) {
        consoleJ1850(line + 5);
        return;
    }
    if (strncasecmp(line, "swcan", 5) == 0 && (line[5] == ' ' || line[5] == '\0')) {
        consoleSwcan(line + 5);
        return;
    }
    // Phase 12: profiles / auto-detect / logging / harness / DBC consoles.
    if (strncasecmp(line, "autodetect", 10) == 0 && (line[10] == ' ' || line[10] == '\0')) {
        consoleAutodetect(line + 10);
        return;
    }
    if (strncasecmp(line, "profile", 7) == 0 && (line[7] == ' ' || line[7] == '\0')) {
        consoleProfile(line + 7);
        return;
    }
    if (strncasecmp(line, "log", 3) == 0 && (line[3] == ' ' || line[3] == '\0')) {
        consoleLog(line + 3);
        return;
    }
    if (strncasecmp(line, "harness", 7) == 0 && (line[7] == ' ' || line[7] == '\0')) {
        consoleHarness(line + 7);
        return;
    }
    if (strncasecmp(line, "dbc", 3) == 0 && (line[3] == ' ' || line[3] == '\0')) {
        consoleDbc(line + 3);
        return;
    }
    // Phase 12: m2ret/slcan modernization consoles (bus-touching commands
    // refused unless the matching mode is active).
    if (strncasecmp(line, "m2ret", 5) == 0 && (line[5] == ' ' || line[5] == '\0')) {
        m2ret::console(line + 5);
        return;
    }
    if (strncasecmp(line, "slcan", 5) == 0 && (line[5] == ' ' || line[5] == '\0')) {
        slcan::console(line + 5);
        return;
    }
    if (strncasecmp(line, "mode", 4) == 0) {
        const char* arg = line + 4;
        while (*arg == ' ') arg++;
        if (strcasecmp(arg, "list") == 0 || *arg == '\0') {
            SerialUSB.println(F("Modes:"));
            for (uint8_t i = 0; i < (uint8_t)Mode::COUNT; i++) {
                Mode m = (Mode)i;
                SerialUSB.print(F("  "));
                SerialUSB.print(modeName(m));
                if (m == currentMode) SerialUSB.print(F("  [current]"));
                if (!modeImplemented(m)) SerialUSB.print(F("  (not implemented yet)"));
                SerialUSB.println();
            }
            return;
        }
        Mode m;
        if (modeFromName(arg, m)) {
            requestMode(m);
        } else {
            SerialUSB.print(F("Unknown mode: "));
            SerialUSB.println(arg);
        }
        return;
    }
    SerialUSB.println(F("Unknown command. Type 'help'."));
}

void ModeManager::printStatus()
{
    SerialUSB.print(F("Mode: "));
    SerialUSB.println(modeName(currentMode));
    SerialUSB.print(F("CAN begun (enable-pin override active): CAN0="));
    SerialUSB.print(can0Begun ? F("yes") : F("no"));
    SerialUSB.print(F(" CAN1="));
    SerialUSB.println(can1Begun ? F("yes") : F("no"));
    SerialUSB.print(F("Settings: CAN0=")); SerialUSB.print(can0BaudSet);
    SerialUSB.print(F(" CAN1=")); SerialUSB.print(can1BaudSet);
    SerialUSB.print(F(" SWCAN=")); SerialUSB.print(swcanSpeedSet);
    SerialUSB.print(F(" features=0x")); SerialUSB.println(features, HEX);
    SerialUSB.println(F("Resources (Phase 5): M2RET: CAN0, CAN1, SWCAN, UART0/Bee. "
                         "ELM/STN: CAN0 or J1850 block + UART0/Bee. "
                         "UDS/OBD2/PROG: configured CAN bus + UART0/Bee (lazy). "
                         "MITM: CAN0 + CAN1 + UART0/Bee (begun at entry). "
                         "PROG entry auto-suspends the MITM rule table (safety)."));
}

// ---- mm control-channel ops ----
void ModeManager::mmGetStatus(uint8_t& mode, uint32_t& c0, uint32_t& c1,
                              uint32_t& sw, uint8_t& feat)
{
    mode = (uint8_t)currentMode;
    c0 = can0BaudSet; c1 = can1BaudSet; sw = swcanSpeedSet; feat = features;
}

static bool validCanBaud(uint32_t b)
{
    return b == 100000 || b == 125000 || b == 250000 || b == 500000 ||
           b == 800000 || b == 1000000;
}

uint8_t ModeManager::mmSetCanBaud(uint8_t bus, uint32_t baud, uint8_t& applied)
{
    applied = 0;
    if (bus > 1 || !validCanBaud(baud)) return mm::ST_BAD_ARG;
    if (bus == 0) can0BaudSet = baud; else can1BaudSet = baud;
    // Live-apply only in M2RET mode (GVRET cmd 5 is the other live path;
    // both write through here eventually). ELM applies its protocol baud
    // on next ATSP/activation — staged, documented.
    if (currentMode == Mode::M2RET) {
        // Grok audit: begin() can fail — only claim begun/present/applied
        // on success. The flags must mean "successfully begun".
        bool ok;
        if (bus == 0) { ok = Can0.begin(baud, 255) != 0; can0Begun = ok; }
        else { ok = Can1.begin(baud, 255) != 0; can1Begun = ok; }
        if (!ok) return mm::ST_ERR;
        busstat::setPresent(bus == 0 ? busstat::Bus::CAN_BUS0 : busstat::Bus::CAN_BUS1,
                            true);  // Phase 6
        m2ret::setPromiscuousMode();
        m2ret::setCanSpeeds(can0BaudSet, can1BaudSet);  // keep GVRET replies true
        applied = 1;
    }
    return mm::ST_OK;
}

uint8_t ModeManager::mmSetSwcanSpeed(uint32_t speed, uint8_t& applied)
{
    applied = 0;
    // TH8056 GMLAN speeds (VERIFIED: audit HW-055/HW-056 context).
    if (speed != 33333 && speed != 83333) return mm::ST_BAD_ARG;
    swcanSpeedSet = speed;
    if (currentMode == Mode::M2RET) {
        m2ret::swcanSetSpeed(speed);  // live re-setup if active
        applied = 1;
    }
    return mm::ST_OK;
}

uint8_t ModeManager::mmSetFeature(uint8_t feat, uint8_t enable)
{
    if (feat != mm::FEAT_J1850 && feat != mm::FEAT_SWCAN) return mm::ST_BAD_ARG;
    if (enable) features |= feat; else features &= (uint8_t)~feat;
    if (feat == mm::FEAT_J1850) {
        elm::setJ1850Enabled(enable != 0);
        // Audit fix M-3 (2026-10-08): STN shares the ELM core and can hold
        // the J1850 block too — the old ELM327-only check left the ISR
        // attached and the rail powered in STN mode after an explicit
        // user disable.
        if (!enable &&
            (currentMode == Mode::ELM327 || currentMode == Mode::STN)) {
            // Forcing the feature off while ELM holds J1850: release now
            // (safe; ELM will NO DATA on next J1850 use until re-enabled).
            j1850::release();
        }
    }
    return mm::ST_OK;
}

// ---- ELM/UDS/OBD2 bus primitives (Phase 2/3) ----
// G2 (2026-10-08): return success; the begun flag means "successfully
// begun" so callers and the resource logic see true hardware state.
bool canBegin0(uint32_t baud)
{
    bool ok = Can0.begin(baud, 255) != 0;  // 255: skip transceiver-enable pin driving
    can0Begun = ok;
    busstat::setPresent(busstat::Bus::CAN_BUS0, ok);  // Phase 6: bus-off polling gate
    return ok;
}

void canEnd0()
{
    // G3 (2026-10-08): clear the flag — it tracks current ownership,
    // not historical initialization.
    if (can0Begun) Can0.disable();  // pin-safe: enablePin==255 was set
    can0Begun = false;
    busstat::setPresent(busstat::Bus::CAN_BUS0, false);
}

bool canBegin1(uint32_t baud)
{
    bool ok = Can1.begin(baud, 255) != 0;  // 255: skip transceiver-enable pin driving
    can1Begun = ok;
    busstat::setPresent(busstat::Bus::CAN_BUS1, ok);
    return ok;
}

void canEnd1()
{
    // G3 (2026-10-08): clear the flag — it tracks current ownership,
    // not historical initialization.
    if (can1Begun) Can1.disable();  // pin-safe: enablePin==255 was set
    can1Begun = false;
    busstat::setPresent(busstat::Bus::CAN_BUS1, false);
}

} // namespace modes
