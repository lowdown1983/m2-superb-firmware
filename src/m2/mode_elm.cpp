// M2 multi-mode firmware — ELM327 mode implementation (Phase 2).
// See mode_elm.h for the contract.
//
// AT response formats follow genuine ELM327 behavior:
//   - every response ends with "\r" (+ "\n" if ATL1) and the ">" prompt;
//   - "OK" for accepted AT commands, "?" for unknown, "NO DATA" on timeout,
//   - "STOPPED" when a monitor session is interrupted (genuine ELM327).
// Echo (ATE, default on), spaces (ATS, default on), headers (ATH, default
// off), linefeeds (ATL, default off) match ELM327 power-on defaults.
// (INFERRED: verified against the ELM327 command summary, not a live chip.)
//
// OBD-II over CAN uses ISO 15765-4 11/29-bit with the shared isotp module;
// over J1850 it uses 3-byte headers (default 68 6A F1, ATSH-overridable)
// with the j1850 driver (CRC handled there).

#include "mode_elm.h"
#include "debug.h"
#include "mode_manager.h"  // modes::canBegin0/canEnd0
#include "mm_channel.h"
#include "isotp.h"
#include "j1850.h"
#include "kline.h"      // Phase 7: ISO 9141-2 / KWP2000
#include "busfilter.h"  // Phase 6: shared idMatch() primitive
#include "busstat.h"    // Phase 6: RX stats in the monitor path
#include "capgate.h"    // Phase 9: capability gates
#include "silent_pins.h"  // Audit fix F-2: TJA1051 S-pin drive on listen-only
#include <due_can.h>

namespace elm {

namespace {

// ---- ELM state (power-on defaults = genuine ELM327) ----
bool echoOn = true;
bool linefeed = false;
bool headers = false;
bool spaces = true;
uint8_t adaptive = 1;        // ATAT: stored, timing not adapted (INFERRED)
uint8_t stSetting = 32;      // ATST: timeout in 4ms units (genuine default 32)
bool allowLong = true;       // ATAL (we always allow multi-frame anyway)
bool caf = true, cfc = true; // ATCAF/ATCFC (stored)
bool j1850en = true;         // feature gate from manager
bool protoAuto = true;       // ATSP0
uint8_t protoNum = 0;        // 0=auto, else 1,2,6,7,8,9,0xB,0xC
uint8_t activeProto = 0;     // resolved protocol in use (0 = none yet)
uint8_t hdr1850[3] = { 0x68, 0x6A, 0xF1 };  // ATSH for J1850 (INFERRED default)
uint8_t hdrKline[3] = { 0x68, 0x6A, 0xF1 };  // Phase 7: ATSH for K-line
                                            // (INFERRED default; reset per
                                            // protocol at busActivate)
// Phase 9: ELM depth.
// ATSH CAN header-override application (was accepted-but-not-applied):
uint32_t canTxIdOvr = 0;   // override TX arbitration ID...
bool canTxIdSet = false;   // ...when true (reset on ATZ/protocol change)
// ATCRA/ATCF/ATCM CAN RX filtering (software, via shared idMatch):
uint32_t canRxFilter = 0;
uint32_t canRxMask = 0;    // 0 = accept all (default)
bool canRxFilterSet = false;
// KWP2000 init controls (v1.5-era AT commands; INFERRED defaults):
bool kwCheck = true;       // ATKW0/1: keyword checking
uint8_t isoInitAddr = 0x33;  // ATIIA: init (target) address
uint32_t isoBaud = 10400;  // ATIB: ISO baud (10->10400, 48->4800, 96->9600)
uint8_t wakeupMult = 0;    // ATSW: wakeup interval multiplier (stored)
uint8_t wakeupMsg[8];      // ATWM: wakeup message bytes (stored)
uint8_t wakeupLen = 0;
bool autoRx = true;        // ATAR0/1: automatic receive (stored)

// ELM327 protocol numbers (genuine). B/C are user-defined on real chips;
// here B = CAN 11/125k, C = CAN 11/100k (documented assignment).
// Phase 7: 3/4/5 are the K-line protocols (ISO 9141-2, KWP2000 5-baud,
// KWP2000 fast init). A (genuine: SAE J1939) is deliberately NOT added —
// J1939 needs different addressing/transport semantics, out of Phase 7's
// K-line scope (see docs/PHASE7_NOTES.md).
struct PDesc {
    uint8_t num; const char* desc; bool isCan; bool isJ1850; bool isKline;
    uint32_t canBaud; bool ext; j1850::Proto jp; kline::Init kInit;
};
const PDesc PROTOS[] = {
    { 1,    "SAE J1850 PWM",          false, true,  false, 0,      false, j1850::Proto::JPWM, kline::Init::NONE },
    { 2,    "SAE J1850 VPW",          false, true,  false, 0,      false, j1850::Proto::VPW,  kline::Init::NONE },
    { 3,    "ISO 9141-2",             false, false, true,  0,      false, j1850::Proto::VPW,  kline::Init::SLOW_5BAUD },
    { 4,    "ISO 14230-4 KWP (5 baud init)", false, false, true,  0,      false, j1850::Proto::VPW,  kline::Init::SLOW_5BAUD },
    { 5,    "ISO 14230-4 KWP (fast init)",   false, false, true,  0,      false, j1850::Proto::VPW,  kline::Init::FAST },
    { 6,    "ISO 15765-4 (CAN 11/500)", true, false, false, 500000, false, j1850::Proto::VPW, kline::Init::NONE },
    { 7,    "ISO 15765-4 (CAN 29/500)", true, false, false, 500000, true,  j1850::Proto::VPW, kline::Init::NONE },
    { 8,    "ISO 15765-4 (CAN 11/250)", true, false, false, 250000, false, j1850::Proto::VPW, kline::Init::NONE },
    { 9,    "ISO 15765-4 (CAN 29/250)", true, false, false, 250000, true,  j1850::Proto::VPW, kline::Init::NONE },
    { 0xB,  "USER1 (CAN 11/125)",     true,  false, false, 125000, false, j1850::Proto::VPW, kline::Init::NONE },
    { 0xC,  "USER2 (CAN 11/100)",     true,  false, false, 100000, false, j1850::Proto::VPW, kline::Init::NONE },
};
const PDesc* findProto(uint8_t n)
{
    for (uint8_t i = 0; i < sizeof(PROTOS) / sizeof(PROTOS[0]); i++)
        if (PROTOS[i].num == n) return &PROTOS[i];
    return nullptr;
}

// Auto-detect order. INFERRED: not specified by ELM; chosen most-common-first.
const uint8_t AUTO_ORDER[] = { 6, 8, 7, 9, 2, 1 };

// ---- bus primitives ----
bool canActive = false;

// mm-channel divert state for the ELM byte pump (shared with monitor()).
bool mmBinActive = false;
bool sawF1 = false;

void busRelease()
{
    if (canActive) { modes::canEnd0(); canActive = false; }
    j1850::release();
    kline::release();  // Phase 7: idempotent; rail refcount keeps J1850 safe
    activeProto = 0;
}

bool busActivate(uint8_t pnum)
{
    const PDesc* pd = findProto(pnum);
    if (!pd) return false;
    if (pd->isJ1850 && !j1850en) return false;  // feature gate
    busRelease();
    if (pd->isCan) {
        // G2: honor the begin() result — don't claim the bus if init failed.
        if (!modes::canBegin0(pd->canBaud)) return false;
        // Promiscuous RX: ELM filters in software by ID (functional 0x7DF
        // gets 0x7E8-0x7EF answers). Open all mailboxes like M2RET's
        // setPromiscuousMode() (M2RET.ino:398-412).
        for (int m = 0; m < 7; m++) {
            Can0.setRXFilter(m, 0, 0, (m < 3));  // ids 0-2 ext, 3-6 std
        }
        canActive = true;
    } else if (pd->isJ1850) {
        if (!j1850::acquire(pd->jp)) return false;
    } else {
        // Phase 7: K-line. The init sequence TRANSMITS on the bus by
        // construction (5-baud 0x33 / fast-init wake + StartCommunication) —
        // this call site IS the transmit-authority gate: busActivate runs
        // only from user-initiated OBD/ATTP/monitor activity via
        // ensureProtocol(), the same gate as ELM's CAN/J1850 OBD commands
        // (see docs/SAFETY_REVIEW.md, Phase 7 section).
        if (!kline::acquire()) return false;
        // Phase 9: ATIIA init address feeds the K-line target address.
        kline::setAddrs(isoInitAddr, 0xF1);
        bool ok = (pd->kInit == kline::Init::FAST)
                      ? kline::initFast()
                      : kline::initSlow(isoInitAddr);
        if (!ok) { kline::release(); return false; }
        // INFERRED default headers per protocol (ATSH-overridable):
        // 9141 -> 68 6A F1; KWP2000 -> C1 33 F1.
        if (pnum == 3)      { hdrKline[0] = 0x68; hdrKline[1] = 0x6A; hdrKline[2] = 0xF1; }
        else               { hdrKline[0] = 0xC1; hdrKline[1] = 0x33; hdrKline[2] = 0xF1; }
    }
    activeProto = pnum;
    return true;
}

uint16_t obdTimeoutMs() { return (uint16_t)stSetting * 4; }

// Forward: defined below atCommand().
void monitor(uint32_t filterId, bool useFilter, int32_t maxFrames);

// Phase 4: STN state (defined in the STN block further below; declared here
// so monitor() can honor STCMM during STM/STMA/STMR/STMT sessions).
extern bool stnMode;
extern uint8_t stCmm;

// ---- response formatting ----
void printHex2(uint8_t b)
{
    const char* H = "0123456789ABCDEF";
    Serial.write((uint8_t)H[(b >> 4) & 0xF]);
    Serial.write((uint8_t)H[b & 0xF]);
}

void endLine()
{
    Serial.write('\r');
    if (linefeed) Serial.write('\n');
}

void prompt()
{
    Serial.write('\r');
    if (linefeed) Serial.write('\n');
    Serial.write('>');
}

// Print one data line: bytes as hex, spaces per ATS, headers per ATH.
void printDataLine(const uint8_t* hdr, uint8_t hdrLen,
                   const uint8_t* data, uint8_t dataLen)
{
    bool first = true;
    auto put = [&](uint8_t b) {
        if (!first && spaces) Serial.write(' ');
        printHex2(b);
        first = false;
    };
    if (headers) for (uint8_t i = 0; i < hdrLen; i++) put(hdr[i]);
    for (uint8_t i = 0; i < dataLen; i++) put(data[i]);
    endLine();
}

void reply(const char* s)  // single-line text reply + prompt
{
    // Genuine ELM327 framing (2026-10-10): text replies terminate \r\r>
    // (was \r> — a strict-parser deviation found by differential probe).
    Serial.print(s);
    Serial.write('\r');
    prompt();
}

// ---- OBD-II over CAN ----
void obdCan(const PDesc* pd, const uint8_t* reqBytes, uint8_t reqLen)
{
    // Phase 9: ATSH CAN header-override application. When set, the TX
    // arbitration ID comes from ATSH instead of functional addressing.
    uint32_t txId = canTxIdSet ? canTxIdOvr
                               : (pd->ext ? 0x18DB33F1UL : 0x7DF);
    isotp::Endpoint ep{ &Can0, txId, 0, pd->ext };
    dbg::hexBytes(F("ELM TX"), txId, reqBytes, reqLen);
    isotp::Response resps[8];  // 2026-10-10: was [4]; the Bolt EV has 5
    // ECUs answering functional requests — the 5th response was dropped,
    // hiding its PIDs from app sensor scans.
    // 2026-10-08: quietMs 200->25. The 200ms multi-ECU listen window capped
    // throughput at ~5 PIDs/sec; 25ms still collects staggered ECU replies
    // but doesn't idle on single-ECU vehicles (Bolt EV: 6.9 -> ~30/sec).
    uint8_t n = isotp::requestMulti(ep, reqBytes, reqLen, resps, 8,
                                    obdTimeoutMs(), 25);
    if (n == 0) { dbg::println(F("ELM: NO DATA (no ECU response)")); reply("NO DATA"); return; }
    for (uint8_t i = 0; i < n; i++) {
        // Phase 9: ATCRA/ATCF/ATCM software RX filter (shared idMatch).
        if (canRxFilterSet &&
            !busfilter::idMatch(resps[i].senderId, canRxFilter, canRxMask))
            continue;
        dbg::hexBytes(F("ELM RX"), resps[i].senderId, resps[i].data, resps[i].len);
        uint8_t hdr[4]; uint8_t hdrLen = 0;
        if (pd->ext) {
            // 29-bit header echo: priority/type/target/source bytes.
            hdr[0] = (uint8_t)(resps[i].senderId >> 24);
            hdr[1] = (uint8_t)(resps[i].senderId >> 16);
            hdr[2] = (uint8_t)(resps[i].senderId >> 8);
            hdr[3] = (uint8_t)resps[i].senderId;
            hdrLen = 4;
        } else {
            hdr[0] = (uint8_t)(resps[i].senderId >> 8);
            hdr[1] = (uint8_t)resps[i].senderId;
            hdrLen = 2;
        }
        printDataLine(hdr, hdrLen, resps[i].data, resps[i].len);
    }
    prompt();
}

// ---- OBD-II over J1850 ----
void obdJ1850(const PDesc* pd, const uint8_t* reqBytes, uint8_t reqLen)
{
    (void)pd;
    uint8_t tx[16];
    if ((size_t)reqLen + 3 > sizeof(tx)) { reply("?"); return; }
    tx[0] = hdr1850[0]; tx[1] = hdr1850[1]; tx[2] = hdr1850[2];
    for (uint8_t i = 0; i < reqLen; i++) tx[3 + i] = reqBytes[i];
    uint8_t rx[32]; uint8_t rxLen = 0;
    // INFERRED timeout floor: J1850 ECUs can be slow; use max(ATST, 1s).
    uint16_t to = obdTimeoutMs() < 1000 ? 1000 : obdTimeoutMs();
    if (!j1850::transact(tx, (uint8_t)(reqLen + 3), rx, sizeof(rx), rxLen, to)) {
        reply("NO DATA");
        return;
    }
    // rx = header(3) + data (CRC stripped by driver).
    if (rxLen <= 3) { reply("NO DATA"); return; }
    printDataLine(rx, 3, rx + 3, (uint8_t)(rxLen - 3));
    prompt();
}

// ---- OBD-II over K-line (Phase 7) ----
void obdKline(const PDesc* pd, const uint8_t* reqBytes, uint8_t reqLen)
{
    uint8_t rx[80]; uint8_t rxLen = 0;
    // INFERRED timeout floor (like J1850): K-line ECUs can be slow.
    uint16_t to = obdTimeoutMs() < 1000 ? 1000 : obdTimeoutMs();
    bool ok;
    if (pd->num == 3) {
        // ISO 9141-2: [hdr0 hdr1 hdr2] + mode/PID + checksum.
        ok = kline::request9141(hdrKline, reqBytes, reqLen,
                                rx, sizeof(rx), rxLen, to);
    } else {
        // KWP2000: first request byte is the SID, rest is data.
        if (reqLen == 0) { reply("?"); return; }
        ok = kline::kwpService(reqBytes[0], reqBytes + 1,
                               (uint8_t)(reqLen - 1),
                               rx, sizeof(rx), rxLen, to);
    }
    if (dbg::enabled) dbg::hexBytes(F("ELM KLINE RX"), 0, rx, rxLen);
    if (!ok || rxLen == 0) {
        dbg::println(F("ELM: NO DATA (K-line)"));
        reply("NO DATA");
        return;
    }
    // rx = header + data (checksum stripped by driver).
    if (rxLen <= 3) printDataLine(nullptr, 0, rx, rxLen);
    else printDataLine(rx, 3, rx + 3, (uint8_t)(rxLen - 3));
    prompt();
}

// Auto protocol detect: try 0100 in AUTO_ORDER. INFERRED order.
//
// Phase 12 hook: K-line probing (kline::probe()) belongs to the Phase 12
// auto-detection engine. K-line protocols are deliberately NOT in
// AUTO_ORDER — 5-baud/fast init is slow and transmits on the bus by
// construction, so it must not run as a side effect of ATSP0.
bool autoDetect()
{
    uint8_t probe[2] = { 0x01, 0x00 };
    dbg::println(F("ELM: auto-detect start"));
    for (uint8_t i = 0; i < sizeof(AUTO_ORDER); i++) {
        uint8_t p = AUTO_ORDER[i];
        const PDesc* pd = findProto(p);
        if (!pd || (pd->isJ1850 && !j1850en)) continue;
        dbg::print(F("ELM: trying proto ")); dbg::print(p); dbg::println(F("..."));
        if (!busActivate(p)) continue;
        bool got = false;
        if (pd->isCan) {
            uint32_t txId = pd->ext ? 0x18DB33F1UL : 0x7DF;
            isotp::Endpoint ep{ &Can0, txId, 0, pd->ext };
            uint8_t r[8]; uint8_t rl = 0;
            got = isotp::request(ep, probe, 2, r, sizeof(r), rl, 500);
            dbg::print(F("ELM: CAN probe ")); dbg::println(got ? F("GOT RESPONSE") : F("no response"));
        } else {
            uint8_t tx[5] = { hdr1850[0], hdr1850[1], hdr1850[2], 0x01, 0x00 };
            uint8_t rx[32]; uint8_t rl = 0;
            got = j1850::transact(tx, 5, rx, sizeof(rx), rl, 800);
            dbg::print(F("ELM: J1850 probe ")); dbg::println(got ? F("GOT RESPONSE") : F("no response"));
        }
        if (got) { protoNum = p; dbg::print(F("ELM: proto locked: ")); dbg::println(p); return true; }
    }
    busRelease();
    dbg::println(F("ELM: auto-detect FAILED (no ECU answered)"));
    return false;
}

bool ensureProtocol()
{
    if (activeProto != 0) return true;
    if (protoAuto) return autoDetect();
    return busActivate(protoNum);
}

// ---- AT command dispatch ----
void atCommand(const char* c)  // c = text after "AT", already uppercased
{
    // Two-char commands with numeric/on-off suffixes first.
    if (c[0] == 'Z' && c[1] == '\0') {           // ATZ: reset
        busRelease();
        begin();                                 // power-on defaults
        Serial.print(F("\rELM327 v1.5\r"));      // genuine \r\r> framing
        prompt();
        return;
    }
    if (c[0] == 'I' && c[1] == '\0') { reply("ELM327 v1.5"); return; }
    if (!strcmp(c, "@1")) { reply("M2-SuperB"); return; }  // device description
    if (c[0] == 'E' && (c[1] == '0' || c[1] == '1') && c[2] == '\0') {
        echoOn = (c[1] == '1'); reply("OK"); return;
    }
    if (c[0] == 'L' && (c[1] == '0' || c[1] == '1') && c[2] == '\0') {
        linefeed = (c[1] == '1'); reply("OK"); return;
    }
    if (c[0] == 'S' && (c[1] == '0' || c[1] == '1') && c[2] == '\0') {
        spaces = (c[1] == '1'); reply("OK"); return;
    }
    if (c[0] == 'H' && (c[1] == '0' || c[1] == '1') && c[2] == '\0') {
        headers = (c[1] == '1'); reply("OK"); return;
    }
    if (!strncmp(c, "SP", 2) && c[2] != '\0' && c[3] == '\0') {
        // ATSP n: n = 0-9, A, B, C (hex digit).
        char d = c[2];
        uint8_t n = (d <= '9') ? (uint8_t)(d - '0') : (uint8_t)(10 + d - 'A');
        if (d == '0') { protoAuto = true; protoNum = 0; busRelease(); reply("OK"); return; }
        const PDesc* pd = findProto(n);
        if (!pd || (pd->isJ1850 && !j1850en)) { reply("?"); return; }
        protoAuto = false; protoNum = n;
        busRelease();  // genuine ELM inits on next OBD command
        reply("OK");
        return;
    }
    if (!strncmp(c, "TP", 2) && c[2] != '\0' && c[3] == '\0') {
        // ATTP: try protocol (same parse as ATSP).
        char d = c[2];
        uint8_t n = (d <= '9') ? (uint8_t)(d - '0') : (uint8_t)(10 + d - 'A');
        const PDesc* pd = findProto(n);
        if (!pd || (pd->isJ1850 && !j1850en)) { reply("?"); return; }
        if (!busActivate(n)) { reply("NO DATA"); return; }
        protoAuto = false; protoNum = n;
        reply("OK");
        return;
    }
    if (!strncmp(c, "DP", 2) && c[2] == '\0') {  // ATDP
        if (!ensureProtocol()) { reply("NO DATA"); return; }
        reply(findProto(activeProto)->desc);
        return;
    }
    if (!strncmp(c, "DPN", 3) && c[3] == '\0') {  // ATDPN
        // Genuine: "A0" when protocol is AUTO and nothing connected yet
        // (2026-10-10: was "0", a deviation that can confuse app init).
        char s[3] = { (char)(protoNum < 10 ? '0' + protoNum : 'A' + protoNum - 10), '\0', '\0' };
        if (protoAuto && activeProto == 0) { s[0] = 'A'; s[1] = '0'; }
        else if (protoAuto && activeProto != 0) {
            uint8_t a = activeProto;
            s[0] = (char)(a < 10 ? '0' + a : 'A' + a - 10);
        }
        reply(s);
        return;
    }
    if (!strcmp(c, "RV")) { reply("12.4V"); return; }  // INFERRED: no calibrated ADC yet
    if (!strncmp(c, "AT", 2) && c[2] >= '0' && c[2] <= '2' && c[3] == '\0') {
        adaptive = (uint8_t)(c[2] - '0'); reply("OK"); return;  // stored only
    }
    if (!strncmp(c, "ST", 2) && c[2] != '\0') {
        // ATST hh: hex timeout in 4ms units (genuine).
        char* e; long v = strtol(c + 2, &e, 16);
        if (*e == '\0' && v >= 0 && v <= 255) { stSetting = (uint8_t)v; reply("OK"); }
        else reply("?");
        return;
    }
    if (!strncmp(c, "CAF", 3) && (c[3] == '0' || c[3] == '1') && c[4] == '\0') {
        caf = (c[3] == '1'); reply("OK"); return;
    }
    if (!strncmp(c, "CFC", 3) && (c[3] == '0' || c[3] == '1') && c[4] == '\0') {
        cfc = (c[3] == '1'); reply("OK"); return;
    }
    if (!strcmp(c, "PB")) { reply("OK"); return; }  // protocol B options: accepted
    if (!strcmp(c, "BI")) { reply("OK"); return; }  // bypass init
    if (!strcmp(c, "PC")) { busRelease(); reply("OK"); return; }  // protocol close
    if (!strcmp(c, "WS")) { begin(); reply("OK"); return; }       // warm start
    if (!strcmp(c, "AL")) { allowLong = true; reply("OK"); return; }
    if (!strcmp(c, "NL")) { allowLong = false; reply("OK"); return; }
    if (!strncmp(c, "SH", 2)) {
        // ATSH: set header.
        // Phase 9: the CAN header-override is now APPLIED (was
        // accepted-but-not-applied). Genuine ELM327 semantics:
        //  - CAN 11-bit protocols (6/8/B/C): up to 3 hex digits set the TX
        //    arbitration ID (e.g. ATSH 7E0).
        //  - CAN 29-bit protocols (7/9): up to 8 hex digits set the TX ID
        //    (e.g. ATSH 18DB33F1).
        //  - J1850/K-line: exactly 3 header bytes (INFERRED defaults
        //    68 6A F1). Phase 7: routes to the K-line header when a K-line
        //    protocol is selected (ATSP's protoNum wins; else active).
        const PDesc* apd = nullptr;
        if (protoNum != 0) apd = findProto(protoNum);
        else if (activeProto != 0) apd = findProto(activeProto);
        const char* h = c + 2;
        size_t hl = strlen(h);
        if (apd != nullptr && apd->isCan) {
            char* e; unsigned long v = strtoul(h, &e, 16);
            uint32_t maxId = apd->ext ? 0x1FFFFFFFUL : 0x7FFUL;
            if (hl == 0 || hl > 8 || *e != '\0' || v > maxId) { reply("?"); return; }
            canTxIdOvr = (uint32_t)v;
            canTxIdSet = true;
            reply("OK");
            return;
        }
        uint8_t* hdr = (apd != nullptr && apd->isKline) ? hdrKline : hdr1850;
        if (hl == 6) {
            uint8_t b[3]; bool ok = true;
            for (uint8_t i = 0; i < 3; i++) {
                char tmp[3] = { h[2 * i], h[2 * i + 1], '\0' };
                char* e; long v = strtol(tmp, &e, 16);
                if (*e != '\0') ok = false; else b[i] = (uint8_t)v;
            }
            if (ok) { hdr[0] = b[0]; hdr[1] = b[1]; hdr[2] = b[2]; }
            reply(ok ? "OK" : "?");
        } else {
            reply("OK");
        }
        return;
    }
    // Phase 9: ATCRA/ATCF/ATCM — CAN RX filter commands (genuine ELM327).
    // ATCRA/ATCF set the receive-address filter; ATCM sets the mask.
    // Applied in software on the RX display paths (monitor + OBD replies).
    if (!strncmp(c, "CRA", 3) || !strncmp(c, "CF", 2)) {
        const char* h = (c[1] == 'R') ? c + 3 : c + 2;  // CRA -> +3, CF -> +2
        char* e; unsigned long v = strtoul(h, &e, 16);
        size_t hl = strlen(h);
        if (hl == 0 || hl > 8 || *e != '\0' || v > 0x1FFFFFFFUL) { reply("?"); return; }
        canRxFilter = (uint32_t)v;
        if (canRxMask == 0) canRxMask = 0x1FFFFFFFUL;  // exact-ID default
        canRxFilterSet = true;
        reply("OK");
        return;
    }
    if (!strncmp(c, "CM", 2)) {
        const char* h = c + 2;
        char* e; unsigned long v = strtoul(h, &e, 16);
        size_t hl = strlen(h);
        if (hl == 0 || hl > 8 || *e != '\0' || v > 0x1FFFFFFFUL) { reply("?"); return; }
        canRxMask = (uint32_t)v;
        canRxFilterSet = (v != 0) || canRxFilterSet;
        if (v == 0) canRxFilterSet = false;  // mask 0 = accept all = filter off
        reply("OK");
        return;
    }
    // Phase 9: remaining high-value v1.5-era AT commands (parity gaps).
    if (!strcmp(c, "FI")) {
        // ATFI: fast init (KWP2000). Runs on the K-line when acquired.
        if (!ensureProtocol()) { reply("NO DATA"); return; }
        const PDesc* fpd = findProto(activeProto);
        if (fpd == nullptr || !fpd->isKline) { reply("?"); return; }
        reply(kline::initFast() ? "OK" : "NO DATA");
        return;
    }
    if ((c[0] == 'K' && c[1] == 'W' && (c[2] == '0' || c[2] == '1') && c[3] == '\0')) {
        kwCheck = (c[2] == '1'); reply("OK"); return;  // ATKW: stored
    }
    if (!strncmp(c, "IIA", 3)) {
        // ATIIA hh: ISO init address (KWP target address).
        const char* h = c + 3;
        if (strlen(h) != 2) { reply("?"); return; }
        char tmp[3] = { h[0], h[1], '\0' };
        char* e; long v = strtol(tmp, &e, 16);
        if (*e != '\0') { reply("?"); return; }
        isoInitAddr = (uint8_t)v;
        reply("OK");
        return;
    }
    if (!strncmp(c, "IB", 2)) {
        // ATIB10/48/96: ISO baud rate (genuine ELM327 values).
        const char* h = c + 2;
        if (!strcmp(h, "10")) isoBaud = 10400;
        else if (!strcmp(h, "48")) isoBaud = 4800;
        else if (!strcmp(h, "96")) isoBaud = 9600;
        else { reply("?"); return; }
        reply("OK");
        return;
    }
    if (!strncmp(c, "SW", 2)) {
        // ATSW hh: wakeup interval multiplier (stored; INFERRED use).
        const char* h = c + 2;
        if (strlen(h) != 2) { reply("?"); return; }
        char tmp[3] = { h[0], h[1], '\0' };
        char* e; long v = strtol(tmp, &e, 16);
        if (*e != '\0') { reply("?"); return; }
        wakeupMult = (uint8_t)v;
        reply("OK");
        return;
    }
    if (!strncmp(c, "WM", 2)) {
        // ATWM: set wakeup message (stored hex bytes; INFERRED use).
        const char* h = c + 2;
        size_t hl = strlen(h);
        if (hl == 0 || (hl & 1) || hl > 16) { reply("?"); return; }
        wakeupLen = 0;
        for (size_t i = 0; i < hl; i += 2) {
            char tmp[3] = { h[i], h[i + 1], '\0' };
            char* e; long v = strtol(tmp, &e, 16);
            if (*e != '\0') { reply("?"); return; }
            wakeupMsg[wakeupLen++] = (uint8_t)v;
        }
        reply("OK");
        return;
    }
    if (!strcmp(c, "RD")) {
        // ATRD: read one available CAN frame (INFERRED: ATMA with 1 frame).
        if (!ensureProtocol()) { reply("NO DATA"); return; }
        const PDesc* rpd = findProto(activeProto);
        if (rpd == nullptr || !rpd->isCan) { reply("?"); return; }
        monitor(0, false, 1);
        return;
    }
    if (c[0] == 'A' && c[1] == 'R' && (c[2] == '0' || c[2] == '1') && c[3] == '\0') {
        autoRx = (c[2] == '1'); reply("OK"); return;  // ATAR: stored
    }
    if (!strncmp(c, "PP", 2)) { reply("OK"); return; }  // programmable params: accepted
    if (!strcmp(c, "D")) { begin(); reply("OK"); return; }  // set defaults
    if (!strcmp(c, "M")) { reply("OK"); return; }  // memory: accepted
    if (!strcmp(c, "RD")) { reply("?"); return; }  // INFERRED: not verified; Torque doesn't use it
    if (!strcmp(c, "MA")) { monitor(0, false, -1); return; }
    if (!strncmp(c, "MR", 2) || !strncmp(c, "MT", 2)) {
        // ATMR hh / ATMT hh: monitor with ID filter (INFERRED simplification:
        // CAN 11-bit ID filter, J1850 first-header-byte filter).
        char* e; long v = strtol(c + 2, &e, 16);
        if (*e != '\0' || v < 0 || v > 0x1FFFFFFF) { reply("?"); return; }
        monitor((uint32_t)v, true, -1);
        return;
    }
    reply("?");  // unknown AT command (genuine ELM327)
}

// ---- monitor (ATMA/MR/MT; STN: STM/STMA/STMR/STMT) ----
// maxFrames < 0 = run until interrupted (genuine ELM327); >= 0 = auto-exit
// after that many frames (STN STM [n] / STMA [n], FRPM §8.9).
void monitor(uint32_t filterId, bool useFilter, int32_t maxFrames)
{
    if (!ensureProtocol()) { reply("NO DATA"); return; }
    const PDesc* pd = findProto(activeProto);
    // STCMM 0 (Phase 4): silent monitoring — SAM3X listen-only mode
    // (CAN_MR_ABM via due_can's enable_autobaud_listen_mode()). The
    // controller transmits nothing: no ACKs, no error frames. Restored on
    // every exit path below. INFERRED on silicon (no bench); the ACK
    // suppression is a controller feature (due_can.cpp:530-534), not
    // something firmware can do per-frame.
    bool silent = (stnMode && stCmm == 0 && pd->isCan);
    if (silent) {
        Can0.enable_autobaud_listen_mode();
        silentpins::enter(0);  // HSC_S HIGH: transceiver-level silence (F-2)
    }
    int32_t shown = 0;
    for (;;) {
        // mm-channel divert first: control frames must not stop the monitor.
        while (Serial.available() > 0) {
            uint8_t b = Serial.read();
            if (mmBinActive) {
                if (mm::feedByte(b)) mmBinActive = false;
                continue;
            }
            if (b == mm::SYNC && !sawF1) { sawF1 = true; continue; }
            if (sawF1) {
                sawF1 = false;
                if (b == mm::CMD_MULTIMODE) { mm::reset(); mmBinActive = true; continue; }
                if (b == mm::SYNC) { sawF1 = true; continue; }  // F1 F1 7E edge
            }
            // Any other received character stops the monitor (genuine ELM327).
            if (silent) { Can0.disable_autobaud_listen_mode(); silentpins::exit(0); }
            reply("STOPPED");
            return;
        }
        if (pd->isCan) {
            while (Can0.available() > 0) {
                CAN_FRAME f;
                Can0.read(f);
                busstat::noteRx(busstat::Bus::CAN_BUS0, f.length);
                // Phase 6: the monitor's single-ID filter goes through the
                // shared idMatch() primitive (one implementation, not a
                // second copy of mask logic).
                if (useFilter &&
                    !busfilter::idMatch(f.id, filterId,
                                        pd->ext ? 0x1FFFFFFFUL : 0x7FFUL))
                    continue;
                // Phase 9: ATCRA/ATCF/ATCM filter also applies in monitor.
                if (canRxFilterSet &&
                    !busfilter::idMatch(f.id, canRxFilter, canRxMask))
                    continue;
                uint8_t hdr[4]; uint8_t hl = 0;
                if (pd->ext) {
                    hdr[0] = (uint8_t)(f.id >> 24); hdr[1] = (uint8_t)(f.id >> 16);
                    hdr[2] = (uint8_t)(f.id >> 8); hdr[3] = (uint8_t)f.id; hl = 4;
                } else { hdr[0] = (uint8_t)(f.id >> 8); hdr[1] = (uint8_t)f.id; hl = 2; }
                printDataLine(hdr, hl, f.data.bytes, f.length);
                if (maxFrames >= 0 && ++shown >= maxFrames) {
                    if (silent) { Can0.disable_autobaud_listen_mode(); silentpins::exit(0); }
                    prompt();  // auto-exit: back to prompt, not STOPPED (FRPM)
                    return;
                }
            }
        } else if (pd->isJ1850) {
            uint8_t buf[32]; uint8_t n = 0;
            while (j1850::pollFrame(buf, sizeof(buf), n)) {
                printDataLine(nullptr, 0, buf, n);
                if (maxFrames >= 0 && ++shown >= maxFrames) {
                    prompt();
                    return;
                }
            }
        } else {
            // Phase 7: K-line monitor. pollRx() assembles frames from the
            // UART by inter-byte gap; pollFrame() honors the shared filter
            // table (Phase 6 pattern).
            kline::pollRx();
            uint8_t buf[80]; uint8_t n = 0;
            while (kline::pollFrame(buf, sizeof(buf), n)) {
                printDataLine(nullptr, 0, buf, n);
                if (maxFrames >= 0 && ++shown >= maxFrames) {
                    prompt();
                    return;
                }
            }
        }
    }
}

// ---- STN extension (Phase 4) ----
// STN mode = this ELM core with ST-command parsing enabled. Genuine STN
// chips are ELM327 supersets (FRPM: "fully compatible with the ELM327 AT
// command set"), so every AT command above keeps working; the commands
// below are the extension. Semantics are from the OBDLink Family Reference
// and Programming Manual (FRPM Rev E, 2025-01-10) and the STN11XX datasheet
// Rev B (2009-12-04). Anything not verifiable there answers '?' — the
// subset boundary is documented in docs/PHASE4_NOTES.md.
bool stnMode = false;

// STN device identity (FRPM §8.4: STI -> "STN<device_id> vX.Y.Z",
// STDI -> "<device_name> rX.Y"). INFERRED: tools detect STN capability by
// the "STN" prefix (measured on a real OBDLink MX+: STI -> "STN2255
// v5.12.4", STDI -> "OBDLink MX+ r3.1.3"); the version string here is ours.
uint8_t stCmm = 0;      // STCMM: 0=rx-only/silent (default), 1=normal node,
                        // 2=rx error frames too (not supported on SAM3X)
uint8_t stCaf = 0;      // STCAF addressing format (0=normal)
uint8_t stCafTt = 0;    // STCAF target-address extension byte
bool stSegRx = false;   // STCSEGR (stored; see notes)
bool stSegTx = false;   // STCSEGT (stored; see notes)
uint16_t stCstmMs = 0;  // STCSTM STmin offset, ms (stored, not wired)
uint16_t stCtorFc = 75, stCtorCf = 150;  // STCTOR timeouts, ms (FRPM defaults)

struct StFcPair { uint32_t tx; uint32_t rx; };
StFcPair stFcPairs[4];
uint8_t nStFcPairs = 0;

// STPX with l: (data length) prints "DATA>" and waits for the next line.
struct StpxPend {
    bool active = false;
    uint32_t txId = 0;
    bool ext = false;
    uint8_t dataLen = 0;
    uint16_t tmoMs = 0;   // 0 = ATST default
    uint8_t rCount = 0;   // 0 = collect until timeout
    uint8_t xAddr = 0;    // Phase 6: STPX x: target address
    bool xGiven = false;
};
StpxPend stpxPend;

// Parse even-length hex into bytes. Returns count, or -1 on error.
int parseHexBytes(const char* s, uint8_t* out, uint8_t cap)
{
    size_t n = strlen(s);
    if (n == 0 || (n & 1)) return -1;
    if (n / 2 > cap) return -1;
    for (size_t i = 0; i < n; i += 2) {
        char tmp[3] = { s[i], s[i + 1], '\0' };
        char* e; long v = strtol(tmp, &e, 16);
        if (*e != '\0') return -1;
        out[i / 2] = (uint8_t)v;
    }
    return (int)(n / 2);
}

// Parse decimal in [lo,hi]; -1 on error.
int32_t parseDec(const char* s, int32_t lo, int32_t hi)
{
    if (*s == '\0') return -1;
    char* e; long v = strtol(s, &e, 10);
    if (*e != '\0' || v < lo || v > hi) return -1;
    return v;
}

// Parse a CAN ID in hex (1-8 digits). Sets ext by digit count (>3 = 29-bit,
// INFERRED convention matching FORScan usage). Returns false on error.
bool parseCanId(const char* s, uint32_t& id, bool& ext)
{
    size_t n = strlen(s);
    if (n == 0 || n > 8) return false;
    char* e; unsigned long v = strtoul(s, &e, 16);
    if (*e != '\0' || v > 0x1FFFFFFFUL) return false;
    id = (uint32_t)v;
    ext = (n > 3);
    return true;
}

void stpxExec(uint32_t txId, bool ext, const uint8_t* data, uint8_t len,
              uint16_t tmoMs, uint8_t rCount,
              uint8_t xAddr = 0, bool xGiven = false)
{
    if (!ensureProtocol()) { reply("NO DATA"); return; }
    const PDesc* pd = findProto(activeProto);
    if (pd == nullptr || !pd->isCan) {
        reply("?");  // Phase 4: STPX is CAN-only (documented limitation)
        return;
    }
    isotp::Endpoint ep{ &Can0, txId, 0, ext };
    // Phase 6: STPX x: is now WIRED to the ISO-TP path (was "stored, not
    // wired" in Phase 4) via extended addressing + the AE byte.
    isotp::Config tpCfg;
    const isotp::Config* cfgp = nullptr;
    if (xGiven) {
        tpCfg.addrType = isotp::AddrType::EXTENDED;
        tpCfg.addrExt = xAddr;
        cfgp = &tpCfg;
    }
    isotp::TxResult tr = isotp::transmit(ep, data, len, cfgp);
    // "FC RX TIMEOUT" matches genuine STN wording (measured on real HW).
    if (tr == isotp::TxResult::FC_TIMEOUT) { reply("FC RX TIMEOUT"); return; }
    if (tr != isotp::TxResult::OK) { reply("NO DATA"); return; }
    // RX: raw frames, printed individually (genuine STN behavior — measured).
    // Response filter: STCFCPA pairs registered for this txId win;
    // otherwise accept any ID (documented). Phase 6: the pair match goes
    // through the shared busfilter::idMatch() primitive.
    bool usePairs = false;
    for (uint8_t i = 0; i < nStFcPairs; i++)
        if (stFcPairs[i].tx == txId) { usePairs = true; break; }
    uint16_t tmo = (tmoMs != 0) ? tmoMs : obdTimeoutMs();
    uint8_t got = 0;
    uint32_t start = millis(), lastAct = start;
    for (;;) {
        if (rCount != 0 && got >= rCount) break;
        if ((uint32_t)(millis() - start) >= tmo) break;
        // Quiet exit once something arrived (INFERRED, mirrors isotp).
        if (got > 0 && (uint32_t)(millis() - lastAct) >= 200) break;
        while (Can0.available() > 0) {
            CAN_FRAME f;
            Can0.read(f);
            busstat::noteRx(busstat::Bus::CAN_BUS0, f.length);
            if (((bool)f.extended) != ext) continue;
            if (usePairs) {
                // Phase 6: pair match via the shared idMatch() primitive.
                bool ok = false;
                for (uint8_t i = 0; i < nStFcPairs; i++)
                    if (stFcPairs[i].tx == txId &&
                        busfilter::idMatch(f.id, stFcPairs[i].rx, 0x1FFFFFFFUL)) {
                        ok = true; break;
                    }
                if (!ok) continue;
            }
            // Phase 6: with STPX x:, the AE byte leads the payload — it must
            // match the requested target address (the frame is still printed
            // raw, AE byte included, like genuine STN).
            if (xGiven && (f.length == 0 || f.data.bytes[0] != xAddr)) continue;
            uint8_t hdr[4]; uint8_t hl = 0;
            if (ext) {
                hdr[0] = (uint8_t)(f.id >> 24); hdr[1] = (uint8_t)(f.id >> 16);
                hdr[2] = (uint8_t)(f.id >> 8); hdr[3] = (uint8_t)f.id; hl = 4;
            } else {
                hdr[0] = (uint8_t)(f.id >> 8); hdr[1] = (uint8_t)f.id; hl = 2;
            }
            printDataLine(hdr, hl, f.data.bytes, f.length);
            got++;
            lastAct = millis();
            if (rCount != 0 && got >= rCount) break;
        }
    }
    if (got == 0) reply("NO DATA");
    else prompt();
}

// STPX param1 [, ..., paramN] (FRPM §8.8). Params are id:value pairs:
// h: header/CAN ID (else 0x7DF default, INFERRED), d: data hex,
// l: data length -> DATA> prompt, t: timeout ms (INFERRED units),
// r: expected response count, x: extended address (stored, not wired),
// f: flags (accepted, ignored on CAN).
void stpx(const char* a)
{
    uint32_t txId = 0x7DF; bool ext = false;
    uint8_t data[64]; uint8_t dLen = 0;
    bool dGiven = false, lGiven = false;
    uint16_t lLen = 0, tmoMs = 0;
    uint8_t rCount = 0;
    uint8_t xAddr = 0;      // Phase 6: STPX x: extended (target) address
    bool xGiven = false;
    // Tokenize on ','. Spaces were already stripped by handleLine().
    char work[128];
    strncpy(work, a, sizeof(work) - 1);
    work[sizeof(work) - 1] = '\0';
    for (char* tok = strtok(work, ","); tok != nullptr; tok = strtok(nullptr, ",")) {
        if (tok[1] != ':') { reply("?"); return; }  // id:value form (FRPM)
        char id = tok[0];
        const char* v = tok + 2;
        if (id == 'H' || id == 'h') {
            if (!parseCanId(v, txId, ext)) { reply("?"); return; }
        } else if (id == 'D' || id == 'd') {
            int n = parseHexBytes(v, data, sizeof(data));
            if (n < 0) { reply("?"); return; }
            dLen = (uint8_t)n; dGiven = true;
        } else if (id == 'L' || id == 'l') {
            int32_t n = parseDec(v, 1, 64);
            if (n < 0) { reply("?"); return; }
            lLen = (uint16_t)n; lGiven = true;
        } else if (id == 'T' || id == 't') {
            int32_t n = parseDec(v, 1, 60000);
            if (n < 0) { reply("?"); return; }
            tmoMs = (uint16_t)n;
        } else if (id == 'R' || id == 'r') {
            int32_t n = parseDec(v, 1, 255);
            if (n < 0) { reply("?"); return; }
            rCount = (uint8_t)n;
        } else if (id == 'X' || id == 'x') {
            // Phase 6: extended address (ISO 15765) is now WIRED to the
            // ISO-TP path via extended addressing (was stored-only).
            uint8_t xb[1];
            if (parseHexBytes(v, xb, 1) != 1) { reply("?"); return; }
            xAddr = xb[0]; xGiven = true;
        } else if (id == 'F' || id == 'f') {
            // Flags (auto-checksum etc.): accepted, ignored on CAN.
            uint8_t fb[4];
            if (parseHexBytes(v, fb, sizeof(fb)) < 0) { reply("?"); return; }
        } else { reply("?"); return; }
    }
    if (dGiven && lGiven) { reply("?"); return; }  // FRPM: d and l exclusive
    if (!dGiven && !lGiven) { reply("?"); return; }  // FRPM: d or l required
    if (lGiven) {
        // DATA> flow: the next line is the data (FRPM §8.8).
        Serial.print(F("DATA>"));
        stpxPend.active = true;
        stpxPend.txId = txId; stpxPend.ext = ext;
        stpxPend.dataLen = (uint8_t)lLen;
        stpxPend.tmoMs = tmoMs; stpxPend.rCount = rCount;
        stpxPend.xAddr = xAddr; stpxPend.xGiven = xGiven;
        return;
    }
    stpxExec(txId, ext, data, dLen, tmoMs, rCount, xAddr, xGiven);
}

// STM [n] / STMA [n] / STMR hh / STMT hh (FRPM §8.9; STMR/STMT are
// STN-spelled aliases of the ATMR/ATMT filter behavior).
void stMonitorCmd(const char* a)
{
    uint32_t fId = 0; int32_t n = -1;
    if (*a == 'A') {
        a++;
        if (*a != '\0') {
            n = parseDec(a, 1, 32767);
            if (n < 0) { reply("?"); return; }
        }
        monitor(fId, false, n);
        return;
    }
    if (*a == 'R' || *a == 'T') {
        char* e; long v = strtol(a + 1, &e, 16);
        if (*(a + 1) == '\0' || *e != '\0' || v < 0 || v > 0x1FFFFFFF) {
            reply("?");
            return;
        }
        monitor((uint32_t)v, true, -1);
        return;
    }
    if (*a != '\0') {
        n = parseDec(a, 1, 32767);
        if (n < 0) { reply("?"); return; }
    }
    monitor(0, false, n);
}

void stCommand(const char* c)  // c = text after "ST", uppercased, no spaces
{
    // Device ID (FRPM §8.4).
    if (c[0] == 'I' && c[1] == '\0') { reply("STN1170 v4.0.0"); return; }
    if (!strcmp(c, "DI")) { reply("M2-SuperB r4.0"); return; }
    // Custom transmit (FRPM §8.8).
    if (!strncmp(c, "PX", 2)) { stpx(c + 2); return; }
    // Monitoring (FRPM §8.9).
    if (c[0] == 'M') { stMonitorCmd(c + 1); return; }
    // CAN-specific (FRPM §8.8). Longest prefixes first.
    if (!strncmp(c, "CFCPA", 5)) {
        // STCFCPA txadd,rxadd — flow-control address pair.
        const char* a = c + 5;
        const char* comma = strchr(a, ',');
        if (comma == nullptr || nStFcPairs >= 4) { reply("?"); return; }
        char txs[12], rxs[12];
        size_t tl = (size_t)(comma - a);
        if (tl == 0 || tl >= sizeof(txs) || strlen(comma + 1) >= sizeof(rxs)) {
            reply("?"); return;
        }
        memcpy(txs, a, tl); txs[tl] = '\0';
        strcpy(rxs, comma + 1);
        uint32_t tx, rx; bool ex1, ex2;
        if (!parseCanId(txs, tx, ex1) || !parseCanId(rxs, rx, ex2)) {
            reply("?"); return;
        }
        stFcPairs[nStFcPairs].tx = tx;
        stFcPairs[nStFcPairs].rx = rx;
        nStFcPairs++;
        reply("OK");
        return;
    }
    if (!strcmp(c, "CFCPC")) { nStFcPairs = 0; reply("OK"); return; }
    if (!strncmp(c, "CAF", 3)) {
        // STCAF format [, tt] — addressing format (stored; normal=0 wired).
        const char* a = c + 3;
        if (a[0] < '0' || a[0] > '2') { reply("?"); return; }
        uint8_t fmt = (uint8_t)(a[0] - '0');
        uint8_t tt = 0;
        if (a[1] == ',') {
            uint8_t tb[1];
            if (parseHexBytes(a + 2, tb, 1) != 1) { reply("?"); return; }
            if (fmt == 0) { reply("?"); return; }  // tt needs format 1/2 (FRPM)
            tt = tb[0];
        } else if (a[1] != '\0') { reply("?"); return; }
        stCaf = fmt; stCafTt = tt;
        reply("OK");
        return;
    }
    if (!strncmp(c, "CMM", 3)) {
        // STCMM mode — the genuine STN "silent" control (FRPM §8.8):
        // 0 = receive-only, no CAN ACKs (default); 1 = normal node;
        // 2 = also receive error frames (not supported on SAM3X/due_can).
        if ((c[3] == '0' || c[3] == '1' || c[3] == '2') && c[4] == '\0') {
            stCmm = (uint8_t)(c[3] - '0');
            reply("OK");
        } else reply("?");
        return;
    }
    if (!strncmp(c, "CSEGR", 5) || !strncmp(c, "CSEGT", 5)) {
        bool isRx = (c[3] == 'R');
        if ((c[5] == '0' || c[5] == '1') && c[6] == '\0') {
            if (isRx) stSegRx = (c[5] == '1'); else stSegTx = (c[5] == '1');
            reply("OK");  // stored; STPX segments per FRPM regardless
        } else reply("?");
        return;
    }
    if (!strncmp(c, "CSTM", 4)) {
        // STCSTM ms — STmin offset, decimal ms up to 3 places (stored).
        const char* a = c + 4;
        char* e; double v = strtod(a, &e);
        if (a == e || *e != '\0' || v < 0 || v > 127) { reply("?"); return; }
        stCstmMs = (uint16_t)v;
        reply("OK");
        return;
    }
    if (!strncmp(c, "CTOR", 4)) {
        // STCTOR fcTimeout,cfTimeout — ms pair (stored; FRPM defaults 75/150).
        const char* a = c + 4;
        const char* comma = strchr(a, ',');
        if (comma == nullptr) { reply("?"); return; }
        char s1[12], s2[12];
        size_t l1 = (size_t)(comma - a);
        if (l1 == 0 || l1 >= sizeof(s1) || strlen(comma + 1) >= sizeof(s2)) {
            reply("?"); return;
        }
        memcpy(s1, a, l1); s1[l1] = '\0';
        strcpy(s2, comma + 1);
        int32_t v1 = parseDec(s1, 0, 60000), v2 = parseDec(s2, 0, 60000);
        if (v1 < 0 || v2 < 0) { reply("?"); return; }
        stCtorFc = (uint16_t)v1; stCtorCf = (uint16_t)v2;
        reply("OK");
        return;
    }
    // Honest boundary: these are not FRPM ST commands (STSL*/STCS* are
    // PowerSave/CAN-family prefixes, not standalone commands; STWCF,
    // STDP/STDPN don't exist — use ATDP/ATDPN).
    reply("?");
}

// ---- OBD-II hex request dispatch ----
void obdRequest(const char* hexLine)
{
    // Parse hex pairs into bytes.
    uint8_t req[32]; uint8_t reqLen = 0;
    size_t n = strlen(hexLine);
    if (n == 0 || (n & 1) || n > 64) { reply("?"); return; }
    for (size_t i = 0; i < n && reqLen < sizeof(req); i += 2) {
        char tmp[3] = { hexLine[i], hexLine[i + 1], '\0' };
        char* e; long v = strtol(tmp, &e, 16);
        if (*e != '\0') { reply("?"); return; }
        req[reqLen++] = (uint8_t)v;
    }
    if (!ensureProtocol()) { reply("NO DATA"); return; }
    // Phase 9: capability gate — services 04 (clear) and 08 (control)
    // change ECU state; they need ALLOW_DIAGNOSTIC_WRITE.
    if (reqLen >= 1 && (req[0] == 0x04 || req[0] == 0x08)) {
        if (!capgate::require(capgate::Grant::ALLOW_DIAGNOSTIC_WRITE,
                              "ELM OBD service")) {
            reply("?");
            return;
        }
    }
    const PDesc* pd = findProto(activeProto);
    if (pd->isCan) obdCan(pd, req, reqLen);
    else if (pd->isJ1850) obdJ1850(pd, req, reqLen);
    else obdKline(pd, req, reqLen);  // Phase 7
}

void handleLine(char* line)
{
    if (echoOn) { Serial.print(line); Serial.print('\r'); }
    // Uppercase + strip spaces (genuine ELM tolerates spaces in AT).
    char up[96];
    uint8_t p = 0;
    for (uint8_t i = 0; line[i] != '\0' && p < sizeof(up) - 1; i++) {
        char ch = line[i];
        if (ch == ' ' || ch == '\t') continue;
        if (ch >= 'a' && ch <= 'z') ch = (char)(ch - 'a' + 'A');
        up[p++] = ch;
    }
    up[p] = '\0';
    if (p == 0) { prompt(); return; }
    dbg::print(F("ELM< ")); dbg::println(up);
    // STPX l: leaves a pending DATA> — the next line is raw hex data.
    if (stpxPend.active) {
        uint8_t db[64];
        int n = parseHexBytes(up, db, sizeof(db));
        bool ok = (n >= 0 && (uint8_t)n == stpxPend.dataLen);
        uint32_t txId = stpxPend.txId; bool ext = stpxPend.ext;
        uint16_t tmo = stpxPend.tmoMs; uint8_t rc = stpxPend.rCount;
        uint8_t xa = stpxPend.xAddr; bool xg = stpxPend.xGiven;
        stpxPend.active = false;
        if (!ok) { reply("?"); return; }
        stpxExec(txId, ext, db, (uint8_t)n, tmo, rc, xa, xg);
        return;
    }
    // Phase 4: ST commands are dispatched when STN mode is on. "AT" is
    // checked second so ATST etc. still reach the AT parser.
    if (stnMode && up[0] == 'S' && up[1] == 'T') stCommand(up + 2);
    else if (up[0] == 'A' && up[1] == 'T') atCommand(up + 2);
    else obdRequest(up);
}

// ---- Serial pump with mm-channel divert ----
char lineBuf[96];
uint8_t linePos = 0;

void pumpByte(uint8_t c)
{
    if (mmBinActive) {
        if (mm::feedByte(c)) mmBinActive = false;
        return;
    }
    if (sawF1) {
        sawF1 = false;
        if (c == mm::CMD_MULTIMODE) { mm::reset(); mmBinActive = true; return; }
        if (c == mm::SYNC) { sawF1 = true; return; }  // F1 F1 7E edge: re-arm
        // Stray 0xF1 (not a control frame): drop it, process c normally.
    } else if (c == mm::SYNC) {
        sawF1 = true;
        return;
    }
    if (c == '\r' || c == '\n') {
        if (linePos > 0) {
            lineBuf[linePos] = '\0';
            handleLine(lineBuf);
            linePos = 0;
        }
    } else if (linePos < sizeof(lineBuf) - 1) {
        lineBuf[linePos++] = (char)c;
    } else {
        linePos = 0;  // overflow: reset (INFERRED policy)
    }
}

} // namespace (anonymous)

void begin()
{
    // Power-on defaults (genuine ELM327).
    echoOn = true; linefeed = false; headers = false; spaces = true;
    adaptive = 1; stSetting = 32; allowLong = true; caf = true; cfc = true;
    protoAuto = true; protoNum = 0; activeProto = 0; canActive = false;
    hdr1850[0] = 0x68; hdr1850[1] = 0x6A; hdr1850[2] = 0xF1;
    hdrKline[0] = 0x68; hdrKline[1] = 0x6A; hdrKline[2] = 0xF1;  // Phase 7
    // Phase 9: ELM depth state reset.
    canTxIdOvr = 0; canTxIdSet = false;
    canRxFilter = 0; canRxMask = 0; canRxFilterSet = false;
    kwCheck = true; isoInitAddr = 0x33; isoBaud = 10400;
    wakeupMult = 0; wakeupLen = 0; autoRx = true;
    mmBinActive = false; sawF1 = false; linePos = 0;
    // Phase 4: STN state reset (stnMode itself is set by stn::begin()).
    stCmm = 0; stCaf = 0; stCafTt = 0;
    stSegRx = false; stSegTx = false;
    stCstmMs = 0; stCtorFc = 75; stCtorCf = 150;
    nStFcPairs = 0;
    stpxPend.active = false;
    // NOTE: no bus is touched here (ownership rule: acquire on first use).
    // The ELM "init" happens lazily in ensureProtocol().
}

void loop()
{
    while (Serial.available() > 0) pumpByte((uint8_t)Serial.read());
}

void exit()
{
    busRelease();  // idempotent; manager's quiesceBuses() follows
}

void setJ1850Enabled(bool en) { j1850en = en; }

void setStnMode(bool en) { stnMode = en; }

} // namespace elm
