// M2 multi-mode firmware — UDS/diagnostics mode implementation (Phase 3).
// See mode_uds.h for the contract.
//
// Service/NRC semantics per ISO 14229-1 (public standard); transport per
// ISO 15765-2 via the shared isotp module. Nothing about the standard is
// invented here; anything not testable without a bench is INFERRED.

#include "mode_uds.h"
#include "mode_manager.h"  // modes::canBegin0/1, canEnd0/1
#include "mm_channel.h"    // mm::ST_* codes
#include "isotp.h"
#include "udsreg.h"   // Phase 9: DID/RID registries
#include "capgate.h"  // Phase 9: capability gates
#include "xfer.h"     // Phase 9: shared 0x34/0x36/0x37 download engine
#include "discover.h" // Phase 9: vehicle discovery engine
#include <due_can.h>

namespace uds {

namespace {

// ---- config (staged; survives mode switches) ----
uint8_t busSel = 0;            // 0 = CAN0, 1 = CAN1
uint32_t baud = 500000;        // allow-listed
uint32_t txId = 0x7E0;         // physical request ID
uint32_t rxId = 0x7E8;         // physical response ID
bool funcMode = false;         // true = functional 0x7DF
bool tpEnabled = false;
uint16_t tpIntervalMs = 2000;  // INFERRED default
uint8_t secLevel = 0;          // Phase 9: highest unlocked 0x27 level
                               // (tracked only; 0x27 stays a stub — the
                               // user supplies keys, we track successes)

// Phase 9: shared download engine state (0x34/0x36/0x37 via UDS mode).
xfer::State dl;

// ---- runtime state ----
bool active = false;           // mode entered
bool busUp = false;            // CAN begun for the current config
uint8_t session = 0x01;        // current diagnostic session
uint32_t lastActivity = 0;     // last UDS TX (incl. tester-present)
uint32_t lastTp = 0;

// Arm-confirm for destructive services (0x11 ECUReset, 0x2E WriteDID).
// First invocation arms; the identical command within ARM_MS executes.
constexpr uint32_t ARM_MS = 10000;
bool armed = false;
uint32_t armedAt = 0;
uint8_t armedSvc = 0;
// M1 fix (review): armedData must cover the full service buffer (12B).
// It was 8B while confirmGate could be called with len up to 11 — the
// compare loop read past the end (UB) and confirms could never match.
uint8_t armedData[12];
uint8_t armedLen = 0;

// mm-channel divert state for the Serial pump (same pattern as ELM).
bool mmBinActive = false;
bool sawF1 = false;

constexpr uint32_t SESSION_TIMEOUT_MS = 5000;  // INFERRED (ISO 14229 S3)

bool validBaud(uint32_t b)
{
    return b == 100000 || b == 125000 || b == 250000 || b == 500000 ||
           b == 800000 || b == 1000000;
}

CANRaw* busPtr() { return (busSel == 0) ? &Can0 : &Can1; }

bool busAcquire()
{
    if (busUp) return true;
    if (busSel == 0) modes::canBegin0(baud);
    else modes::canBegin1(baud);
    // Promiscuous RX (ELM pattern): functional 0x7DF gets 0x7E8..0x7EF
    // answers; isotp filters by rxAnyId in software.
    CANRaw* b = busPtr();
    for (int m = 0; m < 7; m++) b->setRXFilter(m, 0, 0, (m < 3));
    busUp = true;
    return true;
}

void busRelease()
{
    if (!busUp) return;
    if (busSel == 0) modes::canEnd0(); else modes::canEnd1();
    busUp = false;
}

isotp::Endpoint endpoint()
{
    isotp::Endpoint ep;
    ep.bus = busPtr();
    ep.extended = false;  // Phase 3: 11-bit only (documented)
    if (funcMode) { ep.txId = 0x7DF; ep.rxAnyId = 0; }
    else { ep.txId = txId; ep.rxAnyId = rxId; }
    return ep;
}

// ---- hex helpers ----
bool hexByte(const char* t, uint8_t& out)
{
    if (t[0] == '\0' || t[2] != '\0') return false;
    char* e; long v = strtol(t, &e, 16);
    if (*e != '\0' || v < 0 || v > 0xFF) return false;
    out = (uint8_t)v;
    return true;
}
bool hexU32(const char* t, uint32_t& out)
{
    if (t[0] == '\0') return false;
    char* e; unsigned long v = strtoul(t, &e, 16);
    if (*e != '\0' || v > 0x1FFFFFFF) return false;
    out = (uint32_t)v;
    return true;
}
void printHex2(uint8_t b)
{
    const char* H = "0123456789ABCDEF";
    SerialUSB.write((uint8_t)H[(b >> 4) & 0xF]);
    SerialUSB.write((uint8_t)H[b & 0xF]);
}
void printBytes(const uint8_t* d, uint8_t n)
{
    for (uint8_t i = 0; i < n; i++) {
        if (i) SerialUSB.write(' ');
        printHex2(d[i]);
    }
}

// ---- core transact: send svc[], collect responses ----
struct TxResult {
    uint8_t nResp = 0;
    isotp::Response r[4];
};

TxResult transact(const uint8_t* svc, uint8_t svcLen, uint16_t timeoutMs = 1000)
{
    TxResult t;
    if (!active || !busAcquire()) return t;
    isotp::Endpoint ep = endpoint();
    t.nResp = isotp::requestMulti(ep, svc, svcLen, t.r, 4, timeoutMs, 200);
    uint32_t now = millis();
    lastActivity = now;
    // Session tracking: positive 0x10 response adopts the requested session.
    // m6 fix (review): a session change invalidates any unlocked security
    // level — secLevel must not go stale across sessions.
    if (svcLen >= 2 && svc[0] == 0x10) {
        for (uint8_t i = 0; i < t.nResp; i++) {
            if (t.r[i].len >= 1 && t.r[i].data[0] == 0x50) {
                if (svc[1] != session) secLevel = 0;
                session = svc[1];
                break;
            }
        }
    }
    return t;
}

void printTx(const uint8_t* svc, uint8_t svcLen)
{
    SerialUSB.print(F("TX "));
    SerialUSB.print(funcMode ? F("7DF") : String(txId, HEX));
    SerialUSB.print(F(": "));
    printBytes(svc, svcLen);
    SerialUSB.println();
}

void printResponses(const TxResult& t, uint8_t svc)
{
    if (t.nResp == 0) {
        SerialUSB.println(F("NO DATA"));
        return;
    }
    for (uint8_t i = 0; i < t.nResp; i++) {
        const isotp::Response& r = t.r[i];
        SerialUSB.print(F("RX "));
        SerialUSB.print(String(r.senderId, HEX));
        SerialUSB.print(F(" ("));
        SerialUSB.print(r.len);
        SerialUSB.print(F("): "));
        printBytes(r.data, r.len);
        if (r.len >= 3 && r.data[0] == 0x7F && r.data[1] == svc) {
            SerialUSB.print(F("  -> NEGATIVE, "));
            SerialUSB.print(nrcText(r.data[2]));
            SerialUSB.print(F(" (NRC 0x"));
            printHex2(r.data[2]);
            SerialUSB.print(F(")"));
        } else if (r.len >= 1 && r.data[0] == (uint8_t)(svc + 0x40)) {
            SerialUSB.print(F("  -> positive"));
        } else {
            SerialUSB.print(F("  -> UNEXPECTED response SID"));
        }
        SerialUSB.println();
    }
}

// Arm-confirm gate for 0x11 / 0x2E. Returns true if the command may run.
bool confirmGate(uint8_t svc, const uint8_t* data, uint8_t len)
{
    // M1 fix (review): clamp len to the armedData capacity so the compare
    // and copy loops can never run past the buffer.
    if (len > sizeof(armedData)) len = sizeof(armedData);
    uint32_t now = millis();
    if (armed && (uint32_t)(now - armedAt) < ARM_MS &&
        armedSvc == svc && armedLen == len) {
        bool same = true;
        for (uint8_t i = 0; i < len; i++)
            if (armedData[i] != data[i]) { same = false; break; }
        if (same) { armed = false; return true; }
    }
    armed = true; armedAt = now; armedSvc = svc; armedLen = len;
    for (uint8_t i = 0; i < len; i++)
        armedData[i] = data[i];
    SerialUSB.println(F("WARNING: this service can change vehicle behavior."));
    if (svc == 0x11) SerialUSB.println(F("ECUReset will reset the ECU."));
    if (svc == 0x2E) SerialUSB.println(F("WriteDataByIdentifier writes to the ECU."));
    SerialUSB.println(F("Repeat the IDENTICAL command within 10 s to execute."));
    return false;
}

// Phase 9: service -> minimum capability grant. This is the enforcement
// point that keeps a 0x2E/0x31/flash command from behaving like a read.
capgate::Grant grantFor(uint8_t svc, const uint8_t* data, uint8_t len)
{
    switch (svc) {
        case 0x11:  // ECUReset
        case 0x14:  // ClearDiagnosticInformation
        case 0x2E:  // WriteDataByIdentifier
        // M2 fix (review): these change ECU state but were falling through
        // to ALLOW_TRANSMIT (write-as-read). 0x28 disables communication,
        // 0x85 disables DTC storage, 0x86 configures unsolicited responses.
        case 0x28:  // CommunicationControl
        case 0x85:  // ControlDTCSetting
        case 0x86:  // ResponseOnEvent
            return capgate::Grant::ALLOW_DIAGNOSTIC_WRITE;
        case 0x2F:  // InputOutputControlByIdentifier
            return capgate::Grant::ALLOW_ACTUATOR_CONTROL;
        // M2 fix (review): 0x02 is stopRoutine (state-changing), 0x03 is
        // requestRoutineResults (a pure read). The old code had them
        // swapped — 0x03 gated as WRITE, 0x02 as TRANSMIT.
        case 0x31:  // RoutineControl
            if (len >= 1 && (data[0] == 0x01 || data[0] == 0x02))
                return capgate::Grant::ALLOW_DIAGNOSTIC_WRITE;
            return capgate::Grant::ALLOW_TRANSMIT;
        case 0x34: case 0x36: case 0x37:  // transfer/download
            return capgate::Grant::ALLOW_FLASH_PROGRAMMING;
        case 0x27:  // SecurityAccess stub: passthrough only
        default:
            return capgate::Grant::ALLOW_TRANSMIT;
    }
}

void sendRaw(const uint8_t* svc, uint8_t svcLen)
{
    if (svcLen == 0) return;
    // 0x14 with no group: default to FFFFFF (all DTCs) — ISO 14229-1.
    uint8_t buf[12]; uint8_t n = svcLen;
    for (uint8_t i = 0; i < svcLen && i < sizeof(buf); i++) buf[i] = svc[i];
    if (svc[0] == 0x14 && svcLen == 1) {
        buf[1] = 0xFF; buf[2] = 0xFF; buf[3] = 0xFF; n = 4;
    }
    if (!active) {
        SerialUSB.println(F("UDS mode is not active. Switch to 'uds' mode first."));
        return;
    }
    // Phase 9: capability gate first (formalized authority).
    if (!capgate::require(grantFor(buf[0], buf + 1, (uint8_t)(n - 1)), "uds service")) return;
    // Arm-confirm stays as defense-in-depth for 0x11/0x2E.
    if ((buf[0] == 0x11 || buf[0] == 0x2E)) {
        if (!confirmGate(buf[0], buf + 1, (uint8_t)(n - 1))) return;
    }
    printTx(buf, n);
    TxResult t = transact(buf, n);
    printResponses(t, buf[0]);
    // Phase 9: registry-assisted output.
    if (t.nResp > 0) {
        const isotp::Response& r0 = t.r[0];
        if (buf[0] == 0x22 && r0.len >= 3 && r0.data[0] == 0x62) {
            // ReadDataByIdentifier positive: decode each DID.
            uint16_t did = (uint16_t)r0.data[1] * 256 + r0.data[2];
            const udsreg::DidEntry* de = udsreg::findDid(did);
            char dec[64];
            if (de != nullptr) {
                SerialUSB.print(F("  DID 0x"));
                SerialUSB.print(String(did, HEX));
                SerialUSB.print(F(" ")); SerialUSB.print(de->name);
                if (de->flags & udsreg::DID_SEC)
                    SerialUSB.print(F(" [may need 0x27]"));
                if (udsreg::decodeDid(did, r0.data + 3, (uint8_t)(r0.len - 3),
                                      dec, sizeof(dec))) {
                    SerialUSB.print(F(" = ")); SerialUSB.print(dec);
                }
                SerialUSB.println();
            }
        } else if (buf[0] == 0x31 && r0.len >= 4 && r0.data[0] == 0x71) {
            uint16_t rid = (uint16_t)r0.data[2] * 256 + r0.data[3];
            const udsreg::RidEntry* re = udsreg::findRid(rid);
            if (re != nullptr) {
                SerialUSB.print(F("  routine 0x"));
                SerialUSB.print(String(rid, HEX));
                SerialUSB.print(F(" ")); SerialUSB.println(re->name);
            }
        } else if (buf[0] == 0x27 && r0.len >= 2 && r0.data[0] == 0x67) {
            // Phase 9: track the unlocked security level (stub passthrough;
            // the user supplies the key, we only record success).
            // sendKey level = requestSeed level + 1.
            secLevel = r0.data[1];
            SerialUSB.print(F("  security level now 0x"));
            SerialUSB.println(String(secLevel, HEX));
            SerialUSB.println(F("  (tracked only — 0x27 remains a stub)"));
        }
    }
}

void printHelp()
{
    SerialUSB.println(F("UDS console (config works any time; bus commands need 'uds' mode):"));
    SerialUSB.println(F("  uds bus 0|1 | uds baud <n> | uds tx <hex> | uds rx <hex> | uds func|phys"));
    SerialUSB.println(F("  uds tp on|off|<ms> | uds session | uds sec | uds discover"));
    SerialUSB.println(F("  uds <svcHex> [dataHex...]   e.g. uds 1003 | uds 22 F190 | uds 19 02 FF"));
    SerialUSB.println(F("  uds didinfo <didHex> | uds ridinfo <ridHex>   (registry lookup)"));
    SerialUSB.println(F("  uds dl <addrHex> <lenHex> [alfidHex]  (0x34, gated+confirm)"));
    SerialUSB.println(F("  uds ul <hex...> [last]  (0x36 blocks; `last` ends the stream)"));
    SerialUSB.println(F("  uds xferdone <fileCrc32Hex>  (CRC-verified 0x37 finalize)"));
    SerialUSB.println(F("  0x11/0x2E need repeat-to-confirm. 0x27 is a stub (no key algorithm)."));
    SerialUSB.println(F("  Writes need `cap grant` escalation (see docs/SAFETY_REVIEW.md)."));
}

// ---- Phase 9: shared download engine (0x34/0x36/0x37) via xfer ----
bool udsTransferData(void* ctx, uint8_t blockSeq,
                     const uint8_t* data, uint8_t len,
                     uint8_t* resp, uint8_t respCap, uint8_t& respLen)
{
    (void)ctx;
    uint8_t req[2 + xfer::CHUNK_MAX];
    req[0] = 0x36; req[1] = blockSeq;
    for (uint8_t i = 0; i < len; i++) req[2 + i] = data[i];
    TxResult t = transact(req, (uint8_t)(2 + len), 5000);
    respLen = 0;
    if (t.nResp == 0) return false;
    uint8_t n = (t.r[0].len <= respCap) ? (uint8_t)t.r[0].len : respCap;
    for (uint8_t i = 0; i < n; i++) resp[i] = t.r[0].data[i];
    respLen = n;
    return true;
}
bool udsTransferExit(void* ctx)
{
    (void)ctx;
    const uint8_t req[1] = { 0x37 };
    TxResult t = transact(req, 1, 1000);
    return t.nResp > 0 && t.r[0].len >= 1 && t.r[0].data[0] == 0x77;
}
void udsNote(void* ctx, const char* msg)
{
    (void)ctx;
    SerialUSB.print(F("[uds xfer] "));
    SerialUSB.println(msg);
}
xfer::Io udsXferIo()
{
    xfer::Io io;
    io.ctx = nullptr;
    io.transferData = udsTransferData;
    io.transferExit = udsTransferExit;
    io.note = udsNote;
    return io;
}

void udsDownload(uint32_t addr, uint32_t len, uint8_t alfid)
{
    // 0x34 RequestDownload: [0x34, dfi, alfid, addr..., size...]
    uint8_t aLen = (uint8_t)(alfid & 0x0F);
    uint8_t sLen = (uint8_t)((alfid >> 4) & 0x0F);
    if (aLen == 0 || aLen > 4 || sLen == 0 || sLen > 4) {
        SerialUSB.println(F("Bad alfid (nibbles must be 1..4)."));
        return;
    }
    uint8_t req[3 + 8];
    req[0] = 0x34; req[1] = 0x00; req[2] = alfid;
    for (uint8_t i = 0; i < aLen; i++)
        req[3 + i] = (uint8_t)((addr >> (8 * (aLen - 1 - i))) & 0xFF);
    for (uint8_t i = 0; i < sLen; i++)
        req[3 + aLen + i] = (uint8_t)((len >> (8 * (sLen - 1 - i))) & 0xFF);
    uint8_t reqLen = (uint8_t)(3 + aLen + sLen);
    if (!capgate::require(capgate::Grant::ALLOW_FLASH_PROGRAMMING, "uds 0x34 download"))
        return;
    // M1 fix (review): pass the request payload (minus the service byte)
    // directly — confirmGate clamps to its 12B capacity. The old cgData[9]
    // staging array could not hold the full payload (up to 10B).
    if (!confirmGate(0x34, req + 1, (uint8_t)(reqLen - 1))) return;
    printTx(req, reqLen);
    TxResult t = transact(req, reqLen);
    printResponses(t, 0x34);
    if (t.nResp == 0 || t.r[0].len < 1 || t.r[0].data[0] != 0x74) return;
    uint32_t maxBlock = 0;
    if (t.r[0].len >= 2) {
        uint8_t mbLen = (uint8_t)((t.r[0].data[1] >> 4) & 0x0F);
        for (uint8_t i = 0; i < mbLen && 2 + i < t.r[0].len; i++)
            maxBlock = (maxBlock << 8) | t.r[0].data[2 + i];
    }
    xfer::open(dl, maxBlock, millis());
    SerialUSB.print(F("Download open. ECU max block: "));
    SerialUSB.println(maxBlock ? String(maxBlock) : F("(unknown)"));
    SerialUSB.println(F("Send blocks with `uds ul <hex...> [last]`;"));
    SerialUSB.println(F("finish with `uds xferdone <fileCrc32Hex>` (CRC enforced in code)."));
}

// M3 fix (review): flags carries the end-of-stream marker (console `ul`
// ... last). xferdone now requires the file CRC32 and goes through
// xfer::finish(), which enforces the CRC32 compare + streamDone + 0x37
// IN CODE — not as a printed warning.
void udsUpload(const uint8_t* data, uint8_t len, uint8_t flags)
{
    if (!capgate::require(capgate::Grant::ALLOW_FLASH_PROGRAMMING, "uds 0x36 upload"))
        return;
    if (!dl.open || dl.streamDone) {
        SerialUSB.println(F("No open download (run `uds dl` first)."));
        return;
    }
    xfer::Io io = udsXferIo();
    uint16_t ack = 0;
    uint8_t rc = xfer::feedChunk(dl, io, dl.expectedSeq, flags,
                                 data, len, xfer::crc16(data, len), ack);
    if (rc != 0) SerialUSB.println(F("Block rejected (see above)."));
    else {
        SerialUSB.print(F("Block ")); SerialUSB.print(ack);
        SerialUSB.println(F(" accepted."));
    }
}

void printSession()
{
    SerialUSB.print(F("Session: 0x")); printHex2(session);
    SerialUSB.print(F("  bus: CAN")); SerialUSB.print(busSel);
    SerialUSB.print(F(" @ ")); SerialUSB.print(baud);
    SerialUSB.print(F("  addressing: "));
    if (funcMode) SerialUSB.print(F("functional 0x7DF"));
    else {
        SerialUSB.print(F("physical TX 0x")); SerialUSB.print(String(txId, HEX));
        SerialUSB.print(F(" RX 0x")); SerialUSB.print(String(rxId, HEX));
    }
    SerialUSB.print(F("  tester-present: "));
    SerialUSB.print(tpEnabled ? F("on") : F("off"));
    if (tpEnabled) { SerialUSB.print(F(" every ")); SerialUSB.print(tpIntervalMs); SerialUSB.print(F(" ms")); }
    SerialUSB.print(F("  bus: ")); SerialUSB.println(busUp ? F("up") : F("down (lazy)"));
}

} // namespace (anonymous)

const char* nrcText(uint8_t nrc)
{
    // ISO 14229-1 negative response codes (public standard values).
    switch (nrc) {
        case 0x10: return "generalReject";
        case 0x11: return "serviceNotSupported";
        case 0x12: return "subFunctionNotSupported";
        case 0x13: return "incorrectMessageLengthOrInvalidFormat";
        case 0x21: return "busyRepeatRequest";
        case 0x22: return "conditionsNotCorrect";
        case 0x24: return "requestSequenceError";
        case 0x25: return "noResponseFromSubnetComponent";
        case 0x26: return "failurePreventsExecutionOfRequestedAction";
        case 0x31: return "requestOutOfRange";
        case 0x33: return "securityAccessDenied";
        case 0x35: return "invalidKey";
        case 0x36: return "exceedNumberOfAttempts";
        case 0x37: return "requiredTimeDelayNotExpired";
        case 0x70: return "uploadDownloadNotAccepted";
        case 0x71: return "transferDataSuspended";
        case 0x72: return "generalProgrammingFailure";
        case 0x73: return "wrongBlockSequenceCounter";
        case 0x78: return "requestCorrectlyReceived_ResponsePending";
        case 0x7E: return "subFunctionNotSupportedInActiveSession";
        case 0x7F: return "serviceNotSupportedInActiveSession";
        default:   return "unknown NRC";
    }
}

void begin()
{
    active = true;
    busUp = false;      // lazy: first command acquires (ownership rule)
    session = 0x01;
    secLevel = 0;       // Phase 9: security state never survives mode entry
    xfer::reset(dl);    // Phase 9: no transfer survives mode entry
    tpEnabled = false;
    armed = false;
    mmBinActive = false; sawF1 = false;
    lastActivity = millis();
    SerialUSB.println(F("UDS mode: CAN ISO-TP only (11-bit). Bus acquired on first command."));
}

void loop()
{
    // mm-channel divert on the Bee link (same pattern as ELM).
    while (Serial.available() > 0) {
        uint8_t c = (uint8_t)Serial.read();
        if (mmBinActive) {
            if (mm::feedByte(c)) mmBinActive = false;
            continue;
        }
        if (sawF1) {
            sawF1 = false;
            if (c == mm::CMD_MULTIMODE) { mm::reset(); mmBinActive = true; continue; }
            if (c == mm::SYNC) { sawF1 = true; continue; }
        } else if (c == mm::SYNC) {
            sawF1 = true;
            continue;
        }
        // Non-control bytes in UDS mode are ignored (no AT stream here).
    }
    if (!active) return;
    uint32_t now = millis();
    // Tester-present keepalive.
    if (tpEnabled && busUp && (uint32_t)(now - lastTp) >= tpIntervalMs) {
        const uint8_t tp[2] = { 0x3E, 0x00 };
        transact(tp, 2, 500);
        lastTp = now;
    }
    // Session timeout without tester-present traffic (ISO 14229 S3_server).
    if (session != 0x01 && (uint32_t)(now - lastActivity) >= SESSION_TIMEOUT_MS) {
        session = 0x01;
        secLevel = 0;  // Phase 9: security state dies with the session
        SerialUSB.println(F("[uds] session timed out -> default (0x01)"));
        lastActivity = now;
    }
    // Phase 9: shared download-engine watchdog (stalled 0x36 stream).
    if (dl.open) {
        xfer::Io io = udsXferIo();
        if (xfer::watchdog(dl, io, now))
            SerialUSB.println(F("[uds] download watchdog fired (see above)."));
    }
}

void exit()
{
    if (dl.open) {
        xfer::Io io = udsXferIo();
        xfer::abort(dl, io, "mode exit during transfer");
    }
    busRelease();   // idempotent; manager's quiesceBuses() follows
    active = false;
    armed = false;
    secLevel = 0;
}

void console(char* line)
{
    // line: text after "uds" (may be empty/"uds").
    while (*line == ' ') line++;
    if (*line == '\0') { printHelp(); return; }

    char* save = nullptr;
    char* tok = strtok_r(line, " \t", &save);
    if (!tok) { printHelp(); return; }

    if (strcasecmp(tok, "bus") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        if (a && (a[0] == '0' || a[0] == '1') && a[1] == '\0') {
            uint8_t nb = (uint8_t)(a[0] - '0');
            if (nb != busSel) { busSel = nb; busRelease(); }  // re-acquire lazily
            SerialUSB.print(F("UDS bus: CAN")); SerialUSB.println(busSel);
        } else SerialUSB.println(F("usage: uds bus 0|1"));
        return;
    }
    if (strcasecmp(tok, "baud") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        uint32_t b = a ? (uint32_t)strtoul(a, nullptr, 10) : 0;
        if (validBaud(b)) {
            if (b != baud) { baud = b; busRelease(); }
            SerialUSB.print(F("UDS baud: ")); SerialUSB.println(baud);
        } else SerialUSB.println(F("usage: uds baud <100000|125000|250000|500000|800000|1000000>"));
        return;
    }
    if (strcasecmp(tok, "tx") == 0 || strcasecmp(tok, "rx") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        uint32_t v = 0;
        if (a && hexU32(a, v) && v <= 0x7FF) {
            if (tok[0] == 't' || tok[0] == 'T') txId = v; else rxId = v;
            SerialUSB.print(tok); SerialUSB.print(F(" ID: 0x"));
            SerialUSB.println(String(v, HEX));
        } else SerialUSB.println(F("usage: uds tx|rx <hex, 11-bit>"));
        return;
    }
    if (strcasecmp(tok, "func") == 0) {
        funcMode = true; SerialUSB.println(F("Functional addressing (0x7DF)."));
        return;
    }
    if (strcasecmp(tok, "phys") == 0) {
        funcMode = false; SerialUSB.println(F("Physical addressing."));
        return;
    }
    if (strcasecmp(tok, "tp") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        if (a && strcasecmp(a, "off") == 0) {
            tpEnabled = false; SerialUSB.println(F("Tester-present off."));
        } else if (a && strcasecmp(a, "on") == 0) {
            tpEnabled = true; lastTp = 0; SerialUSB.println(F("Tester-present on."));
        } else if (a) {
            uint32_t ms = (uint32_t)strtoul(a, nullptr, 10);
            if (ms >= 100 && ms <= 10000) {
                tpIntervalMs = (uint16_t)ms; tpEnabled = true; lastTp = 0;
                SerialUSB.print(F("Tester-present every "));
                SerialUSB.print(ms); SerialUSB.println(F(" ms."));
            } else SerialUSB.println(F("usage: uds tp on|off|<100..10000 ms>"));
        } else SerialUSB.println(F("usage: uds tp on|off|<ms>"));
        return;
    }
    if (strcasecmp(tok, "session") == 0) { printSession(); return; }
    if (strcasecmp(tok, "sec") == 0) {
        SerialUSB.print(F("Security level: 0x"));
        SerialUSB.println(String(secLevel, HEX));
        SerialUSB.println(F("(tracked only — 0x27 remains a stub, no key algorithm)"));
        return;
    }
    if (strcasecmp(tok, "didinfo") == 0 || strcasecmp(tok, "ridinfo") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        uint32_t v = 0;
        bool isDid = (tok[0] == 'd' || tok[0] == 'D');
        if (!a || !hexU32(a, v) || v > 0xFFFF) {
            SerialUSB.println(isDid ? F("usage: uds didinfo <didHex>") : F("usage: uds ridinfo <ridHex>"));
            return;
        }
        if (isDid) {
            const udsreg::DidEntry* de = udsreg::findDid((uint16_t)v);
            if (de == nullptr) { SerialUSB.println(F("Unknown DID.")); return; }
            SerialUSB.print(de->name);
            if (de->flags & udsreg::DID_SEC) SerialUSB.print(F(" [may need 0x27]"));
            SerialUSB.println();
        } else {
            const udsreg::RidEntry* re = udsreg::findRid((uint16_t)v);
            if (re == nullptr) { SerialUSB.println(F("Unknown RID.")); return; }
            SerialUSB.println(re->name);
        }
        return;
    }
    if (strcasecmp(tok, "dl") == 0) {
        char* aA = strtok_r(nullptr, " \t", &save);
        char* aL = strtok_r(nullptr, " \t", &save);
        char* aF = strtok_r(nullptr, " \t", &save);
        uint32_t addr = 0, len = 0; uint8_t alfid = 0x44, f = 0;
        if (!active) { SerialUSB.println(F("UDS mode is not active.")); return; }
        if (!aA || !hexU32(aA, addr) || !aL || !hexU32(aL, len) || len == 0 ||
            (aF && !hexByte(aF, f))) {
            SerialUSB.println(F("usage: uds dl <addrHex> <lenHex> [alfidHex]"));
            return;
        }
        if (aF) alfid = f;
        udsDownload(addr, len, alfid);
        return;
    }
    if (strcasecmp(tok, "ul") == 0) {
        // M3 fix (review): `last` marks the final block (end-of-stream).
        if (!active) { SerialUSB.println(F("UDS mode is not active.")); return; }
        uint8_t data[xfer::CHUNK_MAX]; uint8_t n = 0; uint8_t b = 0;
        uint8_t flags = 0;
        for (char* a = strtok_r(nullptr, " \t", &save);
             a && n < sizeof(data);
             a = strtok_r(nullptr, " \t", &save)) {
            if (strcasecmp(a, "last") == 0) { flags |= 0x01; continue; }
            if (!hexByte(a, b)) { SerialUSB.println(F("Bad hex byte.")); return; }
            data[n++] = b;
        }
        if (n == 0) { SerialUSB.println(F("usage: uds ul <hex...> [last]")); return; }
        udsUpload(data, n, flags);
        return;
    }
    if (strcasecmp(tok, "xferdone") == 0) {
        // M3 fix (review): the file CRC32 is REQUIRED and enforced in
        // code via xfer::finish() (CRC compare + streamDone + 0x37).
        // A printed warning is documentation, not an interlock.
        if (!active) { SerialUSB.println(F("UDS mode is not active.")); return; }
        if (!capgate::require(capgate::Grant::ALLOW_FLASH_PROGRAMMING, "uds xferdone"))
            return;
        char* aC = strtok_r(nullptr, " \t", &save);
        uint32_t fileCrc = 0;
        if (!aC || !hexU32(aC, fileCrc)) {
            SerialUSB.println(F("usage: uds xferdone <fileCrc32Hex>"));
            return;
        }
        xfer::Io io = udsXferIo();
        if (xfer::finish(dl, io, fileCrc))
            SerialUSB.println(F("Transfer exited (0x37 positive, CRC verified)."));
        else
            SerialUSB.println(F("Finalize FAILED (see above) — transfer aborted."));
        return;
    }
    if (strcasecmp(tok, "discover") == 0) {
        if (!active) { SerialUSB.println(F("UDS mode is not active.")); return; }
        busRelease();
        discover::Result r;
        SerialUSB.println(F("Discovering (CAN)..."));
        if (discover::run(busSel, r)) discover::print(r);
        else SerialUSB.println(F("No ECU answered."));
        return;
    }

    // Otherwise: raw service request. First token = service byte.
    uint8_t svc[12]; uint8_t svcLen = 0;
    uint8_t b = 0;
    if (!hexByte(tok, b)) { printHelp(); return; }
    svc[svcLen++] = b;
    for (char* a = strtok_r(nullptr, " \t", &save);
         a && svcLen < sizeof(svc);
         a = strtok_r(nullptr, " \t", &save)) {
        if (!hexByte(a, b)) {
            SerialUSB.println(F("Bad hex byte."));
            return;
        }
        svc[svcLen++] = b;
    }
    if (svc[0] == 0x27) {
        SerialUSB.println(F("NOTE: 0x27 SecurityAccess is a stub — the seed is"));
        SerialUSB.println(F("passed through and printed, but NO key algorithm is"));
        SerialUSB.println(F("implemented (OEM algorithms are proprietary)."));
    }
    sendRaw(svc, svcLen);
}

uint8_t mmTx(const uint8_t* svc, uint8_t svcLen,
             uint8_t* resp, uint8_t* respLen, uint8_t* nResp)
{
    *nResp = 0; *respLen = 0;
    if (!active) return mm::ST_ERR;   // UDS_TX is only valid in UDS mode
    if (svcLen == 0 || svcLen > 12) return mm::ST_BAD_ARG;
    // Phase 9: the capability gate applies to the mm path too — a 0x2E
    // from the web GUI must not behave like a read.
    if (!capgate::require(grantFor(svc[0], svc + 1, (uint8_t)(svcLen - 1)),
                          "uds mm service"))
        return mm::ST_ERR;
    TxResult t = transact(svc, svcLen);
    *nResp = t.nResp;
    if (t.nResp > 0) {
        uint8_t n = t.r[0].len;
        for (uint8_t i = 0; i < n; i++) resp[i] = t.r[0].data[i];
        *respLen = n;
    }
    return mm::ST_OK;
}

uint8_t mmCfg(uint8_t bus, uint32_t baudIn, uint32_t txIdIn, uint32_t rxIdIn,
              uint8_t func, uint16_t tpMs)
{
    if (bus > 1 || !validBaud(baudIn) || txIdIn > 0x7FF || rxIdIn > 0x7FF)
        return mm::ST_BAD_ARG;
    bool busChanged = (bus != busSel) || (baudIn != baud);
    busSel = bus; baud = baudIn; txId = txIdIn; rxId = rxIdIn;
    funcMode = (func != 0);
    if (tpMs == 0) tpEnabled = false;
    else { tpEnabled = true; tpIntervalMs = tpMs; lastTp = 0; }
    if (busChanged) busRelease();  // re-acquire lazily with new settings
    return mm::ST_OK;
}

void mmStatus(uint8_t& sSession, uint8_t& sBus, uint32_t& sBaud,
              uint32_t& sTxId, uint32_t& sRxId, uint8_t& sFunc, uint8_t& sTpOn)
{
    sSession = session; sBus = busSel; sBaud = baud;
    sTxId = txId; sRxId = rxId;
    sFunc = funcMode ? 1 : 0; sTpOn = tpEnabled ? 1 : 0;
}

// Phase 9: mm-channel discovery (releases our bus first, like the console).
uint8_t mmDiscover(uint8_t bus)
{
    if (bus > 1) return mm::ST_BAD_ARG;
    if (!active) return mm::ST_ERR;  // only valid in UDS mode
    if (dl.open) {
        xfer::Io io = udsXferIo();
        xfer::abort(dl, io, "discovery requested during transfer");
    }
    busRelease();
    discover::Result r;
    busSel = bus;
    return discover::run(bus, r) ? mm::ST_OK : mm::ST_ERR;
}

} // namespace uds
