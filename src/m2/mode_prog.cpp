// M2 multi-mode firmware — PROG (programming) mode implementation (Phase 5).
// See mode_prog.h for the contract and docs/SAFETY_REVIEW.md for the safety
// case this file implements. Service semantics per ISO 14229-1 (public
// standard — cited, never invented); transport per ISO 15765-2 via isotp.h.
// Anything not testable without a bench is INFERRED.

#include "mode_prog.h"
#include "mode_manager.h"  // modes::canBegin0/1, canEnd0/1
#include "mode_mitm.h"     // mitm::suspendRules() (called by the manager)
#include "mode_uds.h"      // uds::nrcText() (shared NRC table)
#include "mm_channel.h"    // mm::ST_* codes, mm::Divert/divertByte
#include "isotp.h"
#include "xfer.h"          // Phase 9: shared 0x34/0x36/0x37 download engine
#include <due_can.h>

namespace prog {

namespace {

// ---- config (staged; survives mode switches) ----
uint8_t busSel = 0;
uint32_t baud = 500000;
uint32_t txId = 0x7E0;
uint32_t rxId = 0x7E8;
bool configured = false;   // set ONLY by explicit bus/baud/tx/rx (interlock c)

// ---- interlock state ----
bool armed = false;
uint32_t armedAt = 0;
bool powerOk = false;      // user attestation: stable 12V+ (NOT a measurement)
constexpr uint32_t ARM_MS = 120000;  // INFERRED: long enough for a flash seq

// ---- runtime ----
bool active = false;
bool busUp = false;
uint8_t session = 0x01;
uint8_t state = ST_IDLE;
uint8_t seedLevel = 0;     // last 0x27 requestSeed level (for sendKey level+1)

// Phase 9: download streaming state lives in the shared xfer module now
// (no image buffering — one chunk at a time). PROG's interlocks
// (armed, powerOk, configured) stay here, untouched.
xfer::State dl;
bool tpOn = false;         // tester-present auto-enabled in session
uint32_t lastTp = 0;
uint8_t mitmSuspended = 0;

constexpr uint32_t TP_MS = 2000;        // INFERRED tester-present interval
constexpr uint8_t PROG_CHUNK_MAX = xfer::CHUNK_MAX;  // fits the 48-byte mm frame cap

mm::Divert divert;

// Phase 9: CRC helpers moved to xfer (crc16, crc32feed, crcInit).

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
    if (*e != '\0' || v > 0xFFFFFFFFUL) return false;
    out = (uint32_t)v;
    return true;
}
void printHex2(uint8_t b)
{
    const char* H = "0123456789ABCDEF";
    SerialUSB.write((uint8_t)H[(b >> 4) & 0xF]);
    SerialUSB.write((uint8_t)H[b & 0xF]);
}
void printHexBytes(const uint8_t* d, uint8_t n)
{
    for (uint8_t i = 0; i < n; i++) {
        if (i) SerialUSB.write(' ');
        printHex2(d[i]);
    }
}
void printHex32(uint32_t v)
{
    printHex2((uint8_t)((v >> 24) & 0xFF));
    printHex2((uint8_t)((v >> 16) & 0xFF));
    printHex2((uint8_t)((v >> 8) & 0xFF));
    printHex2((uint8_t)(v & 0xFF));
}
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
    ep.extended = false;  // Phase 5: 11-bit only (same scope as UDS Phase 3)
    ep.txId = txId;
    ep.rxAnyId = rxId;
    return ep;
}

const char* stateName(uint8_t s)
{
    switch (s) {
        case ST_IDLE: return "IDLE";
        case ST_SESSION: return "SESSION";
        case ST_UNLOCKED: return "UNLOCKED";
        case ST_ERASED: return "ERASED";
        case ST_DOWNLOADING: return "DOWNLOADING";
        case ST_STREAM_DONE: return "STREAM_DONE";
        case ST_DONE: return "DONE";
        case ST_ABORTED: return "ABORTED";
        case ST_ERROR: return "ERROR";
        default: return "?";
    }
}

// ---- interlock gates ----
bool needActive()
{
    if (!active) {
        SerialUSB.println(F("PROG mode is not active. Switch to 'prog' mode first."));
        return false;
    }
    return true;
}
bool needArmed()
{
    if (!armed || (uint32_t)(millis() - armedAt) >= ARM_MS) {
        armed = false;
        SerialUSB.println(F("Not armed. Run `prog arm` first (arming lasts 120 s)."));
        return false;
    }
    return true;
}
bool needPower()
{
    if (!powerOk) {
        SerialUSB.println(F("No power attestation. Run `prog power ok` to confirm"));
        SerialUSB.println(F("a stable 12V+ supply (see docs/SAFETY_REVIEW.md §4)."));
        return false;
    }
    return true;
}
bool needConfigured()
{
    if (!configured) {
        SerialUSB.println(F("Addressing not configured. Set bus/tx/rx explicitly first."));
        return false;
    }
    return true;
}

// Send one service, get one response. Returns true on positive response.
// respLen = response length; negative responses are reported, not hidden.
bool txService(const uint8_t* svc, uint8_t svcLen,
               uint8_t* resp, uint8_t respCap, uint8_t& respLen,
               uint16_t timeoutMs = 1000)
{
    respLen = 0;
    if (!busAcquire()) return false;
    isotp::Endpoint ep = endpoint();
    if (!isotp::request(ep, svc, svcLen, resp, respCap, respLen, timeoutMs)) {
        SerialUSB.println(F("NO DATA (timeout)"));
        return false;
    }
    if (respLen >= 3 && resp[0] == 0x7F) {
        SerialUSB.print(F("NEGATIVE: "));
        SerialUSB.print(uds::nrcText(resp[2]));
        SerialUSB.print(F(" (NRC 0x"));
        printHex2(resp[2]);
        SerialUSB.println(')');
        return false;
    }
    if (respLen >= 1 && resp[0] == (uint8_t)(svc[0] + 0x40)) return true;
    SerialUSB.print(F("UNEXPECTED response SID 0x"));
    printHex2(resp[0]);
    SerialUSB.println();
    return false;
}

// Phase 9: xfer Io callbacks (PROG transport).
bool progTransferData(void* ctx, uint8_t blockSeq,
                     const uint8_t* data, uint8_t len,
                     uint8_t* resp, uint8_t respCap, uint8_t& respLen)
{
    (void)ctx;
    // ISO 14229-1 TransferData: [0x36, blockSequenceCounter, data...].
    uint8_t req[2 + xfer::CHUNK_MAX];
    req[0] = 0x36; req[1] = blockSeq;
    for (uint8_t i = 0; i < len; i++) req[2 + i] = data[i];
    uint8_t r[16]; uint8_t rl = 0;
    // ECU flash-write per block can be slow — long timeout (INFERRED).
    if (!txService(req, (uint8_t)(2 + len), r, sizeof(r), rl, 5000))
        return false;
    uint8_t n = (rl <= respCap) ? rl : respCap;
    for (uint8_t i = 0; i < n; i++) resp[i] = r[i];
    respLen = n;
    return true;
}
bool progTransferExit(void* ctx)
{
    (void)ctx;
    const uint8_t t37[1] = { 0x37 };
    uint8_t r[16]; uint8_t rl = 0;
    if (!busUp) return false;
    isotp::Endpoint ep = endpoint();
    if (!isotp::request(ep, t37, 1, r, sizeof(r), rl, 500)) return false;
    return rl >= 1 && r[0] == 0x77;
}
void progNote(void* ctx, const char* msg)
{
    (void)ctx;
    SerialUSB.print(F("[prog] "));
    SerialUSB.println(msg);
}
xfer::Io progXferIo()
{
    xfer::Io io;
    io.ctx = nullptr;
    io.transferData = progTransferData;
    io.transferExit = progTransferExit;
    io.note = progNote;
    return io;
}

void abortTransfer(const char* reason, bool userAbort)
{
    // Phase 9: the shared engine does the best-effort 0x37 + state clear;
    // PROG clears its own arm/session state here (interlocks intact).
    if (state == ST_DOWNLOADING || state == ST_STREAM_DONE) {
        xfer::Io io = progXferIo();
        xfer::abort(dl, io, reason);
    }
    state = userAbort ? ST_ABORTED : ST_ERROR;
    armed = false; tpOn = false;
    SerialUSB.println(F("Transfer state cleared and disarmed. See docs/SAFETY_REVIEW.md §5."));
}

// ---- service flows ----

bool doSession()
{
    const uint8_t req[2] = { 0x10, 0x02 };  // programmingSession
    uint8_t r[16]; uint8_t rl = 0;
    SerialUSB.println(F("TX: 10 02 (DiagnosticSessionControl/programmingSession)"));
    if (!txService(req, 2, r, sizeof(r), rl)) return false;
    session = 0x02;
    state = ST_SESSION;
    tpOn = true; lastTp = 0;  // keep the session alive during the sequence
    SerialUSB.println(F("Positive: programming session active."));
    return true;
}

bool doSeed(uint8_t level, bool fromConsole)
{
    const uint8_t req[2] = { 0x27, level };
    uint8_t r[128]; uint8_t rl = 0;
    if (fromConsole) { SerialUSB.print(F("TX: 27 ")); printHex2(level); SerialUSB.println(); }
    if (!txService(req, 2, r, sizeof(r), rl)) return false;
    // Positive: [0x67, level, seed...] (ISO 14229-1). The seed is shown;
    // NO key algorithm exists here — the user supplies the key.
    seedLevel = level;
    SerialUSB.print(F("Seed (level 0x"));
    printHex2(level);
    SerialUSB.print(F("): "));
    if (rl > 2) printHexBytes(r + 2, (uint8_t)(rl - 2));
    else SerialUSB.print(F("(none — zero-length seed)"));
    SerialUSB.println();
    SerialUSB.println(F("Compute the key with your own tooling and send it via `prog key`."));
    return true;
}

bool doKey(const uint8_t* key, uint8_t keyLen)
{
    uint8_t req[2 + 64];
    if (keyLen == 0 || keyLen > 64) {
        SerialUSB.println(F("Empty or oversized key."));
        return false;
    }
    req[0] = 0x27; req[1] = (uint8_t)(seedLevel + 1);
    for (uint8_t i = 0; i < keyLen; i++) req[2 + i] = key[i];
    uint8_t r[16]; uint8_t rl = 0;
    SerialUSB.println(F("TX: 27 <key> (sendKey — user-supplied, never computed here)"));
    if (!txService(req, (uint8_t)(2 + keyLen), r, sizeof(r), rl)) return false;
    state = ST_UNLOCKED;
    SerialUSB.println(F("Positive: security unlocked."));
    return true;
}

bool doErase(uint16_t routineId)
{
    // ISO 14229-1 RoutineControl: [0x31, 0x01=startRoutine, routineId BE16].
    const uint8_t req[4] = { 0x31, 0x01,
        (uint8_t)(routineId >> 8), (uint8_t)(routineId & 0xFF) };
    uint8_t r[16]; uint8_t rl = 0;
    SerialUSB.print(F("TX: 31 01 "));
    printHex2(req[2]); SerialUSB.write(' '); printHex2(req[3]);
    SerialUSB.println(F(" (startRoutine)"));
    // Erase can take seconds — long timeout (INFERRED).
    if (!txService(req, 4, r, sizeof(r), rl, 30000)) return false;
    state = ST_ERASED;
    SerialUSB.println(F("Positive: erase routine completed."));
    return true;
}

bool doDownload(uint32_t addr, uint32_t len, uint8_t alfid)
{
    // ISO 14229-1 RequestDownload:
    // [0x34, dataFormatIdentifier, addressAndLengthFormatIdentifier,
    //  memoryAddress[], memorySize[]].
    // alfid: low nibble = address byte count, high nibble = size byte count.
    uint8_t aLen = (uint8_t)(alfid & 0x0F);
    uint8_t sLen = (uint8_t)((alfid >> 4) & 0x0F);
    if (aLen == 0 || aLen > 4 || sLen == 0 || sLen > 4) {
        SerialUSB.println(F("Bad alfid (nibbles must be 1..4)."));
        return false;
    }
    uint8_t req[3 + 8];
    req[0] = 0x34; req[1] = 0x00; req[2] = alfid;  // dfi 0x00 = uncompressed/unencrypted
    for (uint8_t i = 0; i < aLen; i++)
        req[3 + i] = (uint8_t)((addr >> (8 * (aLen - 1 - i))) & 0xFF);
    for (uint8_t i = 0; i < sLen; i++)
        req[3 + aLen + i] = (uint8_t)((len >> (8 * (sLen - 1 - i))) & 0xFF);
    uint8_t reqLen = (uint8_t)(3 + aLen + sLen);
    uint8_t r[16]; uint8_t rl = 0;
    SerialUSB.print(F("TX: "));
    printHexBytes(req, reqLen);
    SerialUSB.println(F(" (RequestDownload)"));
    if (!txService(req, reqLen, r, sizeof(r), rl)) return false;
    // Positive: [0x74, lengthFormatIdentifier, maxNumberOfBlockLength...].
    // Phase 9: parsed into a local; the shared engine stores it (dl.maxBlockLen).
    uint32_t maxBlock = 0;
    if (rl >= 2) {
        uint8_t mbLen = (uint8_t)((r[1] >> 4) & 0x0F);
        for (uint8_t i = 0; i < mbLen && (uint8_t)(2 + i) < rl; i++)
            maxBlock = (maxBlock << 8) | r[2 + i];
    }
    SerialUSB.print(F("Positive: download accepted. ECU max block length: "));
    if (maxBlock) {
        SerialUSB.print(maxBlock);
        if (maxBlock < (uint32_t)(PROG_CHUNK_MAX + 2))
            SerialUSB.print(F("  WARNING: smaller than our 43-byte 0x36 blocks!"));
    } else SerialUSB.print(F("(not reported)"));
    SerialUSB.println();
    // Phase 9: open the shared download engine.
    // F-03-01: the 0x34-declared length is now the engine's invariant.
    if (!xfer::open(dl, len, maxBlock, millis())) {
        SerialUSB.println(F("Download rejected: zero declared length."));
        return false;
    }
    state = ST_DOWNLOADING;
    SerialUSB.println(F("Send image chunks now (web /prog upload, or mm channel)."));
    return true;
}

// One download chunk -> one 0x36 TransferData, immediately (streaming).
// Phase 9: the shared xfer engine does the work; PROG maps the result
// onto its state machine and keeps its interlocks. Returns mm ST_*.
uint8_t feedChunk(uint16_t seq, uint8_t flags, const uint8_t* data, uint8_t len,
                  uint16_t crc, uint16_t& ackSeq)
{
    ackSeq = seq;
    if (state != ST_DOWNLOADING) {
        SerialUSB.println(F("[prog] chunk rejected: not in DOWNLOADING state."));
        return mm::ST_ERR;
    }
    xfer::Io io = progXferIo();
    uint8_t rc = xfer::feedChunk(dl, io, seq, flags, data, len, crc, ackSeq);
    if (rc == 0) {
        if (dl.streamDone) {
            state = ST_STREAM_DONE;
            SerialUSB.println(F("Run `prog done` / finalize to compare CRCs."));
        }
        return mm::ST_OK;
    }
    // Shared engine already aborted (best-effort 0x37 sent).
    state = ST_ERROR;
    armed = false; tpOn = false;
    SerialUSB.println(F("Transfer state cleared and disarmed. See docs/SAFETY_REVIEW.md §5."));
    return (rc == 1) ? mm::ST_BAD_ARG : mm::ST_ERR;
}

bool doDone(uint32_t fileCrc32)
{
    if (state != ST_STREAM_DONE) {
        SerialUSB.println(F("No complete stream to finalize (need LAST chunk first)."));
        return false;
    }
    uint32_t finalCrc = dl.txCrc32 ^ 0xFFFFFFFFUL;
    SerialUSB.print(F("TX CRC32: 0x")); printHex32(finalCrc);
    SerialUSB.print(F("  file CRC32: 0x")); printHex32(fileCrc32);
    SerialUSB.println();
    // Phase 9: shared finalize (CRC compare + 0x37).
    xfer::Io io = progXferIo();
    if (!xfer::finish(dl, io, fileCrc32)) {
        state = ST_ERROR;
        armed = false; tpOn = false;
        SerialUSB.println(F("Transfer state cleared and disarmed. See docs/SAFETY_REVIEW.md §5."));
        return false;
    }
    state = ST_DONE;
    armed = false; tpOn = false;
    SerialUSB.println(F("Positive: transfer complete and verified. Disarmed."));
    return true;
}

bool doVerify(uint32_t addr, uint32_t len, uint8_t alfid,
                uint32_t* rxCrcOut, uint8_t* matchOut)
{
    // Optional read-back: 0x23 ReadMemoryByAddress over the flashed region,
    // CRC32 of the read-back vs the transmitted CRC32. Availability is
    // INFERRED — not all ECUs support 0x23 in programming session.
    if (state != ST_DONE && state != ST_STREAM_DONE) {
        SerialUSB.println(F("Nothing to verify against (no completed download)."));
        return false;
    }
    uint8_t aLen = (uint8_t)(alfid & 0x0F);
    uint8_t sLen = (uint8_t)((alfid >> 4) & 0x0F);
    if (aLen == 0 || aLen > 4 || sLen == 0 || sLen > 4) {
        SerialUSB.println(F("Bad alfid (nibbles must be 1..4)."));
        return false;
    }
    uint32_t rxCrc = 0xFFFFFFFFUL;
    uint32_t remaining = len;
    uint32_t cur = addr;
    constexpr uint16_t READ_SZ = 128;  // per-read chunk (INFERRED sane size)
    SerialUSB.println(F("Read-back verify (0x23)..."));
    while (remaining > 0) {
        uint16_t want = (remaining > READ_SZ) ? READ_SZ : (uint16_t)remaining;
        uint8_t req[3 + 8];
        req[0] = 0x23; req[1] = alfid;
        for (uint8_t i = 0; i < aLen; i++)
            req[2 + i] = (uint8_t)((cur >> (8 * (aLen - 1 - i))) & 0xFF);
        // size field width = sLen bytes; encode `want`
        for (uint8_t i = 0; i < sLen; i++)
            req[2 + aLen + i] = (uint8_t)(((uint32_t)want >> (8 * (sLen - 1 - i))) & 0xFF);
        uint8_t reqLen = (uint8_t)(2 + aLen + sLen);
        uint8_t r[160]; uint8_t rl = 0;
        if (!txService(req, reqLen, r, sizeof(r), rl, 2000)) {
            SerialUSB.println(F("Verify aborted: ECU did not answer 0x23."));
            SerialUSB.println(F("(0x23 availability is INFERRED — the ECU may not support it.)"));
            return false;
        }
        // Positive: [0x63, data...]. r[0] already checked == 0x63 by txService.
        uint8_t got = (uint8_t)(rl - 1);
        rxCrc = xfer::crc32feed(rxCrc, r + 1, got);
        cur += got;
        if (got == 0 || (uint32_t)got > remaining) break;  // avoid infinite loop
        remaining -= got;
        if (got < want) break;  // ECU gave less than asked; stop
    }
    uint32_t finalRx = rxCrc ^ 0xFFFFFFFFUL;
    uint32_t finalTx = dl.txCrc32 ^ 0xFFFFFFFFUL;
    SerialUSB.print(F("Read-back CRC32: 0x")); printHex32(finalRx);
    SerialUSB.print(F("  transmitted CRC32: 0x")); printHex32(finalTx);
    SerialUSB.println();
    if (finalRx == finalTx) {
        SerialUSB.println(F("VERIFY: MATCH — ECU content equals transmitted image."));
        if (rxCrcOut) *rxCrcOut = finalRx;
        if (matchOut) *matchOut = 1;
        return true;
    }
    SerialUSB.println(F("VERIFY: MISMATCH — do NOT trust this flash."));
    if (rxCrcOut) *rxCrcOut = finalRx;
    if (matchOut) *matchOut = 0;
    return false;
}

bool doReset()
{
    const uint8_t req[2] = { 0x11, 0x01 };  // hardReset
    uint8_t r[16]; uint8_t rl = 0;
    SerialUSB.println(F("TX: 11 01 (ECUReset/hardReset)"));
    if (!txService(req, 2, r, sizeof(r), rl)) return false;
    session = 0x01; state = ST_IDLE; tpOn = false; armed = false;
    SerialUSB.println(F("Positive: ECU reset. Session back to default; disarmed."));
    return true;
}

void printStatus()
{
    SerialUSB.print(F("PROG state: ")); SerialUSB.println(stateName(state));
    SerialUSB.print(F("  session: 0x")); printHex2(session);
    SerialUSB.print(F("  bus: CAN")); SerialUSB.print(busSel);
    SerialUSB.print(F(" @ ")); SerialUSB.print(baud);
    SerialUSB.print(F("  TX 0x")); SerialUSB.print(String(txId, HEX));
    SerialUSB.print(F(" RX 0x")); SerialUSB.print(String(rxId, HEX));
    SerialUSB.print(F("  configured: ")); SerialUSB.println(configured ? F("yes") : F("NO"));
    SerialUSB.print(F("  armed: "));
    if (armed && (uint32_t)(millis() - armedAt) < ARM_MS) SerialUSB.println(F("yes"));
    else SerialUSB.println(F("no"));
    SerialUSB.print(F("  power attested: ")); SerialUSB.println(powerOk ? F("yes") : F("NO"));
    SerialUSB.print(F("  tester-present: ")); SerialUSB.println(tpOn ? F("on") : F("off"));
    SerialUSB.print(F("  bytes sent: ")); SerialUSB.print(dl.bytesRx);
    SerialUSB.print(F("  chunks: ")); SerialUSB.print(dl.chunksRx);
    SerialUSB.print(F("  ECU max block: "));
    SerialUSB.println(dl.maxBlockLen ? String(dl.maxBlockLen) : F("(unknown)"));
    SerialUSB.print(F("  TX CRC32: 0x")); printHex32(dl.txCrc32 ^ 0xFFFFFFFFUL); SerialUSB.println();
    SerialUSB.print(F("  MITM rules suspended on entry: ")); SerialUSB.println(mitmSuspended);
}

void printHelp()
{
    SerialUSB.println(F("PROG console (config any time; bus commands need 'prog' mode):"));
    SerialUSB.println(F("  prog status | prog bus 0|1 | prog baud <n> | prog tx <hex> | prog rx <hex>"));
    SerialUSB.println(F("  prog arm | prog power ok"));
    SerialUSB.println(F("  prog session | prog seed <lvl> | prog key <hex...>  (key=user-supplied)"));
    SerialUSB.println(F("  prog erase <ridHex> | prog download <addrHex> <lenHex> [alfidHex]"));
    SerialUSB.println(F("  prog verify <addrHex> <lenHex> [alfidHex] | prog done <fileCrc32Hex>"));
    SerialUSB.println(F("  prog reset | prog abort"));
    SerialUSB.println(F("Destructive steps need `prog arm` (+ power attestation for erase/download)."));
    SerialUSB.println(F("READ docs/SAFETY_REVIEW.md BEFORE flashing anything."));
}

} // namespace (anonymous)

void begin(uint8_t suspended)
{
    active = true;
    busUp = false;      // lazy: first command acquires (ownership rule)
    session = 0x01;
    state = ST_IDLE;
    configured = false; // interlock (c): must be set explicitly each entry
    armed = false;
    powerOk = false;    // attestation does NOT survive a mode switch
    seedLevel = 0;
    xfer::reset(dl);  // Phase 9: shared download engine state
    tpOn = false;
    mitmSuspended = suspended;
    divert.sawF1 = false; divert.binActive = false;
    xfer::crcInit();
    SerialUSB.println(F("PROG mode: UDS flashing over CAN ISO-TP (11-bit)."));
    SerialUSB.println(F("READ docs/SAFETY_REVIEW.md before use. Nothing is armed."));
    if (suspended > 0) {
        SerialUSB.print(F("[prog] MITM rule table auto-suspended: "));
        SerialUSB.print(suspended);
        SerialUSB.println(F(" rule(s) cleared. They are NOT restored on exit —"));
        SerialUSB.println(F("re-add them deliberately via `mitm rule add` / the /mitm page."));
    }
}

void loop()
{
    // mm control-channel divert on Serial (chunk upload + supervision).
    while (Serial.available() > 0) {
        uint8_t b = (uint8_t)Serial.read();
        mm::divertByte(divert, b);
    }
    if (!active) return;
    uint32_t now = millis();
    // Tester-present keepalive while a non-default session is up.
    if (tpOn && busUp && session != 0x01 &&
        (uint32_t)(now - lastTp) >= TP_MS) {
        const uint8_t tp[2] = { 0x3E, 0x00 };
        isotp::Endpoint ep = endpoint();
        uint8_t r[8]; uint8_t rl = 0;
        isotp::request(ep, tp, 2, r, sizeof(r), rl, 500);
        lastTp = now;
    }
    // Chunk watchdog: a stalled upload must not leave the ECU hanging.
    // Phase 9: shared engine watchdog; PROG clears its own state after.
    if (state == ST_DOWNLOADING) {
        xfer::Io io = progXferIo();
        if (xfer::watchdog(dl, io, now)) {
            state = ST_ERROR;
            armed = false; tpOn = false;
            SerialUSB.println(F("Transfer state cleared and disarmed. See docs/SAFETY_REVIEW.md §5."));
        }
    }
}

void exit()
{
    if (state == ST_DOWNLOADING || state == ST_STREAM_DONE)
        abortTransfer("mode exit during transfer", true);
    busRelease();   // idempotent; manager's quiesceBuses() follows
    active = false;
    armed = false;
    tpOn = false;
}

void console(char* line)
{
    while (*line == ' ') line++;
    if (*line == '\0') { printHelp(); return; }

    char* save = nullptr;
    char* tok = strtok_r(line, " \t", &save);
    if (!tok) { printHelp(); return; }

    if (strcasecmp(tok, "status") == 0) { printStatus(); return; }
    if (strcasecmp(tok, "help") == 0) { printHelp(); return; }

    if (strcasecmp(tok, "bus") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        if (a && (a[0] == '0' || a[0] == '1') && a[1] == '\0') {
            uint8_t nb = (uint8_t)(a[0] - '0');
            if (nb != busSel) { busSel = nb; busRelease(); }
            configured = true;
            SerialUSB.print(F("PROG bus: CAN")); SerialUSB.println(busSel);
        } else SerialUSB.println(F("usage: prog bus 0|1"));
        return;
    }
    if (strcasecmp(tok, "baud") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        uint32_t b = a ? (uint32_t)strtoul(a, nullptr, 10) : 0;
        if (validBaud(b)) {
            if (b != baud) { baud = b; busRelease(); }
            configured = true;
            SerialUSB.print(F("PROG baud: ")); SerialUSB.println(baud);
        } else SerialUSB.println(F("usage: prog baud <100000|125000|250000|500000|800000|1000000>"));
        return;
    }
    if (strcasecmp(tok, "tx") == 0 || strcasecmp(tok, "rx") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        uint32_t v = 0;
        if (a && hexU32(a, v) && v <= 0x7FF) {
            if (tok[0] == 't' || tok[0] == 'T') txId = v; else rxId = v;
            configured = true;
            SerialUSB.print(tok); SerialUSB.print(F(" ID: 0x"));
            SerialUSB.println(String(v, HEX));
        } else SerialUSB.println(F("usage: prog tx|rx <hex, 11-bit>"));
        return;
    }
    if (strcasecmp(tok, "arm") == 0) {
        if (!needActive()) return;
        armed = true; armedAt = millis();
        SerialUSB.println(F("Armed for 120 s. Destructive steps will now run."));
        return;
    }
    if (strcasecmp(tok, "power") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        if (a && strcasecmp(a, "ok") == 0) {
            powerOk = true;
            SerialUSB.println(F("Power attested: stable 12V+ confirmed (user attestation,"));
            SerialUSB.println(F("NOT a measurement — see docs/SAFETY_REVIEW.md §4)."));
        } else SerialUSB.println(F("usage: prog power ok"));
        return;
    }
    if (strcasecmp(tok, "abort") == 0) {
        if (!needActive()) return;
        abortTransfer("user abort", true);
        return;
    }
    // ---- bus-touching commands below ----
    if (strcasecmp(tok, "session") == 0) {
        if (!needActive() || !needConfigured()) return;
        doSession();
        return;
    }
    if (strcasecmp(tok, "seed") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        uint8_t lvl = 0;
        if (!needActive() || !needConfigured() || !a || !hexByte(a, lvl)) {
            SerialUSB.println(F("usage: prog seed <levelHex>"));
            return;
        }
        doSeed(lvl, true);
        return;
    }
    if (strcasecmp(tok, "key") == 0) {
        if (!needActive() || !needConfigured() || !needArmed()) return;
        uint8_t key[64]; uint8_t n = 0; uint8_t b = 0;
        for (char* a = strtok_r(nullptr, " \t", &save);
             a && n < sizeof(key);
             a = strtok_r(nullptr, " \t", &save)) {
            if (!hexByte(a, b)) { SerialUSB.println(F("Bad hex byte.")); return; }
            key[n++] = b;
        }
        if (n == 0) { SerialUSB.println(F("usage: prog key <hex...>")); return; }
        doKey(key, n);
        return;
    }
    if (strcasecmp(tok, "erase") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        uint32_t rid = 0;
        if (!needActive() || !needConfigured() || !needArmed() || !needPower() ||
            !a || !hexU32(a, rid) || rid > 0xFFFF) {
            SerialUSB.println(F("usage: prog erase <routineIdHex>"));
            return;
        }
        doErase((uint16_t)rid);
        return;
    }
    if (strcasecmp(tok, "download") == 0) {
        char* aA = strtok_r(nullptr, " \t", &save);
        char* aL = strtok_r(nullptr, " \t", &save);
        char* aF = strtok_r(nullptr, " \t", &save);
        uint32_t addr = 0, len = 0; uint8_t alfid = 0x44;
        uint8_t f = 0;
        if (!needActive() || !needConfigured() || !needArmed() || !needPower() ||
            !aA || !hexU32(aA, addr) || !aL || !hexU32(aL, len) || len == 0 ||
            (aF && !hexByte(aF, f))) {
            SerialUSB.println(F("usage: prog download <addrHex> <lenHex> [alfidHex]"));
            return;
        }
        if (aF) alfid = f;
        doDownload(addr, len, alfid);
        return;
    }
    if (strcasecmp(tok, "verify") == 0) {
        char* aA = strtok_r(nullptr, " \t", &save);
        char* aL = strtok_r(nullptr, " \t", &save);
        char* aF = strtok_r(nullptr, " \t", &save);
        uint32_t addr = 0, len = 0; uint8_t alfid = 0x44;
        uint8_t f = 0;
        if (!needActive() || !needConfigured() ||
            !aA || !hexU32(aA, addr) || !aL || !hexU32(aL, len) || len == 0 ||
            (aF && !hexByte(aF, f))) {
            SerialUSB.println(F("usage: prog verify <addrHex> <lenHex> [alfidHex]"));
            return;
        }
        if (aF) alfid = f;
        doVerify(addr, len, alfid, nullptr, nullptr);
        return;
    }
    if (strcasecmp(tok, "done") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        uint32_t fcrc = 0;
        if (!needActive() || !needArmed() || !a || !hexU32(a, fcrc)) {
            SerialUSB.println(F("usage: prog done <fileCrc32Hex>"));
            return;
        }
        doDone(fcrc);
        return;
    }
    if (strcasecmp(tok, "reset") == 0) {
        if (!needActive() || !needArmed()) return;
        doReset();
        return;
    }
    printHelp();
}

// ---- mm-channel ops ----

uint8_t mmCfg(uint8_t bus, uint32_t baudIn, uint32_t txIdIn, uint32_t rxIdIn)
{
    if (bus > 1 || !validBaud(baudIn) || txIdIn > 0x7FF || rxIdIn > 0x7FF)
        return mm::ST_BAD_ARG;
    bool busChanged = (bus != busSel) || (baudIn != baud);
    busSel = bus; baud = baudIn; txId = txIdIn; rxId = rxIdIn;
    configured = true;   // explicit selection (interlock c)
    if (busChanged) busRelease();  // re-acquire lazily
    return mm::ST_OK;
}

uint8_t mmTx(const uint8_t* svc, uint8_t svcLen,
             uint8_t* resp, uint8_t* respLen, uint8_t* nResp)
{
    *nResp = 0; *respLen = 0;
    if (!active) return mm::ST_ERR;   // PROG_TX only valid in PROG mode
    if (svcLen == 0 || svcLen > 12) return mm::ST_BAD_ARG;
    // Defense in depth (SAFETY_REVIEW.md §3): destructive services require a
    // live arm even on this raw path. The web wizard always arms first
    // (progArmWeb) and the console enforces arm in console(), so legitimate
    // flows are unaffected — this closes the hole where a future caller
    // forgets to arm. Same pattern does NOT apply to CMD_UDS_TX (diagnostics
    // only); PROG mode can brick an ECU, so it gets the stronger rule.
    if (svcLen >= 1) {
        switch (svc[0]) {
            case 0x11: case 0x2E: case 0x31:
            case 0x34: case 0x36: case 0x37:
                // F3 (2026-10-08): destructive services require a live arm
                // AND power attestation. mmArm() latches powerOk only when
                // powerOkIn is true, so arming without attestation leaves
                // powerOk=false and these services are refused here.
                if (!armed || !powerOk ||
                    (uint32_t)(millis() - armedAt) >= ARM_MS) {
                    armed = false; powerOk = false;
                    return mm::ST_ERR;  // not armed/attested: refuse
                }
                break;
            default: break;
        }
    }
    // No M2-side arm beyond the above: the web wizard confirms per step
    // (same pattern as CMD_UDS_TX) and the console path enforces arm.
    uint8_t r[64]; uint8_t rl = 0;
    if (!busAcquire()) return mm::ST_ERR;
    isotp::Endpoint ep = endpoint();
    if (!isotp::request(ep, svc, svcLen, r, sizeof(r), rl, 1000))
        return mm::ST_OK;  // transport done; nResp=0 reports "no data"
    *nResp = 1;
    uint8_t n = (rl <= 64) ? rl : 64;
    for (uint8_t i = 0; i < n; i++) resp[i] = r[i];
    *respLen = n;
    // Session tracking mirrors uds::transact.
    if (svcLen >= 2 && svc[0] == 0x10 && rl >= 1 && r[0] == 0x50) {
        session = svc[1];
        if (session != 0x01) { state = ST_SESSION; tpOn = true; lastTp = 0; }
    }
    return mm::ST_OK;
}

uint8_t mmData(uint16_t seq, uint8_t flags, const uint8_t* data, uint8_t len,
               uint16_t crc, uint16_t& ackSeq)
{
    if (!active) return mm::ST_ERR;
    if (len > PROG_CHUNK_MAX) return mm::ST_BAD_ARG;
    return feedChunk(seq, flags, data, len, crc, ackSeq);
}

uint8_t mmDone(uint32_t fileCrc32, uint32_t& txCrc, uint8_t& match)
{
    txCrc = dl.txCrc32 ^ 0xFFFFFFFFUL;
    match = 0;
    if (!active) return mm::ST_ERR;
    if (!armed || (uint32_t)(millis() - armedAt) >= ARM_MS) {
        armed = false;
        return mm::ST_ERR;  // finalize is destructive: needs arm
    }
    if (state != ST_STREAM_DONE) return mm::ST_ERR;
    if (txCrc != fileCrc32) {
        abortTransfer("end-to-end CRC32 MISMATCH — image rejected, NOT finalized", false);
        return mm::ST_ERR;
    }
    const uint8_t req[1] = { 0x37 };
    uint8_t r[16]; uint8_t rl = 0;
    if (!txService(req, 1, r, sizeof(r), rl)) {
        abortTransfer("0x37 RequestTransferExit failed", false);
        return mm::ST_ERR;
    }
    state = ST_DONE;
    armed = false; tpOn = false;
    match = 1;
    return mm::ST_OK;
}

uint8_t mmDownload(uint32_t addr, uint32_t len, uint8_t alfid)
{
    if (!active) return mm::ST_ERR;
    if (!configured) return mm::ST_ERR;
    if (!armed || (uint32_t)(millis() - armedAt) >= ARM_MS) {
        armed = false;
        return mm::ST_ERR;
    }
    if (!powerOk) return mm::ST_ERR;
    if (len == 0) return mm::ST_BAD_ARG;
    return doDownload(addr, len, alfid) ? mm::ST_OK : mm::ST_ERR;
}

uint8_t mmVerify(uint32_t addr, uint32_t len, uint8_t alfid,
                 uint8_t& match, uint32_t& rxCrc)
{
    match = 0; rxCrc = 0;
    if (!active) return mm::ST_ERR;
    if (!configured) return mm::ST_ERR;
    if (len == 0) return mm::ST_BAD_ARG;
    return doVerify(addr, len, alfid, &rxCrc, &match) ? mm::ST_OK : mm::ST_ERR;
}

void mmAbort()
{
    if (!active) return;
    abortTransfer("mm-channel abort", true);
}

uint8_t mmArm(uint8_t powerOkIn, uint8_t& armedOut)
{
    armedOut = 0;
    if (!active) return mm::ST_ERR;
    if (powerOkIn) powerOk = true;
    armed = true; armedAt = millis();
    armedOut = 1;
    return mm::ST_OK;
}

void mmStatus(uint8_t* out)
{
    out[0] = state;
    out[1] = session;
    out[2] = busSel;
    out[3] = (uint8_t)(baud & 0xFF); out[4] = (uint8_t)((baud >> 8) & 0xFF);
    out[5] = (uint8_t)((baud >> 16) & 0xFF); out[6] = (uint8_t)((baud >> 24) & 0xFF);
    out[7] = (uint8_t)(txId & 0xFF); out[8] = (uint8_t)((txId >> 8) & 0xFF);
    out[9] = (uint8_t)((txId >> 16) & 0xFF); out[10] = (uint8_t)((txId >> 24) & 0xFF);
    out[11] = (uint8_t)(rxId & 0xFF); out[12] = (uint8_t)((rxId >> 8) & 0xFF);
    out[13] = (uint8_t)((rxId >> 16) & 0xFF); out[14] = (uint8_t)((rxId >> 24) & 0xFF);
    out[15] = configured ? 1 : 0;
    out[16] = powerOk ? 1 : 0;
    out[17] = (armed && (uint32_t)(millis() - armedAt) < ARM_MS) ? 1 : 0;
    out[18] = (uint8_t)(dl.bytesRx & 0xFF); out[19] = (uint8_t)((dl.bytesRx >> 8) & 0xFF);
    out[20] = (uint8_t)((dl.bytesRx >> 16) & 0xFF); out[21] = (uint8_t)((dl.bytesRx >> 24) & 0xFF);
    out[22] = (uint8_t)(dl.chunksRx & 0xFF); out[23] = (uint8_t)((dl.chunksRx >> 8) & 0xFF);
    uint32_t tc = dl.txCrc32 ^ 0xFFFFFFFFUL;
    out[24] = (uint8_t)(tc & 0xFF); out[25] = (uint8_t)((tc >> 8) & 0xFF);
    out[26] = (uint8_t)((tc >> 16) & 0xFF); out[27] = (uint8_t)((tc >> 24) & 0xFF);
    out[28] = mitmSuspended;
}

} // namespace prog
