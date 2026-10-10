// M2 multi-mode firmware — ISO 9141-2 / KWP2000 K-line implementation (Phase 7).
// See kline.h for pin map, timing sources, and the INFERRED banner.
//
// EVERYTHING HERE IS INFERRED UNTIL TESTED AGAINST A K-LINE VEHICLE.
// (The available test vehicle, a 2023 Chevy Bolt EV, is CAN-only.)

#include "kline.h"

#ifndef SIM_M2
#include "rail9141.h"
#include "j1850.h"
#include "busstat.h"
#include "busfilter.h"
#include "txsched.h"
#include "debug.h"
#endif

namespace kline {

uint8_t checksum(const uint8_t* d, uint8_t n)
{
    uint8_t s = 0;
    for (uint8_t i = 0; i < n; i++) s = (uint8_t)(s + d[i]);
    return s;
}

#ifdef SIM_M2
// ---- host sim: no USART0/1, no pins. Pure-logic only (checksum above). ----
bool acquire() { return false; }
void release() {}
bool acquired() { return false; }
bool sendOnly(const uint8_t*, uint8_t) { return false; }
bool transact(const uint8_t*, uint8_t, uint8_t*, uint8_t, uint8_t&, uint16_t)
{ return false; }
bool initSlow(uint8_t, uint8_t, bool) { return false; }
bool initFast(uint8_t) { return false; }
Init initDone() { return Init::NONE; }
bool request9141(const uint8_t[3], const uint8_t*, uint8_t, uint8_t*, uint8_t, uint8_t&,
                 uint16_t)
{ return false; }
bool kwpService(uint8_t, const uint8_t*, uint8_t, uint8_t*, uint8_t, uint8_t&,
                uint16_t)
{ return false; }
bool readMemoryByAddress(uint32_t, uint8_t, uint8_t*, uint8_t&) { return false; }
bool testerPresent() { return false; }
void setTiming(const Timing&) {}
void getTiming(Timing& t) { t = Timing(); }
void setAddrs(uint8_t, uint8_t) {}
void getStatus(Status& s) { s = Status(); }
void testerPresentStart(uint32_t) {}
void testerPresentStop() {}
void pollRx() {}
bool pollFrame(uint8_t*, uint8_t, uint8_t&) { return false; }
uint8_t queueDepth() { return 0; }
bool probe() { return false; }

#else
// ============================ hardware build ============================

namespace {

bool isAcquired = false;
Init curInit = Init::NONE;
uint8_t kwpKb1 = 0, kwpKb2 = 0;
uint8_t tgtAddr = 0x33;    // ECU target address (functional default)
uint8_t tstAddr = 0xF1;    // tester address
Timing tm;

// ---- init timings (us/ms). SOURCES: ISO 9141-2 / ISO 14230-2 (INFERRED) ----
constexpr uint16_t BAUD5_BIT_MS = 200;    // 5 baud
constexpr uint16_t SLOW_IDLE_MS = 300;     // Tidle before slow init
constexpr uint16_t SYNC_TIMEOUT_MS = 300;  // wait for 0x55 after 5-baud byte
constexpr uint16_t KB_GAP_MS = 60;         // inter-keyword read timeout
constexpr uint16_t FAST_WAKE_MS = 25;      // Twup low/high
constexpr uint16_t RAIL_SETTLE_MS = 50;    // carried over from J1850 (INFERRED)

// ---- frame queue (monitor path), mirrors j1850's discipline ----
constexpr uint8_t FRAME_MAX = 64;  // KWP2000 frames can exceed J1850's 12
constexpr uint8_t Q_DEPTH = 4;
uint8_t qLen[Q_DEPTH] = { 0, 0, 0, 0 };
uint8_t qBuf[Q_DEPTH][FRAME_MAX];
uint8_t qHead = 0, qTail = 0;

// tester-present scheduler slot (0xFF = none)
uint8_t tpSlot = 0xFF;

// ---- transceiver sleep control ----
// SLP_N is active-LOW by naming convention (TJA1027 symbol, pack
// constraints tja1027-kline). Polarity INFERRED until bench-verified.
inline void transceiversNormal()
{
    pinMode(K_SLP, OUTPUT); digitalWrite(K_SLP, HIGH);
    pinMode(L_SLP, OUTPUT); digitalWrite(L_SLP, HIGH);
    delayMicroseconds(50);  // covers tinit(norm) 7-20 us + margin
}
inline void transceiversSleep()
{
    digitalWrite(K_SLP, LOW);
    digitalWrite(L_SLP, LOW);
    delayMicroseconds(50);  // covers tgotosleep 2-10 us + margin
}

inline void flushK()
{
    while (Serial1.available() > 0) Serial1.read();
}

// Write with tester inter-byte time P4 (INFERRED enforcement).
void writePaced(const uint8_t* d, uint8_t n)
{
    for (uint8_t i = 0; i < n; i++) {
        Serial1.write(d[i]);
        if (i + 1 < n) delay(tm.p4Min);
    }
    Serial1.flush();  // Due: wait until the UART actually shifted everything
}

// Read exactly n bytes (echo verification). Returns false on timeout.
bool readExact(uint8_t* d, uint8_t n, uint16_t timeoutMs)
{
    uint32_t start = millis();
    uint8_t got = 0;
    while (got < n) {
        while (Serial1.available() > 0 && got < n) d[got++] = (uint8_t)Serial1.read();
        if (got >= n) return true;
        if ((uint32_t)(millis() - start) >= timeoutMs) return false;
    }
    return true;
}

// Collect a response frame: first byte within timeoutMs, then bytes until
// the inter-byte gap exceeds 2*P1max (INFERRED frame delimiter for K-line,
// which has no explicit end-of-frame). Strips+verifies the checksum.
bool collectFrame(uint8_t* rx, uint8_t rxCap, uint8_t& rxLen, uint16_t timeoutMs)
{
    rxLen = 0;
    uint32_t start = millis();
    while (Serial1.available() == 0) {
        if ((uint32_t)(millis() - start) >= timeoutMs) return false;
    }
    uint32_t lastByte = millis();
    const uint16_t gapMax = (uint16_t)(tm.p1Max * 2);
    while (rxLen < rxCap) {
        while (Serial1.available() > 0 && rxLen < rxCap) {
            rx[rxLen++] = (uint8_t)Serial1.read();
            lastByte = millis();
        }
        if ((uint32_t)(millis() - lastByte) >= gapMax) break;
    }
    if (rxLen < 2) return false;  // need at least 1 data byte + checksum
    uint8_t cs = rx[rxLen - 1];
    if (checksum(rx, (uint8_t)(rxLen - 1)) != cs) {
        dbg::println(F("[kline] response checksum mismatch"));
        return false;
    }
    rxLen--;  // strip checksum
    return true;
}

// Bit-bang one byte at 5 baud on a GPIO pin (dominant-LOW like UART).
void bitBang5(uint8_t pin, uint8_t b)
{
    digitalWrite(pin, LOW); delay(BAUD5_BIT_MS);          // start bit
    for (uint8_t i = 0; i < 8; i++) {                     // LSB first
        digitalWrite(pin, (b & (1 << i)) ? HIGH : LOW);
        delay(BAUD5_BIT_MS);
    }
    digitalWrite(pin, HIGH); delay(BAUD5_BIT_MS);         // stop bit
}

// Push a frame into the monitor queue (drop-new on full, like j1850).
void pushFrame(const uint8_t* buf, uint8_t len)
{
    if (len == 0 || len > FRAME_MAX) {
        busstat::noteRxOverflow(busstat::Bus::KLINE);
        return;
    }
    uint8_t next = (uint8_t)((qHead + 1) % Q_DEPTH);
    if (next == qTail) { busstat::noteRxOverflow(busstat::Bus::KLINE); return; }
    for (uint8_t i = 0; i < len; i++) qBuf[qHead][i] = buf[i];
    qLen[qHead] = len;
    qHead = next;
}

// W-2: fire-and-forget keep-alive for the txsched callback. The 0x3E
// subfunction is response-suppressed by definition, so the keep-alive
// must not wait on the (INFERRED) transceiver echo — sendOnly()'s
// 200 ms echo wait would stall the whole main loop every period.
// Echo verification stays in sendOnly()/transact() for real traffic.
void testerPresentFire()
{
    if (curInit == Init::NONE) return;
    uint8_t req[6] = { 0x82, tgtAddr, tstAddr, 0x3E, 0x00, 0x00 };
    req[5] = checksum(req, 5);
    flushK();
    writePaced(req, 6);  // includes Serial1.flush(); best-effort, no echo wait
    busstat::noteTx(busstat::Bus::KLINE, 6);
}

void tpFire(uint8_t)  // txsched callback: tester-present keep-alive
{
    if (isAcquired) testerPresentFire();
}

// Never run K-line traffic while the J1850 driver owns its block: the two
// share the rail, and interleaved inits would violate INV-MODE50's spirit
// (pin 50 must not move under J1850 traffic). ELM's busRelease() already
// sequences this; this is defense in depth for console/mm callers.
inline bool j1850InUse() { return j1850::acquired(); }

} // namespace

bool acquire()
{
    if (isAcquired) return false;
    // 1. Shared rail (refcounted; stays up if J1850 holds it). The K-line
    //    has no mode-select pin to settle, so INV-PWR49's ordering
    //    constraint is trivially satisfied.
    rail9141::powerUp(rail9141::Owner::KLINE);
    // 2. Transceivers to normal.
    transceiversNormal();
    // 3. UARTs at 10400 8N1 (TJA1027 max 20 kBd — pack VERIFIED).
    Serial1.begin(KLINE_BAUD);
    Serial2.begin(KLINE_BAUD);  // L-line: wake use (Phase 7)
    delay(RAIL_SETTLE_MS);
    flushK();
    busstat::setPresent(busstat::Bus::KLINE, true);
    isAcquired = true;
    dbg::println(F("[kline] acquired (rail+SLP+UARTs @10400)"));
    return true;
}

void release()
{
    testerPresentStop();
    if (isAcquired) {
        Serial1.end();
        Serial2.end();
        transceiversSleep();
        isAcquired = false;
        curInit = Init::NONE;
    }
    // Rail released refcounted: stays HIGH if J1850 still holds it
    // (INV-PWR49). Idempotent per owner.
    rail9141::powerDown(rail9141::Owner::KLINE);
    busstat::setPresent(busstat::Bus::KLINE, false);
    qHead = 0; qTail = 0;
    dbg::println(F("[kline] released"));
}

bool acquired() { return isAcquired; }

bool sendOnly(const uint8_t* tx, uint8_t txLen)
{
    if (!isAcquired || txLen == 0 || tx == nullptr) return false;
    uint8_t frame[FRAME_MAX + 1];
    if (txLen > FRAME_MAX) return false;
    for (uint8_t i = 0; i < txLen; i++) frame[i] = tx[i];
    frame[txLen] = checksum(tx, txLen);
    flushK();
    writePaced(frame, (uint8_t)(txLen + 1));
    busstat::noteTx(busstat::Bus::KLINE, (uint8_t)(txLen + 1));
    // Echo check: the TJA1027 loops our TX bytes back to RX (INFERRED).
    uint8_t echo[FRAME_MAX + 1];
    if (!readExact(echo, (uint8_t)(txLen + 1), 200)) {
        busstat::noteTxFail(busstat::Bus::KLINE);
        dbg::println(F("[kline] echo timeout"));
        return false;
    }
    for (uint8_t i = 0; i <= txLen; i++) {
        if (echo[i] != frame[i]) {
            busstat::noteTxFail(busstat::Bus::KLINE);
            dbg::println(F("[kline] echo mismatch"));
            return false;
        }
    }
    return true;
}

bool transact(const uint8_t* req, uint8_t reqLen,
              uint8_t* rx, uint8_t rxCap, uint8_t& rxLen,
              uint16_t timeoutMs)
{
    rxLen = 0;
    if (!isAcquired || reqLen == 0 || req == nullptr ||
        rx == nullptr || rxCap < 2) return false;
    if (!sendOnly(req, reqLen)) return false;
    uint16_t to = (timeoutMs != 0) ? timeoutMs : tm.p2Max;
    uint8_t tmp[FRAME_MAX];
    uint8_t n = 0;
    if (!collectFrame(tmp, sizeof(tmp), n, to)) {
        dbg::println(F("[kline] NO DATA (no response)"));
        return false;
    }
    // Phase 6 pattern: consult the shared filter table when non-empty.
    // addr = ECU source address (response header byte 2, INFERRED layout).
    uint32_t srcAddr = (n >= 3) ? tmp[2] : busfilter::ADDR_ANY;
    if (!busfilter::accept(busfilter::Bus::KLINE, busfilter::Proto::KLINE,
                           srcAddr, 0, busfilter::Dir::RX, tmp, n)) {
        busstat::noteDropped(busstat::Bus::KLINE);
        dbg::println(F("[kline] response filtered"));
        return false;
    }
    if (n > rxCap) { busstat::noteRxOverflow(busstat::Bus::KLINE); return false; }
    for (uint8_t i = 0; i < n; i++) rx[i] = tmp[i];
    rxLen = n;
    busstat::noteRx(busstat::Bus::KLINE, (uint8_t)(n + 1));  // +checksum byte
    if (dbg::enabled) dbg::hexBytes(F("[kline] RX"), srcAddr, rx, rxLen);
    return true;
}

bool initSlow(uint8_t targetAddr, uint8_t retries, bool useL)
{
    if (!isAcquired) return false;
    if (j1850InUse()) return false;  // never fight the J1850 driver
    for (uint8_t attempt = 0; attempt < retries; attempt++) {
        if (dbg::enabled) {
            dbg::print(F("[kline] 5-baud init attempt "));
            dbg::println((uint32_t)(attempt + 1));
        }
        // 1. Idle the bus (recessive/HIGH) for Tidle.
        Serial1.end();
        if (useL) Serial2.end();
        pinMode(K_TX, OUTPUT);
        digitalWrite(K_TX, HIGH);
        if (useL) { pinMode(L_TX, OUTPUT); digitalWrite(L_TX, HIGH); }
        delay(SLOW_IDLE_MS);
        // 2. Bit-bang the address byte at 5 baud (200 ms/bit, INFERRED).
        bitBang5(K_TX, targetAddr);
        if (useL) bitBang5(L_TX, targetAddr);
        // 3. Back to UART @10400 for the keyword handshake.
        //    INFERRED: pinMode(OUTPUT)->Serial1.begin() re-muxes the pin
        //    from GPIO back to USART0 peripheral (standard Due behavior,
        //    untested here).
        Serial1.begin(KLINE_BAUD);
        if (useL) Serial2.begin(KLINE_BAUD);
        flushK();
        // 4. Sync 0x55, then KB1, KB2.
        uint8_t sync = 0;
        if (!readExact(&sync, 1, SYNC_TIMEOUT_MS) || sync != 0x55) continue;
        uint8_t kb[2];
        if (!readExact(kb, 2, KB_GAP_MS)) continue;
        // 5. Tester sends inverted KB2; ECU answers inverted address.
        uint8_t invKb2 = (uint8_t)~kb[1];
        uint8_t invAddr = 0;
        Serial1.write(invKb2); Serial1.flush();
        if (!readExact(&invAddr, 1, KB_GAP_MS)) continue;
        if (invAddr != (uint8_t)~targetAddr) continue;
        kwpKb1 = kb[0]; kwpKb2 = kb[1];
        tgtAddr = targetAddr;
        curInit = Init::SLOW_5BAUD;
        busstat::noteRx(busstat::Bus::KLINE, 4);
        if (dbg::enabled) {
            dbg::print(F("[kline] 5-baud init OK, KB="));
            dbg::print((uint32_t)kb[0], HEX);
            dbg::print(F(" "));
            dbg::println((uint32_t)kb[1], HEX);
        }
        return true;
    }
    dbg::println(F("[kline] 5-baud init FAILED"));
    return false;
}

bool initFast(uint8_t retries)
{
    if (!isAcquired) return false;
    if (j1850InUse()) return false;
    for (uint8_t attempt = 0; attempt < retries; attempt++) {
        if (dbg::enabled) {
            dbg::print(F("[kline] fast init attempt "));
            dbg::println((uint32_t)(attempt + 1));
        }
        // 1. Wake-up pattern: 25 ms dominant LOW, 25 ms recessive HIGH
        //    (ISO 14230 Twup, INFERRED) via GPIO bit-bang.
        Serial1.end();
        pinMode(K_TX, OUTPUT);
        digitalWrite(K_TX, LOW); delay(FAST_WAKE_MS);
        digitalWrite(K_TX, HIGH); delay(FAST_WAKE_MS);
        // 2. Back to UART @10400 (mux switchback INFERRED, see initSlow).
        Serial1.begin(KLINE_BAUD);
        flushK();
        // 3. StartCommunication: C1 33 F1 81 CS (ISO 14230-2, INFERRED).
        uint8_t sc[4] = { 0xC1, 0x33, 0xF1, 0x81 };
        uint8_t rx[FRAME_MAX]; uint8_t rxLen = 0;
        if (!transact(sc, sizeof(sc), rx, sizeof(rx), rxLen, 500)) continue;
        // Positive response: [fmt tgt src C1 KB1 KB2] (checksum stripped).
        if (rxLen < 6 || rx[3] != 0xC1) continue;
        kwpKb1 = rx[4]; kwpKb2 = rx[5];
        tgtAddr = 0x33;
        curInit = Init::FAST;
        if (dbg::enabled) {
            dbg::print(F("[kline] fast init OK, KB="));
            dbg::print((uint32_t)kwpKb1, HEX);
            dbg::print(F(" "));
            dbg::println((uint32_t)kwpKb2, HEX);
        }
        return true;
    }
    dbg::println(F("[kline] fast init FAILED"));
    return false;
}

Init initDone() { return curInit; }

bool request9141(const uint8_t hdr[3], const uint8_t* payload, uint8_t payloadLen,
                 uint8_t* rx, uint8_t rxCap, uint8_t& rxLen,
                 uint16_t timeoutMs)
{
    if (curInit == Init::NONE || hdr == nullptr) return false;
    uint8_t req[FRAME_MAX];
    if ((uint16_t)payloadLen + 3 > FRAME_MAX) return false;
    req[0] = hdr[0]; req[1] = hdr[1]; req[2] = hdr[2];
    for (uint8_t i = 0; i < payloadLen; i++) req[3 + i] = payload[i];
    return transact(req, (uint8_t)(payloadLen + 3), rx, rxCap, rxLen, timeoutMs);
}

bool kwpService(uint8_t sid, const uint8_t* data, uint8_t dataLen,
                uint8_t* rx, uint8_t rxCap, uint8_t& rxLen,
                uint16_t timeoutMs)
{
    if (curInit == Init::NONE) return false;
    uint8_t total = (uint8_t)(1 + dataLen);  // SID + data
    if (total > 63 || (uint16_t)total + 4 > FRAME_MAX) return false;
    uint8_t req[FRAME_MAX];
    req[0] = (uint8_t)(0x80 | total);  // fmt: addr present, len in low 6 bits
    req[1] = tgtAddr; req[2] = tstAddr; req[3] = sid;
    for (uint8_t i = 0; i < dataLen; i++) req[4 + i] = data[i];
    return transact(req, (uint8_t)(total + 3), rx, rxCap, rxLen, timeoutMs);
}

bool readMemoryByAddress(uint32_t addr, uint8_t len,
                         uint8_t* out, uint8_t& outLen)
{
    outLen = 0;
    if (curInit == Init::NONE || out == nullptr || len == 0) return false;
    // 0x23 ReadMemoryByAddress, ALFID 0x33 (3-byte addr, 3-byte size —
    // INFERRED common default; ECU support itself is INFERRED).
    uint8_t data[7] = {
        0x33,
        (uint8_t)((addr >> 16) & 0xFF), (uint8_t)((addr >> 8) & 0xFF),
        (uint8_t)(addr & 0xFF),
        0x00, 0x00, len,
    };
    uint8_t rx[FRAME_MAX]; uint8_t rxLen = 0;
    if (!kwpService(0x23, data, sizeof(data), rx, sizeof(rx), rxLen)) return false;
    // Positive response SID 0x63; data starts after [fmt tgt src 0x63].
    if (rxLen < 5 || rx[3] != 0x63) return false;
    uint8_t n = (uint8_t)(rxLen - 4);
    if (n > len) n = len;
    for (uint8_t i = 0; i < n; i++) out[i] = rx[4 + i];
    outLen = n;
    return true;
}

bool testerPresent()
{
    if (curInit == Init::NONE) return false;
    // 0x3E with response-suppressed subfunction (INFERRED: 0x00) — the ECU
    // stays silent; this is a keep-alive, not a transaction.
    uint8_t data[1] = { 0x00 };
    uint8_t req[6] = { 0x82, tgtAddr, tstAddr, 0x3E, data[0], 0x00 };
    req[5] = checksum(req, 5);
    return sendOnly(req, 5);
}

void setTiming(const Timing& t) { tm = t; }
void getTiming(Timing& t) { t = tm; }
void setAddrs(uint8_t target, uint8_t tester) { tgtAddr = target; tstAddr = tester; }

void getStatus(Status& s)
{
    s.acquired = isAcquired;
    s.initDone = curInit;
    s.kb1 = kwpKb1; s.kb2 = kwpKb2;
    s.target = tgtAddr; s.tester = tstAddr;
    s.timing = tm;
}

void testerPresentStart(uint32_t periodMs)
{
    testerPresentStop();
    if (periodMs == 0 || !isAcquired) return;
    txsched::Slot s;
    s.txEn = false;  // callback-only: the callback does the K-line TX
    s.periodMs = periodMs;
    s.offsetMs = periodMs;
    s.count = -1;
    s.onFire = tpFire;
    tpSlot = txsched::add(s);
    if (dbg::enabled) {
        dbg::print(F("[kline] tester-present every "));
        dbg::print(periodMs);
        dbg::println(F(" ms"));
    }
}

void testerPresentStop()
{
    if (tpSlot != 0xFF) { txsched::remove(tpSlot); tpSlot = 0xFF; }
}

void pollRx()
{
    if (!isAcquired) return;
    // Assemble frames by inter-byte gap (> 2*P1max ends a frame, INFERRED).
    static uint8_t abuf[FRAME_MAX];
    static uint8_t alen = 0;
    static uint32_t lastByte = 0;
    static bool inGap = true;
    const uint16_t gapMax = (uint16_t)(tm.p1Max * 2);
    while (Serial1.available() > 0 && alen < FRAME_MAX) {
        abuf[alen++] = (uint8_t)Serial1.read();
        lastByte = millis();
        inGap = false;
    }
    if (!inGap && alen > 0 && (uint32_t)(millis() - lastByte) >= gapMax) {
        // Frame complete: verify checksum, queue it.
        if (alen >= 2 && checksum(abuf, (uint8_t)(alen - 1)) == abuf[alen - 1]) {
            pushFrame(abuf, (uint8_t)(alen - 1));  // strip checksum
            busstat::noteRx(busstat::Bus::KLINE, alen);
        } else {
            busstat::noteDropped(busstat::Bus::KLINE);
        }
        alen = 0;
        inGap = true;
    }
    if (alen >= FRAME_MAX) {  // overflow guard
        busstat::noteRxOverflow(busstat::Bus::KLINE);
        alen = 0; inGap = true;
    }
}

bool pollFrame(uint8_t* buf, uint8_t cap, uint8_t& len)
{
    len = 0;
    if (qHead == qTail || buf == nullptr) return false;
    uint8_t t = qTail;
    uint8_t n = qLen[t];
    // Phase 6 pattern: honor the shared filter table when non-empty.
    uint32_t srcAddr = (n >= 3) ? qBuf[t][2] : busfilter::ADDR_ANY;
    if (!busfilter::accept(busfilter::Bus::KLINE, busfilter::Proto::KLINE,
                           srcAddr, 0, busfilter::Dir::RX, qBuf[t], n)) {
        busstat::noteDropped(busstat::Bus::KLINE);
        qTail = (uint8_t)((qTail + 1) % Q_DEPTH);
        return false;
    }
    len = (n <= cap) ? n : cap;
    for (uint8_t i = 0; i < len; i++) buf[i] = qBuf[t][i];
    qTail = (uint8_t)((qTail + 1) % Q_DEPTH);
    return true;
}

uint8_t queueDepth()
{
    return (uint8_t)((qHead + Q_DEPTH - qTail) % Q_DEPTH);
}

bool probe()
{
    // Phase 12 auto-detect hook — STUB. The Phase 12 auto-detection engine
    // will call this to test for K-line activity without a full init.
    // NOT implemented in Phase 7 (roadmap: hooks only).
    return false;
}

#endif // SIM_M2

} // namespace kline
