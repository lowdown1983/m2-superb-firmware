// M2 multi-mode firmware — SAE J1850 implementation (Phases 2, 7, 8).
// See j1850.h for pin map, timing sources, resource allocation, and the
// INFERRED banner. Phase 8 adds: explicit RX/TX state machines, PWM
// arbitration, header decode, formal idle detection, error counters, an
// optional CRC gate on the monitor path, and Phase 6 transport wiring.
// The Phase 7 rail9141 integration (pin-50-before-49, refcounted release)
// is kept verbatim.

#include "j1850.h"
#include "rail9141.h"
#include "busfilter.h"  // Phase 6: unified filter model on the RX path
#include "busstat.h"    // Phase 6: per-bus stats (J1850 = bus 3)
#include "debug.h"

namespace j1850 {

namespace {

// ---- VPW timings (us). SOURCE: matafonoff/J1850-VPW-Arduino-Transceiver-
// Library, vehicle-tested. Names match that library's #defines.
constexpr uint16_t VPW_RX_SOF_MIN = 163, VPW_RX_SOF_MAX = 239;
constexpr uint16_t VPW_RX_EOD_MIN = 164, VPW_RX_EOD_MAX = 239;
constexpr uint16_t VPW_RX_EOF_MIN = 239;
constexpr uint16_t VPW_RX_SHORT_MIN = 34, VPW_RX_SHORT_MAX = 96;
constexpr uint16_t VPW_RX_SHORT_IGNORE = 5;
constexpr uint16_t VPW_RX_LONG_MIN = 97, VPW_RX_LONG_MAX = 163;
constexpr uint16_t VPW_TX_SHORT = 64, VPW_TX_LONG = 128;
constexpr uint16_t VPW_TX_SOF = 200, VPW_TX_EOD = 200, VPW_TX_EOF = 280;
constexpr uint16_t VPW_TX_IFS = 300;  // INFERRED (spec nominal 300us)
constexpr uint16_t VPW_ARB_TOL = 10;  // arbitration tolerance (library value)

// ---- PWM timings (us). SOURCE: SparkFun OBD-II guide for '1'/8us,
// '0'/16us, SOF/48us; retrolib BAE 5030 for fixed-bit-time/arbitration
// rule. RX windows and EOD/EOF/IFS are INFERRED symmetric windows.
constexpr uint16_t PWM_BIT_TIME = 24;          // 1/41600 baud
constexpr uint16_t PWM_TX_ONE = 8, PWM_TX_ZERO = 16, PWM_TX_SOF = 48;
constexpr uint16_t PWM_TX_EOD = 200, PWM_TX_IFS = 300;  // INFERRED
constexpr uint16_t PWM_ARB_TOL = 4;            // INFERRED arbitration tolerance
constexpr uint16_t PWM_RX_ONE_MIN = 5, PWM_RX_ONE_MAX = 12;    // INFERRED
constexpr uint16_t PWM_RX_ZERO_MIN = 12, PWM_RX_ZERO_MAX = 28; // INFERRED
constexpr uint16_t PWM_RX_SOF_MIN = 40, PWM_RX_SOF_MAX = 56;   // INFERRED
constexpr uint16_t PWM_RX_EOD_GAP = 150;  // passive gap => end of frame, INFERRED

constexpr uint8_t FRAME_MAX = 16;  // J1850 max 12 bytes + margin
constexpr uint8_t Q_DEPTH = 4;

// ---- RX state machine (Phase 8: explicit states, was an implicit
// inFrame flag). The ISR walks IDLE -> IN_FRAME -> IDLE; any out-of-window
// symbol aborts to IDLE and counts symErr.
enum class RxSt : uint8_t { IDLE, IN_FRAME };

struct IsrState {
    volatile RxSt st = RxSt::IDLE;
    volatile uint8_t bit = 0;          // bits assembled in current byte
    volatile uint8_t byteIdx = 0;
    volatile uint8_t acc = 0;
    volatile unsigned long lastEdge = 0;
    volatile uint8_t buf[FRAME_MAX];
    // queue
    volatile uint8_t qHead = 0, qTail = 0;
    volatile uint8_t qLen[Q_DEPTH];
    volatile uint8_t qBuf[Q_DEPTH][FRAME_MAX];
};

IsrState ist;
bool isAcquired = false;
Proto curProto = Proto::VPW;
uint8_t rxPin = 0;
// INFERRED-adaptive: sampled at acquire(); bus idles passive.
volatile bool rxIdleHigh = true;

Counters cnt;
uint8_t monHdrLen = 3;   // monitor-path default header length (OBD-style)
bool crcGateOn = false;

inline void countSymErr() { cnt.symErr++; }

// J1850 CRC-8. SOURCE: matafonoff library crc() (init 0xFF, poly 0x1D,
// final complement) — the standard J1850 CRC.
uint8_t crc8(const uint8_t* msg, uint8_t nbytes)
{
    uint8_t crc_reg = 0xFF, poly;
    for (uint8_t bc = 0; bc < nbytes; bc++) {
        uint8_t bit_point = 0x80;
        for (uint8_t i = 0; i < 8; i++, bit_point >>= 1) {
            if (bit_point & msg[bc]) {
                poly = (crc_reg & 0x80) ? 1 : 0x1C;
                crc_reg = ((crc_reg << 1) | 1) ^ poly;
            } else {
                poly = (crc_reg & 0x80) ? 0x1D : 0;
                crc_reg = (crc_reg << 1) ^ poly;
            }
        }
    }
    return ~crc_reg;
}

inline bool isActiveLevel(bool pinHigh) { return pinHigh != rxIdleHigh; }

void abortRxIsr()
{
    countSymErr();
    ist.st = RxSt::IDLE;
    ist.bit = 0; ist.byteIdx = 0; ist.acc = 0;
}

void pushFrameIsr(const volatile uint8_t* buf, uint8_t len)
{
    if (len == 0 || len > FRAME_MAX) return;
    uint8_t h = ist.qHead;
    uint8_t n = ist.qTail;
    uint8_t next = (uint8_t)((h + 1) % Q_DEPTH);
    if (next == n) {
        cnt.qOverflow++;  // queue full: drop new (documented policy)
        return;
    }
    for (uint8_t i = 0; i < len; i++) ist.qBuf[h][i] = buf[i];
    ist.qLen[h] = len;
    ist.qHead = next;
}

void endFrameIsr()
{
    if (ist.byteIdx > 0 || ist.bit > 0) {
        uint8_t tmp[FRAME_MAX];
        uint8_t n = ist.byteIdx;
        for (uint8_t i = 0; i < n; i++) tmp[i] = ist.buf[i];
        pushFrameIsr(tmp, n);
    }
    ist.st = RxSt::IDLE;
    ist.bit = 0; ist.byteIdx = 0; ist.acc = 0;
}

inline void shiftBit(bool one)
{
    ist.acc = (uint8_t)((ist.acc << 1) | (one ? 1 : 0));
    if (++ist.bit == 8) {
        if (ist.byteIdx < FRAME_MAX) ist.buf[ist.byteIdx++] = ist.acc;
        ist.acc = 0; ist.bit = 0;
        if (ist.byteIdx >= FRAME_MAX) endFrameIsr();
    }
}

// ---- VPW RX edge classifier as an explicit state machine ----
// IDLE: only an SOF (active 163-239us) enters IN_FRAME.
// IN_FRAME: active pulses classify as EOF / EOD / data bit (long=1,
//   short=0); passive pulses classify as data bit (short=1, long=0).
// Anything out of window -> abortRxIsr() (counts symErr).
void vpwEdge(bool newHigh, unsigned long now)
{
    unsigned long diff = now - ist.lastEdge;  // micros() wraps ~70min; fine
    ist.lastEdge = now;
    if (diff < VPW_RX_SHORT_IGNORE) { cnt.glitch++; return; }  // glitch filter

    bool endedActive = isActiveLevel(!newHigh);
    if (ist.st == RxSt::IDLE) {
        if (endedActive && diff >= VPW_RX_SOF_MIN && diff <= VPW_RX_SOF_MAX) {
            ist.st = RxSt::IN_FRAME; ist.bit = 0; ist.byteIdx = 0; ist.acc = 0;
        }
        return;
    }
    if (endedActive) {
        if (diff > VPW_RX_EOF_MIN) { endFrameIsr(); return; }       // EOF
        if (diff >= VPW_RX_EOD_MIN) { endFrameIsr(); return; }      // EOD (+IFR ignored)
        if (diff < VPW_RX_SHORT_MIN) { abortRxIsr(); return; }      // too short
        shiftBit(diff >= VPW_RX_LONG_MIN);  // active long=1, short=0
    } else {
        if (diff <= VPW_RX_SHORT_MAX) shiftBit(true);        // passive short=1
        else if (diff <= VPW_RX_LONG_MAX) shiftBit(false);   // passive long=0
        else abortRxIsr();                                    // too long
    }
}

// ---- PWM RX edge classifier as an explicit state machine ----
// IDLE: only an SOF (active 40-56us) enters IN_FRAME.
// IN_FRAME: active pulses are data bits (8us=1, 16us=0); a passive gap
// longer than PWM_RX_EOD_GAP ends the frame. Out of window -> abort.
void pwmEdge(bool newHigh, unsigned long now)
{
    unsigned long diff = now - ist.lastEdge;
    ist.lastEdge = now;
    if (diff < 3) { cnt.glitch++; return; }  // glitch filter, INFERRED
    bool endedActive = isActiveLevel(!newHigh);
    if (ist.st == RxSt::IDLE) {
        if (endedActive && diff >= PWM_RX_SOF_MIN && diff <= PWM_RX_SOF_MAX) {
            ist.st = RxSt::IN_FRAME; ist.bit = 0; ist.byteIdx = 0; ist.acc = 0;
        }
        return;
    }
    if (endedActive) {
        if (diff >= PWM_RX_ONE_MIN && diff <= PWM_RX_ONE_MAX) shiftBit(true);
        else if (diff > PWM_RX_ZERO_MIN && diff <= PWM_RX_ZERO_MAX) shiftBit(false);
        else abortRxIsr();
    } else {
        if (diff > PWM_RX_EOD_GAP) endFrameIsr();
        // otherwise: inter-bit passive time, nothing to do
    }
}

void rxIsr()
{
    unsigned long now = micros();
    bool high = digitalRead(rxPin) == HIGH;
    if (curProto == Proto::VPW) vpwEdge(high, now);
    else pwmEdge(high, now);
}

// ---- TX helpers ----
// INFERRED: pin HIGH = bus active (bench must confirm transceiver polarity).
inline void vpwDrive(bool active) { digitalWrite(J1850P_TX, active ? HIGH : LOW); }
inline void pwmDrive(bool active) {
    digitalWrite(J1850P_TX, active ? HIGH : LOW);
    digitalWrite(J1850N_TX, active ? LOW : HIGH);
}
inline bool vpwSense() { return isActiveLevel(digitalRead(J1850_VPW_RX) == HIGH); }
inline bool pwmSense() { return isActiveLevel(digitalRead(J1850_PWM_RX) == HIGH); }

// Blocking microsecond delay that keeps the value exact-ish.
inline void spinUs(unsigned long us)
{
    unsigned long t0 = micros();
    while ((unsigned long)(micros() - t0) < us) { /* spin */ }
}

// C-1: per-section IRQ masking for TX. Driven bus edges need microsecond
// determinism, so each drive+spin pair runs masked; passive arbitration
// polls run UNMASKED so CAN/UART ISRs stay live (the sense loops carry
// tolerance bands — VPW_ARB_TOL etc. — for exactly this reason).
// Worst-case continuous blackout is now one driven section: VPW SOF
// (200us) or one VPW active pulse (<=128us); PWM sections are <=48us.
// The old full-frame mask blacked out ~34ms (starved CAN RX ISRs, Bee
// UART ring overrun). micros() is IRQ-safe (PENDSTSET readback).
inline void vpwDriveSpin(bool active, unsigned long us)
{
    noInterrupts();
    vpwDrive(active);
    spinUs(us);
    interrupts();
}
inline void pwmDriveSpin(bool active, unsigned long us)
{
    noInterrupts();
    pwmDrive(active);
    spinUs(us);
    interrupts();
}

// ---- TX state machine (Phase 8: explicit phases; still blocking
// bit-bang — timing-critical sections masked per C-1, polls unmasked).
enum class TxPh : uint8_t {
    WAIT_IDLE, SOF, DATA, EOD, IFS, DONE, ARB_LOST, BUS_BUSY
};

// VPW TX with arbitration. Caller must have detached the RX ISR (we poll
// the pin directly). Driven sections are IRQ-masked (see vpwDriveSpin);
// arbitration polls run with interrupts enabled.
TxPh vpwTransmit(const uint8_t* frame, uint8_t len)
{
    // WAIT_IDLE via the shared primitive (5 ms timeout, INFERRED).
    if (!waitIdle(5)) return TxPh::BUS_BUSY;
    // SOF: active 200us.
    vpwDriveSpin(true, VPW_TX_SOF);
    // DATA: passive symbol first (arbitration window, unmasked), then the
    // masked active pulse.
    for (uint8_t bi = 0; bi < len; bi++) {
        uint8_t b = frame[bi];
        for (int8_t bit = 7; bit >= 0; bit--) {
            bool one = (b >> bit) & 1;
            uint16_t passiveUs = one ? VPW_TX_LONG : VPW_TX_SHORT;
            vpwDrive(false);
            unsigned long ps = micros();
            while ((unsigned long)(micros() - ps) < passiveUs) {
                if (vpwSense()) {
                    unsigned long remaining = passiveUs - (unsigned long)(micros() - ps);
                    if (remaining > VPW_ARB_TOL) { vpwDrive(false); return TxPh::ARB_LOST; }
                    break;  // within tolerance: resync, keep going
                }
            }
            uint16_t activeUs = one ? VPW_TX_SHORT : VPW_TX_LONG;
            vpwDriveSpin(true, activeUs);
        }
    }
    // EOD: passive 200us with collision check (matafonoff behavior).
    vpwDrive(false);
    unsigned long es = micros();
    while ((unsigned long)(micros() - es) < VPW_TX_EOD) {
        if (vpwSense()) return TxPh::ARB_LOST;
    }
    // IFS: passive 300us, no check (INFERRED).
    spinUs(VPW_TX_IFS);
    return TxPh::DONE;
}

// PWM TX with arbitration (Phase 8): differential, active = P HIGH / N LOW
// (INFERRED polarities). Arbitration rule (retrolib BAE 5030): long
// dominates short — if we transmit a '1' (8us active) and the bus stays
// active past our pulse + tolerance, another node is sending '0' and we
// lost arbitration.
TxPh pwmTransmit(const uint8_t* frame, uint8_t len)
{
    // WAIT_IDLE via the shared primitive (5 ms timeout, INFERRED).
    if (!waitIdle(5)) return TxPh::BUS_BUSY;
    // SOF: 48us active.
    pwmDriveSpin(true, PWM_TX_SOF);
    // DATA: fixed 24us bit time; active pulse 8us ('1') or 16us ('0').
    for (uint8_t bi = 0; bi < len; bi++) {
        uint8_t b = frame[bi];
        for (int8_t bit = 7; bit >= 0; bit--) {
            bool one = (b >> bit) & 1;
            uint16_t onUs = one ? PWM_TX_ONE : PWM_TX_ZERO;
            // Drive our pulse, then release BEFORE sampling: sampling
            // while still driving would read our own transceiver echo
            // (self-sensing bug — caught in Phase 8 self-review).
            // C-1: the whole drive-spin-release is one masked section —
            // an ISR between spin end and release would stretch our pulse
            // and corrupt the arbitration sample below. The sample poll
            // itself runs unmasked.
            unsigned long bitT0;
            noInterrupts();
            pwmDrive(true);
            bitT0 = micros();
            spinUs(onUs);
            pwmDrive(false);
            interrupts();
            // Arbitration sample: the bus must go passive now that our
            // pulse ended. If it stays active past onUs + tolerance, a
            // longer ('0') pulse from another node dominates -> we lost.
            // (The few us of transceiver turn-off are inside the
            // tolerance; INFERRED.)
            bool lost = false;
            while (pwmSense()) {
                unsigned long el = (unsigned long)(micros() - bitT0);
                if (el > (unsigned long)(onUs + PWM_ARB_TOL)) { lost = true; break; }
                if (el > (unsigned long)(PWM_TX_ZERO + 50)) { lost = true; break; }  // W-4: stuck bus concedes
            }
            if (lost) return TxPh::ARB_LOST;
            // Passive remainder of the fixed 24us bit time.
            unsigned long el = (unsigned long)(micros() - bitT0);
            if (el < PWM_BIT_TIME) spinUs(PWM_BIT_TIME - el);
        }
    }
    // EOD + IFS (INFERRED). Bus is already passive from the last bit's
    // release; the redundant drive edge needs no mask.
    pwmDrive(false); spinUs(PWM_TX_EOD);
    spinUs(PWM_TX_IFS);
    return TxPh::DONE;
}

} // namespace

const Counters& counters() { return cnt; }
void resetCounters() { cnt = Counters(); }

bool decodeHeader(const uint8_t* frame, uint8_t n, uint8_t hdrLen, Header& h)
{
    h = Header();
    if ((hdrLen != 1 && hdrLen != 3) || frame == nullptr) return false;
    if (n < hdrLen + 1) return false;  // header + at least CRC
    h.len = hdrLen;
    h.h0 = frame[0];
    if (hdrLen == 3) {
        h.h1 = frame[1]; h.h2 = frame[2];
        h.priority = h.h0;      // OBD-style: raw priority byte (e.g. 0x68/0x48)
        h.target = h.h1;
        h.source = h.h2;        // the Phase 6 OEM-seam node byte
    } else {
        h.priority = (uint8_t)(h.h0 >> 5);  // INFERRED: bits 7-5
        h.source = h.h0;
    }
    return true;
}

void setHeaderLen(uint8_t n) { if (n == 1 || n == 3) monHdrLen = n; }
uint8_t headerLen() { return monHdrLen; }
void setCrcGate(bool on) { crcGateOn = on; }
bool crcGate() { return crcGateOn; }

bool acquire(Proto p)
{
    if (isAcquired) return false;
    curProto = p;
    // 1. Mode select FIRST (INV-MODE50 / INV-PWR49, pack v1.4.0: pin 50
    //    settled before pin 49 HIGH; pin 50 never moves under traffic).
    //    Phase 7: this also fixes the old order (rail before mode select).
    //    Phase 8: kept verbatim.
    pinMode(J1850_PWM_VPW, OUTPUT);
    digitalWrite(J1850_PWM_VPW, (p == Proto::JPWM) ? HIGH : LOW);
    // 2. Shared 'J1850 9141 ON' rail via rail9141 (Phase 7: refcounted with
    //    the K-line driver; Macchina Docs.md J1850 snippet: HIGH = power).
    rail9141::powerUp(rail9141::Owner::J1850);
    // 3. TX pins to idle-passive.
    pinMode(J1850P_TX, OUTPUT);
    pinMode(J1850N_TX, OUTPUT);
    if (p == Proto::JPWM) { digitalWrite(J1850P_TX, LOW); digitalWrite(J1850N_TX, HIGH); }
    else { digitalWrite(J1850P_TX, LOW); digitalWrite(J1850N_TX, LOW); }
    delay(50);  // INFERRED rail/transceiver settle time
    // 4. RX pin + idle sampling (INFERRED-adaptive polarity).
    rxPin = (p == Proto::JPWM) ? J1850_PWM_RX : J1850_VPW_RX;
    pinMode(rxPin, INPUT);
    rxIdleHigh = (digitalRead(rxPin) == HIGH);
    ist.st = RxSt::IDLE; ist.qHead = 0; ist.qTail = 0;
    ist.lastEdge = micros();
    attachInterrupt(digitalPinToInterrupt(rxPin), rxIsr, CHANGE);
    isAcquired = true;
    busstat::setPresent(busstat::Bus::J1850, true);
    dbg::print(F("[j1850] acquired "));
    dbg::println(p == Proto::JPWM ? F("PWM") : F("VPW"));
    return true;
}

void release()
{
    if (isAcquired) {
        detachInterrupt(digitalPinToInterrupt(rxPin));
        isAcquired = false;
        dbg::println(F("[j1850] released"));
    }
    // TX pins to safe idle (both LOW).
    // Rail: refcounted release — pin 49 goes LOW only when the K-line
    // driver isn't holding it (INV-PWR49, Phase 7). Idempotent: safe to
    // call when never acquired.
    // Audit fix F-3 (2026-10-08): pin 50 (mode select) is only driven
    // after the rail is confirmed down. Changing it while K-line still
    // holds pin 49 HIGH would flip J1850's mode-select on a live rail;
    // re-acquire sets pin 50 before powerUp (INV-MODE50), so leaving it
    // untouched here is safe.
    digitalWrite(J1850P_TX, LOW);
    digitalWrite(J1850N_TX, LOW);
    rail9141::powerDown(rail9141::Owner::J1850);
    if (!rail9141::powered()) {
        digitalWrite(J1850_PWM_VPW, LOW);  // VPW default, rail confirmed off
    }
    busstat::setPresent(busstat::Bus::J1850, false);
}

bool acquired() { return isAcquired; }
Proto proto() { return curProto; }

bool idle()
{
    if (!isAcquired) return false;
    // Instantaneous: the pin currently reads passive. (A stretched
    // passive read is the caller's waitIdle business.)
    bool high = digitalRead(rxPin) == HIGH;
    return !isActiveLevel(high);
}

bool waitIdle(uint32_t timeoutMs)
{
    if (!isAcquired) return false;
    // Require a continuous passive stretch of one EOF (VPW) / EOD gap
    // (PWM) — the same thresholds the TX state machine uses.
    uint16_t needUs = (curProto == Proto::VPW) ? VPW_TX_EOF : PWM_RX_EOD_GAP;
    uint32_t t0 = millis();
    unsigned long passiveStart = micros();
    while ((uint32_t)(millis() - t0) < timeoutMs) {
        if (!idle()) {
            passiveStart = micros();
        } else if ((unsigned long)(micros() - passiveStart) >= needUs) {
            return true;
        }
    }
    return false;
}

// Drain one raw frame from the ISR queue (no filter/gate). Returns false
// when empty.
static bool drainRaw(uint8_t* buf, uint8_t cap, uint8_t& len)
{
    len = 0;
    noInterrupts();
    if (ist.qHead == ist.qTail) { interrupts(); return false; }
    uint8_t t = ist.qTail;
    uint8_t n = ist.qLen[t];
    interrupts();
    len = (n <= cap) ? n : cap;
    for (uint8_t i = 0; i < len; i++) buf[i] = ist.qBuf[t][i];
    noInterrupts();
    ist.qTail = (uint8_t)((ist.qTail + 1) % Q_DEPTH);
    interrupts();
    return true;
}

// Shared filter check for the RX paths (Phase 6 pattern): true when the
// table is empty or the frame passes. Fills hdrOut when provided.
static bool passesFilter(const uint8_t* raw, uint8_t n, uint8_t hdrLen,
                         Header* hdrOut)
{
    Header h;
    uint32_t addr = busfilter::ADDR_ANY;
    uint32_t id = raw[0];  // Filter.id: J1850 header byte in low 8
    if (decodeHeader(raw, n, hdrLen, h)) {
        addr = h.source;  // the Phase 6 OEM-seam node byte
        if (hdrOut) *hdrOut = h;
    }
    if (busfilter::count() == 0) return true;
    busfilter::Proto pr = (curProto == Proto::JPWM)
        ? busfilter::Proto::J1850_PWM : busfilter::Proto::J1850_VPW;
    if (!busfilter::accept(busfilter::Bus::J1850, pr, addr, id,
                           busfilter::Dir::RX, raw, n)) {
        busstat::noteDropped(busstat::Bus::J1850);
        return false;
    }
    return true;
}

// Apply the Phase 8 monitor pipeline to a raw frame: CRC gate (optional),
// then the shared filter check. Returns true when the frame survives.
static bool monitorPipeline(const uint8_t* raw, uint8_t n)
{
    // A sub-2-byte "frame" is malformed ISR output (pushFrameIsr already
    // rejects len==0); count it rather than dropping it silently.
    if (n < 2) { cnt.symErr++; return false; }
    // CRC gate (optional, default off).
    if (crcGateOn) {
        if (crc8(raw, (uint8_t)(n - 1)) != raw[n - 1]) {
            cnt.crcErr++;
            return false;
        }
    }
    return passesFilter(raw, n, monHdrLen, nullptr);
}

bool transact(const uint8_t* tx, uint8_t txLen,
              uint8_t* rx, uint8_t rxCap, uint8_t& rxLen,
              uint16_t timeoutMs, uint8_t hdrLen)
{
    rxLen = 0;
    if (!isAcquired || txLen == 0 || txLen + 1 > FRAME_MAX) return false;
    if (hdrLen == 0) hdrLen = monHdrLen;
    // Clear stale frames.
    noInterrupts(); ist.qHead = 0; ist.qTail = 0; interrupts();

    uint8_t frame[FRAME_MAX];
    for (uint8_t i = 0; i < txLen; i++) frame[i] = tx[i];
    frame[txLen] = crc8(tx, txLen);

    // Bit-bang with per-section IRQ masking (C-1): driven edges run masked
    // for microsecond determinism, arbitration polls run unmasked so CAN
    // RX and Bee UART ISRs stay live. Worst-case continuous blackout is
    // one driven section (VPW SOF 200us) — never the ~34ms full frame the
    // old global mask held. RX ISR detached so our own TX isn't captured.
    detachInterrupt(digitalPinToInterrupt(rxPin));
    TxPh res;
    if (curProto == Proto::VPW) res = vpwTransmit(frame, (uint8_t)(txLen + 1));
    else res = pwmTransmit(frame, (uint8_t)(txLen + 1));
    ist.lastEdge = micros(); ist.st = RxSt::IDLE;
    attachInterrupt(digitalPinToInterrupt(rxPin), rxIsr, CHANGE);
    if (res == TxPh::ARB_LOST) {
        cnt.arbLost++;
        busstat::noteTxFail(busstat::Bus::J1850);
        dbg::println(F("[j1850] arbitration lost"));
        return false;
    }
    if (res != TxPh::DONE) {
        busstat::noteTxFail(busstat::Bus::J1850);
        dbg::println(F("[j1850] TX failed (bus busy)"));
        return false;
    }
    busstat::noteTx(busstat::Bus::J1850, (uint8_t)(txLen + 1));

    // Wait for a CRC-valid response.
    uint32_t start = millis();
    uint8_t tmp[FRAME_MAX]; uint8_t n;
    while ((uint32_t)(millis() - start) < timeoutMs) {
        if (drainRaw(tmp, sizeof(tmp), n)) {
            if (n >= 2 && crc8(tmp, (uint8_t)(n - 1)) == tmp[n - 1]) {
                // Phase 8: the response path honors the filter table too
                // (Phase 6 pattern, same as isotp). A filtered response
                // keeps waiting within the timeout.
                if (!passesFilter(tmp, n, hdrLen, nullptr)) continue;
                rxLen = (n - 1 <= rxCap) ? (uint8_t)(n - 1) : rxCap;
                for (uint8_t i = 0; i < rxLen; i++) rx[i] = tmp[i];
                busstat::noteRx(busstat::Bus::J1850, rxLen);
                return true;
            }
            cnt.crcErr++;  // CRC failure: keep waiting within timeout
            dbg::println(F("[j1850] response CRC fail, waiting"));
        }
    }
    return false;
}

bool pollFrame(uint8_t* buf, uint8_t cap, uint8_t& len)
{
    len = 0;
    uint8_t raw[FRAME_MAX]; uint8_t n;
    // Drain until a frame survives the monitor pipeline (or queue empty).
    while (drainRaw(raw, sizeof(raw), n)) {
        if (monitorPipeline(raw, n)) {
            len = (n <= cap) ? n : cap;
            for (uint8_t i = 0; i < len; i++) buf[i] = raw[i];
            busstat::noteRx(busstat::Bus::J1850, len);
            return true;
        }
        // dropped by CRC gate or filter — already counted; keep draining
    }
    return false;
}

uint8_t queueDepth()
{
    noInterrupts();
    uint8_t d = (uint8_t)((ist.qHead + Q_DEPTH - ist.qTail) % Q_DEPTH);
    interrupts();
    return d;
}

} // namespace j1850
