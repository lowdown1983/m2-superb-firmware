// M2 multi-mode firmware — SWCAN hardened driver implementation (Phase 8).
// See swcan.h for the hardware truth, resource claims, and the INFERRED banner.

#include "swcan.h"
#include "busfilter.h"  // Phase 6: unified filter model on the RX path
#include "busstat.h"    // Phase 6: per-bus stats (SWCAN = bus 4)
#include "debug.h"
#include <MCP2515_sw_can.h>
#ifndef SIM_M2
#include <MCP2515_defs.h>  // register addresses + MODE_* constants (vendored lib).
                           // Guarded: no shim provides this on the host sim,
                           // and every use below is already in non-SIM branches
                           // (reviewer M4).
#endif

namespace swcan {

namespace {

// The vendored Macchina object. Constructed once; the MCP2515 constructor
// does pinMode(CS, OUTPUT) + pinMode(INT, INPUT) (VERIFIED MCP2515.cpp),
// which satisfies the pack rule for SWC_INT (pin 67 = INPUT + PIO IRQ).
SWcan hw(SPI0_CS3, SWC_INT);

bool isAcquired = false;
Mode curMode = Mode::SLEEP;
uint32_t curSpeed = 33333;
bool wantListenOnly = false;

// EFLG bits (MCP2515 datasheet §REGISTER 9-1; behavior INFERRED here).
constexpr uint8_t EFLG_TXBO  = 0x80;  // bus-off
constexpr uint8_t EFLG_TXEP  = 0x40;  // TX error-passive
constexpr uint8_t EFLG_RXEP  = 0x20;  // RX error-passive
constexpr uint8_t EFLG_TXWAR = 0x10;
constexpr uint8_t EFLG_RXWAR = 0x08;
constexpr uint8_t EFLG_EWARN = 0x04;
constexpr uint8_t EFLG_RX1OVR = 0x02;
constexpr uint8_t EFLG_RX0OVR = 0x01;

// CANCTRL bits (MCP2515 datasheet §REGISTER 9-3). ABAT aborts all pending
// TX buffers — the vendored macchina/MCP2515@0.3.1 exposes no Abort API,
// so W-5 sets the bit directly (CANCTRL address via MCP2515_defs.h).
constexpr uint8_t CANCTRL_ABAT = 0x10;

// TH8056 mode pins (pack VERIFIED): M0 = pin 64, M1 = pin 65.
//   sleep      = M0 LOW,  M1 LOW
//   high-speed = M0 HIGH, M1 LOW
//   HV wake-up = M0 LOW,  M1 HIGH
//   normal     = M0 HIGH, M1 HIGH
void driveModePins(Mode m)
{
    pinMode(SWC_M0, OUTPUT);
    pinMode(SWC_M1, OUTPUT);
    switch (m) {
        case Mode::SLEEP:     digitalWrite(SWC_M0, LOW);  digitalWrite(SWC_M1, LOW);  break;
        case Mode::HIGHSPEED: digitalWrite(SWC_M0, HIGH); digitalWrite(SWC_M1, LOW);  break;
        case Mode::HV_WAKEUP: digitalWrite(SWC_M0, LOW);  digitalWrite(SWC_M1, HIGH); break;
        case Mode::NORMAL:    digitalWrite(SWC_M0, HIGH); digitalWrite(SWC_M1, HIGH); break;
    }
}

// Datasheet settling waits (TH8056 §2.5 AC table, pack VERIFIED):
//   Sleep -> Normal <= 50 ms; Normal <-> High-speed/HVWU <= 30 ms;
//   Standby -> Sleep 100-500 ms.
// The ->SLEEP wait uses the VERIFIED Standby->Sleep minimum (100 ms):
// the closest pack value for our direct Normal->Sleep transition (the
// exact mapping is INFERRED — the 4-mode table has no separate standby).
// Safe direction: never shorter than the VERIFIED minimum.
void settleFor(Mode from, Mode to)
{
    if (from == to) return;
    if (from == Mode::SLEEP) delay(50);       // Sleep -> Normal <= 50 ms
    else if (to == Mode::SLEEP) delay(100);   // Standby -> Sleep 100-500 ms (VERIFIED min)
    else delay(30);                            // Normal <-> High-speed/HVWU <= 30 ms
}

void isrTrampoline() { hw.intHandler(); }

// SPI ARBITER (Phase 8, reviewer M2): the vendored MCP2515 lib performs
// multi-transfer SPI sequences with NO interrupt guards, and the PIOC16 ISR
// runs hw.intHandler() — which itself does SPI (Read CANINTF, Write,
// ReadBuffer...). If the ISR preempts a main-loop transaction mid-sequence,
// bytes interleave: corrupted EFLG/TEC reads (phantom bus-off, spurious
// recover()), or worse, a corrupted TX buffer loaded onto the vehicle bus.
// Every main-context transaction below is therefore wrapped in
// noInterrupts()/interrupts() — this is the arbiter that
// RESOURCE_OWNERSHIP.md Phase-1 checklist item 4 requires. The ISR side
// needs no guard: it already runs with interrupts disabled, and no other
// ISR in the firmware performs SPI. delay() inside guarded sections is safe
// on SAM3X (micros()-based, no interrupt dependency).

#ifndef SIM_M2
// Health check: CANSTAT should read a sane mode value, never 0xFF
// (0xFF = MISO pulled high = SPI dead / chip absent).
// PRECONDITION: call with the SPI arbiter (noInterrupts) already held —
// noInterrupts() does not nest-count on Arduino, so this must not guard
// itself when called from guarded sections.
bool healthOk()
{
    uint8_t cs = hw.Read(CANSTAT);
    return cs != 0xFF;
}
#endif

void applyListenOnly()
{
#ifndef SIM_M2
    noInterrupts();  // SPI ARBITER
    hw.setListenOnlyMode(wantListenOnly);
    interrupts();
#else
    (void)0;  // no listen-only on the host shim (INFERRED stand-in)
#endif
    dbg::print(F("[swcan] listen-only "));
    dbg::println(wantListenOnly ? F("ON") : F("off"));
}

} // namespace

bool acquire(uint32_t speed)
{
    if (isAcquired) return false;
    dbg::print(F("[swcan] acquire @ "));
    dbg::println(speed);
#ifndef SIM_M2
    // MCP2515 init (16 MHz crystal, VERIFIED M2RET.ino:139 via setupSW).
    // Init() returns the baud on success, 0 on failure.
    // SPI ARBITER: guarded (see note above).
    noInterrupts();
    int initRes = hw.Init(speed, 16);
    bool healthy = (initRes != 0) && healthOk();
    interrupts();
    if (!healthy) {
        dbg::println(F("[swcan] MCP2515 init/health FAILED"));
        return false;
    }
#else
    hw.setupSW(speed);  // shim records the speed (INFERRED stand-in)
#endif
    curSpeed = speed;
    // Listen-only BEFORE the transceiver wakes: the MCP2515 leaves Init()
    // in MODE_NORMAL, so applying it here — while the TH8056 is still
    // asleep — is the only way the controller never sees the live bus in
    // normal mode. Same ordering as recover() below. No-op for
    // non-passive callers (wantListenOnly=false -> MODE_NORMAL, already).
    applyListenOnly();
    // TH8056 to NORMAL with the Sleep->Normal settle. driveModePins is
    // GPIO-only, so this is safe even if the SPI init above were skipped.
    driveModePins(Mode::NORMAL);
    settleFor(Mode::SLEEP, Mode::NORMAL);
    curMode = Mode::NORMAL;
    attachInterrupt(SWC_INT, isrTrampoline, FALLING);
    isAcquired = true;
    busstat::setPresent(busstat::Bus::SWCAN, true);
    dbg::println(F("[swcan] acquired, TH8056 NORMAL"));
    return true;
}

void release()
{
    // INT first (ownership rule: IRQ before transceiver), then sleep.
    detachInterrupt(SWC_INT);
#ifndef SIM_M2
    // Card quiesce rule: sleep the MCP2515 — not just the TH8056 — when the
    // controller was initialized. ABAT first so no queued TX buffer can
    // fire on a later wake (same pattern as W-5). SPI ARBITER guarded.
    // Skipped when never acquired: the GPIO fallback below covers it.
    if (isAcquired) {
        noInterrupts();
        hw.Write(CANCTRL, (uint8_t)(hw.Read(CANCTRL) | CANCTRL_ABAT));
        hw.Mode(MODE_SLEEP);
        interrupts();
    }
#endif
    Mode was = curMode;
    driveModePins(Mode::SLEEP);  // GPIO-only: safe even if never acquired
    // Reviewer M3: the sleep transition gets its settle wait too, so a
    // release->acquire cycle can't re-drive NORMAL mid-transition.
    settleFor(was, Mode::SLEEP);
    curMode = Mode::SLEEP;
    if (isAcquired) {
        isAcquired = false;
        dbg::println(F("[swcan] released, TH8056 SLEEP"));
    }
    busstat::setPresent(busstat::Bus::SWCAN, false);
}

bool acquired() { return isAcquired; }
Mode mode() { return curMode; }
uint32_t speed() { return curSpeed; }

bool setMode(Mode m)
{
    if (!isAcquired) return false;
    if (m == curMode) return true;
    // HV_WAKEUP is entered via waitWake() (it needs the wake poll); direct
    // setMode(HV_WAKEUP) is allowed but the caller owns wake detection.
    driveModePins(m);
    settleFor(curMode, m);
    dbg::print(F("[swcan] TH8056 mode -> "));
    dbg::println((uint32_t)m);
    curMode = m;
    return true;
}

bool reconfigure(uint32_t speed)
{
    if (!isAcquired) { curSpeed = speed; return true; }  // staged for later
    // Reviewer M1: SavvyCAN re-sends SET_EXT_BUSES on every connect, usually
    // with the same speed — skip the release/acquire cycle when nothing
    // changed (the caller still applies listen-only separately).
    if (speed == curSpeed) return true;
    dbg::print(F("[swcan] reconfigure @ "));
    dbg::println(speed);
    release();
    return acquire(speed);
}

bool waitWake(uint32_t timeoutMs)
{
    if (!isAcquired) return false;
    driveModePins(Mode::HV_WAKEUP);
    settleFor(curMode, Mode::HV_WAKEUP);
    curMode = Mode::HV_WAKEUP;
    dbg::println(F("[swcan] HV wake-up armed, waiting for bus activity"));
    // The tWUF 10-70 us filter is in TH8056 hardware; we poll the INT pin
    // (the MCP2515 INT fires on wake in HVWU). INFERRED until a real wake.
    uint32_t t0 = millis();
    while ((uint32_t)(millis() - t0) < timeoutMs) {
        if (digitalRead(SWC_INT) == LOW) {
            dbg::println(F("[swcan] wake detected -> NORMAL"));
            setMode(Mode::NORMAL);
            return true;
        }
    }
    dbg::println(F("[swcan] wake wait timed out"));
    setMode(Mode::NORMAL);
    return false;
}

void setListenOnly(bool lo)
{
    wantListenOnly = lo;
    if (isAcquired) applyListenOnly();
}

bool listenOnly() { return wantListenOnly; }

bool sendFrame(CAN_FRAME& frame)
{
    if (!isAcquired) { busstat::noteTxFail(busstat::Bus::SWCAN); return false; }
    noInterrupts();  // SPI ARBITER
    bool ok = hw.sendFrame(frame);
    interrupts();
    if (ok) busstat::noteTx(busstat::Bus::SWCAN, frame.length);
    else {
        busstat::noteTxFail(busstat::Bus::SWCAN);
        dbg::print(F("[swcan] TX refused, id="));
        dbg::println(frame.id, HEX);
    }
    return ok;
}

bool getFrame(CAN_FRAME& frame, uint32_t* tsUs)
{
    if (!isAcquired) return false;
    noInterrupts();  // SPI ARBITER
    bool got = hw.GetRXFrame(frame);
    interrupts();
    if (!got) return false;
    uint32_t ts = micros();  // drain-time stamp (INFERRED precision)
    if (tsUs) *tsUs = ts;
    // Phase 6 pattern: the unified filter table gates RX when non-empty.
    if (busfilter::count() > 0) {
        if (!busfilter::accept(busfilter::Bus::SWCAN, busfilter::Proto::SWCAN,
                               busfilter::ADDR_ANY, frame.id,
                               busfilter::Dir::RX,
                               frame.data.bytes, frame.length)) {
            busstat::noteDropped(busstat::Bus::SWCAN);
            return false;
        }
    }
    busstat::noteRx(busstat::Bus::SWCAN, frame.length);
    return true;
}

void poll()
{
    if (!isAcquired) return;
#ifdef SIM_M2
    return;  // no registers on the host sim
#else
    // Health: unreadable registers 3 polls in a row -> recover().
    // 0xFF on MISO = SPI dead / chip absent (INFERRED heuristic).
    static uint8_t badReads = 0;
    // SPI ARBITER: the two register reads are one guarded sequence.
    noInterrupts();
    uint8_t eflg = hw.Read(EFLG);
    uint8_t tec = hw.Read(TEC);
    interrupts();
    if (eflg == 0xFF && tec == 0xFF) {
        if (++badReads >= 3) {
            badReads = 0;
            dbg::println(F("[swcan] registers unreadable -> recover()"));
            recover();
        }
        return;
    }
    badReads = 0;
    // Bus-off: EFLG.TXBO. The MCP2515 auto-recovers after 128x11 recessive
    // bits per its datasheet; we count it, log it, and re-request normal
    // mode to make sure we come back in the mode we expect.
    if (eflg & EFLG_TXBO) {
        busstat::noteBusOff(busstat::Bus::SWCAN);
        dbg::print(F("[swcan] BUS-OFF (TEC="));
        dbg::print((uint32_t)tec);
        dbg::println(F("), re-requesting NORMAL"));
        noInterrupts();  // SPI ARBITER
        // W-5: clear stale error flags and abort any queued TX *before*
        // re-requesting NORMAL — otherwise a bus-off caused by a flooding
        // txsched slot would resume transmitting the stale queue the
        // moment the controller recovers (128x11 recessive bits later).
        hw.Write(EFLG, 0x00);
        hw.Write(CANCTRL, (uint8_t)(hw.Read(CANCTRL) | CANCTRL_ABAT));
        hw.Mode(MODE_NORMAL);
        interrupts();
    } else if (eflg & (EFLG_TXWAR | EFLG_RXWAR | EFLG_EWARN)) {
        dbg::print(F("[swcan] error-warning, EFLG="));
        dbg::println((uint32_t)eflg, HEX);
    }
    if (eflg & (EFLG_RX0OVR | EFLG_RX1OVR)) {
        busstat::noteRxOverflow(busstat::Bus::SWCAN);
        dbg::println(F("[swcan] MCP2515 RX buffer overrun"));
    }
#endif
}

bool recover()
{
    dbg::println(F("[swcan] recover: Reset + re-init"));
#ifdef SIM_M2
    return true;  // nothing to recover on the host shim
#else
    detachInterrupt(SWC_INT);
    // SPI ARBITER: reset + re-init is one guarded sequence (the ISR is
    // already detached, but the guard documents the invariant).
    noInterrupts();
    hw.Reset();
    interrupts();
    delay(10);
    noInterrupts();
    int initRes = hw.Init(curSpeed, 16);
    bool healthy = (initRes != 0) && healthOk();
    interrupts();
    if (!healthy) {
        dbg::println(F("[swcan] recover FAILED"));
        return false;
    }
    applyListenOnly();
    // Re-apply the TH8056 mode we were in (default NORMAL).
    Mode m = curMode;
    driveModePins(m);
    settleFor(Mode::SLEEP, m);
    attachInterrupt(SWC_INT, isrTrampoline, FALLING);
    dbg::println(F("[swcan] recover OK"));
    return true;
#endif
}

void setHwFilter(uint8_t filter, uint32_t id, bool ext)
{
#ifndef SIM_M2
    noInterrupts();  // SPI ARBITER
    hw.SetRXFilter(filter, id, ext);
    interrupts();
    dbg::print(F("[swcan] hw filter "));
    dbg::println((uint32_t)filter);
#else
    (void)filter; (void)id; (void)ext;
#endif
}

void setHwMask(uint8_t mask, uint32_t maskVal)
{
#ifndef SIM_M2
    noInterrupts();  // SPI ARBITER
    hw.SetRXMask(mask, maskVal);
    interrupts();
#else
    (void)mask; (void)maskVal;
#endif
}

void hsEnter()
{
    if (!isAcquired) return;
    setMode(Mode::HIGHSPEED);
}

void hsExit()
{
    if (!isAcquired) return;
    setMode(Mode::NORMAL);
}

} // namespace swcan
