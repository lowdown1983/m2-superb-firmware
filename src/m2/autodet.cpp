// M2 multi-mode firmware — auto-detection engine (Phase 12).
// See autodet.h for the safety-critical design.

#include "autodet.h"
#include "capgate.h"
#include "busstat.h"
#include "j1850.h"
#include "swcan.h"
#include "kline.h"
#include "debug.h"
#include <due_can.h>
#include <string.h>

namespace autodet {
namespace {

Result last;
bool running = false;
volatile bool cancelFlag = false;

const uint32_t CAN_BAUDS[] = { 125000, 250000, 500000, 1000000 };
constexpr uint8_t N_BAUDS = 4;
constexpr uint16_t LISTEN_MS = 350;  // per baud/format dwell (INFERRED)

// Known diagnostic arbitration IDs (generic SAE row; OEM-specific rows
// come from oem::DIAG_ACCESS in a later refinement).
bool isKnownId(uint32_t id, bool ext)
{
    if (ext) return false;  // INFERRED: 29-bit diag IDs are OEM-specific
    if (id == 0x7DF) return true;
    if (id >= 0x7E8 && id <= 0x7EF) return true;
    return false;
}

// One listen-only CAN sweep step. Returns frames seen.
uint32_t canListen(uint32_t baud, bool ext, uint32_t& knownHits)
{
    // Pin-safety: 255 = no transceiver-enable pin driving (invariant).
    if (!Can0.begin(baud, 255)) return 0;
    Can0.setListenOnlyMode(true);  // due_can 2.1.6: silent monitoring
    for (int f = 0; f < 7; f++) Can0.setRXFilter(f, 0, 0, ext);
    uint32_t t0 = millis();
    uint32_t frames = 0;
    knownHits = 0;
    CAN_FRAME fr;
    while (millis() - t0 < LISTEN_MS && !cancelFlag) {
        if (Can0.available() > 0) {
            Can0.read(fr);
            if ((bool)fr.extended == ext) {
                frames++;
                if (isKnownId(fr.id, ext)) knownHits++;
            }
        }
    }
    Can0.setListenOnlyMode(false);
    Can0.disable();  // pin-safe: begun with 255 above
    return frames;
}

// Passive J1850 listen: power the block (rail9141 via acquire), count
// valid frames via the monitor path. NO transmit.
uint32_t j1850Listen(bool pwm, uint32_t ms)
{
    if (!j1850::acquire(pwm ? j1850::Proto::JPWM : j1850::Proto::VPW))
        return 0;
    uint32_t t0 = millis();
    uint32_t frames = 0;
    uint8_t buf[16];
    uint8_t len = 0;
    while (millis() - t0 < ms && !cancelFlag) {
        if (j1850::pollFrame(buf, sizeof(buf), len)) frames++;
    }
    j1850::release();
    return frames;
}

// Passive SWCAN listen at 33.33k. The MCP2515 is held in listen-only mode
// for the whole window (no ACKs, no transmits — genuinely passive): the
// flag is set BEFORE acquire() so the controller never participates in
// the live bus in normal mode, and restored AFTER release() on every
// exit path so later SWCAN users (M2RET, gateway) are unaffected.
uint32_t swcanListen(uint32_t ms)
{
    swcan::setListenOnly(true);
    if (!swcan::acquire(33333)) { swcan::setListenOnly(false); return 0; }
    uint32_t t0 = millis();
    uint32_t frames = 0;
    CAN_FRAME fr;
    uint32_t ts = 0;
    while (millis() - t0 < ms && !cancelFlag) {
        swcan::poll();
        if (swcan::getFrame(fr, &ts)) frames++;
    }
    swcan::release();
    swcan::setListenOnly(false);
    return frames;
}

void setNote(const char* s)
{
    strncpy(last.note, s, sizeof(last.note) - 1);
    last.note[sizeof(last.note) - 1] = '\0';
}

} // namespace (anonymous)

bool start(bool allowTx)
{
    if (running) return false;
    // GATE 1: the whole engine requires ALLOW_TRANSMIT. The CAN stage is
    // listen-only, but the engine as a whole may transmit (K-line stage),
    // so the gate applies up front — no silent partial runs.
    if (!capgate::require(capgate::Grant::ALLOW_TRANSMIT,
                          "auto-detect engine")) {
        return false;
    }
    running = true;
    cancelFlag = false;
    memset(&last, 0, sizeof(last));

    dbg::println(F("[autodet] start"));

    // ---- Stage 1: CAN listen-only sweep ----
    uint32_t bestFrames = 0, bestKnown = 0, bestBaud = 0;
    bool bestExt = false;
    for (uint8_t b = 0; b < N_BAUDS && !cancelFlag; b++) {
        for (uint8_t e = 0; e < 2 && !cancelFlag; e++) {
            uint32_t known = 0;
            uint32_t n = canListen(CAN_BAUDS[b], e != 0, known);
            dbg::print(F("[autodet] CAN "));
            dbg::print(CAN_BAUDS[b]);
            dbg::print(e ? F(" 29b: ") : F(" 11b: "));
            dbg::print(n);
            dbg::println(F(" frames"));
            // Prefer known diagnostic IDs, then raw frame count.
            if (known > bestKnown ||
                (known == bestKnown && n > bestFrames)) {
                bestKnown = known;
                bestFrames = n;
                bestBaud = CAN_BAUDS[b];
                bestExt = (e != 0);
            }
        }
    }
    if (bestFrames > 0) {
        last.found = true;
        last.proto = bestExt ? PROTO_CAN_29 : PROTO_CAN_11;
        last.baud = bestBaud;
        last.bus = 0;
        last.framesSeen = bestFrames;
        // Confidence: known IDs >> raw chatter.
        last.confidence = bestKnown > 0 ? 90
            : (bestFrames > 20 ? 60 : 30);
        char nb[48];
        snprintf(nb, sizeof(nb), "CAN %lu %s (%lu frames%lu known)",
                 (unsigned long)bestBaud, bestExt ? "29b" : "11b",
                 (unsigned long)bestFrames, (unsigned long)bestKnown);
        setNote(nb);
        last.done = true;
        running = false;
        return true;
    }

    // ---- Stage 2: K-line (TRANSMITS by construction) ----
    if (allowTx) {
        // kline::probe() is the Phase 7 stub (returns false) — the stage
        // exists in the ordering, but there is no implementation to call
        // beyond the stub. When a real probe lands, it runs here, inside
        // the capgate + explicit-allowTx gates established above.
        dbg::println(F("[autodet] K-line probe (stub): no result"));
        (void)kline::probe();
    } else {
        dbg::println(F("[autodet] K-line stage skipped (no TX consent)"));
    }
    if (cancelFlag) goto cancelled;

    // ---- Stage 3: J1850 passive listen (VPW then PWM) ----
    {
        uint32_t n = j1850Listen(false, LISTEN_MS);
        dbg::print(F("[autodet] J1850 VPW passive: "));
        dbg::print(n);
        dbg::println(F(" frames"));
        if (n > 0) {
            last.found = true;
            last.proto = PROTO_J1850_VPW;
            last.framesSeen = n;
            last.confidence = 70;
            setNote("J1850 VPW (passive listen)");
            last.done = true;
            running = false;
            return true;
        }
    }
    if (cancelFlag) goto cancelled;
    {
        uint32_t n = j1850Listen(true, LISTEN_MS);
        dbg::print(F("[autodet] J1850 PWM passive: "));
        dbg::print(n);
        dbg::println(F(" frames"));
        if (n > 0) {
            last.found = true;
            last.proto = PROTO_J1850_PWM;
            last.framesSeen = n;
            last.confidence = 70;
            setNote("J1850 PWM (passive listen)");
            last.done = true;
            running = false;
            return true;
        }
    }
    if (cancelFlag) goto cancelled;

    // ---- Stage 4: SWCAN passive listen ----
    {
        uint32_t n = swcanListen(LISTEN_MS * 2);  // 33.33k is slow
        dbg::print(F("[autodet] SWCAN passive: "));
        dbg::print(n);
        dbg::println(F(" frames"));
        if (n > 0) {
            last.found = true;
            last.proto = PROTO_SWCAN;
            last.framesSeen = n;
            last.confidence = 70;
            setNote("SWCAN normal (passive listen)");
        } else {
            setNote("no protocol detected on any stage");
        }
    }

    last.done = true;
    running = false;
    return true;

cancelled:
    setNote("cancelled by user");
    last.done = true;
    running = false;
    return true;
}

void cancel() { cancelFlag = true; }

const Result& lastResult() { return last; }

void print(const Result& r)
{
    SerialUSB.println(F("--- auto-detect ---"));
    if (!r.done) {
        SerialUSB.println(F("no run yet"));
        return;
    }
    SerialUSB.print(F("result: "));
    SerialUSB.println(r.found ? F("FOUND") : F("NOT FOUND"));
    if (r.found) {
        const char* pn = "?";
        switch (r.proto) {
            case PROTO_CAN_11: pn = "CAN 11-bit"; break;
            case PROTO_CAN_29: pn = "CAN 29-bit"; break;
            case PROTO_KLINE: pn = "K-line"; break;
            case PROTO_J1850_VPW: pn = "J1850 VPW"; break;
            case PROTO_J1850_PWM: pn = "J1850 PWM"; break;
            case PROTO_SWCAN: pn = "SWCAN"; break;
            default: break;
        }
        SerialUSB.print(F("protocol: "));
        SerialUSB.println(pn);
        if (r.baud) {
            SerialUSB.print(F("baud: "));
            SerialUSB.println(r.baud);
        }
        SerialUSB.print(F("confidence: "));
        SerialUSB.print(r.confidence);
        SerialUSB.println(F("%"));
    }
    SerialUSB.print(F("detail: "));
    SerialUSB.println(r.note);
}

} // namespace autodet
