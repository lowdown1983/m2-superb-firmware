// M2 multi-mode firmware — OBD2 poller implementation (Phase 3).
// See mode_obd2.h for the contract. PID formulas: SAE J1979 (public).
// DTC text: SAE J2012 (public). Behavior without a bench: INFERRED.

#include "mode_obd2.h"
#include "mode_manager.h"  // modes::canBegin0/1, canEnd0/1
#include "mm_channel.h"
#include "isotp.h"
#include "txsched.h"  // Phase 6: the scheduler IS the poll engine
#include "pidreg.h"   // Phase 9: data-driven PID registry
#include "capgate.h"  // Phase 9: capability gates
#include "discover.h" // Phase 9: vehicle discovery engine
#include <due_can.h>

namespace obd2 {

namespace {

// ---- config (staged) ----
uint8_t busSel = 0;
uint32_t baud = 500000;
uint16_t intervalMs = 1000;
// Phase 9: the poll list is (service, PID) pairs. Service 01 default.
uint8_t pids[16] = { 0x0C, 0x0D, 0x05, 0x0F, 0x11, 0x42 };  // RPM, speed, ECT, IAT, TPS, volts
uint8_t pidsvc[16] = { 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 };
uint8_t nPids = 6;

// ---- runtime ----
bool active = false;
bool running = false;
bool busUp = false;
bool protoLocked = false;
uint32_t detectBaud = 500000;
uint32_t lastDetectAt = 0;
constexpr uint32_t DETECT_RETRY_MS = 5000;  // INFERRED

// Phase 6: the TX scheduler is the poll engine. The sweep is registered as
// a callback-only slot (no CAN frame sent by the scheduler itself — the
// callback does the request/response sweep via isotp, exactly as before).
uint8_t sweepSlot = 0xFF;  // 0xFF = no slot

// mm-channel divert state (same pattern as ELM/UDS).
bool mmBinActive = false;
bool sawF1 = false;

bool validBaud(uint32_t b)
{
    return b == 100000 || b == 125000 || b == 250000 || b == 500000 ||
           b == 800000 || b == 1000000;
}

void busRelease()
{
    if (!busUp) return;
    if (busSel == 0) modes::canEnd0(); else modes::canEnd1();
    busUp = false;
    protoLocked = false;
}

bool busAcquire(uint32_t b)
{
    if (busSel == 0) modes::canBegin0(b);
    else modes::canBegin1(b);
    CANRaw* bp = (busSel == 0) ? &Can0 : &Can1;
    for (int m = 0; m < 7; m++) bp->setRXFilter(m, 0, 0, (m < 3));
    busUp = true;
    return true;
}

isotp::Endpoint endpoint()
{
    isotp::Endpoint ep;
    ep.bus = (busSel == 0) ? &Can0 : &Can1;
    ep.extended = false;
    ep.txId = 0x7DF;     // functional request (SAE J1979)
    ep.rxAnyId = 0;      // any responder (0x7E8..)
    return ep;
}

// Phase 9: the decodePid() switch is GONE — decoding is data-driven via
// the pidreg registry (SAE J1979 formulas). fmtU/fmt1 removed with it.

void printHex2S(uint8_t b)  // to Serial (Bee link)
{
    const char* H = "0123456789ABCDEF";
    Serial.write((uint8_t)H[(b >> 4) & 0xF]);
    Serial.write((uint8_t)H[b & 0xF]);
}

// Emit one CSV line on the Bee link: ts_ms,<svcHex>,<pidHex>,<value>
void csvLine(uint8_t svc, uint8_t pid, const char* value)
{
    Serial.print(millis());
    Serial.print(F(","));
    printHex2S(svc);
    Serial.print(F(","));
    printHex2S(pid);
    Serial.print(F(","));
    Serial.println(value);
}

// Poll one (service, PID); returns true if an ECU answered.
bool pollPid(uint8_t svc, uint8_t pid)
{
    uint8_t req[2] = { svc, pid };
    isotp::Response r;
    uint8_t n = isotp::requestMulti(endpoint(), req, 2, &r, 1, 1000, 0);
    if (n == 0) return false;
    // Expect (0x40+svc) pid ...; tolerate extra bytes.
    if (r.len < 2 || r.data[0] != (uint8_t)(0x40 + svc) || r.data[1] != pid)
        return false;
    char val[48];
    // Phase 9: data-driven decode via the PID registry (replaces the
    // decodePid() switch). Unknown (svc,pid) -> raw hex fallback.
    const pidreg::PidEntry* e = pidreg::find(svc, pid);
    if (e == nullptr || !pidreg::decode(e, r.data + 2, (uint8_t)(r.len - 2),
                                        val, sizeof(val))) {
        size_t p = 0;
        for (uint8_t i = 2; i < r.len && p + 2 < sizeof(val); i++) {
            static const char* H = "0123456789ABCDEF";
            val[p++] = H[(r.data[i] >> 4) & 0xF];
            val[p++] = H[r.data[i] & 0xF];
        }
        val[p] = '\0';
    }
    csvLine(svc, pid, val);
    return true;
}

bool detectOnce()
{
    // Try 0100 at detectBaud; alternate baud on failure.
    busRelease();
    busAcquire(detectBaud);
    uint8_t req[2] = { 0x01, 0x00 };
    isotp::Response r;
    uint8_t n = isotp::requestMulti(endpoint(), req, 2, &r, 1, 800, 0);
    if (n > 0 && r.len >= 2 && r.data[0] == 0x41 && r.data[1] == 0x00) {
        protoLocked = true;
        return true;
    }
    detectBaud = (detectBaud == 500000) ? 250000 : 500000;
    return false;
}

void sweep()
{
    for (uint8_t i = 0; i < nPids; i++) {
        pollPid(pidsvc[i], pids[i]);
        // Keep the mm divert alive between requests (blocking TX otherwise
        // starves the control channel).
        while (Serial.available() > 0) {
            uint8_t c = (uint8_t)Serial.read();
            if (mmBinActive) { if (mm::feedByte(c)) mmBinActive = false; continue; }
            if (sawF1) {
                sawF1 = false;
                if (c == mm::CMD_MULTIMODE) { mm::reset(); mmBinActive = true; continue; }
                if (c == mm::SYNC) { sawF1 = true; continue; }
            } else if (c == mm::SYNC) { sawF1 = true; continue; }
        }
    }
}

// Phase 6: the scheduler is the poll engine. sweepTick matches the
// txsched::Slot::onFire signature; the slot itself is callback-only
// (txEn=false) — the callback does the request/response sweep via isotp,
// exactly as the old interval timer did.
void sweepTick(uint8_t) { sweep(); }

void slotRemove()
{
    if (sweepSlot != 0xFF) { txsched::remove(sweepSlot); sweepSlot = 0xFF; }
}

void slotArm()
{
    slotRemove();
    txsched::Slot s;
    s.txEn = false;          // callback-only: the poll-engine tick
    s.bus = 0;               // unused for callback-only slots
    s.periodMs = intervalMs;
    s.offsetMs = 0;          // sweep immediately (like the old lastSweepAt=0)
    s.count = -1;            // infinite
    s.onFire = sweepTick;
    sweepSlot = txsched::add(s);
}

void startPoller()
{
    slotRemove();
    running = true; protoLocked = false; detectBaud = 500000; lastDetectAt = 0;
}

void stopPoller()
{
    slotRemove();
    running = false;
}

// SAE J2012 DTC text: 2 bytes -> [PCBU][0-3][0-9A-F][0-9A-F][0-9A-F]
void dtcText(uint8_t hi, uint8_t lo, char* out)
{
    static const char T[] = { 'P', 'C', 'B', 'U' };
    static const char H[] = "0123456789ABCDEF";
    out[0] = T[(hi >> 6) & 0x03];
    out[1] = (char)('0' + ((hi >> 4) & 0x03));
    out[2] = H[hi & 0x0F];
    out[3] = H[(lo >> 4) & 0x0F];
    out[4] = H[lo & 0x0F];
    out[5] = '\0';
}

// Phase 9: DTC scan generalized over service 03 (stored), 07 (pending),
// 0A (permanent). SAE J2012 DTC text.
void dtcScan(uint8_t svc)
{
    if (!active) {
        SerialUSB.println(F("OBD2 mode is not active."));
        return;
    }
    if (!busUp) { busAcquire(baud); }  // lazy (ownership rule)
    uint8_t req[1] = { svc };
    isotp::Response rs[4];
    uint8_t n = isotp::requestMulti(endpoint(), req, 1, rs, 4, 1500, 200);
    if (n == 0) { SerialUSB.println(F("NO DATA")); return; }
    uint8_t wantSid = (uint8_t)(0x40 + svc);
    for (uint8_t i = 0; i < n; i++) {
        const isotp::Response& r = rs[i];
        if (r.len < 1 || r.data[0] != wantSid) continue;
        SerialUSB.print(F("DTCs from 0x"));
        SerialUSB.print(String(r.senderId, HEX));
        SerialUSB.println(F(":"));
        uint8_t cnt = (uint8_t)((r.len - 1) / 2);
        if (cnt == 0) SerialUSB.println(F("  (none)"));
        for (uint8_t k = 0; k < cnt; k++) {
            char t[6];
            dtcText(r.data[1 + 2 * k], r.data[2 + 2 * k], t);
            SerialUSB.print(F("  ")); SerialUSB.println(t);
        }
    }
}

// Phase 9: service 04 (clear DTCs) — state-changing, needs the
// ALLOW_DIAGNOSTIC_WRITE grant.
void dtcClear()
{
    if (!active) { SerialUSB.println(F("OBD2 mode is not active.")); return; }
    if (!capgate::require(capgate::Grant::ALLOW_DIAGNOSTIC_WRITE, "OBD2 clear DTCs"))
        return;
    if (!busUp) { busAcquire(baud); }
    uint8_t req[1] = { 0x04 };
    isotp::Response r;
    uint8_t n = isotp::requestMulti(endpoint(), req, 1, &r, 1, 1000, 0);
    if (n > 0 && r.len >= 1 && r.data[0] == 0x44)
        SerialUSB.println(F("Clear accepted."));
    else
        SerialUSB.println(F("NO DATA"));
}

// Phase 9: service 02 freeze frame: 02 <pid> <frame#>.
void freezeFrame(uint8_t pid, uint8_t frame)
{
    if (!active) { SerialUSB.println(F("OBD2 mode is not active.")); return; }
    if (!busUp) { busAcquire(baud); }
    uint8_t req[3] = { 0x02, pid, frame };
    isotp::Response r;
    uint8_t n = isotp::requestMulti(endpoint(), req, 3, &r, 1, 1000, 0);
    if (n == 0 || r.len < 3 || r.data[0] != 0x42 || r.data[1] != pid) {
        SerialUSB.println(F("NO DATA"));
        return;
    }
    char val[48];
    const pidreg::PidEntry* e = pidreg::find(0x02, pid);  // falls back to 01
    if (e == nullptr || !pidreg::decode(e, r.data + 3, (uint8_t)(r.len - 3),
                                        val, sizeof(val)))
        snprintf(val, sizeof(val), "(no decoder)");
    SerialUSB.print(F("FF "));
    if (pid < 0x10) SerialUSB.print('0');
    SerialUSB.print(String(pid, HEX));
    SerialUSB.print(F(" frame ")); SerialUSB.print(frame);
    SerialUSB.print(F(": ")); SerialUSB.println(val);
}

// Phase 9: service 08 (control on-board system) — state-changing, gated.
void obdControl(uint8_t tid)
{
    if (!active) { SerialUSB.println(F("OBD2 mode is not active.")); return; }
    if (!capgate::require(capgate::Grant::ALLOW_DIAGNOSTIC_WRITE, "OBD2 service 08"))
        return;
    if (!busUp) { busAcquire(baud); }
    uint8_t req[2] = { 0x08, tid };
    isotp::Response r;
    uint8_t n = isotp::requestMulti(endpoint(), req, 2, &r, 1, 1000, 0);
    if (n > 0 && r.len >= 2 && r.data[0] == 0x48 && r.data[1] == tid)
        SerialUSB.println(F("Control accepted."));
    else
        SerialUSB.println(F("NO DATA"));
}

// Phase 9: service 09 info PIDs via the registry (0902 = VIN).
void vehInfo(uint8_t pid)
{
    if (!active) { SerialUSB.println(F("OBD2 mode is not active.")); return; }
    if (!busUp) { busAcquire(baud); }
    uint8_t req[2] = { 0x09, pid };
    isotp::Response r;
    uint8_t n = isotp::requestMulti(endpoint(), req, 2, &r, 1, 1500, 200);
    if (n == 0 || r.len < 2 || r.data[0] != 0x49 || r.data[1] != pid) {
        SerialUSB.println(F("NO DATA"));
        return;
    }
    char val[64];
    const pidreg::PidEntry* e = pidreg::find(0x09, pid);
    if (e == nullptr || !pidreg::decode(e, r.data + 2, (uint8_t)(r.len - 2),
                                        val, sizeof(val)))
        snprintf(val, sizeof(val), "(no decoder)");
    SerialUSB.print(e ? e->name : "info");
    SerialUSB.print(F(": ")); SerialUSB.println(val);
}

bool hexByte(const char* t, uint8_t& out)
{
    if (t[0] == '\0' || t[2] != '\0') return false;
    char* e; long v = strtol(t, &e, 16);
    if (*e != '\0' || v < 0 || v > 0xFF) return false;
    out = (uint8_t)v;
    return true;
}

void printStatus()
{
    SerialUSB.print(F("Poller: ")); SerialUSB.println(running ? F("running") : F("stopped"));
    SerialUSB.print(F("Bus: CAN")); SerialUSB.print(busSel);
    SerialUSB.print(F(" @ ")); SerialUSB.print(baud);
    SerialUSB.print(F("  protocol: "));
    SerialUSB.println(protoLocked ? F("locked") : F("auto-detecting"));
    SerialUSB.print(F("Interval: ")); SerialUSB.print(intervalMs); SerialUSB.println(F(" ms"));
    SerialUSB.print(F("PIDs: "));
    for (uint8_t i = 0; i < nPids; i++) {
        if (i) SerialUSB.print(F(","));
        if (pidsvc[i] < 0x10) SerialUSB.print(F("0"));
        SerialUSB.print(String(pidsvc[i], HEX));
        if (pids[i] < 0x10) SerialUSB.print(F("0"));
        SerialUSB.print(String(pids[i], HEX));
    }
    SerialUSB.println();
}

} // namespace (anonymous)

void begin()
{
    active = true;
    running = false;
    busUp = false;
    protoLocked = false;
    detectBaud = 500000;
    lastDetectAt = 0;
    sweepSlot = 0xFF;
    mmBinActive = false; sawF1 = false;
    SerialUSB.println(F("OBD2 poller: 'obd2 start' begins. CSV goes to the Bee link."));
}

void loop()
{
    // mm-channel divert on the Bee link (same pattern as ELM/UDS).
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
    }
    if (!active || !running) return;
    uint32_t now = millis();
    if (!protoLocked) {
        if ((uint32_t)(now - lastDetectAt) >= DETECT_RETRY_MS) {
            lastDetectAt = now;
            if (detectOnce()) {
                SerialUSB.print(F("[obd2] protocol locked at "));
                SerialUSB.println(detectBaud);
                slotArm();  // Phase 6: scheduler takes over the sweep cadence
            }
        }
        return;
    }
    // Phase 6: the sweep cadence is a txsched slot now (fired from
    // ModeManager::loop via txsched::poll()); nothing to do here.
}

void exit()
{
    stopPoller();
    busRelease();  // idempotent; manager's quiesceBuses() follows
    active = false;
}

void console(char* line)
{
    while (*line == ' ') line++;
    if (*line == '\0') {
        SerialUSB.println(F("obd2 start|stop|interval <ms>|pids <list>|bus 0|1|baud <n>"));
        SerialUSB.println(F("obd2 dtc|dtcpend|dtcperm|clear|vin|ff <pid> [frame]|ctl <tid>|info <pid>"));
        SerialUSB.println(F("obd2 discover|pidinfo <svc> <pid>|status"));
        return;
    }
    char* save = nullptr;
    char* tok = strtok_r(line, " \t", &save);
    if (!tok) return;

    if (strcasecmp(tok, "start") == 0) {
        if (!active) { SerialUSB.println(F("OBD2 mode is not active.")); return; }
        startPoller();
        SerialUSB.println(F("Poller started (auto-detect 500k/250k)."));
        return;
    }
    if (strcasecmp(tok, "stop") == 0) {
        stopPoller();
        SerialUSB.println(F("Poller stopped."));
        return;
    }
    if (strcasecmp(tok, "interval") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        uint32_t ms = a ? (uint32_t)strtoul(a, nullptr, 10) : 0;
        if (ms >= 100 && ms <= 60000) {
            intervalMs = (uint16_t)ms;
            // Phase 6: live re-arm of the scheduler slot when polling.
            if (sweepSlot != 0xFF) txsched::setPeriod(sweepSlot, intervalMs);
            SerialUSB.print(F("Interval: ")); SerialUSB.println(intervalMs);
        } else SerialUSB.println(F("usage: obd2 interval <100..60000>"));
        return;
    }
    if (strcasecmp(tok, "pids") == 0) {
        // Phase 9: entries are (service,PID): "010C" = svc 01 pid 0C,
        // "0902" = svc 09 pid 02. Bare "0C" still means (01,0C).
        char* a = strtok_r(nullptr, " \t", &save);
        if (!a) { SerialUSB.println(F("usage: obd2 pids 010C,010D,0902")); return; }
        uint8_t np[16], ns[16]; uint8_t nn = 0;
        char* save2 = nullptr;
        for (char* p = strtok_r(a, ",", &save2); p && nn < 16; p = strtok_r(nullptr, ",", &save2)) {
            size_t L = strlen(p);
            uint8_t svc = 0x01, pid = 0;
            char* e = nullptr;
            if (L == 4) {
                char sb[3] = { p[0], p[1], '\0' };
                char pb[3] = { p[2], p[3], '\0' };
                long sv = strtol(sb, &e, 16);
                if (*e != '\0' || sv < 1 || sv > 10) { SerialUSB.println(F("Bad service hex.")); return; }
                svc = (uint8_t)sv;
                long pv = strtol(pb, &e, 16);
                if (*e != '\0') { SerialUSB.println(F("Bad PID hex.")); return; }
                pid = (uint8_t)pv;
            } else {
                const char* t = (L > 2) ? p + L - 2 : p;
                if (!hexByte(t, pid)) { SerialUSB.println(F("Bad PID hex.")); return; }
            }
            ns[nn] = svc; np[nn] = pid; nn++;
        }
        if (nn == 0) { SerialUSB.println(F("No PIDs parsed.")); return; }
        nPids = nn;
        for (uint8_t i = 0; i < nn; i++) { pids[i] = np[i]; pidsvc[i] = ns[i]; }
        SerialUSB.print(nPids); SerialUSB.println(F(" (svc,PID) pair(s) set."));
        return;
    }
    if (strcasecmp(tok, "bus") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        if (a && (a[0] == '0' || a[0] == '1') && a[1] == '\0') {
            uint8_t nb = (uint8_t)(a[0] - '0');
            if (nb != busSel) { busSel = nb; stopPoller(); busRelease(); }
            SerialUSB.print(F("OBD2 bus: CAN")); SerialUSB.println(busSel);
        } else SerialUSB.println(F("usage: obd2 bus 0|1"));
        return;
    }
    if (strcasecmp(tok, "baud") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        uint32_t b = a ? (uint32_t)strtoul(a, nullptr, 10) : 0;
        if (validBaud(b)) {
            if (b != baud) { baud = b; stopPoller(); busRelease(); }
            SerialUSB.print(F("OBD2 baud: ")); SerialUSB.println(baud);
        } else SerialUSB.println(F("usage: obd2 baud <100000|125000|250000|500000|800000|1000000>"));
        return;
    }
    if (strcasecmp(tok, "dtc") == 0) { dtcScan(0x03); return; }
    if (strcasecmp(tok, "dtcpend") == 0) { dtcScan(0x07); return; }
    if (strcasecmp(tok, "dtcperm") == 0) { dtcScan(0x0A); return; }
    if (strcasecmp(tok, "clear") == 0) { dtcClear(); return; }
    if (strcasecmp(tok, "vin") == 0) { vehInfo(0x02); return; }
    if (strcasecmp(tok, "info") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        uint8_t pid = 0;
        if (!a || !hexByte(a, pid)) { SerialUSB.println(F("usage: obd2 info <pidHex>")); return; }
        vehInfo(pid);
        return;
    }
    if (strcasecmp(tok, "ff") == 0) {
        char* aP = strtok_r(nullptr, " \t", &save);
        char* aF = strtok_r(nullptr, " \t", &save);
        uint8_t pid = 0, frame = 0;
        if (!aP || !hexByte(aP, pid)) { SerialUSB.println(F("usage: obd2 ff <pidHex> [frame]")); return; }
        if (aF) {
            char* e = nullptr; long f = strtol(aF, &e, 10);
            if (*e != '\0' || f < 0 || f > 255) { SerialUSB.println(F("Bad frame.")); return; }
            frame = (uint8_t)f;
        }
        freezeFrame(pid, frame);
        return;
    }
    if (strcasecmp(tok, "ctl") == 0) {
        char* a = strtok_r(nullptr, " \t", &save);
        uint8_t tid = 0;
        if (!a || !hexByte(a, tid)) { SerialUSB.println(F("usage: obd2 ctl <testIdHex>")); return; }
        obdControl(tid);
        return;
    }
    if (strcasecmp(tok, "pidinfo") == 0) {
        char* aS = strtok_r(nullptr, " \t", &save);
        char* aP = strtok_r(nullptr, " \t", &save);
        uint8_t svc = 0, pid = 0;
        char* e = nullptr;
        if (!aS || !aP) { SerialUSB.println(F("usage: obd2 pidinfo <svcHex> <pidHex>")); return; }
        long sv = strtol(aS, &e, 16);
        if (*e != '\0' || sv < 1 || sv > 10) { SerialUSB.println(F("Bad service.")); return; }
        svc = (uint8_t)sv;
        if (!hexByte(aP, pid)) { SerialUSB.println(F("Bad PID.")); return; }
        const pidreg::PidEntry* pe = pidreg::find(svc, pid);
        if (pe == nullptr) { SerialUSB.println(F("Unknown (svc,PID) — no registry entry.")); return; }
        SerialUSB.print(pe->name);
        SerialUSB.print(F("  unit: ")); SerialUSB.print(pe->unit);
        SerialUSB.print(F("  bytes: ")); SerialUSB.println(pe->nBytes);
        return;
    }
    if (strcasecmp(tok, "discover") == 0) {
        // Phase 9: stop our own bus activity first, then run the scan.
        stopPoller();
        busRelease();
        discover::Result r;
        SerialUSB.println(F("Discovering (CAN)..."));
        if (discover::run(busSel, r)) {
            discover::print(r);
        } else {
            SerialUSB.println(F("No ECU answered."));
        }
        return;
    }
    if (strcasecmp(tok, "status") == 0) { printStatus(); return; }
    SerialUSB.println(F("Unknown. See `obd2` for the command list."));
}

uint8_t mmCfg(uint8_t bus, uint32_t baudIn, uint16_t intervalIn,
              const uint8_t* pidsIn, uint8_t nPidsIn)
{
    if (bus > 1 || !validBaud(baudIn)) return mm::ST_BAD_ARG;
    if (intervalIn < 100 || intervalIn > 60000) return mm::ST_BAD_ARG;
    if (nPidsIn == 0 || nPidsIn > 16) return mm::ST_BAD_ARG;
    stopPoller();  // reconfig stops the poller (safe)
    busSel = bus; baud = baudIn; intervalMs = intervalIn;
    nPids = nPidsIn;
    for (uint8_t i = 0; i < nPidsIn; i++) pids[i] = pidsIn[i];
    busRelease();
    return mm::ST_OK;
}

uint8_t mmCtl(uint8_t start, uint8_t& runningOut)
{
    if (start > 1) return mm::ST_BAD_ARG;
    if (!active && start) return mm::ST_ERR;  // only valid in OBD2 mode
    if (start) {
        startPoller();
    } else {
        stopPoller();
    }
    runningOut = running ? 1 : 0;
    return mm::ST_OK;
}

void mmStatus(uint8_t& runningOut, uint8_t& busOut,
              uint16_t& intervalMsOut, uint8_t& nPidsOut)
{
    runningOut = running ? 1 : 0;
    busOut = busSel;
    intervalMsOut = intervalMs;
    nPidsOut = nPids;
}

// Phase 9: mm-channel discovery. Stops our own bus activity first (same
// discipline as the console `obd2 discover`).
uint8_t mmDiscover(uint8_t bus)
{
    if (bus > 1) return mm::ST_BAD_ARG;
    if (!active) return mm::ST_ERR;  // only valid in OBD2 mode
    stopPoller();
    busRelease();
    discover::Result r;
    busSel = bus;
    return discover::run(bus, r) ? mm::ST_OK : mm::ST_ERR;
}

} // namespace obd2
