// sim_shim/vehicle_ecu.cpp — scripted OBD + UDS ECU for [env:sim_m2].
#include "vehicle_ecu.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// Sim bus instances live in SimCAN_globals.cpp at global scope.
extern CANRaw Can1;

namespace vehicle_ecu {
namespace {

bool g_enabled = true;

// ---- mutable vehicle state (defaults match prior static INFERRED values) ----
// rpm: engineering RPM 0..16383; OBD raw = rpm * 4 (clamped to 0xFFFF)
// speed: km/h 0..255
// coolant_c: degrees C -40..215; OBD byte = c + 40
// throttle_pct: 0..100; OBD byte = pct * 255 / 100
int g_rpm = 3000;
int g_speed = 80;
int g_coolant_c = 90;
int g_throttle_pct = 25;

constexpr uint8_t  MAP_KPA = 100;
constexpr uint8_t  IAT = 60;          // 20 °C (still static)
constexpr uint8_t  FUEL_LEVEL = 128;  // 50% (still static)

static const char VIN17[] = "1G1YY25R6X5100001";
static const char PART_NO[] = "M2-SIM-ECU";
static uint8_t g_session = 0x01;

enum class Sig : uint8_t { RPM, SPEED, COOLANT, THROTTLE, NONE };

struct Ramp {
    bool active = false;
    int from = 0;
    int to = 0;
    int steps = 0;   // number of intervals (>=1)
    int i = 0;       // 0..steps inclusive (steps+1 samples)
};

Ramp g_ramp_rpm, g_ramp_speed, g_ramp_coolant, g_ramp_throttle;

Ramp& rampFor(Sig s)
{
    switch (s) {
    case Sig::RPM: return g_ramp_rpm;
    case Sig::SPEED: return g_ramp_speed;
    case Sig::COOLANT: return g_ramp_coolant;
    case Sig::THROTTLE: return g_ramp_throttle;
    default: return g_ramp_rpm;
    }
}

int* valuePtr(Sig s)
{
    switch (s) {
    case Sig::RPM: return &g_rpm;
    case Sig::SPEED: return &g_speed;
    case Sig::COOLANT: return &g_coolant_c;
    case Sig::THROTTLE: return &g_throttle_pct;
    default: return &g_rpm;
    }
}

// Advance ramp one step and write into the live value. Called on PID read.
void advanceOnRead(Sig s)
{
    Ramp& r = rampFor(s);
    if (!r.active) return;
    int* v = valuePtr(s);
    // value at sample i: from + (to-from)*i/steps
    long num = (long)r.from + ((long)r.to - r.from) * r.i / r.steps;
    *v = (int)num;
    if (r.i >= r.steps) {
        r.active = false;  // hold at `to`
        *v = r.to;
    } else {
        r.i++;
    }
}

int clampRpm(int v) { return v < 0 ? 0 : (v > 16383 ? 16383 : v); }
int clampSpeed(int v) { return v < 0 ? 0 : (v > 255 ? 255 : v); }
int clampCoolant(int v) { return v < -40 ? -40 : (v > 215 ? 215 : v); }
int clampThrottle(int v) { return v < 0 ? 0 : (v > 100 ? 100 : v); }

uint16_t rpmRaw()
{
    long raw = (long)g_rpm * 4;
    if (raw > 65535) raw = 65535;
    return (uint16_t)raw;
}
uint8_t coolantByte()
{
    int b = g_coolant_c + 40;
    if (b < 0) b = 0;
    if (b > 255) b = 255;
    return (uint8_t)b;
}
uint8_t throttleByte()
{
    return (uint8_t)((g_throttle_pct * 255) / 100);
}

// Pending multi-frame TX
struct MfPending {
    bool active;
    CANRaw* bus;
    uint32_t id;
    bool ext;
    uint8_t data[64];
    uint16_t len;
    uint16_t sent;
    uint8_t sn;
    uint8_t bs_remaining;  // last FC's Block Size (0 = send all remaining)
    uint8_t stmin;         // last FC's STmin, recorded; treated as 0 in host sim
};
MfPending g_mf = {};

void injectRaw(CANRaw* bus, uint32_t id, bool ext, const uint8_t d[8])
{
    if (!bus) return;
    CAN_FRAME f;
    f.id = id;
    f.extended = ext ? 1 : 0;
    f.rtr = 0;
    f.length = 8;
    memcpy(f.data.bytes, d, 8);
    bus->injectFrame(f);
}

void injectSf(CANRaw* bus, uint32_t id, bool ext,
              const uint8_t* payload, uint8_t n)
{
    if (!bus || n == 0 || n > 7) return;
    uint8_t d[8];
    memset(d, 0, 8);
    d[0] = n;
    memcpy(d + 1, payload, n);
    injectRaw(bus, id, ext, d);
}

void injectNrc(CANRaw* bus, uint32_t id, bool ext, uint8_t sid, uint8_t nrc)
{
    uint8_t p[3] = { 0x7F, sid, nrc };
    injectSf(bus, id, ext, p, 3);
}

void startMulti(CANRaw* bus, uint32_t id, bool ext,
                const uint8_t* payload, uint16_t len)
{
    if (!bus || len < 8 || len > sizeof(g_mf.data)) return;
    memcpy(g_mf.data, payload, len);
    g_mf.len = len;
    g_mf.bus = bus;
    g_mf.id = id;
    g_mf.ext = ext;
    g_mf.sn = 1;
    g_mf.bs_remaining = 0;
    g_mf.stmin = 0;
    g_mf.active = true;

    uint8_t ff[8];
    memset(ff, 0, 8);
    ff[0] = 0x10 | ((len >> 8) & 0x0F);
    ff[1] = (uint8_t)(len & 0xFF);
    memcpy(ff + 2, payload, 6);
    g_mf.sent = 6;
    injectRaw(bus, id, ext, ff);
}

// Emit up to `budget` consecutive frames (budget == 0 → all remaining,
// per ISO 15765-2 Block Size semantics). STmin is intentionally not delayed:
// the host sim has no meaningful microsecond clock for inter-CF gaps, so
// STmin is parsed and stored but treated as 0 (documented limitation).
void emitCfBlock(uint8_t budget)
{
    uint8_t n = budget;
    while (g_mf.sent < g_mf.len && (budget == 0 || n > 0)) {
        uint8_t cf[8];
        memset(cf, 0, 8);
        cf[0] = 0x20 | (g_mf.sn & 0x0F);
        uint8_t cnt = (uint8_t)((g_mf.len - g_mf.sent > 7)
                                   ? 7 : (g_mf.len - g_mf.sent));
        memcpy(cf + 1, g_mf.data + g_mf.sent, cnt);
        injectRaw(g_mf.bus, g_mf.id, g_mf.ext, cf);
        g_mf.sent = (uint16_t)(g_mf.sent + cnt);
        g_mf.sn = (uint8_t)((g_mf.sn + 1) & 0x0F);
        if (budget != 0) n--;
    }
    if (g_mf.sent >= g_mf.len) g_mf.active = false;
}

void continueMultiOnFc(CANRaw* bus, const CAN_FRAME& tx)
{
    if (!g_mf.active || bus != g_mf.bus) return;
    if (tx.length < 1 || (tx.data.bytes[0] & 0xF0) != 0x30) return;
    // Flow Status: 0 = ContinueToSend; 1 = Wait, 2 = Overflow → abort the
    // transfer cleanly, no further CFs (ISO 15765-2).
    if ((tx.data.bytes[0] & 0x0F) != 0) { g_mf.active = false; return; }
    if (tx.length < 3) return;  // malformed FC: BS + STmin bytes required
    uint8_t bs = tx.data.bytes[1];
    g_mf.stmin = tx.data.bytes[2];  // recorded; treated as 0 (see above)
    g_mf.bs_remaining = bs;
    emitCfBlock(bs);
}

uint8_t buildMode01(uint8_t pid, uint8_t* out, uint8_t outMax)
{
    if (outMax < 2) return 0;
    out[0] = 0x41;
    out[1] = pid;
    switch (pid) {
    case 0x00:
        if (outMax < 6) return 0;
        // Claim exactly the PIDs implemented in the 01-20 group:
        // byte layout per ISO 15031: support bit for PID n is bit
        // (7-((n-1)%8)) of byte ((n-1)/8).
        // PID05: n-1=4,  byte0 bit3 -> 0x08
        // PID0B: n-1=10, byte1 bit5 -> 0x20
        // PID0C: n-1=11, byte1 bit4 -> 0x10
        // PID0D: n-1=12, byte1 bit3 -> 0x08
        // PID0F: n-1=14, byte1 bit1 -> 0x02
        // PID11: n-1=16, byte2 bit7 -> 0x80
        // (PID 1C=28 and 2F=47 live in higher groups, discovered via
        // 0120/0140 — correctly absent here.)
        out[2] = 0x08;
        out[3] = 0x3A;
        out[4] = 0x80;
        out[5] = 0x00;
        return 6;
    case 0x05:
        advanceOnRead(Sig::COOLANT);
        if (outMax < 3) return 0;
        out[2] = coolantByte();
        return 3;
    case 0x0B:
        if (outMax < 3) return 0;
        out[2] = MAP_KPA;
        return 3;
    case 0x0C:
        advanceOnRead(Sig::RPM);
        if (outMax < 4) return 0;
        {
            uint16_t raw = rpmRaw();
            out[2] = (uint8_t)(raw >> 8);
            out[3] = (uint8_t)(raw & 0xFF);
        }
        return 4;
    case 0x0D:
        advanceOnRead(Sig::SPEED);
        if (outMax < 3) return 0;
        out[2] = (uint8_t)clampSpeed(g_speed);
        return 3;
    case 0x0F:
        if (outMax < 3) return 0;
        out[2] = IAT;
        return 3;
    case 0x11:
        advanceOnRead(Sig::THROTTLE);
        if (outMax < 3) return 0;
        out[2] = throttleByte();
        return 3;
    case 0x1C:
        if (outMax < 3) return 0;
        out[2] = 0x01;
        return 3;
    case 0x2F:
        if (outMax < 3) return 0;
        out[2] = FUEL_LEVEL;
        return 3;
    default:
        return 0;
    }
}

bool handleUds(CANRaw* bus, uint32_t respId, bool ext,
               const uint8_t* req, uint8_t reqLen)
{
    if (reqLen < 1) return false;
    const uint8_t sid = req[0];

    switch (sid) {
    case 0x10: {
        if (reqLen < 2) { injectNrc(bus, respId, ext, sid, 0x13); return true; }
        uint8_t sess = req[1];
        if (sess != 0x01 && sess != 0x02 && sess != 0x03) {
            injectNrc(bus, respId, ext, sid, 0x12);
            return true;
        }
        g_session = sess;
        uint8_t p[2] = { 0x50, sess };
        injectSf(bus, respId, ext, p, 2);
        return true;
    }
    case 0x3E: {
        if (reqLen < 2) { injectNrc(bus, respId, ext, sid, 0x13); return true; }
        if (req[1] == 0x80) return true;
        uint8_t p[2] = { 0x7E, (uint8_t)(req[1] & 0x7F) };
        injectSf(bus, respId, ext, p, 2);
        return true;
    }
    case 0x22: {
        if (reqLen < 3) { injectNrc(bus, respId, ext, sid, 0x13); return true; }
        uint16_t did = (uint16_t)((req[1] << 8) | req[2]);
        uint8_t buf[48];
        buf[0] = 0x62;
        buf[1] = req[1];
        buf[2] = req[2];
        uint16_t plen = 3;

        switch (did) {
        case 0xF186:
            buf[3] = g_session;
            injectSf(bus, respId, ext, buf, 4);
            return true;
        case 0xF187: case 0xF188: case 0xF18A: case 0xF197: {
            size_t sl = strlen(PART_NO);
            memcpy(buf + 3, PART_NO, sl);
            plen = (uint16_t)(3 + sl);
            if (plen <= 7) injectSf(bus, respId, ext, buf, (uint8_t)plen);
            else startMulti(bus, respId, ext, buf, plen);
            return true;
        }
        case 0xF190: {
            memcpy(buf + 3, VIN17, 17);
            startMulti(bus, respId, ext, buf, 20);
            return true;
        }
        default:
            injectNrc(bus, respId, ext, sid, 0x31);
            return true;
        }
    }
    case 0x19: {
        if (reqLen < 2) { injectNrc(bus, respId, ext, sid, 0x13); return true; }
        if (req[1] == 0x02) {
            uint8_t p[3] = { 0x59, 0x02, 0xFF };
            injectSf(bus, respId, ext, p, 3);
            return true;
        }
        injectNrc(bus, respId, ext, sid, 0x12);
        return true;
    }
    case 0x27: {
        if (reqLen < 2) { injectNrc(bus, respId, ext, sid, 0x13); return true; }
        uint8_t level = req[1];
        if ((level & 1) == 1) {
            uint8_t p[6] = { 0x67, level, 0xDE, 0xAD, 0xBE, 0xEF };
            injectSf(bus, respId, ext, p, 6);
            return true;
        }
        injectNrc(bus, respId, ext, sid, 0x35);
        return true;
    }
    case 0x11:
        injectNrc(bus, respId, ext, sid, 0x22);
        return true;
    default:
        injectNrc(bus, respId, ext, sid, 0x11);
        return true;
    }
}

bool isFunctional11(uint32_t id) { return id == 0x7DFul; }
bool isPhysical11(uint32_t id)   { return id == 0x7E0ul; }
bool isFunctional29(uint32_t id) { return id == 0x18DB33F1ul; }
bool isPhysical29(uint32_t id)   { return id == 0x18DA10F1ul; }

uint32_t respIdFor(uint32_t txId, bool ext)
{
    (void)txId;
    if (ext) return 0x18DAF110ul;
    return 0x7E8ul;
}

Sig parseSig(const char* name)
{
    if (!name) return Sig::NONE;
    if (!strcasecmp(name, "rpm")) return Sig::RPM;
    if (!strcasecmp(name, "speed")) return Sig::SPEED;
    if (!strcasecmp(name, "coolant")) return Sig::COOLANT;
    if (!strcasecmp(name, "throttle")) return Sig::THROTTLE;
    return Sig::NONE;
}

const char* sigName(Sig s)
{
    switch (s) {
    case Sig::RPM: return "rpm";
    case Sig::SPEED: return "speed";
    case Sig::COOLANT: return "coolant";
    case Sig::THROTTLE: return "throttle";
    default: return "?";
    }
}

}  // namespace

void setEnabled(bool on) { g_enabled = on; }
bool enabled() { return g_enabled; }

bool handleConsoleLine(const char* line,
                       void (*reply)(const char* msg, void* ctx),
                       void* ctx)
{
    if (!line || !reply) return false;
    // Skip leading spaces
    while (*line == ' ' || *line == '\t') line++;
    if (strncasecmp(line, "vehicle", 7) != 0) return false;
    const char* p = line + 7;
    if (*p != '\0' && *p != ' ' && *p != '\t') return false;
    while (*p == ' ' || *p == '\t') p++;

    // tokenize into max 5 args
    char buf[128];
    strncpy(buf, p, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    char* args[5] = {};
    int nargs = 0;
    char* save = nullptr;
    for (char* t = strtok_r(buf, " \t", &save); t && nargs < 5;
         t = strtok_r(nullptr, " \t", &save)) {
        args[nargs++] = t;
    }

    if (!strcasecmp(args[0], "help")) {
        reply("vehicle set  rpm|speed|coolant|throttle <value>", ctx);
        reply("vehicle ramp rpm|speed|coolant|throttle <from> <to> <steps>", ctx);
        reply("vehicle show", ctx);
        reply("vehicle tx   <idhex> <datahex>  (test-only raw frame to scripted ECU)", ctx);
        reply("vehicle mf   (multi-frame TX state)", ctx);
        return true;
    }

    if (!strcasecmp(args[0], "show")) {
        char msg[96];
        snprintf(msg, sizeof(msg),
                 "rpm=%d speed=%d coolant=%dC throttle=%d%%",
                 g_rpm, g_speed, g_coolant_c, g_throttle_pct);
        reply(msg, ctx);
        auto rampNote = [&](Sig s) {
            Ramp& r = rampFor(s);
            if (!r.active) return;
            char m[80];
            snprintf(m, sizeof(m), "  ramp %s %d->%d step %d/%d",
                     sigName(s), r.from, r.to, r.i, r.steps);
            reply(m, ctx);
        };
        rampNote(Sig::RPM);
        rampNote(Sig::SPEED);
        rampNote(Sig::COOLANT);
        rampNote(Sig::THROTTLE);
        return true;
    }

    if (!strcasecmp(args[0], "set") && nargs >= 3) {
        Sig s = parseSig(args[1]);
        if (s == Sig::NONE) {
            reply("ERR unknown signal (rpm|speed|coolant|throttle)", ctx);
            return true;
        }
        char* end = nullptr;
        long v = strtol(args[2], &end, 0);
        if (end == args[2] || *end != '\0') {
            reply("ERR bad value", ctx);
            return true;
        }
        switch (s) {
        case Sig::RPM: g_rpm = clampRpm((int)v); rampFor(s).active = false; break;
        case Sig::SPEED: g_speed = clampSpeed((int)v); rampFor(s).active = false; break;
        case Sig::COOLANT: g_coolant_c = clampCoolant((int)v); rampFor(s).active = false; break;
        case Sig::THROTTLE: g_throttle_pct = clampThrottle((int)v); rampFor(s).active = false; break;
        default: break;
        }
        char msg[64];
        snprintf(msg, sizeof(msg), "OK %s=%ld", args[1], v);
        reply(msg, ctx);
        return true;
    }

    if (!strcasecmp(args[0], "ramp") && nargs >= 5) {
        Sig s = parseSig(args[1]);
        if (s == Sig::NONE) {
            reply("ERR unknown signal", ctx);
            return true;
        }
        char* e1 = nullptr, *e2 = nullptr, *e3 = nullptr;
        long from = strtol(args[2], &e1, 0);
        long to = strtol(args[3], &e2, 0);
        long steps = strtol(args[4], &e3, 0);
        if (e1 == args[2] || e2 == args[3] || e3 == args[4] ||
            *e1 || *e2 || *e3 || steps < 1 || steps > 10000) {
            reply("ERR usage: vehicle ramp <sig> <from> <to> <steps>", ctx);
            return true;
        }
        // clamp endpoints
        int f = (int)from, t = (int)to;
        switch (s) {
        case Sig::RPM: f = clampRpm(f); t = clampRpm(t); break;
        case Sig::SPEED: f = clampSpeed(f); t = clampSpeed(t); break;
        case Sig::COOLANT: f = clampCoolant(f); t = clampCoolant(t); break;
        case Sig::THROTTLE: f = clampThrottle(f); t = clampThrottle(t); break;
        default: break;
        }
        Ramp& r = rampFor(s);
        r.active = true;
        r.from = f;
        r.to = t;
        r.steps = (int)steps;
        r.i = 0;
        *valuePtr(s) = f;
        char msg[80];
        snprintf(msg, sizeof(msg), "OK ramp %s %d->%d in %d steps",
                 sigName(s), f, t, (int)steps);
        reply(msg, ctx);
        return true;
    }

    // Test-only raw frame injection, straight to the scripted ECU.
    // Lets pytest drive SF/FC sequences (BS=1, FS abort, 29-bit FC) without
    // the M2 ISO-TP client racing its own FC. Delivered via onTx(&Can1, ...)
    // so responses land in Can1's RX queue, keeping Can0 pristine.
    if (!strcasecmp(args[0], "tx") && nargs >= 3) {
        char* end = nullptr;
        unsigned long id = strtoul(args[1], &end, 16);
        size_t dl = strlen(args[2]);
        bool ok = (end != args[1] && *end == '\0' && id <= 0x1FFFFFFFul &&
                   dl > 0 && dl <= 16 && (dl & 1) == 0);
        uint8_t d[8] = {};
        for (size_t i = 0; ok && i < dl / 2; i++) {
            char byte[3] = { args[2][2 * i], args[2][2 * i + 1], '\0' };
            char* e2 = nullptr;
            unsigned long v = strtoul(byte, &e2, 16);
            if (e2 == byte || *e2 != '\0') ok = false;
            else d[i] = (uint8_t)v;
        }
        if (!ok) {
            reply("ERR usage: vehicle tx <idhex> <datahex, up to 16 chars>", ctx);
            return true;
        }
        CAN_FRAME f;
        f.id = (uint32_t)id;
        f.extended = (id > 0x7FFul) ? 1 : 0;
        f.rtr = 0;
        f.length = (uint8_t)(dl / 2);
        memcpy(f.data.bytes, d, 8);
        onTx(&Can1, f);
        reply("OK tx", ctx);
        return true;
    }

    if (!strcasecmp(args[0], "mf")) {
        char msg[96];
        snprintf(msg, sizeof(msg), "mf active=%d len=%u sent=%u sn=%u bs=%u stmin=%u",
                 g_mf.active ? 1 : 0, g_mf.len, g_mf.sent, g_mf.sn,
                 g_mf.bs_remaining, g_mf.stmin);
        reply(msg, ctx);
        return true;
    }

    reply("ERR vehicle help", ctx);
    return true;
}

void onTx(CANRaw* bus, const CAN_FRAME& tx)
{
    if (!g_enabled || !bus) return;
    if (tx.rtr) return;
    if (tx.length < 1) return;

    const bool ext = tx.extended != 0;

    // Flow Control for an outstanding multi-frame TX: accepted on the same
    // address class that started the transfer — 11-bit physical/functional
    // (0x7E0/0x7DF) or 29-bit physical (0x18DA10F1).
    const bool fcHere =
        (!ext && (tx.id == 0x7E0ul || tx.id == 0x7DFul)) ||
        (ext && tx.id == 0x18DA10F1ul);
    if (g_mf.active && fcHere) {
        continueMultiOnFc(bus, tx);
        if ((tx.data.bytes[0] & 0xF0) == 0x30) return;
    }

    const bool interesting =
        (!ext && (isFunctional11(tx.id) || isPhysical11(tx.id))) ||
        (ext && (isFunctional29(tx.id) || isPhysical29(tx.id)));
    if (!interesting) return;

    const uint8_t pci = tx.data.bytes[0];
    if ((pci & 0xF0) != 0x00) return;
    const uint8_t n = pci & 0x0F;
    if (n < 1 || n > 7 || tx.length < (uint8_t)(1 + n)) return;

    const uint8_t* req = tx.data.bytes + 1;
    const uint8_t sid = req[0];
    const uint32_t rid = respIdFor(tx.id, ext);

    if (sid == 0x01) {
        if (n < 2) return;
        uint8_t payload[8];
        uint8_t plen = buildMode01(req[1], payload, sizeof(payload));
        if (plen == 0) return;
        injectSf(bus, rid, ext, payload, plen);
        return;
    }

    handleUds(bus, rid, ext, req, n);
}

}  // namespace vehicle_ecu
