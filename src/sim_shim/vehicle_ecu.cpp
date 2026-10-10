// sim_shim/vehicle_ecu.cpp — scripted OBD + UDS ECU for [env:sim_m2].
#include "vehicle_ecu.h"
#include <string.h>

namespace vehicle_ecu {
namespace {

bool g_enabled = true;

// ---- static "vehicle" state (INFERRED constants) ----
constexpr uint16_t RPM_RAW = 0x2EE0;  // 3000 RPM
constexpr uint8_t  SPEED_KMH = 80;
constexpr uint8_t  COOLANT = 130;     // 90 °C
constexpr uint8_t  MAP_KPA = 100;
constexpr uint8_t  IAT = 60;          // 20 °C
constexpr uint8_t  TPS = 64;          // ~25%
constexpr uint8_t  FUEL_LEVEL = 128;  // 50%

// ISO 14229-ish DID payloads (INFERRED).
static const char VIN17[] = "1G1YY25R6X5100001";  // 17 chars
static const char PART_NO[] = "M2-SIM-ECU";        // short ASCII
static uint8_t g_session = 0x01;                   // tracks 0x10 requests

// Pending multi-frame TX (ECU → tester) after Flow Control.
struct MfPending {
    bool active;
    CANRaw* bus;
    uint32_t id;
    bool ext;
    uint8_t data[64];
    uint16_t len;
    uint16_t sent;   // bytes already put on the wire (after FF data)
    uint8_t sn;
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

// Start multi-frame: send First Frame, wait for tester FC in onTx.
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
    g_mf.active = true;

    uint8_t ff[8];
    memset(ff, 0, 8);
    ff[0] = 0x10 | ((len >> 8) & 0x0F);
    ff[1] = (uint8_t)(len & 0xFF);
    memcpy(ff + 2, payload, 6);
    g_mf.sent = 6;
    injectRaw(bus, id, ext, ff);
}

void continueMultiOnFc(CANRaw* bus, const CAN_FRAME& tx)
{
    if (!g_mf.active || bus != g_mf.bus) return;
    // Flow Control: PCI 0x30
    if ((tx.data.bytes[0] & 0xF0) != 0x30) return;

    while (g_mf.sent < g_mf.len) {
        uint8_t cf[8];
        memset(cf, 0, 8);
        cf[0] = 0x20 | (g_mf.sn & 0x0F);
        uint8_t n = (uint8_t)((g_mf.len - g_mf.sent > 7)
                                  ? 7 : (g_mf.len - g_mf.sent));
        memcpy(cf + 1, g_mf.data + g_mf.sent, n);
        injectRaw(g_mf.bus, g_mf.id, g_mf.ext, cf);
        g_mf.sent = (uint16_t)(g_mf.sent + n);
        g_mf.sn = (uint8_t)((g_mf.sn + 1) & 0x0F);
    }
    g_mf.active = false;
}

// ---- OBD Mode 01 ----
uint8_t buildMode01(uint8_t pid, uint8_t* out, uint8_t outMax)
{
    if (outMax < 2) return 0;
    out[0] = 0x41;
    out[1] = pid;
    switch (pid) {
    case 0x00:
        if (outMax < 6) return 0;
        out[2] = 0x98; out[3] = 0x3B; out[4] = 0x80; out[5] = 0x11;
        return 6;
    case 0x05:
        if (outMax < 3) return 0;
        out[2] = COOLANT;
        return 3;
    case 0x0B:
        if (outMax < 3) return 0;
        out[2] = MAP_KPA;
        return 3;
    case 0x0C:
        if (outMax < 4) return 0;
        out[2] = (uint8_t)(RPM_RAW >> 8);
        out[3] = (uint8_t)(RPM_RAW & 0xFF);
        return 4;
    case 0x0D:
        if (outMax < 3) return 0;
        out[2] = SPEED_KMH;
        return 3;
    case 0x0F:
        if (outMax < 3) return 0;
        out[2] = IAT;
        return 3;
    case 0x11:
        if (outMax < 3) return 0;
        out[2] = TPS;
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

// ---- UDS service dispatch ----
// req[] = UDS payload (no PCI), reqLen bytes. Returns false if ignored.
bool handleUds(CANRaw* bus, uint32_t respId, bool ext,
               const uint8_t* req, uint8_t reqLen)
{
    if (reqLen < 1) return false;
    const uint8_t sid = req[0];

    switch (sid) {
    case 0x10: {  // DiagnosticSessionControl
        if (reqLen < 2) {
            injectNrc(bus, respId, ext, sid, 0x13);  // incorrectMessageLength
            return true;
        }
        uint8_t sess = req[1];
        // Accept default (0x01), programming (0x02), extended (0x03).
        if (sess != 0x01 && sess != 0x02 && sess != 0x03) {
            injectNrc(bus, respId, ext, sid, 0x12);  // subFunctionNotSupported
            return true;
        }
        g_session = sess;
        uint8_t p[2] = { 0x50, sess };
        injectSf(bus, respId, ext, p, 2);
        return true;
    }
    case 0x3E: {  // TesterPresent
        if (reqLen < 2) {
            injectNrc(bus, respId, ext, sid, 0x13);
            return true;
        }
        // 0x00 = with response, 0x80 = suppress
        if (req[1] == 0x80) return true;  // silent
        uint8_t p[2] = { 0x7E, (uint8_t)(req[1] & 0x7F) };
        injectSf(bus, respId, ext, p, 2);
        return true;
    }
    case 0x22: {  // ReadDataByIdentifier
        if (reqLen < 3) {
            injectNrc(bus, respId, ext, sid, 0x13);
            return true;
        }
        uint16_t did = (uint16_t)((req[1] << 8) | req[2]);
        uint8_t buf[48];
        buf[0] = 0x62;
        buf[1] = req[1];
        buf[2] = req[2];
        uint16_t plen = 3;

        switch (did) {
        case 0xF186:  // ActiveDiagnosticSession
            buf[3] = g_session;
            plen = 4;
            injectSf(bus, respId, ext, buf, (uint8_t)plen);
            return true;
        case 0xF187:  // ManufacturerSparePartNumber (ASCII)
        case 0xF188:
        case 0xF18A:
        case 0xF197: {
            const char* s = PART_NO;
            size_t sl = strlen(s);
            if (3 + sl > sizeof(buf)) sl = sizeof(buf) - 3;
            memcpy(buf + 3, s, sl);
            plen = (uint16_t)(3 + sl);
            if (plen <= 7)
                injectSf(bus, respId, ext, buf, (uint8_t)plen);
            else
                startMulti(bus, respId, ext, buf, plen);
            return true;
        }
        case 0xF190: {  // VIN
            size_t sl = 17;
            memcpy(buf + 3, VIN17, sl);
            plen = (uint16_t)(3 + sl);  // 20 bytes → multi-frame
            startMulti(bus, respId, ext, buf, plen);
            return true;
        }
        default:
            injectNrc(bus, respId, ext, sid, 0x31);  // requestOutOfRange
            return true;
        }
    }
    case 0x19: {  // ReadDTCInformation — report empty DTC list
        if (reqLen < 2) {
            injectNrc(bus, respId, ext, sid, 0x13);
            return true;
        }
        // 0x02 reportDTCByStatusMask → 0x59 0x02 statusAvail (0) + no DTCs
        if (req[1] == 0x02) {
            uint8_t p[3] = { 0x59, 0x02, 0xFF };
            injectSf(bus, respId, ext, p, 3);
            return true;
        }
        injectNrc(bus, respId, ext, sid, 0x12);
        return true;
    }
    case 0x27: {  // SecurityAccess — seed stub only
        if (reqLen < 2) {
            injectNrc(bus, respId, ext, sid, 0x13);
            return true;
        }
        uint8_t level = req[1];
        if ((level & 1) == 1) {
            // requestSeed → fixed seed
            uint8_t p[6] = { 0x67, level, 0xDE, 0xAD, 0xBE, 0xEF };
            injectSf(bus, respId, ext, p, 6);
            return true;
        }
        // sendKey — always reject (no algorithm)
        injectNrc(bus, respId, ext, sid, 0x35);  // invalidKey
        return true;
    }
    case 0x11:  // ECUReset — NRC conditionsNotCorrect unless extended
        injectNrc(bus, respId, ext, sid, 0x22);
        return true;
    default:
        injectNrc(bus, respId, ext, sid, 0x11);  // serviceNotSupported
        return true;
    }
}

bool isFunctional11(uint32_t id) { return id == 0x7DFul; }
bool isPhysical11(uint32_t id)   { return id == 0x7E0ul; }
bool isFunctional29(uint32_t id) { return id == 0x18DB33F1ul; }
bool isPhysical29(uint32_t id)   { return id == 0x18DA10F1ul; }

uint32_t respIdFor(uint32_t txId, bool ext)
{
    if (ext) {
        if (txId == 0x18DA10F1ul) return 0x18DAF110ul;
        return 0x18DAF110ul;
    }
    if (txId == 0x7E0ul) return 0x7E8ul;
    return 0x7E8ul;
}

}  // namespace

void setEnabled(bool on) { g_enabled = on; }
bool enabled() { return g_enabled; }

void onTx(CANRaw* bus, const CAN_FRAME& tx)
{
    if (!g_enabled || !bus) return;
    if (tx.rtr) return;
    if (tx.length < 1) return;

    const bool ext = tx.extended != 0;

    // Multi-frame continuation: tester FC addressed to our ECU request id
    // arrives as TX on the same bus (tester → ECU). Physical 0x7E0.
    if (g_mf.active &&
        (!ext && (tx.id == 0x7E0ul || tx.id == 0x7DFul))) {
        continueMultiOnFc(bus, tx);
        // Fall through only if not FC; continueMulti returns after CF blast.
        if ((tx.data.bytes[0] & 0xF0) == 0x30) return;
    }

    const bool interesting =
        (!ext && (isFunctional11(tx.id) || isPhysical11(tx.id))) ||
        (ext && (isFunctional29(tx.id) || isPhysical29(tx.id)));
    if (!interesting) return;

    const uint8_t pci = tx.data.bytes[0];
    if ((pci & 0xF0) != 0x00) return;  // SF requests only (new transactions)
    const uint8_t n = pci & 0x0F;
    if (n < 1 || n > 7 || tx.length < (uint8_t)(1 + n)) return;

    const uint8_t* req = tx.data.bytes + 1;
    const uint8_t sid = req[0];
    const uint32_t rid = respIdFor(tx.id, ext);

    // OBD Mode 01 (ELM path)
    if (sid == 0x01) {
        if (n < 2) return;
        uint8_t payload[8];
        uint8_t plen = buildMode01(req[1], payload, sizeof(payload));
        if (plen == 0) return;
        injectSf(bus, rid, ext, payload, plen);
        return;
    }

    // UDS services
    handleUds(bus, rid, ext, req, n);
}

}  // namespace vehicle_ecu
