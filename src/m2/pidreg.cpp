// M2 multi-mode firmware — OBD-II PID registry data + decoders (Phase 9).
// Formulas per SAE J1979 (public standard). INFERRED on hardware.

#include "pidreg.h"

namespace pidreg {

namespace {

// Shorthand: LINEAR(svc,pid,nbytes,extract,signed,num,den,off,dec,min,max,unit,name)
#define L(svc, pid, nb, ex, sg, num, den, off, dec, mn, mx, unit, name) \
    { svc, pid, nb, Extract::ex, sg, num, den, off, dec, mn, mx, unit, name, DecKind::LINEAR }
#define B(svc, pid, name) \
    { svc, pid, 4, Extract::PAYLOAD, false, 1, 1, 0, 0, INT32_MIN, INT32_MAX, "", name, DecKind::BITMASK }
#define A9(pid, nb, name) \
    { 0x09, pid, nb, Extract::PAYLOAD, false, 1, 1, 0, 0, INT32_MIN, INT32_MAX, "", name, DecKind::ASCII }

const PidEntry TABLE[] = {
    // ---- service 01: support masks + status ----
    B(0x01, 0x00, "PIDs supported 01-20"),
    B(0x01, 0x01, "Monitor status since DTCs cleared"),
    B(0x01, 0x20, "PIDs supported 21-40"),
    B(0x01, 0x40, "PIDs supported 41-60"),
    B(0x01, 0x60, "PIDs supported 61-80"),
    B(0x01, 0x80, "PIDs supported 81-A0"),
    B(0x01, 0xA0, "PIDs supported A1-C0"),
    B(0x01, 0xC0, "PIDs supported C1-E0"),
    // ---- service 01: scalar PIDs (SAE J1979) ----
    L(0x01, 0x04, 1, A, false, 100, 255, 0, 1, 0, 1000, "%", "Calculated engine load"),
    L(0x01, 0x05, 1, A, false, 1, 1, -40, 0, -40, 215, "C", "Engine coolant temperature"),
    L(0x01, 0x06, 1, A, false, 100, 128, -1000, 1, -1000, 992, "%", "Short term fuel trim Bank 1"),
    L(0x01, 0x07, 1, A, false, 100, 128, -1000, 1, -1000, 992, "%", "Long term fuel trim Bank 1"),
    L(0x01, 0x08, 1, A, false, 100, 128, -1000, 1, -1000, 992, "%", "Short term fuel trim Bank 2"),
    L(0x01, 0x09, 1, A, false, 100, 128, -1000, 1, -1000, 992, "%", "Long term fuel trim Bank 2"),
    L(0x01, 0x0B, 1, A, false, 1, 1, 0, 0, 0, 255, "kPa", "Intake manifold absolute pressure"),
    L(0x01, 0x0C, 2, AB, false, 1, 4, 0, 0, 0, 16383, "rpm", "Engine RPM"),
    L(0x01, 0x0D, 1, A, false, 1, 1, 0, 0, 0, 255, "km/h", "Vehicle speed"),
    L(0x01, 0x0E, 1, A, false, 1, 2, -640, 1, -640, 635, "deg", "Timing advance"),
    L(0x01, 0x0F, 1, A, false, 1, 1, -40, 0, -40, 215, "C", "Intake air temperature"),
    L(0x01, 0x10, 2, AB, false, 1, 100, 0, 2, 0, 65535, "g/s", "MAF air flow rate"),
    L(0x01, 0x11, 1, A, false, 100, 255, 0, 1, 0, 1000, "%", "Throttle position"),
    B(0x01, 0x13, "Oxygen sensors present"),
    L(0x01, 0x14, 2, A, false, 1, 200, 0, 3, 0, 1275, "V", "O2 sensor 1 voltage"),
    L(0x01, 0x15, 2, A, false, 1, 200, 0, 3, 0, 1275, "V", "O2 sensor 2 voltage"),
    L(0x01, 0x16, 2, A, false, 1, 200, 0, 3, 0, 1275, "V", "O2 sensor 3 voltage"),
    L(0x01, 0x17, 2, A, false, 1, 200, 0, 3, 0, 1275, "V", "O2 sensor 4 voltage"),
    L(0x01, 0x1C, 1, A, false, 1, 1, 0, 0, 0, 255, "", "OBD standards"),
    L(0x01, 0x1F, 2, AB, false, 1, 1, 0, 0, 0, 65535, "s", "Run time since engine start"),
    L(0x01, 0x21, 2, AB, false, 1, 1, 0, 0, 0, 65535, "km", "Distance traveled with MIL on"),
    L(0x01, 0x2F, 1, A, false, 100, 255, 0, 1, 0, 1000, "%", "Fuel tank level input"),
    L(0x01, 0x31, 2, AB, false, 1, 1, 0, 0, 0, 65535, "km", "Distance since codes cleared"),
    L(0x01, 0x33, 1, A, false, 1, 1, 0, 0, 0, 255, "kPa", "Absolute barometric pressure"),
    L(0x01, 0x42, 2, AB, false, 1, 1000, 0, 3, 0, 65535, "V", "Control module voltage"),
    L(0x01, 0x43, 2, AB, false, 100, 255, 0, 1, 0, 25700, "%", "Absolute load value"),
    L(0x01, 0x44, 2, AB, false, 2, 65535, 0, 3, 0, 2000, "lambda", "Fuel/air equivalence ratio"),
    L(0x01, 0x45, 1, A, false, 100, 255, 0, 1, 0, 1000, "%", "Relative throttle position"),
    L(0x01, 0x46, 1, A, false, 1, 1, -40, 0, -40, 215, "C", "Ambient air temperature"),
    L(0x01, 0x47, 1, A, false, 100, 255, 0, 1, 0, 1000, "%", "Absolute throttle position B"),
    L(0x01, 0x49, 1, A, false, 100, 255, 0, 1, 0, 1000, "%", "Accelerator pedal position D"),
    L(0x01, 0x4A, 1, A, false, 100, 255, 0, 1, 0, 1000, "%", "Accelerator pedal position E"),
    L(0x01, 0x4C, 1, A, false, 100, 255, 0, 1, 0, 1000, "%", "Commanded throttle actuator"),
    L(0x01, 0x51, 1, A, false, 1, 1, 0, 0, 0, 23, "", "Fuel type"),
    L(0x01, 0x52, 1, A, false, 100, 255, 0, 1, 0, 1000, "%", "Ethanol fuel %"),
    L(0x01, 0x5E, 2, AB, false, 1, 20, 0, 2, 0, 327675, "L/h", "Engine fuel rate"),
    // ---- service 06: on-board monitoring ----
    B(0x06, 0x00, "MIDs supported 01-20"),
    // ---- service 09: vehicle information ----
    B(0x09, 0x00, "PIDs supported (mode 09)"),
    A9(0x02, 17, "VIN"),
    A9(0x04, 16, "Calibration ID"),
    { 0x09, 0x06, 4, Extract::PAYLOAD, false, 1, 1, 0, 0,
      INT32_MIN, INT32_MAX, "", "Calibration verification numbers", DecKind::RAW },
    A9(0x0A, 20, "ECU name"),
};

const uint16_t TABLE_N = sizeof(TABLE) / sizeof(TABLE[0]);

int64_t extractRaw(const PidEntry* e, const uint8_t* d, uint8_t n)
{
    uint8_t a = (n > 0) ? d[0] : 0;
    uint8_t b = (n > 1) ? d[1] : 0;
    switch (e->extract) {
        case Extract::A: return a;
        case Extract::B: return b;
        case Extract::AB:
            if (e->isSigned) return (int16_t)((uint16_t)a * 256 + b);
            return (int64_t)a * 256 + b;
        case Extract::ABCD: {
            uint32_t v = ((uint32_t)a << 24) | ((uint32_t)b << 16) |
                         ((n > 2 ? (uint32_t)d[2] : 0) << 8) |
                         (n > 3 ? (uint32_t)d[3] : 0);
            if (e->isSigned) return (int32_t)v;
            return (int64_t)v;
        }
        case Extract::PAYLOAD: return 0;
    }
    return 0;
}

void fmtScaled(char* o, size_t cap, int64_t scaled, uint8_t dec)
{
    // scaled is in 10^-dec units.
    bool neg = scaled < 0;
    if (neg) scaled = -scaled;
    int64_t div = 1;
    for (uint8_t i = 0; i < dec; i++) div *= 10;
    int64_t ip = (dec == 0) ? scaled : scaled / div;
    int64_t fp = (dec == 0) ? 0 : scaled % div;
    if (dec == 0) {
        snprintf(o, cap, "%s%lld", neg ? "-" : "", (long long)ip);
    } else {
        // zero-padded fraction
        char fmt[16];
        snprintf(fmt, sizeof(fmt), "%%s%%lld.%%0%dlld", dec);
        snprintf(o, cap, fmt, neg ? "-" : "", (long long)ip, (long long)fp);
    }
}

} // namespace (anonymous)

const PidEntry* find(uint8_t svc, uint8_t pid)
{
    for (uint16_t i = 0; i < TABLE_N; i++)
        if (TABLE[i].svc == svc && TABLE[i].pid == pid) return &TABLE[i];
    // Service 0x02 (freeze frame) shares the 0x01 formulas.
    if (svc == 0x02) {
        for (uint16_t i = 0; i < TABLE_N; i++)
            if (TABLE[i].svc == 0x01 && TABLE[i].pid == pid) return &TABLE[i];
    }
    return nullptr;
}

bool decode(const PidEntry* e, const uint8_t* d, uint8_t n,
            char* out, size_t cap)
{
    if (e == nullptr || out == nullptr || cap == 0) return false;
    static const char* H = "0123456789ABCDEF";
    switch (e->kind) {
        case DecKind::LINEAR: {
            int64_t raw = extractRaw(e, d, n);
            int64_t mul = 1;
            for (uint8_t i = 0; i < e->dec; i++) mul *= 10;
            // scaled = raw * num * 10^dec / den + off  (int64 to avoid overflow)
            int64_t scaled = (e->den != 0)
                ? (raw * (int64_t)e->num * mul) / (int64_t)e->den + (int64_t)e->off
                : raw * (int64_t)e->num * mul + (int64_t)e->off;
            char num[40];
            fmtScaled(num, sizeof(num), scaled, e->dec);
            if (e->unit[0] != '\0') snprintf(out, cap, "%s %s", num, e->unit);
            else snprintf(out, cap, "%s", num);
            return true;
        }
        case DecKind::BITMASK: {
            size_t p = 0;
            for (uint8_t i = 0; i < n && p + 3 < cap; i++) {
                if (i) out[p++] = ' ';
                out[p++] = H[(d[i] >> 4) & 0xF];
                out[p++] = H[d[i] & 0xF];
            }
            out[p] = '\0';
            return true;
        }
        case DecKind::ASCII: {
            size_t p = 0;
            for (uint8_t i = 0; i < n && p + 1 < cap; i++) {
                char c = (char)d[i];
                out[p++] = (c >= 32 && c < 127) ? c : '.';
            }
            out[p] = '\0';
            return true;
        }
        case DecKind::RAW: {
            size_t p = 0;
            for (uint8_t i = 0; i < n && p + 2 < cap; i++) {
                out[p++] = H[(d[i] >> 4) & 0xF];
                out[p++] = H[d[i] & 0xF];
            }
            out[p] = '\0';
            return true;
        }
    }
    return false;
}

uint16_t count() { return TABLE_N; }
const PidEntry* at(uint16_t i) { return (i < TABLE_N) ? &TABLE[i] : nullptr; }

} // namespace pidreg
