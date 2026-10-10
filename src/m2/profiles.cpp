// M2 multi-mode firmware — vehicle profiles (Phase 12). See profiles.h.

#include "profiles.h"
#include "flashstore.h"
#include "isotp.h"
#include "busfilter.h"
#include "capgate.h"
#include "mode_manager.h"  // manager() accessor for staged baud apply
#include "mode_gateway.h"  // gateway table export/import
#include "debug.h"
#include <string.h>

namespace profiles {
namespace {

Profile ram;          // working copy (what save() captures / load() applies)
bool ramValid = false;

// Pack the live ISO-TP default into the 15-byte CMD_ISOTP_CFG layout.
void captureIsotp(uint8_t* out)
{
    isotp::Config c;
    isotp::getDefault(c);
    out[0] = (uint8_t)(c.nBsMs & 0xFF);
    out[1] = (uint8_t)((c.nBsMs >> 8) & 0xFF);
    out[2] = (uint8_t)(c.nCrMs & 0xFF);
    out[3] = (uint8_t)((c.nCrMs >> 8) & 0xFF);
    out[4] = (uint8_t)(c.p2Ms & 0xFF);
    out[5] = (uint8_t)((c.p2Ms >> 8) & 0xFF);
    out[6] = (uint8_t)(c.p2StarMs & 0xFF);
    out[7] = (uint8_t)((c.p2StarMs >> 8) & 0xFF);
    out[8] = c.maxBlockSize;
    out[9] = c.stMin;
    out[10] = c.autoFlowControl ? 1 : 0;
    out[11] = (uint8_t)(c.maxPayload & 0xFF);
    out[12] = (uint8_t)((c.maxPayload >> 8) & 0xFF);
    out[13] = (uint8_t)c.addrType;
    out[14] = c.addrExt;
}

// Apply a 15-byte blob as the ISO-TP default (same validation as the mm
// path: reject out-of-range rather than clamping silently).
bool applyIsotp(const uint8_t* p)
{
    isotp::Config c;
    c.nBsMs = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
    c.nCrMs = (uint16_t)p[2] | ((uint16_t)p[3] << 8);
    c.p2Ms = (uint16_t)p[4] | ((uint16_t)p[5] << 8);
    c.p2StarMs = (uint16_t)p[6] | ((uint16_t)p[7] << 8);
    c.maxBlockSize = p[8];
    c.stMin = p[9];
    if (p[10] > 1) return false;
    c.autoFlowControl = (p[10] != 0);
    c.maxPayload = (uint16_t)p[11] | ((uint16_t)p[12] << 8);
    if (c.maxPayload < 8 || c.maxPayload > isotp::ISOTP_BUF_MAX) return false;
    if (p[13] > 5) return false;
    c.addrType = (isotp::AddrType)p[13];
    c.addrExt = p[14];
    isotp::setDefault(c);
    return true;
}

uint16_t serialize(const Profile& pr, uint8_t* out)
{
    // Fixed layout, no padding surprises (#pragma pack(1) on the struct).
    memcpy(out, &pr, sizeof(Profile));
    return sizeof(Profile);
}

bool deserialize(const uint8_t* in, uint16_t len, Profile& pr)
{
    if (len != sizeof(Profile)) return false;
    memcpy(&pr, in, sizeof(Profile));
    if (pr.version != PROFILE_VERSION) return false;
    pr.name[15] = '\0';
    if (pr.nFilters > MAX_PFILTERS || pr.nGwMaps > MAX_PGWMAPS) return false;
    if (pr.txPerm > 4 || pr.proto > PROTO_SWCAN) return false;
    return true;
}

} // namespace (anonymous)

void init()
{
    flashstore::init();
    uint8_t buf[sizeof(Profile)];
    uint16_t n = flashstore::load(buf, sizeof(buf));
    ramValid = (n > 0) && deserialize(buf, n, ram);
}

bool has() { return ramValid; }

const char* name()
{
    return ramValid ? ram.name : "";
}

bool save(const char* pname)
{
    if (!pname || !pname[0]) return false;
    Profile pr;
    memset(&pr, 0, sizeof(pr));
    pr.version = PROFILE_VERSION;
    strncpy(pr.name, pname, 15);
    pr.name[15] = '\0';

    // Capture live settings.
    modes::ModeManager* mgr = modes::manager();
    uint8_t mode;
    uint32_t c0, c1, sw;
    uint8_t feat;
    if (mgr) {
        mgr->mmGetStatus(mode, c0, c1, sw, feat);
        pr.can0Baud = c0;
        pr.can1Baud = c1;
        pr.swcanSpeed = sw;
    } else {
        pr.can0Baud = 500000;
        pr.can1Baud = 500000;
        pr.swcanSpeed = 33333;
    }
    pr.proto = PROTO_CAN;
    pr.frameFmt = 0;
    pr.diagAddr = 0;
    captureIsotp(pr.isotpCfg);

    pr.nFilters = 0;
    for (uint8_t i = 0; i < busfilter::count() && pr.nFilters < MAX_PFILTERS;
         i++) {
        if (busfilter::pack(i, pr.filters[pr.nFilters]))
            pr.nFilters++;
    }

    pr.nGwMaps = 0;
    {
        uint8_t nTotal = 0, nRet = 0;
        uint16_t version = 0;
        // mmMapList packs 22B entries: [idx][21B payload]. Walk the table.
        for (uint8_t s = 0; s < 8 && pr.nGwMaps < MAX_PGWMAPS; s++) {
            uint8_t out[22];
            if (gateway::mmMapList(s, out, nTotal, nRet, version) != 0 ||
                nRet == 0)
                break;
            memcpy(pr.gwMaps[pr.nGwMaps], out + 1, 21);
            pr.nGwMaps++;
            if (s + 1 >= nTotal) break;
        }
    }

    pr.txPerm = (uint8_t)capgate::current();
    pr.logMode = 1;   // on-demand (logrec default)
    pr.retryLim = 3;
    pr.reserved = 0;

    uint8_t buf[sizeof(Profile)];
    uint16_t n = serialize(pr, buf);
    if (!flashstore::save(buf, n)) {
        dbg::println(F("[profiles] save: flashstore FAILED"));
        return false;
    }
    ram = pr;
    ramValid = true;
    dbg::print(F("[profiles] saved '"));
    dbg::print(pr.name);
    dbg::println(F("'"));
    return true;
}

bool load()
{
    if (!ramValid) return false;
    modes::ModeManager* mgr = modes::manager();
    uint8_t applied = 0;
    if (mgr) {
        // Staged: applied=0 path just records; modes pick it up on entry.
        // If a mode currently owns the bus, mmSetCanBaud applies live —
        // same semantics as the web GUI path (documented, not special).
        mgr->mmSetCanBaud(0, ram.can0Baud, applied);
        mgr->mmSetCanBaud(1, ram.can1Baud, applied);
        mgr->mmSetSwcanSpeed(ram.swcanSpeed, applied);
    }
    if (!applyIsotp(ram.isotpCfg)) {
        dbg::println(F("[profiles] load: bad ISO-TP blob, skipped"));
    }

    busfilter::clear();
    for (uint8_t i = 0; i < ram.nFilters; i++) {
        busfilter::Filter f;
        const uint8_t* p = ram.filters[i];
        f.used = true;
        f.bus = (busfilter::Bus)p[1];
        f.proto = (busfilter::Proto)p[2];
        f.addr = (uint32_t)p[3] | ((uint32_t)p[4] << 8) |
                 ((uint32_t)p[5] << 16) | ((uint32_t)p[6] << 24);
        f.id = (uint32_t)p[7] | ((uint32_t)p[8] << 8) |
               ((uint32_t)p[9] << 16) | ((uint32_t)p[10] << 24);
        f.mask = (uint32_t)p[11] | ((uint32_t)p[12] << 8) |
                 ((uint32_t)p[13] << 16) | ((uint32_t)p[14] << 24);
        f.dir = (busfilter::Dir)p[15];
        f.payOff = p[16];
        f.payMask = p[17];
        f.payVal = p[18];
        f.invert = p[19] != 0;
        uint8_t idx = 0;
        // Reuse the mm add path shape: build a Filter and add directly.
        (void)idx;
        busfilter::add(f);
    }

    gateway::mmMapDel(0xFF, applied);  // clear all first
    for (uint8_t i = 0; i < ram.nGwMaps; i++) {
        uint8_t idx = 0;
        // MODIFY mappings need arm-confirm on the mm path; a profile load
        // is itself an explicit user action, but the arm is the second
        // factor — stage them via the normal path (first call stages).
        // Documented limitation: MODIFY entries need a second identical
        // load to arm-confirm. PASS entries apply immediately.
        gateway::mmMapAdd(ram.gwMaps[i], idx);
    }

    // Transmit permissions: escalate toward the stored ceiling through the
    // normal upward-only path (never silently, never downward).
    if (ram.txPerm > (uint8_t)capgate::current())
        capgate::escalate((capgate::Grant)ram.txPerm);

    dbg::print(F("[profiles] loaded '"));
    dbg::print(ram.name);
    dbg::println(F("'"));
    return true;
}

void clearProfile()
{
    ramValid = false;
    memset(&ram, 0, sizeof(ram));
    // Tombstone: save a zero-version record so a stale profile can't
    // resurrect after a power cycle.
    uint8_t buf[sizeof(Profile)];
    memset(buf, 0, sizeof(buf));
    flashstore::save(buf, sizeof(buf));
}

void show()
{
    SerialUSB.println(F("--- vehicle profile ---"));
    if (!ramValid) {
        SerialUSB.println(F("no stored profile"));
        return;
    }
    SerialUSB.print(F("name: "));
    SerialUSB.println(ram.name);
    SerialUSB.print(F("CAN0/CAN1/SWCAN: "));
    SerialUSB.print(ram.can0Baud);
    SerialUSB.print(F(" / "));
    SerialUSB.print(ram.can1Baud);
    SerialUSB.print(F(" / "));
    SerialUSB.println(ram.swcanSpeed);
    SerialUSB.print(F("proto: "));
    SerialUSB.print(ram.proto);
    SerialUSB.print(F(" diagAddr: 0x"));
    SerialUSB.println(ram.diagAddr, HEX);
    SerialUSB.print(F("filters: "));
    SerialUSB.print(ram.nFilters);
    SerialUSB.print(F(" gwMaps: "));
    SerialUSB.println(ram.nGwMaps);
    SerialUSB.print(F("txPerm: "));
    SerialUSB.print(capgate::name((capgate::Grant)ram.txPerm));
    SerialUSB.print(F(" logMode: "));
    SerialUSB.print(ram.logMode);
    SerialUSB.print(F(" retryLim: "));
    SerialUSB.println(ram.retryLim);
    SerialUSB.print(F("flash writes: "));
    SerialUSB.println(flashstore::writes());
}

bool info(char* outName, uint8_t& version, uint32_t& writes)
{
    if (!ramValid) return false;
    strncpy(outName, ram.name, 16);
    version = ram.version;
    writes = flashstore::writes();
    return true;
}

} // namespace profiles
