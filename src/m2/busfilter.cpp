// M2 multi-mode firmware — unified bus filter model (Phase 6).
// See busfilter.h for the contract and the Phase 9/12 seam note.

#include "busfilter.h"

namespace busfilter {

namespace {
Filter table[MAX_FILTERS];
} // namespace

bool matchOne(const Filter& f, Bus bus, Proto proto, uint32_t addr,
              uint32_t id, Dir dir, const uint8_t* pl, uint8_t pln)
{
    if (!f.used) return false;
    bool ok = true;
    if (f.bus != Bus::ANY && f.bus != bus) ok = false;
    if (f.proto != Proto::ANY && f.proto != proto) ok = false;
    if (f.addr != ADDR_ANY && f.addr != addr) ok = false;
    if (!idMatch(id, f.id, f.mask)) ok = false;
    if (f.dir != Dir::BOTH && f.dir != dir) ok = false;
    if (f.payMask != 0) {
        if (pl == nullptr || pln <= f.payOff) {
            ok = false;
        } else if (((pl[f.payOff] ^ f.payVal) & f.payMask) != 0) {
            ok = false;
        }
    }
    return f.invert ? !ok : ok;
}

bool accept(Bus bus, Proto proto, uint32_t addr, uint32_t id, Dir dir,
            const uint8_t* pl, uint8_t pln)
{
    bool any = false;
    for (uint8_t i = 0; i < MAX_FILTERS; i++) {
        if (!table[i].used) continue;
        any = true;
        if (matchOne(table[i], bus, proto, addr, id, dir, pl, pln))
            return true;
    }
    return !any;  // empty table: accept everything (modes unchanged)
}

int8_t add(const Filter& f)
{
    for (uint8_t i = 0; i < MAX_FILTERS; i++) {
        if (!table[i].used) {
            table[i] = f;
            table[i].used = true;
            return (int8_t)i;
        }
    }
    return -1;
}

bool remove(uint8_t idx)
{
    if (idx >= MAX_FILTERS || !table[idx].used) return false;
    table[idx].used = false;
    return true;
}

void clear()
{
    for (uint8_t i = 0; i < MAX_FILTERS; i++) table[i].used = false;
}

uint8_t count()
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < MAX_FILTERS; i++) if (table[i].used) n++;
    return n;
}

const Filter* get(uint8_t idx)
{
    if (idx >= MAX_FILTERS || !table[idx].used) return nullptr;
    return &table[idx];
}

uint8_t pack(uint8_t idx, uint8_t* out)
{
    const Filter* f = get(idx);
    if (f == nullptr || out == nullptr) return 0;
    out[0] = idx;
    out[1] = (uint8_t)f->bus;
    out[2] = (uint8_t)f->proto;
    out[3] = (uint8_t)(f->addr & 0xFF);
    out[4] = (uint8_t)((f->addr >> 8) & 0xFF);
    out[5] = (uint8_t)((f->addr >> 16) & 0xFF);
    out[6] = (uint8_t)((f->addr >> 24) & 0xFF);
    out[7] = (uint8_t)(f->id & 0xFF);
    out[8] = (uint8_t)((f->id >> 8) & 0xFF);
    out[9] = (uint8_t)((f->id >> 16) & 0xFF);
    out[10] = (uint8_t)((f->id >> 24) & 0xFF);
    out[11] = (uint8_t)(f->mask & 0xFF);
    out[12] = (uint8_t)((f->mask >> 8) & 0xFF);
    out[13] = (uint8_t)((f->mask >> 16) & 0xFF);
    out[14] = (uint8_t)((f->mask >> 24) & 0xFF);
    out[15] = (uint8_t)f->dir;
    out[16] = f->payOff;
    out[17] = f->payMask;
    out[18] = f->payVal;
    out[19] = f->invert ? 1 : 0;
    return 20;
}

} // namespace busfilter
