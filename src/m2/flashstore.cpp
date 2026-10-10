// M2 multi-mode firmware — flash-page emulated EEPROM (Phase 12).
// See flashstore.h for the backing-store documentation.

#include "flashstore.h"
#include "debug.h"

#ifndef SIM_M2
#include <sam.h>  // EFC1, IFLASH1_ADDR, EEFC_FCR_* (CMSIS-Atmel)
#endif

namespace flashstore {
namespace {

#ifndef SIM_M2
// Last two pages of IFLASH1 (EFC1 manages 0x000C0000-0x000FFFFF).
constexpr uint32_t SLOT_A_ADDR = IFLASH1_ADDR + IFLASH1_SIZE - 2 * PAGE_BYTES;
constexpr uint32_t SLOT_B_ADDR = IFLASH1_ADDR + IFLASH1_SIZE - 1 * PAGE_BYTES;
// EFC1-relative page numbers (FARG is per-EFC).
constexpr uint16_t SLOT_A_PAGE =
    (uint16_t)((SLOT_A_ADDR - IFLASH1_ADDR) / PAGE_BYTES);
constexpr uint16_t SLOT_B_PAGE =
    (uint16_t)((SLOT_B_ADDR - IFLASH1_ADDR) / PAGE_BYTES);
#endif

int8_t activeSlot = -1;   // 0 = A, 1 = B, -1 = none valid
uint32_t saveCounter = 0;
uint8_t pageBuf[PAGE_BYTES];

#ifndef SIM_M2
void readPage(uint32_t addr, uint8_t* out)
{
    const volatile uint8_t* p = (const volatile uint8_t*)addr;
    for (uint16_t i = 0; i < PAGE_BYTES; i++) out[i] = p[i];
}

// Erase + write one full page from pageBuf. IRQs masked: the EFC stalls
// the bus while FRDY is low, and an interrupt vector fetch mid-sequence
// would wedge. Latch the page with 32-bit writes, then EWP.
bool writePage(uint32_t addr, uint16_t efcPage)
{
    __disable_irq();
    volatile uint32_t* p = (volatile uint32_t*)addr;
    const uint32_t* src = (const uint32_t*)pageBuf;
    for (uint16_t i = 0; i < PAGE_BYTES / 4; i++) p[i] = src[i];
    // NOTE: the Arduino SAM core's CMSIS names the command field
    // EEFC_FCR_FCMD(value) (0x3 = Erase Page and Write Page); the
    // EEFC_FCR_FCMD_EWP named constant only exists in the standalone
    // framework-cmsis-atmel headers.
    EFC1->EEFC_FCR = EEFC_FCR_FKEY(0x5A) | EEFC_FCR_FARG(efcPage) |
                     EEFC_FCR_FCMD(0x3);
    while (!(EFC1->EEFC_FSR & EEFC_FSR_FRDY)) { /* stall with bus */ }
    __enable_irq();
    // Read-back verify.
    const volatile uint32_t* rp = (const volatile uint32_t*)addr;
    for (uint16_t i = 0; i < PAGE_BYTES / 4; i++) {
        if (rp[i] != src[i]) return false;
    }
    return true;
}
#endif

bool headerOk(const uint8_t* page, Header& h)
{
    h.magic = (uint32_t)page[0] | ((uint32_t)page[1] << 8) |
              ((uint32_t)page[2] << 16) | ((uint32_t)page[3] << 24);
    h.version = (uint16_t)page[4] | ((uint16_t)page[5] << 8);
    h.len = (uint16_t)page[6] | ((uint16_t)page[7] << 8);
    h.counter = (uint32_t)page[8] | ((uint32_t)page[9] << 8) |
                ((uint32_t)page[10] << 16) | ((uint32_t)page[11] << 24);
    h.crc32 = (uint32_t)page[12] | ((uint32_t)page[13] << 8) |
              ((uint32_t)page[14] << 16) | ((uint32_t)page[15] << 24);
    if (h.magic != MAGIC || h.version != FORMAT_VERSION) return false;
    if (h.len > PAYLOAD_MAX) return false;
    return crc32(page + HDR_BYTES, h.len) == h.crc32;
}

} // namespace (anonymous)

uint32_t crc32(const uint8_t* data, uint16_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320UL : (crc >> 1);
    }
    return crc ^ 0xFFFFFFFFUL;
}

bool init()
{
#ifdef SIM_M2
    activeSlot = -1;
    return false;  // host has no EFC; profiles stay RAM-only in sim
#else
    uint8_t pa[PAGE_BYTES], pb[PAGE_BYTES];
    readPage(SLOT_A_ADDR, pa);
    readPage(SLOT_B_ADDR, pb);
    Header ha, hb;
    bool oka = headerOk(pa, ha);
    bool okb = headerOk(pb, hb);
    if (oka && okb) {
        activeSlot = (hb.counter > ha.counter) ? 1 : 0;
        saveCounter = (hb.counter > ha.counter) ? hb.counter : ha.counter;
    } else if (oka) {
        activeSlot = 0;
        saveCounter = ha.counter;
    } else if (okb) {
        activeSlot = 1;
        saveCounter = hb.counter;
    } else {
        activeSlot = -1;
        saveCounter = 0;
    }
    return activeSlot >= 0;
#endif
}

bool save(const uint8_t* data, uint16_t len)
{
    if (!data || len == 0 || len > PAYLOAD_MAX) return false;
#ifdef SIM_M2
    return false;
#else
    // Write to the INACTIVE slot; it becomes active only after verify.
    int8_t target = (activeSlot == 0) ? 1 : 0;
    uint32_t addr = (target == 0) ? SLOT_A_ADDR : SLOT_B_ADDR;
    uint16_t page = (target == 0) ? SLOT_A_PAGE : SLOT_B_PAGE;
    saveCounter++;

    for (uint16_t i = 0; i < PAGE_BYTES; i++) pageBuf[i] = 0xFF;
    pageBuf[0] = (uint8_t)(MAGIC & 0xFF);
    pageBuf[1] = (uint8_t)((MAGIC >> 8) & 0xFF);
    pageBuf[2] = (uint8_t)((MAGIC >> 16) & 0xFF);
    pageBuf[3] = (uint8_t)((MAGIC >> 24) & 0xFF);
    pageBuf[4] = (uint8_t)(FORMAT_VERSION & 0xFF);
    pageBuf[5] = (uint8_t)((FORMAT_VERSION >> 8) & 0xFF);
    pageBuf[6] = (uint8_t)(len & 0xFF);
    pageBuf[7] = (uint8_t)((len >> 8) & 0xFF);
    pageBuf[8] = (uint8_t)(saveCounter & 0xFF);
    pageBuf[9] = (uint8_t)((saveCounter >> 8) & 0xFF);
    pageBuf[10] = (uint8_t)((saveCounter >> 16) & 0xFF);
    pageBuf[11] = (uint8_t)((saveCounter >> 24) & 0xFF);
    uint32_t crc = crc32(data, len);
    pageBuf[12] = (uint8_t)(crc & 0xFF);
    pageBuf[13] = (uint8_t)((crc >> 8) & 0xFF);
    pageBuf[14] = (uint8_t)((crc >> 16) & 0xFF);
    pageBuf[15] = (uint8_t)((crc >> 24) & 0xFF);
    for (uint16_t i = 0; i < len; i++) pageBuf[HDR_BYTES + i] = data[i];

    if (!writePage(addr, page)) {
        dbg::println(F("[flashstore] save: EFC write/verify FAILED"));
        return false;
    }
    // Re-read the header through the normal path before trusting it.
    uint8_t verify[PAGE_BYTES];
    readPage(addr, verify);
    Header h;
    if (!headerOk(verify, h)) {
        dbg::println(F("[flashstore] save: post-write CRC FAILED"));
        return false;
    }
    activeSlot = target;
    return true;
#endif
}

uint16_t load(uint8_t* out, uint16_t outCap)
{
    if (activeSlot < 0 || !out) return 0;
#ifdef SIM_M2
    return 0;
#else
    uint32_t addr = (activeSlot == 0) ? SLOT_A_ADDR : SLOT_B_ADDR;
    uint8_t page[PAGE_BYTES];
    readPage(addr, page);
    Header h;
    if (!headerOk(page, h)) return 0;
    if (h.len > outCap) return 0;
    for (uint16_t i = 0; i < h.len; i++) out[i] = page[HDR_BYTES + i];
    return h.len;
#endif
}

bool valid() { return activeSlot >= 0; }
uint32_t writes() { return saveCounter; }

} // namespace flashstore
