#pragma once
// M2 multi-mode firmware — flash-page emulated EEPROM (Phase 12).
//
// The SAM3X8E has NO real EEPROM and the Arduino SAM core ships no EEPROM
// emulation. This module implements the honest equivalent: the LAST TWO
// flash pages (0x000FFC00 slot A, 0x000FFE00 slot B — IFLASH1, EFC1) hold
// one A/B pair of CRC-protected records. Saves go to the inactive slot;
// the active slot is the newest valid one. A power loss mid-save can only
// strand the inactive slot — the previous record stays valid.
//
// Backing store (documented, not hidden):
//  - 2 x 512-byte flash pages at the top of IFLASH1 (EFC1 page 510/511).
//  - Writes use EEFC "Erase Page and Write Page" (EWP) with IRQs disabled
//    (the EFC stalls the bus while FRDY is low).
//  - Wear: Atmel rates the embedded flash program/erase endurance in the
//    low tens of thousands of cycles (INFERRED — datasheet figure not
//    re-verified in this phase). Mitigations: saves are EXPLICIT only
//    (`profile save`, never background), A/B means each save costs one
//    page cycle, and writes() exposes the cycle counter.
//  - Size limit: PROFILE_MAX (496 - header) bytes per record. Firmware
//    must stay below 0x000FFC00 (510 KB) — we are at ~27%, checked at
//    build time by the developer (see PHASE12_NOTES.md).
//  - The sim build (#ifndef SIM_M2 guard) stubs this out: host has no EFC.
//
// Evidence: VERIFIED = compiles. INFERRED = EFC sequence untested on
// silicon in this phase (no flash writes exercised on the M2 yet); every
// save does a read-back + CRC verify, so a bad write is detected loudly.

#include <Arduino.h>
#include <stdint.h>

namespace flashstore {

constexpr uint16_t PAGE_BYTES = 512;
constexpr uint16_t HDR_BYTES = 16;
constexpr uint16_t PAYLOAD_MAX = PAGE_BYTES - HDR_BYTES;  // 496
constexpr uint32_t MAGIC = 0x50524653UL;  // 'PRFS'
constexpr uint16_t FORMAT_VERSION = 1;

// Slot header, 16 bytes, little-endian.
struct Header {
    uint32_t magic;    // MAGIC
    uint16_t version;  // FORMAT_VERSION
    uint16_t len;      // payload bytes used
    uint32_t counter;  // monotonic save counter (wear + newest-wins)
    uint32_t crc32;    // CRC32 over the payload bytes
};

// init(): read both slots, select the newest valid one. Returns true when
// at least one valid slot exists.
bool init();

// save(): write `len` payload bytes to the inactive slot, then read back
// and CRC-verify. Returns true on verified success. IRQs are masked during
// the EFC sequence (~ms). Explicit saves only — never called in the
// background.
bool save(const uint8_t* data, uint16_t len);

// load(): copy the active slot's payload into out (outCap must be >=
// payload len). Returns the payload length, or 0 when no valid slot.
uint16_t load(uint8_t* out, uint16_t outCap);

// valid(): a good slot is selected. writes(): total verified saves.
bool valid();
uint32_t writes();

// CRC32 (IEEE 802.3) — shared with the profile layer.
uint32_t crc32(const uint8_t* data, uint16_t len);

} // namespace flashstore
