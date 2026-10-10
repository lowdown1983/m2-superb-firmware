#pragma once
// M2 multi-mode firmware — diagnostic session logging (Phase 12).
//
// The M2 produces compact log records into a RAM ring (pre/post-trigger
// buffering); the SuperB POLLS them over the Bee mm channel
// (CMD_LOG_POLL, 2 records per 48-byte reply) and persists to SPIFFS
// with rotation (see the Phase 11 notes — the M2 has no SD card, so the
// M2->Bee->SuperB path is the sane one).
//
// Record (23 bytes, packed):
//   [tsMs LE32][bus][dir][kind][ecuAddr][svc LE16][val LE32][rawLen][raw8]
// bus = busfilter::Bus encoding; dir 0=RX 1=TX; kind 0=CAN 1=OBD 2=UDS
// 3=DBC-signal; svc = service/DID/PID (or CAN ID low 16 for kind 0);
// val = decoded value as scaled int32 (0 when none); raw = raw bytes.
//
// Formats (native binary is stored; the rest are converted at download
// on the SuperB /logs page):
//  - native binary: the 23-byte records, verbatim
//  - GVRET: 0xF1 binary frames (M2RET emitFrame shape)
//  - CRTD: "(ts) canN ID#data" text lines (SavvyCAN capture format)
//  - CSV: ts_ms,bus,dir,kind,ecu,svc,val,rawhex
//
// Triggers: kind/svc match, manual arm (pre=N records kept, post=M
// recorded then auto-stop), or always-on. Overwrites count as losses.
//
// Privacy: records may contain VINs / vehicle data. Logs stay LOCAL
// (SPIFFS + web download only); nothing is uploaded anywhere.
//
// Evidence: VERIFIED = compiles. INFERRED = record/trigger behavior
// untested against a vehicle.

#include <Arduino.h>
#include <stdint.h>

namespace logrec {

constexpr uint8_t REC_BYTES = 23;
constexpr uint8_t RING_N = 48;     // RAM ring: 48 * 23 = 1104 bytes
// Records per CMD_LOG_POLL reply: 1. The mm reply buffer is 48 bytes
// (MAX_PAYLOAD) and a reply is 3 + nRec*23 — 2 records would be 49 bytes,
// a 1-byte stack overflow. The SuperB drains with repeated polls.
constexpr uint8_t PER_POLL = 1;

enum : uint8_t {
    KIND_CAN = 0,
    KIND_OBD = 1,
    KIND_UDS = 2,
    KIND_DBC = 3,
};

enum : uint8_t {
    DIR_RX = 0,
    DIR_TX = 1,
};

// Capture control.
void start();             // begin capturing (honors trigger config)
void stop();              // stop capturing
void clear();             // drop the ring + reset counters
bool active();

// Trigger: capture only records matching (kind, svc); svc 0xFFFF = any.
// Pass kind 0xFF to disable trigger filtering (capture everything).
void setTrigger(uint8_t kind, uint16_t svc);
void clearTrigger();

// Pre/post: keep the last `pre` records on trigger fire, then record
// `post` more and auto-stop. pre=0/post=0 = continuous while active.
void setPrePost(uint8_t pre, uint16_t post);

// Emit one record (drops when inactive, unless always-armed pre-buffer).
void record(uint8_t bus, uint8_t dir, uint8_t kind, uint8_t ecuAddr,
            uint16_t svc, int32_t val, const uint8_t* raw, uint8_t rawLen);

// Poll interface for the SuperB (CMD_LOG_POLL): copies up to PER_POLL
// oldest records into out (PER_POLL*REC_BYTES), oldest first. Returns
// the number of records copied; they are consumed (removed from ring).
uint8_t poll(uint8_t* out);

// Counters.
uint32_t produced();   // records ever produced
uint32_t dropped();     // lost to ring overwrite
uint32_t streamed();   // consumed via poll()
uint16_t pending();    // currently buffered

// Human-readable status on SerialUSB.
void status();

} // namespace logrec
