#pragma once
// M2 multi-mode firmware — shared ISO 14229-1 download engine (Phase 9).
//
// ONE implementation of the 0x34/0x36/0x37 streaming state machine,
// shared by PROG mode and UDS mode (roadmap: "refactor, don't duplicate").
// Chunk framing (sequence numbers, per-chunk CRC16, block-sequence
// counter, end-to-end CRC32, chunk watchdog) lives here; the mode
// supplies the transport via the Io callbacks and keeps its own safety
// interlocks (PROG: arm + power attestation + explicit config; UDS:
// capability gate + arm-confirm).
//
// Evidence: VERIFIED = compiles (logic moved verbatim from PROG Phase 5).
// INFERRED = untested against a real ECU.

#include <Arduino.h>
#include <stdint.h>

namespace xfer {

constexpr uint8_t CHUNK_MAX = 41;        // fits the 48-byte mm frame cap
constexpr uint32_t CHUNK_WD_MS = 30000; // stalled-upload watchdog (INFERRED)

struct State {
    uint16_t expectedSeq = 0;
    uint8_t blockSeq = 1;        // ISO 14229-1: starts at 1, wraps 0xFF->0x00
    uint32_t bytesRx = 0;
    uint32_t declaredLen = 0;    // F-03-01: image length from the 0x34
                                 // request; finish() requires bytesRx ==
                                 // declaredLen (0 = rejected at open).
    uint16_t chunksRx = 0;
    uint32_t txCrc32 = 0xFFFFFFFFUL;  // IEEE CRC32 over flashed bytes
    uint32_t maxBlockLen = 0;    // from 0x34 response (0 = ECU didn't say)
    uint32_t lastChunkAt = 0;
    bool open = false;           // 0x34 accepted, download in progress
    bool streamDone = false;     // LAST chunk received
};

// Mode-supplied transport. The mode owns its endpoint, session, and
// safety interlocks; xfer only moves bytes.
struct Io {
    void* ctx = nullptr;
    // Send [0x36, blockSeq, data...]; fill resp on any response.
    // Returns true when a response was received (positive or negative).
    bool (*transferData)(void* ctx, uint8_t blockSeq,
                         const uint8_t* data, uint8_t len,
                         uint8_t* resp, uint8_t respCap,
                         uint8_t& respLen) = nullptr;
    // Send 0x37 RequestTransferExit; true on positive response.
    bool (*transferExit)(void* ctx) = nullptr;
    // One user-visible message line (mode prints it its own way).
    void (*note)(void* ctx, const char* msg) = nullptr;
};

void reset(State& s);

// Call after a positive 0x34 (RequestDownload). declaredLen is the image
// length sent in the 0x34 request (F-03-01: the engine enforces it —
// chunks past it are rejected, finish() requires bytesRx == declaredLen).
// maxBlockLen from the 0x74 response (0 when the ECU didn't report one).
// Returns false (leaves the engine closed) on a zero declared length.
bool open(State& s, uint32_t declaredLen, uint32_t maxBlockLen, uint32_t nowMs);

// One download chunk -> one 0x36 TransferData, immediately (streaming,
// no image buffering). Returns 0=OK, 1=BAD_ARG, 2=ERR.
uint8_t feedChunk(State& s, Io& io, uint16_t seq, uint8_t flags,
                  const uint8_t* data, uint8_t len, uint16_t crc,
                  uint16_t& ackSeq);

// Finalize: compare end-to-end CRC32 against the file CRC, then 0x37.
// True when the transfer verified and exited cleanly.
bool finish(State& s, Io& io, uint32_t fileCrc32);

// Abort: best-effort 0x37, state cleared. The mode clears its own
// arm/session state separately.
void abort(State& s, Io& io, const char* reason);

// Chunk watchdog: call from the mode loop. Returns true when it aborted
// a stalled upload.
bool watchdog(State& s, Io& io, uint32_t nowMs);

uint16_t crc16(const uint8_t* d, uint8_t n);   // CRC16-CCITT-FALSE (per chunk)
uint32_t crc32feed(uint32_t crc, const uint8_t* d, uint8_t n);  // IEEE
void crcInit();

} // namespace xfer
