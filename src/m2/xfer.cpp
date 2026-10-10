// M2 multi-mode firmware — shared download engine (Phase 9).
// Logic moved verbatim from PROG mode (Phase 5); the modes supply Io.

#include "xfer.h"

namespace xfer {

namespace {

uint32_t crc32tab[256];
bool crcTabInit = false;

void noteFmt(Io& io, const char* a, uint32_t v, const char* b)
{
    // tiny formatter to avoid printf in the shared module
    char msg[80];
    size_t p = 0;
    while (*a && p + 1 < sizeof(msg)) msg[p++] = *a++;
    // decimal v
    char tmp[12]; uint8_t n = 0;
    if (v == 0) tmp[n++] = '0';
    else { char rev[12]; uint8_t m = 0; while (v && m < sizeof(rev)) { rev[m++] = (char)('0' + v % 10); v /= 10; } while (m) tmp[n++] = rev[--m]; }
    for (uint8_t i = 0; i < n && p + 1 < sizeof(msg); i++) msg[p++] = tmp[i];
    while (*b && p + 1 < sizeof(msg)) msg[p++] = *b++;
    msg[p] = '\0';
    if (io.note) io.note(io.ctx, msg);
}

} // namespace (anonymous)

void reset(State& s) { s = State(); }

void open(State& s, uint32_t maxBlockLen, uint32_t nowMs)
{
    reset(s);
    s.open = true;
    s.maxBlockLen = maxBlockLen;
    s.lastChunkAt = nowMs;
}

uint8_t feedChunk(State& s, Io& io, uint16_t seq, uint8_t flags,
                  const uint8_t* data, uint8_t len, uint16_t crc,
                  uint16_t& ackSeq)
{
    ackSeq = seq;
    if (!s.open || s.streamDone) {
        if (io.note) io.note(io.ctx, "chunk rejected: no open download");
        return 2;
    }
    if (seq != s.expectedSeq) {
        char msg[64];
        snprintf(msg, sizeof(msg), "sequence error: got %u, expected %u",
                 seq, s.expectedSeq);
        abort(s, io, msg);
        return 1;
    }
    if (len > CHUNK_MAX || crc16(data, len) != crc) {
        abort(s, io, "chunk CRC16 mismatch (link corruption?)");
        return 1;
    }
    // m8 fix (review): the ECU's advertised max block length is now
    // ENFORCED, not just warned about. Sending an oversized 0x36 relied
    // on ECU rejection; fail-closed instead.
    if (s.maxBlockLen > 0 && (uint32_t)len + 2 > s.maxBlockLen) {
        abort(s, io, "chunk exceeds ECU max block length");
        return 1;
    }
    if (len == 0) {
        // Zero-length LAST chunk: pure end-of-stream marker. No 0x36.
        if (flags & 0x01) {
            s.streamDone = true;
            s.expectedSeq++;
            s.lastChunkAt = millis();
            noteFmt(io, "stream complete: ", s.chunksRx, " chunks");
            return 0;
        }
        abort(s, io, "zero-length non-LAST chunk");
        return 1;
    }
    // The 0x36 frame itself is built by io.transferData (mode-specific
    // transport); the engine only verifies the 0x76 echo here.
    uint8_t r[16]; uint8_t rl = 0;
    if (!io.transferData || !io.transferData(io.ctx, s.blockSeq, data, len, r, sizeof(r), rl)) {
        abort(s, io, "0x36 TransferData failed (NRC/timeout)");
        return 2;
    }
    // Positive: [0x76, blockSequenceCounter echo] — verify the echo.
    if (rl < 2 || r[0] != 0x76 || r[1] != s.blockSeq) {
        abort(s, io, "0x76 blockSequenceCounter echo mismatch");
        return 2;
    }
    s.txCrc32 = crc32feed(s.txCrc32, data, len);
    s.bytesRx += len;
    s.chunksRx++;
    s.expectedSeq++;
    s.blockSeq++;  // uint8_t wraps 0xFF->0x00 per ISO 14229-1
    s.lastChunkAt = millis();
    if (flags & 0x01) {
        s.streamDone = true;
        noteFmt(io, "stream complete: ", s.chunksRx, " chunks");
    }
    return 0;
}

bool finish(State& s, Io& io, uint32_t fileCrc32)
{
    if (!s.open || !s.streamDone) {
        if (io.note) io.note(io.ctx, "no complete stream to finalize");
        return false;
    }
    uint32_t finalCrc = s.txCrc32 ^ 0xFFFFFFFFUL;
    char msg[64];
    snprintf(msg, sizeof(msg), "TX CRC32 0x%08lX vs file 0x%08lX",
             (unsigned long)finalCrc, (unsigned long)fileCrc32);
    if (io.note) io.note(io.ctx, msg);
    if (finalCrc != fileCrc32) {
        abort(s, io, "end-to-end CRC32 MISMATCH — image rejected, NOT finalized");
        return false;
    }
    if (!io.transferExit || !io.transferExit(io.ctx)) {
        abort(s, io, "0x37 RequestTransferExit failed");
        return false;
    }
    reset(s);
    if (io.note) io.note(io.ctx, "transfer complete and verified");
    return true;
}

void abort(State& s, Io& io, const char* reason)
{
    // Best-effort 0x37 if a download was open (terminates the data
    // transfer on the ECU side per ISO 14229-1).
    if (s.open && io.transferExit) io.transferExit(io.ctx);
    reset(s);
    if (io.note) {
        char msg[96];
        snprintf(msg, sizeof(msg), "ABORT: %s", reason ? reason : "?");
        io.note(io.ctx, msg);
    }
}

bool watchdog(State& s, Io& io, uint32_t nowMs)
{
    if (s.open && !s.streamDone &&
        (uint32_t)(nowMs - s.lastChunkAt) >= CHUNK_WD_MS) {
        abort(s, io, "chunk watchdog: no data for 30 s");
        return true;
    }
    return false;
}

uint16_t crc16(const uint8_t* d, uint8_t n)
{
    uint16_t c = 0xFFFF;
    for (uint8_t i = 0; i < n; i++) {
        c ^= (uint16_t)d[i] << 8;
        for (uint8_t b = 0; b < 8; b++)
            c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
    }
    return c;
}

uint32_t crc32feed(uint32_t crc, const uint8_t* d, uint8_t n)
{
    // UDS-XFER-001: table init is a shared-engine invariant, not a PROG
    // side effect. crcInit() is idempotent (crcTabInit guard); the
    // single-threaded firmware makes the lazy check safe. This covers
    // UDS mode (which never called crcInit) and every future caller.
    crcInit();
    for (uint8_t i = 0; i < n; i++)
        crc = crc32tab[(crc ^ d[i]) & 0xFF] ^ (crc >> 8);
    return crc;
}

void crcInit()
{
    if (crcTabInit) return;
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (uint8_t b = 0; b < 8; b++)
            c = (c & 1) ? (0xEDB88320UL ^ (c >> 1)) : (c >> 1);
        crc32tab[i] = c;
    }
    crcTabInit = true;
}

} // namespace xfer
