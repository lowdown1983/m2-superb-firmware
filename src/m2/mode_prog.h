#pragma once
// M2 multi-mode firmware — PROG (programming) mode (Phase 5).
//
// ECU flashing/tuning over UDS (ISO 14229-1 service semantics — cited,
// never invented) on CAN ISO-TP (ISO 15765-2 via isotp.h).
//
// Flow: explicit addressing config -> 0x10 programming session -> 0x27
// security access (STUB: user supplies the key; no algorithm invented) ->
// optional 0x31 erase routine -> 0x34 RequestDownload -> 0x36 TransferData
// blocks (streamed from the SuperB over the F1 7E channel, one chunk per
// block, no full-image RAM buffer) -> 0x37 RequestTransferExit -> 0x11
// reset. Optional 0x23 read-back verify (INFERRED availability).
//
// READ docs/SAFETY_REVIEW.md FIRST. It is the safety case; this file
// implements it. Interlocks enforced here:
//   (a) destructive steps need `prog arm` (console, 120 s INFERRED window)
//       or CMD_PROG_ARM (web, per-step checkbox). `prog abort` never
//       needs arm.
//   (b) mode entry goes through requestMode()->quiesceBuses(); entry also
//       auto-suspends the MITM rule table (mitm::suspendRules) with an
//       explicit notice; exit does NOT restore rules.
//   (c) destructive steps need the `configured` flag (explicit bus/tx/rx).
//   Power: `prog power ok` / CMD_PROG_ARM power bit attests a stable 12V+
//   supply. ATRV is an uncalibrated placeholder (Phase 2) and gates
//   NOTHING. This limitation is documented, not worked around.
// Abort: any error mid-transfer -> best-effort 0x37, state cleared,
// disarmed, LOUD console + status reporting. Chunk watchdog (30 s,
// INFERRED) aborts a stalled upload. Power loss mid-flash is
// unrecoverable by firmware (see safety review).
//
// Console (SerialUSB): "prog ..." — config works any time (staged);
// bus-touching commands require PROG mode active.
//   prog                 help (prints the safety-review pointer)
//   prog status          config + state + counters + CRCs
//   prog bus 0|1 | prog baud <n> | prog tx <hex> | prog rx <hex>
//   prog arm             arm destructive steps (120 s)
//   prog power ok        attest stable 12V+ supply (required: erase/download)
//   prog session         0x10 0x02 programming session
//   prog seed <level>    0x27 requestSeed (prints seed; no arm needed)
//   prog key <hex...>    0x27 sendKey, user-supplied key (ARMED)
//   prog erase <ridHex>  0x31 startRoutine (ARMED + power)
//   prog download <addrHex> <lenHex> [alfidHex]   0x34 (ARMED + power)
//   prog verify <addrHex> <lenHex> [alfidHex]     0x23 read-back CRC compare
//   prog done <fileCrc32Hex>  compare + 0x37 (ARMED)
//   prog reset           0x11 0x01 hard reset (ARMED)
//   prog abort           abort sequence (never needs arm)
//
// Resource claims: the configured CAN bus (lazy acquire, like UDS) +
// UART0/Bee (never released). exit() releases before the manager's
// quiesceBuses(). CAN ONLY (11-bit, like UDS Phase 3).
//
// Evidence: VERIFIED = compiles / ISO 14229-1 + ISO 15765-2 public
// standard semantics. INFERRED = untested against real ECUs (no bench).

#include <Arduino.h>

namespace prog {

// PROG transfer states (reported via CMD_PROG_STATUS).
enum State : uint8_t {
    ST_IDLE = 0,
    ST_SESSION = 1,     // 0x10 programming session positive
    ST_UNLOCKED = 2,    // 0x27 sendKey positive
    ST_ERASED = 3,      // 0x31 routine completed (optional)
    ST_DOWNLOADING = 4, // 0x34 accepted; expecting CMD_PROG_DATA chunks
    ST_STREAM_DONE = 5, // all chunks received; awaiting PROG_DONE/done
    ST_DONE = 6,        // 0x37 positive; transfer complete
    ST_ABORTED = 7,
    ST_ERROR = 8,
};

void begin(uint8_t mitmSuspended);  // reset state; bus acquired lazily
void loop();    // tester-present keepalive + chunk watchdog + Serial pump
void exit();    // abort-if-active, release bus (idempotent)

// USB-console entry. `line` is the text after "prog".
void console(char* line);

// ---- mm-channel ops (manager delegates; see mm_channel.h) ----
// All return mm ST_* codes. mmTx enforces arm for destructive services
// (0x11/0x2E/0x31/0x34/0x36/0x37) on the M2 side; the console path also
// enforces arm in console().
uint8_t mmCfg(uint8_t bus, uint32_t baud, uint32_t txId, uint32_t rxId);
uint8_t mmTx(const uint8_t* svc, uint8_t svcLen,
             uint8_t* resp, uint8_t* respLen, uint8_t* nResp);
// Chunk upload: seq must equal the expected counter; flags bit0 = LAST;
// crc = CRC16-CCITT-FALSE over data[0..len). Each accepted chunk is sent
// immediately as one 0x36 TransferData (streaming; no image buffering).
// ackSeq = the accepted sequence number. Any violation aborts loudly.
uint8_t mmData(uint16_t seq, uint8_t flags, const uint8_t* data, uint8_t len,
               uint16_t crc, uint16_t& ackSeq);
// Finalize: compare accumulated TX CRC32 vs fileCrc32; on match send 0x37.
// match=1 on success; txCrc = accumulated CRC32 either way. Requires arm.
uint8_t mmDone(uint32_t fileCrc32, uint32_t& txCrc, uint8_t& match);
// Start a download: 0x34 RequestDownload + reset streaming state.
// Requires active + configured + armed + powerOk.
uint8_t mmDownload(uint32_t addr, uint32_t len, uint8_t alfid);
// Read-back verify: 0x23 over [addr, addr+len), CRC32 vs transmitted.
// match=1 on MATCH; rxCrc = read-back CRC32. Read-only (no arm needed).
// Availability INFERRED (ECU-dependent).
uint8_t mmVerify(uint32_t addr, uint32_t len, uint8_t alfid,
                 uint8_t& match, uint32_t& rxCrc);
void mmAbort();  // abort sequence (idempotent, never needs arm)
uint8_t mmArm(uint8_t powerOk, uint8_t& armed);
// out = 27-byte status blob: [state][session][bus][baud LE32][txId LE32]
// [rxId LE32][configured][powerOk][armed][bytesRx LE32][chunksRx LE16]
// [txCrc32 LE32][mitmSuspended]. (chunksTotal is SuperB-side; see web.)
void mmStatus(uint8_t* out);

} // namespace prog
