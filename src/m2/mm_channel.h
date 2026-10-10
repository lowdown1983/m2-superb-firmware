#pragma once
// M2 multi-mode firmware — SuperB<->M2 control channel (Phase 2).
//
// Framing (BOTH directions, ALL modes):
//     0xF1 0x7E LEN PAYLOAD[LEN] XOR
// XOR = XOR of all PAYLOAD bytes (GVRET checksumCalc style).
//
// Why this is safe in-band:
//   - 0xF1 is the GVRET sync byte; the command byte position in M2RET only
//     defines 0-14 and 0xE7 (M2RET.h enum GVRET_PROTOCOL @ ad9d5af;
//     CommProtocol.txt). 0x7E is never emitted by SavvyCAN — its connect
//     sequence is E7 E7 F1 0C F1 06 F1 07 F1 01 F1 09 (PHASE1_NOTES.md).
//   - Our GVRET parser resyncs to IDLE on unknown command bytes
//     (mode_m2ret.cpp GET_COMMAND default), and the 0x7E case below enters a
//     length-prefixed sub-state, so payload bytes can never be mistaken for
//     commands.
//   - In ELM mode the byte pump diverts F1 7E before line assembly; 0xF1
//     never appears in legitimate AT text.
//   - The SuperB filters app->M2 traffic and drops any F1 7E pair, so a
//     connected app (Torque/SavvyCAN/pen-test tool) can NEVER inject a
//     control frame. The M2 also only honors frames arriving on Serial.
//
// SAFETY INVARIANT: CMD_SET_MODE is dispatched exclusively through
// ModeManager::requestMode(), i.e. quiesceBuses() (all buses silent) then
// acquire. There is no backdoor around the safe-switch path.
//
// Evidence: VERIFIED = compiles / primary-source fact. INFERRED = untested
// on hardware.

#include <Arduino.h>

namespace mm {

constexpr uint8_t SYNC = 0xF1;
constexpr uint8_t CMD_MULTIMODE = 0x7E;
constexpr uint8_t MAX_PAYLOAD = 48;

// Commands (payload[0]). Replies use cmd|0x80 as payload[0].
enum : uint8_t {
    CMD_SET_MODE      = 0x01,  // req: [modeIdx]                      -> [status]
    CMD_GET_STATUS    = 0x02,  // req: []                             -> [status, mode, can0 LE32, can1 LE32, swcan LE32, features]
    CMD_SET_CAN_BAUD  = 0x03,  // req: [bus 0/1] [baud LE32]          -> [status, applied(1=live,0=staged)]
    CMD_SET_SWCAN_SPEED = 0x04,// req: [speed LE32]                   -> [status, applied]
    CMD_SET_FEATURE   = 0x05,  // req: [featureId] [0/1]              -> [status]
    // Phase 3: UDS + OBD2 supervision.
    CMD_UDS_TX        = 0x06,  // req: [svcLen] [svc bytes...]        -> [status, nResp, rLen, rData...] (first response; rest counted)
                              //   ST_ERR if UDS mode not active. Web GUI must
                              //   confirm 0x11/0x2E itself (M2 has no web-side arm).
    CMD_UDS_CFG       = 0x07,  // req: [bus 0/1][baud LE32][txId LE32][rxId LE32][func 0/1][tpMs LE16]
                              //   -> [status]. tpMs=0 disables tester-present.
                              //   Staged; bus re-acquired lazily on next command.
    CMD_OBD2_CFG      = 0x08,  // req: [bus 0/1][baud LE32][interval LE16][nPids][pid...]
                              //   -> [status]. Stops a running poller (safe).
    CMD_OBD2_CTL      = 0x09,  // req: [start 0/1]                    -> [status, running]
                              //   ST_ERR if OBD2 mode not active.
    CMD_UDS_STATUS    = 0x0A,  // req: []                             -> [status, session, bus, baud LE32, txId LE32, rxId LE32, func, tpOn]
    CMD_OBD2_STATUS   = 0x0B,  // req: []                             -> [status, running, bus, interval LE16, nPids]
    // Phase 4: MITM supervision.
    CMD_MITM_RULE_ADD = 0x0C,  // req: [id LE32][mask LE32][dir][action][modOff][modMask][modVal][rateLim LE16] (15)
                               //   -> [status, idx]. ST_ERR if the table is full.
    CMD_MITM_RULE_DEL = 0x0D,  // req: [idx] (0xFF = clear all)       -> [status, nRules]
    CMD_MITM_RULE_LIST= 0x0E,  // req: [startIdx]                      -> [status, nTotal, nRet, entries...]
                               //   each entry 20B: [idx][id LE32][mask LE32][dir][action]
                               //   [modOff][modMask][modVal][rateLim LE16][hits LE32].
                               //   Max 2 entries per reply (48-byte cap).
    CMD_MITM_STATS    = 0x0F,  // req: []                             -> [status, fwdAB LE32, fwdBA LE32,
                               //   dropAB LE32, dropBA LE32, modAB LE32, modBA LE32,
                               //   loopSupp LE32, rateDrop LE32, loadA, loadB,
                               //   dir, stealth, nRules]  (38 bytes total)
    CMD_MITM_CFG      = 0x10,  // WRITE req: [baudA LE32][baudB LE32][dir][stealth][loopWinMs LE16][defRateLim LE16] (14)
                               //   -> [status, applied]. Live-reinits the buses
                               //   when MITM is active, else staged.
                               // READ req: [] -> [status, baudA LE32, baudB LE32,
                               //   dir, stealth, loopWinMs LE16, defRateLim LE16,
                               //   active] (17 bytes total: current staged/live
                               //   config, so the web UI can prefill honestly).
    // Phase 5: PROG (programming) supervision. No M2-side arm on PROG_TX:
    // the web wizard confirms per step (same pattern as CMD_UDS_TX); the
    // console path enforces `prog arm` itself. READ docs/SAFETY_REVIEW.md.
    CMD_PROG_CFG      = 0x11,  // req: [bus][baud LE32][txId LE32][rxId LE32] (14)
                              //   -> [status]. Explicit addressing (interlock c);
                              //   bus re-acquired lazily.
    CMD_PROG_TX       = 0x12,  // req: [svcLen][svc...] -> [status, nResp, rLen,
                              //   rData...] (first response; rest counted).
                              //   ST_ERR if PROG mode not active.
    CMD_PROG_DATA     = 0x13,  // req: [seq LE16][flags][len][data...][crc16 LE16]
                              //   flags bit0=LAST. len <= 41 (48-byte cap).
                              //   -> [status, ackSeq LE16]. Each accepted chunk
                              //   becomes one 0x36 TransferData immediately
                              //   (streaming; no image buffering). CRC/seq
                              //   violation -> transfer ABORTED loudly.
    CMD_PROG_DONE     = 0x14,  // req: [fileCrc32 LE32] -> [status, match,
                              //   txCrc32 LE32]. Compares end-to-end CRC32;
                              //   on match sends 0x37 RequestTransferExit.
    CMD_PROG_STATUS   = 0x15,  // req: [] -> [status, state][session][bus]
                              //   [baud LE32][txId LE32][rxId LE32]
                              //   [configured][powerOk][armed][bytesRx LE32]
                              //   [chunksRx LE16][txCrc32 LE32][mitmSuspended]
                              //   (29 bytes total).
    CMD_PROG_ABORT    = 0x16,  // req: [] -> [status]. Abort sequence; never
                              //   needs arm.
    CMD_PROG_ARM      = 0x17,  // req: [powerOk 0/1] -> [status, armed].
                              //   Arms destructive steps (120 s, INFERRED);
                              //   optionally records power attestation.
    CMD_PROG_DL       = 0x18,  // req: [addr LE32][len LE32][alfid] (9)
                              //   -> [status]. 0x34 RequestDownload + reset
                              //   streaming state. Needs configured+armed+power.
    CMD_PROG_VERIFY   = 0x19,  // req: [addr LE32][len LE32][alfid] (9)
                              //   -> [status, match, rxCrc32 LE32]. 0x23
                              //   read-back CRC compare (INFERRED availability).
    // Phase 6: transport-layer supervision (scheduler, filters, stats,
    // ISO-TP config). Same pattern as the 0x06-0x0B supervision block.
    CMD_TXSCHED_ADD   = 0x1A,  // req: [bus][id LE32][ext][len][data..]
                              //   [periodMs LE32][offsetMs LE32][count LE32]
                              //   (20+len bytes; len<=8; count -1=infinite)
                              //   -> [status, slot]
    CMD_TXSCHED_DEL   = 0x1B,  // req: [slot] -> [status]
    CMD_TXSCHED_LIST  = 0x1C,  // req: [start] -> [status, nTotal, nRet, entries...]
                              //   25B/entry, 1 per reply (48-byte cap):
                              //   [slot][bus][id LE32][ext][len][period LE32]
                              //   [offset LE32][count LE32][active][fires LE32]
    CMD_TXSCHED_CTL   = 0x1D,  // req: [slot][start 0/1] -> [status, active]
    CMD_FILTER_ADD    = 0x1E,  // req: 20B [bus][proto][addr LE32][id LE32]
                              //   [mask LE32][dir][payOff][payMask][payVal][invert]
                              //   -> [status, idx]
    CMD_FILTER_DEL    = 0x1F,  // req: [idx] (0xFF = clear all) -> [status, nLeft]
    CMD_FILTER_LIST   = 0x20,  // req: [start] -> [status, nTotal, nRet, entries...]
                              //   20B/entry, 2 per reply: [idx][bus][proto]
                              //   [addr LE32][id LE32][mask LE32][dir]
                              //   [payOff][payMask][payVal][invert]
    CMD_BUS_STATS     = 0x21,  // req: [bus 0..4] -> [status, 32B blob:
                              //   0-1 native CAN, 2 K-line (Phase 7),
                              //   3 J1850, 4 SWCAN (Phase 8)]
                              //   txFrames/rxFrames/txBytes/rxBytes/
                              //   rxOverflow/txFail/busOff/dropped LE32]
                              //   (Phase 7: bus 2 = K-line)
    CMD_ISOTP_CFG     = 0x22,  // WRITE req: 15B [nBsMs LE16][nCrMs LE16]
                              //   [p2Ms LE16][p2StarMs LE16][bs][stMin]
                              //   [autoFc][maxPayload LE16][addrType][addrExt]
                              //   -> [status]. Sets the global default
                              //   ISO-TP config (used when a mode passes no
                              //   explicit cfg).
                              // READ req: [] -> [status, 15B blob as above].
    CMD_KLINE_STATUS  = 0x23,  // req: [] -> [status, acquired, initDone,
                              //   kb1, kb2, target, tester] (Phase 7).
    CMD_KLINE_INIT    = 0x24,  // req: [type 1=slow 5-baud, 2=fast]
                              //   -> [status, kb1, kb2]. Runs the init
                              //   sequence (TRANSMITS by construction;
                              //   refused unless ELM/STN mode is active).
    // Phase 9: discovery + capability grants. Note: CMD_DISCOVER_LIST is
    // 0x27 in the mm namespace only — unrelated to UDS service 0x27
    // (SecurityAccess), which is a different protocol's service byte.
    CMD_DISCOVER_START  = 0x25,  // req: [bus 0/1] -> [status]
    CMD_DISCOVER_STATUS = 0x26,  // req: [] -> [status, done, nEcus, baud LE32]
    CMD_DISCOVER_LIST   = 0x27,  // req: [startIdx]
                                // -> [status, nTotal, nRet, entries...]
                                // 11B/entry, 4 per reply (48-byte cap):
                                // [txId LE32][rxId LE32][nSvcs][vinOk]
                                // [nodeAddr]
    CMD_CAP_GRANT       = 0x28,  // WRITE req: [level 0..4]
                                // -> [status, grant]. Upward-only, prints
                                // a console warning, never silent.
                                // READ req: [0xFF] -> [status, grant].
    // Phase 10: gateway supervision (CAN0 <-> SWCAN mapping tables).
    CMD_GW_MAP_ADD      = 0x29,  // req: 21B [srcBus][srcId LE32][srcMask LE32]
                                //   [srcExt][dstBus][dstId LE32][dstExt]
                                //   [action][modOff][modMask][modVal][dropPolicy]
                                //   -> [status, idx]. ST_ERR if the table is
                                //   full. MODIFY mappings need
                                //   ALLOW_DIAGNOSTIC_WRITE + arm-confirm:
                                //   the first call stages (idx=0xFE), the
                                //   IDENTICAL call within 10 s installs.
    CMD_GW_MAP_DEL      = 0x2A,  // req: [idx] (0xFF = clear all)
                                //   -> [status, nMaps]
    CMD_GW_MAP_LIST     = 0x2B,  // req: [start]
                                //   -> [status, nTotal, nRet, version LE16,
                                //   entries...] each entry 22B:
                                //   [idx][srcBus][srcId LE32][srcMask LE32]
                                //   [srcExt][dstBus][dstId LE32][dstExt]
                                //   [action][modOff][modMask][modVal][dropPolicy]
                                //   1 entry per reply (48-byte cap).
    CMD_GW_STATS        = 0x2C,  // req: []
                                //   -> [status, fwdCS LE32, fwdSC LE32,
                                //   dropCS LE32, dropSC LE32, modCS LE32,
                                //   modSC LE32, version LE16, nMaps,
                                //   suspended, active] (30 bytes total:
                                //   status + 29-byte blob)
    CMD_GW_CFG          = 0x2D,  // WRITE req: [canBaud LE32][swcanSpeed LE32]
                                //   [dropPolicy][drainPerLoop] (10)
                                //   -> [status, applied]. Live-reinits the
                                //   buses when GATEWAY is active, else staged.
                                // READ req: [] -> [status, canBaud LE32,
                                //   swcanSpeed LE32, dropPolicy,
                                //   drainPerLoop, active, suspended] (13).
    // Phase 11: SuperB sleep/wake manager (vehicle voltage).
    CMD_GET_VOLTAGE     = 0x2E,  // req: [] -> [status, mV LE16] (vehicle
                                //   supply via V_SENSE divider; INFERRED
                                //   calibration until measured per unit)
    // Phase 12: profiles / auto-detect / logging / harness / DBC.
    CMD_LOG_POLL        = 0x2F,  // req: [] -> [status, nRec, 23B records...]
                                //   (nRec is 0..1: 3+2*23 would exceed the
                                //   48B payload cap; records consumed)
    CMD_LOG_CTL         = 0x30,  // req: [op 0=start 1=stop 2=clear] -> [status]
    CMD_PROFILE_SAVE    = 0x31,  // req: [name 16B] -> [status]
    CMD_PROFILE_LOAD    = 0x32,  // req: [] -> [status]
    CMD_PROFILE_INFO    = 0x33,  // req: [] -> [status, has, ver, writes LE32,
                                //   name 16B]
    CMD_AUTODET_START   = 0x34,  // req: [allowTx 0/1] -> [status]. EXPLICIT
                                //   bus takeover (quiesce -> run -> reacquire);
                                //   allowTx=1 enables K-line/J1850/SWCAN
                                //   transmitting stages (capgate-gated).
    CMD_AUTODET_STATUS  = 0x35,  // req: [] -> [status, done, found, proto,
                                //   baud LE32, confidence]
    CMD_HARNESS_RUN     = 0x36,  // req: [mask LE16] -> [status, nTests, nPass]
                                //   REFUSED unless selfcheck passed.
    CMD_HARNESS_STATUS  = 0x37,  // req: [] -> [status, ran, nTests, nPass]
    CMD_DBC_INFO        = 0x38,  // req: [] -> [status, nDbc, nMsg LE16,
                                //   nSig LE16, nEnum LE16]
    CMD_DBC_DECODE      = 0x39,  // req: [id LE32][ext][len][data..] ->
                                //   [status, nSig] (console does detail)
};

// Reply status codes (payload[1]).
enum : uint8_t {
    ST_OK = 0,
    ST_ERR = 1,
    ST_UNKNOWN_CMD = 2,
    ST_BAD_ARG = 3,
};

// Feature IDs for CMD_SET_FEATURE.
enum : uint8_t {
    FEAT_J1850 = 0x01,  // J1850 rail + protocols (ELM ATSP1/2 need this)
    FEAT_SWCAN = 0x02,  // SWCAN bus (M2RET mode)
};

// Operations the mode manager registers. Plain function pointers avoid an
// include cycle with mode_manager.h.
struct Ops {
    // Safe-switch path ONLY. Returns false if refused/not implemented.
    bool (*requestMode)(uint8_t modeIdx);
    uint8_t (*currentMode)();
    void (*getStatus)(uint8_t& mode, uint32_t& can0Baud, uint32_t& can1Baud,
                      uint32_t& swcanSpeed, uint8_t& features);
    // 0=ok (ST_* codes); *applied=1 if the bus was live-reconfigured,
    // 0 if the value was staged for next mode entry.
    uint8_t (*setCanBaud)(uint8_t bus, uint32_t baud, uint8_t& applied);
    uint8_t (*setSwcanSpeed)(uint32_t speed, uint8_t& applied);
    uint8_t (*setFeature)(uint8_t feat, uint8_t enable);
    // Phase 3: UDS + OBD2 supervision (nullptr-safe: dispatch checks).
    // udsTx: ST_OK + first response in resp/*respLen, count in *nResp;
    //   ST_ERR when UDS mode is not active; ST_BAD_ARG on bad args.
    uint8_t (*udsTx)(const uint8_t* svc, uint8_t svcLen,
                     uint8_t* resp, uint8_t* respLen, uint8_t* nResp);
    uint8_t (*udsCfg)(uint8_t bus, uint32_t baud, uint32_t txId, uint32_t rxId,
                      uint8_t func, uint16_t tpMs);
    void (*udsStatus)(uint8_t& session, uint8_t& bus, uint32_t& baud,
                      uint32_t& txId, uint32_t& rxId, uint8_t& func, uint8_t& tpOn);
    uint8_t (*obd2Cfg)(uint8_t bus, uint32_t baud, uint16_t intervalMs,
                       const uint8_t* pids, uint8_t nPids);
    uint8_t (*obd2Ctl)(uint8_t start, uint8_t& running);
    void (*obd2Status)(uint8_t& running, uint8_t& bus,
                       uint16_t& intervalMs, uint8_t& nPids);
    // Phase 4: MITM supervision (nullptr-safe: dispatch checks).
    // p = 15-byte rule payload (see CMD_MITM_RULE_ADD); idx = table index.
    uint8_t (*mitmRuleAdd)(const uint8_t* p, uint8_t& idx);
    // idx = 0xFF clears all; nRules = remaining count.
    uint8_t (*mitmRuleDel)(uint8_t idx, uint8_t& nRules);
    // start = first index; entries packed 20B each into out; nRet = count.
    uint8_t (*mitmRuleList)(uint8_t start, uint8_t* out,
                            uint8_t& nTotal, uint8_t& nRet);
    // out = 37-byte stats blob (see CMD_MITM_STATS layout).
    void (*mitmStats)(uint8_t* out);
    // p = 14-byte config payload (see CMD_MITM_CFG); applied=1 if live.
    uint8_t (*mitmCfg)(const uint8_t* p, uint8_t& applied);
    // out = 15-byte config blob: [baudA LE32][baudB LE32][dir][stealth]
    // [loopWinMs LE16][defRateLim LE16][active] (CMD_MITM_CFG read path).
    void (*mitmCfgGet)(uint8_t* out);
    // Phase 5: PROG supervision (nullptr-safe: dispatch checks).
    // progCfg: ST_OK; sets the explicit-addressing flag (interlock c).
    uint8_t (*progCfg)(uint8_t bus, uint32_t baud, uint32_t txId, uint32_t rxId);
    // progTx: ST_OK + first response in resp/*respLen, count in *nResp;
    //   ST_ERR when PROG mode is not active; ST_BAD_ARG on bad args.
    //   No M2-side arm (web confirms per step; console enforces arm).
    uint8_t (*progTx)(const uint8_t* svc, uint8_t svcLen,
                      uint8_t* resp, uint8_t* respLen, uint8_t* nResp);
    // progData: streaming chunk -> one 0x36 TransferData. ackSeq = accepted
    //   sequence. CRC/sequence violation aborts the transfer loudly.
    uint8_t (*progData)(uint16_t seq, uint8_t flags, const uint8_t* data,
                        uint8_t len, uint16_t crc, uint16_t& ackSeq);
    // progDone: compare end-to-end CRC32; on match send 0x37.
    //   match=1 on success; txCrc = accumulated CRC32 either way.
    uint8_t (*progDone)(uint32_t fileCrc32, uint32_t& txCrc, uint8_t& match);
    // progAbort: abort sequence (idempotent; never needs arm).
    void (*progAbort)();
    // progArm: arm destructive steps (120 s INFERRED); powerOk records the
    //   stable-12V+ attestation. armed=1 on success.
    uint8_t (*progArm)(uint8_t powerOk, uint8_t& armed);
    // progStatus: fills the 29-byte status blob (see CMD_PROG_STATUS).
    void (*progStatus)(uint8_t* out);
    // Phase 5b: download + verify (own the PROG state machine; the web
    // wizard cannot drive doDownload/doVerify through raw PROG_TX).
    // progDownload: ST_OK on 0x34 positive; needs configured+armed+power.
    uint8_t (*progDownload)(uint32_t addr, uint32_t len, uint8_t alfid);
    // progVerify: ST_OK + match/rxCrc; read-only. INFERRED availability.
    uint8_t (*progVerify)(uint32_t addr, uint32_t len, uint8_t alfid,
                          uint8_t& match, uint32_t& rxCrc);
    // Phase 6: transport supervision (nullptr-safe: dispatch checks).
    // txschedAdd: p = variable-length add payload (see CMD_TXSCHED_ADD);
    //   slot = new slot index. txschedList: 25-byte entries into out.
    uint8_t (*txschedAdd)(const uint8_t* p, uint8_t pLen, uint8_t& slot);
    uint8_t (*txschedDel)(uint8_t slot);
    uint8_t (*txschedList)(uint8_t start, uint8_t* out,
                           uint8_t& nTotal, uint8_t& nRet);
    uint8_t (*txschedCtl)(uint8_t slot, uint8_t start, uint8_t& active);
    // filterAdd: p = 19-byte payload (see CMD_FILTER_ADD); idx = index.
    // filterDel: idx = 0xFF clears all; nLeft = remaining.
    // filterList: 20-byte entries into out (nRet <= 2).
    uint8_t (*filterAdd)(const uint8_t* p, uint8_t& idx);
    uint8_t (*filterDel)(uint8_t idx, uint8_t& nLeft);
    uint8_t (*filterList)(uint8_t start, uint8_t* out,
                          uint8_t& nTotal, uint8_t& nRet);
    // busStats: out = 32-byte stats blob (see CMD_BUS_STATS).
    uint8_t (*busStats)(uint8_t bus, uint8_t* out);
    // isotpCfg: p = 15-byte config payload (see CMD_ISOTP_CFG write).
    uint8_t (*isotpCfg)(const uint8_t* p);
    // isotpCfgGet: out = 15-byte config blob (CMD_ISOTP_CFG read).
    void (*isotpCfgGet)(uint8_t* out);
    // Phase 7: K-line supervision (nullptr-safe: dispatch checks).
    // klineStatus: out = 6-byte status blob (see CMD_KLINE_STATUS).
    void (*klineStatus)(uint8_t* out);
    // klineInit: type 1 = slow 5-baud init, 2 = fast init; kb1/kb2 =
    // keyword bytes. ST_ERR unless ELM/STN mode is active (transmit
    // authority) or the init fails.
    uint8_t (*klineInit)(uint8_t type, uint8_t& kb1, uint8_t& kb2);
    // Phase 9: discovery + capability supervision (nullptr-safe).
    // discoverStart: run a scan on bus 0/1 (mode must be OBD2/UDS).
    uint8_t (*discoverStart)(uint8_t bus);
    // discoverStatus: [done][nEcus][baud LE32] into out (6 bytes).
    void (*discoverStatus)(uint8_t* out);
    // discoverList: page of ECU entries from startIdx; 11B/entry into out;
    // nTotal = total ECUs, nRet = entries returned.
    void (*discoverList)(uint8_t startIdx, uint8_t* out,
                         uint8_t& nTotal, uint8_t& nRet);
    // capGrant: level 0..4 = escalate (upward-only, warned), 0xFF = query.
    // Returns the new (or current) grant level.
    uint8_t (*capGrant)(uint8_t level);
    // Phase 10: gateway supervision (nullptr-safe: dispatch checks).
    // gwMapAdd: p = 21-byte mapping payload (see CMD_GW_MAP_ADD);
    //   idx = table index (0xFE = staged for arm-confirm).
    uint8_t (*gwMapAdd)(const uint8_t* p, uint8_t& idx);
    // gwMapDel: idx = 0xFF clears all; nMaps = remaining count.
    uint8_t (*gwMapDel)(uint8_t idx, uint8_t& nMaps);
    // gwMapList: start = first index; 22-byte entries into out;
    //   nRet <= 1; version = table version.
    uint8_t (*gwMapList)(uint8_t start, uint8_t* out,
                         uint8_t& nTotal, uint8_t& nRet,
                         uint16_t& version);
    // gwStats: out = 29-byte stats blob (see CMD_GW_STATS).
    void (*gwStats)(uint8_t* out);
    // gwCfg: p = 10-byte config payload (see CMD_GW_CFG); applied=1 if live.
    uint8_t (*gwCfg)(const uint8_t* p, uint8_t& applied);
    // gwCfgGet: out = 12-byte config blob (CMD_GW_CFG read path).
    void (*gwCfgGet)(uint8_t* out);
    // Phase 11: SuperB sleep/wake manager (nullptr-safe: dispatch checks).
    // getVoltage: mV = vehicle supply in millivolts (V_SENSE divider).
    void (*getVoltage)(uint16_t& mV);
    // Phase 12: profiles / auto-detect / logging / harness / DBC
    // (nullptr-safe: dispatch checks).
    // logPoll: out = up to PER_POLL*23B records; nRec = records copied.
    uint8_t (*logPoll)(uint8_t* out, uint8_t& nRec);
    // logCtl: op 0=start 1=stop 2=clear.
    uint8_t (*logCtl)(uint8_t op);
    // profileSave: name = up to 16 chars (NUL-padded).
    uint8_t (*profileSave)(const char* name);
    // profileLoad: apply the stored profile.
    uint8_t (*profileLoad)();
    // profileInfo: out = [has][ver][writes LE32][name 16B] (22 bytes).
    void (*profileInfo)(uint8_t* out);
    // autodetStart: explicit bus takeover (quiesce -> run -> reacquire);
    // allowTx enables the transmitting stages (capgate-gated inside).
    uint8_t (*autodetStart)(uint8_t allowTx);
    // autodetStatus: out = [done][found][proto][baud LE32][confidence].
    void (*autodetStatus)(uint8_t* out);
    // harnessRun: mask = test bitmask; nTests/nPass = outcome. REFUSED
    // unless the loopback selfcheck passed (returns ST_ERR then).
    uint8_t (*harnessRun)(uint16_t mask, uint8_t& nTests, uint8_t& nPass);
    // harnessStatus: out = [ran][nTests][nPass].
    void (*harnessStatus)(uint8_t* out);
    // dbcInfo: out = [nDbc][nMsg LE16][nSig LE16][nEnum LE16].
    void (*dbcInfo)(uint8_t* out);
    // dbcDecode: decode a raw frame; returns the signal count.
    uint8_t (*dbcDecode)(uint32_t id, uint8_t ext, const uint8_t* data,
                         uint8_t len);
};

void setOps(const Ops* ops);

// Divert helper for modes whose data plane is NOT line-oriented (MITM):
// feed every Serial byte through it; returns true when the byte was
// consumed by the control channel (or is part of an F1 7E prefix being
// held). Mirrors the divert logic in the ELM byte pump.
struct Divert {
    bool sawF1 = false;
    bool binActive = false;
};
bool divertByte(Divert& d, uint8_t b);

// Receiver: call reset() when the F1 7E prefix is seen, then feedByte() for
// every following byte. Returns true when a complete frame was received and
// dispatched (caller should then resume normal parsing).
void reset();
bool feedByte(uint8_t b);

// Encoder for the SuperB side / tests: builds a frame into out[].
// Returns total frame length (0 if it would exceed outCap).
uint8_t encode(uint8_t cmd, const uint8_t* payload, uint8_t payloadLen,
               uint8_t* out, uint8_t outCap);

} // namespace mm
