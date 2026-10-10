// M2 multi-mode firmware — control channel implementation (Phase 2).
// See mm_channel.h for the framing contract and safety invariant.

#include "mm_channel.h"
#include <string.h>  // Phase 12: memcpy for CMD_PROFILE_SAVE

namespace mm {

namespace {
const Ops* ops = nullptr;

enum class RxState : uint8_t { LEN, PAYLOAD, XOR };
RxState rxState = RxState::LEN;
uint8_t rxLen = 0;
uint8_t rxPos = 0;
uint8_t rxXor = 0;
uint8_t rxBuf[MAX_PAYLOAD];

uint8_t xorSum(const uint8_t* p, uint8_t n)
{
    uint8_t x = 0;
    for (uint8_t i = 0; i < n; i++) x ^= p[i];
    return x;
}

void putU32(uint8_t* p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF); p[3] = (uint8_t)((v >> 24) & 0xFF);
}

uint32_t getU32(const uint8_t* p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

void putU16(uint8_t* p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)((v >> 8) & 0xFF);
}

uint16_t getU16(const uint8_t* p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// NOTE: replies always go to Serial (the Bee link). The SuperB's RX diverter
// extracts F1 7E frames before app broadcast, so apps never see them.
void sendReply(const uint8_t* payload, uint8_t len)
{
    // F1 (2026-10-08): was [3 + MAX_PAYLOAD] (51B) — a 48B payload needs
    // SYNC+CMD+LEN+48+XOR = 52B. One-byte stack overflow at max length.
    uint8_t frame[4 + MAX_PAYLOAD];
    frame[0] = SYNC; frame[1] = CMD_MULTIMODE; frame[2] = len;
    for (uint8_t i = 0; i < len; i++) frame[3 + i] = payload[i];
    frame[3 + len] = xorSum(payload, len);
    Serial.write(frame, (size_t)(4 + len));
}

void dispatch()
{
    // Minimal two-byte error reply helper.
    uint8_t rep[MAX_PAYLOAD];
    if (rxLen < 1 || ops == nullptr) {
        // No ops registered (shouldn't happen after begin()) or empty frame.
        return;
    }
    uint8_t cmd = rxBuf[0];
    rep[0] = cmd | 0x80;

    switch (cmd) {
        case CMD_SET_MODE: {
            if (rxLen != 2) { rep[1] = ST_BAD_ARG; sendReply(rep, 2); return; }
            // THE safety invariant: only via the manager's safe-switch path.
            bool ok = ops->requestMode(rxBuf[1]);
            rep[1] = ok ? ST_OK : ST_ERR;
            sendReply(rep, 2);
            break;
        }
        case CMD_GET_STATUS: {
            uint8_t mode; uint32_t c0, c1, sw; uint8_t feat;
            ops->getStatus(mode, c0, c1, sw, feat);
            rep[1] = ST_OK; rep[2] = mode;
            putU32(rep + 3, c0); putU32(rep + 7, c1); putU32(rep + 11, sw);
            rep[15] = feat;
            sendReply(rep, 16);
            break;
        }
        case CMD_SET_CAN_BAUD: {
            if (rxLen != 6) { rep[1] = ST_BAD_ARG; sendReply(rep, 2); return; }
            uint8_t applied = 0;
            rep[1] = ops->setCanBaud(rxBuf[1], getU32(rxBuf + 2), applied);
            rep[2] = applied;
            sendReply(rep, 3);
            break;
        }
        case CMD_SET_SWCAN_SPEED: {
            if (rxLen != 5) { rep[1] = ST_BAD_ARG; sendReply(rep, 2); return; }
            uint8_t applied = 0;
            rep[1] = ops->setSwcanSpeed(getU32(rxBuf + 1), applied);
            rep[2] = applied;
            sendReply(rep, 3);
            break;
        }
        case CMD_SET_FEATURE: {
            if (rxLen != 3) { rep[1] = ST_BAD_ARG; sendReply(rep, 2); return; }
            rep[1] = ops->setFeature(rxBuf[1], rxBuf[2]);
            sendReply(rep, 2);
            break;
        }
        case CMD_UDS_TX: {
            // req: [svcLen][svc...]. Reply: [cmd|80][status][nResp][rLen][rData...]
            if (rxLen < 3 || rxBuf[1] == 0 || rxBuf[1] > (uint8_t)(rxLen - 2) ||
                ops->udsTx == nullptr) {
                rep[1] = (ops->udsTx == nullptr) ? ST_ERR : ST_BAD_ARG;
                sendReply(rep, 2);
                return;
            }
            uint8_t rdata[64]; uint8_t rlen = 0; uint8_t nResp = 0;
            // F-03-03: carry the destination capacity (mmcTransact outCap
            // pattern) — uds::mmTx clamps the ISO-TP response to it.
            rep[1] = ops->udsTx(rxBuf + 2, rxBuf[1], rdata, sizeof(rdata), &rlen, &nResp);
            rep[2] = nResp;
            // Truncate the first response to fit MAX_PAYLOAD (documented).
            uint8_t room = (uint8_t)(MAX_PAYLOAD - 4);
            uint8_t cn = (rlen <= room) ? rlen : room;
            rep[3] = cn;
            for (uint8_t i = 0; i < cn; i++) rep[4 + i] = rdata[i];
            sendReply(rep, (uint8_t)(4 + cn));
            break;
        }
        case CMD_UDS_CFG: {
            // req: [bus][baud LE32][txId LE32][rxId LE32][func][tpMs LE16] = 16
            if (rxLen != 17 || ops->udsCfg == nullptr) {
                rep[1] = ST_BAD_ARG; sendReply(rep, 2); return;
            }
            rep[1] = ops->udsCfg(rxBuf[1], getU32(rxBuf + 2), getU32(rxBuf + 6),
                                 getU32(rxBuf + 10), rxBuf[14], getU16(rxBuf + 15));
            sendReply(rep, 2);
            break;
        }
        case CMD_UDS_STATUS: {
            if (ops->udsStatus == nullptr) { rep[1] = ST_ERR; sendReply(rep, 2); return; }
            uint8_t session, bus, func, tpOn; uint32_t baud, txId, rxId;
            ops->udsStatus(session, bus, baud, txId, rxId, func, tpOn);
            rep[1] = ST_OK; rep[2] = session; rep[3] = bus;
            putU32(rep + 4, baud); putU32(rep + 8, txId); putU32(rep + 12, rxId);
            rep[16] = func; rep[17] = tpOn;
            sendReply(rep, 18);
            break;
        }
        case CMD_OBD2_CFG: {
            // req: [bus][baud LE32][interval LE16][nPids][pids...]
            if (rxLen < 9 || ops->obd2Cfg == nullptr) {
                rep[1] = ST_BAD_ARG; sendReply(rep, 2); return;
            }
            uint8_t nPids = rxBuf[8];
            if (nPids == 0 || nPids > 16 || (uint8_t)(9 + nPids) != rxLen) {
                rep[1] = ST_BAD_ARG; sendReply(rep, 2); return;
            }
            rep[1] = ops->obd2Cfg(rxBuf[1], getU32(rxBuf + 2), getU16(rxBuf + 6),
                                  rxBuf + 9, nPids);
            sendReply(rep, 2);
            break;
        }
        case CMD_OBD2_CTL: {
            if (rxLen != 2 || ops->obd2Ctl == nullptr) {
                rep[1] = ST_BAD_ARG; sendReply(rep, 2); return;
            }
            uint8_t running = 0;
            rep[1] = ops->obd2Ctl(rxBuf[1], running);
            rep[2] = running;
            sendReply(rep, 3);
            break;
        }
        case CMD_OBD2_STATUS: {
            if (ops->obd2Status == nullptr) { rep[1] = ST_ERR; sendReply(rep, 2); return; }
            uint8_t running, bus, nPids; uint16_t interval;
            ops->obd2Status(running, bus, interval, nPids);
            rep[1] = ST_OK; rep[2] = running; rep[3] = bus;
            putU16(rep + 4, interval); rep[6] = nPids;
            sendReply(rep, 7);
            break;
        }
        case CMD_MITM_RULE_ADD: {
            // req: 15-byte rule payload (see mm_channel.h).
            if (rxLen != 16 || ops->mitmRuleAdd == nullptr) {
                rep[1] = (ops->mitmRuleAdd == nullptr) ? ST_ERR : ST_BAD_ARG;
                sendReply(rep, 2);
                return;
            }
            uint8_t idx = 0;
            rep[1] = ops->mitmRuleAdd(rxBuf + 1, idx);
            rep[2] = idx;
            sendReply(rep, 3);
            break;
        }
        case CMD_MITM_RULE_DEL: {
            if (rxLen != 2 || ops->mitmRuleDel == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            uint8_t nRules = 0;
            rep[1] = ops->mitmRuleDel(rxBuf[1], nRules);
            rep[2] = nRules;
            sendReply(rep, 3);
            break;
        }
        case CMD_MITM_RULE_LIST: {
            // req: [startIdx]. Reply: [status][nTotal][nRet][entries...],
            // 20 bytes per entry, max 2 per reply (48-byte frame cap).
            if (rxLen != 2 || ops->mitmRuleList == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            uint8_t nTotal = 0, nRet = 0;
            rep[1] = ops->mitmRuleList(rxBuf[1], rep + 4, nTotal, nRet);
            if (nRet > 2) nRet = 2;  // defensive: ops contract is <=2
            rep[2] = nTotal; rep[3] = nRet;
            sendReply(rep, (uint8_t)(4 + nRet * 20));
            break;
        }
        case CMD_MITM_STATS: {
            if (ops->mitmStats == nullptr) { rep[1] = ST_ERR; sendReply(rep, 2); return; }
            rep[1] = ST_OK;
            ops->mitmStats(rep + 2);  // 37-byte blob
            sendReply(rep, 39);
            break;
        }
        case CMD_MITM_CFG: {
            // WRITE: 14-byte config payload (see mm_channel.h).
            // READ: empty payload -> current config blob (17-byte reply).
            if (rxLen == 1) {
                if (ops->mitmCfgGet == nullptr) {
                    rep[1] = ST_ERR; sendReply(rep, 2); return;
                }
                rep[1] = ST_OK;
                ops->mitmCfgGet(rep + 2);  // 15-byte blob
                sendReply(rep, 17);
                break;
            }
            if (rxLen != 15 || ops->mitmCfg == nullptr) {
                rep[1] = (ops->mitmCfg == nullptr) ? ST_ERR : ST_BAD_ARG;
                sendReply(rep, 2);
                return;
            }
            uint8_t applied = 0;
            rep[1] = ops->mitmCfg(rxBuf + 1, applied);
            rep[2] = applied;
            sendReply(rep, 3);
            break;
        }
        case CMD_PROG_CFG: {
            // req: [bus][baud LE32][txId LE32][rxId LE32] = 14 bytes.
            if (rxLen != 15 || ops->progCfg == nullptr) {
                rep[1] = (ops->progCfg == nullptr) ? ST_ERR : ST_BAD_ARG;
                sendReply(rep, 2);
                return;
            }
            rep[1] = ops->progCfg(rxBuf[1], getU32(rxBuf + 2),
                                  getU32(rxBuf + 6), getU32(rxBuf + 10));
            sendReply(rep, 2);
            break;
        }
        case CMD_PROG_TX: {
            // req: [svcLen][svc...]. Reply: [cmd|80][status][nResp][rLen][rData...]
            if (rxLen < 3 || rxBuf[1] == 0 || rxBuf[1] > (uint8_t)(rxLen - 2) ||
                ops->progTx == nullptr) {
                rep[1] = (ops->progTx == nullptr) ? ST_ERR : ST_BAD_ARG;
                sendReply(rep, 2);
                return;
            }
            uint8_t rdata[64]; uint8_t rlen = 0; uint8_t nResp = 0;
            rep[1] = ops->progTx(rxBuf + 2, rxBuf[1], rdata, &rlen, &nResp);
            rep[2] = nResp;
            uint8_t room = (uint8_t)(MAX_PAYLOAD - 4);
            uint8_t cn = (rlen <= room) ? rlen : room;
            rep[3] = cn;
            for (uint8_t i = 0; i < cn; i++) rep[4 + i] = rdata[i];
            sendReply(rep, (uint8_t)(4 + cn));
            break;
        }
        case CMD_PROG_DATA: {
            // req: [seq LE16][flags][len][data...][crc16 LE16].
            if (rxLen < 8 || ops->progData == nullptr) {
                rep[1] = (ops->progData == nullptr) ? ST_ERR : ST_BAD_ARG;
                sendReply(rep, 2);
                return;
            }
            uint16_t seq = getU16(rxBuf + 1);
            uint8_t flags = rxBuf[3];
            uint8_t dlen = rxBuf[4];
            // Total: 1(cmd)+2+1+1+dlen+2 = 7+dlen must equal rxLen.
            if (dlen > 41 || (uint8_t)(7 + dlen) != rxLen) {
                rep[1] = ST_BAD_ARG; sendReply(rep, 2); return;
            }
            uint16_t crc = getU16(rxBuf + 5 + dlen);
            uint16_t ackSeq = 0;
            rep[1] = ops->progData(seq, flags, rxBuf + 5, dlen, crc, ackSeq);
            rep[2] = (uint8_t)(ackSeq & 0xFF);
            rep[3] = (uint8_t)((ackSeq >> 8) & 0xFF);
            sendReply(rep, 4);
            break;
        }
        case CMD_PROG_DONE: {
            // req: [fileCrc32 LE32].
            if (rxLen != 5 || ops->progDone == nullptr) {
                rep[1] = (ops->progDone == nullptr) ? ST_ERR : ST_BAD_ARG;
                sendReply(rep, 2);
                return;
            }
            uint32_t txCrc = 0; uint8_t match = 0;
            rep[1] = ops->progDone(getU32(rxBuf + 1), txCrc, match);
            rep[2] = match;
            putU32(rep + 3, txCrc);
            sendReply(rep, 7);
            break;
        }
        case CMD_PROG_STATUS: {
            if (ops->progStatus == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            rep[1] = ST_OK;
            ops->progStatus(rep + 2);  // 29-byte blob
            sendReply(rep, 31);
            break;
        }
        case CMD_PROG_ABORT: {
            if (ops->progAbort == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            ops->progAbort();
            rep[1] = ST_OK;
            sendReply(rep, 2);
            break;
        }
        case CMD_PROG_ARM: {
            // req: [powerOk 0/1].
            if (rxLen != 2 || ops->progArm == nullptr) {
                rep[1] = (ops->progArm == nullptr) ? ST_ERR : ST_BAD_ARG;
                sendReply(rep, 2);
                return;
            }
            uint8_t armed = 0;
            rep[1] = ops->progArm(rxBuf[1], armed);
            rep[2] = armed;
            sendReply(rep, 3);
            break;
        }
        case CMD_PROG_DL: {
            // req: [addr LE32][len LE32][alfid].
            if (rxLen != 10 || ops->progDownload == nullptr) {
                rep[1] = (ops->progDownload == nullptr) ? ST_ERR : ST_BAD_ARG;
                sendReply(rep, 2);
                return;
            }
            rep[1] = ops->progDownload(getU32(rxBuf + 1), getU32(rxBuf + 5),
                                       rxBuf[9]);
            sendReply(rep, 2);
            break;
        }
        case CMD_PROG_VERIFY: {
            // req: [addr LE32][len LE32][alfid].
            if (rxLen != 10 || ops->progVerify == nullptr) {
                rep[1] = (ops->progVerify == nullptr) ? ST_ERR : ST_BAD_ARG;
                sendReply(rep, 2);
                return;
            }
            uint8_t match = 0; uint32_t rxCrc = 0;
            rep[1] = ops->progVerify(getU32(rxBuf + 1), getU32(rxBuf + 5),
                                     rxBuf[9], match, rxCrc);
            rep[2] = match;
            putU32(rep + 3, rxCrc);
            sendReply(rep, 7);
            break;
        }
        case CMD_TXSCHED_ADD: {
            // req: [bus][id LE32][ext][len][data..][periodMs LE32]
            //      [offsetMs LE32][count LE32]  (20+len bytes total)
            if (rxLen < 21 || ops->txschedAdd == nullptr) {
                rep[1] = (ops->txschedAdd == nullptr) ? ST_ERR : ST_BAD_ARG;
                sendReply(rep, 2);
                return;
            }
            uint8_t dlen = rxBuf[7];
            if (dlen > 8 || (uint8_t)(20 + dlen) != rxLen) {
                rep[1] = ST_BAD_ARG; sendReply(rep, 2); return;
            }
            uint8_t slot = 0xFF;
            rep[1] = ops->txschedAdd(rxBuf + 1, (uint8_t)(rxLen - 1), slot);
            rep[2] = slot;
            sendReply(rep, 3);
            break;
        }
        case CMD_TXSCHED_DEL: {
            if (rxLen != 2 || ops->txschedDel == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            rep[1] = ops->txschedDel(rxBuf[1]);
            sendReply(rep, 2);
            break;
        }
        case CMD_TXSCHED_LIST: {
            // 25-byte entries, 1 per reply (48-byte frame cap).
            if (rxLen != 2 || ops->txschedList == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            uint8_t nTotal = 0, nRet = 0;
            rep[1] = ops->txschedList(rxBuf[1], rep + 4, nTotal, nRet);
            if (nRet > 1) nRet = 1;  // defensive: ops contract is <=1
            rep[2] = nTotal; rep[3] = nRet;
            sendReply(rep, (uint8_t)(4 + nRet * 25));
            break;
        }
        case CMD_TXSCHED_CTL: {
            if (rxLen != 3 || ops->txschedCtl == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            uint8_t active = 0;
            rep[1] = ops->txschedCtl(rxBuf[1], rxBuf[2], active);
            rep[2] = active;
            sendReply(rep, 3);
            break;
        }
        case CMD_FILTER_ADD: {
            // req: 20-byte filter payload (see mm_channel.h).
            if (rxLen != 21 || ops->filterAdd == nullptr) {
                rep[1] = (ops->filterAdd == nullptr) ? ST_ERR : ST_BAD_ARG;
                sendReply(rep, 2);
                return;
            }
            uint8_t idx = 0xFF;
            rep[1] = ops->filterAdd(rxBuf + 1, idx);
            rep[2] = idx;
            sendReply(rep, 3);
            break;
        }
        case CMD_FILTER_DEL: {
            if (rxLen != 2 || ops->filterDel == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            uint8_t nLeft = 0;
            rep[1] = ops->filterDel(rxBuf[1], nLeft);
            rep[2] = nLeft;
            sendReply(rep, 3);
            break;
        }
        case CMD_FILTER_LIST: {
            // 20-byte entries, 2 per reply (48-byte frame cap).
            if (rxLen != 2 || ops->filterList == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            uint8_t nTotal = 0, nRet = 0;
            rep[1] = ops->filterList(rxBuf[1], rep + 4, nTotal, nRet);
            if (nRet > 2) nRet = 2;  // defensive: ops contract is <=2
            rep[2] = nTotal; rep[3] = nRet;
            sendReply(rep, (uint8_t)(4 + nRet * 20));
            break;
        }
        case CMD_BUS_STATS: {
            if (rxLen != 2 || ops->busStats == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            rep[1] = ST_OK;
            if (ops->busStats(rxBuf[1], rep + 2) != ST_OK) {
                rep[1] = ST_BAD_ARG; sendReply(rep, 2); return;
            }
            sendReply(rep, 34);
            break;
        }
        case CMD_ISOTP_CFG: {
            // WRITE: 15-byte config payload. READ: empty payload.
            if (rxLen == 1) {
                if (ops->isotpCfgGet == nullptr) {
                    rep[1] = ST_ERR; sendReply(rep, 2); return;
                }
                rep[1] = ST_OK;
                ops->isotpCfgGet(rep + 2);  // 15-byte blob
                sendReply(rep, 17);
                break;
            }
            if (rxLen != 16 || ops->isotpCfg == nullptr) {
                rep[1] = (ops->isotpCfg == nullptr) ? ST_ERR : ST_BAD_ARG;
                sendReply(rep, 2);
                return;
            }
            rep[1] = ops->isotpCfg(rxBuf + 1);
            sendReply(rep, 2);
            break;
        }
        case CMD_KLINE_STATUS: {  // Phase 7
            if (rxLen != 1 || ops->klineStatus == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            rep[1] = ST_OK;
            ops->klineStatus(rep + 2);  // 6-byte blob
            sendReply(rep, 8);
            break;
        }
        case CMD_KLINE_INIT: {  // Phase 7
            if (rxLen != 2 || ops->klineInit == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            uint8_t kb1 = 0, kb2 = 0;
            uint8_t st = ops->klineInit(rxBuf[1], kb1, kb2);
            rep[1] = st;
            rep[2] = kb1; rep[3] = kb2;
            sendReply(rep, (st == ST_OK) ? 4 : 2);
            break;
        }
        case CMD_DISCOVER_START: {  // Phase 9
            if (rxLen != 2 || ops->discoverStart == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            rep[1] = ops->discoverStart(rxBuf[1]);
            sendReply(rep, 2);
            break;
        }
        case CMD_DISCOVER_STATUS: {  // Phase 9
            if (rxLen != 1 || ops->discoverStatus == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            rep[1] = ST_OK;
            ops->discoverStatus(rep + 2);  // 6-byte blob
            sendReply(rep, 8);
            break;
        }
        case CMD_DISCOVER_LIST: {  // Phase 9 (0x27 in the mm namespace only)
            if (rxLen != 2 || ops->discoverList == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            uint8_t nTotal = 0, nRet = 0;
            rep[1] = ST_OK;
            ops->discoverList(rxBuf[1], rep + 4, nTotal, nRet);
            rep[2] = nTotal; rep[3] = nRet;
            sendReply(rep, 4 + (uint8_t)(nRet * 11));
            break;
        }
        case CMD_CAP_GRANT: {  // Phase 9
            if (rxLen != 2 || ops->capGrant == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            uint8_t g = ops->capGrant(rxBuf[1]);
            rep[1] = ST_OK;
            rep[2] = g;
            sendReply(rep, 3);
            break;
        }
        case CMD_GW_MAP_ADD: {  // Phase 10
            if (rxLen != 22 || ops->gwMapAdd == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            uint8_t idx = 0xFF;
            rep[1] = ops->gwMapAdd(rxBuf + 1, idx);
            rep[2] = idx;
            sendReply(rep, (rep[1] == ST_OK) ? 3 : 2);
            break;
        }
        case CMD_GW_MAP_DEL: {  // Phase 10
            if (rxLen != 2 || ops->gwMapDel == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            uint8_t nMaps = 0;
            rep[1] = ops->gwMapDel(rxBuf[1], nMaps);
            rep[2] = nMaps;
            sendReply(rep, (rep[1] == ST_OK) ? 3 : 2);
            break;
        }
        case CMD_GW_MAP_LIST: {  // Phase 10
            if (rxLen != 2 || ops->gwMapList == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            uint8_t nTotal = 0, nRet = 0;
            uint16_t version = 0;
            rep[1] = ops->gwMapList(rxBuf[1], rep + 6, nTotal, nRet, version);
            rep[2] = nTotal; rep[3] = nRet;
            rep[4] = (uint8_t)(version & 0xFF);
            rep[5] = (uint8_t)((version >> 8) & 0xFF);
            sendReply(rep, 6 + (uint8_t)(nRet * 22));
            break;
        }
        case CMD_GW_STATS: {  // Phase 10
            if (rxLen != 1 || ops->gwStats == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            rep[1] = ST_OK;
            ops->gwStats(rep + 2);  // 29-byte blob
            sendReply(rep, 31);
            break;
        }
        case CMD_GW_CFG: {  // Phase 10
            if (ops->gwCfg == nullptr || ops->gwCfgGet == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            if (rxLen == 1) {  // READ
                rep[1] = ST_OK;
                ops->gwCfgGet(rep + 2);  // 12-byte blob
                sendReply(rep, 14);
            } else if (rxLen == 11) {  // WRITE: 10-byte payload
                uint8_t applied = 0;
                rep[1] = ops->gwCfg(rxBuf + 1, applied);
                rep[2] = applied;
                sendReply(rep, (rep[1] == ST_OK) ? 3 : 2);
            } else {
                rep[1] = ST_BAD_ARG; sendReply(rep, 2);
            }
            break;
        }
        case CMD_GET_VOLTAGE: {  // Phase 11
            if (rxLen != 1 || ops->getVoltage == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            uint16_t mV = 0;
            ops->getVoltage(mV);
            rep[1] = ST_OK;
            rep[2] = (uint8_t)(mV & 0xFF);
            rep[3] = (uint8_t)((mV >> 8) & 0xFF);
            sendReply(rep, 4);
            break;
        }
        // ---- Phase 12: profiles / auto-detect / logging / harness / DBC ----
        case CMD_LOG_POLL: {
            if (rxLen != 1 || ops->logPoll == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            uint8_t nRec = 0;
            uint8_t n = ops->logPoll(rep + 3, nRec);
            (void)n;
            if (nRec > 1) nRec = 1;  // defensive: ops contract is <=1
            rep[1] = ST_OK;
            rep[2] = nRec;
            sendReply(rep, 3 + nRec * 23);
            break;
        }
        case CMD_LOG_CTL: {
            if (rxLen != 2 || ops->logCtl == nullptr) {
                rep[1] = ST_BAD_ARG; sendReply(rep, 2); return;
            }
            rep[1] = ops->logCtl(rxBuf[1]);
            sendReply(rep, 2);
            break;
        }
        case CMD_PROFILE_SAVE: {
            if (rxLen != 17 || ops->profileSave == nullptr) {
                rep[1] = ST_BAD_ARG; sendReply(rep, 2); return;
            }
            char name[17];
            memcpy(name, rxBuf + 1, 16);
            name[16] = '\0';
            rep[1] = ops->profileSave(name);
            sendReply(rep, 2);
            break;
        }
        case CMD_PROFILE_LOAD: {
            if (rxLen != 1 || ops->profileLoad == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            rep[1] = ops->profileLoad();
            sendReply(rep, 2);
            break;
        }
        case CMD_PROFILE_INFO: {
            if (rxLen != 1 || ops->profileInfo == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            rep[1] = ST_OK;
            ops->profileInfo(rep + 2);  // 22 bytes
            sendReply(rep, 24);
            break;
        }
        case CMD_AUTODET_START: {
            if (rxLen != 2 || ops->autodetStart == nullptr) {
                rep[1] = ST_BAD_ARG; sendReply(rep, 2); return;
            }
            rep[1] = ops->autodetStart(rxBuf[1]);
            sendReply(rep, 2);
            break;
        }
        case CMD_AUTODET_STATUS: {
            if (rxLen != 1 || ops->autodetStatus == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            rep[1] = ST_OK;
            ops->autodetStatus(rep + 2);  // 8 bytes
            sendReply(rep, 10);
            break;
        }
        case CMD_HARNESS_RUN: {
            if (rxLen != 3 || ops->harnessRun == nullptr) {
                rep[1] = ST_BAD_ARG; sendReply(rep, 2); return;
            }
            uint16_t mask = (uint16_t)rxBuf[1] | ((uint16_t)rxBuf[2] << 8);
            uint8_t nTests = 0, nPass = 0;
            rep[1] = ops->harnessRun(mask, nTests, nPass);
            rep[2] = nTests;
            rep[3] = nPass;
            sendReply(rep, 4);
            break;
        }
        case CMD_HARNESS_STATUS: {
            if (rxLen != 1 || ops->harnessStatus == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            rep[1] = ST_OK;
            ops->harnessStatus(rep + 2);  // 3 bytes
            sendReply(rep, 5);
            break;
        }
        case CMD_DBC_INFO: {
            if (rxLen != 1 || ops->dbcInfo == nullptr) {
                rep[1] = ST_ERR; sendReply(rep, 2); return;
            }
            rep[1] = ST_OK;
            ops->dbcInfo(rep + 2);  // 7 bytes
            sendReply(rep, 9);
            break;
        }
        case CMD_DBC_DECODE: {
            if (rxLen < 8 || ops->dbcDecode == nullptr) {
                rep[1] = ST_BAD_ARG; sendReply(rep, 2); return;
            }
            uint32_t id = (uint32_t)rxBuf[1] | ((uint32_t)rxBuf[2] << 8) |
                          ((uint32_t)rxBuf[3] << 16) | ((uint32_t)rxBuf[4] << 24);
            uint8_t ext = rxBuf[5];
            uint8_t len = rxBuf[6];
            if (len > 8 || rxLen != (uint8_t)(7 + len)) {
                rep[1] = ST_BAD_ARG; sendReply(rep, 2); return;
            }
            rep[1] = ST_OK;
            rep[2] = ops->dbcDecode(id, ext, rxBuf + 7, len);
            sendReply(rep, 3);
            break;
        }
        default: {
            rep[1] = ST_UNKNOWN_CMD;
            sendReply(rep, 2);
            break;
        }
    }
}

} // namespace

void setOps(const Ops* o) { ops = o; }

bool divertByte(Divert& d, uint8_t b)
{
    if (d.binActive) {
        if (feedByte(b)) d.binActive = false;
        return true;
    }
    if (d.sawF1) {
        d.sawF1 = false;
        if (b == CMD_MULTIMODE) { reset(); d.binActive = true; return true; }
        if (b == SYNC) { d.sawF1 = true; return true; }  // F1 F1 7E edge
        return false;  // stray F1: dropped, b delivered to caller
    }
    if (b == SYNC) { d.sawF1 = true; return true; }
    return false;
}

void reset()
{
    rxState = RxState::LEN;
    rxLen = 0; rxPos = 0; rxXor = 0;
}

bool feedByte(uint8_t b)
{
    switch (rxState) {
        case RxState::LEN:
            rxLen = b;
            if (rxLen == 0 || rxLen > MAX_PAYLOAD) { reset(); return true; }
            rxPos = 0; rxXor = 0;
            rxState = RxState::PAYLOAD;
            return false;
        case RxState::PAYLOAD:
            rxBuf[rxPos++] = b;
            rxXor ^= b;
            if (rxPos >= rxLen) rxState = RxState::XOR;
            return false;
        case RxState::XOR: {
            bool ok = (b == rxXor);
            if (ok) dispatch();
            // Bad checksum: drop silently (GVRET convention: bytes between
            // checksum and 0xF1 are thrown away; M2RET.ino:691).
            reset();
            return true;
        }
    }
    return true;  // unreachable
}

uint8_t encode(uint8_t cmd, const uint8_t* payload, uint8_t payloadLen,
               uint8_t* out, uint8_t outCap)
{
    // S4: the frame is F1 7E LEN CMD PAYLOAD XOR = payloadLen + 5 bytes.
    // The old (4 + payloadLen) undercount let the XOR write one byte
    // past a buffer sized exactly to the reported total (ASan-confirmed).
    uint8_t total = (uint8_t)(5 + payloadLen);  // F1 7E LEN CMD PAYLOAD XOR
    if (total > outCap || payloadLen > MAX_PAYLOAD - 1) return 0;
    out[0] = SYNC; out[1] = CMD_MULTIMODE; out[2] = (uint8_t)(1 + payloadLen);
    out[3] = cmd;
    uint8_t x = cmd;
    for (uint8_t i = 0; i < payloadLen; i++) { out[4 + i] = payload[i]; x ^= payload[i]; }
    out[4 + payloadLen] = x;
    return total;
}

} // namespace mm
