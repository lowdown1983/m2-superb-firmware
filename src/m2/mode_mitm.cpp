// M2 multi-mode firmware — MITM mode implementation (Phase 4).
// See mode_mitm.h for the contract and exact rule semantics.

#include "mode_mitm.h"
#include "mode_manager.h"  // modes::canBegin0/1, canEnd0/1
#include "mm_channel.h"
#include "busfilter.h"  // Phase 6: shared idMatch() for the rule engine
#include "busstat.h"    // Phase 6: per-bus stats feed
#include "capgate.h"    // Phase 9: MODIFY rules need ALLOW_TRANSMIT
#include <due_can.h>

namespace mitm {

namespace {

// ---- directions / actions (match mm_channel.h payload encoding) ----
enum : uint8_t { D_A2B = 0, D_B2A = 1, D_BOTH = 2 };
enum : uint8_t { A_PASS = 0, A_DROP = 1, A_MODIFY = 2, A_COUNT = 3, A_LOG = 4 };

constexpr uint8_t MAX_RULES = 8;

struct Rule {
    uint32_t id = 0, mask = 0;
    uint8_t dir = D_BOTH, action = A_PASS;
    uint8_t modOff = 0, modMask = 0, modVal = 0;
    uint16_t rateLim = 0;       // frames/sec, 0 = unlimited
    // runtime:
    uint32_t hits = 0;
    uint32_t winStart = 0;      // rate-limit window
    uint16_t winCount = 0;
};

struct Stats {
    uint32_t fwdAB = 0, fwdBA = 0;
    uint32_t dropAB = 0, dropBA = 0;
    uint32_t modAB = 0, modBA = 0;
    uint32_t loopSupp = 0, rateDrop = 0;
};

// Loop-suppression ring: recently forwarded frames.
struct FwdRec {
    uint32_t id = 0;
    uint8_t len = 0;
    uint8_t data[8] = {0};
    uint8_t dir = 0;            // direction it was forwarded (0=A2B,1=B2A)
    uint32_t t = 0;
};
constexpr uint8_t FWD_RING = 16;

Rule rules[MAX_RULES];
uint8_t nRules = 0;
Stats stats;
FwdRec fwdRing[FWD_RING];
uint8_t fwdHead = 0;

bool active = false;            // between begin() and exit()
uint32_t baudA = 500000, baudB = 500000;
uint8_t dirMode = D_BOTH;       // global direction gate
bool stealth = true;
uint16_t loopWinMs = 5;         // INFERRED tuning
uint16_t defRateLim = 0;        // default-rule rate cap, 0 = unlimited

// Bus-load estimation windows (INFERRED: ~128 bits nominal per frame).
uint32_t loadWinA = 0, loadWinB = 0;
uint16_t loadFramesA = 0, loadFramesB = 0;
uint8_t loadPctA = 0, loadPctB = 0;

mm::Divert divert;

bool validBaud(uint32_t b)
{
    return b == 100000 || b == 125000 || b == 250000 || b == 500000 ||
           b == 800000 || b == 1000000;
}

void promiscuous(CANRaw* bus)
{
    // Open all RX mailboxes (same pattern as ELM/M2RET).
    for (int m = 0; m < 7; m++) bus->setRXFilter(m, 0, 0, (m < 3));
}

uint8_t calcLoad(uint16_t frames, uint32_t baud)
{
    uint32_t pct = (uint32_t)frames * 128UL * 100UL / baud;
    return (pct > 100) ? 100 : (uint8_t)pct;
}

void printHex2(uint8_t b)
{
    const char* H = "0123456789ABCDEF";
    Serial.write((uint8_t)H[(b >> 4) & 0xF]);
    Serial.write((uint8_t)H[b & 0xF]);
}

// LOG action: emit a copy on the Bee link for the SuperB web UI.
// Format: MITMLOG,<A2B|B2A>,<id hex>,<ext 0/1>,<len>,<data hex>
// The SuperB sniffs MITMLOG lines (mode-gated) into its web ring.
void emitLog(uint8_t dirAB, const CAN_FRAME& f)
{
    Serial.print(F("MITMLOG,"));
    Serial.print(dirAB == 0 ? F("A2B") : F("B2A"));
    Serial.print(',');
    Serial.print(f.id, HEX);
    Serial.print(',');
    Serial.print(f.extended ? 1 : 0);
    Serial.print(',');
    Serial.print(f.length);
    Serial.print(',');
    for (uint8_t i = 0; i < f.length && i < 8; i++) printHex2(f.data.bytes[i]);
    Serial.println();
}

bool loopSuppressed(uint8_t dirAB, const CAN_FRAME& f)
{
    if (loopWinMs == 0) return false;
    uint32_t now = millis();
    for (uint8_t i = 0; i < FWD_RING; i++) {
        const FwdRec& r = fwdRing[i];
        if (r.dir == dirAB) continue;             // same direction: not ours
        if (r.id != f.id || r.len != f.length) continue;
        bool same = true;
        for (uint8_t k = 0; k < f.length && k < 8; k++)
            if (r.data[k] != f.data.bytes[k]) { same = false; break; }
        if (!same) continue;
        if ((uint32_t)(now - r.t) < loopWinMs) return true;
    }
    return false;
}

void recordForward(uint8_t dirAB, const CAN_FRAME& f)
{
    FwdRec& r = fwdRing[fwdHead];
    r.id = f.id; r.len = f.length; r.dir = dirAB; r.t = millis();
    for (uint8_t k = 0; k < 8; k++)
        r.data[k] = (k < f.length) ? f.data.bytes[k] : 0;
    fwdHead = (uint8_t)((fwdHead + 1) % FWD_RING);
}

void doForward(uint8_t dirAB, CAN_FRAME& f)
{
    CANRaw* dst = (dirAB == 0) ? &Can1 : &Can0;
    busstat::Bus db = (dirAB == 0) ? busstat::Bus::CAN_BUS1 : busstat::Bus::CAN_BUS0;
    if (dst->sendFrame(f)) {
        if (dirAB == 0) stats.fwdAB++; else stats.fwdBA++;
        busstat::noteTx(db, f.length);
        recordForward(dirAB, f);
    } else {
        // TX ring full: frame lost.
        if (dirAB == 0) stats.dropAB++; else stats.dropBA++;
        busstat::noteTxFail(db);
    }
}

// Rate-limit check for a matched rule (or the default rule via defRateLim
// passed as lim). Returns true if the frame may proceed.
bool rateOk(Rule& r, uint16_t lim)
{
    if (lim == 0) return true;
    uint32_t now = millis();
    if ((uint32_t)(now - r.winStart) >= 1000) {
        r.winStart = now;
        r.winCount = 0;
    }
    if (r.winCount >= lim) { stats.rateDrop++; return false; }
    r.winCount++;
    return true;
}

// A scratch "rule" for the default pass-all path's rate limiter.
Rule defaultRule;

void handleFrame(uint8_t dirAB, CAN_FRAME& f)
{
    // Phase 6: drops feed the shared per-bus stats (src = where it came from).
    busstat::Bus sb = (dirAB == 0) ? busstat::Bus::CAN_BUS0 : busstat::Bus::CAN_BUS1;
    // 1. Global direction gate.
    if (dirMode == D_A2B && dirAB == D_B2A) {
        stats.dropBA++; busstat::noteDropped(sb); return;
    }
    if (dirMode == D_B2A && dirAB == D_A2B) {
        stats.dropAB++; busstat::noteDropped(sb); return;
    }
    // 2. Loop suppression (our own forwarded echo).
    if (loopSuppressed(dirAB, f)) {
        stats.loopSupp++; busstat::noteDropped(sb); return;
    }
    // 3. First matching rule wins. Phase 6: ID/mask via the shared
    // busfilter::idMatch() primitive (identical semantics to the old
    // inline ((f.id & mask) == (rule & mask)) form).
    for (uint8_t i = 0; i < nRules; i++) {
        Rule& r = rules[i];
        if (!busfilter::idMatch(f.id, r.id, r.mask)) continue;
        if (r.dir != D_BOTH && r.dir != dirAB) continue;
        r.hits++;
        if (!rateOk(r, r.rateLim)) { busstat::noteDropped(sb); return; }
        switch (r.action) {
            case A_DROP:
                if (dirAB == 0) stats.dropAB++; else stats.dropBA++;
                busstat::noteDropped(sb);
                return;
            case A_MODIFY: {
                bool modified = false;
                if (r.modOff < f.length && r.modOff < 8) {
                    uint8_t b = f.data.bytes[r.modOff];
                    f.data.bytes[r.modOff] =
                        (uint8_t)((b & (uint8_t)~r.modMask) | (r.modVal & r.modMask));
                    modified = true;
                }
                // modOff out of range: forwarded unmodified, not counted
                // (documented in mode_mitm.h).
                if (modified) {
                    if (dirAB == 0) stats.modAB++; else stats.modBA++;
                }
                doForward(dirAB, f);
                return;
            }
            case A_LOG:
                emitLog(dirAB, f);
                doForward(dirAB, f);
                return;
            case A_COUNT:
            case A_PASS:
            default:
                doForward(dirAB, f);
                return;
        }
    }
    // 4. Default: pass-all-both-directions (subject to the default rate cap).
    if (!rateOk(defaultRule, defRateLim)) { busstat::noteDropped(sb); return; }
    doForward(dirAB, f);
}

void pumpBus(uint8_t bus)
{
    CANRaw* src = (bus == 0) ? &Can0 : &Can1;
    uint8_t dirAB = (bus == 0) ? D_A2B : D_B2A;
    busstat::Bus sb = (bus == 0) ? busstat::Bus::CAN_BUS0 : busstat::Bus::CAN_BUS1;
    while (src->available() > 0) {
        CAN_FRAME f;
        src->read(f);
        if (bus == 0) loadFramesA++; else loadFramesB++;
        busstat::noteRx(sb, f.length);
        handleFrame(dirAB, f);
    }
}

// ---- console helpers ----
void conPrint(const __FlashStringHelper* s) { SerialUSB.println(s); }

void printRule(uint8_t i, const Rule& r)
{
    SerialUSB.print(F("  [")); SerialUSB.print(i); SerialUSB.print(F("] id=0x"));
    SerialUSB.print(r.id, HEX); SerialUSB.print(F(" mask=0x"));
    SerialUSB.print(r.mask, HEX); SerialUSB.print(F(" dir="));
    SerialUSB.print(r.dir == 0 ? F("a2b") : r.dir == 1 ? F("b2a") : F("both"));
    SerialUSB.print(F(" action="));
    SerialUSB.print(r.action == 0 ? F("pass") : r.action == 1 ? F("drop") :
                     r.action == 2 ? F("modify") : r.action == 3 ? F("count") : F("log"));
    if (r.action == A_MODIFY) {
        SerialUSB.print(F(" off=")); SerialUSB.print(r.modOff);
        SerialUSB.print(F(" mask=0x")); SerialUSB.print(r.modMask, HEX);
        SerialUSB.print(F(" val=0x")); SerialUSB.print(r.modVal, HEX);
    }
    if (r.rateLim) { SerialUSB.print(F(" rate=")); SerialUSB.print(r.rateLim); SerialUSB.print(F("/s")); }
    SerialUSB.print(F(" hits=")); SerialUSB.println(r.hits);
}

bool parseDirTok(const char* t, uint8_t& d)
{
    if (!strcasecmp(t, "a2b")) { d = D_A2B; return true; }
    if (!strcasecmp(t, "b2a")) { d = D_B2A; return true; }
    if (!strcasecmp(t, "both")) { d = D_BOTH; return true; }
    return false;
}

bool parseActTok(const char* t, uint8_t& a)
{
    if (!strcasecmp(t, "pass")) { a = A_PASS; return true; }
    if (!strcasecmp(t, "drop")) { a = A_DROP; return true; }
    if (!strcasecmp(t, "modify")) { a = A_MODIFY; return true; }
    if (!strcasecmp(t, "count")) { a = A_COUNT; return true; }
    if (!strcasecmp(t, "log")) { a = A_LOG; return true; }
    return false;
}

uint8_t addRule(const Rule& r)
{
    if (nRules >= MAX_RULES) return 0xFF;
    rules[nRules] = r;
    return nRules++;
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

} // namespace (anonymous)

void begin()
{
    // Acquire BEFORE enable (ownership rule): CAN0 + CAN1, independent
    // bauds. UART0/Bee was begun in setup() and is never released.
    modes::canBegin0(baudA);
    modes::canBegin1(baudB);
    promiscuous(&Can0);
    promiscuous(&Can1);
    // Stats reset on entry; rules persist (config, like UDS addressing).
    stats = Stats();
    defaultRule = Rule();
    fwdHead = 0;
    for (uint8_t i = 0; i < FWD_RING; i++) fwdRing[i] = FwdRec();
    loadWinA = loadWinB = millis();
    loadFramesA = loadFramesB = 0;
    loadPctA = loadPctB = 0;
    divert.sawF1 = false; divert.binActive = false;
    active = true;
    SerialUSB.print(F("[mitm] bridge up: CAN0<->CAN1 @ "));
    SerialUSB.print(baudA); SerialUSB.print('/'); SerialUSB.println(baudB);
}

void loop()
{
    // mm control-channel divert on Serial (supervision stays live; the
    // bridge has no other Serial data plane).
    while (Serial.available() > 0) {
        uint8_t b = (uint8_t)Serial.read();
        mm::divertByte(divert, b);
    }
    pumpBus(0);
    pumpBus(1);
    // 1 s bus-load windows.
    uint32_t now = millis();
    if ((uint32_t)(now - loadWinA) >= 1000) {
        loadPctA = calcLoad(loadFramesA, baudA);
        loadFramesA = 0; loadWinA = now;
    }
    if ((uint32_t)(now - loadWinB) >= 1000) {
        loadPctB = calcLoad(loadFramesB, baudB);
        loadFramesB = 0; loadWinB = now;
    }
}

void exit()
{
    active = false;
    modes::canEnd0();
    modes::canEnd1();
}

void console(char* line)
{
    while (*line == ' ') line++;
    // Tokenize (strtok is fine: line is a scratch buffer).
    char* tok[10];
    uint8_t ntok = 0;
    for (char* t = strtok(line, " \t"); t && ntok < 10; t = strtok(nullptr, " \t"))
        tok[ntok++] = t;
    if (ntok == 0 || !strcasecmp(tok[0], "help")) {
        conPrint(F("mitm rule add <id> <mask> <dir> <action> [off mask val] [rate=N]"));
        conPrint(F("mitm rule del <n> | mitm rule list | mitm clear"));
        conPrint(F("mitm stats | mitm stealth on|off | mitm dir a2b|b2a|both"));
        conPrint(F("dir: a2b b2a both; action: pass drop modify count log"));
        return;
    }
    if (!strcasecmp(tok[0], "stats")) {
        SerialUSB.print(F("fwd A->B=")); SerialUSB.print(stats.fwdAB);
        SerialUSB.print(F(" B->A=")); SerialUSB.println(stats.fwdBA);
        SerialUSB.print(F("drop A->B=")); SerialUSB.print(stats.dropAB);
        SerialUSB.print(F(" B->A=")); SerialUSB.println(stats.dropBA);
        SerialUSB.print(F("modified A->B=")); SerialUSB.print(stats.modAB);
        SerialUSB.print(F(" B->A=")); SerialUSB.println(stats.modBA);
        SerialUSB.print(F("loop-suppressed=")); SerialUSB.print(stats.loopSupp);
        SerialUSB.print(F(" rate-limited=")); SerialUSB.println(stats.rateDrop);
        SerialUSB.print(F("bus load A=")); SerialUSB.print(loadPctA);
        SerialUSB.print(F("% B=")); SerialUSB.print(loadPctB); SerialUSB.println('%');
        SerialUSB.print(F("dir="));
        SerialUSB.print(dirMode == 0 ? F("a2b") : dirMode == 1 ? F("b2a") : F("both"));
        SerialUSB.print(F(" stealth=")); SerialUSB.print(stealth ? F("on") : F("off"));
        SerialUSB.print(F(" loopWin=")); SerialUSB.print(loopWinMs); SerialUSB.println(F("ms"));
        SerialUSB.print(F("rules=")); SerialUSB.println(nRules);
        return;
    }
    if (!strcasecmp(tok[0], "clear")) {
        nRules = 0;
        stats = Stats();
        SerialUSB.println(F("[mitm] rules and stats cleared."));
        return;
    }
    if (!strcasecmp(tok[0], "stealth") && ntok >= 2) {
        if (!strcasecmp(tok[1], "on")) stealth = true;
        else if (!strcasecmp(tok[1], "off")) stealth = false;
        else { conPrint(F("usage: mitm stealth on|off")); return; }
        SerialUSB.print(F("[mitm] stealth "));
        SerialUSB.println(stealth ? F("on (pure forward; see docs)") : F("off"));
        return;
    }
    if (!strcasecmp(tok[0], "dir") && ntok >= 2) {
        uint8_t d;
        if (!parseDirTok(tok[1], d)) { conPrint(F("usage: mitm dir a2b|b2a|both")); return; }
        dirMode = d;
        SerialUSB.print(F("[mitm] direction "));
        SerialUSB.println(tok[1]);
        return;
    }
    if (!strcasecmp(tok[0], "rule") && ntok >= 2) {
        if (!strcasecmp(tok[1], "list")) {
            if (nRules == 0) conPrint(F("(no rules; default pass-all)"));
            for (uint8_t i = 0; i < nRules; i++) printRule(i, rules[i]);
            return;
        }
        if (!strcasecmp(tok[1], "del") && ntok >= 3) {
            char* e; long v = strtol(tok[2], &e, 10);
            if (*e != '\0' || v < 0 || v >= nRules) { conPrint(F("bad index")); return; }
            for (uint8_t i = (uint8_t)v; i + 1 < nRules; i++) rules[i] = rules[i + 1];
            nRules--;
            SerialUSB.println(F("[mitm] rule deleted."));
            return;
        }
        if (!strcasecmp(tok[1], "add") && ntok >= 6) {
            Rule r;
            char* e;
            r.id = (uint32_t)strtoul(tok[2], &e, 16);
            if (*e != '\0') { conPrint(F("bad id hex")); return; }
            r.mask = (uint32_t)strtoul(tok[3], &e, 16);
            if (*e != '\0') { conPrint(F("bad mask hex")); return; }
            if (!parseDirTok(tok[4], r.dir)) { conPrint(F("bad dir")); return; }
            if (!parseActTok(tok[5], r.action)) { conPrint(F("bad action")); return; }
            uint8_t pi = 6;
            if (r.action == A_MODIFY) {
                if (ntok < 9) { conPrint(F("modify needs: off mask val")); return; }
                long off = strtol(tok[6], &e, 10);
                if (*e != '\0' || off < 0 || off > 7) { conPrint(F("bad off (0-7)")); return; }
                r.modOff = (uint8_t)off;
                r.modMask = (uint8_t)strtoul(tok[7], &e, 16);
                if (*e != '\0') { conPrint(F("bad mod mask")); return; }
                r.modVal = (uint8_t)strtoul(tok[8], &e, 16);
                if (*e != '\0') { conPrint(F("bad mod val")); return; }
                pi = 9;
            }
            for (; pi < ntok; pi++) {
                if (!strncasecmp(tok[pi], "rate=", 5)) {
                    long rl = strtol(tok[pi] + 5, &e, 10);
                    if (*e != '\0' || rl < 0 || rl > 60000) {
                        conPrint(F("bad rate")); return;
                    }
                    r.rateLim = (uint16_t)rl;
                } else { conPrint(F("bad param")); return; }
            }
            // Phase 9: MODIFY rules originate modified frames — they need
            // the ALLOW_TRANSMIT grant (MITM defaults to READ_ONLY stealth).
            if (r.action == A_MODIFY &&
                !capgate::require(capgate::Grant::ALLOW_TRANSMIT, "mitm modify rule")) {
                conPrint(F("escalate with `cap grant 1` first"));
                return;
            }
            uint8_t idx = addRule(r);
            if (idx == 0xFF) { conPrint(F("rule table full (8)")); return; }
            SerialUSB.print(F("[mitm] rule added at ["));
            SerialUSB.print(idx); SerialUSB.println(']');
            return;
        }
    }
    conPrint(F("usage: mitm rule|stats|clear|stealth|dir|help"));
}

// ---- mm-channel ops ----

uint8_t mmRuleAdd(const uint8_t* p, uint8_t& idx)
{
    // p: [id LE32][mask LE32][dir][action][modOff][modMask][modVal][rateLim LE16]
    Rule r;
    r.id = getU32(p); r.mask = getU32(p + 4);
    r.dir = p[8]; r.action = p[9];
    r.modOff = p[10]; r.modMask = p[11]; r.modVal = p[12];
    r.rateLim = (uint16_t)(p[13] | ((uint16_t)p[14] << 8));
    if (r.dir > D_BOTH || r.action > A_LOG || r.modOff > 7)
        return mm::ST_BAD_ARG;
    // Phase 9: same MODIFY gate on the mm path.
    if (r.action == A_MODIFY &&
        !capgate::require(capgate::Grant::ALLOW_TRANSMIT, "mitm modify rule"))
        return mm::ST_ERR;
    idx = addRule(r);
    return (idx == 0xFF) ? mm::ST_ERR : mm::ST_OK;
}

uint8_t mmRuleDel(uint8_t idx, uint8_t& nRulesOut)
{
    if (idx == 0xFF) { nRules = 0; }
    else {
        if (idx >= nRules) return mm::ST_BAD_ARG;
        for (uint8_t i = idx; i + 1 < nRules; i++) rules[i] = rules[i + 1];
        nRules--;
    }
    nRulesOut = nRules;
    return mm::ST_OK;
}

uint8_t mmRuleList(uint8_t start, uint8_t* out,
                   uint8_t& nTotal, uint8_t& nRet)
{
    nTotal = nRules;
    nRet = 0;
    if (start >= nRules) return mm::ST_OK;
    for (uint8_t i = start; i < nRules && nRet < 2; i++, nRet++) {
        uint8_t* e = out + nRet * 20;
        const Rule& r = rules[i];
        e[0] = i;
        putU32(e + 1, r.id); putU32(e + 5, r.mask);
        e[9] = r.dir; e[10] = r.action;
        e[11] = r.modOff; e[12] = r.modMask; e[13] = r.modVal;
        e[14] = (uint8_t)(r.rateLim & 0xFF); e[15] = (uint8_t)(r.rateLim >> 8);
        putU32(e + 16, r.hits);
    }
    return mm::ST_OK;
}

void mmStats(uint8_t* out)
{
    putU32(out, stats.fwdAB); putU32(out + 4, stats.fwdBA);
    putU32(out + 8, stats.dropAB); putU32(out + 12, stats.dropBA);
    putU32(out + 16, stats.modAB); putU32(out + 20, stats.modBA);
    putU32(out + 24, stats.loopSupp); putU32(out + 28, stats.rateDrop);
    out[32] = loadPctA; out[33] = loadPctB;
    out[34] = dirMode; out[35] = stealth ? 1 : 0; out[36] = nRules;
}

uint8_t mmCfg(const uint8_t* p, uint8_t& applied)
{
    // p: [baudA LE32][baudB LE32][dir][stealth][loopWinMs LE16][defRateLim LE16]
    uint32_t bA = getU32(p), bB = getU32(p + 4);
    uint8_t d = p[8], st = p[9];
    uint16_t lw = (uint16_t)(p[10] | ((uint16_t)p[11] << 8));
    uint16_t dr = (uint16_t)(p[12] | ((uint16_t)p[13] << 8));
    if (!validBaud(bA) || !validBaud(bB) || d > D_BOTH || st > 1)
        return mm::ST_BAD_ARG;
    baudA = bA; baudB = bB; dirMode = d; stealth = (st != 0);
    loopWinMs = lw; defRateLim = dr;
    applied = 0;
    if (active) {
        // Live re-init of the bridge's own buses (safe: they're ours).
        modes::canBegin0(baudA);
        modes::canBegin1(baudB);
        promiscuous(&Can0);
        promiscuous(&Can1);
        applied = 1;
    }
    return mm::ST_OK;
}

void mmCfgGet(uint8_t* out)
{
    putU32(out, baudA); putU32(out + 4, baudB);
    out[8] = dirMode; out[9] = stealth ? 1 : 0;
    out[10] = (uint8_t)(loopWinMs & 0xFF); out[11] = (uint8_t)(loopWinMs >> 8);
    out[12] = (uint8_t)(defRateLim & 0xFF); out[13] = (uint8_t)(defRateLim >> 8);
    out[14] = active ? 1 : 0;
}

uint8_t suspendRules()
{
    uint8_t was = nRules;
    if (was > 0) {
        nRules = 0;
        SerialUSB.print(F("[mitm] PROG entry: rule table auto-suspended, "));
        SerialUSB.print(was);
        SerialUSB.println(F(" rule(s) cleared (NOT restored on exit)."));
    }
    return was;
}

} // namespace mitm
