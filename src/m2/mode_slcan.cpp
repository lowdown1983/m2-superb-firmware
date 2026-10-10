// M2 multi-mode firmware — SLCAN / Lawicel ASCII transport (Phase 12).
// See mode_slcan.h.

#include "mode_slcan.h"
#include "mm_channel.h"  // mm::Divert/divertByte (audit fix M-2)
#include "busstat.h"
#include "busfilter.h"
#include "capgate.h"
#include "debug.h"
#include <due_can.h>
#include <string.h>
#include <stdlib.h>

namespace slcan {
namespace {

bool open_ = false;
uint32_t speed = 500000;
bool timestamps = false;
uint32_t accMask = 0;   // 0 = no acceptance filtering
uint32_t accCode = 0;
bool accExt = false;

const uint32_t SPEEDS[9] = { 10000, 20000, 50000, 100000, 125000,
                             250000, 500000, 800000, 1000000 };

uint8_t hexVal(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0xFF;
}

void reply(const char* s)
{
    Serial.print(s);
    Serial.print('\r');
}
void replyChar(char c)
{
    Serial.print(c);
    Serial.print('\r');
}

// Emit one RX frame as t/T ASCII. Applies the acceptance filter when set.
void emitFrame(const CAN_FRAME& fr)
{
    if (accMask != 0) {
        if (!busfilter::idMatch(fr.id, accCode, accMask)) return;
        if ((bool)fr.extended != accExt) return;
    }
    char buf[40];
    int n = 0;
    if (fr.extended) {
        n = snprintf(buf, sizeof(buf), "T%08lX", (unsigned long)fr.id);
    } else {
        n = snprintf(buf, sizeof(buf), "t%03lX", (unsigned long)(fr.id & 0x7FF));
    }
    n += snprintf(buf + n, sizeof(buf) - n, "%X", fr.length & 0xF);
    for (uint8_t i = 0; i < fr.length && i < 8; i++)
        n += snprintf(buf + n, sizeof(buf) - n, "%02X", fr.data.bytes[i]);
    if (timestamps) {
        uint16_t ms = (uint16_t)(millis() & 0xFFFF);
        snprintf(buf + n, sizeof(buf) - n, "%04X", ms);
    }
    reply(buf);
    busstat::noteRx(busstat::Bus::CAN_BUS0, fr.length);
}

bool doOpen()
{
    if (open_) return true;
    if (!capgate::require(capgate::Grant::ALLOW_TRANSMIT, "SLCAN open"))
        return false;
    // Pin-safety: 255 = no transceiver-enable pin driving (invariant).
    if (!Can0.begin(speed, 255)) return false;
    for (int f = 0; f < 7; f++) Can0.setRXFilter(f, 0, 0, false);
    busstat::setPresent(busstat::Bus::CAN_BUS0, true);
    open_ = true;
    dbg::print(F("[slcan] open @"));
    dbg::println(speed);
    return true;
}

void doClose()
{
    if (!open_) return;
    Can0.disable();
    busstat::setPresent(busstat::Bus::CAN_BUS0, false);
    open_ = false;
    dbg::println(F("[slcan] closed"));
}

// Parse t/T/r/R transmit commands. Returns true on valid parse.
bool parseTx(const char* line, bool ext, bool rtr)
{
    size_t idChars = ext ? 8 : 3;
    size_t need = 1 + idChars + 1;  // cmd + id + len
    if (strlen(line) < need) return false;
    uint32_t id = 0;
    for (size_t i = 0; i < idChars; i++) {
        uint8_t v = hexVal(line[1 + i]);
        if (v > 0xF) return false;
        id = (id << 4) | v;
    }
    uint8_t len = hexVal(line[1 + idChars]);
    if (len > 8) return false;
    if (!rtr && strlen(line) < need + len * 2) return false;
    CAN_FRAME f;
    f.id = id;
    f.extended = ext;
    f.length = len;
    f.rtr = rtr ? 1 : 0;
    for (uint8_t i = 0; i < len && !rtr; i++) {
        uint8_t hi = hexVal(line[need + i * 2]);
        uint8_t lo = hexVal(line[need + i * 2 + 1]);
        if (hi > 0xF || lo > 0xF) return false;
        f.data.bytes[i] = (hi << 4) | lo;
    }
    if (!capgate::require(capgate::Grant::ALLOW_TRANSMIT, "SLCAN transmit"))
        return false;
    bool ok = Can0.sendFrame(f);
    if (ok) busstat::noteTx(busstat::Bus::CAN_BUS0, len);
    else busstat::noteTxFail(busstat::Bus::CAN_BUS0);
    return ok;
}

void pollAll()
{
    CAN_FRAME fr;
    while (Can0.available() > 0) {
        Can0.read(fr);
        emitFrame(fr);
    }
}

void handleLine(char* line)
{
    // Strip trailing CR/LF.
    size_t n = strlen(line);
    while (n > 0 && (line[n - 1] == '\r' || line[n - 1] == '\n'))
        line[--n] = '\0';
    if (n == 0) return;
    char c = line[0];
    switch (c) {
        case 'O':
            replyChar(doOpen() ? 'z' : 0x07);  // 'z' = ok; BEL = error
            break;
        case 'C':
            doClose();
            replyChar('z');
            break;
        case 'S':
            if (n == 2 && line[1] >= '0' && line[1] <= '8') {
                speed = SPEEDS[line[1] - '0'];
                if (open_) { doClose(); doOpen(); }
                replyChar('z');
            } else replyChar(0x07);
            break;
        case 't':
            replyChar(parseTx(line, false, false) ? 'z' : 0x07);
            break;
        case 'T':
            replyChar(parseTx(line, true, false) ? 'z' : 0x07);
            break;
        case 'r':
            replyChar(parseTx(line, false, true) ? 'z' : 0x07);
            break;
        case 'R':
            replyChar(parseTx(line, true, true) ? 'z' : 0x07);
            break;
        case 'F':
            reply("F00");
            break;
        case 'V':
            reply("V1013");
            break;
        case 'N':
            reply("N2208");
            break;
        case 'Z':
            if (n == 2 && (line[1] == '0' || line[1] == '1')) {
                timestamps = (line[1] == '1');
                replyChar('z');
            } else replyChar(0x07);
            break;
        case 'M':
        case 'm': {
            // M = mask, m = code; 8 hex chars (extended form).
            if (n != 9) { replyChar(0x07); break; }
            uint32_t v = 0;
            bool ok = true;
            for (int i = 0; i < 8; i++) {
                uint8_t hv = hexVal(line[1 + i]);
                if (hv > 0xF) { ok = false; break; }
                v = (v << 4) | hv;
            }
            if (!ok) { replyChar(0x07); break; }
            if (c == 'M') accMask = v; else { accCode = v; accExt = true; }
            if (c == 'm' && v <= 0x7FF) accExt = false;
            replyChar('z');
            break;
        }
        case 'P': {
            CAN_FRAME fr;
            if (Can0.available() > 0) {
                Can0.read(fr);
                emitFrame(fr);
            }
            replyChar('z');
            break;
        }
        case 'A':
            pollAll();
            replyChar('z');
            break;
        default:
            replyChar(0x07);  // BEL = unknown command
            break;
    }
}

char lineBuf[64];
uint8_t linePos = 0;

// Audit fix M-2 (2026-10-08): mm control-channel divert. Without this, the
// web GUI's mode-switch button times out in SLCAN mode.
mm::Divert divert;

} // namespace (anonymous)

void begin()
{
    open_ = false;
    linePos = 0;
    dbg::println(F("[slcan] mode begin (channel closed until 'O')"));
}

void loop()
{
    // Bee UART command pump (CR-terminated lines), with the mm
    // control-channel divert in front: bytes the divert consumes (F1 7E
    // frames) never reach the SLCAN line parser.
    while (Serial.available() > 0) {
        uint8_t b = (uint8_t)Serial.read();
        if (mm::divertByte(divert, b)) continue;
        char c = (char)b;
        if (c == '\r' || c == '\n') {
            lineBuf[linePos] = '\0';
            handleLine(lineBuf);
            linePos = 0;
        } else if (linePos < sizeof(lineBuf) - 1) {
            lineBuf[linePos++] = c;
        } else {
            linePos = 0;  // overlong line: resync
        }
    }
    // Auto-emit RX frames (Lawicel streams them; P/A poll on demand).
    if (open_) pollAll();
}

void exit()
{
    doClose();
    linePos = 0;
}

void console(char* line)
{
    if (strncmp(line, "status", 6) == 0) {
        SerialUSB.print(F("slcan: "));
        SerialUSB.print(open_ ? F("open") : F("closed"));
        SerialUSB.print(F(" @"));
        SerialUSB.println(speed);
    } else {
        SerialUSB.println(F("usage: slcan status"));
    }
}

} // namespace slcan
