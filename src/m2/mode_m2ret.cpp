// M2RET mode — GVRET binary protocol over Serial (Bee UART @115200).
//
// Line-by-line fidelity notes reference M2RET.ino @ ad9d5af unless noted.

#include "mode_m2ret.h"
#include "mm_channel.h"
#include "busfilter.h"  // Phase 6: unified filter model on GVRET output
#include "busstat.h"    // Phase 6: per-bus stats
#include "swcan.h"      // Phase 8: hardened SWCAN driver (TH8056 state machine)
#include "debug.h"      // Phase 8: dbg:: for the M2RET fixes
#include "capgate.h"    // Phase 12: SET_SECURITY_LEVEL -> grant escalation
#include "logrec.h"      // Phase 12: LOG START/STOP/STATUS/CLEAR
#include "mode_manager.h"// Phase 12: SET_BUS_CONFIG staging
#include "silent_pins.h"  // Audit fix F-2: TJA1051 S-pin drive on listen-only
#include <due_can.h>
#include <string.h>     // Phase 12: console tokenizing
#include <stdlib.h>     // Phase 12: atoi/atol/strtoul

// GVRET command bytes. VERIFIED: M2RET.h enum GVRET_PROTOCOL @ ad9d5af.
enum : uint8_t {
    PROTO_BUILD_CAN_FRAME  = 0,
    PROTO_TIME_SYNC        = 1,
    PROTO_DIG_INPUTS       = 2,
    PROTO_ANA_INPUTS       = 3,
    PROTO_SET_DIG_OUT      = 4,
    PROTO_SETUP_CANBUS     = 5,
    PROTO_GET_CANBUS_PARAMS = 6,
    PROTO_GET_DEV_INFO     = 7,
    PROTO_SET_SW_MODE      = 8,
    PROTO_KEEPALIVE        = 9,
    PROTO_SET_SYSTYPE      = 10,
    PROTO_ECHO_CAN_FRAME   = 11,
    PROTO_GET_NUMBUSES     = 12,
    PROTO_GET_EXT_BUSES    = 13,
    PROTO_SET_EXT_BUSES    = 14,
};

// Bus indices on the wire. VERIFIED: CommProtocol.txt + M2RET.ino usage.
enum : uint8_t { BUS_CAN0 = 0, BUS_CAN1 = 1, BUS_SWCAN = 2 };

// Parser states. VERIFIED: M2RET.h enum STATE @ ad9d5af.
// MM_CTRL is our extension (Phase 2): the multimode control channel
// (0xF1 0x7E ...). 0x7E is not a GVRET command (commands are 0-14 + 0xE7),
// and SavvyCAN never sends it, so this cannot collide with app traffic.
enum class PState : uint8_t {
    IDLE, GET_COMMAND, BUILD_CAN_FRAME, TIME_SYNC, SET_DIG_OUTPUTS,
    SETUP_CANBUS, SET_SINGLEWIRE_MODE, SET_SYSTYPE, ECHO_CAN_FRAME,
    SETUP_EXT_BUSES, MM_CTRL
};

namespace m2ret {
namespace {

// ---- runtime settings (RAM only in Phase 1; M2RET persists these in EEPROM)
uint32_t can0Speed = 500000;  // VERIFIED M2RET defaults (M2RET.ino:131/133)
uint32_t can1Speed = 500000;
uint32_t swcanSpeed = 33333;  // VERIFIED M2RET default (M2RET.ino:139)
bool can0Enabled = true;
bool can1Enabled = true;
bool can0ListenOnly = false;
bool can1ListenOnly = false;
bool swcanEnabled = true;
bool swcanListenOnly = false;
uint8_t sysType = 0;

// Tracks whether due_can begin(baud, 255) has run (set by the mode manager
// via m2ret::begin(bool)). disable() is only pin-safe after that: a raw
// disable() would drive Arduino 50/48 (J1850_PWM_VPW / 5V buck-boost).
// See mode_manager.cpp header.
bool canSafeToDisable = false;

// ---- parser state
PState state = PState::IDLE;
uint8_t buff[20];
int step = 0;
uint32_t buildInt = 0;
uint32_t linWord = 0;    // Phase 8: LIN1/LIN2 word accumulator
uint32_t lin1Word = 0;   // Phase 8: parsed LIN1 word (truthfulness check)
uint32_t lin2Word = 0;   // Phase 8: parsed LIN2 word (truthfulness check)
CAN_FRAME buildFrame;
uint8_t outBus = 0;

// XOR checksum. VERIFIED: M2RET.ino:418-427 ("Get the value of XOR'ing all
// the bytes together").
uint8_t checksumCalc(const uint8_t* buffer, int length)
{
    uint8_t valu = 0;
    for (int c = 0; c < length; c++) valu ^= buffer[c];
    return valu;
}

// Emit one received CAN frame on the Bee link in GVRET binary format.
// VERIFIED: M2RET.ino sendFrameToUSB() binary branch (lines 540-560):
//   0xF1, 0x00, timestamp micros LE(4), id LE(4, bit31=extended),
//   len + (bus<<4), data bytes, trailing 0x00.
// M2RET buffers and flushes every ~2ms; Phase 1 writes directly (simpler;
// the Bee UART at 115200 is the bottleneck either way).
void emitFrame(const CAN_FRAME& frame, uint8_t whichBus, uint32_t tsUs = 0)
{
    // Phase 6: the unified filter table gates GVRET output. Empty table =
    // accept everything (M2RET behavior unchanged until filters exist).
    // Bus indices line up with busfilter::Bus (CAN0=0, CAN1=1, SWCAN=2).
    if (busfilter::count() > 0) {
        busfilter::Proto pr = (whichBus == BUS_SWCAN)
            ? busfilter::Proto::SWCAN : busfilter::Proto::CANBUS;
        if (!busfilter::accept((busfilter::Bus)whichBus, pr,
                               busfilter::ADDR_ANY, frame.id,
                               busfilter::Dir::RX,
                               frame.data.bytes, frame.length)) {
            // Phase 8: drops counted for every bus the stats track
            // (native CAN in Phase 6, SWCAN added in Phase 8).
            busstat::Bus sb;
            if (whichBus == BUS_CAN0) sb = busstat::Bus::CAN_BUS0;
            else if (whichBus == BUS_CAN1) sb = busstat::Bus::CAN_BUS1;
            else sb = busstat::Bus::SWCAN;
            busstat::noteDropped(sb);
            return;
        }
    }
    // Phase 8: the SWCAN driver supplies a drain-time timestamp (INFERRED
    // precision); other buses stamp at emission. 0 = stamp now.
    uint32_t now = (tsUs != 0) ? tsUs : micros();
    uint32_t id = frame.id;
    if (frame.extended) id |= 0x80000000UL;
    uint8_t hdr[11];
    hdr[0] = 0xF1;
    hdr[1] = 0x00;
    hdr[2] = (uint8_t)(now & 0xFF);
    hdr[3] = (uint8_t)(now >> 8);
    hdr[4] = (uint8_t)(now >> 16);
    hdr[5] = (uint8_t)(now >> 24);
    hdr[6] = (uint8_t)(id & 0xFF);
    hdr[7] = (uint8_t)(id >> 8);
    hdr[8] = (uint8_t)(id >> 16);
    hdr[9] = (uint8_t)(id >> 24);
    hdr[10] = (frame.length & 0x0F) | (uint8_t)(whichBus << 4);
    Serial.write(hdr, 11);
    for (uint8_t c = 0; c < frame.length && c < 8; c++) {
        Serial.write(frame.data.bytes[c]);
    }
    Serial.write((uint8_t)0x00);  // trailing zero (CommProtocol.txt)
}

void sendCanFrame(uint8_t bus, const CAN_FRAME& frame)
{
    // VERIFIED dispatch: M2RET.ino:1058-1060.
    bool ok = false;
    if (bus == BUS_CAN0) ok = Can0.sendFrame(const_cast<CAN_FRAME&>(frame));
    else if (bus == BUS_CAN1) ok = Can1.sendFrame(const_cast<CAN_FRAME&>(frame));
    // Phase 8: SWCAN goes through the hardened driver (stats + dbg inside).
    else if (bus == BUS_SWCAN) ok = swcan::sendFrame(const_cast<CAN_FRAME&>(frame));
    // Phase 6: TX stats for the native buses (the swcan driver reports its
    // own via busstat::Bus::SWCAN).
    if (bus == BUS_CAN0 || bus == BUS_CAN1) {
        busstat::Bus sb = (bus == BUS_CAN1) ? busstat::Bus::CAN_BUS1
                                            : busstat::Bus::CAN_BUS0;
        if (ok) busstat::noteTx(sb, frame.length);
        else busstat::noteTxFail(sb);
    }
}

void applyCanBusConfig(uint8_t which, uint32_t word)
{
    // VERIFIED: M2RET.ino SETUP_CANBUS (lines 1098-1195).
    CANRaw& can = (which == 0) ? Can0 : Can1;
    uint32_t& speed = (which == 0) ? can0Speed : can1Speed;
    bool& enabled = (which == 0) ? can0Enabled : can1Enabled;
    bool& listenOnly = (which == 0) ? can0ListenOnly : can1ListenOnly;
    if (word > 0) {
        if (word & 0x80000000UL) {
            enabled = (word & 0x40000000UL) != 0;
            listenOnly = (word & 0x20000000UL) != 0;
            // Audit fix F-2: drive the TJA1051 S pin alongside the ABM
            // bit (pack INV-SILENT). which 0 -> HSC_S (71), 1 -> MSC_S (74).
            if (listenOnly) {
                can.enable_autobaud_listen_mode();
                silentpins::enter(which);
            } else {
                can.disable_autobaud_listen_mode();
                silentpins::exit(which);
            }
        } else {
            enabled = true;  // M2RET old behavior: bare speed => enable
        }
        word &= 0xFFFFF;
        if (word > 1000000) word = 1000000;
        // 255 = no transceiver-enable pin (M2 pin-safety rule, see
        // mode_manager.cpp header). M2RET does Can0.begin(speed, 255).
        // Grok audit: begin() can fail — only claim begun/safe-to-disable
        // on success; a failed begin leaves the bus disabled.
        if (can.begin(word, 255) != 0) {
            speed = word;
            canSafeToDisable = true;
            setPromiscuousMode();
        } else {
            enabled = false;
        }
    } else {
        // Guarded: a disable() before any begin(...,255) would drive
        // Arduino 50/48 (J1850_PWM_VPW / 5V buck-boost). Cannot happen via
        // the boot path (manager begins both CANs first), but a stray
        // disable-all command must never be the thing that finds out.
        if (canSafeToDisable) can.disable();
        enabled = false;
    }
}

void handleCommandByte(uint8_t cmd)
{
    uint32_t now = micros();
    switch (cmd) {
        case PROTO_BUILD_CAN_FRAME:
            state = PState::BUILD_CAN_FRAME; buff[0] = 0xF1; step = 0; break;
        case PROTO_TIME_SYNC: {
            // VERIFIED: M2RET.ino:841-850. Immediate reply, stays IDLE.
            uint8_t r[6] = {0xF1, PROTO_TIME_SYNC,
                (uint8_t)(now & 0xFF), (uint8_t)(now >> 8),
                (uint8_t)(now >> 16), (uint8_t)(now >> 24)};
            Serial.write(r, 6);
            break;
        }
        case PROTO_DIG_INPUTS: {
            // SIMPLIFICATION (Phase 1): no digital-input subsystem; reply
            // zero bitfield + XOR checksum.
            // PROTOCOL NOTE: CommProtocol.txt says a reply is "0xF1 followed
            // by the command byte", i.e. byte 1 = 2 here. M2RET's
            // implementation writes 6 in that position (M2RET.ino:857 —
            // apparent copy-paste from the GET_CANBUS_PARAMS handler just
            // below it). This implementation follows the documented protocol
            // (byte 1 = command byte). SavvyCAN never issues this command
            // during connect, so the difference is unobservable in practice.
            uint8_t r[4] = {0xF1, PROTO_DIG_INPUTS, 0x00, 0x00};
            r[3] = checksumCalc(r, 3);
            Serial.write(r, 4);
            break;
        }
        case PROTO_ANA_INPUTS: {
            // SIMPLIFICATION (Phase 1): zeros. M2RET returns 7 real analog
            // values (M2RET.ino:864-879).
            uint8_t r[17] = {0xF1, PROTO_ANA_INPUTS};
            r[16] = checksumCalc(r, 9);  // M2RET checksums first 9 bytes only
            Serial.write(r, 17);
            break;
        }
        case PROTO_SET_DIG_OUT:
            state = PState::SET_DIG_OUTPUTS; buff[0] = 0xF1; break;
        case PROTO_SETUP_CANBUS:
            state = PState::SETUP_CANBUS; step = 0; buff[0] = 0xF1; break;
        case PROTO_GET_CANBUS_PARAMS: {
            // VERIFIED: M2RET.ino:892-912. 12-byte reply.
            uint8_t r[12];
            r[0] = 0xF1; r[1] = PROTO_GET_CANBUS_PARAMS;
            r[2] = (can0Enabled ? 1 : 0) | (can0ListenOnly ? 0x10 : 0);
            r[3] = can0Speed & 0xFF; r[4] = (can0Speed >> 8) & 0xFF;
            r[5] = (can0Speed >> 16) & 0xFF; r[6] = (can0Speed >> 24) & 0xFF;
            r[7] = (can1Enabled ? 1 : 0) | (can1ListenOnly ? 0x10 : 0);
            r[8] = can1Speed & 0xFF; r[9] = (can1Speed >> 8) & 0xFF;
            r[10] = (can1Speed >> 16) & 0xFF; r[11] = (can1Speed >> 24) & 0xFF;
            Serial.write(r, 12);
            break;
        }
        case PROTO_GET_DEV_INFO: {
            // SIMPLIFICATION: build 1, zeros for EEPROM/file/autostart
            // fields (no persistence in Phase 1). M2RET.ino:914-924.
            uint8_t r[8] = {0xF1, PROTO_GET_DEV_INFO, 1, 0, 0, 0, 0, 0};
            Serial.write(r, 8);
            break;
        }
        case PROTO_SET_SW_MODE:
            state = PState::SET_SINGLEWIRE_MODE; step = 0; buff[0] = 0xF1; break;
        case PROTO_KEEPALIVE: {
            // VERIFIED: M2RET.ino:945-951. "if it isn't dead it responds DEAD"
            uint8_t r[4] = {0xF1, PROTO_KEEPALIVE, 0xDE, 0xAD};
            Serial.write(r, 4);
            break;
        }
        case PROTO_SET_SYSTYPE:
            state = PState::SET_SYSTYPE; step = 0; buff[0] = 0xF1; break;
        case PROTO_ECHO_CAN_FRAME:
            state = PState::ECHO_CAN_FRAME; buff[0] = 0xF1; step = 0; break;
        case PROTO_GET_NUMBUSES: {
            // VERIFIED: M2RET.ino:967-972. 3 = CAN0, CAN1, SWCAN.
            uint8_t r[3] = {0xF1, PROTO_GET_NUMBUSES, 3};
            Serial.write(r, 3);
            break;
        }
        case PROTO_GET_EXT_BUSES: {
            // VERIFIED: M2RET.ino:974-991. 17-byte reply. LIN buses report
            // disabled/0 (M2RET never implemented them either).
            uint8_t r[17];
            r[0] = 0xF1; r[1] = PROTO_GET_EXT_BUSES;
            r[2] = (swcanEnabled ? 1 : 0) | (swcanListenOnly ? 0x10 : 0);
            r[3] = swcanSpeed & 0xFF; r[4] = (swcanSpeed >> 8) & 0xFF;
            r[5] = (swcanSpeed >> 16) & 0xFF; r[6] = (swcanSpeed >> 24) & 0xFF;
            r[7] = 0; r[8] = 0; r[9] = 0; r[10] = 0; r[11] = 0;      // LIN1
            r[12] = 0; r[13] = 0; r[14] = 0; r[15] = 0; r[16] = 0;   // LIN2
            Serial.write(r, 17);
            break;
        }
        case PROTO_SET_EXT_BUSES:
            state = PState::SETUP_EXT_BUSES; step = 0; buff[0] = 0xF1; break;
        case mm::CMD_MULTIMODE:
            // Phase 2 control channel: F1 7E LEN PAYLOAD XOR. Length-prefixed
            // so payload bytes can never be mistaken for commands.
            mm::reset();
            state = PState::MM_CTRL;
            break;
        default:
            state = PState::IDLE;  // unknown command: resync (M2RET comment
            break;                 // "Any bytes between checksum and 0xF1 are thrown away")
    }
}

void handleByte(uint8_t inByte)
{
    // VERIFIED state machine: M2RET.ino loop() (lines 820-1280).
    switch (state) {
        case PState::IDLE:
            if (inByte == 0xF1) {
                state = PState::GET_COMMAND;
            } else if (inByte == 0xE7) {
                // VERIFIED: M2RET.ino:829-832. 0xE7 enables binary comm and
                // sets promiscuous mode.
                setPromiscuousMode();
            } else {
                // Phase 1: the Bee link is a pure GVRET data plane; stray
                // bytes are ignored. (M2RET routes these to its text console
                // on SerialUSB; ours lives on SerialUSB separately.)
            }
            break;
        case PState::GET_COMMAND:
            handleCommandByte(inByte);
            break;
        case PState::BUILD_CAN_FRAME: {
            // VERIFIED: M2RET.ino:1018-1075.
            buff[1 + step] = inByte;
            switch (step) {
                case 0: buildFrame.id = inByte; break;
                case 1: buildFrame.id |= (uint32_t)inByte << 8; break;
                case 2: buildFrame.id |= (uint32_t)inByte << 16; break;
                case 3:
                    buildFrame.id |= (uint32_t)inByte << 24;
                    if (buildFrame.id & 0x80000000UL) {
                        buildFrame.id &= 0x7FFFFFFFUL;
                        buildFrame.extended = true;
                    } else buildFrame.extended = false;
                    break;
                case 4: outBus = inByte & 3; break;
                case 5:
                    buildFrame.length = inByte & 0xF;
                    if (buildFrame.length > 8) buildFrame.length = 8;
                    break;
                default:
                    if (step < (int)buildFrame.length + 6) {
                        buildFrame.data.bytes[step - 6] = inByte;
                    } else {
                        state = PState::IDLE;
                        // Phase 8 fix (was: calculated-but-not-checked, like
                        // upstream M2RET.ino:1049): enforce the XOR checksum.
                        // A bad checksum means a corrupt host frame — drop
                        // it, never transmit it.
                        if (checksumCalc(buff, step) != inByte) {
                            dbg::println(F("[m2ret] BUILD_CAN_FRAME: bad checksum, dropped"));
                            break;
                        }
                        // Phase 8 fix: bus-count truthfulness — only buses
                        // 0..2 exist (CAN0, CAN1, SWCAN).
                        if (outBus > BUS_SWCAN) {
                            dbg::println(F("[m2ret] BUILD_CAN_FRAME: bad bus, dropped"));
                            break;
                        }
                        // VERIFIED GMLAN quirk (M2RET.ino:1052-1067): id
                        // 0x100 on SWCAN goes out in high-speed mode.
                        // Phase 8: via the hardened driver's hsEnter/hsExit
                        // (datasheet settling waits included).
                        bool hs = (buildFrame.id == 0x100 && outBus == BUS_SWCAN);
                        if (hs) swcan::hsEnter();
                        buildFrame.rtr = 0;
                        sendCanFrame(outBus, buildFrame);
                        if (hs) swcan::hsExit();
                    }
                    break;
            }
            step++;
            break;
        }
        case PState::TIME_SYNC:
            state = PState::IDLE;  // no payload; reply was immediate
            break;
        case PState::SET_DIG_OUTPUTS:
            // SIMPLIFICATION: accepted, ignored (no output subsystem).
            state = PState::IDLE;
            break;
        case PState::SETUP_CANBUS: {
            // VERIFIED: M2RET.ino:1098-1195.
            switch (step) {
                case 0: buildInt = inByte; break;
                case 1: buildInt |= (uint32_t)inByte << 8; break;
                case 2: buildInt |= (uint32_t)inByte << 16; break;
                case 3:
                    buildInt |= (uint32_t)inByte << 24;
                    applyCanBusConfig(0, buildInt);
                    break;
                case 4: buildInt = inByte; break;
                case 5: buildInt |= (uint32_t)inByte << 8; break;
                case 6: buildInt |= (uint32_t)inByte << 16; break;
                case 7:
                    buildInt |= (uint32_t)inByte << 24;
                    applyCanBusConfig(1, buildInt);
                    state = PState::IDLE;
                    setPromiscuousMode();
                    break;
            }
            step++;
            break;
        }
        case PState::SET_SINGLEWIRE_MODE: {
            // Phase 8 fix (was: the mode byte was never consumed — it fell
            // through and was parsed as the next command byte, desyncing
            // the parser). Upstream M2RET.ino:1197-1203 ignores this command
            // (behavior VERIFIED against the archived source); we map the
            // mode byte onto the TH8056 truthfully: 0 sleep, 1 high-speed,
            // 2 HV wake-up, 3 normal. Anything else is rejected (logged).
            swcan::Mode m = swcan::Mode::SLEEP;
            bool valid = true;
            switch (inByte) {
                case 0: m = swcan::Mode::SLEEP; break;
                case 1: m = swcan::Mode::HIGHSPEED; break;
                case 2: m = swcan::Mode::HV_WAKEUP; break;
                case 3: m = swcan::Mode::NORMAL; break;
                default: valid = false; break;
            }
            if (!valid) {
                dbg::print(F("[m2ret] SET_SW_MODE: bad mode byte "));
                dbg::println((uint32_t)inByte);
            } else if (swcan::acquired()) {
                swcan::setMode(m);
            } else {
                dbg::println(F("[m2ret] SET_SW_MODE: SWCAN not active, ignored"));
            }
            state = PState::IDLE;
            break;
        }
        case PState::SET_SYSTYPE:
            sysType = inByte;  // RAM only in Phase 1 (no EEPROM)
            state = PState::IDLE;
            break;
        case PState::ECHO_CAN_FRAME: {
            // VERIFIED: M2RET.ino:1211-1263. Same framing as BUILD_CAN_FRAME
            // but bus is 1 bit; the frame is re-emitted as if received.
            // NOTE: M2RET re-emits with whichBus hardcoded to 0
            // (M2RET.ino:1258) — replicated faithfully.
            buff[1 + step] = inByte;
            switch (step) {
                case 0: buildFrame.id = inByte; break;
                case 1: buildFrame.id |= (uint32_t)inByte << 8; break;
                case 2: buildFrame.id |= (uint32_t)inByte << 16; break;
                case 3:
                    buildFrame.id |= (uint32_t)inByte << 24;
                    if (buildFrame.id & 0x80000000UL) {
                        buildFrame.id &= 0x7FFFFFFFUL;
                        buildFrame.extended = true;
                    } else buildFrame.extended = false;
                    break;
                case 4: outBus = inByte & 1; break;
                case 5:
                    buildFrame.length = inByte & 0xF;
                    if (buildFrame.length > 8) buildFrame.length = 8;
                    break;
                default:
                    if (step < (int)buildFrame.length + 6) {
                        buildFrame.data.bytes[step - 6] = inByte;
                    } else {
                        state = PState::IDLE;
                        // Phase 8: enforce the checksum (was ignored, like
                        // upstream M2RET.ino:1049).
                        if (checksumCalc(buff, step) != inByte) {
                            dbg::println(F("[m2ret] ECHO_CAN_FRAME: bad checksum, dropped"));
                            break;
                        }
                        // Phase 8 fix — DEVIATION from upstream
                        // (M2RET.ino:1258 hardcodes bus 0): honor the bus
                        // bit the host actually sent.
                        emitFrame(buildFrame, (uint8_t)(outBus & 1));
                    }
                    break;
            }
            step++;
            break;
        }
        case PState::SETUP_EXT_BUSES: {
            // VERIFIED: M2RET.ino:1266-1330. 12 bytes: SWCAN word, LIN1
            // word, LIN2 word.
            switch (step) {
                case 0: buildInt = inByte; break;
                case 1: buildInt |= (uint32_t)inByte << 8; break;
                case 2: buildInt |= (uint32_t)inByte << 16; break;
                case 3: {
                    buildInt |= (uint32_t)inByte << 24;
                    // Phase 8: SWCAN consistency — every path goes through
                    // the hardened driver (was: mode() driven before
                    // setupSW(), the listen-only flag was set but never
                    // applied to the MCP2515, and the disable path left
                    // the PIOC16 ISR attached).
                    if (buildInt > 0) {
                        bool en = true;
                        if (buildInt & 0x80000000UL) {
                            en = (buildInt & 0x40000000UL) != 0;
                            swcanListenOnly = (buildInt & 0x20000000UL) != 0;
                        }
                        uint32_t spd = buildInt & 0xFFFFF;
                        if (spd > 100000) spd = 100000;
                        if (spd == 0) spd = 33333;  // INFERRED sane default
                        swcanSpeed = spd;
                        if (en) {
                            // Phase 8: listen-only is now actually applied
                            // (was: flag set, MCP2515 never told).
                            // Reviewer M1 (Phase 8): acquire() refuses when
                            // already acquired (M2RET entry acquires), so a
                            // re-sent SET_EXT_BUSES (every SavvyCAN connect)
                            // must go through reconfigure() — otherwise the
                            // new speed/listen-only are staged but never
                            // reach the MCP2515 while GET_EXT_BUSES reports
                            // them (silent reported-vs-actual divergence).
                            if (swcan::acquired()) {
                                swcan::reconfigure(spd);
                                swcan::setListenOnly(swcanListenOnly);
                            } else if (swcan::acquire(spd)) {
                                swcan::setListenOnly(swcanListenOnly);
                            }
                            swcanEnabled = swcan::acquired();
                        } else {
                            swcan::release();
                            swcanEnabled = false;
                        }
                    } else {
                        swcan::release();
                        swcanEnabled = false;
                    }
                    break;
                }
                case 4: case 8:
                    linWord = inByte;
                    break;
                case 5: case 6: case 9: case 10:
                    linWord |= (uint32_t)inByte << (8 * (step % 4));
                    break;
                case 7:
                    linWord |= (uint32_t)inByte << 24;
                    lin1Word = linWord;
                    break;
                case 11: {
                    linWord |= (uint32_t)inByte << 24;
                    lin2Word = linWord;
                    // Phase 8 fix: LIN handling truthfulness. There is no
                    // LIN driver (the LIN master stack is reserved for
                    // optional Phase 10b — NOT built here). A zero word is
                    // a silent no-op; a non-zero word is the host requesting
                    // LIN we cannot provide — say so instead of pretending
                    // success (was: accepted and ignored unconditionally,
                    // like upstream M2RET whose LIN init is commented out).
                    if (lin1Word != 0 || lin2Word != 0) {
                        dbg::println(F("[m2ret] SET_EXT_BUSES: LIN requested but unsupported (no LIN driver); ignored"));
                    }
                    state = PState::IDLE;
                    break;
                }
                default:
                    break;
            }
            step++;
            break;
        }
        case PState::MM_CTRL:
            // Phase 2 control channel (mm_channel.h). feedByte() returns
            // true when the frame completed and was dispatched.
            if (mm::feedByte(inByte)) state = PState::IDLE;
            break;
    }
}

} // namespace (anonymous)

void begin(bool canSafe)
{
    state = PState::IDLE;
    step = 0;
    canSafeToDisable = canSafe;
}

void setPromiscuousMode()
{
    // VERIFIED: M2RET.ino:398-412.
    for (int f = 0; f < 3; f++) {
        Can0.setRXFilter(f, 0, 0, true);
        Can1.setRXFilter(f, 0, 0, true);
    }
    for (int f = 3; f < 7; f++) {
        Can0.setRXFilter(f, 0, 0, false);
        Can1.setRXFilter(f, 0, 0, false);
    }
}

// Phase 8: all SWCAN lifecycle goes through the hardened swcan:: driver
// (TH8056 state machine, datasheet settling, health monitoring). The ISR
// trampoline lives in swcan.cpp now.

void swcanSleep()
{
    // Hardened release(): TH8056 to sleep via GPIO, plus MCP2515 to sleep
    // when it was acquired (card quiesce rule). Still safe when the
    // MCP2515 was never initialized — release() degrades to GPIO-only.
    // Note: up to ~100 ms TH8056 settle, only when SWCAN was acquired.
    swcan::release();
}

void swcanEnter()
{
    // Phase 8: acquire() does MCP2515 init + TH8056 NORMAL (with the
    // Sleep->Normal settle) + INT attach, and tracks presence.
    swcan::acquire(swcanSpeed);  // 33333 default (M2RET.ino:139)
    swcanEnabled = swcan::acquired();
}

// Live speed change (mm SET_SWCAN_SPEED). Updates the stored speed; if the
// SWCAN block is currently acquired, re-runs init immediately.
void swcanSetSpeed(uint32_t speed)
{
    swcanSpeed = speed;
    if (swcan::acquired()) {
        swcan::reconfigure(swcanSpeed);
        swcanEnabled = swcan::acquired();
    }
}

void setCanSpeeds(uint32_t c0, uint32_t c1)
{
    can0Speed = c0;
    can1Speed = c1;
}

void swcanExit()
{
    swcan::release();  // Phase 8: INT detached, TH8056 to sleep, idempotent
    swcanEnabled = false;
}

void loop()
{
    // Pump the Bee UART through the GVRET parser. Cap per-loop iterations
    // so a flooded link can't starve CAN polling (INFERRED tuning: 128, the
    // same bound M2RET uses for its serial pump — M2RET.ino:814).
    int serialCnt = 0;
    while (Serial.available() > 0 && serialCnt < 128) {
        serialCnt++;
        handleByte((uint8_t)Serial.read());
    }

    // Poll buses and emit received frames. VERIFIED pattern: M2RET.ino
    // loop() (~lines 750-775): Can0/Can1 via available()/read(), SWCAN via
    // GetRXFrame().
    CAN_FRAME incoming;
    if (Can0.available() > 0) {
        Can0.read(incoming);
        busstat::noteRx(busstat::Bus::CAN_BUS0, incoming.length);
        emitFrame(incoming, BUS_CAN0);
    }
    if (Can1.available() > 0) {
        Can1.read(incoming);
        busstat::noteRx(busstat::Bus::CAN_BUS1, incoming.length);
        emitFrame(incoming, BUS_CAN1);
    }
    // Phase 8: the hardened SWCAN driver drains with a drain-time
    // timestamp, applies the filter table, feeds stats, and runs its
    // MCP2515 health + bus-off poll (no-op unless acquired).
    swcan::poll();
    uint32_t swTs = 0;
    if (swcan::getFrame(incoming, &swTs)) {
        emitFrame(incoming, BUS_SWCAN, swTs);
    }
}

// ---- Phase 12: M2RET modernization console ----
// Text commands on the USB console when M2RET mode is active:
//   CAPABILITIES GET_VERSION GET_BUSES GET_BUS_STATUS SET_BUS_CONFIG
//   SET_FILTER CLEAR_FILTER GET_STATS RESET_STATS
//   LOG START|STOP|STATUS|CLEAR  SET_SECURITY_LEVEL <0..4>
//
// These are the human-console face of the mm supervision channel; they
// reuse the same underlying ops (busstat, busfilter, capgate, logrec).

void console(char* line)
{
    // Tokenize the verb (uppercase per the command names).
    char* sp = strchr(line, ' ');
    if (sp) *sp = '\0';
    const char* arg = sp ? sp + 1 : "";

    if (strcmp(line, "CAPABILITIES") == 0) {
        SerialUSB.println(F("GET_VERSION GET_BUSES GET_BUS_STATUS "
                             "SET_BUS_CONFIG SET_FILTER CLEAR_FILTER "
                             "GET_STATS RESET_STATS LOG SET_SECURITY_LEVEL"));
    } else if (strcmp(line, "GET_VERSION") == 0) {
        // Phase stamp: keep in sync with the boot banner.
        SerialUSB.println(F("M2-MULTIMODE phase12+ (7 modes + gateway + "
                             "slcan + harness)"));
    } else if (strcmp(line, "GET_BUSES") == 0) {
        SerialUSB.println(F("0=CAN0 1=CAN1 2=SWCAN 3=J1850 4=KLINE"));
    } else if (strcmp(line, "GET_BUS_STATUS") == 0) {
        for (int b = 0; b < (int)busstat::Bus::COUNT; b++) {
            const busstat::Stats& s = busstat::get((busstat::Bus)b);
            SerialUSB.print(F("bus"));
            SerialUSB.print(b);
            SerialUSB.print(busstat::present((busstat::Bus)b) ? F("+ ") : F("- "));
            SerialUSB.print(F("tx"));
            SerialUSB.print(s.txFrames);
            SerialUSB.print(F("/rx"));
            SerialUSB.print(s.rxFrames);
            SerialUSB.print(F(" ovf"));
            SerialUSB.print(s.rxOverflow);
            SerialUSB.print(F(" tfail"));
            SerialUSB.print(s.txFail);
            SerialUSB.print(F(" boff"));
            SerialUSB.print(s.busOff);
            SerialUSB.print(F(" drop"));
            SerialUSB.println(s.dropped);
        }
    } else if (strcmp(line, "SET_BUS_CONFIG") == 0) {
        // SET_BUS_CONFIG <bus 0|1> <baud> — staged via the manager
        // (applied live if the mode owns the bus, same as the web path).
        int bus = atoi(arg);
        const char* bs = strchr(arg, ' ');
        if (!bs || (bus != 0 && bus != 1)) {
            SerialUSB.println(F("usage: SET_BUS_CONFIG <0|1> <baud>"));
            return;
        }
        uint32_t baud = (uint32_t)atol(bs + 1);
        uint8_t applied = 0;
        modes::ModeManager* mgr = modes::manager();
        if (!mgr || mgr->mmSetCanBaud((uint8_t)bus, baud, applied) != 0) {
            SerialUSB.println(F("ERR"));
        } else {
            SerialUSB.print(F("OK applied="));
            SerialUSB.println(applied);
        }
    } else if (strcmp(line, "SET_FILTER") == 0) {
        // SET_FILTER <bus> <proto> <id-hex> <mask-hex> [dir 0|1|2]
        // Minimal form of the 20-byte filter (addr=any, no payload cond).
        char tmp[64];
        strncpy(tmp, arg, sizeof(tmp) - 1);
        tmp[sizeof(tmp) - 1] = '\0';
        char* tok = strtok(tmp, " ");
        if (!tok) { SerialUSB.println(F("usage: SET_FILTER <bus> <proto> <id-hex> <mask-hex> [dir]")); return; }
        int bus = atoi(tok);
        tok = strtok(nullptr, " "); if (!tok) { SerialUSB.println(F("ERR args")); return; }
        int proto = atoi(tok);
        tok = strtok(nullptr, " "); if (!tok) { SerialUSB.println(F("ERR args")); return; }
        uint32_t id = strtoul(tok, nullptr, 16);
        tok = strtok(nullptr, " "); if (!tok) { SerialUSB.println(F("ERR args")); return; }
        uint32_t mask = strtoul(tok, nullptr, 16);
        tok = strtok(nullptr, " ");
        int dir = tok ? atoi(tok) : 2;
        busfilter::Filter f;
        f.used = true;
        f.bus = (busfilter::Bus)bus;
        f.proto = (busfilter::Proto)proto;
        f.id = id;
        f.mask = mask;
        f.dir = (busfilter::Dir)dir;
        int8_t idx = busfilter::add(f);
        if (idx < 0) SerialUSB.println(F("ERR full"));
        else { SerialUSB.print(F("OK idx=")); SerialUSB.println(idx); }
    } else if (strcmp(line, "CLEAR_FILTER") == 0) {
        busfilter::clear();
        SerialUSB.println(F("OK"));
    } else if (strcmp(line, "GET_STATS") == 0) {
        for (int b = 0; b < (int)busstat::Bus::COUNT; b++) {
            const busstat::Stats& s = busstat::get((busstat::Bus)b);
            SerialUSB.print(b);
            SerialUSB.print(F(":"));
            SerialUSB.print(s.txFrames); SerialUSB.print(F(","));
            SerialUSB.print(s.rxFrames); SerialUSB.print(F(","));
            SerialUSB.print(s.txBytes); SerialUSB.print(F(","));
            SerialUSB.print(s.rxBytes); SerialUSB.print(F(","));
            SerialUSB.print(s.rxOverflow); SerialUSB.print(F(","));
            SerialUSB.print(s.txFail); SerialUSB.print(F(","));
            SerialUSB.print(s.busOff); SerialUSB.print(F(","));
            SerialUSB.println(s.dropped);
        }
    } else if (strcmp(line, "RESET_STATS") == 0) {
        busstat::resetAll();
        SerialUSB.println(F("OK"));
    } else if (strcmp(line, "LOG") == 0) {
        if (strcmp(arg, "START") == 0) { logrec::start(); SerialUSB.println(F("OK")); }
        else if (strcmp(arg, "STOP") == 0) { logrec::stop(); SerialUSB.println(F("OK")); }
        else if (strcmp(arg, "STATUS") == 0) logrec::status();
        else if (strcmp(arg, "CLEAR") == 0) { logrec::clear(); SerialUSB.println(F("OK")); }
        else SerialUSB.println(F("usage: LOG START|STOP|STATUS|CLEAR"));
    } else if (strcmp(line, "SET_SECURITY_LEVEL") == 0) {
        // Maps onto the Phase 9 capability gates (upward-only, warned,
        // never above the mode ceiling). The "security level" name is the
        // M2RET-modernization alias for the grant level.
        int lvl = atoi(arg);
        if (lvl < 0 || lvl > 4) {
            SerialUSB.println(F("usage: SET_SECURITY_LEVEL <0..4>"));
            return;
        }
        capgate::Grant g = capgate::escalate((capgate::Grant)lvl);
        SerialUSB.print(F("OK level="));
        SerialUSB.println(capgate::name(g));
    } else {
        SerialUSB.println(F("unknown; try CAPABILITIES"));
    }
}

} // namespace m2ret
