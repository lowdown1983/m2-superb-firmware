#pragma once
// M2 multi-mode firmware — SAE J1850 driver (Phases 2, 7, 8).
//
// Drives the M2's J1850 hardware (pins VERIFIED in variant.h/variant.cpp and
// Macchina Docs.md):
//   PS_J1850_9141 = 49/PB5  (HIGH = rail power on — SHARED with K-line,
//                             owned via rail9141, Phase 7)
//   J1850_PWM_VPW = 50/PB8  (HIGH = PWM 41.6k Ford / LOW = VPW 10.4k GM)
//   J1850_PWM_RX  = 51/PC28  J1850_VPW_RX = 52/PC26
//   J1850P_TX     = 53/PC18  J1850N_TX    = 54/PC23
//
// TIMING SOURCES (nothing invented):
//   VPW symbols: matafonoff/J1850-VPW-Arduino-Transceiver-Library (Ms-PL,
//     vehicle-tested on a Chrysler Pacifica). TX: SOF 200us, short 64us,
//     long 128us, EOD 200us, EOF 280us. RX windows: SOF 163-239us,
//     short 34-96us, long 97-163us, EOD 164-239us, glitch filter <5us.
//     Bit rule: active-long or passive-short = 1; active-short or
//     passive-long = 0.
//   PWM symbols: SparkFun "OBD II Getting Started" guide ('1' = 8us,
//     '0' = 16us, SOF = 48us @ 41.6kbaud) + retrolib BAE 5030 (PWM =
//     fixed bit time, long dominates short). EOD/EOF/IFS values are
//     INFERRED scalings, documented in j1850.cpp.
//   CRC-8/J1850: the crc() routine from the matafonoff library
//     (init 0xFF, poly 0x1D, final complement).
//
// RX METHOD (Phase 2, kept in Phase 8): GPIO pin-change interrupts +
// micros() timestamps — the matafonoff approach, proven on-vehicle. The
// TC6/TC7 timer-capture channels these pins are routed to are DELIBERATELY
// NOT used: register-level TC code cannot be validated without a bench,
// and the GPIO approach is the cited proven one. Consequence: PWM 8us
// symbols are near the interrupt-latency floor on SAM3X — flagged
// INFERRED, needs a vehicle test. Phase 8 makes the timer allocation
// EXPLICIT (see RESOURCE_OWNERSHIP.md): J1850 owns the GPIO+IRQ pins and
// explicitly does NOT claim TC6/TC7, keeping the mutual exclusion with
// SWCAN-SOF (TC7_B) and RGB-blue (TC6_A) intact.
//
// POLARITY: the RX idle level is sampled at acquire() (bus idles passive),
// so RX adapts to the transceiver without assuming its polarity. TX
// polarity (pin HIGH = bus active) is INFERRED — bench must confirm the
// transceiver does not invert.
//
// PHASE 8 HARDENING: explicit RX/TX state machines, PWM arbitration
// (long-dominates-short), header decode (1- and 3-byte), formal idle
// detection, error counters, optional CRC gate on the monitor path, and
// Phase 6 transport wiring (busfilter gates on RX, busstat counters).
// The rail9141 integration from Phase 7 is kept verbatim: pin 50 settled
// before pin 49 HIGH (INV-MODE50/INV-PWR49), release via rail9141.
//
// EVERYTHING IN THIS DRIVER IS INFERRED UNTIL TESTED ON A VEHICLE.

#include <Arduino.h>

namespace j1850 {

enum class Proto { VPW, JPWM }; // JPWM, not PWM: CMSIS defines PWM as a peripheral-base macro

// Error counters (Phase 8). Also mirrored into busstat where a per-bus
// counter exists (rxOverflow / txFail / dropped).
struct Counters {
    uint32_t symErr = 0;      // out-of-window symbols (RX state machine aborts)
    uint32_t crcErr = 0;       // CRC failures (monitor gate + transact)
    uint32_t arbLost = 0;      // arbitration losses (VPW + PWM)
    uint32_t qOverflow = 0;    // monitor queue overruns
    uint32_t glitch = 0;       // sub-threshold edges filtered
};
const Counters& counters();
void resetCounters();

// Decoded J1850 header (Phase 8). hdrLen is 1 or 3; for 3-byte headers
// (OBD-style) h0=priority, h1=target, h2=source.
struct Header {
    uint8_t len = 0;       // 1 or 3 (0 = decode failed)
    uint8_t h0 = 0, h1 = 0, h2 = 0;
    uint8_t priority = 0;  // h0>>5 for 1-byte (INFERRED); raw h0 for 3-byte
    uint8_t target = 0;    // 3-byte only
    uint8_t source = 0;    // 3-byte only (the Phase 6 OEM-seam node byte)
};
// Decode the header at frame[0..]. hdrLen: 1 or 3. False on bad hdrLen or
// a frame too short to hold the header.
bool decodeHeader(const uint8_t* frame, uint8_t n, uint8_t hdrLen, Header& h);
// Monitor-path default header length (1 or 3). transact() uses the
// request's own header length. Default 3 (OBD-style).
void setHeaderLen(uint8_t n);
uint8_t headerLen();

// CRC gate for the monitor path (Phase 8): when true, pollFrame() drops
// frames with bad CRC (counted as crcErr). Default false — ATMA shows raw
// traffic, like upstream tools.
void setCrcGate(bool on);
bool crcGate();

// Acquire the J1850 block. Powers the rail (via rail9141), selects PWM/VPW,
// attaches the RX interrupt. Returns false if already acquired.
bool acquire(Proto p);
// Release: ISR detached, TX pins to safe idle, rail released refcounted.
// Idempotent.
void release();
bool acquired();
Proto proto();

// Formal idle detection (Phase 8): true when the bus has been passive for
// at least one EOF (VPW) / EOD gap (PWM).
bool idle();
// Block until idle() or timeoutMs elapses. Used before every transmission.
bool waitIdle(uint32_t timeoutMs);

// Blocking transact: sends tx[txLen] (header+data; CRC-8 appended here),
// waits for one response frame, verifies CRC, strips it into rx[].
// hdrLen: header length of the response for node-byte extraction (0 =
// the monitor default). Returns true on success. Disables interrupts
// around TX (bit-bang timing).
bool transact(const uint8_t* tx, uint8_t txLen,
              uint8_t* rx, uint8_t rxCap, uint8_t& rxLen,
              uint16_t timeoutMs = 1000, uint8_t hdrLen = 0);

// Monitor support: while acquired, received frames are queued regardless;
// pollFrame() drains one (CRC gate + busfilter apply here). Used by ELM ATMA.
bool pollFrame(uint8_t* buf, uint8_t cap, uint8_t& len);
uint8_t queueDepth();

} // namespace j1850
