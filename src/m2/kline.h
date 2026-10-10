#pragma once
// M2 multi-mode firmware — ISO 9141-2 / KWP2000 (ISO 14230) K-line driver (Phase 7).
//
// HARDWARE (VERIFIED, knowledge pack v1.4.0 — never the banned v2.6.x specs):
//   K-line: TJA1027 U7; TX = Arduino 55/PA11 (TXD0/USART0 = Serial1 TX),
//           RX = 56/PA10 (RXD0 = Serial1 RX), SLP = 57/PB4 (transceiver
//           sleep control; SLP_N naming => active-LOW assumed, INFERRED).
//           OBD-II pin 7. Max 20 kBd (S-TJA1027); we run 10400.
//   L-line: TJA1027 U8; TX = 58/PA13, RX = 59/PA12 (Serial2), SLP = 60/PB7.
//           Wake/secondary line (OBD-II pin 15).
//   Rail:   pin 49/PB5 PS_J1850_9141 — the 'J1850 9141 ON' net is SHARED
//           with J1850 (pack conflicts C7). Owned via rail9141 (refcounted;
//           INV-PWR49: pin 49 LOW only when BOTH J1850 and K-line quiesced).
//
// TIMING SOURCES (all INFERRED until a K-line vehicle test):
//   ISO 9141-2 slow init: 5 baud (200 ms/bit), address 0x33, sync 0x55,
//     keyword bytes KB1/KB2, tester sends ~KB2, ECU sends ~0x33. Idle
//     Tidle >= 300 ms before init. (ISO 9141-2 standard values.)
//   ISO 14230 fast init: 25 ms LOW / 25 ms HIGH wake-up (Twup), then
//     StartCommunication [C1 33 F1 81 CS] at 10400 baud. (ISO 14230-2.)
//   KWP2000 timing P1..P4 defaults: P1 0-20 ms, P2 25-50 ms, P3 55-5000 ms,
//     P4 5-20 ms (ISO 14230-2 defaults).
//   TJA1027: go-to-normal 2-10 us, normal-mode init 7-20 us, go-to-sleep
//     2-10 us (pack constraints tja1027-specs, VERIFIED from datasheet).
//   Checksum (9141 and KWP2000): 8-bit sum of all bytes mod 256.
//
// SAFETY: init sequences TRANSMIT on the bus by construction (5-baud 0x33
// byte, fast-init wake + StartCommunication). They may run only under the
// owning mode's transmit authority — same gate as ELM OBD commands
// (see docs/SAFETY_REVIEW.md, Phase 7 section). The driver itself enforces
// nothing beyond acquire/release; the caller (ELM mode / console / mm)
// owns the gate.
//
// Evidence: VERIFIED = compiles + pack pin citations. INFERRED = every
// timing, the SLP_N polarity, the 5-baud GPIO/UART mux switchback, echo
// behavior, and all ECU interactions.

#include <Arduino.h>

namespace kline {

// ---- pins (VERIFIED: variant.h; pack source/m2-pins.yaml) ----
constexpr uint8_t K_TX = 55;   // PA11 TXD0 — Serial1 TX
constexpr uint8_t K_RX = 56;   // PA10 RXD0 — Serial1 RX
constexpr uint8_t K_SLP = 57;  // PB4 transceiver sleep (SLP_N, active-LOW INFERRED)
constexpr uint8_t L_TX = 58;   // PA13 TXD1 — Serial2 TX
constexpr uint8_t L_RX = 59;   // PA12 RXD1 — Serial2 RX
constexpr uint8_t L_SLP = 60;  // PB7 transceiver sleep (SLP_N, active-LOW INFERRED)

constexpr uint32_t KLINE_BAUD = 10400;

// Init flavors. NONE = acquired but no init run yet.
enum class Init : uint8_t { NONE = 0, SLOW_5BAUD = 1, FAST = 2 };

// KWP2000 timing parameters P1..P4 in ms (ISO 14230-2 defaults, INFERRED).
struct Timing {
    uint16_t p1Min = 0, p1Max = 20;      // ECU inter-byte time
    uint16_t p2Min = 25, p2Max = 50;      // ECU response time
    uint16_t p3Min = 55, p3Max = 5000;    // tester: min gap between requests
    uint16_t p4Min = 5, p4Max = 20;       // tester inter-byte time
};

struct Status {
    bool acquired = false;
    Init initDone = Init::NONE;
    uint8_t kb1 = 0, kb2 = 0;   // keyword bytes from the last init
    uint8_t target = 0x33;      // ECU target address in use
    uint8_t tester = 0xF1;      // tester address in use
    Timing timing;
};

// ---- resource management (acquire-on-entry / release-on-exit) ----
// acquire(): rail via rail9141 (shared with J1850), transceivers to normal
// (SLP HIGH — INFERRED polarity), Serial1/Serial2 at 10400. Returns false
// if already acquired. Touches no bus traffic.
bool acquire();
// release(): tester-present stop, UARTs end, transceivers to sleep (SLP
// LOW — INFERRED), rail released (stays up if J1850 still holds it).
// Idempotent — safe for quiesceBuses().
void release();
bool acquired();

// ---- primitives ----
uint8_t checksum(const uint8_t* d, uint8_t n);  // 8-bit sum mod 256

// sendOnly: TX bytes + echo verification, no response expected
// (used for tester-present 0x3E/0x00 keep-alive).
bool sendOnly(const uint8_t* tx, uint8_t txLen);

// transact: TX req (header+data, checksum appended here), echo verified,
// then wait for one response frame; response checksum verified and
// stripped into rx[]. timeoutMs = 0 selects the P2-based default.
// The response passes busfilter::accept(KLINE/KLINE/srcAddr) when the
// filter table is non-empty (Phase 6 pattern); filtered frames are
// dropped+counted and the wait continues.
bool transact(const uint8_t* req, uint8_t reqLen,
              uint8_t* rx, uint8_t rxCap, uint8_t& rxLen,
              uint16_t timeoutMs = 0);

// ---- init (TRANSMITS on the bus — needs the mode's transmit authority) ----
// 5-baud init: idle 300 ms, bit-bang 0x33 (or targetAddr) at 5 baud on the
// K-line (L-line mirrors it when useL), then 10400-baud keyword handshake.
// retries = full-sequence attempts (INFERRED: 3).
bool initSlow(uint8_t targetAddr = 0x33, uint8_t retries = 3, bool useL = false);
// Fast init: 25 ms LOW / 25 ms HIGH wake, then StartCommunication at
// 10400 baud; parses KB1/KB2 from the positive response.
bool initFast(uint8_t retries = 3);
Init initDone();

// ---- protocol services (require a completed init) ----
// ISO 9141: [hdr0 hdr1 hdr2] + payload + checksum, one response.
// (ELM passes its hdrKline[3], default {0x68,0x6A,0xF1} — INFERRED default.)
bool request9141(const uint8_t hdr[3], const uint8_t* payload, uint8_t payloadLen,
                 uint8_t* rx, uint8_t rxCap, uint8_t& rxLen,
                 uint16_t timeoutMs = 0);
// KWP2000 generic service: [fmt|len tgt src sid data... cs].
bool kwpService(uint8_t sid, const uint8_t* data, uint8_t dataLen,
                uint8_t* rx, uint8_t rxCap, uint8_t& rxLen,
                uint16_t timeoutMs = 0);
// KWP2000 block transfer: 0x23 ReadMemoryByAddress (INFERRED availability).
bool readMemoryByAddress(uint32_t addr, uint8_t len,
                         uint8_t* out, uint8_t& outLen);
// KWP2000 0x3E TesterPresent (response-suppressed keep-alive).
bool testerPresent();

// ---- configuration ----
void setTiming(const Timing& t);
void getTiming(Timing& t);
void setAddrs(uint8_t target, uint8_t tester);  // Phase 9/12 seam: matches
                                                // the filter model's addr slot
void getStatus(Status& s);

// ---- tester-present keep-alive (Phase 6 txsched integration) ----
// Callback-only scheduler slot: sends 0x3E every periodMs while acquired.
// 0 stops. Removed automatically by release().
void testerPresentStart(uint32_t periodMs);
void testerPresentStop();

// ---- monitor support (ELM ATMA) ----
// pollRx(): drain Serial1 into the frame queue (call from the mode's loop);
// frames are delimited by inter-byte gap > 2*P1max (INFERRED). pollFrame()
// drains one queued frame (filter table honored).
void pollRx();
bool pollFrame(uint8_t* buf, uint8_t cap, uint8_t& len);
uint8_t queueDepth();

// ---- Phase 12 auto-detect hook (STUB) ----
// Reserved entry point for the Phase 12 auto-detection engine.
// NOT implemented — returns false. Do not call expecting a probe.
bool probe();

} // namespace kline
