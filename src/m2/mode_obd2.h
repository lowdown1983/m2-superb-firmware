#pragma once
// M2 multi-mode firmware — OBD2 poller mode (Phase 3).
//
// Standalone SAE J1979 PID poller: no phone/app needed. Polls a configurable
// PID list over CAN ISO-TP (functional 0x7DF) and emits CSV lines on the
// Bee link (Serial @115200, immutable):
//     ts_ms,svc,pid,value
// e.g.  123456,01,0C,850 rpm  (RPM)
//       123457,01,42,12.400 V (control-module voltage)
// `value` is the registry-decoded quantity (Phase 9: pidreg), or raw hex
// when the (service,PID) has no decoder. The SuperB bridges this text
// stream to TCP:23/BT and keeps recent lines for the web GUI live view.
//
// Protocol: CAN 11-bit, auto-detect 500k then 250k (INFERRED order), retry
// every 5 s until an ECU answers 0100. PID formulas per SAE J1979 (public
// standard); DTC text per SAE J2012.
//
// Console (SerialUSB): "obd2 ..." — config works any time (staged);
// bus-touching commands need this mode active.
//   obd2 start | obd2 stop
//   obd2 interval <ms>        sweep interval, default 1000 (100..60000)
//   obd2 pids 010C,010D,05    PID list (hex, comma-separated, max 16)
//   obd2 bus 0|1 | obd2 baud <n>
//   obd2 dtc                  mode 03 DTC scan on demand (decoded P0xxx)
//   obd2 status
//
// Resource claims: the configured CAN bus (lazy acquire on start/DTC) +
// UART0/Bee (never released). exit() stops the poller and releases the bus.
//
// Evidence: VERIFIED = compiles / SAE J1979 + J2012 public formulas.
// INFERRED = untested against real ECUs (no bench).

#include <Arduino.h>

namespace obd2 {

void begin();   // reset poll state; protocol unlocked, poller stopped
void loop();    // detect + sweep + Serial pump (mm divert)
void exit();    // stop poller, release bus (idempotent)

// USB-console entry. `line` is the text after "obd2".
void console(char* line);

// mm-channel ops (manager delegates; see mm_channel.h).
// mmCfg stages config (stops a running poller — safe). pids[] are raw
// PID bytes, nPids 1..16.
uint8_t mmCfg(uint8_t bus, uint32_t baud, uint16_t intervalMs,
              const uint8_t* pids, uint8_t nPids);
uint8_t mmCtl(uint8_t start, uint8_t& runningOut);
void mmStatus(uint8_t& runningOut, uint8_t& busOut,
              uint16_t& intervalMsOut, uint8_t& nPidsOut);

// Phase 9: run the discovery engine (stops the poller, releases the bus
// first). For CMD_DISCOVER_START dispatch.
uint8_t mmDiscover(uint8_t bus);

} // namespace obd2
