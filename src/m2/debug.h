#pragma once
// Unified bus-level debug logging for all M2 modes (2026-10-08).
//
// One flag, one console command (`debug on` / `debug off`), usable from every
// mode. When enabled, modes log their bus activity to the USB console:
//   - ELM/STN: AT commands in, protocol detect steps, OBD TX/RX, NO DATA
//   - M2RET:   mode entry/exit, bus error counters
//   - UDS:     service TX, response RX, ISO-TP events
//   - OBD2:    poller activity (PID, response/no-response)
//   - MITM:    rule matches (stats; per-frame is too verbose by default)
//   - PROG:    programming steps (already verbose; gated here too)
//
// Use: plug USB into a laptop, `pio device monitor -e m2`, `debug on`.
// The flag is runtime-toggled; no reflash needed to enable/disable.

#include <Arduino.h>

namespace dbg {

extern bool enabled;  // set via `debug on` / `debug off` console command

inline void print(const __FlashStringHelper* msg) {
    if (enabled) SerialUSB.print(msg);
}
inline void println(const __FlashStringHelper* msg) {
    if (enabled) SerialUSB.println(msg);
}
inline void print(const char* msg) {
    if (enabled) SerialUSB.print(msg);
}
inline void println(const char* msg) {
    if (enabled) SerialUSB.println(msg);
}
inline void print(uint32_t v, int base = DEC) {
    if (enabled) SerialUSB.print(v, base);
}
inline void println(uint32_t v, int base = DEC) {
    if (enabled) SerialUSB.println(v, base);
}

// Hex dump helper: "TX 7DF: 02 01 00"
inline void hexBytes(const __FlashStringHelper* tag, uint32_t id,
                     const uint8_t* d, uint8_t n) {
    if (!enabled) return;
    SerialUSB.print(tag);
    SerialUSB.print(F(" "));
    SerialUSB.print(id, HEX);
    SerialUSB.print(F(":"));
    for (uint8_t i = 0; i < n; i++) {
        SerialUSB.print(F(" "));
        if (d[i] < 0x10) SerialUSB.print(F("0"));
        SerialUSB.print(d[i], HEX);
    }
    SerialUSB.println();
}

} // namespace dbg
