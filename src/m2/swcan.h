#pragma once
// M2 multi-mode firmware — SWCAN (single-wire CAN) hardened driver (Phase 8).
//
// Wraps the vendored Macchina SWcan lib (TH8056 transceiver + MCP2515
// controller) with an explicit TH8056 state machine, datasheet
// mode-transition timings, MCP2515 health monitoring + recovery, bus-off
// detection, hardware filter support, and Phase 6 transport wiring
// (busfilter gates, busstat counters, dbg:: events).
//
// HARDWARE TRUTH (knowledge pack v1.4.0 — VERIFIED, never the banned specs):
//   TH8056 U11: pin 64/SWC_M0 -> MODE0, pin 65/SWC_M1 -> MODE1
//     (S-SCH-I sheet 16; constraints.yaml tja1027-specs... swcan-modes).
//   Mode table (TH8056 datasheet §3.2 Fig.5, p.15): 00 sleep, 10 high-speed,
//     01 high-voltage wake-up, 11 normal (33.333 kbit/s). Weak internal
//     pull-downs -> undriven pins default to sleep.
//   Transition timings (TH8056 datasheet §2.5 AC table): Normal <->
//     High-speed/HVWU <= 30 ms; Normal -> Standby <= 500 us; Standby ->
//     Sleep 100-500 ms; Sleep -> Normal <= 50 ms; wake-up filter tWUF
//     10-70 us; TxD dominant clamp 10-30 ms.
//   MCP2515: SPI0, CS = pin 63 (SPI0_CS3/PB23), INT = pin 67 (PC16/SWC_INT).
//     Pin 67 is configured INPUT by the MCP2515 constructor (VERIFIED in
//     MCP2515.cpp) — satisfying the pack rule that SWC_INT must be INPUT
//     with PIO interrupt, not the variant default OUTPUT.
//   SWC_M0's alternate function is TC0_CHB0 (pack conflict) — firmware never
//     configures TC0 capture while SWCAN owns M0. SWC_SOF (pin 68, TC7_B)
//     capture is NOT used: TC7 is mutually exclusive with J1850_PWM_RX
//     (TC7_A) per the timer_rule (conflicts.yaml INV-TC7); frame timestamps
//     below are drain-time micros() (INFERRED precision) until a TC7 owner
//     is designated.
//
// RESOURCE CLAIMS (ownership rule): SPI0 (sole onboard claimant per
// INV-SPI0 — NPCS3; no other SPI0 client exists in firmware), the PIOC16
// callback slot (single slot per pack — the ISR trampoline), SWC_M0/SWC_M1
// GPIOs. acquire/release discipline; release() is idempotent (safe for
// quiesceBuses). sleepGpio() is GPIO-only and safe before/after any
// MCP2515 init.
//
// EVERYTHING hardware-touching is INFERRED until tested on a SWCAN vehicle
// (the 2023 Bolt EV is CAN-only; no GM-LAN here).

#include <Arduino.h>
#include <due_can.h>  // CAN_FRAME (same include mode_m2ret.cpp uses)

namespace swcan {

enum class Mode : uint8_t {
    SLEEP = 0,
    NORMAL = 1,
    HIGHSPEED = 2,
    HV_WAKEUP = 3,
};

// Acquire the SWCAN block: MCP2515 init at `speed` (16 MHz crystal),
// TH8056 to NORMAL with the Sleep->Normal <= 50 ms settle, INT attached,
// presence tracked. Returns false if already acquired or the MCP2515 does
// not answer the health check.
bool acquire(uint32_t speed);
// Release: INT detached, TH8056 to sleep, presence cleared. Idempotent.
void release();
bool acquired();
Mode mode();
uint32_t speed();

// Set the TH8056 mode with the datasheet settling wait. Legal only while
// acquired. SLEEP->active waits <= 50 ms; active<->active waits <= 30 ms;
// ->SLEEP waits 100 ms (the VERIFIED Standby->Sleep minimum from TH8056
// §2.5; the exact Normal->Sleep mapping is INFERRED).
bool setMode(Mode m);

// GPIO-only sleep: drives M0/M1 LOW without touching SPI and without the
// settle wait. Safe before or after any MCP2515 init (the quiesceBuses()
// path). No consumer depends on sleep completion — the next acquire()
// carries the Sleep->Normal settle — so the immediate return keeps mode
// switches fast. (Full release() does settle.)
void sleepGpio();

// Re-run init at a new speed while acquired (release + acquire). Used for
// live speed changes.
bool reconfigure(uint32_t speed);

// High-voltage wake-up: enter HV_WAKEUP, wait up to timeoutMs for the INT
// line to signal bus activity (the tWUF 10-70 us filter is in hardware;
// we poll the pin), then transition to NORMAL. Returns true on wake.
bool waitWake(uint32_t timeoutMs);

// MCP2515 listen-only (receives, never ACKs). Applied immediately when
// acquired; remembered across recover().
void setListenOnly(bool lo);
bool listenOnly();

// Transmit one frame. False when not acquired or the mailbox refused it.
// Feeds busstat (tx / txFail).
bool sendFrame(CAN_FRAME& frame);

// Drain one received frame; optional drain-time timestamp in *tsUs
// (micros(), INFERRED precision). Applies the busfilter table
// (SWCAN/RX) when non-empty; filtered frames are counted as dropped.
// Feeds busstat (rx / dropped).
bool getFrame(CAN_FRAME& frame, uint32_t* tsUs = nullptr);

// Call from the PIOC16 ISR trampoline (owned by this module).
// NOTE: the trampoline calls the vendored hw.intHandler() directly;
// there is intentionally no swcan::isr() wrapper (a previous revision had
// one — it was dead API and has been removed).

// Health + bus-off poll: call once per main loop while acquired. Reads
// EFLG/TEC; on TXBO counts a bus-off, logs it, and re-requests normal
// mode (the MCP2515 auto-recovers after 128x11 recessive bits per its
// datasheet); on repeated unreadable registers runs recover().
void poll();

// Full MCP2515 recovery: Reset + re-init at the acquired speed + re-apply
// the TH8056 mode and listen-only. Returns true if the controller answers
// afterwards. NOTE: hardware RX filters set via setHwFilter/setHwMask are
// NOT re-applied here — no caller sets them yet; when one does, it must
// re-program them after recover() (and note the vendored SetRXMask drops
// the controller to MODE_CONFIG mid-call, so never call it on a live bus
// without re-requesting the operating mode after).
bool recover();

// Hardware RX filtering on the MCP2515 (RXB0/RXB1). ext=false for 11-bit.
// Masks are programmed separately via setHwMask (the vendored lib keeps
// mask and filter registers independent). (0,0)+permissive mask = accept all.
void setHwFilter(uint8_t filter, uint32_t id, bool ext);
void setHwMask(uint8_t mask, uint32_t maskVal);

// GMLAN quirk helper (VERIFIED M2RET.ino:1052-1067): id 0x100 on SWCAN
// goes out in high-speed mode. hsEnter() transitions to HIGHSPEED (with
// settle); the caller transmits; hsExit() returns to NORMAL. Prefer over
// raw mode() calls so the settling waits are never skipped.
void hsEnter();
void hsExit();

} // namespace swcan
