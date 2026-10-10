#pragma once
// M2 multi-mode firmware — mode manager (Phase 1).
//
// Implements the acquire/release discipline from docs/RESOURCE_OWNERSHIP.md:
//   - entering a mode acquires its resources BEFORE enabling them;
//   - a mode switch ALWAYS quiesces the buses first (native CAN controllers
//     to init/off, SWCAN TH8056 to sleep, J1850 power rail off);
//   - exiting releases peripherals, IRQs, GPIO alternate functions, timers,
//     power controls and transports.
//
// Phase 1: only M2RET is implemented. Selecting any other mode prints a
// notice and the manager stays in the current mode.
//
// Phase 3: UDS + OBD2 implemented (mode_uds.h, mode_obd2.h). SuperB-initiated
// switching via the mm control channel (F1 7E, see mm_channel.h) — SET_MODE
// dispatches ONLY through requestMode() (safe-switch path).
//
// Mode selection inputs:
//   - physical Button1 (Arduino 22, active LOW): short press cycles to the
//     next IMPLEMENTED mode; long press (>1s) returns to M2RET.
//   - SerialUSB text console: "mode list", "mode <name>", "status", "help".
//   - mm channel (SuperB web GUI): SET_MODE / GET_STATUS / SET_CAN_BAUD /
//     SET_SWCAN_SPEED / SET_FEATURE.
//
// Evidence labels: VERIFIED = compiles / primary-source fact.
// INFERRED = hardware behavior not testable without a bench.

#include <Arduino.h>

namespace modes {

// VERIFIED (M2RET M2RET.h @ ad9d5af, enum GVRET_PROTOCOL; project scope
// agreed 2026-10-04): the seven planned modes.
enum class Mode : uint8_t {
    M2RET = 0,
    ELM327,
    STN,
    UDS,
    MITM,
    OBD2,
    PROG,
    GATEWAY,  // Phase 10: CAN0 <-> SWCAN protocol translation
    SLCAN,    // Phase 12: Lawicel ASCII transport (CAN0, Bee UART)
    HARNESS,  // Phase 12: virtual ECU self-test (CAN0<->CAN1 loopback)
    COUNT
};

const char* modeName(Mode m);
bool modeFromName(const char* name, Mode& out);
// Which modes have an implementation behind them (Phase 5: all seven).
bool modeImplemented(Mode m);
// Phase 12: global accessor for modules that need staged settings
// (profiles). Set in ModeManager::begin(); null before that.
class ModeManager;
ModeManager* manager();

class ModeManager {
public:
    void begin();          // sets up button GPIO, quiesces buses, enters M2RET
    void loop();           // pumps button, SerialUSB console, current mode loop
    bool requestMode(Mode target);  // safe switch; false if refused
    Mode current() const { return currentMode; }

    // mm control-channel ops (public for the C-style ops table; see
    // mm_channel.h). Applied-live vs staged is reported via `applied`.
    void mmGetStatus(uint8_t& mode, uint32_t& c0, uint32_t& c1,
                     uint32_t& sw, uint8_t& feat);
    uint8_t mmSetCanBaud(uint8_t bus, uint32_t baud, uint8_t& applied);
    uint8_t mmSetSwcanSpeed(uint32_t speed, uint8_t& applied);
    uint8_t mmSetFeature(uint8_t feat, uint8_t enable);

private:
    Mode currentMode = Mode::ELM327;  // 2026-10-08: ELM327 is the default boot
                                     // mode (was M2RET); auto-detects protocol

    void enterM2RET();
    void exitM2RET();
    void enterElm();
    void exitElm();
    void enterStn();
    void exitStn();
    void enterUds();
    void exitUds();
    void enterMitm();
    void exitMitm();
    void enterObd2();
    void exitObd2();
    void enterProg();
    void exitProg();
    void enterGateway();
    void exitGateway();
    void enterSlcan();    // Phase 12
    void exitSlcan();
    void enterHarness();  // Phase 12
    void exitHarness();

public:
    // Phase 12: explicit bus-takeover support for auto-detect (public:
    // called from the console/mm static callbacks). quiesceBuses is
    // idempotent; reacquire re-enters the current mode (grant resets,
    // same as a normal switch).
    void reacquire();
    void quiesceBuses();

private:
    void switchTo(Mode target);  // Phase 12: shared switch body

    // Bus settings (mm channel / web GUI). Applied live where safe.
    uint32_t can0BaudSet = 500000;   // VERIFIED M2RET default (M2RET.ino:131)
    uint32_t can1BaudSet = 500000;   // VERIFIED M2RET default (M2RET.ino:133)
    uint32_t swcanSpeedSet = 33333;  // VERIFIED M2RET default (M2RET.ino:139)
    uint8_t features = 0x03;         // FEAT_J1850 | FEAT_SWCAN (mm_channel.h)

    void pollButton();
    void pollSerialConsole();
    void handleConsoleLine(char* line);
    void printStatus();

    // Button state (Button1 = Arduino 22, active LOW).
    // INFERRED: INPUT_PULLUP is used so the pin is well-defined even if the
    // board's external pull-up were absent; Macchina's own passthrough sketch
    // uses plain INPUT and treats LOW as pressed, which implies a pull-up
    // exists on the board — INPUT_PULLUP is compatible with that.
    static constexpr uint8_t BUTTON_PIN = 22;  // VERIFIED: variant.h
    uint32_t buttonDownAt = 0;
    bool buttonWasDown = false;
    uint32_t lastDebounceAt = 0;
};

// ELM-mode bus primitives (Phase 2). begin(baud,255) avoids the
// transceiver-enable pin hazard (see mode_manager.cpp header); the per-bus
// begun flags track pin-safety for disable(). Phase 3: UDS/OBD2 also use
// CAN1, so the flags are per-controller — disable() is only ever called on
// a controller whose begin(...,255) already ran.
bool canBegin0(uint32_t baud);
void canEnd0();
bool canBegin1(uint32_t baud);
void canEnd1();

} // namespace modes
