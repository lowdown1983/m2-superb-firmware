#include <Arduino.h>
#include <SPI.h>
#include "mode_manager.h"

// M2 multi-mode firmware — Phase 1.
//
// RESOURCE OWNERSHIP: peripheral claims live in the mode manager and the
// per-mode files (see docs/RESOURCE_OWNERSHIP.md). main() claims nothing
// itself beyond the two serial ports.
// HARDWARE RULE (verified, immutable): the M2<->SuperB Bee link is `Serial`
// (XBEE_RX=PA8/Arduino 0, XBEE_TX=PA9/Arduino 1) at 115200 bps. This rate is
// NOT configurable — do not parameterize it.

static modes::ModeManager modeManager;

void setup()
{
    // Bee UART to the SuperB. 115200 is immutable — see comment above.
    // (No wait-for-serial: on the SAM3X, Serial is a hardware UART that is
    // always active once begun.)
    Serial.begin(115200);

    // USB debug / control console (mode commands). The GVRET data plane
    // lives on Serial (Bee); the human console lives here.
    SerialUSB.begin(115200);

    // SPI for the SWCAN MCP2515 (TH8056). The MCP2515 library's
    // SPI.transfer() blocks forever on an uninitialized peripheral, so this
    // MUST run before modeManager.begin() -> swcanEnter() -> setupSW().
    // (The library sets clock/mode per-transaction; begin() enables the HW.)
    SPI.begin();

    // Audit fix F-1 (2026-10-08): the SAM3X ADC defaults to 10-bit, but
    // mmGetVoltageCb() scales V_SENSE for 12-bit (4.469 mV/LSB). Without
    // this, a 12.6 V battery reads ~3.1 V and the SuperB sleep/wake manager
    // permanently misreads engine state. analogRead()'s only call site is
    // the voltage callback, so 12-bit globally is safe.
    analogReadResolution(12);

    // Give the native-USB host time to enumerate before anything prints:
    // on reset the port drops and re-enumerates (~1 s), and prints issued
    // before that are lost. A fixed pause only delays boot; unlike
    // while(!SerialUSB) it can never hang on OBD2 power with no USB host.
    delay(1500);

    modeManager.begin();
}

void loop()
{
    modeManager.loop();
}
