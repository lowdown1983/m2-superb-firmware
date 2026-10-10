#pragma once
// M2RET mode — GVRET binary protocol over the Bee UART (Serial @115200).
//
// PRIMARY SOURCE: M2RET by Collin Kidder, commit ad9d5af67d8e7c3330c0281e0403c2750e023f1c.
//   - Protocol framing/commands: M2RET.ino loop() state machine (~lines 820-1280),
//     CommProtocol.txt, M2RET.h enum GVRET_PROTOCOL.
//   - Frame TX format: M2RET.ino sendFrameToUSB() binary branch (lines 540-560).
//   - Promiscuous filters: M2RET.ino setPromiscuousMode() (lines 398-412).
//   - SWCAN ISR: M2RET.ino CANHandler() (lines 115-117) -> SWCAN.intHandler().
//
// DELIBERATE DEVIATIONS from M2RET (documented, Phase 1 simplifications):
//   - Transport is Serial (Bee UART to the SuperB), not SerialUSB. M2RET speaks
//     GVRET on SerialUSB; here SerialUSB is the human control console (mode
//     commands) and the Bee link is the pure GVRET data plane.
//   - No EEPROM settings persistence (SAM3X has no EEPROM; M2RET uses the
//     24LC32A via due_wire). All settings live in RAM and reset on reboot.
//   - No Lawicel ASCII mode, no text config console, no SD logging, no
//     digital-toggle system, no 12VIO/ADC subsystems.
//   - PROTO_DIG_INPUTS / PROTO_ANA_INPUTS reply zeros (no input subsystem).
//   - PROTO_SET_DIG_OUT is accepted and ignored (no output subsystem).
//   - PROTO_GET_DEV_INFO reports build 1 and zeros for EEPROM/file fields.
//
// Resource claims (ownership rule): CAN0, CAN1, SWCAN (SPI0 CS3 + PIOC16
// callback slot), UART0/Serial. This mode MUST NOT touch TC6/TC7, PWM6, or
// any other collision-table resource.

#include <Arduino.h>

namespace m2ret {

// Lifecycle (called by the mode manager).
void begin(bool canSafeToDisable);   // reset GVRET parser state; the flag records
                                     // whether due_can begin(...,255) has run, so
                                     // applyCanBusConfig() never calls disable()
                                     // before the transceiver-pin hazard is cleared
void loop();    // pump Serial bytes + poll CAN buses

// Acquire/release helpers for the SWCAN half of this mode (Phase 8: all
// routed through the hardened swcan:: driver — TH8056 state machine,
// datasheet settling, MCP2515 health monitoring).
void swcanEnter();  // swcan::acquire(speed): MCP2515 init + NORMAL + INT
void swcanExit();   // swcan::release(): INT detached + TH8056 sleep
void swcanSleep();  // quiesce-path sleep via swcan::release(); safe even if never initialized
void swcanSetSpeed(uint32_t speed);  // live speed change (mm channel)

// mm channel live-apply sync: keeps GVRET cmd-5/6 replies consistent when
// the manager re-bauds CAN0/CAN1 outside the GVRET path.
void setCanSpeeds(uint32_t c0, uint32_t c1);

// Open RX filters on all 7 mailboxes of CAN0/CAN1.
// VERIFIED: M2RET.ino:398-412 (filters 0-2 extended, 3-6 standard, id 0/mask 0).
void setPromiscuousMode();

// Phase 12 modernization console: text commands on the USB console when
// M2RET mode is active ("m2ret CAPABILITIES", ...). `line` is the text
// after "m2ret ".
void console(char* line);

} // namespace m2ret
