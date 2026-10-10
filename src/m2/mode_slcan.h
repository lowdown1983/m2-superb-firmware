#pragma once
// M2 multi-mode firmware — SLCAN / Lawicel ASCII transport (Phase 12).
//
// ASCII serial CAN protocol for apps that speak Lawicel (CAN232/CANUSB
// style tools). Runs on the Bee UART (@115200) like the other host modes.
//
// Supported commands (Lawicel subset):
//   O      open CAN channel (staged speed, default 500k)
//   C      close channel
//   S0..S8 set CAN bitrate (0=10k 1=20k 2=50k 3=100k 4=125k 5=250k
//          6=500k 7=800k 8=1M)
//   tIIILD[DD..]  transmit standard frame (III=3 hex ID, L=len, D=data)
//   TIIIIIIIILDD.. transmit extended frame (8 hex ID)
//   rIIIL / RIIIIIIIIL  transmit RTR frame
//   F      status flags -> F00
//   V      version -> V1013
//   N      serial number -> N2208
//   Z0/Z1  timestamps off/on (2-byte ms timestamp appended)
//   Miiiiiii / mIIIIIIII  acceptance mask/code (stored; applied to RX
//          filter via the shared busfilter idMatch when set)
//   P      poll: one pending frame | A: poll all pending
//   <cr>   every command and reply is CR-terminated (Lawicel)
//
// Received CAN frames are emitted as t/T ASCII lines (Z1 appends the
// timestamp). TX honors the Phase 9 capgate (ALLOW_TRANSMIT).
//
// Evidence: VERIFIED = compiles. INFERRED = interop with real SLCAN
// apps untested (no bench).

#include <Arduino.h>

namespace slcan {

void begin();   // acquire CAN0 @staged speed (begin(baud,255))
void loop();    // pump Bee UART commands + CAN0 RX
void exit();    // release CAN0

// USB-console entry (the Bee link carries the SLCAN protocol itself;
// the USB console mirrors a few helpers). `line` is text after "slcan".
void console(char* line);

} // namespace slcan
