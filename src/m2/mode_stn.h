#pragma once
// M2 multi-mode firmware — STN mode (Phase 4).
//
// STN11xx-compatible command subset, implemented as an EXTENSION of the
// ELM core (mode_elm.*): stn::begin() enables ST-command parsing on the
// shared ELM engine; every AT command keeps working (genuine STN chips are
// ELM327 supersets — OBDLink FRPM: "fully compatible with the ELM327 AT
// command set"). No parser is duplicated.
//
// Subset (FRPM-verified; full list + honest boundary in docs/PHASE4_NOTES.md):
//   STI / STDI          device identity (STN1170-compatible string)
//   STPX h: d: l: t: r: x: f:   custom CAN transmit + raw-frame RX
//     (x: = extended/target address, wired to ISO-TP in Phase 6)
//   STM [n] / STMA [n]  monitor (filters as-set / all), optional count
//   STMR hh / STMT hh   monitor with ID filter (STN-spelled ATMR/ATMT)
//   STCMM 0|1|2         CAN monitoring mode (0 = silent, no ACKs)
//   STCFCPA / STCFCPC   flow-control address pairs
//   STCAF / STCSEGR / STCSEGT / STCSTM / STCTOR  (accepted, stored)
// '?' for: STSL*, STCS (standalone), STWCF, STDP/STDPN — not FRPM commands.
//
// Transport: Serial (Bee UART @115200, immutable) — the SuperB bridges app
// traffic exactly like ELM mode. The mm control channel (F1 7E) is diverted
// in the shared byte pump.
//
// Resource claims: same as ELM — CAN0 (lazy, CAN protocols) or the J1850
// block (lazy, protocols 1/2) + UART0/Bee (never released). exit() releases
// before the manager's quiesceBuses().
//
// Evidence: VERIFIED = compiles / FRPM-cited. INFERRED = untested on HW.

#include <Arduino.h>

namespace stn {

void begin();   // enable ST parsing on the ELM core; enter mode
void loop();    // shared ELM pump
void exit();    // release bus resources (idempotent)

} // namespace stn
