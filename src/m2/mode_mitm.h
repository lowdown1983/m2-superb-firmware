#pragma once
// M2 multi-mode firmware — MITM (man-in-the-middle) mode (Phase 4).
//
// Dual-CAN bridge CAN0<->CAN1 with an ordered rule engine:
//
//   Rule = { id/mask match, direction (A->B / B->A / both), action }.
//   Actions: PASS, DROP, MODIFY (byte replace), COUNT (pass + count),
//            LOG (pass + emit a copy on the Bee link for the web UI).
//   First matching rule wins; anything unmatched hits the compiled-in
//   default: pass-all-both-directions.
//
// MODIFY semantics (exact): data[modOff] = (data[modOff] & ~modMask)
//   | (modVal & modMask). If modOff >= frame length the modification is
//   skipped, the frame is forwarded unmodified, and the modified counter
//   is NOT incremented (the rule hit still counts).
//
// Loop suppression: a frame identical (id+length+data) to one forwarded in
// the opposite direction within loopWinMs (default 5 ms, INFERRED) is
// dropped, not re-forwarded. Without this, bridging two taps of the SAME
// physical bus ping-pongs every frame forever. Trade-off (documented): two
// genuinely different ECUs emitting identical frames within the window on
// opposite buses will also suppress one copy.
//
// Stealth (honest definition): the bridge NEVER originates CAN frames
// locally — it only forwards what it received. CAN ACK is physical-layer
// (the transceiver/controller ACKs in hardware); no firmware can suppress
// it short of listen-only mode, which a forwarding bridge cannot use.
// The stealth flag is stored and reported; in Phase 4 it has no additional
// behavioral effect (it gates future active-origin features).
//
// Rate limiter: optional per-rule frames/sec cap (sliding 1 s window);
// excess frames are dropped. Tuning is INFERRED.
//
// Bus load: frames/sec per bus, converted with a nominal ~128 bits/frame
// (INFERRED estimate, not a measurement).
//
// Console (SerialUSB): "mitm ..." — rule add/del/list, stats, clear,
// stealth on|off, dir a2b|b2a|both. Config works even when MITM is not
// the active mode (staged, like UDS).
//
// Resource claims: CAN0 + CAN1 (both begun at entry, independent bauds) +
// UART0/Bee (never released). quiesceBuses() already handles both native
// controllers (per-controller begun flags) — no manager change needed.
// No SWCAN/J1850 here (TC7/SWCAN-SOF collision is a separate design).
//
// Evidence: VERIFIED = compiles. INFERRED = untested on hardware.

#include <Arduino.h>

namespace mitm {

void begin();   // acquire CAN0+CAN1, promiscuous filters, reset stats
void loop();    // pump both buses + Serial mm-divert
void exit();    // release both CAN controllers (idempotent)

// USB-console entry. `line` is the text after "mitm".
void console(char* line);

// ---- mm-channel ops (manager delegates; see mm_channel.h) ----
// p = 15 bytes: [id LE32][mask LE32][dir][action][modOff][modMask][modVal]
//               [rateLim LE16]. idx = new table index. ST_ERR if full.
uint8_t mmRuleAdd(const uint8_t* p, uint8_t& idx);
// idx = 0xFF clears all. nRules = remaining count.
uint8_t mmRuleDel(uint8_t idx, uint8_t& nRules);
// start = first index; entries packed 20 bytes each into out
// ([idx][id LE32][mask LE32][dir][action][modOff][modMask][modVal]
//  [rateLim LE16][hits LE32]); nRet <= 2 per the 48-byte frame cap.
uint8_t mmRuleList(uint8_t start, uint8_t* out,
                   uint8_t& nTotal, uint8_t& nRet);
// out = 37-byte stats blob: [fwdAB LE32][fwdBA LE32][dropAB LE32]
// [dropBA LE32][modAB LE32][modBA LE32][loopSupp LE32][rateDrop LE32]
// [loadA][loadB][dir][stealth][nRules].
void mmStats(uint8_t* out);
// p = 14 bytes: [baudA LE32][baudB LE32][dir][stealth][loopWinMs LE16]
//               [defRateLim LE16]. applied=1 when the buses were
//               re-inited live (MITM active), 0 when staged.
uint8_t mmCfg(const uint8_t* p, uint8_t& applied);
// out = 15-byte config blob: [baudA LE32][baudB LE32][dir][stealth]
// [loopWinMs LE16][defRateLim LE16][active] (CMD_MITM_CFG read path).
void mmCfgGet(uint8_t* out);

// Phase 5 (PROG safety): suspend the rule table on PROG entry. A bridge
// that modifies flash payloads is a brick vector (docs/SAFETY_REVIEW.md
// §2), so entering PROG clears the table with an explicit notice.
// Returns the number of rules cleared. There is NO restore path —
// re-adding rules afterwards is a deliberate user action.
uint8_t suspendRules();

} // namespace mitm
