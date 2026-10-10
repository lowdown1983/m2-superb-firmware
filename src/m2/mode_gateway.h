#pragma once
// M2 multi-mode firmware — GATEWAY mode (Phase 10): CAN0 <-> SWCAN.
//
// A SEPARATE mode, not a MITM extension (docs/ROADMAP_PHASES_6_PLUS.md
// Appendix A): MITM is transparent interception on one protocol (never
// originates); a gateway crosses bitrates (500k vs 33.33k ~15:1),
// controllers (SAM3X native vs MCP2515/SPI), and frame semantics (GM-LAN),
// so every crossing needs ID remapping, buffering with an explicit
// overflow policy, and its own safety case. The MITM ordered rule engine
// is untouched — the gateway has its own mapping-table data model.
//
// Mapping = { srcBus/srcId/srcMask/srcExt -> dstBus/dstId/dstExt,
//             action (PASS | MODIFY byte-replace), drop policy }.
// First matching mapping wins. Tables are data-driven, versioned
// (tableVersion bumps on every edit/suspend), and live in RAM
// (EEPROM persistence is Phase 12).
//
// Rate adaptation: per-direction queues (CAN0->SWCAN 16, SWCAN->CAN0 8).
// Overflow drops by explicit policy (drop-oldest default / drop-newest,
// global default + per-mapping override) with per-mapping loss counters
// AND busstat::noteDropped — never silently.
//
// Safety (docs/SAFETY_REVIEW.md §12): a gateway ORIGINATES frames by
// construction — MITM's "never originate" stealth property does NOT
// transfer. Minimum authority ALLOW_TRANSMIT (mode default); MODIFY
// mappings (payload fabrication) need ALLOW_DIAGNOSTIC_WRITE +
// arm-confirm (repeat-identical within 10 s, same mechanism as UDS
// confirmGate). PROG entry auto-suspends (clears the table, never
// silently restored — re-adding mappings is the explicit re-enable).
//
// Resource claims: CAN0 (begun at entry, staged baud) + SWCAN via
// swcan:: (acquired at entry, staged speed) + UART0/Bee (never
// released). quiesceBuses() handles both (native via begun flags,
// TH8056 via sleepGpio) — no manager change needed beyond the
// standard enter/exit pair.
//
// Console (SerialUSB): "gateway ..." — status, map add/del/list,
// cfg, enable. Config works any time; forwarding runs only in
// GATEWAY mode.
//
// Evidence: VERIFIED = compiles. INFERRED = untested on hardware
// (no GM-LAN vehicle; the Bolt EV is CAN-only).

#include <Arduino.h>

namespace gateway {

// Buses in mapping-table encoding.
enum : uint8_t { BUS_CAN0 = 0, BUS_SWCAN = 1 };
// Mapping actions.
enum : uint8_t { ACT_PASS = 0, ACT_MODIFY = 1 };
// Overflow policies (per-mapping; POL_GLOBAL = use the global default).
enum : uint8_t { POL_DROP_OLDEST = 0, POL_DROP_NEWEST = 1, POL_GLOBAL = 2 };

constexpr uint8_t MAX_MAPS = 8;
constexpr uint8_t Q_CS = 16;  // CAN0 -> SWCAN queue depth
constexpr uint8_t Q_SC = 8;   // SWCAN -> CAN0 queue depth

void begin();   // acquire CAN0 + SWCAN, reset stats (mappings persist: config)
void loop();    // pump both buses, translate, drain TX queues
void exit();    // release both buses, drain-or-drop queues (documented)

// USB-console entry. `line` is the text after "gateway".
void console(char* line);

// ---- mm-channel ops (manager delegates; see mm_channel.h) ----
// p = 21 bytes: [srcBus][srcId LE32][srcMask LE32][srcExt]
//               [dstBus][dstId LE32][dstExt]
//               [action][modOff][modMask][modVal][dropPolicy]
// idx = new table index. ST_ERR if full / bad args / grant denied.
// MODIFY mappings additionally need arm-confirm: the first call stages
// and returns ST_OK with idx=0xFE (confirm pending); the IDENTICAL call
// within 10 s executes. (Same mechanism as UDS confirmGate.)
uint8_t mmMapAdd(const uint8_t* p, uint8_t& idx);
// idx = 0xFF clears all. nMaps = remaining count.
uint8_t mmMapDel(uint8_t idx, uint8_t& nMaps);
// start = first index; entries packed 22 bytes each into out:
// [idx][srcBus][srcId LE32][srcMask LE32][srcExt][dstBus][dstId LE32]
// [dstExt][action][modOff][modMask][modVal][dropPolicy]. nRet <= 1 per
// the 48-byte reply cap (6-byte header + 22). version = table version.
uint8_t mmMapList(uint8_t start, uint8_t* out,
                  uint8_t& nTotal, uint8_t& nRet, uint16_t& version);
// out = 29-byte stats blob: [fwdCS LE32][fwdSC LE32][dropCS LE32]
// [dropSC LE32][modCS LE32][modSC LE32][version LE16][nMaps]
// [suspended][active].
void mmStats(uint8_t* out);
// p = 10 bytes: [canBaud LE32][swcanSpeed LE32][dropPolicy][drainPerLoop].
// applied=1 when the buses were re-inited live (gateway active),
// 0 when staged.
uint8_t mmCfg(const uint8_t* p, uint8_t& applied);
// out = 12-byte config blob: [canBaud LE32][swcanSpeed LE32]
// [dropPolicy][drainPerLoop][active][suspended] (CMD_GW_CFG read path).
void mmCfgGet(uint8_t* out);

// PROG-entry auto-suspend (docs/SAFETY_REVIEW.md §2 brick vector): clears
// the mapping table with an explicit notice. Returns the number of
// mappings cleared. There is NO silent restore — re-adding mappings
// (console or mm) is the explicit re-enable and clears the suspended
// flag. Called from ModeManager::enterProg alongside mitm::suspendRules.
uint8_t suspend();

// Explicit re-enable (console `gateway enable`); also cleared by any
// successful map add. Returns true if the flag was set.
bool clearSuspended();
bool suspended();

} // namespace gateway
