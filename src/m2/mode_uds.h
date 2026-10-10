#pragma once
// M2 multi-mode firmware — UDS/diagnostics mode (Phase 3).
//
// Unified Diagnostic Services over CAN ISO-TP (ISO 15765-2 transport via
// isotp.h; service/NRC semantics per ISO 14229-1 — cited, never invented).
// Phase 3 transport is CAN ONLY (11-bit); J1850 diagnostics are out of
// scope for this phase.
//
// Console (SerialUSB): "uds ..." — config commands work any time (staged);
// bus-touching commands require this mode to be active.
//   uds bus 0|1        select CAN bus (default 0)
//   uds baud <n>       CAN baud (default 500000; allow-listed)
//   uds tx <hex>       physical request ID (default 7E0)
//   uds rx <hex>       physical response ID (default 7E8)
//   uds func | phys    functional (0x7DF, all ECUs) vs physical addressing
//   uds tp on|off|<ms> tester-present keepalive (default off, 2000 ms)
//   uds session        show current session + addressing config
//   uds <svcHex> [dataHex...]   raw service request, e.g.:
//     uds 1003          DiagnosticSessionControl / extendedSession
//     uds 22 F190       ReadDataByIdentifier / VIN
//     uds 19 02 FF      ReportDTCByStatusMask / all
//     uds 14000000      ClearDiagnosticInformation (group FFFFFF abbreviated)
//   0x11 (ECUReset) and 0x2E (WriteDataByIdentifier) are ARM-CONFIRM:
//   the first invocation prints a warning and arms for 10 s; repeating the
//   identical command executes. They are never executed automatically.
//   0x27 (SecurityAccess) is a STUB: requestSeed/sendKey are passed through
//   and the seed printed, but NO key algorithm is implemented (OEM
//   algorithms are proprietary — never invent one).
//
// Session state machine: a positive 0x10 response sets the session; without
// tester-present traffic the session drops to default (0x01) after
// SESSION_TIMEOUT_MS (5000 ms, INFERRED per ISO 14229 S3_server).
//
// Resource claims: the configured CAN bus (lazy acquire on first command,
// like ELM) + UART0/Bee (never released). exit() releases before the
// manager's quiesceBuses().
//
// Evidence: VERIFIED = compiles / ISO 14229-1 + ISO 15765-2 public standard
// semantics. INFERRED = untested against real ECUs (no bench).

#include <Arduino.h>

namespace uds {

void begin();   // reset session state; bus acquired lazily
void loop();    // tester-present keepalive + session timeout + Serial pump
void exit();    // release bus (idempotent)

// USB-console entry. `line` is the text after "uds" (may be just "uds").
void console(char* line);

// mm-channel ops (manager delegates; see mm_channel.h).
// mmTx sends one raw service using the current addressing config. Returns
// an mm ST_* code; *nResp = responses collected, first response bytes in
// resp/*respLen (truncated by the caller contract, not here).
uint8_t mmTx(const uint8_t* svc, uint8_t svcLen,
             uint8_t* resp, uint8_t* respLen, uint8_t* nResp);
// mmCfg stages addressing/bus config (applied on next bus acquire; if the
// bus is already up and bus/baud changed, it is dropped and re-acquired
// lazily). tpMs: 0 = tester-present off, else keepalive interval.
uint8_t mmCfg(uint8_t bus, uint32_t baud, uint32_t txId, uint32_t rxId,
              uint8_t func, uint16_t tpMs);
void mmStatus(uint8_t& session, uint8_t& bus, uint32_t& baud,
              uint32_t& txId, uint32_t& rxId, uint8_t& func, uint8_t& tpOn);

// NRC -> text (ISO 14229-1); "unknown NRC" for unlisted values.
const char* nrcText(uint8_t nrc);

// Phase 9: run the discovery engine (releases the bus first).
// For CMD_DISCOVER_START dispatch.
uint8_t mmDiscover(uint8_t bus);

} // namespace uds
