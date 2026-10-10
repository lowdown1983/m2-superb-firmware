#pragma once
// M2 multi-mode firmware — ISO 15765-2 transport layer (Phases 2+6).
//
// SHARED module: ELM/STN/UDS/OBD2/PROG all use it; single-frame,
// first-frame / flow-control / consecutive-frame in both directions.
//
// Phase 6: first-class configurability — STmin, block size, P2/P2*
// timers, N_Bs/N_Cr timeouts, extended + mixed addressing, manual vs
// automatic flow-control generation, cancellation/timeout API, and
// payloads above 64 bytes (up to ISOTP_BUF_MAX). The F5 (2026-10-08)
// BS/STmin enforcement is folded into this design (txPayload honors the
// peer's FC BS/STmin; our own FC advertises cfg.maxBlockSize/cfg.stMin).
//
// SEAM FOR PHASE 9/12 (OEM database): AddrType + addrExt are the
// transport-level addressing knobs the OEM database will drive —
// physical vs functional addressing, ECU target addresses (extended /
// mixed addressing AE byte), per-ECU ISO-TP timing (N_Bs/N_Cr/P2/P2*,
// block size, STmin). Phase 6 plumbs the knobs; Phase 9's PID/DID
// registries and discovery engine will populate them from the OEM
// database's diagnostic-access data without a transport refactor.
//
// SOURCES: ISO 15765-2 frame layout (PCI nibble in data[0]: 0x0N single
// frame, 0x1N first frame, 0x2N consecutive, 0x3N flow control) is the
// public standard layout; N_As/N_Bs/N_Cr = 1000 ms defaults are the
// standard's defaults. No bench exists — label INFERRED where noted.
//
// Evidence: VERIFIED = compiles. INFERRED = untested against real ECUs.

#include <Arduino.h>
#include <due_can.h>  // CANRaw, CAN_FRAME (collin80/due_can 2.1.6)

namespace isotp {

struct Endpoint {
    CANRaw* bus;       // &Can0 or &Can1 (due_can globals)
    uint32_t txId;     // arbitration ID we transmit on
    uint32_t rxAnyId;  // 0 = accept any RX ID (filtered by caller), else exact
    bool extended;     // 29-bit (true) vs 11-bit (false); see AddrType::AUTO
};

// ISO 15765-2 addressing types. EXTENDED/MIXED_11 put the address
// extension (target address) byte first in the CAN payload and shift the
// PCI by one byte. MIXED_29: the AE rides in the 29-bit CAN ID (built by
// the caller into txId); on-the-wire framing is 29-bit normal — the ID
// construction rules are ECU/OEM-specific and live in Phase 9's data,
// not here.
enum class AddrType : uint8_t {
    AUTO = 0,      // derive from Endpoint.extended (11 vs 29-bit normal)
    NORMAL_11 = 1,
    NORMAL_29 = 2,
    EXTENDED = 3,
    MIXED_11 = 4,
    MIXED_29 = 5,
};

// Hard reassembly ceiling. Config.maxPayload (<= this) sets the live cap.
constexpr uint16_t ISOTP_BUF_MAX = 256;

struct Config {
    AddrType addrType = AddrType::AUTO;
    uint8_t addrExt = 0;        // AE / target-address byte (EXTENDED, MIXED_11)
    uint16_t nBsMs = 1000;      // N_Bs: how long TX waits for Flow Control
    uint16_t nCrMs = 1000;      // N_Cr: how long RX waits for the next CF
    uint16_t p2Ms = 50;         // P2: request -> first response (informational;
                                //   sizes nothing by itself, reported for Phase 9)
    uint16_t p2StarMs = 5000;   // P2*: extended timing; used as the overall RX
                                //   window when requestMulti gets timeoutMs == 0
    uint8_t maxBlockSize = 0;   // BS we advertise in OUR Flow Control (0 = send-all)
    uint8_t stMin = 0;          // STmin byte we advertise in OUR Flow Control
    bool autoFlowControl = true;  // false: caller sends FC via sendFlowControl()
    uint16_t maxPayload = 64;     // reassembly cap, 8..ISOTP_BUF_MAX
    const volatile bool* cancel = nullptr;  // cooperative cancel flag
};

// One fully reassembled response.
struct Response {
    uint32_t senderId;              // CAN ID the response arrived on
    uint8_t data[ISOTP_BUF_MAX];    // payload bytes (PCI + AE stripped)
    uint16_t len;
};

// Blocking multi-response transact: sends req[reqLen], collects up to
// maxResp complete responses. Stops early once at least one response is in
// and the bus has been quiet for quietMs. Returns number collected (0 =
// none / timeout / cancelled). timeoutMs == 0 selects the P2* window
// (cfg->p2StarMs). Single-threaded; drains stale RX frames before TX.
// When the shared busfilter table is non-empty, received frames must also
// pass busfilter::accept() (drops are counted in busstat).
uint8_t requestMulti(const Endpoint& ep,
                     const uint8_t* req, uint8_t reqLen,
                     Response* out, uint8_t maxResp,
                     uint16_t timeoutMs = 1000, uint16_t quietMs = 200,
                     const Config* cfg = nullptr);

// Convenience single-response wrapper.
inline bool request(const Endpoint& ep, const uint8_t* req, uint8_t reqLen,
                    uint8_t* resp, uint8_t respCap, uint8_t& respLen,
                    uint16_t timeoutMs = 1000, const Config* cfg = nullptr)
{
    Response r;
    uint8_t n = requestMulti(ep, req, reqLen, &r, 1, timeoutMs, 0, cfg);
    if (n == 0) return false;
    respLen = (r.len <= respCap) ? (uint8_t)r.len : respCap;
    for (uint8_t i = 0; i < respLen; i++) resp[i] = r.data[i];
    return true;
}

// Transmit-only (Phase 4): sends req as a single frame or ISO-TP-
// segmented (with FC handling). No RX phase. Used by STPX, which
// collects raw response frames itself (real STN behavior).
enum class TxResult : uint8_t { OK, SEND_FAIL, FC_TIMEOUT, CANCELLED };
TxResult transmit(const Endpoint& ep, const uint8_t* req, uint8_t reqLen,
                  const Config* cfg = nullptr);

// Manual Flow Control generation (cfg->autoFlowControl == false, or STN
// STCFC* style explicit control): send one FC frame for this endpoint.
// fs: 0=CTS, 1=WAIT, 2=OVFLW. Uses cfg addressing (AE byte when set).
TxResult sendFlowControl(const Endpoint& ep, uint8_t fs, uint8_t bs,
                         uint8_t stMin, const Config* cfg = nullptr);

// Global default config: used whenever a caller passes cfg == nullptr.
// Initialized to the historical behavior (normal addressing from the
// endpoint flag, 1000 ms N_Bs/N_Cr, auto FC, 64-byte cap). CMD_ISOTP_CFG
// (mm) and the `isotp set` console command mutate it; `isotp show`
// reports it. Phase 9 will drive per-ECU configs explicitly instead.
void setDefault(const Config& cfg);
void getDefault(Config& cfg);

} // namespace isotp
