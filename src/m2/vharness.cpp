// M2 multi-mode firmware — virtual ECU / self-test harness (Phase 12).
// See vharness.h.

#include "vharness.h"
#include "isotp.h"
#include "busstat.h"
#include "debug.h"
#include <due_can.h>
#include <string.h>

namespace vharness {
namespace {

// Scripted ECU identity.
constexpr uint32_t ECU_REQ_ID = 0x7E0;  // tester -> ECU (we listen on CAN1)
constexpr uint32_t ECU_RESP_ID = 0x7E8; // ECU -> tester (we send on CAN1)
constexpr uint32_t TEST_BAUD = 500000;

// Scripted VIN (20 bytes for the multi-frame test).
static const char VIN20[] = "1G1BL52P7TR115520___";  // 17 + 3 pad = 20

// Fault-injection modes for the ECU simulator.
enum class SimMode : uint8_t { NORMAL, DELAY_CF, DROP_CF, BAD_SN };
SimMode simMode = SimMode::NORMAL;

SuiteResult last;
bool loopbackOk = false;
bool begun = false;

void txCan1(uint32_t id, const uint8_t* d, uint8_t len)
{
    CAN_FRAME f;
    f.id = id;
    f.extended = false;
    f.length = len;
    f.rtr = 0;
    for (uint8_t i = 0; i < len && i < 8; i++) f.data.bytes[i] = d[i];
    Can1.sendFrame(f);
    busstat::noteTx(busstat::Bus::CAN_BUS1, len);
}

// Send an ISO-TP multi-frame response by hand (full control for fault
// injection). ffLen = total payload length; payload = bytes after PCI.
void txMulti(uint32_t id, const uint8_t* payload, uint16_t payloadLen)
{
    uint8_t ff[8];
    ff[0] = 0x10 | ((payloadLen >> 8) & 0x0F);
    ff[1] = payloadLen & 0xFF;
    uint8_t n0 = (payloadLen > 6) ? 6 : (uint8_t)payloadLen;
    memcpy(ff + 2, payload, n0);
    txCan1(id, ff, 8);

    // Wait for the tester's Flow Control (simple poll, 500ms).
    uint32_t t0 = millis();
    CAN_FRAME fr;
    bool fcOk = false;
    while (millis() - t0 < 500) {
        if (Can1.available() > 0) {
            Can1.read(fr);
            if (fr.id == ECU_REQ_ID && fr.length >= 3 &&
                (fr.data.bytes[0] & 0xF0) == 0x30) {
                fcOk = true;
                break;
            }
        }
    }
    if (!fcOk) return;

    uint16_t sent = n0;
    uint8_t sn = 1;
    bool first = true;
    while (sent < payloadLen) {
        if (simMode == SimMode::DROP_CF && !first) return;  // stall: N_Cr
        if (simMode == SimMode::DELAY_CF) delay(10);        // STmin-ish
        uint8_t cf[8];
        uint8_t thisSn = sn;
        if (simMode == SimMode::BAD_SN && !first) thisSn = (sn + 3) & 0x0F;
        cf[0] = 0x20 | thisSn;
        uint8_t n = (payloadLen - sent > 7) ? 7 : (uint8_t)(payloadLen - sent);
        memcpy(cf + 1, payload + sent, n);
        for (uint8_t i = 1 + n; i < 8; i++) cf[i] = 0xAA;  // pad (visible)
        txCan1(id, cf, 8);
        sent += n;
        sn = (sn + 1) & 0x0F;
        first = false;
    }
}

// ECU simulator: answer scripted requests arriving on CAN1.
void pumpSim()
{
    CAN_FRAME fr;
    while (Can1.available() > 0) {
        Can1.read(fr);
        busstat::noteRx(busstat::Bus::CAN_BUS1, fr.length);
        if (fr.id != ECU_REQ_ID || fr.length < 3) continue;
        uint8_t pci = fr.data.bytes[0];
        if ((pci & 0xF0) != 0x00) continue;  // single-frame requests only
        uint8_t svcLen = pci & 0x0F;
        if (svcLen < 2) continue;
        uint8_t sid = fr.data.bytes[1];
        uint8_t pid = fr.data.bytes[2];

        if (sid == 0x01 && pid == 0x0C) {
            // OBD 010C: RPM 0xBEBA -> 12206.5 rpm (scripted constant).
            uint8_t r[8] = { 0x04, 0x41, 0x0C, 0xBE, 0xBA, 0x00, 0x00, 0x00 };
            txCan1(ECU_RESP_ID, r, 8);
        } else if (sid == 0x09 && pid == 0x02) {
            // 0902 VIN: 20-byte multi-frame.
            uint8_t vin[20];
            memcpy(vin, VIN20, 20);
            txMulti(ECU_RESP_ID, vin, 20);
        } else if (sid == 0x22 && fr.length >= 4 &&
                   fr.data.bytes[2] == 0xF1 && fr.data.bytes[3] == 0x90) {
            // UDS 22 F190: scripted DID (4 data bytes).
            uint8_t r[8] = { 0x06, 0x62, 0xF1, 0x90, 0xDE, 0xAD, 0xBE, 0x00 };
            txCan1(ECU_RESP_ID, r, 8);
        } else if (sid == 0x22) {
            // Any other DID: negative response (requestOutOfRange).
            uint8_t r[8] = { 0x03, 0x7F, 0x22, 0x31, 0x00, 0x00, 0x00, 0x00 };
            txCan1(ECU_RESP_ID, r, 8);
        }
    }
}

bool txMarker(CANRaw& txBus, CANRaw& rxBus, uint32_t id)
{
    // Drain stale frames first.
    CAN_FRAME fr;
    while (rxBus.available() > 0) rxBus.read(fr);
    CAN_FRAME f;
    f.id = id;
    f.extended = false;
    f.length = 8;
    const char* m = "M2HARNES";
    for (uint8_t i = 0; i < 8; i++) f.data.bytes[i] = (uint8_t)m[i];
    f.rtr = 0;
    txBus.sendFrame(f);
    uint32_t t0 = millis();
    while (millis() - t0 < 200) {
        if (rxBus.available() > 0) {
            rxBus.read(fr);
            if (fr.id == id && fr.length == 8 &&
                memcmp(fr.data.bytes, "M2HARNES", 8) == 0)
                return true;
        }
    }
    return false;
}

void setResult(SuiteResult& out, uint8_t idx, uint16_t id, bool pass,
               const char* note)
{
    out.tests[idx].id = id;
    out.tests[idx].pass = pass;
    strncpy(out.tests[idx].note, note, 39);
    out.tests[idx].note[39] = '\0';
    out.nTests++;
    if (pass) out.nPass++;
}

// Tester endpoint on CAN0.
isotp::Endpoint testerEp()
{
    isotp::Endpoint ep;
    ep.bus = &Can0;
    ep.txId = ECU_REQ_ID;
    ep.rxAnyId = ECU_RESP_ID;
    ep.extended = false;
    return ep;
}

} // namespace (anonymous)

void begin()
{
    // Pin-safety: 255 = no transceiver-enable pin driving (invariant).
    Can0.begin(TEST_BAUD, 255);
    Can1.begin(TEST_BAUD, 255);
    for (int f = 0; f < 7; f++) {
        Can0.setRXFilter(f, 0, 0, false);
        Can1.setRXFilter(f, 0, 0, false);
    }
    busstat::setPresent(busstat::Bus::CAN_BUS0, true);
    busstat::setPresent(busstat::Bus::CAN_BUS1, true);
    loopbackOk = false;
    begun = true;
    dbg::println(F("[vharness] begin: CAN0+CAN1 @500k (loopback required)"));
}

void loop()
{
    if (!begun) return;
    pumpSim();  // the scripted ECU lives on CAN1
}

void exit()
{
    busstat::setPresent(busstat::Bus::CAN_BUS0, false);
    busstat::setPresent(busstat::Bus::CAN_BUS1, false);
    Can0.disable();
    Can1.disable();
    begun = false;
    loopbackOk = false;
    dbg::println(F("[vharness] exit"));
}

bool selfcheck()
{
    if (!begun) return false;
    // Marker both directions. A vehicle bus will not echo these.
    bool a = txMarker(Can0, Can1, 0x7FF);
    bool b = txMarker(Can1, Can0, 0x7FE);
    loopbackOk = a && b;
    dbg::print(F("[vharness] selfcheck: "));
    dbg::println(loopbackOk ? F("LOOPBACK OK") : F("FAILED"));
    return loopbackOk;
}

bool runSuite(uint16_t mask, SuiteResult& out)
{
    memset(&out, 0, sizeof(out));
    if (!begun || !loopbackOk) {
        dbg::println(F("[vharness] runSuite REFUSED: no passing selfcheck"));
        return false;
    }
    out.ran = true;
    uint8_t ti = 0;
    isotp::Endpoint ep = testerEp();

    // T1: OBD 010C single-frame.
    if (mask & T_OBD_SINGLE) {
        simMode = SimMode::NORMAL;
        uint8_t req[2] = { 0x01, 0x0C };
        uint8_t resp[32];
        uint8_t rlen = 0;
        bool ok = isotp::request(ep, req, 2, resp, sizeof(resp), rlen, 1000);
        bool pass = ok && rlen >= 4 && resp[0] == 0x41 && resp[1] == 0x0C &&
                    resp[2] == 0xBE && resp[3] == 0xBA;
        setResult(out, ti++, T_OBD_SINGLE, pass,
                  pass ? "010C -> 41 0C BE BA" : "bad/no response");
    }
    // T2: 0902 VIN multi-frame reassembly.
    if (mask & T_ISOTP_MULTI) {
        simMode = SimMode::NORMAL;
        uint8_t req[2] = { 0x09, 0x02 };
        uint8_t resp[32];
        uint8_t rlen = 0;
        bool ok = isotp::request(ep, req, 2, resp, sizeof(resp), rlen, 2000);
        bool pass = ok && rlen == 20 && memcmp(resp, "1G1BL52P7", 9) == 0;
        setResult(out, ti++, T_ISOTP_MULTI, pass,
                  pass ? "20B VIN reassembled" : "reassembly failed");
    }
    // T3: negative response 7F 22 31.
    if (mask & T_NEG_RESP) {
        simMode = SimMode::NORMAL;
        uint8_t req[3] = { 0x22, 0xF1, 0x91 };
        uint8_t resp[32];
        uint8_t rlen = 0;
        bool ok = isotp::request(ep, req, 3, resp, sizeof(resp), rlen, 1000);
        bool pass = ok && rlen >= 3 && resp[0] == 0x7F && resp[1] == 0x22 &&
                    resp[2] == 0x31;
        setResult(out, ti++, T_NEG_RESP, pass,
                  pass ? "7F 22 31 seen" : "neg response missed");
    }
    // T4: STmin-delayed CFs still reassemble.
    if (mask & T_STMIN_DELAY) {
        simMode = SimMode::DELAY_CF;
        uint8_t req[2] = { 0x09, 0x02 };
        uint8_t resp[32];
        uint8_t rlen = 0;
        bool ok = isotp::request(ep, req, 2, resp, sizeof(resp), rlen, 3000);
        bool pass = ok && rlen == 20;
        setResult(out, ti++, T_STMIN_DELAY, pass,
                  pass ? "10ms-spaced CFs ok" : "delayed CFs failed");
        simMode = SimMode::NORMAL;
    }
    // T5: stalled CF stream -> N_Cr timeout (NEGATIVE test: expect failure).
    if (mask & T_DROP_CF) {
        simMode = SimMode::DROP_CF;
        isotp::Config cfg;
        isotp::getDefault(cfg);
        cfg.nCrMs = 400;  // short watchdog for the test
        uint8_t req[2] = { 0x09, 0x02 };
        uint8_t resp[32];
        uint8_t rlen = 0;
        bool ok = isotp::request(ep, req, 2, resp, sizeof(resp), rlen,
                                 2000, &cfg);
        bool pass = !ok;  // we WANT the timeout here
        setResult(out, ti++, T_DROP_CF, pass,
                  pass ? "N_Cr fired as expected" : "stalled stream hung?!");
        simMode = SimMode::NORMAL;
    }
    // T6: bad sequence number -> error (NEGATIVE test: expect failure).
    if (mask & T_BAD_SN) {
        simMode = SimMode::BAD_SN;
        uint8_t req[2] = { 0x09, 0x02 };
        uint8_t resp[32];
        uint8_t rlen = 0;
        bool ok = isotp::request(ep, req, 2, resp, sizeof(resp), rlen, 2000);
        bool pass = !ok;
        setResult(out, ti++, T_BAD_SN, pass,
                  pass ? "bad SN rejected" : "bad SN accepted?!");
        simMode = SimMode::NORMAL;
    }
    // T7: 200-frame burst overload — must not lock up.
    if (mask & T_OVERLOAD) {
        uint32_t rx0 = 0;
        CAN_FRAME f;
        f.extended = false;
        f.length = 8;
        f.rtr = 0;
        for (uint8_t i = 0; i < 8; i++) f.data.bytes[i] = i;
        for (uint16_t i = 0; i < 200; i++) {
            f.id = 0x100 + (i & 0x7F);
            Can1.sendFrame(f);
            // Pump CAN0 RX between TXs (cooperative, like the main loop).
            int n = 0;
            while (Can0.available() > 0 && n < 16) {
                Can0.read(f);
                rx0++;
                n++;
            }
        }
        // Drain the rest.
        uint32_t t0 = millis();
        while (millis() - t0 < 500) {
            while (Can0.available() > 0) {
                Can0.read(f);
                rx0++;
            }
            if (rx0 >= 200) break;
        }
        bool pass = (rx0 >= 190);  // allow a few drops, not a lockup
        char nb[40];
        snprintf(nb, sizeof(nb), "rx %lu/200", (unsigned long)rx0);
        setResult(out, ti++, T_OVERLOAD, pass, nb);
    }
    // T8: UDS 22 F190 scripted DID.
    if (mask & T_UDS_DID) {
        simMode = SimMode::NORMAL;
        uint8_t req[3] = { 0x22, 0xF1, 0x90 };
        uint8_t resp[32];
        uint8_t rlen = 0;
        bool ok = isotp::request(ep, req, 3, resp, sizeof(resp), rlen, 1000);
        bool pass = ok && rlen >= 6 && resp[0] == 0x62 && resp[1] == 0xF1 &&
                    resp[2] == 0x90;
        setResult(out, ti++, T_UDS_DID, pass,
                  pass ? "62 F1 90 + data" : "DID response bad");
    }

    last = out;
    dbg::print(F("[vharness] suite done: "));
    dbg::print(out.nPass);
    dbg::print(F("/"));
    dbg::println(out.nTests);
    return true;
}

const SuiteResult& lastResult() { return last; }

void print(const SuiteResult& r)
{
    SerialUSB.println(F("--- harness suite ---"));
    if (!r.ran) {
        SerialUSB.println(F("not run yet"));
        return;
    }
    for (uint8_t i = 0; i < r.nTests; i++) {
        SerialUSB.print(r.tests[i].pass ? F("PASS ") : F("FAIL "));
        SerialUSB.print(F("T+"));
        SerialUSB.print(r.tests[i].id, HEX);
        SerialUSB.print(F(" "));
        SerialUSB.println(r.tests[i].note);
    }
    SerialUSB.print(r.nPass);
    SerialUSB.print(F("/"));
    SerialUSB.print(r.nTests);
    SerialUSB.println(F(" passed"));
}

} // namespace vharness
