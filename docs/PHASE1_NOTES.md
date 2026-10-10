# Phase 1 Notes — M2RET mode + SuperB wireless bridge

No bench exists, so everything below is **compile-verified only**. Nothing has
been flashed; no bus, button, LED, or radio behavior has been observed on
hardware. Evidence labels are used throughout: **VERIFIED** = compiles and/or
sourced from a primary source; **INFERRED** = hardware behavior assumed but
untestable here.

## What was built

**M2 (`src/m2/`)**
- `mode_manager.h/.cpp` — mode enum {m2ret, elm327, stn, uds, mitm, obd2,
  prog}; only M2RET implemented (others print "not implemented in Phase 1"
  and the manager stays put). Safe switching: every transition runs
  `quiesceBuses()` (CAN controllers to init/off, TH8056 to sleep, J1850 rail
  off) before the new mode acquires its resources. Selection via Button1
  (short press = cycle, long press >1 s = back to M2RET) and SerialUSB text
  console (`help`, `mode list`, `mode <name>`, `status`). Boots into M2RET.
- `mode_m2ret.h/.cpp` — GVRET binary protocol on `Serial` (Bee UART @115200,
  immutable). Implements commands 0–14 plus the 0xE7 binary-enable byte,
  faithful to M2RET (see sources below).
- `main.cpp` — begins `Serial` (Bee) + `SerialUSB` (control console), hands
  off to the mode manager.

**SuperB (`src/superb/main.cpp`)** — bridge only:
- Bluetooth Classic SPP, device name `M2-SuperB` (arduino-esp32
  `BluetoothSerial`).
- WiFi AP+STA: open AP `M2-SuperB`; STA attempts a placeholder network and a
  failed join never blocks the AP or bridging.
- TCP server on port 23, raw byte pipe to the Bee UART (SavvyCAN GVRET-TCP
  target). Up to 3 clients; a 4th evicts the oldest (documented
  last-writer-wins policy).
- Bridging: Bee UART → broadcast to BT + all TCP clients; clients → Bee UART
  gathered per client and written in one call (single-threaded loop =
  natural serialization).
- GPIO26 heartbeat LED (INFERRED from Macchina's flashing docs). No USB
  diagnostics exist on the socketed SuperB, and the BT stream is the GVRET
  data plane, so nothing is ever printed to BT.

## Protocol sources (M2RET)

Primary source only — nothing reconstructed from memory:
- Repo: https://github.com/collin80/M2RET, commit
  `ad9d5af67d8e7c3330c0281e0403c2750e023f1c` (cloned to /tmp for reference).
- `CommProtocol.txt` — framing doc (0xF1 command prefix, 0xE7 binary enable,
  per-command byte layouts, RX frame format).
- `M2RET.h` — `enum STATE` (parser states), `enum GVRET_PROTOCOL`
  (commands 0–14).
- `M2RET.ino` — the binary state machine (`loop()`, ~lines 820–1280),
  `sendFrameToUSB()` binary branch (lines 540–560), `setPromiscuousMode()`
  (398–412), `checksumCalc()` XOR (418–427), `CANHandler()` → 
  `SWCAN.intHandler()` (115–117), `Can0.begin(speed, 255)` (320),
  SWCAN enable/disable sequences (`SWCANEN` handler; `SETUP_EXT_BUSES`).
- `config.h` — defaults: CAN0/CAN1 500000, SWCAN 33333.

Implemented GVRET commands: 0 build/send frame (incl. the id==0x100-on-SWCAN
→ high-speed-mode quirk, M2RET.ino:1052–1067), 1 time sync, 2 digital inputs
(zeros), 3 analog inputs (zeros), 4 set digital out (accepted/ignored),
5 setup CAN buses, 6 get CAN params, 7 device info (build 1, zeros), 8 set
single-wire mode (accepted/ignored), 9 keepalive (DE AD), 10 set system type
(RAM only), 11 echo frame, 12 get bus count (3), 13 get ext buses, 14 setup
ext buses (SWCAN live; LIN words accepted/ignored, as upstream).

One upstream discrepancy found and resolved: `CommProtocol.txt` says a reply
is "0xF1 followed by the command byte", but M2RET's DIG_INPUTS handler writes
`6` as byte 1 (apparent copy-paste from GET_CANBUS_PARAMS). This
implementation follows the documented protocol (byte 1 = 2) and notes the
quirk in a code comment. SavvyCAN never issues this command at connect, so
it is unobservable in practice.

## Library choices (pinned, `[env:m2]`)

| Library | Version | Why |
|---|---|---|
| `collin80/due_can` | 2.1.6 (exact) | M2RET's CAN driver; only registry version |
| `collin80/can_common` | 0.4.0 (exact) | due_can 2.1.6 `#include`s `<can_common.h>` but declares no dependency — added explicitly |
| `macchina/MCP2515` | 0.3.1 (exact) | Base class for SWcan; provides `intHandler()`/`GetRXFrame()` used by M2RET's pattern |
| `lib/SWcan` (vendored) | Macchina Single-Wire-CAN-mcp2515, MIT | No registry package provides `SWcan::setupSW()/mode()`; copied verbatim from https://github.com/macchina/Single-Wire-CAN-mcp2515 (headers + `License.txt` kept) |

Compile-time warnings: none in project code (`src/m2/*`, `lib/SWcan/*`,
`src/superb/*`). The third-party `macchina/MCP2515@0.3.1` emits
`-Wreturn-type` / `-Wmaybe-uninitialized` warnings; left as-is (upstream
code, also used verbatim by M2RET).

## INFERRED vs VERIFIED

**VERIFIED (primary source or compiler):**
- GVRET framing/commands/semantics — M2RET source @ ad9d5af (cited per
  command in `mode_m2ret.cpp`).
- SWCAN sleep encoding: `mode(0)` = M0 LOW + M1 LOW (MCP2515_sw_can.cpp:49–52).
- due_can enable-pin hazard: `begin()` drives Arduino 50/48 by default
  (due_can.cpp:267–269); on the M2 those are J1850_PWM_VPW and the 5 V
  buck-boost (variant.cpp). Mitigated by always passing 255
  (`CAN_COMMON::begin(baud, 255)` sets `enablePin=255`; `init()` then skips
  pin driving). `disable()` is only called after a `begin(...,255)`.
- `CANRaw::disable()` clears `CAN_MR_CANEN` (due_can.cpp:458).
- SavvyCAN GVRET-TCP uses port 23 (three independent community
  implementations: esphome_gvret, evj55-dashboard, shady-elm) and its
  connect sequence is `e7 e7 f1 0c f1 06 f1 07 f1 01 f1 09` (SavvyCAN issue
  log) — all covered by the implementation.
- Both environments compile warning-free (project code); `pio run` green.

**INFERRED (no bench; must be validated on hardware):**
- Button1 polarity/travel (active LOW assumed from Macchina's passthrough
  sketch; `INPUT_PULLUP` chosen defensively), debounce 50 ms, 1 s long-press
  threshold.
- DS2 heartbeat was Phase 0; Phase 1 removed the LED blink from main loop
  (mode manager owns the loop now). LED behavior on real hardware untested.
- SuperB GPIO26 as status LED (from Macchina flashing-docs blink example).
- `huge_app.csv` partition: assumes a 4 MB WROOM-32D (audit open question).
  Needed because Bluedroid + WiFi exceed the default 1 MB app partition.
- STA placeholder credentials; open AP (no WPA2 yet).
- SWCAN `setupSW(33333)` register values (audit HW-055 still open); the
  vendored library computes them, but no live capture exists.
- TCP port 23 for *our* device (ecosystem-consistent, unconfirmed here).
- `Serial.setRxBufferSize(1024)` burst sizing.

## How SavvyCAN should connect (once flashed)

- **WiFi:** join AP `M2-SuperB` (or put the SuperB on your LAN in Phase 2),
  SavvyCAN → Connection → "Network Connection", host = SuperB IP, port 23.
  Set M2RET mode on the M2 (was default at boot through 2026-10-07; ELM327
  is the default since 2026-10-08). SavvyCAN will send
  `E7 E7 F1 0C F1 06 F1 07 F1 01 F1 09` and start streaming.
- **Bluetooth:** pair `M2-SuperB` SPP, then SavvyCAN serial connection at
  115200 to the BT COM port.
- **Direct USB:** M2's USB port does NOT carry GVRET in this firmware (that
  port is the human control console). This is a deliberate Phase 1 split:
  data plane = Bee link, control plane = USB.

## Known simplifications (Phase 1)

1. No EEPROM persistence — all GVRET settings are RAM-only and reset on
   reboot (M2RET uses the 24LC32A; wiring that is Phase 2+).
2. No Lawicel ASCII mode, no text config console, no SD logging.
3. Digital/analog input commands reply zeros; digital outputs ignored.
4. Device info reports build 1 with zeroed EEPROM/file fields.
5. SPI0 "arbitration" is trivially satisfied (single client); a real arbiter
   is needed if a second SPI0 client ever appears (see RESOURCE_OWNERSHIP.md
   checklist item 4).
6. Concurrent BT + TCP clients: last-writer-wins, no per-client session
   state.

## Hooks left for Phase 2

- **Web GUI** (SuperB): mode switching from the GUI needs a safe M2↔SuperB
  command channel. Deliberately deferred: in Phase 1 the Bee link is a pure
  GVRET byte pipe, and there is no in-band command that can't collide with
  GVRET traffic. Phase 2 must define an out-of-band channel (e.g. a second
  TCP port for control, or a GVRET escape sequence) *before* wiring GUI
  buttons to mode switches. Until then: modes switch via M2 Button1/USB only.
- STA credentials, WPA2 AP, web server, OTA partition layout.
- EEPROM (24LC32A) persistence for GVRET settings.
- ELM327 mode (J1850 PWM/VPW now confirmed in scope), STN subset, UDS,
  MITM, OBD2 poller, PROG — each gets acquire/release lists from
  docs/RESOURCE_OWNERSHIP.md; note TC6/TC7/PWM6 remain unclaimed in Phase 1.
- Bench validation checklist: flash both, Bee echo, GVRET connect from
  SavvyCAN over TCP and BT, mode-switch bus quiesce observed on a scope or
  second CAN node, button feel, LED polarity.
