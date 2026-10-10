# SIM — host-native simulation harness

**Status: VERIFIED** — both sim envs compile from the unmodified firmwares and
all 5 smoke tests pass on Linux (2026-10-07).

## What this is

A way to prove the M2 and SuperB firmwares are alive and talking to each
other **without hardware**. Both firmwares compile **unmodified** against sim
shims and run as ordinary Linux programs on your workstation. The Bee UART
link between them is a byte pipe through a small Python broker.

```
                  ┌─────────────────────────────┐
                  │      sim/broker.py          │
                  │  127.0.0.1:14001 ←→ :14002 │
                  │  logs sim/traffic.log      │
                  └──────┬──────────────┬───────┘
                         │ Bee bytes    │ Bee bytes
            ┌────────────▼──┐      ┌────▼──────────────┐
            │ sim_m2        │      │ sim_superb        │
            │ (M2 firmware, │      │ (SuperB firmware, │
            │  unmodified)  │      │  unmodified)      │
            │               │      │                   │
            │ Serial →TCP   │      │ Serial →TCP       │
            │ SerialUSB     │      │ Web GUI → :18080  │
            │  → your       │      │ TCP:23  → :14023  │
            │  terminal     │      │ BT SPP  → :14024  │
            └───────────────┘      └───────────────────┘
```

## What it proves / does NOT prove

**Proves (Marcus's definition of done):**
1. The M2 firmware is present and running — banner + console answers.
2. The SuperB firmware is present and running — web GUI answers.
3. The two communicate over the Bee link — the SuperB's `m2_ok` flag is only
   set after decoding a **real** `F1 7E` `CMD_GET_STATUS` reply from the M2;
   mode switches via the web GUI drive the M2's real mode manager.

**Does NOT prove (stays INFERRED until hardware/vehicle):**
- CAN electrical behavior, bus timing, arbitration — the sim CAN buses are
  **loopbacks** (a sent frame is readable on the same controller), not a
  real two-node bus.
- J1850 PWM/VPW bit timing — the GPIO/interrupt stubs never fire.
- SWCAN transceiver behavior, WiFi/BT radios, NVS wear, 12V rail, timing of
  `millis()`-based state machines under load.
- Anything about a real vehicle. Vehicle-behavior simulation is deferred
  (see Roadmap below).

## Port map

| Port | What | Notes |
|---|---|---|
| 14001 | Broker side A — sim_m2 Bee `Serial` | override: `SIM_BEE_PORT` (M2) |
| 14002 | Broker side B — sim_superb Bee `Serial` | override: `SIM_BEE_PORT` (SuperB) |
| 18080 | SuperB web GUI | firmware's port 80 (80 needs root) |
| 14023 | SuperB TCP:23 bridge | firmware's port 23 (23 needs root) |
| 14024 | BT SPP shim | "BT SPP is TCP in sim" — connect any TCP client |

Baud rate is meaningless in sim (byte pipe); 115200 is documented, not enforced.

## Interactive console

```bash
cd ~/workspace/m2-superb-firmware
pio run -e sim_m2 -e sim_superb   # or: ./sim/run.sh does this for you
./sim/run.sh
```

`run.sh` builds both envs, starts the broker (log: `sim/traffic.log`), starts
the SuperB sim in the background, waits for all ports, then attaches the
**M2's SerialUSB console to your terminal**. Try:

```
help            # console commands
status          # Mode: m2ret, CAN begun, settings
mode list       # all seven modes
mode elm327     # switch mode (quiesce path runs, instant in sim)
```

Then open **http://127.0.0.1:18080** — the real SuperB web GUI. The status
page shows the M2's live mode/buses (fetched over the virtual Bee link).
Connect a TCP client to **127.0.0.1:14024** (the BT shim) and type `ATI`
after switching the M2 to ELM mode — you'll get `ELM327 v1.5` proxied back
through the SuperB.

Ctrl-C (or Ctrl-D on the console) stops everything.

## Automated smoke test

```bash
cd ~/workspace/m2-superb-firmware
pio run -e sim_m2 -e sim_superb
python3 -m pytest sim/tests/smoke.py -v
```

The test starts its own broker + sims (nothing else may hold the sim ports)
and proves, in order:

| Test | Claim proved |
|---|---|
| `test_a_m2_alive` | M2 boots (banner) and answers `status` → `m2ret` |
| `test_b_superb_alive` | `GET /api/status` → 200 JSON |
| `test_c_bee_link` | `m2_ok:true` — a real `F1 7E` round-trip over the Bee link |
| `test_d_mode_switch_roundtrip` | `POST /mode_set` switches the M2 to `elm327` and back |
| `test_e_bt_app_data_path` | `ATI` over the BT shim → M2 → `ELM327` proxied back |

Exit code is nonzero on any failure. Run it before every flash.

## Reading sim/traffic.log

Every Bee-link byte in both directions, timestamped:

```
2026-10-07T12:52:46.665 B->A 5 bytes: f1 7e 01 02 02 |.~...|
2026-10-07T12:52:46.665 A->B 20 bytes: f1 7e 10 82 00 ... |.~....|
```

`B->A` = SuperB→M2 (`f1 7e 01 02 02` = `CMD_GET_STATUS` request),
`A->B` = M2→SuperB (`82` = reply to command `0x02`). `|...|` is printable
ASCII (`.` = non-printable). This is the fastest way to debug the control
channel: you see exactly what the firmwares put on the wire.

## How it works (for developers)

- `src/sim_shim/` — `Arduino.h` (String, Print, HardwareSerial, GPIO stubs,
  millis/micros/delay, interrupts, EEPROM, map/min/max, ESP.restart),
  `due_can.h` (`CAN_FRAME`/`BytesUnion` copied **verbatim** from
  collin80/can_common so the layout matches; `CANRaw` = virtual loopback),
  `MCP2515_sw_can.h` (same SWcan API, loopback), `sim_main.cpp` (`main()`
  calling the real `setup()`/`loop()`), `sim_support.cpp` (socket/stdio
  transports). M2 pin-name constants (`J1850P_TX` etc.) are repeated
  verbatim from `variants/m2/variant.h` (values only; the variant isn't on
  the sim include path).
- `src/sim_shim_esp/` — `WiFi.h` (real TCP `WiFiServer`/`WiFiClient`; AP/STA
  are recorded no-ops), `WebServer.h` (real HTTP/1.1: chunked streaming
  responses, urlencoded args, multipart upload driving `HTTPUpload`),
  `BluetoothSerial.h` (TCP server standing in for SPP),
  `Preferences.h` (file-backed NVS in `sim/nvs/`).
- `platformio.ini` — `[env:sim_m2]` / `[env:sim_superb]` are **appended**;
  the `[env:m2]` / `[env:superb]` sections are untouched and their
  `build_src_filter` never matches `sim_shim*`.
- `sim/broker.py`, `sim/run.sh`, `sim/tests/smoke.py` — as described above.

**Additive-only rule:** nothing under `src/m2/`, `src/superb/`, `lib/`,
`boards/`, `variants/`, `docs/` (except this file) was modified.

## Verify on Bazzite

```bash
# one-time: PlatformIO + pytest (pick the route from SETUP.md you use)
pip install --user platformio pytest        # or: pipx install platformio

cd ~/workspace/m2-superb-firmware
pio run -e sim_m2 -e sim_superb             # native platform: gcc only, no toolchains
python3 -m pytest sim/tests/smoke.py -v    # expect 5 passed
./sim/run.sh                               # interactive session
```

Notes:
- The `native` platform needs only gcc/make — no ARM/ESP toolchains, no USB.
- If `pio` isn't on PATH in the Flatpak VS Code terminal, run from a host
  terminal or the distrobox described in SETUP.md §1.
- First build downloads the `native` platform (~tens of MB) — needs network.

## Scripted vehicle ECU (Step 2 — 2026-10-10)

`src/sim_shim/vehicle_ecu.*` answers OBD and UDS on CAN0/CAN1 via
`injectFrame()`:

**OBD Mode 01** (ELM path): PIDs 00/05/0B/0C/0D/0F/11/1C/2F — static
INFERRED values (RPM 3000, speed 80 km/h, …).

**UDS** (console `uds …` path):
- 0x10 DiagnosticSessionControl (01/02/03)
- 0x22 ReadDataByIdentifier (F186 session, F187/F188/F18A/F197 part no,
  F190 VIN multi-frame)
- 0x3E TesterPresent
- 0x19 ReadDTCInformation (02 → empty list)
- 0x27 SecurityAccess (seed stub / key reject)
- other SIDs → NRC 0x11

Bus model: same-controller TX is not echoed to own RX (ISO-TP safe).
CAN0↔CAN1 are cross-bridged for HARNESS selfcheck.

Tests:
```bash
pio run -e sim_m2 -e sim_superb
python3 -m pytest sim/tests/smoke.py \
  sim/tests/test_vehicle_elm.py sim/tests/test_vehicle_uds.py -v
```

## Roadmap (deferred)

- **Scripted vehicle depth:** more UDS services (0x14 clear DTC, 0x2E
  write), Mode 09 OBD multi-frame, periodic broadcast frames,
  python-driven scenario scripts.
- **Fault injection:** drop/corrupt Bee bytes via the broker to test the
  control channel's resync and timeout paths.
- **J1850 timing sim:** virtual microsecond-accurate GPIO for the PWM/VPW
  driver (still INFERRED vs. silicon, but exercises the bit logic).
- **CI:** run the smoke + vehicle tests on every commit before hardware flash.
