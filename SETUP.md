# M2 + SuperB Firmware — Setup Guide (Phase 0)

This guide takes you from zero to building both firmwares. You don't need any
PlatformIO experience — follow it top to bottom the first time.

**What this project is**

```
m2-superb-firmware/
├── platformio.ini        # build config: two environments, one per chip
├── boards/
│   └── macchina_m2.json  # M2 board definition (real Macchina variant)
├── variants/
│   └── m2/               # Macchina's official M2 pin variant (verified)
├── src/
│   ├── m2/main.cpp       # M2 firmware (SAM3X8E) — prints "M2-MULTIMODE phase0"
│   └── superb/main.cpp   # SuperB firmware (ESP32) — prints "SUPERB phase0"
└── SETUP.md              # this file
```

- **env `m2`** — the Macchina M2 (Atmel SAM3X8E). Builds the vehicle-side firmware.
- **env `superb`** — the SuperB (ESP32-WROOM-32D). Builds the wireless-bridge firmware.

**Hardware rule you never need to touch:** the M2↔SuperB Bee link is the M2's
`Serial` at **115200 bps, immutable**. It's hardcoded in both firmwares with a
comment explaining why. Don't parameterize it.

---

## 1. Install the tools (do this once) — Bazzite

Your workstation is **Bazzite** (Fedora-based, immutable/atomic). That changes
three things vs. a normal Linux install:

1. **No system-wide installs** — the root filesystem is read-only. Apps come
   from **Flatpak** (preferred), a **distrobox** container, or `rpm-ostree`
   layering. We use the first two; no layering needed here.
2. **Flatpak apps can't see USB devices by default** — we must explicitly grant
   device access, or serial ports (`/dev/ttyACM0`) will be invisible inside the
   app. This is the #1 cause of "M2 won't connect" on Bazzite.
3. **Fedora's ModemManager grabs serial devices** — it will fight PlatformIO
   for the M2's port. We neuter it for the M2 below.

### Path A (recommended): VS Code as a Flatpak

1. Install VS Code from Flathub:
   ```bash
   flatpak install flathub com.visualstudio.code
   ```
2. Launch it, open Extensions (Ctrl+Shift+X), install **"PlatformIO IDE"**.
   Reload when asked. The extension bundles its own PlatformIO Core, Python,
   and toolchains — nothing else to install.
3. **Grant the sandbox access to your USB serial devices.** Without this step,
   PlatformIO inside VS Code cannot see the M2 at all:
   ```bash
   flatpak override --device=all com.visualstudio.code
   ```
   (Prefer a GUI? Use Flatseal → VS Code → enable "All devices".)
   Then fully quit VS Code (not just close the window) and reopen it.
4. **Add yourself to the `dialout` group** (serial-port access on the host):
   ```bash
   sudo usermod -aG dialout $USER
   ```
   Then **log out and back in** (group membership only takes effect on login).
5. **Stop ModemManager from grabbing the M2.** The M2 is a USB CDC-ACM
   serial device, and ModemManager probes every such device on plug-in —
   exactly the "connects, then the port vanishes mid-upload" symptom. The
   reliable fix on Bazzite is to disable it:
   ```bash
   sudo systemctl disable --now ModemManager
   ```
   This persists across reboots (`/etc` is writable on rpm-ostree). If you
   actually use a cellular modem on this machine, skip this step and rely on
   the udev rule in §1b instead.
6. Open the project: **File → Open Folder…** → select `m2-superb-firmware`.
   PlatformIO will download the M2 (SAM3X) and ESP32 toolchains on first
   build. This takes a few minutes once.

### Path B (alternative): `pio` CLI in a distrobox container

If you'd rather live in a terminal:

1. Create a container (shares your home dir, so the project is visible):
   ```bash
   distrobox create -n piobox --image fedora:42
   distrobox enter piobox
   ```
2. Inside the container:
   ```bash
   sudo dnf install -y python3-pip
   pip install platformio
   pio --version
   ```
3. **Device passthrough:** the container needs the M2's serial device. The
   device node (`/dev/ttyACM0`) appears and disappears as the M2 resets, so
   pass the whole device class, not one node. Create the container with:
   ```bash
   distrobox create -n piobox --image fedora:42 \
     --additional-flags "--device-cgroup-rule='c 166:* rmw'"
   ```
   (`166` is the `ttyACM` major number; this covers `ttyACM0`–`ttyACM31`.)
4. Steps 4–5 of Path A (dialout group, ModemManager) still apply — they are
   host-level, not container-level.

Use `pio run -e m2`, `pio run -e superb`, etc. from `~/workspace/m2-superb-firmware/`
inside the container.

---

## 1b. USB connectivity check (do this before first flash)

The M2 presents itself over USB as **VID 0x2341 / PID 0x003e** (verified in
Macchina's own `boards.txt`, `arduino-boards-sam` package). During upload,
bossac briefly resets it into the SAM-BA bootloader, where it re-enumerates
as **VID 0x03eb / PID 0x6124** (standard Atmel SAM3X ROM behavior, same as an
Arduino Due) — then returns to 2341:003e when the sketch runs.

Run the bundled diagnostics script; it checks every known Bazzite blocker and
tells you exactly what to fix:

```bash
./scripts/bazzite-usb-check.sh
```

What it checks: OS is Bazzite/atomic, you're in `dialout`, a udev rule exists
for the M2 (both PIDs), ModemManager is disabled or ignoring the M2,
`/dev/ttyACM*` is visible and writable, the M2 enumerates on `lsusb`, and
(if installed) the VS Code Flatpak has device access.

**Udev rule (if the script says it's missing):** create
`/etc/udev/rules.d/99-macchina-m2.rules` with:

```
# Macchina M2 — normal sketch mode (CDC-ACM serial)
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="003e", \
  GROUP="dialout", MODE="0660", ENV{ID_MM_DEVICE_IGNORE}="1"
# Macchina M2 — SAM-BA bootloader during bossac upload
SUBSYSTEM=="tty", ATTRS{idVendor}=="03eb", ATTRS{idProduct}=="6124", \
  GROUP="dialout", MODE="0660", ENV{ID_MM_DEVICE_IGNORE}="1"
```

Then reload: `sudo udevadm control --reload-rules && sudo udevadm trigger`.
`/etc` persists on rpm-ostree, so this survives reboots and updates. The
`ID_MM_DEVICE_IGNORE` line is belt-and-braces for step 5 above.

**Quick manual sanity check** (M2 plugged in via its micro-USB port):

```bash
lsusb | grep -iE "2341:003e|03eb:6124"   # should show one line
ls -l /dev/ttyACM*                        # should exist, group dialout, rw for you
pio device list                           # (or in VS Code: PIO Home → Devices)
```

If `lsusb` shows the M2 but `/dev/ttyACM*` doesn't exist, the Flatpak override
(Path A, step 3) or the container device rule (Path B, step 3) is missing —
re-do that step, then fully restart the app/container.

> **The SuperB has no USB of its own** — it is flashed through the M2's USB
> port with the SuperB seated in the Bee socket (§4). So "M2 connects"
> is the single gate for both firmwares. Get the M2's port stable first;
> the SuperB procedure then rides on that same port.

---

## 2. Open the project and build

1. In VS Code: **File → Open Folder…** → select `m2-superb-firmware`.
2. Click the PlatformIO icon in the left sidebar (the little alien head).
3. Under **Project Tasks**, you'll see the two environments: `m2` and `superb`.

**Build the M2 firmware** — click `m2 → General → Build`, or in a terminal:

```bash
pio run -e m2
```

**Build the SuperB firmware** — click `superb → General → Build`, or:

```bash
pio run -e superb
```

**Build both at once:** `pio run` (no `-e` flag), or just run both builds.

A successful build ends with something like:

```
RAM:   [          ]   2.6% (used 2600 bytes from 98304 bytes)
Flash: [          ]   2.2% (used 11412 bytes from 524288 bytes)
========================= [SUCCESS] =========================
```

Both environments compile cleanly as of Phase 0 — that was verified before this
guide was written.

---

## 3. Flash the M2

1. Plug the M2 into your computer with a micro-USB cable (the M2's own USB port).
2. Find the port:
   - **VS Code:** it usually auto-detects; if not, set it under
     `m2 → General → Upload` after picking the port, or add
     `upload_port = COMx` (Windows) / `upload_port = /dev/ttyACM0` (Linux) to the
     `[env:m2]` section of `platformio.ini`.
   - **Terminal:** `pio device list` shows connected serial ports.
3. Upload: click `m2 → General → Upload`, or:

```bash
pio run -e m2 -t upload
```

What happens: PlatformIO does the standard Due-style procedure — it briefly
opens the port at 1200 bps to reset the SAM3X into its bootloader, then
`bossac` writes the firmware. You don't need to press any buttons.

4. Open the serial monitor to see the banner: click the **Monitor** task, or:

```bash
pio device monitor -b 115200
```

(The M2's `Serial` in this firmware is the **Bee UART to the SuperB**, not USB.
Over USB you'll use `SerialUSB` in later phases. In Phase 0 the banner goes out
the Bee UART, so you won't see it on USB yet — that's expected.)

---

## 4. Flash the SuperB (read carefully — it's the fiddly one)

The SuperB has no USB port of its own. It is programmed **through the M2's
USB/serial connection**, with the SuperB seated in the M2's Bee socket. This
section is transcribed from Macchina's own documentation ("How to flash your
SuperB using either a USB adapter board or an M2") — I did not invent any step.
Canonical source, re-verified 2026-10-04 and matching this transcription:
https://github.com/macchina/docs/blob/master/superb-docs/flashing-superb.md

> Macchina's note: *"As of right now only the USB adapter board supports
> flashing through the ESP desktop tool; flashing through M2 can only be done
> through the Arduino IDE."*

### 4a. Hardware prep (M2 route)

1. Plug the SuperB into the M2's Bee socket **with the antenna trace / U.FL
   connector facing towards the M2's USB port**. Then plug the M2 into your computer.
2. With **"Macchina M2"** selected as the board, upload this passthrough sketch
   to the M2. It bridges USB ↔ Bee UART at 115200, and maps the M2's buttons to
   the ESP32's reset/bootstrapping pins:

```cpp
int buttonState1 = 0;
int buttonState2 = 0;

void setup() {
  pinMode(DS2, OUTPUT);
  pinMode(DS3, OUTPUT);
  pinMode(XBEE_RST, OUTPUT);
  pinMode(XBEE_MULT4, OUTPUT);
  pinMode(Button1, INPUT);
  pinMode(Button2, INPUT);
  SerialUSB.begin(115200);
  Serial.begin(115200);
}

char rx_byte = 0;

void loop() {
  if (SerialUSB.available() > 0) {
    rx_byte = SerialUSB.read();
    Serial.write(rx_byte);
  }
  if (Serial.available() > 0) {
    rx_byte = Serial.read();
    SerialUSB.write(rx_byte);
  }
  buttonState1 = digitalRead(Button1);
  if (buttonState1 == LOW) {
    digitalWrite(DS2, LOW);
    digitalWrite(XBEE_RST, LOW);     // holds ESP32 in reset (EN low)
  } else {
    digitalWrite(DS2, HIGH);
    digitalWrite(XBEE_RST, HIGH);
  }
  buttonState2 = digitalRead(Button2);
  if (buttonState2 == LOW) {
    digitalWrite(DS3, LOW);
    digitalWrite(XBEE_MULT4, LOW);   // pulls ESP32 IO0 low (bootloader entry)
  } else {
    digitalWrite(DS3, HIGH);
    digitalWrite(XBEE_MULT4, HIGH);
  }
}
```

### 4b. Bootloader mode (both routes)

To place the SuperB into bootloader mode: **hold down the BOOT button (SW2),
press and release the RESET button (SW1), then release the BOOT button.**
Electrically: IO0 must be held low during the EN rising edge.

- On the **M2 route**, BOOT = M2's **Button2**, RESET = M2's **Button1**
  (per the passthrough sketch above).
- On a **USB adapter board**, the board's own BOOT/RESET buttons do this
  (some adapters need VREF bridged to DIO11 on the back of the board first —
  see Macchina's docs photos).

### 4c. Software (M2 + Arduino IDE route, per Macchina)

1. Set up the ESP32 Arduino core in the Arduino IDE (Espressif's board manager).
2. Edit `platform.txt` in the ESP32 package (`…/packages/esp32/hardware/esp32/
   1.0.2/platform.txt` on Windows, `…/1.0.3/platform.txt` on macOS): replace
   `default_reset` with `no_reset` on lines 100 and 101.
   (The M2 can't auto-reset the ESP32, so the uploader must not try.)
3. Replace `boards.txt` with Macchina's custom one (adds a **"SuperB on M2"**
   board entry).
4. Restart the IDE. Open the serial monitor, put the SuperB in bootloader mode
   (§4b) — you should see the ESP32 ROM bootloader greeting.
5. Select **"SuperB on M2"** as the board and upload your sketch
   (e.g. File → Examples → Examples for SuperB on M2 → WiFi → WiFiScan).
6. When done, close the serial monitor, switch the board back to
   **"Macchina M2"**, reopen the monitor, and press RESET (SW1) to run the
   freshly flashed sketch.

### 4d. ⚠️ Honest caveats — read before you flash

- **This procedure is from ~2019** (ESP32 Arduino core 1.0.x era). The
  `platform.txt` line numbers, file paths, and Macchina's custom `boards.txt`
  are version-dependent and very likely stale against today's ESP32 core 3.x.
  Expect to adapt — and tell me what you hit so we can update this guide.
- **The PlatformIO equivalent of §4c is not yet verified.** In principle it's:
  M2 running the passthrough sketch + SuperB manually put in bootloader mode +
  an upload config that doesn't attempt auto-reset. I have *not* proven that
  recipe against hardware, so I'm not writing it here as instructions. This is
  an open item for the first hardware session.
- **Safer alternative if you have one:** an XBee USB adapter board. Macchina's
  docs cover flashing the SuperB that way with the ESP Flash Download Tool or
  the Arduino IDE ("ESP32 Dev Module" board). No M2 passthrough needed.
- Never unplug anything mid-upload. If an upload fails halfway, just put the
  SuperB back in bootloader mode and retry — a failed flash doesn't brick it.

---

## 5. What was verified (and what wasn't)

**Verified without hardware (Phase 0, this machine):**

- `pio run -e m2` compiles and links against the **real Macchina M2 variant**
  (`variants/m2/`, copied verbatim from Macchina's official
  `arduino-boards-sam` repo — the same source our pin audit verified).
  All 12 audited pins spot-checked against the variant: match.
- `pio run -e superb` compiles for ESP32-WROOM-32D (stock `esp32dev` board =
  4 MB flash default).
- The 115200 Bee-link rate is hardcoded, never parameterized, in both firmwares.

**NOT verified — needs hardware:**

- Flashing the M2 over USB (bossac procedure is standard Due-like, but unproven
  on this exact unit).
- Flashing the SuperB through the M2 (see §4d — the docs procedure is old).
- The Bee UART echo between the two chips.
- DS2 LED: Phase 1 removed the Phase 0 heartbeat blink (the mode manager owns
  the loop now); no LED is driven. INFERRED polarity remains unconfirmed.
- CAN, SWCAN, WiFi, Bluetooth, and the GVRET protocol are implemented in
  Phase 1 but **compile-verified only** — no bus traffic, button press, or
  radio behavior has been observed on hardware (user has no bench).
  See `docs/PHASE1_NOTES.md` for the INFERRED list.

---

## 6. Troubleshooting

- **`pio: command not found`** — Path B: you installed the CLI inside the
  distrobox container but are running in a host terminal (enter the container
  first: `distrobox enter piobox`). Path A: the VS Code extension doesn't put
  `pio` on your host PATH — use the VS Code tasks instead.
- **Upload can't find the port** — run `./scripts/bazzite-usb-check.sh`
  first; it diagnoses every known Bazzite blocker. Common leftovers: you
  skipped the log-out/in after joining `dialout`, the Flatpak device override
  needs a full VS Code restart, or a charge-only USB cable.
- **bossac fails / M2 doesn't reset** — try a different USB cable (some are
  charge-only), and make sure nothing else (Arduino IDE serial monitor, etc.)
  has the port open.
- **SuperB upload fails** — re-check bootloader mode (§4b): the most common
  cause is releasing BOOT before the EN rising edge. Watch the serial monitor
  for the ROM bootloader greeting before uploading.

---

## 7. Phase 1 usage (M2RET mode + SuperB bridge)

Phase 1 adds a real mode manager and the M2RET/GVRET implementation on the
M2, plus the Bluetooth/WiFi/TCP bridge on the SuperB. Build with plain
`pio run` (both environments), flash the M2 first, then the SuperB (§3, §4).

**M2 control console (USB serial, 115200):** the M2 boots into M2RET mode.
Type `help` for commands: `mode list`, `mode <name>`, `status`. Only `m2ret`
is implemented — the other six modes politely refuse. **Button1** (the M2's
button): short press cycles modes (no-op in Phase 1), long press (>1 s)
returns to M2RET. Every switch quiesces the buses first (CAN controllers
off, SWCAN transceiver to sleep, J1850 rail off).

**M2RET data plane:** GVRET binary protocol on the Bee UART (115200,
immutable) — this is what the SuperB bridges. The M2's USB port is the human
console only; it does not carry GVRET.

**SuperB bridge:** after boot it brings up Bluetooth SPP as `M2-SuperB`,
an open WiFi AP `M2-SuperB`, and a TCP server on port 23. Bytes flow
M2 ↔ SuperB ↔ app untouched (raw pipe).

**Connecting SavvyCAN (INFERRED — needs hardware to confirm):**
- WiFi: join the `M2-SuperB` AP, then SavvyCAN → new connection → Network
  Connection, host = the SuperB's IP (check your WiFi client list; typically
  192.168.4.1 on the AP), port 23.
- Bluetooth: pair `M2-SuperB`, then a SavvyCAN serial connection to the BT
  COM port at 115200.
- On connect, SavvyCAN sends `E7 E7 F1 0C F1 06 F1 07 F1 01 F1 09`; the M2
  answers with bus count (3: CAN0, CAN1, SWCAN), bus params, device info,
  time sync, and keepalive — then streams frames.

**Known Phase 1 limits (partly lifted in Phase 2):** GVRET settings (baud rates etc.) are RAM-only and
reset on reboot; concurrent BT+TCP clients are last-writer-wins; modes other than M2RET/ELM327 are
stubs. WPA2 on the AP and real STA credentials arrived in Phase 2 (see §8).
Full detail lives in `docs/PHASE1_NOTES.md`.

## 8. Phase 2 usage (ELM327 mode + web GUI)

**ELM327 mode on the M2:** switch via M2 Button1 (short press cycles
M2RET ↔ ELM327), the SerialUSB console (`mode elm327`), or the SuperB web
GUI. In ELM mode the Bee UART speaks the ELM327 AT protocol: Torque and
FORScan connect over Bluetooth SPP (`M2-SuperB`) exactly like a commercial
adapter — `ATZ`, `ATSP0`, `0100`, etc. Protocols: CAN 11/29-bit
(100/125/250/500k/1M via ATSP6/7/8/9/B/C), J1850 PWM (Ford) and J1850 VPW
(GM) via ATSP1/2. Anything the bus doesn't answer returns `NO DATA`.

**Web GUI:** join the `M2-SuperB` AP (WPA2 — default password `M2SuperB!`,
change it under WiFi on first use), then open `http://192.168.4.1/`.
Pages: Status (M2 mode, bus settings, network, clients), Mode (switch with
confirmation — buses are quiesced first), Settings (CAN0/CAN1 baud, SWCAN
speed, J1850/SWCAN toggles), WiFi (AP password, STA credentials; saving
restarts the SuperB). A JSON status API lives at `/api/status`.

**J1850 caveat (INFERRED):** the VPW/PWM driver is compile-verified only —
timings come from a vehicle-tested open-source library and published
references, but no vehicle test has confirmed them on this hardware. Expect
to tune polarities/windows on first contact with a real car; see
`docs/PHASE2_NOTES.md` §3.

**Known Phase 2 limits:** ELM auto-detect order is our own choice (6, 8, 7,
9, 2, 1); `ATRV` reports a placeholder 12.4V (no calibrated ADC yet); STN,
MITM, PROG modes are still stubs (UDS + OBD2 arrived in Phase 3, see §9);
the SuperB web GUI needs the M2 to answer within ~800 ms per request.

## 9. Phase 3 usage (UDS diagnostics + OBD2 poller)

**UDS mode on the M2:** switch via Button1 (now cycles M2RET → ELM327 →
UDS → OBD2), the SerialUSB console (`mode uds`), or the web GUI Mode page.
The console is a UDS terminal on SerialUSB:

- `uds 1003` — extended session; `uds 22 F190` — read VIN; `uds 19 02 FF` —
  all DTCs; `uds 14000000` — clear DTCs (group FFFFFF default).
- `uds bus 0|1`, `uds baud 500000`, `uds tx 7E0`, `uds rx 7E8`,
  `uds func|phys`, `uds tp on|off|<ms>`, `uds session`.
- `uds 11 01` (ECU reset) and `uds 2E …` (write DID) need
  **repeat-to-confirm** within 10 s — they never run on first invocation.
- `uds 27 01` shows the seed but there is **no key algorithm** (stub by
  design; OEM algorithms are proprietary).

Negative responses decode to text (`7F 22 31` → `requestOutOfRange`).
Transport is CAN ISO-TP 11-bit only in this phase.

**OBD2 poller on the M2:** switch to OBD2 mode, then `obd2 start` (or the
web GUI). The M2 auto-detects 500k/250k CAN, then streams CSV lines on the
Bee link — `ts_ms,01,pid,value` (e.g. `123456,01,0C,850`) — bridged live to
TCP:23 and Bluetooth. `obd2 pids 010C,010D,05`, `obd2 interval 500`,
`obd2 dtc` (decoded P0xxx scan), `obd2 stop`, `obd2 status`.

**Web GUI additions:** join the AP and open `http://192.168.4.1/` — new
**UDS** page (service sender with NRC decoding, session buttons,
tester-present toggle, addressing config; 0x11/0x2E need the confirm
checkbox) and **OBD2** page (PID editor, interval, start/stop, live CSV
view). The UDS page requires the M2 to be in UDS mode; the OBD2 page
requires OBD2 mode.

**Known Phase 3 limits (all INFERRED, no bench):** every bus behavior is
untested against real ECUs — session handling, tester-present cadence,
multi-ECU responses, the 500k→250k detect order. No flashing routines ship
yet (`0x34/0x36/0x37` are Phase 5, with a safety review first). See
`docs/PHASE3_NOTES.md` for the full INFERRED list.

## 10. Phase 4 usage (MITM bridge + STN command set)

**MITM mode on the M2:** switch via Button1 (now cycles M2RET → ELM327 →
STN → UDS → MITM → OBD2), the SerialUSB console (`mode mitm`), or the web
GUI Mode page. On entry the M2 begins **CAN0 and CAN1** (default 500k each)
and bridges them. Out of the box the rule table is empty = **pass-all-both-
directions**. Loop suppression (5 ms window) is always on — bridging two
taps of the *same* bus without it ping-pongs every frame forever.

Rule console on SerialUSB (works staged, even outside MITM mode):

- `mitm rule add 7E8 7FF both drop` — drop everything to/from 0x7E8
- `mitm rule add 7E0 7F0 a2b modify 3 FF 00 rate=100` — zero byte 3 of
  0x7E0–0x7EF frames A→B, max 100/s
- `mitm rule add 0 0 both log` — log *everything* to the web live view
- `mitm rule list` / `mitm rule del 0` / `mitm clear` / `mitm stats`
- `mitm dir a2b|b2a|both`, `mitm stealth on|off` (stealth = pure forward;
  CAN ACK is physical-layer and cannot be suppressed — see
  `docs/PHASE4_NOTES.md` §1.5)

**Web GUI:** join the AP, open `http://192.168.4.1/` — new **MITM** page:
live stats (polled), rule table with add/delete/clear, bridge config
(bauds, direction, stealth, *experimental* loop window + rate limits), and a
live view of LOG-action frames. The bridge runs only in MITM mode; rules
and config can be staged from any mode.

**STN mode on the M2:** `mode stn` (or Button1 / web GUI). It behaves like
ELM327 **plus** STN commands — every AT command keeps working:

- `STI` → `STN1170 v4.0.0`, `STDI` → `M2-SuperB r4.0` (this is how tools
  detect STN capability)
- `STPX h:7E0,d:0322F19000000000` — custom transmit; `STPX h:7E0,l:8`
  prints `DATA>` and takes the data on the next line
- `STM` / `STMA 100` — monitor (filters as set / all; `100` = auto-exit
  after 100 frames); `STMR 7E8` — filtered monitor
- `STCMM 0` (default) = **silent monitoring: the controller transmits
  nothing, not even CAN ACKs** (SAM3X listen-only mode, restored on exit)
- `STCFCPA 7E0,7E8` / `STCFCPC`, `STCAF`, `STCSEGR/T`, `STCSTM`, `STCTOR`

**Known Phase 4 limits (all INFERRED, no bench):** nothing here has touched
a real bus — loop-suppression tuning (5 ms), rate-limiter behavior, STCMM
listen-only ACK suppression, STPX edge cases. STPX is CAN-only (J1850 →
`?`); standalone `STSL`/`STCS`, `STWCF`, `STDP`/`STDPN` answer `?` (they are
not FRPM commands — the honest boundary is in `docs/PHASE4_NOTES.md` §2.3).
See `docs/PHASE4_NOTES.md` for the full INFERRED list and pen-test notes.

## 11. Phase 5 usage (PROG mode — ECU flashing)

> **READ `docs/SAFETY_REVIEW.md` FIRST.** It is the safety case for this
> mode, written before the code. The short version is on the `/prog` web
> page itself — it is impossible to miss, by design.

**PROG mode on the M2:** `mode prog` (or Button1 / web GUI Mode page).
ECU flashing/tuning over UDS on CAN: `0x10` programming session → `0x27`
security access (stub — you supply the key from your own tooling; no
algorithm exists here) → optional `0x31` erase routine → `0x34`
RequestDownload → `0x36` TransferData blocks streamed from the web upload
(one 41-byte chunk per block, no image RAM buffer) → CRC32 compare →
`0x37` RequestTransferExit → `0x11` reset. Optional `0x23` read-back
verify (ECU-dependent, INFERRED).

Entering PROG **auto-suspends the MITM rule table** (a bridge rewriting
flash bytes is a brick vector) — rules are cleared with an explicit
notice and are **never** silently restored.

**Console (SerialUSB):**

- `prog status` — state machine, config, arm/power, counters, CRCs
- `prog bus 0|1`, `prog baud <n>`, `prog tx <hex>`, `prog rx <hex>` —
  explicit addressing (required before destructive steps)
- `prog arm` — arm destructive steps (120 s); `prog power ok` — attest a
  stable 12V+ supply (**attestation, not a measurement** — required for
  erase/download)
- `prog session` → `0x10 0x02`; `prog seed <lvl>` → prints seed;
  `prog key <hex…>` → `0x27` sendKey (your key, armed)
- `prog erase <ridHex>` → `0x31` (armed + power)
- `prog download <addrHex> <lenHex> [alfidHex]` → `0x34` (armed + power),
  then stream chunks
- `prog verify <addrHex> <lenHex> [alfidHex]` → `0x23` read-back CRC
  compare (INFERRED availability)
- `prog done <fileCrc32Hex>` → CRC compare + `0x37` (armed)
- `prog reset` → `0x11 0x01` (armed); `prog abort` → abort, always available

**Web GUI:** join the AP, open `http://192.168.4.1/prog` — safety summary,
status, addressing config, per-step wizard (each destructive step has its
own confirm checkbox; erase/download also need the power attestation),
streaming firmware upload with progress bar, finalize (CRC compare +
`0x37`), verify, reset, and a big ABORT button. Live state at
`/api/prog`.

**Known Phase 5 limits (all INFERRED, no bench):** every ECU interaction
(sequencing, NRCs, timeouts: 5 s per `0x36` block, 30 s erase, 2 s
tester-present, 30 s chunk watchdog, 120 s arm); `0x23` availability;
`maxNumberOfBlockLength` parsing; upload throughput. First flash against
a **spare/bench ECU**, never the daily driver. See `docs/PHASE5_NOTES.md`
and `docs/SAFETY_REVIEW.md`.
