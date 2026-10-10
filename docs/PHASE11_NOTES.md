# Phase 11 — SuperB as communications appliance (2026-10-08)

The SuperB becomes a **communications appliance**: WebSocket transport,
single-owner multi-client policy, web authentication with roles mapped to
the Phase 9 capability gates, secure pairing with physical authorization,
signed OTA with anti-rollback, and a sleep/wake manager. The M2 keeps all
vehicle logic (standing architecture) — its only change is one new mm
query (`CMD_GET_VOLTAGE = 0x2E`) for the sleep manager.

Evidence labels: **VERIFIED** = compiles / cited from a primary source.
**INFERRED** = not tested on hardware (everything vehicle-facing;
pairing button path, sleep/wake, and OTA are untested on live silicon).

## 1. WebSocket transport (`ws::`, port 81)

- `WebSocketsServer` on **port 81** (port 80 stays the web GUI), via
  `links2004/WebSockets@^2.7.3` (added to `platformio.ini` lib_deps;
  API VERIFIED against the installed headers).
- **Binary frames = the raw byte stream** — the same bytes as TCP:23, so
  GVRET-capable apps can use `ws://host:81` the way they use TCP:23.
  Broadcast: every Bee-RX chunk goes to TCP + BT + all WS clients (RX is
  never gated — only TX is).
- **Text frames = tiny JSON control**: `{"t":"status"}`,
  `{"t":"claim"}`, `{"t":"release"}`, `{"t":"auth","pw":"..."}`.
  The parser is a minimal hand-rolled field extractor (no JSON library
  on the firmware).
- Binary TX follows the single-owner policy (§2) and — once auth is
  configured — needs role ≥ OPERATOR on that WS client (elevated via the
  text `auth` command, so raw-pipe apps never see a login prompt).

## 2. Multi-client ownership policy (`own::`)

- **One control owner** across TCP:23, BT SPP, and WS binary — the only
  client whose bytes reach the M2's command paths. All others are
  **passive monitors**: they receive the full broadcast, their TX is
  **dropped and counted** (`dropped_tx`, shown on `/clients`) — never
  silent.
- **Claim:** first TX wins when no owner holds it. **Release:** owner
  idle past the timeout (default 5 min, NVS `own_tmo`), owner
  disconnects (TCP slot close/evict, BT `hasClient()` drop, WS
  disconnect), or an operator+ releases it from `/clients`. **No silent
  preemption** — a connected owner is never bumped.
- `dropped_tx` counts **bytes** (not calls) on all three transports —
  the drop accounting lives in the callers, not in `txAllowed()`, so
  the metric is comparable across TCP/BT/WS.
- In-progress OTA/PROG uploads mark activity on every chunk, so the
  sleep manager cannot fire mid-transfer (a stalled upload stops
  marking and may sleep — correct, the transfer is dead anyway).
- The web UI's own mm commands are SuperB-originated and gated by web
  roles (§3) instead of taking ownership.
- Applied uniformly: `pumpClientToUart` (TCP, per-slot idx),
  `pumpBtToUart` (BT), WS `WStype_BIN`.

## 3. Authentication + roles (`auth::`, `pair_::`)

- Roles are the **remote half of the M2 capability gates** (SAFETY_REVIEW
  §13.1): `viewer` (1) = web read-only; `operator` (2) = diagnostics /
  config / PASS maps; `admin` (3) = PROG destructive, MODIFY maps, OTA,
  WiFi, auth, power. The M2 still enforces its own per-mode grants —
  the SuperB role only decides what leaves the SuperB.
- All POST handlers are role-gated: 35 `HTTP_POST` registrations
  (37 handlers counting the two upload DATA handlers) — 33 guarded, 0
  missed (verified by script + adversarial review). 32 via
  `requireRole()` at entry; `pair_set` via the pairing-window check.
  The 4 intentionally open: `auth_login` (the login endpoint itself),
  `auth_logout` (clears only the caller's session), `pair_set`
  (window-gated, not role-gated), and the `ota_upload` response
  renderer (the upload DATA handler refuses the stream at
  `UPLOAD_FILE_START`). GET pages and `/api/*` stay viewer-readable.
  The PROG upload stream likewise refuses at `UPLOAD_FILE_START`
  (the M2's arm state is the backstop, not the gate).
- Passwords: **salted SHA-256** (16-byte `esp_random()` salt per
  account); only salt + hash in NVS. Sessions: 64-bit tokens (16 hex
  chars from `esp_random()`), 4 slots, sliding 30-min expiry (NVS
  `sess_tmo`), `M2SB_SESS` HttpOnly cookie, RAM-only (no session
  survives reboot).
- **Pairing with physical authorization:** hold the M2's Button2 3 s at
  runtime (SuperB GPIO0 via Bee pin 7 — INFERRED runtime path) → 120 s
  window (LED fast-blinks) → `/pair` may set/reset the admin password.
  Before any admin exists, `/pair` is open but every page shows an
  **UNSECURED banner**. Do not hold Button2 across a SuperB reboot
  (GPIO0 low at boot = ESP32 download mode).
- **Unique device identity:** `M2SB-XXXXXX` from the eFuse MAC tail
  (`appl::devIdStr`), shown in the page header, `/api/status`, WS
  status, and logs. AP SSID / BT name stay `M2-SuperB` (stable for
  already-paired phones).
- **Honest limitation:** no TLS — password and cookie cross WiFi in the
  clear; the WPA2 passphrase is part of the trust boundary (documented
  in SAFETY_REVIEW §13.2).

## 4. Signed OTA with anti-rollback (`ota::`)

- **Key management** (standing rule: the private key NEVER lives in the
  firmware tree or any distributed zip):
  - `scripts/ota_keygen.py` → P-256 keypair; private key to
    `~/.ota-keys/superb_ota_priv.pem` (**0600, outside the project**);
    public key to `src/superb/ota_pubkey.h` (+ fingerprint
    `44c2763ef895d1ba`, shown on the `/ota` page).
  - `scripts/sign_ota.py` → package: `M2SBOTA1 | version u32 LE |
    image_len u32 LE | 64-byte ECDSA-SHA256 r||s | image`. VERIFIED:
    sign → self-verify → standalone verify all pass; a 1-byte-tampered
    package fails verification (exit 1).
- **Verify-then-commit:** upload streams through a running SHA-256 while
  `Update.write()` stages to the inactive OTA slot; the ECDSA signature
  is checked **before** `Update.end()` — bad signature ⇒
  `Update.abort()`, image never bootable, running firmware untouched.
- **Anti-rollback:** version must exceed NVS `ota_ver` (fresh NVS = 0).
- **Partitions:** `partitions/superb_ota_4m.csv` (replaces `huge_app.csv`,
  which had no OTA slot): nvs 20K, otadata 8K, **app0/app1 0x1E0000
  each (~1.9 MB)**, spiffs 192K. Current image: 1.68 MB / 85.5% of the
  slot — fits with headroom for growth. 4 MB flash is INFERRED (standard
  WROOM-32D; listed as an open audit item).
- **Recovery (documented on `/ota`):** USB reflash —
  `esptool.py --chip esp32 --port /dev/ttyUSB0 erase_flash`, then
  `pio run -e superb -t upload`. No automatic rollback; keep a
  known-good `.ota`.
- **Scope:** OTA updates the **SuperB only**. M2 OTA is out of scope
  (SAM-BA USB remains the M2 update path; a future phase would need a
  bootloader-level protocol, not a web upload).

## 5. Sleep/wake manager (`pwr::`)

- **Voltage source:** new M2 mm query `CMD_GET_VOLTAGE = 0x2E`
  (`req: [] → [status, mV LE32]`), following the F1 7E / `CMD|0x80` /
  LE supervision template. M2 side: `mmGetVoltageCb` reads V_SENSE
  (Arduino pin 92 = PA6/AD3; divider 0.18033 VERIFIED from the knowledge
  pack; `mV = raw × 446932 / 100000`). **INFERRED calibration** until
  per-unit measured (±4.8% worst-case resistor stack). Sim M2 returns a
  nominal 12600 mV (documented test double).
- **Policy:** engine-off = voltage < `v_off` (default 12.9 V) sustained
  `off_min` (default 10 min) **AND** no client TX / web POST for
  `idle_min` (default 15 min) → **light sleep**. Wake sources: 60 s
  timer (re-poll voltage), GPIO0 (Button2 press), UART0 RX (any M2 Bee
  traffic). Radios stop before sleep (`netDown`), re-init after
  (`pwr::netInit()` — the same function `setup()` calls).
- **Modem doze:** idleness alone (engine possibly on) ⇒
  `WiFi.setSleep(true)`; any activity ⇒ `setSleep(false)`.
- **No sleep-current claims** (pack rule — all currents unmeasured).
  All thresholds/timings are NVS-configurable on `/power` and INFERRED
  until in-vehicle testing.
- `/power` page: live voltage (or "M2 unreachable"), sleep state,
  last wake cause, config form (admin), manual "Sleep now" (admin).

## 6. Web UI additions

New pages (all in the existing `pageHead`/`pageFoot` style, nav
extended): `/auth` (login/logout/status), `/pair` (pairing), `/clients`
(owner, per-client table, drop counter, release), `/ota` (version,
key fingerprint, upload form, reboot, recovery docs), `/power`
(voltage, thresholds, config, manual sleep). `/api/status` gains
`dev_id`, `owner`, `dropped_tx`, `ws_clients`, `volt_mv`, `volt_ok`,
`ota_ver`, `auth` (additive — existing keys unchanged). WS `:81`
control protocol documented in §1.

## 7. Sim compatibility

- New shims: `WebSocketsServer.h` (no-op, API-matched incl. the real
  lib's non-const `String&`), `Update.h` (no-op), `mbedtls/sha256.h`,
  `ecp.h`, `ecdsa.h` + `mbedtls_stub.cpp` (zero-fill hash, always-valid
  verify — INFERRED, not crypto). `esp_random()` + `getEfuseMac()`
  added to the sim Arduino.h; `collectHeaders`/`header()` +
  numeric/bytes `Preferences` API added to the sim shims.
- Sim auth bypass: `requireRole()` always true under `SIM_SUPERB`
  (documented test-harness-only — no security boundary on loopback).
- Sim sleep: `pwr::enterSleep()` logs and skips (no real sleep).
- **VERIFIED:** `sim_superb` builds and runs — `/api/status` shows
  `dev_id: "M2SB-EEFF11"` (fake MAC tail), all five new pages return
  200, pairing → login → admin session works end-to-end over HTTP.
- `sim_m2` still has its **pre-existing** `SPI.h` failure (untouched,
  out of scope); the pytest smoke suite needs both sim binaries so it
  cannot run fully — noted, not worsened.

## 8. INFERRED list (untested on hardware)

- Pairing Button2→GPIO0 runtime read; 120 s window UX.
- Sleep/wake timings, voltage thresholds (12.9/13.2 V, 10/15 min),
  modem-sleep behavior, UART0 wakeup from light sleep.
- OTA applied to live partitions (compile + host crypto verified only).
- 4 MB flash assumption (partition table sizes).
- V_SENSE calibration (±4.8% resistor stack until measured).
- WS binary interop with real GVRET apps (protocol is byte-identical
  to TCP:23, but no app has connected yet).

## 9. Deferred (decisions, unchanged)

- **BLE GATT: not built** (Classic SPP serves all current apps; revisit
  on demand). No BLE code added.
- 0x27 stays a stub; OEM database not consumed; J1939 parked; ChatGPT F4
  deferred; Phase 10b LIN unapproved — none disturbed (verified:
  `src/m2/main.cpp`, `src/m2/debug.h` untouched; pin-safety
  `begin(baud, 255)` invariant holds by grep).

## 10. Key management (standing record)

- Private key: `~/.ota-keys/superb_ota_priv.pem`, mode 0600, created
  2026-10-08. **Never in the firmware tree, never in any zip** — the
  zip-building exclusions keep it out by location (it lives outside
  `~/workspace/m2-superb-firmware/` entirely).
- Public key: `src/superb/ota_pubkey.h` (generated, do-not-hand-edit),
  fingerprint `44c2763ef895d1ba`.
- Rotation: `ota_keygen.py --force` (old public key in already-flashed
  firmware will no longer verify — reflash needed); bump the version
  counter in `sign_ota.py` usage accordingly.

## 11. Adversarial review findings (fixed)

Per the standing practice (AGENTS.md), a separate reviewer audited the
build. Findings and fixes:

- **M1 (major): WS `{"t":"claim"}` had no role check** — an
  unauthenticated viewer could squat control ownership (their own TX
  was still dropped, but the real owner's TX dropped until the idle
  timeout — a control-plane DoS). Fixed: claim now requires role ≥
  OPERATOR, same as WS binary TX.
- **m2:** sleep could fire mid-OTA-upload (only `UPLOAD_FILE_START`
  marked activity). Fixed: both upload DATA handlers mark activity on
  every WRITE chunk (a stalled upload stops marking and may sleep —
  correct, the transfer is dead anyway).
- **m3:** `UPLOAD_FILE_ABORTED` unhandled — a client disconnect
  mid-upload left a begun `Update` that poisoned the next attempt
  ("already running"). Fixed: explicit ABORTED branch → `fail()`;
  START also defensively aborts a stale begun session.
- **m4:** `ota_ver` was written after `Update.end()` — a power loss in
  the window would boot the new image with a stale version, weakening
  anti-rollback. Fixed: version recorded before `end()`.
- **m5:** `netDown()` stopped TCP clients without releasing ownership
  — a dead client stayed owner until the idle timeout. Fixed:
  `own::onDisconnect()` per slot in `netDown()`.
- **m6:** evidence-label corrections — session tokens are 64-bit, not
  128-bit (fixed in both docs); handler count corrected to 33 guarded.
- **m7:** WS `{"t":"status"}` marked activity unconditionally — any
  associated client could pin the device awake. Fixed: only explicit
  actions (auth/claim/release/BIN) mark activity.
- Reviewer-corrected premises: the "LE32" doc (code was always LE16),
  `dropped_tx` units (bytes on all paths), and the guard count.

## 12. Build results (2026-10-08)

- `pio run -e m2`: **SUCCESS**, zero warnings from changed files
  (`mm_channel.*`, `mode_manager.cpp` — the MCP2515/USBCore warnings
  are pre-existing library noise).
- `pio run -e superb`: **SUCCESS**, zero warnings. Flash 85.5%
  (1,680,273 / 1,966,080 B); RAM 20.1%.
- `pio run -e sim_superb`: **SUCCESS**; runtime smoke of the sim
  binary passes (new pages + auth flow + `/api/status` fields).
- Adversarial review: per AGENTS.md standing practice, a separate
  reviewer pass should audit this phase before the zip goes to Marcus
  (recommended: focus on the OTA verify-then-commit ordering, the
  ownership edge cases, and the auth-bypass boundaries).
