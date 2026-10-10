# Safety Review — PROG (Programming) Mode (2026-10-04)

**Status: REQUIRED READING before any use of PROG mode.**
This document is the safety case for ECU flashing/tuning support in the
M2 multi-mode firmware. It was written **before** the PROG implementation
(`src/m2/mode_prog.*`), and the implementation is required to satisfy it.
Anything hardware-touching is **INFERRED** (no bench exists); nothing here
has been validated against a real ECU.

**Scope:** PROG mode performs UDS (ISO 14229-1) flash sequences over CAN:
`0x10` programming session → `0x27` security access → optional `0x31`
erase routine → `0x34` RequestDownload → `0x36` TransferData blocks →
`0x37` RequestTransferExit → `0x11` reset. The firmware is a **transport**:
it moves bytes the user supplies. It does not know what the bytes mean.

---

## 1. What can go wrong (threat model)

| Hazard | Mechanism | Severity |
|--------|-----------|----------|
| **Bricked ECU** | Power loss, bus disruption, or wrong image during `0x36` transfer; ECU left in programming session with partial flash | High — recovery may need bench/bootloader tools outside this project |
| **Wrong target** | Flashing the image into the wrong ECU (wrong CAN ID) or wrong memory address | High — silent corruption of a different controller |
| **MITM corruption** | A staged MITM `modify` rule rewrites flash payload bytes in transit | High — image corrupted *after* CRC check |
| **Voltage sag** | Flash write during cranking/low battery; charge-pump brownout mid-write | High — partial page write |
| **Session drop** | Tester-present lapse → ECU drops programming session mid-transfer | Medium — transfer aborts; ECU state depends on OEM bootloader |
| **Seed/key lockout** | Wrong key attempts → ECU `0x36 exceedNumberOfAttempts` lockout / delay | Medium — time delay, possible lockout |
| **App injection** | A connected app (Torque/SavvyCAN/pen-test tool) injects control frames | Low — mitigated by the F1 7E TX filter (Phase 2, VERIFIED by design) |

---

## 2. MITM interaction — the brick vector (rule, not advice)

A MITM `modify` rule that rewrites even one byte of a `0x36` TransferData
payload corrupts the flashed image **after** every checksum the tooling
computed. The CRC32 the M2 accumulates covers bytes *as sent to the CAN
controller*; a bridge sitting between the tool and the ECU can change them
afterwards. This is the single most dangerous interaction in the system.

**Rule (implemented, non-negotiable):**
- Entering PROG mode **automatically suspends the MITM rule table**:
  the table is cleared, and the count + a notice are printed on the
  SerialUSB console and shown on the `/prog` web page.
- Exiting PROG mode does **NOT** restore the rules. There is no silent
  restore path. The user re-adds rules deliberately via `mitm rule add`
  / the `/mitm` page.
- `CMD_PROG_STATUS` reports `mitmSuspended` (number of rules that were
  cleared on entry) so the web UI can display it honestly.

Why clear instead of "disable with restore": a restore-on-exit path is a
loaded gun pointed at the *next* flash session — one forgotten restore and
the corruption is invisible. Clearing forces a conscious decision.

---

## 3. Interlocks

PROG entry itself goes through the manager's safe-switch path
(`requestMode` → `quiesceBuses()` → acquire), like every mode. On top of
that, PROG has three interlocks. Each is enforced in firmware, not just
documented.

### (a) Explicit user confirmation

- **Console:** `prog arm` arms destructive steps for 120 s (INFERRED
  window — long enough for a human-driven flash sequence, short enough
  that a stale arm is unlikely). `prog key`, `prog erase`, `prog download`,
  `prog done`, `prog reset` **refuse** unless armed. `prog abort` never
  requires arm (aborting must always be available).
- **Web:** every destructive step on `/prog` has its own confirm checkbox;
  the SuperB sends `CMD_PROG_ARM` immediately before the action. No
  checkbox = no arm = the M2 refuses.
- **Raw path:** `mmTx` (the generic `CMD_PROG_TX` service sender) enforces
  the arm on the M2 side too for destructive services
  (0x11/0x2E/0x31/0x34/0x36/0x37) — defense in depth, so a future caller
  that forgets to arm is refused rather than trusted.
- **If skipped:** the command is refused with an explicit message
  ("not armed — run `prog arm` first"). There is no default-armed state;
  boot = disarmed.

### (b) No other mode's bus activity

- Entering PROG runs `quiesceBuses()`: native CAN controllers to
  init/off, SWCAN TH8056 to sleep, J1850 rail off — the car sees nothing
  during the switch (the manager's universal rule, Phase 1).
- The MITM rule suspension (§2) is part of PROG entry, not a separate step.
- **If skipped:** impossible through any supported path — `CMD_SET_MODE`
  dispatches exclusively through `requestMode()`. There is no backdoor.

### (c) Explicit bus + ECU addressing selection

- Destructive PROG commands require the `configured` flag, set only by an
  explicit `prog bus/baud/tx/rx` (console) or the `/prog` config form +
  `CMD_PROG_CFG` (web). Defaults (7E0/7E8) exist in the struct but are
  **not** trusted until explicitly confirmed.
- **If skipped:** refused ("addressing not configured — set bus/tx/rx
  first"). This guards against flashing the wrong ECU because "the
  defaults happened to answer."

---

## 4. Voltage — honest limitation

`ATRV` (ELM voltage read) is an **uncalibrated placeholder** in Phase 2 —
it is not wired to a calibrated measurement and must not gate anything.

**Therefore PROG does NOT gate on a voltage reading.** Instead:

- Console: `prog power ok` records explicit user attestation
  ("stable 12V+ supply confirmed").
- Web: the erase/download forms require a "stable 12V+ supply confirmed"
  checkbox, sent via `CMD_PROG_ARM` with the power bit.
- `prog erase` and `prog download` **refuse** without attestation.

**Limitation, stated plainly:** attestation is a human promise, not a
measurement. A sagging battery, a charger that drops out, or cranking the
starter mid-flash will still brick the ECU, and no firmware in this project
can prevent that. Flash on a stable bench supply or a known-good
charger/maintainer, engine off, and do not touch the ignition during the
transfer. A future hardware revision could add a calibrated ADC voltage
gate; until then, this is the honest ceiling.

---

## 5. Abort behavior

- **Any error mid-transfer** (NRC, timeout, CRC/sequence mismatch, chunk
  watchdog expiry) triggers the abort sequence: best-effort `0x37`
  RequestTransferExit if a download is open, chunk state cleared,
  disarmed, session state reset — and a **loud** report on the console
  (`[prog] ABORT: <reason>`) plus `state=ABORTED/ERROR` in
  `CMD_PROG_STATUS` for the web UI. Nothing about an abort is silent.
- **Chunk watchdog:** if no `CMD_PROG_DATA` chunk arrives for 30 s
  (INFERRED) while `DOWNLOADING`, the M2 aborts unilaterally. A stalled
  web upload must not leave the ECU hanging in an open transfer.
- **Power loss mid-flash is unrecoverable by firmware.** If the M2, the
  SuperB, or the ECU loses power during `0x36` transfer, the ECU may be
  left with a partial image. Recovery is outside this project's scope
  (OEM bootloader / bench tools). This is stated here so nobody learns it
  the surprising way.

---

## 6. Seed/key — stays a stub

`0x27` SecurityAccess is a **transport stub**, exactly as in Phase 3:

- `prog seed <level>` sends RequestSeed and **prints the seed**.
- `prog key <hex>` sends SendKey with **user-supplied** key bytes.
- **No OEM key algorithm is implemented, and none will ever be invented
  here.** The user computes the key with their own tooling (OEM
  documentation, licensed tools) and this firmware only carries the bytes.
- The web `/prog` page has a key hex field with the same stub warning the
  UDS page carries.

---

## 7. Data-path integrity design

- **Per-chunk CRC16** (CCITT-FALSE) on every `CMD_PROG_DATA` frame guards
  the Bee link. A CRC mismatch aborts the transfer — a corrupted chunk is
  never forwarded to the ECU.
- **In-order delivery:** chunks carry a sequence number; any gap or
  duplicate aborts. Each accepted chunk becomes exactly one `0x36`
  TransferData with an incrementing `blockSequenceCounter`; the ECU's
  `0x76` echo is verified against the sent counter (mismatch → abort).
- **End-to-end CRC32** (IEEE) is accumulated by the M2 over every byte
  sent in `0x36` blocks. `CMD_PROG_DONE` / `prog done` compares it against
  the file CRC32 computed by the SuperB during upload. Mismatch → abort,
  no `0x37`-as-success.
- **Optional read-back verify** (`prog verify` / web button): `0x23`
  ReadMemoryByAddress over the flashed region, CRC32 of the read-back
  compared against the transmitted CRC32. Availability is **INFERRED**
  (not all ECUs support `0x23` in programming session); absence is
  reported, not papered over.

---

## 8. What this review does NOT cover

- **No bench validation.** Every timing (NRC windows, chunk watchdog,
  tester-present interval, abort sequencing against a real bootloader) is
  INFERRED until in-vehicle testing. The first real flash should be
  against a **spare/bench ECU**, never the daily driver.
- **No target-specific guidance.** This project deliberately contains no
  ECU-specific flash procedures, memory maps, seed/key algorithms, or
  "how to tune X" instructions. Those live with OEM documentation and
  licensed tooling.
- **STN/ELM/M2RET modes** are unaffected by this review except via §2.

---

## 9. K-line init transmit authority (Phase 7)

**Scope:** ELM/STN protocols 3/4/5 (ISO 9141-2, KWP2000 5-baud, KWP2000
fast init) on the M2's K-line (TJA1027 U7, OBD-II pin 7). This section is
the safety case for the one new transmit behavior Phase 7 adds.

### What the init sequences do on the wire

Both inits **transmit on the vehicle bus by construction** — there is no
listen-only way to start a K-line session:

- **5-baud init:** the tester drives the K-line dominant for a full
  2-second bit-banged `0x33` byte at 5 baud (200 ms/bit), then performs
  the keyword handshake (`0x55` sync, KB1/KB2, inverted-KB2,
  inverted-address) at 10400 baud.
- **Fast init:** the tester drives a 25 ms dominant / 25 ms recessive
  wake-up pattern, then sends `StartCommunication` (`C1 33 F1 81`) at
  10400 baud and reads the keyword bytes from the positive response.

After init, all K-line traffic is **request/response only** (OBD-II
`01 xx` polling, KWP2000 services). The driver implements no flash,
erase, actuator, or routine-control services; KWP2000 `0x27`
SecurityAccess is not implemented on the K-line path (the stub policy
from §6 applies everywhere).

### The gate

Init runs **only** when all three hold:

1. **ELM327 or STN mode is active.** Console `kline init`/`kline tx`
   and `CMD_KLINE_INIT` refuse in any other mode. The USB console is a
   physical-access interface; the mm channel is reachable only via the
   SuperB's authenticated web path.
2. **User-initiated diagnostic activity.** In ELM/STN, init runs from
   `busActivate()`, which is reachable only via `ensureProtocol()` —
   i.e. the user typed an OBD request, `ATTP n`, `ATMA`, or the
   equivalent app action. There is no background or timer-driven init:
   K-line is deliberately **excluded** from `ATSP0` auto-detect (init is
   slow and transmits; auto-probing would chatter the bus).
3. **No conflicting J1850 session.** `kline::initSlow/initFast` refuse
   while the J1850 driver holds its block (defense in depth; ELM's
   `busRelease()` already sequences protocol switches).

This is the **same gate as ELM's existing CAN/J1850 OBD commands** —
no new arm mechanism was added, because ELM/STN is already a
transmit-capable diagnostic mode by design: the user points it at a
vehicle deliberately, and every transmission is a direct consequence of
a command they issued. What Phase 7 does *not* do is expand *who* may
transmit or *what* may be sent: no new modes gained transmit authority,
and no destructive K-line service exists to be gated.

### Shared-rail behavior (why this is in the safety review)

The K-line transceivers share the `J1850 9141 ON` power net (pin 49)
with J1850. The new `rail9141` module refcounts the two owners:

- Pin 49 goes LOW **only when both J1850 and K-line are quiesced**
  (pack rule INV-PWR49) — a J1850 release can no longer brown out an
  active K-line session, and vice versa.
- Pin 50 (J1850 mode select) is settled **before** pin 49 goes HIGH
  (INV-PWR49/INV-MODE50); this also fixed the Phase 2 ordering, which
  powered the rail before settling the mode pin.
- `quiesceBuses()` releases both drivers on every mode switch, so the
  vehicle never sees a half-powered transceiver mid-switch.

### Honest limitations

- Every K-line timing (5-baud bit time, Twup, P1–P4, echo behavior,
  SLP_N polarity, the GPIO↔USART mux switchback for bit-banging) is
  **INFERRED** — the available test vehicle (2023 Chevy Bolt EV) is
  CAN-only, so no K-line vehicle test is possible yet. The first K-line
  test must be against a K-line vehicle with `debug on`, watching the
  init handshake bytes before trusting any higher-level behavior.
- `0x23 ReadMemoryByAddress` availability is ECU-dependent (INFERRED);
  absence is reported, not papered over.

## 10. Capability gates (Phase 9)

Before Phase 9, transmit authority was informal: an arm-confirm on a few
services, a safe-mode switch, and per-mode convention. Phase 9 formalizes
it as **capability gates** — explicit, per-mode grants enforced in code at
every transmit path (console, mm channel, and mode internals).

### 10.1 The grant levels

Grants are hierarchical: each level includes everything below it.

| Level | Grant | Unlocks |
|---|---|---|
| 0 | `READ_ONLY` | Listening only: monitor/sniff, receive, decode. The default everywhere unless a mode says otherwise. |
| 1 | `ALLOW_TRANSMIT` | Any transmit that doesn't change ECU state: OBD reads (01/02/05/06/07/09/0A), UDS 0x10/19/22/23/27/3E, 0x31 requestRoutineResults, discovery probes, K-line init, tester-present. |
| 2 | `ALLOW_DIAGNOSTIC_WRITE` | State-changing diagnostics: UDS 0x11 (reset), 0x14 (clear DTCs), 0x28 (comm control), 0x2E (write DID), 0x31 startRoutine/stopRoutine, 0x85 (DTC setting), 0x86 (response on event), OBD 04 (clear), OBD 08 (control). |
| 3 | `ALLOW_ACTUATOR_CONTROL` | UDS 0x2F (InputOutputControlByIdentifier) — drives physical actuators. |
| 4 | `ALLOW_FLASH_PROGRAMMING` | UDS 0x34/0x36/0x37 (transfer/download). PROG mode only in practice. |

### 10.2 Default grant per mode

| Mode | Default grant | Why |
|---|---|---|
| M2RET | `ALLOW_TRANSMIT` | Pass-through bridge; apps transmit by design. |
| ELM327 | `ALLOW_DIAGNOSTIC_WRITE` | User-driven terminal; Torque etc. legitimately clear DTCs (OBD 04). Gated 04/08 enforced against the grant. |
| STN | `ALLOW_DIAGNOSTIC_WRITE` | Same as ELM (shared handler). |
| UDS | `ALLOW_TRANSMIT` | Raw service access is broader than ELM's OBD subset; writes need explicit escalation. |
| MITM | `READ_ONLY` | Stealth default. `A_MODIFY` rule adds require `ALLOW_TRANSMIT` (escalate first). Ceiling: MITM can never exceed `ALLOW_TRANSMIT`. |
| OBD2 | `ALLOW_TRANSMIT` | Poller is read-only; 04/08 need `ALLOW_DIAGNOSTIC_WRITE`. |
| PROG | `ALLOW_FLASH_PROGRAMMING` | Flashing is the mode's purpose; arm + powerOk + configured interlocks still apply. |
| GATEWAY | `ALLOW_TRANSMIT` | Originates translated frames by construction (§12). PASS mappings need the default; MODIFY mappings need `ALLOW_DIAGNOSTIC_WRITE` + arm-confirm. Ceiling: `ALLOW_DIAGNOSTIC_WRITE`. |

### 10.3 Enforcement points

- **UDS mode** (`mode_uds.cpp::grantFor`): every `sendRaw` (console) and
  `mmTx` (mm channel) maps the service byte to its minimum grant and
  refuses with a message when the grant is missing. 0x11/0x2E keep their
  arm-confirm as defense-in-depth *after* the gate passes.
- **ELM/STN** (`mode_elm.cpp::obdRequest`): OBD services 04 and 08 refuse
  with `?` unless `ALLOW_DIAGNOSTIC_WRITE` is held.
- **OBD2 mode**: `obd2 clear` (04) and `obd2 ctl` (08) refuse unless
  `ALLOW_DIAGNOSTIC_WRITE` is held.
- **MITM** (`mode_mitm.cpp`): `A_MODIFY` rule adds (console and mm) refuse
  unless `ALLOW_TRANSMIT` is held. Pass/drop/log rules are receive-side
  and stay available at `READ_ONLY`.
- **PROG**: unchanged interlocks (arm, powerOk, configured, CRC); the
  mode's default grant already covers 0x34/0x36/0x37.
- **GATEWAY** (`mode_gateway.cpp::addFromPayload`): PASS mapping adds
  require `ALLOW_TRANSMIT`; MODIFY (payload-fabricating) mapping adds
  require `ALLOW_DIAGNOSTIC_WRITE` *and* the arm-confirm
  (repeat-identical within 10 s, same mechanism as UDS `confirmGate`).
  Full safety case in §12.

### 10.4 Escalation — never silent

- Grants are **reset to the mode default on every mode entry**
  (`capgate::enterMode` in `ModeManager::requestMode` and `begin`).
  Escalation never survives a mode switch, a reboot, or a reflash.
- Escalation is **upward-only and explicit**: `cap grant <0..4>` on the
  USB console, or `CMD_CAP_GRANT` (0x28) on the mm channel (`[level]`;
  `[0xFF]` queries). A ceiling per mode prevents over-escalation (MITM
  caps at `ALLOW_TRANSMIT`). Every escalation prints a console warning;
  refusals print why.
- There is deliberately **no downgrade command**: the only way to shed a
  grant is to switch modes (or reboot).

### 10.5 What the gates do NOT do

- They do not authenticate the *user* — anyone with the USB console or
  the mm channel can escalate. They formalize *intent*: a read path can
  never accidentally become a write path, and a 0x2E/0x31/flash command
  cannot be issued without a deliberate, visible escalation step.
- They do not replace the PROG interlocks (§3) or the 0x11/0x2E
  arm-confirm; those stay as defense-in-depth.

## 11. Dual-use statement

This is ECU flashing tooling. It is built as a **transparent, auditable
transport** for the owner's own vehicles and bench ECUs: every destructive
step requires explicit confirmation, every interlock is enforced in code
(not documentation), failures are loud, and the firmware never invents
security material. Keep it that way — do not add target-specific bypasses,
do not weaken the interlocks for convenience, and do not remove the
notices.

## 12. Gateway mode transmit authority (Phase 10)

**Scope:** the GATEWAY mode (`src/m2/mode_gateway.*`) translating frames
between HS-CAN (CAN0) and SWCAN (GM-LAN) per a user-defined, versioned
mapping table.

### Why this is different from MITM

MITM's safety story is "never originates frames" — it only forwards what
it received on the same protocol, and CAN ACK is physical-layer. **None
of that transfers to the gateway.** A gateway originates frames *by
construction*: every crossing re-emits the frame with a new ID (and
optionally fabricated payload bytes) on a different bus, at a different
bitrate, through a different controller. There is no transparent forward.
Treat the gateway as **ALLOW_TRANSMIT minimum** (the mode default), with
payload fabrication gated higher — not as a stealth bridge.

### What the gateway transmits and when

- Forwarding runs **only** in GATEWAY mode, only while the mapping table
  is non-empty and not suspended, and only for frames matching a mapping
  (first match wins; unmatched frames are never forwarded).
- PASS mappings re-emit the frame with a remapped ID (payload unchanged).
- MODIFY mappings additionally rewrite one payload byte
  (`data[off] = (data[off] & ~mask) | (val & mask)`) — payload
  fabrication, see the gate below.
- TX queues are paced (drain cap per loop, default 4); SWCAN ID 0x100
  goes out via the high-speed hop (GMLAN quirk, same as M2RET).

### The gates

1. **Mode default `ALLOW_TRANSMIT`.** PASS mapping adds (console/mm)
   require it — the mode holds it by default, so a deliberate mapping
   add is the intent signal. Escalation ceiling is
   `ALLOW_DIAGNOSTIC_WRITE`.
2. **MODIFY ("write") mappings: `ALLOW_DIAGNOSTIC_WRITE` +
   arm-confirm.** Adding a payload-fabricating mapping requires the
   escalated grant *and* the repeat-identical-command-within-10 s
   arm-confirm (the same mechanism as UDS `confirmGate`). On the mm
   channel the first call stages (idx `0xFE`); the identical call within
   10 s installs. The web UI renders an explicit Confirm page. Note the
   grant is evaluated in the *ambient* mode (mapping config works from
   any mode by design, like MITM rules): from ELM/STN the default grant
   already covers it, so the arm-confirm's explicit WARNING + identical
   repeat carries the intent friction there — consistent with §10.5's
   threat model (intent, not authentication).
3. **PROG-entry auto-suspend.** Entering PROG clears the mapping table
   with an explicit notice (same brick-vector reasoning as the MITM rule
   suspension, §2: a translating gateway during a flash could corrupt the
   transfer). There is **no silent restore** — re-adding mappings
   (console, mm, or web UI) is the explicit re-enable and clears the
   suspended flag. `gateway::loop()` also refuses to forward while the
   suspended flag is set (defense in depth).
4. **No other mode is affected.** The MITM rule engine is untouched; the
   gateway has its own data model. PROG's existing interlocks (§3) are
   unchanged.

### Honest limitations

- Every gateway behavior is **INFERRED** — no GM-LAN vehicle exists here
  (the Bolt EV is CAN-only), so forwarding, the 15:1 overflow policy, and
  the 0x100 high-speed hop are untested on hardware.
- The overflow policy drops frames by design (drop-oldest default);
  loss counters (`busstat` dropped + per-mapping drops) make every drop
  visible — but a dropped frame is still a lost frame, and the gateway
  does not know whether it was safety-relevant. Size the policy for the
  use case; do not gateway a bus you cannot afford to lose frames on.
- Cyclic mappings (A→B and B→A covering the same IDs) can ping-pong
  frames. There is no automatic loop suppression in Phase 10 — do not
  configure cycles.

## 13. SuperB appliance: roles, pairing, OTA (Phase 11)

Phase 11 turns the SuperB into a communications appliance: WebSocket
transport, a single-owner transmit policy, web authentication with roles,
signed OTA, and a sleep/wake manager. The M2 keeps all vehicle logic;
the SuperB is the **remote half** of the Phase 9 capability gates (§10) —
it decides what a remote client may even *attempt*, while the M2 still
enforces its own per-mode grants on everything it receives.

### 13.1 Role → grant mapping

Web clients authenticate to a role; the role gates what the SuperB will
forward or issue. Unauthenticated web clients are **VIEWER**.

| SuperB role | M2 grant it mirrors | Web powers |
|---|---|---|
| `viewer` (1) | `READ_ONLY` | GET pages + `/api/*` only. Every POST refused. |
| `operator` (2) | `ALLOW_TRANSMIT` | Diagnostics and config: mode switch, settings, UDS/OBD2/MITM, gateway PASS maps, release control. |
| `admin` (3) | `ALLOW_DIAGNOSTIC_WRITE` and up | Everything: PROG destructive steps, gateway MODIFY maps, WiFi credentials, OTA, auth management, power config. |

Two deliberate asymmetries, both defense-in-depth rather than holes:

- The M2's own gates still apply. A web `operator` asking for UDS 0x2E
  still needs the M2's `ALLOW_DIAGNOSTIC_WRITE` + arm-confirm; the
  SuperB role only decides whether the request leaves the SuperB.
- Raw pipes (TCP:23 / BT SPP / WS binary) have **no app-layer login** —
  their perimeter is WPA2 / BT pairing plus the single-owner policy
  (§13.3). A raw-pipe client can only reach what the M2's gates allow
  for the current mode; the web roles do not weaken that.

### 13.2 Pairing trust model

- Passwords are **salted SHA-256** (16-byte random salt per account,
  `esp_random()`); only salt + hash live in NVS — never the password.
- Sessions are 64-bit tokens (16 hex chars from `esp_random()`),
  4 slots, sliding 30-minute expiry (NVS-configurable), carried in the
  `M2SB_SESS` HttpOnly cookie. No session survives a reboot (RAM only).
- **First admin / password reset requires physical authorization**:
  hold the M2's Button2 for 3 s at runtime (SuperB GPIO0 via Bee pin 7)
  to open a 120 s pairing window; `/pair` only accepts changes inside
  the window (or before any admin exists — setup mode).
- Before any admin exists, the web UI is open but every page carries an
  **UNSECURED banner** — the device is exactly as exposed as it was in
  Phase 10 until pairing completes. Pairing is a setup step, not an
  afterthought.
- **Honest limitations (INFERRED):** the Button2→GPIO0 runtime path is
  untested on hardware; the pairing window has no brute-force
  rate-limiting beyond the 120 s window itself; there is no TLS — the
  password and session cookie cross the WiFi link in the clear, so the
  WPA2 passphrase is part of the trust boundary.

### 13.3 Single-owner transmit policy

One control owner across TCP:23, BT SPP, and WS binary: the only client
whose bytes reach the M2's command paths. Everyone else is a passive
monitor (receives the full broadcast; TX is dropped, **counted**
`dropped_tx`, and visible on `/clients` — never silent).

- **Claim:** first TX wins when no owner holds it.
- **Release:** owner idle past the timeout (default 5 min,
  NVS `own_tmo`), owner disconnects, or an operator+ releases it from
  the web UI. There is **no silent preemption** — a connected owner is
  never bumped.
- The web UI's own mm commands are SuperB-originated and gated by web
  roles instead (§13.1); they do not take ownership.

### 13.4 What an unauthenticated client can / can't do

| Client | Can | Can't |
|---|---|---|
| Web, no login | View every page and `/api/*` (read-only). | Any POST (mode, config, PROG, OTA, WiFi, power, auth). |
| TCP:23 / BT / WS, no owner held | Receive all broadcast traffic (monitor). | Transmit while another client owns control (bytes dropped + counted). |
| TCP:23 / BT / WS, owner | Transmit to the M2. | Exceed the M2's per-mode capability gates (§10); preempt another owner. |

### 13.5 Signed OTA — verification, anti-rollback, recovery

- **Sign offline:** `scripts/ota_keygen.py` creates the P-256 keypair;
  the **private key lives at `~/.ota-keys/` (outside the firmware tree,
  mode 0600)** and is never in the repo or any zip. `scripts/sign_ota.py`
  packages `firmware.bin` as `magic "M2SBOTA1" | version u32 LE |
  image_len u32 LE | 64-byte ECDSA-SHA256 r\|\|s | image`.
- **Verify before commit:** the SuperB streams the upload through a
  running SHA-256 while staging bytes to the inactive OTA partition;
  the ECDSA signature (against the baked-in public key,
  `src/superb/ota_pubkey.h`) is checked **before** `Update.end()` — a
  bad signature calls `Update.abort()` and the image never becomes
  bootable. The running firmware is untouched by a failed update.
- **Anti-rollback:** version must exceed the NVS-stored `ota_ver`
  (fresh NVS = 0); older or equal versions are rejected with a message.
- **Recovery:** if an update ever bricks the SuperB, reflash over USB:
  `esptool.py --chip esp32 --port /dev/ttyUSB0 erase_flash`, then
  `pio run -e superb -t upload`. There is no automatic rollback — the
  failed image stays in its slot until replaced. Keep a known-good
  `.ota` on hand.
- **Scope:** OTA updates the **SuperB only**. M2 OTA over the Bee link
  is out of scope for Phase 11 (the M2's SAM-BA USB path remains the
  update route; a future phase would need a bootloader-level protocol,
  not a web upload).
- **Honest limitation (INFERRED):** OTA has been compile-verified and
  the package format + signature round-trip tested on the host, but no
  OTA has been applied to live hardware yet.

### 13.6 Privacy: what leaves the device vs what stays local

- **Broadcast:** WiFi AP SSID `M2-SuperB`, BT name `M2-SuperB`, and the
  device identity `M2SB-XXXXXX` (MAC-derived, shown in the UI/logs/API).
  These are visible to anyone in radio range — the identity exists to
  disambiguate multiple units, not to hide them.
- **Stays local:** password salts/hashes (NVS), session tokens (RAM),
  the OTA private key (never leaves Marcus's machine), vehicle voltage
  readings (used for sleep decisions, never transmitted off-device).

## 14. Auto-detect transmit gating + loopback-harness confinement (Phase 12)

### 14.1 Auto-detect: explicit initiation only, layered transmit gates

The auto-detect engine (`src/m2/autodet.*`) is the first firmware
component whose *job description* includes transmitting on buses it has
not identified yet. The CAN stage is listen-only (due_can
`setListenOnlyMode`, zero transmit); the K-line init and J1850/SWCAN
active stages transmit by construction. The gating is layered:

1. **Explicit user initiation only.** `autodet::start()` is reachable
   from exactly two places: the USB console (`autodetect start`) and
   the mm channel (`CMD_AUTODET_START`, itself behind the SuperB's
   operator role). It never runs on boot, never in the background, and
   no mode invokes it as part of normal operation.
2. **Capability gate first.** The engine requires
   `capgate::ALLOW_TRANSMIT` before doing anything — even the
   listen-only CAN sweep. The gate is mode-aware and upward-only, and
   the grant resets on every mode entry (§10).
3. **Separate TX consent for the transmitting stages.** The K-line /
   J1850 / SWCAN active stages require a *second* explicit decision:
   the `tx` word on the console, or the `allowTx` byte on the mm
   command (the SuperB web UI renders it as a separate consent
   checkbox). Without it the engine runs CAN listen-only plus passive
   J1850/SWCAN observation only and says so.
4. **Bus takeover, then restore.** The engine takes over the buses
   (manager `quiesceBuses()` first, so no mode's loop fights it for
   the controllers), runs blocking, and re-enters the current mode
   afterwards — the mode's ownership and the capgate default are
   restored by the normal switch path, never left in a half-owned
   state.

Residual risk (INFERRED, documented): a transmitting probe on the wrong
bus is still a probe. The gates above make it *deliberate*, not
accidental — the user asked for it twice and the firmware never does
it on its own.

### 14.2 Harness: loopback-only confinement, enforced in code

The virtual-ECU harness (`src/m2/vharness.*`) exists to validate the
INFERRED CAN behaviors (ISO-TP reassembly, N_Cr/N_Bs timeouts, sequence
errors) without a vehicle. It must never run against a vehicle bus:

- It is a dedicated HARNESS mode (explicit `mode harness`), entered
  through the normal quiesce-before-switch path.
- `runSuite()` **refuses** unless `selfcheck()` passed in the current
  session: the harness sends marker frames CAN0→CAN1 and CAN1→CAN0 and
  requires both to arrive. A vehicle bus will not echo our markers, so
  the suite cannot start against one. This is a heuristic gate,
  documented as such — not a cryptographic proof of loopback.
- The ECU simulator only answers the scripted test IDs (0x7E0) and
  never initiates traffic on its own.

### 14.3 Log privacy

Diagnostic logs may contain VINs and vehicle data. The design keeps
them local by construction: records live in the M2's RAM ring, cross
the Bee link on poll, and persist to the SuperB's SPIFFS; the web UI
offers download in native/GVRET/CRTD/CSV. There is no upload path —
nothing leaves the device except over the local web download the user
explicitly requests.
