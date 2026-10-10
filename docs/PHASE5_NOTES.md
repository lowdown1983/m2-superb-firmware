# Phase 5 Notes — PROG (Programming) Mode (2026-10-04)

Multi-mode firmware, `~/workspace/m2-superb-firmware/`. M2 = Arduino SAM3X8E
(Macchina M2), SuperB = ESP32-WROOM-32D. PlatformIO, envs `m2` + `superb`.

**Build status:** both envs compile green, **zero project warnings**
(remaining warnings are pre-existing third-party: MCP2515 lib, Arduino
framework USBCore, platformio.ini `src_filter` deprecation). No bench —
everything hardware-touching is **INFERRED** until in-vehicle testing.

**What this phase adds:**

1. **PROG mode** (`src/m2/mode_prog.h/.cpp`) — ECU flashing/tuning over
   UDS (ISO 14229-1) on CAN ISO-TP: 0x10 programming session → 0x27
   security access (stub) → optional 0x31 erase → 0x34 RequestDownload →
   0x36 TransferData (streamed) → 0x37 RequestTransferExit → 0x11 reset,
   plus optional 0x23 read-back verify.
2. **`docs/SAFETY_REVIEW.md`** — the safety case, written BEFORE the
   code: MITM brick-vector rule, the three interlocks, the voltage
   limitation, abort behavior, seed/key stub policy, dual-use statement.
3. **mm_channel** — CMD 0x11–0x19 (PROG supervision + chunked data path).
4. **SuperB web GUI** — `/prog` page: safety summary on the page itself,
   config, per-step wizard with confirms, streaming file upload with
   progress, finalize/verify/reset/abort, `/api/prog`.
5. **`docs/OVERVIEW.md`** — the whole seven-mode system in one place.

---

## 1. Safety review (read first, implemented second)

`docs/SAFETY_REVIEW.md` is a first-class deliverable, not an appendix.
The implementation satisfies it point by point:

| Review § | Rule | Implementation |
|----------|------|----------------|
| §2 MITM brick vector | PROG entry auto-suspends the MITM rule table; exit never restores | `mode_manager.cpp enterProg()` → `mitm::suspendRules()` (clears + counts + notices); `prog::begin(suspended)` prints the notice; `CMD_PROG_STATUS` reports `mitmSuspended` |
| §3a confirmation | Destructive steps need arm | Console `prog arm` (120 s INFERRED window); web per-step checkbox → `CMD_PROG_ARM`; `prog abort` / `CMD_PROG_ABORT` never need arm |
| §3b quiesce | No other mode's bus activity | `requestMode()` → `quiesceBuses()` (universal); MITM suspend is part of entry |
| §3c addressing | Explicit bus/ECU selection | `configured` flag set only by explicit `prog bus/baud/tx/rx` or `CMD_PROG_CFG`; reset on every PROG entry; destructive steps refuse without it |
| §4 voltage | ATRV gates nothing; attestation instead | `prog power ok` / `CMD_PROG_ARM` power bit; `erase`/`download` refuse without it; limitation documented |
| §5 abort | Loud, never silent; 0x37 best-effort; chunk watchdog | `abortTransfer()` (console + status); 30 s chunk watchdog (INFERRED); power-loss statement |
| §6 seed/key | Stub forever | Seed printed, user supplies key, no algorithm; web page carries the warning |
| §7 integrity | CRC16/chunk, CRC32 end-to-end, seq check, 0x76 echo | `feedChunk()`; `mmDone()` compares before 0x37 |

---

## 2. PROG mode design (`src/m2/mode_prog.*`)

### 2.1 State machine

`IDLE → SESSION → UNLOCKED → ERASED? → DOWNLOADING → STREAM_DONE → DONE`,
with `ABORTED` / `ERROR` as loud terminal states. Reported via
`CMD_PROG_STATUS` and `prog status`.

### 2.2 Service flows (ISO 14229-1 semantics — cited, never invented)

- **0x10 0x02** — DiagnosticSessionControl/programmingSession. Positive
  `0x50` adopts the session and auto-enables tester-present (`0x3E 0x00`
  every 2000 ms, INFERRED) for the duration.
- **0x27** — `prog seed <lvl>` sends RequestSeed and prints the seed;
  `prog key <hex>` sends SendKey with **user-supplied** bytes. No key
  algorithm exists and none is invented (OEM algorithms are proprietary).
- **0x31 0x01 <rid>** — RoutineControl/startRoutine (erase). 30 s timeout
  (INFERRED — erases are slow).
- **0x34** — RequestDownload `[0x34, dfi=0x00, alfid, addr, size]`
  (ISO 14229-1: dfi 0x00 = uncompressed/unencrypted; alfid low nibble =
  address bytes, high nibble = size bytes; default `0x44`). The `0x74`
  response's `maxNumberOfBlockLength` is parsed and reported; a warning
  prints if it is smaller than our 43-byte `0x36` blocks.
- **0x36** — TransferData `[0x36, blockSequenceCounter, data]`, counter
  starting at 1 and wrapping `0xFF→0x00` (ISO 14229-1). The `0x76` echo is
  verified against the sent counter — mismatch aborts. 5 s timeout per
  block (INFERRED — flash writes are slow).
- **0x37** — RequestTransferExit, only after the end-to-end CRC32 matches.
- **0x11 0x01** — ECUReset/hardReset (armed).
- **0x23** — ReadMemoryByAddress read-back: CRC32 of the read-back vs the
  transmitted CRC32. **INFERRED availability** — reported, not assumed.

### 2.3 Streaming data path (no image buffer)

The SAM3X has 96 KB RAM; the firmware deliberately holds **no image
buffer**. Each `CMD_PROG_DATA` chunk (≤41 bytes, fits the 48-byte mm frame
cap) becomes exactly one `0x36` TransferData immediately:

```
web file → 41 B chunks → F1 7E [seq LE16][flags][len][data][crc16] → M2
→ CRC16 + sequence check → 0x36 block → 0x76 echo check → CRC32 feed
```

- `flags` bit0 = LAST. A zero-length LAST chunk is the end-of-stream
  marker when the file ends exactly on a boundary (no empty `0x36`).
- Throughput is honestly slow: one ISO-TP round trip per ≤41 bytes.
  Documented as a correctness-first design, not a fast flasher.
- Max image size: unbounded by RAM (streaming), bounded by patience and
  the ECU's declared download length.

### 2.4 Console (SerialUSB)

`prog` (help + safety pointer) · `prog status` · `prog bus 0|1` ·
`prog baud <n>` · `prog tx/rx <hex>` · `prog arm` · `prog power ok` ·
`prog session` · `prog seed <lvl>` · `prog key <hex…>` ·
`prog erase <ridHex>` · `prog download <addrHex> <lenHex> [alfidHex]` ·
`prog verify <addrHex> <lenHex> [alfidHex]` · `prog done <fileCrc32Hex>` ·
`prog reset` · `prog abort`. Config works any time (staged); bus commands
need PROG active. Destructive steps need arm (+ power for erase/download).

### 2.5 mm_channel protocol (CMD 0x11–0x19)

F1 7E framing, `CMD|0x80` + status replies, little-endian payloads:

| CMD | Request | Reply |
|-----|---------|-------|
| 0x11 PROG_CFG | 14B: `[bus][baud LE32][txId LE32][rxId LE32]` | `[status]`; sets the explicit-addressing flag |
| 0x12 PROG_TX | `[svcLen][svc…]` | `[status, nResp, rLen, rData…]`; ST_ERR if PROG not active; no M2-side arm (web confirms per step, like UDS_TX) |
| 0x13 PROG_DATA | `[seq LE16][flags][len][data…][crc16 LE16]` (len ≤ 41) | `[status, ackSeq LE16]`; CRC/seq violation → transfer ABORTED loudly |
| 0x14 PROG_DONE | `[fileCrc32 LE32]` | `[status, match, txCrc32 LE32]`; needs arm; CRC match → `0x37` |
| 0x15 PROG_STATUS | `[]` | 29B: `[status][state][session][bus][baud][txId][rxId][configured][powerOk][armed][bytesRx LE32][chunksRx LE16][txCrc32 LE32][mitmSuspended]` |
| 0x16 PROG_ABORT | `[]` | `[status]`; never needs arm |
| 0x17 PROG_ARM | `[powerOk 0/1]` | `[status, armed]`; 120 s INFERRED window |
| 0x18 PROG_DL | `[addr LE32][len LE32][alfid]` | `[status]`; needs configured+armed+power |
| 0x19 PROG_VERIFY | `[addr LE32][len LE32][alfid]` | `[status, match, rxCrc32 LE32]`; read-only; INFERRED availability |

`mm::Ops` gained nine fields appended at the end; dispatch is
nullptr-safe.

---

## 3. SuperB web GUI (`/prog`, `/api/prog`)

- **Safety summary box at the top of the page itself** (not buried in
  docs): brick risk, MITM auto-suspend, per-step confirms, voltage
  attestation is a promise not a measurement, seed/key stub, power-loss
  statement.
- Status table (state, session, bus/baud/IDs, configured/armed/power,
  bytes/chunks, TX CRC32, MITM-suspended count) via `CMD_PROG_STATUS`.
- Addressing config form → `CMD_PROG_CFG`.
- Wizard: 0x10 session → 0x27 seed/key (stub warning inline) → 0x31 erase
  (confirm + power) → 0x34 download start (confirm + power) → **file
  upload** → finalize (CRC compare + 0x37) → optional 0x23 verify →
  0x11 reset / big ABORT button.
- **Upload** streams via XHR (`/prog_upload` multipart): 41-byte chunks,
  CRC16 per chunk, 8 s per-chunk transact timeout (the M2's `0x36` has a
  5 s ECU timeout). Client-side progress bar via `upload.onprogress` —
  the ESP32 WebServer is single-threaded, so server-side poll progress
  mid-upload is impossible by library design; the M2-side counters remain
  visible in `/api/prog` before/after.
- **`/api/prog`** JSON: `{ok, state, stateName, session, bus, baud, txId,
  rxId, configured, powerOk, armed, bytesRx, chunksRx, txCrc,
  mitmSuspended, upActive, upBytes, upTotal, upChunks, upError}`.
- Mode page: PROG button enabled (was "not implemented"); allow-list
  extended to 0–6.

---

## 4. Resource ownership (Phase 5)

- **PROG claims the configured CAN bus (lazy acquire, like UDS) +
  UART0/Bee** — no new claims vs UDS. `quiesceBuses()` needed no change
  (verified by reading it).
- **MITM interaction:** `enterProg()` calls `mitm::suspendRules()`, which
  clears the rule table with an explicit console notice and returns the
  count. There is no restore path by design (`docs/SAFETY_REVIEW.md` §2).
- No new TC channels, PWM, SPI, PIO interrupts, or DAC. The Phase 2/4
  collision notes stand unchanged.

---

## 5. INFERRED list (Phase 5 — no bench)

- Every ECU interaction: 0x10/0x27/0x31/0x34/0x36/0x37/0x11/0x23
  sequencing, NRC handling, timeouts (1 s default, 5 s per 0x36 block,
  30 s erase, 2 s tester-present, 30 s chunk watchdog, 120 s arm window).
- 0x23 read-back availability (ECU-dependent).
- ECU `maxNumberOfBlockLength` parsing (standard layout, untested).
- Upload throughput and browser tolerance for long XHR uploads.
- The 41-byte chunk / one-0x36-per-chunk throughput trade-off.
- `docs/OVERVIEW.md` consolidates the all-phases INFERRED inventory.

## 6. Deliberately left out

- **No key algorithms** (never — see safety review §6).
- **No target-specific procedures**, memory maps, or routine IDs.
- **No full-image RAM buffering** (streaming by design; §2.3).
- **No voltage gating** on ATRV (uncalibrated placeholder; attestation
  instead — safety review §4).
- **No J1850/SWCAN flashing** (CAN-only, same scope as UDS Phase 3).
- **No automatic retry** of failed blocks (a retry policy is an
  ECU-specific decision; failures abort loudly instead).
