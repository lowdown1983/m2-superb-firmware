# Buffer-Bounds Audit — M2/SuperB Firmware (2026-10-08)

**Auditor:** Cortana (fresh-eyes subagent) · **Scope:** `src/m2/`, `src/superb/`, `lib/SWcan/` (~88 files, ~25k lines) · **Mode:** findings only, no code edited.

## Method

1. Enumerated every `strcpy`/`strcat`/`sprintf`/`snprintf`/`memcpy`/`memmove` in scope via grep; traced each destination's declared size against the length argument and every validation gate on the path from the untrusted source.
2. Traced every untrusted length field to its clamp: Bee F1 7E LEN (M2 `mm::feedByte`, SuperB `mmc::rxByte`), GVRET parser step counters, CAN DLC paths, AT/ST argument parsers (`parseHexBytes`, `parseDec`, `parseCanId`, `hexToBytes`, `parseDecArg`/`parseHexArg`), web form handlers, ISO-TP reassembly, DBC signal extraction, PROG chunk framing.
3. Checked all `sendReply(rep, <computed>)` lengths against `rep[48]` and `frame[52]`; all `mmcTransact(..., resp, rlen)` call sites against the callee's write contract.
4. Swept for large stack locals and recursion.

**Grade discipline:** CRITICAL = will corrupt memory on a constructible input. MAJOR = likely overflow on plausible input. MINOR = theoretically unsafe, hard to trigger. NOTE = could not prove wrong, with reasoning. No style nits.

---

## CRITICAL

### C1 — Stack buffer overflow in SuperB `mmcTransact()` via `resp[8]` + 10-byte `CMD_AUTODET_STATUS` reply

- **Sink:** `src/superb/main.cpp:944-945`
  ```cpp
  rlen = pending.respLen;                                        // up to 64
  for (uint8_t i = 0; i < rlen; i++) resp[i] = pending.resp[i];  // no capacity check
  ```
  `mmcTransact()` takes `uint8_t* resp` with **no output-capacity parameter**. `pending.respLen` is clamped only to `sizeof(pending.resp)` = 64 (`main.cpp:140,157`), not to the caller's buffer.
- **Call site:** `src/superb/main.cpp:4122` (`handleP12AutodetStart`, defined at :4112):
  ```cpp
  uint8_t resp[8];                                                // 8-byte stack array
  ...
  if (ok && mmcTransact(mmc::CMD_AUTODET_STATUS, nullptr, 0, resp, rlen) && ...
  ```
- **Source of length:** M2 `src/m2/mm_channel.cpp:675-681` (`CMD_AUTODET_STATUS`): `ops->autodetStatus(rep + 2); // 8 bytes` then `sendReply(rep, 10);` — a **10-byte** reply, every time.
- **Unsafe operation:** 10 bytes written into `resp[8]` → 2-byte stack overflow (`resp[8]`, `resp[9]`). The following lines then *read* `resp[8]`/`resp[9]` (`(uint32_t)resp[8] << 24`, `resp[9]`), which are out-of-bounds reads regardless.
- **Trigger (normal use, no malice):** operator opens the Phase 12 web page, clicks auto-detect Start (with TX consent), the run finishes, and the confirmation-page fetch issues `CMD_AUTODET_STATUS`. The M2's 10-byte reply overflows the stack on the ESP32. No attacker needed — the success path does it.
- **Fix direction (for the parent):** add an `outCap` parameter to `mmcTransact()` and clamp `rlen`; enlarge the four `resp[8]` buffers to 64.

## MAJOR

### M1 — Same unsafe API, three more latent `resp[8]` call sites

- **Sites:** `src/superb/main.cpp:4090` (`CMD_PROFILE_SAVE`), `:4104` (`CMD_PROFILE_LOAD`), `:4153` (`CMD_HARNESS_RUN`) — all `uint8_t resp[8]` passed to `mmcTransact()`.
- **Why MAJOR not CRITICAL:** the M2's current replies fit — `CMD_PROFILE_SAVE`/`CMD_PROFILE_LOAD`/`CMD_AUTODET_START` reply `sendReply(rep, 2)` (`mm_channel.cpp`, PROFILE_SAVE/LOAD/AUTODET_START cases), `CMD_HARNESS_RUN` replies `sendReply(rep, 4)` (`mm_channel.cpp:684-693`). Safe today, but the safety rests on a cross-file coincidence: any future lengthening of those replies (or a desynced/corrupt M2) silently becomes a stack overflow. The root fix is the same `outCap` parameter as C1.

---

## MINOR

(None proven. Everything below is NOTE — verified safe or unprovable.)

---

## NOTEs (checked, not vulnerable — reasoning recorded)

- **N1 — M2 F1 7E receiver LEN clamp.** `src/m2/mm_channel.cpp:765-783` (`mm::feedByte`): `rxLen = b; if (rxLen == 0 || rxLen > MAX_PAYLOAD) { reset(); ... }` (`MAX_PAYLOAD`=48, `mm_channel.h:36`). `rxBuf[48]` filled only while `rxPos < rxLen ≤ 48`. Safe.
- **N2 — SuperB Bee RX diverter LEN clamp.** `src/superb/main.cpp` (`mmc::rxByte`, `DState::LEN`): `ds = (dLen == 0 || dLen > sizeof(dBuf)) ? IDLE : PAYLOAD`, `dBuf[64]`. Safe.
- **N3 — M2 `sendReply` frame sizing.** `src/m2/mm_channel.cpp:52-59`: `frame[4 + MAX_PAYLOAD]` = 52 bytes; every call passes ≤ 48 (verified each computed length: `udsTx`/`progTx` `cn ≤ MAX_PAYLOAD-4` → ≤48; `CMD_DISCOVER_LIST` `nRet ≤ 4` (`discover.cpp:256`, `nRet < 4`) → `4+4*11=48` exactly; `CMD_GW_MAP_LIST` `nRet ≤ 1` (`mode_gateway.cpp`, `nRet < 1`) → 28; `CMD_TXSCHED_LIST`/`CMD_FILTER_LIST` `nRet ≤ 2`/`≤ 1` → 44/29; `CMD_LOG_POLL` `nRec ≤ PER_POLL = 1` (`logrec.h:42`, `logrec.cpp:143`) → 26). Safe.
- **N4 — SuperB `mmcTransact` request side.** `main.cpp:928-933`: `plen > MAXP-1` rejected (plen ≤ 47), `f[4+MAXP]` = 52, max write `f[4+47]`. Safe.
- **N5 — GVRET parser `buff[20]`.** `src/m2/mode_m2ret.cpp:73`. In `BUILD_CAN_FRAME`/`ECHO_CAN_FRAME`, `buff[1+step]` is written before the switch, but `step` provably ≤ `length+6` ≤ 14 (state returns to IDLE at `step == length+6`, `length ≤ 8` via `& 0xF` + clamp). `buff[15]` max < 20. `SETUP_EXT_BUSES` always reaches IDLE at step 11. `buildFrame.data.bytes[step-6]`, index ≤ 7. Safe.
- **N6 — ELM line handling.** `lineBuf[96]` (`mode_elm.cpp:1080`), `linePos < sizeof(lineBuf)-1` with overflow reset (`:1104-1107`); `up[96]` in `handleLine` (`:1048`), `p < sizeof(up)-1`. `parseHexBytes`/`parseDec`/`parseCanId` all validate (`:707-741`). `ATWM` rejects `hl > 16` → `wakeupMsg[8]` safe. `stpx` `strncpy(work,127)` + NUL; `db[64]` via capped parse. Safe.
- **N7 — `strcpy` sites.** Only two in shipped code (`mode_elm.cpp:936` `strcpy(rxs, comma+1)`, `:1002` `strcpy(s2, comma+1)`); both `rxs`/`s2` are `char[12]` and both are preceded by `strlen(...) >= sizeof(buf)` rejection. Safe. Zero raw `sprintf`; all 45 `snprintf` bounded.
- **N8 — `sendRaw` n-vs-copy mismatch (fragile but safe).** `src/m2/mode_uds.cpp:254-259`: `uint8_t buf[12]; uint8_t n = svcLen;` then copies `min(svcLen, 12)`. If `svcLen > 12`, `transact(buf, n)` would read past `buf`. The only caller (`:700-713`) builds `svc[12]` with `svcLen < sizeof(svc)`, so `svcLen ≤ 12` provably. Safe today; a future caller passing more would introduce an OOB read — worth a defensive clamp, not a finding.
- **N9 — `confirmGate` arm buffer.** `mode_uds.cpp:54,199-217`: `armedData[12]`, `len` clamped to `sizeof(armedData)` before compare/copy (review fix M1). Safe.
- **N10 — Gateway `armPayload`.** `mode_gateway.cpp:73,317`: `memcpy(armPayload, p, 21)`, `armPayload[21]`; caller (`mm_channel.cpp` `CMD_GW_MAP_ADD`) requires `rxLen == 22`, so `p = rxBuf+1` is exactly 21 bytes. Safe.
- **N11 — MITM rule add.** `mode_mitm.cpp:478-484` reads `p[0..14]`; `CMD_MITM_RULE_ADD` requires `rxLen == 16` (`mm_channel.cpp:184-195`). `r.modOff > 7` rejected. Safe.
- **N12 — ISO-TP reassembly.** `src/m2/isotp.cpp`: FF `total > v.maxPayload` rejected (`:341`), `v.maxPayload ≤ ISOTP_BUF_MAX = 256` (`isotp.h:56-71`, `:67-68`); `slot->buf[256]`, writes bounded by `total ≤ 256` (max index 255). SF length `n ≤ 7-ao` checked (`:331`). TX path rejects `reqLen > v.maxPayload` (`:140`). `Response.len` is `uint16_t` (no truncation at 256). Safe.
- **N13 — DBC signal extraction.** `src/m2/dbcsig.cpp:13-40`: Motorola path guards `byte >= 0 && byte < dlen` and `pos` in `[0,64)`; Intel path breaks at `byte >= dlen`. Tables are build-time generated (trusted). Safe.
- **N14 — Profiles/flash.** `src/m2/profiles.cpp:66-77` (`deserialize`): exact-size check before `memcpy(&pr, in, sizeof(Profile))`, `pr.name[15] = '\0'` (`name[16]`), `nFilters ≤ 8` / `nGwMaps ≤ 8` validated. `gwMaps[8][21]` memcpy of 21 (`:144`, loop-guarded). Safe.
- **N15 — PROG chunk framing.** `mm_channel.cpp:280-300` (`CMD_PROG_DATA`): `dlen > 41` rejected, exact `(7 + dlen) == rxLen` check; `xfer::feedChunk` re-checks `len > CHUNK_MAX` (`xfer.cpp:20,57`). Safe.
- **N16 — SuperB firmware-upload accumulator.** `main.cpp:2687-2698,2749-2758`: `acc[41]`, `accN` reset exactly at 41 — can never exceed. Safe.
- **N17 — Web/JSON input.** Hand-rolled `jfield()` (`main.cpp:796-820`) bounded by `n` with a 64-char cap; `hexToBytes` caps at `cap` (`:1365-1381`); `parseDecArg`/`parseHexArg` range-validate (`:2072-2092`); `handleWifiSet`/`handleAuthLogin`/session cookies use Arduino `String` (no fixed buffers); `gwBuildPayload` fills exactly `req[21]` (`:2382-2408`); `checkPw`/`shaSalted`/`eq32` bounded (`:404-440`). `csvFeed` caps at `CSV_LEN-1` + NULs (`:1687-1701`). Safe.
- **N18 — J1850/K-line ISR buffers.** `j1850.cpp:58,138`: `buf[16]`, `byteIdx < FRAME_MAX` guard in ISR; `kline.cpp:492`: `abuf[64]`, `alen < FRAME_MAX` guard. Safe.
- **N19 — `pidreg::fmtScaled`/`extractRaw`.** `dec` is 0..3 from a compile-time table (`pidreg.h:50`); `extractRaw` guards reads with `n` (`pidreg.cpp`). Safe.
- **N20 — Stack/recursion.** Largest locals are 64–256 B (`mode_prog.cpp:461` `r[160]`, `mode_elm.cpp:833` `work[128]`); call depth is shallow (pump → handler → one helper level); no recursion found. Fine on 96 KB (M2) and ESP32 stacks.
- **N21 — `vharness::txCan1` latent.** `vharness.cpp:30-38`: `f.length = len` is set unguarded while the copy loop clamps to 8. All four call sites pass `len = 8`, so no bug — but a future caller with `len > 8` would make `due_can` read past `f.data.bytes[8]`. Test-harness-only code; noted for completeness.
- **N22 — Verified-safe memcpy sites (closed from the open list):** `superb/main.cpp:4023` (`memcpy(pname, resp+8, 16)`) guarded by `rlen < 24` check (`:4019`); `superb/main.cpp:609` (`memcpy(pub+1, OTA_PUBKEY, 64)`) — `OTA_PUBKEY[64]` confirmed (`ota_pubkey.h:9`); `logrec.cpp:148` ring copy bounded by `RING_N` slots and the `PER_POLL` contract; `mm_channel.cpp:644` (`CMD_PROFILE_SAVE`) guarded by `rxLen != 17`.

## Coverage notes

- Highest-risk surfaces all audited: F1 7E framing (both ends), GVRET binary parser, ELM/STN AT parser, all SuperB web POST handlers + WebSocket + TCP:23/BT bridges, ISO-TP, PROG/xfer, MITM/gateway rule paths, DBC decode, profiles/flash.
- The vendored `lib/SWcan/` (MCP2515) contains no `memcpy`/manual buffers of concern.
- Not in scope for this pass (no evidence of issues, not checked exhaustively): mbedTLS usage beyond buffer sizing, timing side-channels in `eq32`/`checkPw` (both look constant-time), heap fragmentation from Arduino `String` on the ESP32.

## Bottom line

**One CRITICAL** (C1: SuperB stack overflow on the normal auto-detect status path — `resp[8]` vs 10-byte `CMD_AUTODET_STATUS` reply) and **one MAJOR** (M1: the same missing-`outCap` API leaves three more `resp[8]` sites one reply-length change away from the same overflow). Everything else checked is either provably bounded or recorded above as NOTE with reasoning.
