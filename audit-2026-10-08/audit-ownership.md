# Resource-Ownership Audit — M2/SuperB Firmware

**Date:** 2026-10-08 · **Auditor:** fresh-eyes subagent (findings only, no code touched)
**Scope:** `src/m2/`, `src/superb/`, `lib/SWcan/` vs. `docs/RESOURCE_OWNERSHIP.md` and `docs/SAFETY_REVIEW.md`
**Evidence labels:** VERIFIED = read in the source cited. INFERRED = hardware behavior not yet vehicle-tested (test vehicle: 2023 Chevy Bolt EV, CAN-only — J1850/SWCAN/K-line behavior is INFERRED by standing rule).

## Method

1. Read `docs/RESOURCE_OWNERSHIP.md` and `docs/SAFETY_REVIEW.md` (§§1–14) as the ownership contract.
2. Read `src/m2/mode_manager.{h,cpp}` in full (2,174 lines): every `enter*()`/`exit*()`, `switchTo()`, `quiesceBuses()`, all switch inputs (Button1, USB console, mm channel), `canBegin0/1`/`canEnd0/1`, `mmSetCanBaud/SwcanSpeed/Feature`.
3. Read the three shared-resource drivers end to end: `j1850.cpp` (acquire/release/ISR/TX bit-bang), `kline.cpp` (acquire/release/init guards), `rail9141.{h,cpp}` (pin-49 refcount), `swcan.cpp` (acquire/release/SPI arbiter/sleep/recover).
4. Traced every mode's Bee-UART (mm control channel) pump: `mode_m2ret`, `mode_elm` (+`mode_stn` delegation), `mode_uds`, `mode_mitm`, `mode_prog`, `mode_gateway`, `mode_slcan`, `vharness`, plus `mm_channel.cpp` `divertByte`.
5. Checked PROG-entry suspend (`mitm::suspendRules`, `gateway::suspend`) for restore paths; checked `capgate` reset on mode entry (read in `switchTo`, mode_manager.cpp:944).
6. Read SuperB `src/superb/main.cpp` TX paths: `mmc::uartWriteFiltered`, `mmcTransact`, `own::txAllowed`, `pending` reply matching, WS/TCP/BT gating.
7. Grepped all `delay()` call sites for ISR context; inventoried `attachInterrupt` claims; confirmed no code claims TC6/TC7 (abstention honored).
8. Checked cross-mode TX hazards: `txsched::poll()` vs `busstat::present`, `busfilter`, OBD2 slot removal on exit.

## What verified clean (no finding)

- **Core switch path.** All three switch inputs — Button1 (`pollButton`→`requestMode`, mode_manager.cpp:993/1004), USB console `mode <name>` (`handleConsoleLine`, :2046), mm `SET_MODE` (`mmRequestMode`, :113) — funnel through `requestMode()`→`switchTo()`, which does `exitOld()` → `quiesceBuses()` → `enterNew()` → `capgate::enterMode()`. No bypass path found.
- **`quiesceBuses()`** (mode_manager.cpp:686–712): CAN0/1 `disable()` gated on begun flags (pin-safety), `m2ret::swcanSleep()`, `j1850::release()`, `kline::release()` — all idempotent.
- **rail9141 refcount** (rail9141.cpp): idempotent per owner, pin 49 HIGH on 0→1, LOW only when both released. No direct pin-49 writes outside it (grep). INV-PWR49/INV-MODE50 ordering honored in `j1850::acquire` (pin 50 before rail).
- **J1850/K-line mutual exclusion:** `kline::initSlow/initFast` refuse while `j1850::acquired()` (kline.cpp:300,353); `j1850::acquire` returns false if already acquired; both `release()`s clear their flags and detach/park hardware.
- **SWCAN SPI arbiter** (swcan.cpp): every main-context MCP2515 transaction wrapped in `noInterrupts()/interrupts()`; ISR (`isrTrampoline`→`hw.intHandler()`) does SPI but runs with interrupts disabled; no other ISR does SPI. J1850 RX ISR is GPIO-only. Sound.
- **MITM auto-suspend on PROG entry:** `mitm::suspendRules()` (mode_mitm.cpp:567) zeroes the rule table; **no restore path exists** (grep for restore/clearRules finds none). `enterProg` (mode_manager.cpp:759) calls it before `prog::begin()`.
- **Gateway auto-suspend:** `gateway::suspend()` (mode_gateway.cpp:555) clears mappings + sets `suspendedFlag`; `begin()` does NOT clear the flag (only prints a notice, :505); `loop()` refuses while suspended (:521); the only un-suspend is explicit mapping add (:335). `clearSuspended()` is declared but never called — dead code, harmless.
- **PROG interlocks:** `prog::begin()` (mode_prog.cpp:540) resets `armed=false`, `powerOk=false`, `configured=false`, `seedLevel=0`; `exit()` (:595) aborts in-flight transfers (best-effort 0x37) and releases the bus. No silent restore.
- **TX scheduler cross-mode:** `txsched::fire()` (txsched.cpp:20) checks `busstat::present()` before touching the controller; quiesce and `canEnd0/1` clear present. OBD2 removes its sweep slot on exit (mode_obd2.cpp:172).
- **No `delay()` in ISRs.** All `delay()` sites are task/loop/setup context. J1850 TX bit-bang uses `micros()` spin loops under `noInterrupts()` (j1850.cpp:221–222, 246–260) — no `delay()` in the time-critical path.
- **Interrupt inventory:** only two ISR owners — J1850 RX GPIO (`j1850.cpp:370`, detached in `release()` and around TX) and SWCAN INT (`swcan.cpp:142`, detached in `release()`/`sleepGpio()`). No code claims TC6/TC7 (abstention documented and honored). due_can is polled, not interrupt-driven, in this tree.
- **SuperB Bee TX:** single-threaded loop; single-owner policy (`own::txAllowed`, main.cpp:328) gates WS (:859,:887), TCP (:3863), and BT (:3882); idle timeout (default 300 s) + disconnect release + `own::poll()` in the main loop (:4460); drops counted, never silent. The F1 7E TX filter (`uartWriteFiltered`, :203) is sound, including the 50 ms held-F1 flush and bypass only for the SuperB's own legitimate `mmcTransact` frames.
- **capgate:** grant reset on every mode entry (`switchTo`, mode_manager.cpp:944).

---

## MAJOR

### M-1. `swcan::sleepGpio()` leaves a stale `isAcquired=true` — auto-detect's SWCAN stage can never run after M2RET/GATEWAY
- **Files:** `src/m2/swcan.cpp:184` (`sleepGpio`), `src/m2/autodet.cpp:80`, `src/m2/mode_manager.cpp:548,1697`
- **Path:** `autodetect start` (USB console, :1697) or mm autodet (:548) → `quiesceBuses()` → `m2ret::swcanSleep()` → `swcan::sleepGpio()` detaches the INT pin and parks the TH8056 **but does not clear `isAcquired`** → `autodet::start()` SWCAN stage calls `swcan::acquire(33333)` → `if (isAcquired) return false` (swcan.cpp:117) → stage returns 0 frames.
- **Trigger condition:** the pre-autodet mode held SWCAN — always true for M2RET (`enterM2RET`→`m2ret::swcanEnter()`→`swcan::acquire()`, mode_manager.cpp:735) and GATEWAY (`gateway::begin`, mode_gateway.cpp:496).
- **Effect:** the SWCAN leg of auto-detect is silently skipped (reported as "no frames", indistinguishable from "no SWCAN vehicle"). The system recovers afterwards (`reacquire()`→`exitOld()`→`swcan::release()` clears the flag), so this is a wrong-result bug, not a stuck bus. INFERRED-until-tested on a GMLAN vehicle, but the code path is deterministic.
- **Contrast:** `j1850::release()` and `kline::release()` both clear their acquired flags, so their autodet stages work. Only the SWCAN sleep path has the stale flag.

### M-2. GATEWAY and SLCAN modes have no mm control-channel pump — web GUI cannot switch the M2 away once entered
- **Files:** `src/m2/mode_gateway.cpp` (zero `Serial.read()` — the entire file), `src/m2/mode_slcan.cpp:238` (pump reads CR-terminated lines only, no F1 7E divert)
- **Path:** SuperB web GUI → `mmcTransact(CMD_SET_MODE, …)` → F1 7E frame on Bee → M2 in GATEWAY mode never reads `Serial` → frame sits in the UART RX buffer → `mmcTransact` times out (800 ms busy-wait). Same in SLCAN: the F1 7E bytes are swallowed into `lineBuf` as garbage until `\r`/overlong-line resync.
- **Effect:** entering GATEWAY or SLCAN from the web GUI strands the user — the GUI's own mode-switch button stops working. Escape still exists via Button1 (`pollButton`→`requestMode`) and the USB console (`mode <name>`, `pollSerialConsole` runs in every mode), so this is loss-of-remote-control, not a trap. HARNESS (`vharness.cpp`, no Bee reads at all) has the same property but is arguably by-design for a scripted self-test mode — flagged as MINOR below.
- **Note:** every other interactive mode services F1 7E — M2RET (command-state, mode_m2ret.cpp:289), ELM/STN (inline divert, mode_elm.cpp:595,1139), UDS (mode_uds.cpp:497), MITM/PROG (`mm::divertByte`, mode_mitm.cpp:335, mode_prog.cpp:570). The divert is implemented **four different ways**, which is presumably how GATEWAY/SLCAN were missed.

### M-3. `mmSetFeature(FEAT_J1850, off)` doesn't release J1850 when STN mode holds it
- **File:** `src/m2/mode_manager.cpp:2122` (`mmSetFeature`), specifically :2128: `if (!enable && currentMode == Mode::ELM327) j1850::release();`
- **Path:** STN mode shares the ELM core (`stn::loop()`→`elm::loop()`, mode_stn.cpp:18; `stn::begin()`→`elm::begin()`), so STN can hold the J1850 block via `elm::busActivate` (mode_elm.cpp:131). Disabling FEAT_J1850 via mm while in STN sets `j1850en=false` (blocks *future* acquires) but skips the `j1850::release()` — ISR stays attached, rail stays powered, TX pins stay driven, contradicting the user's explicit disable. Release only happens later at mode exit (`exitStn`→`elm::exit()`→`busRelease()`).
- **Effect:** ownership hygiene/power-state violation; the J1850 bus idles passive so no frames are emitted, but the hardware is live against the feature flag. The ELM327 branch proves the intent; STN was missed because it reuses the ELM core.

---

## MINOR

### m-1. `mmSetCanBaud` live-apply violates the G1 "begun means successfully begun" invariant
- **File:** `src/m2/mode_manager.cpp:2098`: `if (bus == 0) { Can0.begin(baud, 255); can0Begun = true; }` — the `begin()` return value is discarded, unlike `enterM2RET` (:724) and `canBegin0/1` (:2140) which record `ok`.
- **Path:** mm `SET_CAN_BAUD` while in M2RET mode.
- **Effect:** limited — the safety property that matters (pin-safe `disable()`, which only needs `enablePin==255`) holds regardless of init success, and `busstat::setPresent(...,true)` is set alongside. The flag becomes a lie only in the "controller failed to init" corner, making `status` output and txsched gating over-optimistic.

### m-2. `j1850::release()` drives pin 50 (mode select) even when J1850 was never acquired
- **File:** `src/m2/j1850.cpp:391` — `digitalWrite(J1850_PWM_VPW, LOW)` sits outside the `if (isAcquired)` guard (which only covers the ISR detach).
- **Path:** `quiesceBuses()` calls `j1850::release()` unconditionally on every mode switch.
- **Effect:** benign today — INV-MODE50 ("pin 50 must not change under J1850 traffic") can't be violated when there is no traffic, and no other driver uses pin 50. Fragile if pin 50 is ever shared: a spurious write from an unrelated mode's quiesce would move another owner's select line.

### m-3. HARNESS mode has no Bee/mm pump (same shape as M-2, lower severity)
- **File:** `src/m2/vharness.cpp` — no `Serial` reads.
- **Path:** entering HARNESS via web GUI mm `SET_MODE`; the self-check then runs to completion with no remote supervision or remote abort.
- **Effect:** by-design-adjacent (scripted self-test), but the mode can only be exited via Button1/USB console, and there is no documented statement that the mm channel is intentionally dead here. Recommend either a pump or a one-line design note.

### m-4. M2RET silently degrades if SWCAN acquire fails at entry
- **File:** `src/m2/mode_m2ret.cpp:601` (`swcanEnter`): `swcan::acquire()` failure sets `swcanEnabled=false` with no console warning (contrast `gateway::begin`, mode_gateway.cpp:496, which prints one).
- **Effect:** SWCAN just doesn't work; safe direction (`busstat` present=false, txsched skips). Observability only.

---

## NOTE

- **N-1. Four implementations of the F1 7E divert.** `mm::divertByte` (MITM/PROG), inline state in ELM's monitor loop and `pumpByte`, inline state in UDS, GVRET command-state in M2RET. They agree on the wire format today, but any future framing change must be made in four places — M-2 is the first casualty of this duplication.
- **N-2. `swcan::reconfigure()` while acquired does release→acquire with a 100 ms sleep settle inside** (swcan.cpp:196–208) — correct per the Phase-8 reviewer fix, but it runs with interrupts enabled between the two calls; a concurrent `poll()` from the mode loop can't interleave (single thread), fine.
- **N-3. SuperB `pending` reply matching** (`handleReply`, superb/main.cpp:156) keys on `cmd == pending.cmd`. Safe because `mmcTransact` is non-reentrant (single-threaded, blocking wait) and client F1 7E injection is filtered — but any future async mm frame from the M2 would need a distinct opcode space to avoid stealing a transact reply.
- **N-4. `clearSuspended()` (mode_gateway.h:104) is dead code** — declared, defined, never called. The suspend flag is only cleared by explicit mapping add, which is the correct policy; consider removing the dead declaration so a future caller can't silently lift a PROG-entry suspend.
- **N-5. `mmSetCanBaud` re-begins an already-begun controller** without an intervening `disable()` (mode_manager.cpp:2098). due_can `begin()` re-init appears idempotent in practice (GVRET cmd 5 does the same), but this is INFERRED from the vendored lib, not from a contract.
- **N-6. All J1850/SWCAN/K-line timing claims remain INFERRED** (rail settle 50 ms, TH8056 transitions, VPW/PWM symbol timings) — unchanged from the phase notes; the Bolt EV test vehicle is CAN-only.

## Suggested verification order (not fixes — findings only per task)

1. M-1: enter M2RET, run `autodetect start`, confirm the SWCAN stage is skipped vs. after the fix.
2. M-2: web GUI → GATEWAY → attempt web-GUI mode switch back; confirm timeout.
3. M-3: STN mode, J1850 protocol active, mm `SET_FEATURE FEAT_J1850 off`, probe pin 49 / ISR state.
