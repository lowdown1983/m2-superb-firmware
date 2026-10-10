# Audit triage — 2026-10-08 (Cortana, post-verification)

Three findings-only audits ran over 88 files / ~25k lines:
- `audit-pins.md` — pin usage vs knowledge pack
- `audit-buffers.md` — buffer bounds on untrusted-input surfaces
- `audit-ownership.md` — resource ownership vs RESOURCE_OWNERSHIP.md

I independently verified every CRITICAL and MAJOR against the source before
writing this. Verdicts below are mine; the full reports hold the NOTEs and
method details. No code was modified.

## CRITICAL (verified by me — real, fix before any vehicle flashing work)

### C1 — SuperB stack buffer overflow in `mmcTransact()` (audit-buffers.md)
`src/superb/main.cpp:944-945`: `rlen = pending.respLen; for (i<rlen) resp[i]=pending.resp[i];`
`mmcTransact()` takes `uint8_t* resp` with **no output-capacity parameter**;
`respLen` is clamped only to 64 (internal `pending.resp`), not to the caller's buffer.
`handleP12AutodetStart` (`main.cpp:4122`) declares `uint8_t resp[8]` then calls
`mmcTransact(CMD_AUTODET_STATUS, ...)`; the M2 always replies **10 bytes**
(`src/m2/mm_channel.cpp:681` `sendReply(rep, 10)`). Result: 2-byte stack overflow
**plus** OOB reads of `resp[8]`/`resp[9]` two lines later in the baud computation.
**Trigger: normal use** — open the Phase 12 web page, run auto-detect, open the
confirmation page. The success path does it.
Related MAJOR M1: three more latent `resp[8]` sites (`:4090` PROFILE_SAVE,
`:4104` PROFILE_LOAD, `:4153` HARNESS_RUN) — safe today only because current M2
replies are 2/2/4 bytes. Same root cause.

### F-1 — V_SENSE reads 4x low: 10-bit ADC treated as 12-bit (audit-pins.md)
`src/m2/mode_manager.cpp:500-504` (`mmGetVoltageCb`): scales `analogRead(92)` by
4.469 mV/LSB (correct for 12-bit), but the SAM core defaults to **10-bit** and
nothing in the tree calls `analogReadResolution(12)` (verified: zero hits in
src/ and lib/). A 12.6 V battery reports ~3.1 V. The SuperB sleep/wake
manager's engine-off threshold can never be evaluated correctly — it will
permanently read "engine off". Pin 92 itself is correct.
Safe fix: `analogReadResolution(12)` in M2 `setup()` — `analogRead` has exactly
one call site, nothing else to break.

## MAJOR (verified by me — real bugs)

### M-1 — Auto-detect silently skips its SWCAN stage (audit-ownership.md)
`mmAutodetStartCb` (`mode_manager.cpp:543-549`, the web-GUI/console path) calls
`quiesceBuses()` directly (no mode exit), which runs `m2ret::swcanSleep()` →
`swcan::sleepGpio()` (`swcan.cpp:184`). `sleepGpio()` detaches the ISR and parks
the TH8056 but **leaves `isAcquired=true`**; `swcan::acquire()` (`swcan.cpp:117`)
returns false immediately when the flag is set. So `autodet::start()`'s
`swcan::acquire(33333)` (`autodet.cpp:80`) fails and the SWCAN stage is silently
skipped whenever SWCAN was held. Wrong-result bug, not a stuck bus
(`reacquire()` restores afterwards).

### M-2 — GATEWAY and SLCAN have no mm control-channel pump (audit-ownership.md)
`mode_gateway.cpp` contains **zero** Bee-UART reads; `mode_slcan.cpp`'s pump has
no F1 7E divert. Entering either mode from the web GUI strands remote control —
the GUI's mode-switch button times out (escape via Button1/USB still works).
Root cause: the F1 7E divert is implemented four different ways across modes;
GATEWAY/SLCAN were missed.

### M-3 — `mmSetFeature(FEAT_J1850, off)` doesn't release in STN mode (audit-ownership.md)
`mode_manager.cpp:2128` only calls `j1850::release()` when `currentMode == ELM327`,
but STN shares the ELM core (`mode_stn.cpp:24`: `elm::exit()` "releases CAN0 /
J1850 if held") and can hold the J1850 block — ISR stays attached, rail stays
powered after the user explicitly disabled the feature.

## Downgraded by me (agent said MAJOR, I say MINOR — reasoning)

### F-2 — TJA1051 silent pins 71/74 never driven
`variants/m2/variant.h:192,197` defines `CAN0_CS=71`/`CAN1_CS=74` (the TJA1051 S
pins); nothing in src/ drives them. The pack's INV-SILENT wants S HIGH on
listen-only entry, but listen-only is implemented via the SAM3X ABM bit, which
physically prevents TX and ACKs — the functional goal is met. This is a
defense-in-depth gap, not a functional bug. Optional hardening.

### F-3 — `j1850::release()` drives pin 50 LOW before the rail refcount drops
`src/m2/j1850.cpp:391`: `digitalWrite(J1850_PWM_VPW, LOW)` runs unconditionally,
then `rail9141::powerDown(J1850)` drops pin 49 only if K-line isn't holding it.
Pin 50 is a logic-level mode-select; TX pins are already parked LOW and the ISR
detached, so no bus activity results. Ordering nit; the safety-relevant rail
refcount is correct.

## Fixes applied (Marcus approved 2026-10-08, all build-verified)

1. **C1+M1** — `mmcTransact()` gained a `uint8_t outCap` parameter; the reply
   copy clamps to `min(respLen, outCap)`. All 42 call sites pass
   `sizeof(resp)`; the four `resp[8]` buffers are now `resp[64]`.
   (`src/superb/main.cpp`)
2. **F-1** — `analogReadResolution(12)` added to M2 `setup()`.
   (`src/m2/main.cpp`)
3. **M-1** — `swcan::sleepGpio()` now clears `isAcquired` (verified: quiesce is
   the only caller; restore goes through the normal enter path).
   (`src/m2/swcan.cpp`)
4. **M-2** — `mm::divertByte` pump added to `mode_gateway.cpp` and
   `mode_slcan.cpp` loops (SLCAN feeds bytes through the divert first, then
   its line parser).
5. **M-3** — `mmSetFeature(FEAT_J1850, off)` releases J1850 in STN mode too.
   (`src/m2/mode_manager.cpp`)
6. **F-2** — new `src/m2/silent_pins.h` (HSC_S=71, MSC_S=74 per pack
   INV-SILENT); ELM `monitor()` and M2RET `applyCanBusConfig()` drive S HIGH
   on listen-only entry, LOW on exit.
7. **F-3** — `j1850::release()` drives pin 50 LOW only when
   `!rail9141::powered()` (K-line still holding the rail = leave it).

Collateral repairs found during build verification (pre-existing, not from
the audit): sim shim lacked `SPI.h` (broke when the 10-08 SPI.begin fix
landed) and `setListenOnlyMode()` (broke when autodet.cpp moved to the
due_can 2.1.6 API) — both added to `src/sim_shim/`; `analogReadResolution`
stub added for fix 2. Smoke-test expectations updated for the ELM327 default
boot mode (were stale since the 10-08 boot-mode change).

Verification: `pio run -e m2 -e superb -e sim_m2 -e sim_superb` all SUCCESS;
`pytest sim/tests/smoke.py` 5/5 pass (M2 alive, SuperB alive, Bee round-trip,
web mode-switch, BT app-data path).

## Proposed fix list (superseded — applied above)

1. **C1+M1**: add `outCap` parameter to SuperB `mmcTransact()`; clamp the copy;
   enlarge the four `resp[8]` buffers to fit actual replies. (`src/superb/main.cpp`)
2. **F-1**: add `analogReadResolution(12)` in M2 `setup()`. (`src/m2/main.cpp`)
3. **M-1**: clear `isAcquired` in `swcan::sleepGpio()` (it detaches ISR + parks
   transceiver = effective release); confirm `reacquire()` path still restores.
   (`src/m2/swcan.cpp`)
4. **M-2**: add the F1 7E divert / mm pump to the `mode_gateway.cpp` and
   `mode_slcan.cpp` loops, matching the established per-mode pattern.
5. **M-3**: in `mmSetFeature`, release J1850 whenever the ELM core holds it
   (ELM327 **or** STN), not just ELM327. (`src/m2/mode_manager.cpp:2128`)
6. **F-2** (optional): drive S pins HIGH on listen-only entry, restore on exit.
7. **F-3** (optional): in `j1850::release()`, drive pin 50 only after the rail
   powerDown / when K-line isn't holding the rail.

## Clean bills (verified by the audits, spot-checked by me)

- No signal on the wrong pin anywhere; all 8 `due_can begin()` calls pass 255
  (50/48 hazard neutralized); pin-49 rail refcounting correct.
- All `sendReply` lengths ≤ 48; F1 7E LEN clamped both ends; ISO-TP reassembly
  bounded; no recursion; largest stack locals ~256 B.
- All mode switches funnel through `switchTo()` (exit→quiesce→enter→capgate
  reset); MITM/gateway PROG-entry suspends have no restore paths; no `delay()`
  in ISRs.
