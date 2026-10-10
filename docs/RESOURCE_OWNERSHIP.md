# Resource Ownership — Design-Critical Collisions (M2)

Source: Pass 7 resource-ownership audit, 2026-10-03/04. This file is a
firmware-oriented digest. The canonical registry is the YAML matrix; this doc
never overrides it — if they disagree, the matrix wins.

Reference sources (read-only, in `~/workspace/user/files/`):

- `resource-ownership-matrix_6_ol38.yaml` — canonical ownership registry
  (matrix ID `M2-SuperB-RESOURCE-OWNERSHIP-2026-10-03-PASS7`)
- `M2___SuperB___Four_Resource_Maps_1_ct3n.md` — IRQ / DMA-PDC / timer-PWM /
  queue-buffer maps
- `pass7-resource-ownership_3_7w2r.md` — Pass 7 audit summary

## The ownership rule (the safe-mode-switch mechanism)

From the matrix (`resource-ownership-matrix_6_ol38.yaml`, lines 6–11):

> **default_rule:** One owner at a time unless explicitly shareable. Shared
> buses require arbitration. **Mode entry acquires resources before enabling
> them; mode exit releases peripherals, IRQs, GPIO alternate functions,
> timers, power controls and transports.**
>
> **timer_rule:** A TC channel is one scarce resource shared by TIOA and TIOB;
> **reserve the entire channel when either side is claimed.**

This rule *is* the "toggle mode safely" requirement: a mode switch is not
"stop one loop, start another" — it is **release everything the old mode
owned, then acquire everything the new mode needs, then enable**. The Phase 1
mode manager must encode this; see "Phase 1 checklist" below.

## Collision table

| # | Resource | Claimant A | Claimant B | Rule / consequence | Matrix ref |
|---|----------|-----------|-----------|-------------------|------------|
| 1 | **TC7** (whole channel) | J1850_PWM_RX = TC7_A (Ard 51/PC28) | SWC_SOF = TC7_B (Ard 68/PC29) | timer_rule: claiming either side reserves the **entire** TC7. J1850 mode and SWCAN-SOF use are mutually exclusive. | YAML L1247–1290; J1850 record L880–905 |
| 2 | **TC6** (whole channel) | J1850_VPW_RX = TC6_B (Ard 52/PC26) | RGB_BLUE = TC6_A | Same timer_rule. J1850 VPW mode must take TC6 whole; don't drive the blue LED via TC6 PWM while J1850 owns it. | YAML L1202–1245 |
| 3 | **PWM6** | J1850N_TX (Ard 54/PC23) | — | J1850 owns PWM6 **while active**; release on mode exit. | YAML L1589+; L889–896 |
| 4 | **SPI0 bus** | SWCAN MCP2515 (CS3 = Ard 63/PB23) | any other SPI0 client | `SHARED_ARBITRATED`: SWCAN claims CS3 + control pins while active; **all SPI0 transactions need arbitration**. | YAML L170–200; Pass 7 summary |
| 5 | **PIOC16** | SWCAN INT (Ard 67/PC16) | any other PIO callback | **Single callback slot.** Only one owner at a time; SWCAN installs a falling-edge callback there. | YAML L190–198; Four Maps §1 |
| 6 | **TC0_B** (alt function) | SWC_M0 (Ard 64/PB27) | any TC0 user | SWC_M0's alternate function is TC0_B — don't configure TC0 capture while SWCAN owns M0. | YAML L179–198 (`timer_alt`) |
| 7 | **DAC0 / PB15** | DAC0 | CAN1 RX | Pin-multiplexed. Using DAC0 precludes CAN1 RX and vice versa. | Pass 7 summary |
| 8 | **PWM0–5** | 12VIO GPIO1–6 (alt functions) | generic PWM use | Owned by 12VIO; do not treat as free PWM channels when 12VIO owns them. | Pass 7 summary |
| 9 | **UART0 (Serial)** | SuperB host link (PA8/PA9 @115200, immutable) | — | **Not a spare serial port.** Never repurpose. | Pass 7 summary; truth spec §17 |

Also noted in the matrix but not yet colliding with any planned mode: PWM7 is
alternate-function metadata on I_SENSE_EN/PC24; DMAC has 6 channels (0–5) with
no firmware-wide ownership established yet (all UNKNOWN); ADC uses
peripheral-local PDC, not the central DMAC.

## What Phase 0 touches (deliberately: almost nothing)

- `src/m2/main.cpp`: `Serial` (UART0, the immutable Bee link), `DS2`
  (Ard 14/PD10, plain GPIO output), `millis()` (SysTick — reserved platform
  infrastructure, not an application timer). **No TC channels, no PWM, no
  SPI, no PIO interrupts, no DAC.** Nothing in the collision table is claimed.
- `src/superb/main.cpp`: `Serial` (ESP32 UART0) only.

## Phase 2: ELM/J1850 claims (what changed)

ELM327 mode (`src/m2/mode_elm.cpp`) acquires lazily on first protocol use:
- **CAN protocols (1,2 excluded; 6,7,8,9,B,C):** CAN0 via `modes::canBegin0()`
  (`begin(baud,255)` — the transceiver-pin hazard rule), promiscuous RX
  filters, software ID filtering. Released by `modes::canEnd0()`.
- **J1850 protocols (1,2):** the whole J1850 block via `j1850::acquire()`
  (rail `PS_J1850_9141` HIGH, `J1850_PWM_VPW` mode select, TX pins to idle,
  RX GPIO interrupt attached). Released by `j1850::release()` (ISR detached,
  TX parked, rail LOW). The mode manager's `quiesceBuses()` calls
  `j1850::release()` unconditionally (idempotent).

**Deliberate deviation from the collision table:** rows 1–2 say J1850 RX
claims TC6/TC7 (whole channels). The Phase 2 driver does **not** use timer
capture — RX is GPIO pin-change interrupts + `micros()` (the vehicle-proven
matafonoff approach; register-level TC code cannot be validated without a
bench — see `src/m2/j1850.h` and `docs/PHASE2_NOTES.md`). Consequences:
- TC6/TC7 remain **unclaimed** by firmware. The timer_rule exclusion is
  still recorded: a future TC-capture J1850 implementation must reserve the
  whole channel and stays mutually exclusive with SWCAN-SOF (TC7_B) /
  RGB-blue (TC6_A).
- Row 3 (PWM6): the driver bit-bangs `J1850N_TX` (Ard 54/PC23) as GPIO, not
  via the PWM peripheral — but the pin is still owned while J1850-PWM is
  active and parked on release. Treat as owned-during-use.
- New claim: **GPIO IRQs on PC28/PC26** (Arduino `attachInterrupt(CHANGE)`).
  These are PIO-level interrupts, not the PIOC16 single-callback slot, so no
  conflict with the SWCAN ISR path.

## Phase 3: UDS/OBD2 claims (no change)

UDS mode (`src/m2/mode_uds.cpp`) and OBD2 poller (`src/m2/mode_obd2.cpp`)
claim **only the configured CAN bus (CAN0 or CAN1, lazy acquire) + UART0/Bee**
— the same claim class as ELM's CAN protocols. Nothing exotic: no TC
channels, no PWM, no SPI, no PIO interrupts, no DAC, no J1850 rail. The
per-controller begun flags (`can0Begun`/`can1Begun` in `mode_manager.cpp`)
exist precisely so `quiesceBuses()` never disables a never-begun controller
(which would drive Arduino 50/48 via due_can's default enable pins —
see `docs/PHASE3_NOTES.md` §4). `quiesceBuses()` itself is unchanged in
scope: native CAN controllers off, SWCAN to sleep, J1850 released.

## Phase 4: MITM/STN claims

**MITM mode** (`src/m2/mode_mitm.cpp`) claims **CAN0 + CAN1 + UART0/Bee**.
Unlike ELM/UDS (lazy acquire), MITM begins both native controllers **at mode
entry** (independent bauds, default 500000) with promiscuous RX filters, and
releases them in `mitm::exit()` via `modes::canEnd0()` / `canEnd1()`.
`quiesceBuses()` needed **no change** — it already handles both controllers
through the per-controller begun flags (Phase 3 pin-safety refactor); this
was verified by reading it, not by touching it. The mm control channel
stays live during bridging via the new shared `mm::divertByte()` helper
(serial bytes are diverted, never parsed as bridge data).

**STN mode** claims exactly what ELM claims (CAN0 lazy, or the J1850 block
via `j1850::acquire()`, + UART0/Bee) — it is the ELM core with ST-command
parsing enabled (`stn::` delegates to `elm::`). One new transient behavior:
with `STCMM 0` (the default), *monitor sessions* put the SAM3X controller
into **listen-only mode** (`enable_autobaud_listen_mode()`,
CAN_MR_ABM — due_can.cpp:521-533), restored on every monitor exit. This is
session-scoped, not a mode claim; the manager's quiesce path is unaffected
(disable() from listen-only is the same register clear).

No new TC channels, no PWM, no SPI, no PIO interrupts, no DAC in either mode.
The Phase 2 notes on TC6/TC7 (unclaimed by firmware) and PWM6 (owned
while J1850-PWM is active) stand unchanged.

## Phase 5: PROG claims

**PROG mode** (`src/m2/mode_prog.cpp`) claims **the configured CAN bus
(lazy acquire, like UDS) + UART0/Bee** — no new claims vs UDS.
`quiesceBuses()` needed **no change** (verified by reading it).

The new cross-mode behavior is the **MITM rule suspension**
(`docs/SAFETY_REVIEW.md` §2): `ModeManager::enterProg()` calls
`mitm::suspendRules()`, which clears the rule table with an explicit
console notice and returns the count (surfaced via `CMD_PROG_STATUS`).
Rationale: a bridge that modifies flash payloads is a brick vector — the
suspension is a safety property, not a resource claim. There is **no
restore path by design**; re-adding rules is a deliberate user action, so
no ownership record needs a "suspended rules" state. The `configured`
flag (explicit addressing interlock) and the power-attestation flag are
PROG-local state, reset on every PROG entry — they are not resources.

No new TC channels, no PWM, no SPI, no PIO interrupts, no DAC. The Phase
2/4 collision notes stand unchanged.

## Phase 7: K-line claims

**K-line driver** (`src/m2/kline.h/.cpp`) claims, between `kline::acquire()`
and `kline::release()` (lazy, ELM-driven on protocols 3/4/5 — same
discipline as ELM's CAN/J1850 lazy acquire):

- **USART0 / `Serial1`** (Arduino 55/PA11 TXD0, 56/PA10 RXD0) at 10400
  8N1, plus **PB4** (`LIN_KSLP`, Arduino 57) as GPIO for the TJA1027
  sleep control. 5-baud init and the fast-init wake pattern briefly
  re-mux pin 55 to GPIO for bit-banging, then return it to USART0
  (INFERRED switchback).
- **USART1 / `Serial2`** (Arduino 58/PA13 TXD1, 59/PA12 RXD1) + **PB7**
  (`LIN_LSLP`, Arduino 60): the L-line, wake use only in Phase 7.
- **Pin 49 (`PS_J1850_9141`) — SHARED with J1850**, owned via the new
  `rail9141` module (two-owner refcount, pack INV-PWR49): pin 49 goes LOW
  only when *both* J1850 and K-line are quiesced. Pin 50
  (`J1850_PWM_VPW`) is J1850's alone — the K-line driver never touches it.
  Phase 7 also fixed the Phase 2 ordering bug: `j1850::acquire()` now
  settles pin 50 *before* raising pin 49 (was: rail first).

`quiesceBuses()` now calls `kline::release()` (idempotent) after
`j1850::release()`; the refcount makes the order irrelevant to the final
rail state. New cross-check: `kline::initSlow/initFast` refuse while
`j1850::acquired()` (defense in depth; ELM's `busRelease()` already
sequences protocol switches).

No new TC channels, no PWM, no SPI, no PIO interrupts, no DAC, no new
power rails. The deliberate deviations from the collision table (Phase 2:
GPIO-interrupt J1850 RX, TC6/TC7 unclaimed) stand unchanged.

## Phase 8: J1850 + SWCAN hardening (no new claims; explicit allocation map)

Phase 8 hardens existing authority — no new buses, no new transmit
capability, no new modes. The two drivers move from ad-hoc call sites
into owned modules (`src/m2/j1850.h/.cpp` reworked, new
`src/m2/swcan.h/.cpp` wrapping the vendored SWcan lib); the mode-level
ownership in the OVERVIEW.md table is unchanged (M2RET owns SWCAN, ELM
owns the J1850 block lazily).

**Timer / power allocation map (roadmap requirement — owned explicitly):**

| Resource | J1850 owner | SWCAN owner | Rule |
|---|---|---|---|
| TC7 (whole channel) | NOT claimed (GPIO+micros RX, deliberate) | NOT claimed (no SOF capture) | INV-TC7: claiming either side reserves the whole channel. Both drivers explicitly abstain, so the exclusion holds by construction. A future TC-capture implementation must acquire TC7 whole and is mutually exclusive with the other side. |
| TC6 (whole channel) | NOT claimed | n/a | INV-TC6: J1850 VPW RX (TC6_B) vs RGB_BLUE (TC6_A). Abstained by the driver; RGB code must not touch TC6_CH6 while J1850 is acquired (unchanged from Phase 2). |
| PWM6 | Conceptually owned while J1850-PWM active (pin bit-banged as GPIO, PWM peripheral never enabled) | n/a | INV-PWM6: only the J1850 driver touches PWM6; `release()` parks J1850N_TX. |
| Pin-change IRQs | J1850_PWM_RX (51) / J1850_VPW_RX (52), CHANGE | — | Owned between acquire/release; detached on release. |
| PIOC16 (pin 67) | — | SWCAN INT, FALLING (single callback slot per pack) | Owned between swcan::acquire/release; detached on release. Pin is INPUT (MCP2515 constructor does pinMode INPUT — pack rule satisfied). |
| SPI0 | — | MCP2515 CS3 (pin 63), sole onboard claimant (INV-SPI0) | No other firmware SPI0 client; expansion-header use must follow the INV-SPI0 rules. |
| SWC_M0 (64) / SWC_M1 (65) | — | TH8056 MODE0/MODE1 GPIOs | Owned between acquire/release; release() drives them to sleep (GPIO-only fallback when the MCP2515 was never initialized). M0's TC0_CHB0 alt function is never configured while SWCAN owns it. |
| Pin 49 rail | SHARED via rail9141 (Phase 7, kept verbatim) | n/a (SWCAN has its own supply) | INV-PWR49: pin 50 settled before pin 49 HIGH; pin 49 LOW only when both J1850 and K-line quiesced. |
| K-line USART0/1 bit-bang | n/a | n/a | K-line 5-baud init uses GPIO+delay (no TC); no collision with J1850's micros()-based bit-bang — both are CPU-spin, not timer, driven. |
| txsched | n/a | n/a | millis()-based; no hardware timer. |
| Phase 12 autodet | n/a | n/a | millis()-based dwells; reuses j1850::/swcan:: acquire paths (their rows above) + due_can listen-only on CAN0. Claims no new timer, IRQ, or power resource. |
| Phase 12 vharness | n/a | n/a | CAN0+CAN1 native only (due_can); no new timer/IRQ/power. |
| Phase 12 flashstore | n/a | n/a | EFC1 (IFLASH1 last two pages 0x000FFC00/0x000FFE00). Only the Phase 12 profile layer touches EFC1; IRQs masked during the EWP sequence. |
| Phase 12 SLCAN | n/a | n/a | CAN0 native only; Bee UART (already owned). No new resources. |

`quiesceBuses()` is unchanged in structure: `m2ret::swcanSleep()` now
delegates to `swcan::release()` (TH8056 to sleep via GPIO, plus MCP2515 to
sleep when acquired; GPIO-only fallback when never initialized), `j1850::release()` is unchanged (rail9141 integration
kept verbatim). `m2ret::swcanEnter/Exit/SetSpeed` now delegate to
`swcan::acquire/release/reconfigure`.

## Phase 1 checklist (mode manager)

When the mode manager lands, every mode needs an explicit
**acquire list** and **release list** built from this table:

1. **Acquire before enable.** E.g. ELM/J1850 mode must acquire the J1850
   block (GPIO+IRQ pins per the Phase 8 explicit-abstention note below —
   *not* TC6/TC7; the Phase 2 driver deliberately uses GPIO interrupts,
   so the old "must acquire TC6 + TC7" wording is superseded) and the
   `PS_J1850_9141` power rail (Ard 49/PB5) *before* touching the J1850
   hardware; SWCAN mode must win SPI0 arbitration and claim the PIOC16
   callback slot before talking to the MCP2515. (Phase 8: the timer
   allocation map in the "Phase 8" section above makes the TC6/TC7
   abstention explicit — claiming either side would reserve the whole
   channel per INV-TC6/INV-TC7.)
2. **Release on exit, in reverse.** Disable the peripheral, detach the IRQ /
   callback, return pins from alternate function to GPIO/input, stop the
   timer, drop the power rail, close the transport. A mode must never leave
   an IRQ attached or a TC running after exit — that is how the *next* mode
   inherits a phantom owner.
3. **Conflicts are mode-exclusion, not runtime-sharing.** J1850 vs SWCAN-SOF
   on TC7, J1850-VPW vs RGB-blue on TC6: the mode manager must refuse (or
   serialize with full release in between) — never interleave.
4. **SPI0 is the only shared bus in the plan** — wrap every MCP2515
   transaction (including the ones inside the PIOC16 ISR path) in the
   arbiter, because the ISR path itself performs SPI operations (Four Maps
   §4: the MCP2515 interrupt path is materially different from the native
   CAN mailbox ISR path).
5. **Power rails are resources too.** `PS_J1850_9141` (J1850 power) and the
   TH8056 mode pins (SWC_M0/M1) are part of acquire/release — the safe-mode
   switch already requires quiescing the transceivers; releasing their power
   control is the same discipline.
6. Keep evidence labels: when a mode's resource claim can't be verified
   against the matrix, mark it UNKNOWN and don't ship it as working.
