# F-01 field validation — in-car run sheet

Build under test: `m2-superb-reflash-kit-20261011-ed9d0ee.zip` (commit `ed9d0ee`,
branch `fix/f01-nonblocking-poll`). Simulator-verified (30/30), NOT vehicle-validated —
this run is the validation.

| file | md5 |
|---|---|
| m2-firmware.bin | `c500b69cdbd8f9fdfc98705edcbfba11` |
| superb-firmware.bin | `291a9a841c31855dc158b93880698896` |

**Fallback (field-known-good):** `~/workspace/m2-superb-reflash-kit-20261011-9405769.zip`
(SuperB flash from this kit too if needed). Also on hand: `...-9a7f204.zip`.

What F-01 changed (short version): the M2's blocking ISO-TP waits now pump the Bee
control channel every ~25 ms, so a mode-switch command from the SuperB web GUI is
serviced mid-poll (~0.1 s) instead of timing out at 800 ms and executing late —
the old "caller sees failure, device switches anyway" split-brain. Only
state-changing commands abort a wait; status queries ride through. A timed-out
SuperB command now re-queries the M2's real state instead of reporting stale
"failed". Full notes in the kit's `CHANGELOG-F01.md`.

## Setup

1. Bolt READY (not just on) — powertrain ECUs sleep without it.
2. Laptop on M2 USB: `pio device monitor -b 115200 --port /dev/ttyACM0 | tee f01-field-$(date +%Y%m%d-%H%M).log`
3. In the console: `debug on` (debug instrumentation is in this build).
4. Phone: Torque paired to the adapter BT + SuperB web GUI reachable (join the
   SuperB AP; if Android drops it for "no internet", tell it to stay connected).

## Run order

**0. Flash.** M2: `pio run -e m2 -t upload --upload-port /dev/ttyACM0`
(or bossac line in the kit's REFLASH.md if already in bootloader). SuperB via Bee
adapter: `pio run -e superb -t upload --upload-port /dev/ttyUSB0`.
Verify M2 boot banner on the console, then reload the SuperB GUI
(`M2 unreachable` should be gone).

**1. Baseline.** Torque → connect to the Bolt ECU. PID polling should look exactly
like the `9405769` build (reference: 286 sensors, ~6.9 PIDs/sec on Torque).
Note your actual sensor count — that's the regression number.

**2. The F-01 test.** While Torque is actively polling, switch modes from the
SuperB web GUI (ELM ↔ OBD2). Do this 2–3 times.
- **Expect:** GUI reports `Switched to <mode>` promptly (well under a second);
  M2 console shows `[isotp] RX cancelled` (the mid-poll control frame was serviced)
  then `Switched to mode: <X>`. Torque may hiccup a beat on the mode change —
  that's normal; the fix is about the GUI seeing truth, not zero-disruption.
- **Pass:** switch completes promptly every time with a truthful status.
- **Fail:** GUI hangs/timeouts, M2 switches late, or polling dies after a switch.

**3. Reconciliation tier (only if you see it).** If any GUI action ever reports a
timeout, the status line should now read `(applied — confirmed after timeout)`
and the M2's actual mode should match. Don't try to force a timeout — just note
it if it happens naturally.

**4. Sanity.** A few normal GUI actions (status page, non-state-changing queries)
should behave exactly as before — benign queries must not cancel the poller.

## What to paste back

Just these lines from the USB log per test 2: the `[isotp] RX cancelled` line,
the `Switched to mode:` line, and the web GUI's reported status text for each
switch. Good or bad.

## Rollback

If the ECU won't connect or anything misbehaves: reflash the M2 from the
`9405769` kit (`m2-firmware.bin` from `~/workspace/m2-superb-reflash-kit-20261011-9405769.zip`).
That image is the field-known-good baseline with all Oct 10 fixes — only the
timing rewrite and F-01 are parked by staying on it.
