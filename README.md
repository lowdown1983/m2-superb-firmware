# M2/SuperB Multi-Mode Firmware

PlatformIO firmware for the Macchina M2 (SAM3X8E) and the SuperB
(ESP32-WROOM-32D): seven diagnostic/utility modes on the M2, with the SuperB
as a wireless Bluetooth/WiFi bridge plus a web GUI.

## Modes

- **M2RET** — GVRET-compatible CAN capture/forwarding (works with SavvyCAN)
- **ELM327** — AT command emulation for Torque / FORScan
- **STN** — STN11xx-style command set (subset, built as an ELM-core extension)
- **UDS** — diagnostic services (0x10/11/14/19/22/27/28/2E/3E/85)
- **OBD2** — automatic PID poller, CSV (`ts_ms,01,pid,value`) over the Bee link
- **MITM** — CAN0<->CAN1 bridge with an ordered rule engine
  (pass/drop/modify/count/log, first-match-wins)
- **PROG** — programming mode: session → key → erase → chunked flash streaming
  with CRC16 per chunk and CRC32 end-to-end

SuperB side: Bluetooth SPP (`M2-SuperB`), WiFi AP+STA, TCP:23 bridge, and a
web GUI (status, mode control, settings, WiFi, UDS/OBD2/MITM pages).

## Build

```sh
pio run -e m2      # Macchina M2 (SAM3X8E)
pio run -e superb  # SuperB (ESP32)
pio run            # both
```

New to PlatformIO or to flashing the M2/SuperB? Start with **SETUP.md** —
beginner-friendly, including Bazzite (immutable Linux) USB-serial setup and a
diagnostics script.

## The one immutable hardware rule

The M2↔SuperB link is the Bee UART at **115200 bps**. That rate is never
parameterized anywhere in this project.

## Safety

- Every mode switch quiesces all buses first (CAN controllers silent, SWCAN
  transceiver to sleep).
- Destructive UDS/programming services require explicit arming with a
  10-second confirm window.
- The seed/key service (0x27) is a permanent stub — no key algorithms are
  invented or shipped. See `docs/SAFETY_REVIEW.md`.

## Companion repository

- [m2-oem-database](https://github.com/lowdown1983/m2-oem-database) — Big
  Three bus architecture, diagnostic access, and message catalogs (16
  opendbc DBCs, graded A/B/C). This is the vehicle data the UDS and OBD2
  modes work against.

## Status

All seven modes are compile-verified under PlatformIO. Hardware behavior is
labeled INFERRED until in-vehicle testing — see `docs/` (per-phase notes).
