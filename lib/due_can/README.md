# due_can — vendored fork (m2-superb-firmware)

**Upstream:** https://github.com/collin80/due_can @ 2.1.6
**Vendored:** 2026-10-12, from `.pio/libdeps/m2/due_can/`
**License:** see LICENSE (upstream)

## Why a fork

The upstream 2.1.6 timing table (`src/due_can.h`, `can_bit_time[]`) has a
constraint violation in the 8-TQ row: SJW=3 with Phase_Seg1=Phase_Seg2=2,
but SJW must satisfy `SJW <= min(Phase_Seg1, Phase_Seg2)` (Bosch, *The
Configuration of the CAN Bit Timing*; also the struct's own documented
`uc_sjw = min(uc_phase1, 4)`). The TQ selector picks the 8-TQ row for
100/125/250/500 kbit/s at 84 MHz — including the firmware's 500k default —
so the default native-CAN configuration ran with reduced sync tolerance.

A post-`begin()` register override was tried (2026-10-11) and caused a
vehicle-tested regression (CANEN cleared without mailbox re-init). The
fix belongs at the source: the table itself.

## Change from upstream

- `src/due_can.h`: 8-TQ row SJW `(2 + 1)` → `(1 + 1)` (3 → 2 TQ).
  All other rows unchanged and already constraint-valid.

## Maintenance

If upstream releases a fixed version, diff `src/due_can.h` against it and
re-apply (or drop this fork). The constraint test
`sim/tests/test_duecan_timing.py` guards every table row.
