"""due_can timing-table constraint test.

Guards the vendored due_can fork (lib/due_can, from collin80/due_can@2.1.6):
every row of can_bit_time[] must satisfy SJW <= min(PH1, PH2) per the Bosch
bit-timing paper, and the TQ the library's selector picks for each
supported baud must land on a valid row with acceptable baud error.

This is the regression test for the 2026-10-12 audit finding: upstream's
8-TQ row had SJW=3 with PH1=PH2=2.
"""
import re
from pathlib import Path

HEADER = Path(__file__).resolve().parents[2] / "lib" / "due_can" / "src" / "due_can.h"

MCK = 84_000_000
CAN_MIN_TQ, CAN_MAX_TQ = 8, 25
CAN_BAUDRATE_MAX_DIV = 128

# Must match ModeManager::validCanBaud() in src/m2/mode_manager.cpp
SUPPORTED_BAUDS = [95200, 100000, 125000, 250000, 500000, 800000, 1000000]


def parse_table():
    text = HEADER.read_text()
    m = re.search(r"can_bit_time\[\]\s*=\s*\{(.*?)\};", text, re.S)
    assert m, "can_bit_time[] table not found in due_can.h"
    rows = []
    for rm in re.finditer(
        r"\{\s*(\d+)\s*,\s*\(\s*(\d+)\s*\+\s*1\s*\)\s*,\s*"
        r"\(\s*(\d+)\s*\+\s*1\s*\)\s*,\s*\(\s*(\d+)\s*\+\s*1\s*\)\s*,\s*"
        r"\(\s*(\d+)\s*\+\s*1\s*\)\s*,\s*(\d+)\s*\}",
        m.group(1),
    ):
        tq, prog, ph1, ph2, sjw, sp = (int(g) for g in rm.groups())
        rows.append(
            {"tq": tq, "prog": prog + 1, "ph1": ph1 + 1,
             "ph2": ph2 + 1, "sjw": sjw + 1, "sp": sp}
        )
    assert rows, "no table rows parsed"
    return rows


def select_tq(baud):
    """Mirror of due_can.cpp set_baudrate() TQ selection."""
    best_mod, best_tq = None, CAN_MIN_TQ
    for tq in range(CAN_MIN_TQ, CAN_MAX_TQ + 1):
        if MCK / (baud * tq) <= CAN_BAUDRATE_MAX_DIV:
            mod = MCK % (baud * tq)
            if best_mod is None or mod < best_mod:
                best_mod, best_tq = mod, tq
                if mod == 0:
                    break
    return best_tq


def test_every_row_satisfies_sjw_constraint():
    for r in parse_table():
        assert r["sjw"] <= min(r["ph1"], r["ph2"]), (
            f"TQ={r['tq']}: SJW={r['sjw']} > min(PH1={r['ph1']}, PH2={r['ph2']})"
        )


def test_8tq_row_has_corrected_sjw():
    rows = {r["tq"]: r for r in parse_table()}
    assert rows[8]["sjw"] == 2, f"8-TQ SJW={rows[8]['sjw']}, expected 2 (fork fix)"


def test_selected_rows_valid_and_baud_error_small():
    rows = {r["tq"]: r for r in parse_table()}
    for baud in SUPPORTED_BAUDS:
        tq = select_tq(baud)
        r = rows[tq]
        assert r["sjw"] <= min(r["ph1"], r["ph2"]), (
            f"{baud}: selects TQ={tq} which violates the SJW constraint"
        )
        prescale = MCK // (baud * tq)
        actual = MCK / (prescale * tq)
        err = abs(actual - baud) / baud
        assert err < 0.01, f"{baud}: baud error {err:.3%} too large (TQ={tq})"
