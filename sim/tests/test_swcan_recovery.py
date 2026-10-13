"""SWCAN recovery regression guard.

The 2026-10-11 audit found the bus-off handler forced MODE_NORMAL,
overriding listen-only. The fix (9899bb0) restores the requested mode.
This test guards the source against regression — it does NOT exercise
the MCP2515 (the host sim has no register mocks; see CHANGELOG).

A full register-mock behavioral test is a defined future work item.
"""
from pathlib import Path

SWCAN = Path(__file__).resolve().parents[2] / "src" / "m2" / "swcan.cpp"


def test_busoff_restores_requested_mode():
    src = SWCAN.read_text()
    assert "hw.setListenOnlyMode(wantListenOnly)" in src, (
        "bus-off fix missing: expected hw.setListenOnlyMode(wantListenOnly)"
    )


def test_no_unconditional_mode_normal_in_busoff():
    src = SWCAN.read_text()
    # The bus-off branch must not force NORMAL; the only hw.Mode(MODE_NORMAL)
    # calls should be gone (verified: setListenOnlyMode(false) == Mode(NORMAL)).
    assert "hw.Mode(MODE_NORMAL)" not in src, (
        "regression: unconditional hw.Mode(MODE_NORMAL) found in swcan.cpp"
    )


def test_recover_sleeps_before_reset():
    src = SWCAN.read_text()
    sleep_idx = src.find("driveModePins(Mode::SLEEP)")
    reset_idx = src.find("hw.Reset()")
    assert sleep_idx != -1 and reset_idx != -1, "recover() structure changed"
    # The first SLEEP drive (in recover) must precede the Reset call.
    assert sleep_idx < reset_idx, (
        "regression: TH8056 sleep must precede MCP2515 Reset in recover()"
    )


def test_failed_recover_clears_state():
    src = SWCAN.read_text()
    fail_idx = src.find("[swcan] recover FAILED")
    assert fail_idx != -1, "recover() failure path changed"
    tail = src[fail_idx:fail_idx + 400]
    assert "isAcquired = false" in tail, "failed recover must clear isAcquired"
    assert "setPresent" in tail, "failed recover must clear presence"
