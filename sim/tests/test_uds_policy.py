"""Policy regression tests (Marcus decision 2026-10-10).

Raw UDS 0x34/0x36/0x37 (RequestDownload/TransferData/RequestTransferExit)
are refused on the console raw path — flashing goes only through the
confirm+CRC download engine ('uds dl' / 'uds ul' / 'uds xferdone').

Reuses the shared fixtures from test_vehicle_uds (boots sim_m2 + broker +
vehicle ECU). The refusal happens before any bus traffic, so no ECU
interaction is needed for the refusal cases.
"""
import pytest

from sim.tests.test_vehicle_uds import sims, uds_console  # noqa: F401  (fixtures)


def _send_raw(m2, con, line: bytes) -> bytes:
    m2.stdin.write(line if line.endswith(b"\n") else line + b"\n")
    m2.stdin.flush()
    out = con.expect(b"Refused", timeout=10)
    tail = con.drain(0.5)
    return out + tail


def test_raw_34_refused(uds_console):
    """uds 34 ... -> refused, pointer to the download engine."""
    m2, con = uds_console
    out = _send_raw(m2, con, b"uds 34 00 44 12 34 10")
    assert b"download engine" in out, "no engine pointer: %r" % out[-300:]


def test_raw_36_refused(uds_console):
    """uds 36 ... -> refused."""
    m2, con = uds_console
    out = _send_raw(m2, con, b"uds 36 01 AA BB")
    assert b"download engine" in out, "no engine pointer: %r" % out[-300:]


def test_raw_37_refused(uds_console):
    """uds 37 -> refused."""
    m2, con = uds_console
    out = _send_raw(m2, con, b"uds 37")
    assert b"download engine" in out, "no engine pointer: %r" % out[-300:]


def test_raw_other_services_unaffected(uds_console):
    """A non-transfer raw service still goes through (grant-gated)."""
    m2, con = uds_console
    m2.stdin.write(b"uds 3E 00\n")
    m2.stdin.flush()
    out = con.expect(b"7E", timeout=12)
    assert b"Refused" not in out, "0x3E wrongly refused: %r" % out[-300:]
