#!/usr/bin/env python3
"""Step 2b: scripted vehicle ECU answers UDS services in sim.

Drives the M2 console (SerialUSB → stdin/stdout of sim_m2):
  mode uds
  uds 10 03
  uds 22 F1 86
  uds 3E 00
  uds 22 F1 90   (multi-frame VIN)

Requires: pio run -e sim_m2 -e sim_superb
"""
import os
import select
import socket
import subprocess
import sys
import time

import pytest

PROJ = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
M2_BIN = os.path.join(PROJ, ".pio/build/sim_m2/program")
SB_BIN = os.path.join(PROJ, ".pio/build/sim_superb/program")
BROKER = os.path.join(PROJ, "sim", "broker.py")


def wait_port(port, timeout=20):
    deadline = time.time() + timeout
    while time.time() < deadline:
        s = socket.socket()
        s.settimeout(0.5)
        try:
            s.connect(("127.0.0.1", port))
            s.close()
            return True
        except OSError:
            time.sleep(0.2)
        finally:
            s.close()
    return False


class PipeReader:
    def __init__(self, fobj):
        self.fd = fobj.fileno()
        self.buf = b""

    def expect(self, needle: bytes, timeout=15):
        deadline = time.time() + timeout
        while time.time() < deadline:
            i = self.buf.find(needle)
            if i >= 0:
                out = self.buf[:i + len(needle)]
                self.buf = self.buf[i + len(needle):]
                return out
            r, _, _ = select.select([self.fd], [], [], 0.2)
            if r:
                chunk = os.read(self.fd, 4096)
                if not chunk:
                    break
                self.buf += chunk
        raise AssertionError("timeout waiting for %r (tail %r)"
                             % (needle, self.buf[-300:]))

    def drain(self, settle=0.3):
        time.sleep(settle)
        while True:
            r, _, _ = select.select([self.fd], [], [], 0.05)
            if not r:
                break
            chunk = os.read(self.fd, 4096)
            if not chunk:
                break
            self.buf += chunk
        out = self.buf
        self.buf = b""
        return out


@pytest.fixture(scope="module")
def sims():
    for p in (M2_BIN, SB_BIN):
        assert os.path.isfile(p), "missing binary %s — run pio build first" % p
    procs = []
    try:
        broker = subprocess.Popen(
            [sys.executable, BROKER], cwd=PROJ,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(broker)
        assert wait_port(14001), "broker :14001 never came up"
        assert wait_port(14002), "broker :14002 never came up"

        sb = subprocess.Popen(
            [SB_BIN], cwd=PROJ,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(sb)
        assert wait_port(18080, 25), "SuperB web never came up"

        m2 = subprocess.Popen(
            [M2_BIN], cwd=PROJ, stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
        procs.append(m2)
        yield {"m2": m2, "sb": sb, "broker": broker}
    finally:
        for p in reversed(procs):
            try:
                p.terminate()
            except OSError:
                pass
        for p in reversed(procs):
            try:
                p.wait(timeout=5)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    p.kill()
                except OSError:
                    pass


@pytest.fixture(scope="module")
def uds_console(sims):
    """Boot M2, switch to UDS mode, return (m2, PipeReader)."""
    m2 = sims["m2"]
    con = PipeReader(m2.stdout)
    con.expect(b"M2-MULTIMODE", timeout=20)
    m2.stdin.write(b"mode uds\n")
    m2.stdin.flush()
    con.expect(b"UDS mode", timeout=10)
    con.drain(0.2)
    return m2, con


def _uds_cmd(m2, con, line: bytes, timeout=12):
    m2.stdin.write(line if line.endswith(b"\n") else line + b"\n")
    m2.stdin.flush()
    # Collect until we see RX/NO DATA or a short settle
    deadline = time.time() + timeout
    while time.time() < deadline:
        r, _, _ = select.select([con.fd], [], [], 0.3)
        if r:
            chunk = os.read(con.fd, 4096)
            if chunk:
                con.buf += chunk
        if b"RX " in con.buf or b"NO DATA" in con.buf or b"NEGATIVE" in con.buf:
            # small settle for multi-line
            time.sleep(0.15)
            while True:
                r, _, _ = select.select([con.fd], [], [], 0.05)
                if not r:
                    break
                chunk = os.read(con.fd, 4096)
                if not chunk:
                    break
                con.buf += chunk
            out = con.buf
            con.buf = b""
            return out
    out = con.buf
    con.buf = b""
    return out


def test_u_session_control(uds_console):
    """uds 10 03 → positive 50 03 (extended session)."""
    m2, con = uds_console
    out = _uds_cmd(m2, con, b"uds 10 03")
    assert b"NO DATA" not in out, "no ECU response: %r" % out[-300:]
    assert b"50" in out and b"03" in out, "expected 50 03, got %r" % out[-300:]
    assert b"positive" in out, "expected positive marker: %r" % out[-300:]
    print("PASS: 0x10 0x03 -> 0x50 0x03")


def test_v_tester_present(uds_console):
    """uds 3E 00 → 7E 00."""
    m2, con = uds_console
    out = _uds_cmd(m2, con, b"uds 3E 00")
    assert b"NO DATA" not in out, "no ECU response: %r" % out[-300:]
    assert b"7E" in out and b"00" in out, "expected 7E 00, got %r" % out[-300:]
    print("PASS: 0x3E 0x00 -> 0x7E 0x00")


def test_w_read_did_session(uds_console):
    """uds 22 F1 86 → 62 F1 86 <session> (ActiveDiagnosticSession)."""
    m2, con = uds_console
    # Ensure extended session first
    _uds_cmd(m2, con, b"uds 10 03")
    out = _uds_cmd(m2, con, b"uds 22 F1 86")
    assert b"NO DATA" not in out, "no ECU response: %r" % out[-300:]
    assert b"62" in out and b"F1" in out and b"86" in out, \
        "expected 62 F1 86, got %r" % out[-300:]
    assert b"positive" in out, "expected positive: %r" % out[-300:]
    print("PASS: 0x22 F186 -> 0x62 F186 <session>")


def test_x_read_did_vin_multiframe(uds_console):
    """uds 22 F1 90 → multi-frame VIN (62 F1 90 + 17 ASCII)."""
    m2, con = uds_console
    out = _uds_cmd(m2, con, b"uds 22 F1 90", timeout=15)
    assert b"NO DATA" not in out, "no ECU response: %r" % out[-400:]
    assert b"62" in out and b"F1" in out and b"90" in out, \
        "expected 62 F1 90, got %r" % out[-400:]
    # VIN fragment from vehicle_ecu: 1G1YY25R6X5100001
    assert b"1G1" in out or b"31 47 31" in out or b"YY25" in out \
        or b"5100001" in out or b"positive" in out, \
        "VIN payload missing: %r" % out[-400:]
    print("PASS: 0x22 F190 multi-frame VIN path")


def test_y_unsupported_service_nrc(uds_console):
    """uds 3D 00 (unsupported) → 7F 3D 11 NRC."""
    m2, con = uds_console
    out = _uds_cmd(m2, con, b"uds 3D 00")
    assert b"NO DATA" not in out, "no ECU response: %r" % out[-300:]
    assert b"7F" in out, "expected negative response: %r" % out[-300:]
    print("PASS: unsupported SID -> NRC 0x7F")


