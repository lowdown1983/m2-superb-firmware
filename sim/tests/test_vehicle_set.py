#!/usr/bin/env python3
"""Option 1: vehicle set / ramp — encoding-boundary sweeps in sim.

Console lines starting with `vehicle` are intercepted by the sim SerialUSB
shim and never reach src/m2. Values feed OBD Mode 01 responses.
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
BT_PORT = 14024


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

    def drain(self, settle=0.2):
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
        assert os.path.isfile(p), "missing %s — pio run first" % p
    procs = []
    try:
        broker = subprocess.Popen(
            [sys.executable, BROKER], cwd=PROJ,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(broker)
        assert wait_port(14001) and wait_port(14002)

        sb = subprocess.Popen(
            [SB_BIN], cwd=PROJ,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(sb)
        assert wait_port(18080, 25)
        assert wait_port(BT_PORT, 25)

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


def _console(m2, con, line: bytes, needle=b"OK", timeout=8):
    m2.stdin.write(line if line.endswith(b"\n") else line + b"\n")
    m2.stdin.flush()
    return con.expect(needle, timeout=timeout)


def _bt(cmd: bytes, timeout=12):
    s = socket.create_connection(("127.0.0.1", BT_PORT), timeout=10)
    try:
        s.settimeout(0.5)
        s.sendall(cmd)
        deadline = time.time() + timeout
        buf = b""
        while time.time() < deadline:
            try:
                chunk = s.recv(1024)
            except socket.timeout:
                if b">" in buf:
                    break
                continue
            if not chunk:
                break
            buf += chunk
            if b">" in buf:
                break
        return buf
    finally:
        s.close()


def test_s_vehicle_set_rpm_encoding(sims):
    """vehicle set rpm 0 and rpm 8000 — raw bytes cross high-byte boundary.

    OBD raw = rpm*4: 0 -> 00 00; 8000 -> 7D 00 (0x7D00 = 32000).
    """
    m2 = sims["m2"]
    con = PipeReader(m2.stdout)
    con.expect(b"M2-MULTIMODE", timeout=20)
    con.drain()

    out = _console(m2, con, b"vehicle set rpm 0")
    assert b"OK" in out

    _bt(b"ATSP6\r")
    buf = _bt(b"010C\r")
    assert b"41" in buf and b"0C" in buf
    assert b"00 00" in buf or b"00 00" in buf.replace(b"\r", b" ")
    # tolerate spaced or unspaced
    assert b"00" in buf

    _console(m2, con, b"vehicle set rpm 8000")
    buf = _bt(b"010C\r")
    # 8000*4 = 32000 = 0x7D00
    assert b"7D" in buf and b"00" in buf, "expected 7D 00 for 8000 rpm, got %r" % buf[-200:]
    print("PASS: rpm 0 and 8000 encoding")


def test_t_vehicle_set_coolant_boundary(sims):
    """coolant 99°C vs 100°C — OBD byte = C+40 → 139 (0x8B) vs 140 (0x8C)."""
    m2 = sims["m2"]
    con = PipeReader(m2.stdout)
    # module-scoped m2 already booted; just use console
    # re-bind reader to current stdout position
    con.drain(0.1)

    _console(m2, con, b"vehicle set coolant 99")
    _bt(b"ATSP6\r")
    buf = _bt(b"0105\r")
    assert b"41" in buf and b"05" in buf
    assert b"8B" in buf, "99C -> 0x8B, got %r" % buf[-200:]

    _console(m2, con, b"vehicle set coolant 100")
    buf = _bt(b"0105\r")
    assert b"8C" in buf, "100C -> 0x8C, got %r" % buf[-200:]
    print("PASS: coolant 99/100 encoding boundary")


def test_u_vehicle_ramp_speed(sims):
    """ramp speed 0->30 in 3 steps: values 0,10,20,30 across successive reads."""
    m2 = sims["m2"]
    con = PipeReader(m2.stdout)
    con.drain(0.1)

    out = _console(m2, con, b"vehicle ramp speed 0 30 3", needle=b"OK ramp")
    assert b"OK ramp" in out

    _bt(b"ATSP6\r")
    seen = []
    for _ in range(4):
        buf = _bt(b"010D\r")
        # extract last data byte heuristically — look for 41 0D XX
        assert b"41" in buf and b"0D" in buf, buf[-200:]
        # parse hex tokens after 0D
        parts = buf.upper().replace(b"\r", b" ").split()
        try:
            i = parts.index(b"0D")
            if i + 1 < len(parts):
                seen.append(int(parts[i + 1], 16))
        except ValueError:
            # unspaced "410D00" style
            idx = buf.upper().find(b"410D")
            if idx >= 0 and idx + 6 <= len(buf):
                seen.append(int(buf[idx + 4:idx + 6], 16))
    # Should include 0 and climb toward 30
    assert seen[0] == 0, "first ramp sample should be 0, got %r" % seen
    assert seen[-1] == 30, "last sample should hold at 30, got %r" % seen
    assert seen == sorted(seen), "ramp should be non-decreasing: %r" % seen
    print("PASS: speed ramp 0->30: %r" % seen)


def test_v_vehicle_show(sims):
    m2 = sims["m2"]
    con = PipeReader(m2.stdout)
    con.drain(0.1)
    m2.stdin.write(b"vehicle show\n")
    m2.stdin.flush()
    out = con.expect(b"rpm=", timeout=5)
    assert b"speed=" in out or b"speed=" in con.buf
    print("PASS: vehicle show")
