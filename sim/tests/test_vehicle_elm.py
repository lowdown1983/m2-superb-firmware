#!/usr/bin/env python3
"""Step 2: scripted vehicle ECU answers ELM327 Mode 01 PIDs in sim.

Requires: pio run -e sim_m2 -e sim_superb (binaries under .pio/build/).
Uses the same process fixture pattern as smoke.py.
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
                             % (needle, self.buf[-200:]))

    def read_until_prompt(self, timeout=15):
        """Collect until ELM '>' prompt; return full buffer chunk."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if b">" in self.buf:
                i = self.buf.find(b">")
                out = self.buf[:i + 1]
                self.buf = self.buf[i + 1:]
                return out
            r, _, _ = select.select([self.fd], [], [], 0.2)
            if r:
                chunk = os.read(self.fd, 4096)
                if not chunk:
                    break
                self.buf += chunk
        raise AssertionError("timeout waiting for prompt (tail %r)"
                             % (self.buf[-200:],))


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
        assert wait_port(18080, 25), "SuperB web :18080 never came up"
        assert wait_port(BT_PORT, 25), "SuperB BT shim :14024 never came up"

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


def _bt_session(cmd: bytes, timeout=12):
    """Send one ELM command over the BT shim; return response bytes."""
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


def test_f_elm_pid_010c_rpm(sims):
    """ELM Mode 01 PID 0C (RPM) gets a scripted ECU response via injectFrame.

    Vehicle model returns RPM_RAW=0x2EE0 → 3000 RPM.
    ELM prints hex data; with spaces default ON expect something like
    '41 0C 2E E0' (headers off) before the prompt.
    """
    # Ensure ELM mode (boot default is elm327, but be explicit via console)
    m2 = sims["m2"]
    con = PipeReader(m2.stdout)
    sims["m2con"] = con
    con.expect(b"M2-MULTIMODE", timeout=20)
    m2.stdin.write(b"mode elm327\n")
    m2.stdin.flush()
    time.sleep(0.5)

    # Force protocol 6 (CAN 11/500) to skip auto-detect delays.
    buf = _bt_session(b"ATSP6\r")
    assert b"OK" in buf or b">" in buf, "ATSP6 failed: %r" % buf[-200:]

    buf = _bt_session(b"010C\r", timeout=15)
    # Positive response: mode+0x40, PID, two RPM bytes
    assert b"41" in buf and b"0C" in buf, \
        "no Mode 01 PID 0C response (got %r)" % buf[-300:]
    # Scripted value 0x2EE0
    assert b"2E" in buf and b"E0" in buf, \
        "RPM data mismatch, expected 2E E0 (got %r)" % buf[-300:]
    print("PASS: 010C -> 41 0C 2E E0 (3000 RPM from vehicle_ecu)")


def test_g_elm_pid_0100_supported(sims):
    """0100 (PIDs supported) returns the bitfield from vehicle_ecu."""
    buf = _bt_session(b"ATSP6\r")
    buf = _bt_session(b"0100\r", timeout=15)
    assert b"41" in buf and b"00" in buf, \
        "no 0100 response (got %r)" % buf[-300:]
    # bitfield: 08 3A 80 00 (only implemented PIDs)
    assert b"08" in buf and b"3A" in buf, "missing support bitfield (got %r)" % buf[-300:]
    print("PASS: 0100 -> support bitfield from vehicle_ecu")


def test_h_elm_pid_010d_speed(sims):
    """010D vehicle speed → 80 km/h (0x50)."""
    buf = _bt_session(b"ATSP6\r")
    buf = _bt_session(b"010D\r", timeout=15)
    assert b"41" in buf and b"0D" in buf, \
        "no 010D response (got %r)" % buf[-300:]
    assert b"50" in buf, "speed byte mismatch, expected 50 (got %r)" % buf[-300:]
    print("PASS: 010D -> 41 0D 50 (80 km/h)")
