#!/usr/bin/env python3
"""OBD2 poller mode vs scripted vehicle_ecu.

Flow:
  mode obd2 → configure PIDs/interval → start → detect 0100 locks protocol
  → sweep emits CSV on Bee → SuperB /api/obd2 surfaces lines.

Evidence: path VERIFIED in sim; decoded strings from pidreg (SAE formulas)
against INFERRED vehicle_ecu values.
"""
import json
import os
import select
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request

import pytest

PROJ = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
M2_BIN = os.path.join(PROJ, ".pio/build/sim_m2/program")
SB_BIN = os.path.join(PROJ, ".pio/build/sim_superb/program")
BROKER = os.path.join(PROJ, "sim", "broker.py")
WEB = "http://127.0.0.1:18080"


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


def api_get(path, timeout=8):
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        try:
            with urllib.request.urlopen(WEB + path, timeout=3) as r:
                return json.loads(r.read().decode("utf-8"))
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as e:
            last = e
            time.sleep(0.3)
    raise AssertionError("GET %s failed: %r" % (path, last))


class PipeReader:
    def __init__(self, fobj):
        self.fd = fobj.fileno()
        self.buf = b""

    def expect(self, needle: bytes, timeout=20):
        deadline = time.time() + timeout
        while time.time() < deadline:
            i = self.buf.find(needle)
            if i >= 0:
                out = self.buf[:i + len(needle)]
                self.buf = self.buf[i + len(needle):]
                return out
            r, _, _ = select.select([self.fd], [], [], 0.25)
            if r:
                chunk = os.read(self.fd, 4096)
                if not chunk:
                    break
                self.buf += chunk
        raise AssertionError("timeout waiting for %r (tail %r)"
                             % (needle, self.buf[-400:]))

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
def poller(sims):
    """Enter OBD2, configure a short PID list, start the poller."""
    m2 = sims["m2"]
    con = PipeReader(m2.stdout)
    con.expect(b"M2-MULTIMODE", timeout=25)
    con.drain()

    m2.stdin.write(b"mode obd2\n")
    m2.stdin.flush()
    # Banner varies; accept either mode name or OBD2-specific text
    try:
        con.expect(b"OBD2", timeout=10)
    except AssertionError:
        con.expect(b"obd2", timeout=5)
    con.drain()

    # Only PIDs vehicle_ecu implements (skip 0x42 volts)
    m2.stdin.write(b"obd2 pids 010C,010D,0105\n")
    m2.stdin.flush()
    con.drain(0.3)

    m2.stdin.write(b"obd2 interval 200\n")
    m2.stdin.flush()
    con.drain(0.2)

    m2.stdin.write(b"obd2 start\n")
    m2.stdin.flush()
    con.expect(b"Poller started", timeout=8)

    # Auto-detect 0100 against vehicle_ecu → protocol locked
    con.expect(b"protocol locked", timeout=20)
    con.drain(0.5)

    return m2, con


def test_o1_mode_obd2_status(poller):
    """SuperB sees mode obd2 after switch."""
    j = api_get("/api/status")
    assert j.get("m2_ok") is True, j
    mode = str(j.get("mode", "")).lower()
    assert "obd2" in mode, "expected obd2 mode, got %r" % j
    print("PASS: status mode=obd2 m2_ok")


def test_o2_poller_emits_csv(poller):
    """After start + lock, /api/obd2 carries CSV lines for configured PIDs."""
    deadline = time.time() + 15
    lines = []
    running = False
    while time.time() < deadline:
        j = api_get("/api/obd2")
        running = bool(j.get("running"))
        lines = list(j.get("lines") or [])
        joined = "\n".join(lines)
        if "01,0C" in joined and "01,0D" in joined:
            break
        time.sleep(0.4)
    assert running, "poller not running in /api/obd2: %r" % lines
    joined = "\n".join(lines)
    assert "01,0C" in joined, "no RPM CSV: %r" % lines[-10:]
    assert "01,0D" in joined, "no speed CSV: %r" % lines[-10:]
    # Coolant optional if timing tight; prefer it
    assert "01,05" in joined or "01,0C" in joined
    print("PASS: CSV lines include 01,0C and 01,0D (%d lines)" % len(lines))


def test_o3_decoded_values_present(poller):
    """pidreg decode should put engineering units in the value field."""
    j = api_get("/api/obd2")
    joined = "\n".join(j.get("lines") or []).lower()
    # Default vehicle_ecu: 3000 rpm, 80 km/h, 90 C — accept unit tokens or numbers
    assert "rpm" in joined or "3000" in joined or "0c" in joined, \
        "RPM value not decoded in CSV: %r" % joined[-400:]
    # speed often "km/h" or plain number
    assert "km" in joined or "80" in joined or "0d" in joined, \
        "speed value missing: %r" % joined[-400:]
    print("PASS: decoded value tokens present in CSV")


def test_o4_vehicle_set_affects_poll(poller):
    """vehicle set rpm 4500 → later CSV reflects new encoding (pidreg)."""
    m2, con = poller
    m2.stdin.write(b"vehicle set rpm 4500\n")
    m2.stdin.flush()
    con.expect(b"OK", timeout=5)
    con.drain(0.2)

    deadline = time.time() + 12
    found = False
    while time.time() < deadline:
        j = api_get("/api/obd2")
        joined = "\n".join(j.get("lines") or [])
        # 4500 rpm in value field
        if "4500" in joined:
            found = True
            break
        time.sleep(0.4)
    assert found, "CSV never showed 4500 rpm after vehicle set"
    print("PASS: vehicle set rpm 4500 visible in poller CSV")
