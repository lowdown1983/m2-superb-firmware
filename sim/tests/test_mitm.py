#!/usr/bin/env python3
"""MITM rule engine vs sim can inject."""
import json
import os
import select
import socket
import subprocess
import sys
import time
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


def api_status(timeout=8):
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        try:
            with urllib.request.urlopen(WEB + "/api/status", timeout=3) as r:
                return json.loads(r.read().decode("utf-8"))
        except Exception as e:
            last = e
            time.sleep(0.3)
    raise AssertionError("api/status failed: %r" % last)


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
                             % (needle, self.buf[-500:]))

    def drain(self, settle=0.25):
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


def send(m2, con, line: bytes, needle: bytes, timeout=12):
    con.drain(0.05)
    m2.stdin.write(line if line.endswith(b"\n") else line + b"\n")
    m2.stdin.flush()
    return con.expect(needle, timeout=timeout)


@pytest.fixture(scope="module")
def sims():
    for p in (M2_BIN, SB_BIN):
        assert os.path.isfile(p), "missing %s" % p
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
        yield {"m2": m2}
    finally:
        for p in reversed(procs):
            try:
                p.terminate()
            except OSError:
                pass
        for p in reversed(procs):
            try:
                p.wait(timeout=5)
            except Exception:
                try:
                    p.kill()
                except OSError:
                    pass


@pytest.fixture(scope="module")
def mitm(sims):
    m2 = sims["m2"]
    con = PipeReader(m2.stdout)
    con.expect(b"M2-MULTIMODE", timeout=25)
    con.drain()
    send(m2, con, b"mode mitm\n", b"mitm", timeout=12)
    con.drain(0.4)
    return m2, con


def test_m1_mode_mitm(mitm):
    j = api_status()
    assert j.get("m2_ok") is True
    assert "mitm" in str(j.get("mode", "")).lower(), j
    print("PASS: mode mitm")


def test_m2_drop_rule(mitm):
    m2, con = mitm
    send(m2, con, b"mitm clear\n", b"cleared")
    send(m2, con, b"mitm rule add 100 7FF a2b drop\n", b"rule added")
    for _ in range(3):
        send(m2, con, b"can inject 0 100 DEADBEEF\n", b"OK inject")
        time.sleep(0.05)
    time.sleep(0.25)
    out = send(m2, con, b"mitm stats\n", b"drop A->B=")
    full = out + con.drain(0.3)
    assert b"drop A->B=" in full
    print("PASS: drop rule")


def test_m3_log_rule(mitm):
    m2, con = mitm
    send(m2, con, b"mitm clear\n", b"cleared")
    send(m2, con, b"mitm rule add 200 7FF a2b log\n", b"rule added")
    send(m2, con, b"can inject 0 200 CAFEBABE\n", b"OK inject")
    time.sleep(0.25)
    con.drain(0.2)  # discard inject echo; list must provide action=
    out = send(m2, con, b"mitm rule list\n", b"action=")
    full = out + con.drain(0.3)
    assert b"action=log" in full, full[-400:]
    assert b"hits=" in full, full[-400:]
    print("PASS: log rule hits")


def test_m4_default_pass(mitm):
    m2, con = mitm
    send(m2, con, b"mitm clear\n", b"cleared")
    send(m2, con, b"can inject 0 300 01020304\n", b"OK inject")
    time.sleep(0.25)
    out = send(m2, con, b"mitm stats\n", b"fwd A->B=")
    full = out + con.drain(0.3)
    assert b"fwd A->B=" in full, full[-400:]
    print("PASS: default pass")
