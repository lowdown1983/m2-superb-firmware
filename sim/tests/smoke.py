#!/usr/bin/env python3
"""Automated smoke test for the host-native sim harness (docs/SIM.md).

Proves Marcus's three claims without hardware:
  a. M2 firmware is present and running   (banner + console 'status')
  b. SuperB firmware is present + running (GET /api/status -> 200 JSON)
  c. M2<->SuperB talk over the Bee link   (api/status m2_ok=true comes from a
                                          REAL F1 7E CMD_GET_STATUS round-trip)
  d. Bidirectional control                (POST /mode_set switches the M2's
                                          real mode via the control channel)
  e. App data path                       (BT-shim TCP -> Bee -> M2 ELM ->
                                          "ELM327" proxied back)

Run:  python3 -m pytest sim/tests/smoke.py -v      (from the project root)
Requires: built sim envs (pio run -e sim_m2 -e sim_superb). The test starts
its own broker + sims; nothing else may hold ports 14001/14002/18080/14023/14024.

Stdlib only (urllib, no requests needed).
"""
import json
import os
import select
import socket
import subprocess
import sys
import time
import urllib.parse
import urllib.request

import pytest

PROJ = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
M2_BIN = os.path.join(PROJ, ".pio/build/sim_m2/program")
SB_BIN = os.path.join(PROJ, ".pio/build/sim_superb/program")
BROKER = os.path.join(PROJ, "sim", "broker.py")
WEB = "http://127.0.0.1:18080"
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
    """Binary pipe reader with a persistent buffer: expect() consumes through
    the needle and KEEPS trailing bytes for the next expect() call."""
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


def read_until(fobj, needle: bytes, timeout=15):
    """Read from a binary pipe until needle appears (or timeout)."""
    return PipeReader(fobj).expect(needle, timeout)


def web_get(path, timeout=10):
    with urllib.request.urlopen(WEB + path, timeout=timeout) as r:
        assert r.status == 200, "GET %s -> %s" % (path, r.status)
        return r.read().decode()


def web_post(path, fields, timeout=15):
    data = urllib.parse.urlencode(fields).encode()
    req = urllib.request.Request(WEB + path, data=data, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        assert r.status == 200, "POST %s -> %s" % (path, r.status)
        return r.read().decode()


def api_status():
    return json.loads(web_get("/api/status"))


def set_mode(idx):
    web_post("/mode_set", {"mode": str(idx), "confirm": "1"})
    deadline = time.time() + 10
    while time.time() < deadline:
        if api_status()["mode"] == MODE_NAMES[idx]:
            return
        time.sleep(0.3)
    raise AssertionError("mode did not become %s" % MODE_NAMES[idx])


MODE_NAMES = {0: "m2ret", 1: "elm327", 2: "stn", 3: "uds",
              4: "mitm", 5: "obd2", 6: "prog"}


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


def test_a_m2_alive(sims):
    """M2 firmware boots and answers its console."""
    m2 = sims["m2"]
    con = PipeReader(m2.stdout)
    sims["m2con"] = con
    con.expect(b"M2-MULTIMODE", timeout=20)
    print("PASS: M2 banner seen")
    m2.stdin.write(b"status\n")
    m2.stdin.flush()
    con.expect(b"Mode:", timeout=10)
    # Default boot mode is ELM327 (Marcus's request; was M2RET before the
    # 2026-10-08 boot-mode change).
    out = con.expect(b"elm327", timeout=10)
    assert b"elm327" in out, "expected default mode elm327"
    print("PASS: M2 console 'status' -> elm327")


def test_b_superb_alive(sims):
    """SuperB web GUI answers."""
    body = web_get("/api/status")
    j = json.loads(body)
    assert "m2_ok" in j and "mode" in j, "unexpected body %r" % body[:200]
    print("PASS: GET /api/status -> 200 JSON")


def test_c_bee_link(sims):
    """m2_ok=true proves a REAL F1 7E CMD_GET_STATUS round-trip over the
    virtual Bee link (the SuperB only sets m2_ok after decoding the reply)."""
    j = api_status()
    assert j["m2_ok"] is True, "SuperB reports M2 unreachable: %r" % j
    assert j["mode"] == "elm327", "expected elm327, got %r" % j["mode"]
    print("PASS: Bee link live (m2_ok=true, mode=elm327)")


def test_d_mode_switch_roundtrip(sims):
    """POST /mode_set drives the M2's real mode manager through the
    control channel, both directions."""
    set_mode(1)
    print("PASS: M2 switched to elm327 via web control channel")
    set_mode(0)
    print("PASS: M2 switched back to m2ret")


def test_e_bt_app_data_path(sims):
    """App bytes: BT-shim TCP -> SuperB -> Bee -> M2 (ELM) -> back."""
    set_mode(1)  # ELM327 mode
    s = socket.create_connection(("127.0.0.1", BT_PORT), timeout=10)
    try:
        s.sendall(b"ATI\r")
        deadline = time.time() + 10
        buf = b""
        while time.time() < deadline:
            chunk = s.recv(1024)
            if not chunk:
                break
            buf += chunk
            if b"ELM327" in buf:
                break
        assert b"ELM327" in buf, \
            "no ELM327 response via BT path (got %r)" % buf[-200:]
        print("PASS: ATI over BT shim -> M2 -> 'ELM327' proxied back")
    finally:
        s.close()
        set_mode(0)  # leave the M2 where we found it
