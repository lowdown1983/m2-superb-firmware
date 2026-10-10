#!/usr/bin/env python3
"""Bee-link fault injection via broker :14003.

Proves:
  - 100% drop makes SuperB report m2_ok=false (control channel dies)
  - clearing faults lets the link recover (m2_ok=true again)
  - corrupt traffic is survivable or recoverable after `none`
"""
import json
import os
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
CTL_PORT = 14003
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


def fault_cmd(line: str, timeout=5) -> str:
    s = socket.create_connection(("127.0.0.1", CTL_PORT), timeout=timeout)
    try:
        s.sendall((line.strip() + "\n").encode("utf-8"))
        s.settimeout(timeout)
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(256)
            if not chunk:
                break
            buf += chunk
        return buf.decode("utf-8", errors="replace").strip()
    finally:
        s.close()


def api_status(timeout=8):
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        try:
            with urllib.request.urlopen(WEB + "/api/status", timeout=3) as r:
                return json.loads(r.read().decode("utf-8"))
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as e:
            last = e
            time.sleep(0.3)
    raise AssertionError("api/status failed: %r" % last)


def wait_m2_ok(want: bool, timeout=20):
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        try:
            j = api_status(timeout=3)
            last = j
            if bool(j.get("m2_ok")) is want:
                return j
        except AssertionError as e:
            last = e
        time.sleep(0.4)
    raise AssertionError("m2_ok never became %s (last=%r)" % (want, last))


@pytest.fixture(scope="module")
def sims():
    for p in (M2_BIN, SB_BIN):
        assert os.path.isfile(p), "missing %s — build first" % p
    procs = []
    try:
        broker = subprocess.Popen(
            [sys.executable, BROKER], cwd=PROJ,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(broker)
        assert wait_port(14001) and wait_port(14002) and wait_port(CTL_PORT)

        sb = subprocess.Popen(
            [SB_BIN], cwd=PROJ,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(sb)
        assert wait_port(18080, 25)

        m2 = subprocess.Popen(
            [M2_BIN], cwd=PROJ,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(m2)

        # Baseline healthy link
        wait_m2_ok(True, timeout=25)
        yield {"broker": broker, "m2": m2, "sb": sb}
    finally:
        try:
            fault_cmd("none")
        except OSError:
            pass
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


def test_f0_ctl_help(sims):
    r = fault_cmd("help")
    assert r.startswith("OK"), r
    r = fault_cmd("status")
    assert "drop A=0%" in r, r
    print("PASS: fault control port responds")


def test_f1_full_drop_breaks_link(sims):
    """100% drop both directions → SuperB cannot complete GET_STATUS."""
    fault_cmd("none")
    wait_m2_ok(True, timeout=15)

    r = fault_cmd("drop both 100")
    assert "A=100%" in r and "B=100%" in r, r

    j = wait_m2_ok(False, timeout=25)
    assert j["m2_ok"] is False
    print("PASS: full drop → m2_ok=false")


def test_f2_recover_after_none(sims):
    """Clear faults → link recovers without restarting sims."""
    # Assume previous test left drops on; be explicit
    fault_cmd("drop both 100")
    wait_m2_ok(False, timeout=20)

    r = fault_cmd("none")
    assert "drop A=0%" in r, r

    j = wait_m2_ok(True, timeout=30)
    assert j["m2_ok"] is True
    print("PASS: none → m2_ok recovered")


def test_f3_dropn_one_side(sims):
    """dropn A N drops a burst from M2→SuperB then auto-clears that counter."""
    fault_cmd("none")
    wait_m2_ok(True, timeout=15)

    # Drop a large burst so at least one status poll fails
    fault_cmd("dropn A 50")
    # May or may not flip m2_ok depending on poll timing; require status counts
    time.sleep(1.0)
    r = fault_cmd("status")
    assert "drop=" in r
    # Counter should have decremented (not still 50 unless no traffic)
    # After traffic, dropn A should be lower; if zero, OK
    print("status after dropn: %s" % r)

    fault_cmd("none")
    wait_m2_ok(True, timeout=20)
    print("PASS: dropn path exercised + recovery")


def test_f4_corrupt_then_recover(sims):
    """Heavy corruption; link should recover after clearing."""
    fault_cmd("none")
    wait_m2_ok(True, timeout=15)

    fault_cmd("corrupt both 80")
    # Give polls time to fail under corruption
    time.sleep(3.0)
    # Don't require m2_ok false (corrupt is probabilistic); require recovery
    fault_cmd("none")
    j = wait_m2_ok(True, timeout=30)
    assert j["m2_ok"] is True
    print("PASS: corrupt cleared → healthy link")
