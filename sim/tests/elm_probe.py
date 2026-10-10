#!/usr/bin/env python3
"""Differential probe: run app init sequences through the real
BT-shim -> SuperB -> Bee -> M2(ELM327) path and record EXACT bytes.

Usage: python3 sim/tests/elm_probe.py  (starts its own broker+sims)
Compares against genuine ELM327 v1.5 semantics, not a second device.

What it catches: wrong echo behavior, bad line endings, '?' on commands
apps require (Torque/Car Scanner count those as errors), wrong ATDPN/ATDP.
"""
import os
import socket
import subprocess
import sys
import time

PROJ = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
M2_BIN = os.path.join(PROJ, ".pio/build/sim_m2/program")
SB_BIN = os.path.join(PROJ, ".pio/build/sim_superb/program")
BROKER = os.path.join(PROJ, "sim", "broker.py")
BT_PORT = 14024


def wait_port(port, timeout=25):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            s = socket.create_connection(("127.0.0.1", port), timeout=2)
            s.close()
            return True
        except OSError:
            time.sleep(0.3)
    return False


def transact(s, cmd, timeout=8):
    """Send cmd (no CR needed) and read until the '>' prompt."""
    s.sendall(cmd.encode() + b"\r")
    buf = b""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            chunk = s.recv(4096)
        except socket.timeout:
            break
        if not chunk:
            break
        buf += chunk
        if buf.endswith(b">"):
            break
    return buf


# (command, expected-substring, note)
TORQUE_SEQ = [
    ("ATZ", "ELM327 v1.5", "reset identifies"),
    ("ATE0", "OK", "echo off"),
    ("ATL0", "OK", "linefeeds off"),
    ("ATS0", "OK", "spaces off"),
    ("ATH0", "OK", "headers off (default)"),
    ("ATAT1", "OK", "adaptive timing on (Torque default)"),
    ("ATSP0", "OK", "auto protocol"),
    ("ATDPN", None, "describe protocol by number (record)"),
    ("ATRV", "V", "voltage read (record)"),
    ("0100", "NO DATA", "no car in sim; must be NO DATA not '?'"),
]

CARSCANNER_SEQ = [
    ("ATZ", "ELM327 v1.5", "reset identifies"),
    ("ATE0", "OK", "echo off"),
    ("ATL0", "OK", "linefeeds off"),
    ("ATS0", "OK", "spaces off"),
    ("ATH0", "OK", "headers off"),
    ("ATSP0", "OK", "auto protocol"),
    ("ATSH7DF", "OK", "functional header"),
    ("0100", "NO DATA", "no car in sim"),
    ("ATDP", None, "describe protocol (record)"),
    ("ATDPN", None, "describe protocol number (record)"),
    ("ATPC", "OK", "protocol close"),
]


def run_seq(name, seq):
    print("=" * 64)
    print("SEQUENCE:", name)
    print("=" * 64)
    s = socket.create_connection(("127.0.0.1", BT_PORT), timeout=10)
    s.settimeout(2)
    try:
        problems = 0
        for cmd, expect, note in seq:
            raw = transact(s, cmd)
            # printable form: show CR/LF explicitly
            shown = raw.replace(b"\r", b"<CR>").replace(b"\n", b"<LF>")
            ok = (expect is None) or (expect.encode() in raw)
            flag = "ok " if ok else "FAIL"
            if not ok:
                problems += 1
            print("[%s] %-8s -> %r  (%s)" % (flag, cmd, shown[:120], note))
            if expect is not None and not ok:
                print("       expected substring %r" % expect)
        print("problems: %d/%d" % (problems, len(seq)))
    finally:
        s.close()


def main():
    procs = []
    try:
        broker = subprocess.Popen(
            [sys.executable, BROKER], cwd=PROJ,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(broker)
        assert wait_port(14001) and wait_port(14002), "broker down"
        sb = subprocess.Popen(
            [SB_BIN], cwd=PROJ,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(sb)
        assert wait_port(18080, 25), "web down"
        assert wait_port(BT_PORT, 25), "BT shim down"
        m2 = subprocess.Popen(
            [M2_BIN], cwd=PROJ, stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
        procs.append(m2)
        time.sleep(3)  # let the M2 boot to ELM327 (default mode)
        run_seq("Torque-style init", TORQUE_SEQ)
        run_seq("CarScanner-style init", CARSCANNER_SEQ)
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


if __name__ == "__main__":
    main()
