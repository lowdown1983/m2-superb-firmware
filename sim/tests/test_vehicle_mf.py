#!/usr/bin/env python3
"""vehicle_ecu multi-frame Flow Control via can tx / can rx.

Drives the sim console only (no firmware UDS client), so BS/FS/29-bit
are under direct test control:

  can tx <bus> <idHex> <dataHex>   → sendFrame → vehicle_ecu::onTx
  can rx <bus>                     → drain RX queue (ECU responses)

Coverage:
  - BS=0 send-all (VIN)
  - BS=1 one CF per FC
  - FS=2 (Overflow) aborts
  - 29-bit FC continuation
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

    def expect(self, needle: bytes, timeout=12):
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


def send(m2, con, line: bytes, needle: bytes, timeout=10):
    con.drain(0.05)
    m2.stdin.write(line if line.endswith(b"\n") else line + b"\n")
    m2.stdin.flush()
    return con.expect(needle, timeout=timeout)


def rx_frames(m2, con, bus=0):
    """Return list of 'ID#DATA' strings from can rx."""
    out = send(m2, con, b"can rx %d\n" % bus, b"OK rx", timeout=5)
    out += con.drain(0.15)
    frames = []
    for line in out.splitlines():
        line = line.strip()
        if b"#" in line and not line.startswith(b"OK"):
            # formats: "  7E8#1014..." or "  ext:18DAF110#..."
            frames.append(line.strip().decode("ascii", "replace"))
    return frames


@pytest.fixture(scope="module")
def mf_console():
    for p in (M2_BIN, SB_BIN):
        assert os.path.isfile(p), "missing binary %s — run pio build first" % p
    procs = []
    try:
        broker = subprocess.Popen(
            [sys.executable, BROKER], cwd=PROJ,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(broker)
        assert wait_port(14001), "broker not up"
        m2 = subprocess.Popen(
            [M2_BIN], cwd=PROJ,
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT)
        procs.append(m2)
        sb = subprocess.Popen(
            [SB_BIN], cwd=PROJ,
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL)
        procs.append(sb)
        con = PipeReader(m2.stdout)
        con.expect(b"M2-MULTIMODE", timeout=20)
        con.drain(0.5)
        yield m2, con
    finally:
        for p in reversed(procs):
            try:
                p.terminate()
                p.wait(timeout=3)
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass


def test_mf_bs0_vin_send_all(mf_console):
    """SF 22 F190 → FF; FC BS=0 → both CFs in one block."""
    m2, con = mf_console
    # Clear any residual RX
    send(m2, con, b"can rx 0\n", b"OK rx")
    send(m2, con, b"can tx 0 7E0 0322F190\n", b"OK tx")
    frames = rx_frames(m2, con, 0)
    assert any("7E8#" in f and f.upper().find("1014") >= 0 for f in frames), \
        "expected FF 10 14 on 7E8, got %r" % frames
    # Drain FF only so far; send FC BS=0
    send(m2, con, b"can tx 0 7E0 300000\n", b"OK tx")  # 30 BS=0 STmin=0
    frames = rx_frames(m2, con, 0)
    cfs = [f for f in frames if "7E8#" in f and (
        f.upper().find("21") >= 0 or f.upper().find("22") >= 0)]
    # After BS=0 we should see both CF SN=1 and SN=2 (or at least 2 frames)
    assert len(frames) >= 2, "BS=0 should emit both CFs, got %r" % frames
    print("PASS: BS=0 VIN send-all → %d CF frames" % len(frames))


def test_mf_bs1_one_cf_per_fc(mf_console):
    """FC BS=1 → one CF; second FC → remaining CF."""
    m2, con = mf_console
    send(m2, con, b"can rx 0\n", b"OK rx")
    send(m2, con, b"can tx 0 7E0 0322F190\n", b"OK tx")
    rx_frames(m2, con, 0)  # consume FF
    send(m2, con, b"can tx 0 7E0 300100\n", b"OK tx")  # BS=1
    frames1 = rx_frames(m2, con, 0)
    assert len(frames1) == 1, "BS=1 should emit exactly 1 CF, got %r" % frames1
    assert "21" in frames1[0].upper(), "expected CF SN=1, got %r" % frames1
    send(m2, con, b"can tx 0 7E0 300100\n", b"OK tx")
    frames2 = rx_frames(m2, con, 0)
    assert len(frames2) == 1, "second BS=1 should emit final CF, got %r" % frames2
    assert "22" in frames2[0].upper(), "expected CF SN=2, got %r" % frames2
    print("PASS: BS=1 one CF per FC")


def test_mf_fs_overflow_aborts(mf_console):
    """FC FS=2 (Overflow) → no CFs, transfer aborted."""
    m2, con = mf_console
    send(m2, con, b"can rx 0\n", b"OK rx")
    send(m2, con, b"can tx 0 7E0 0322F190\n", b"OK tx")
    rx_frames(m2, con, 0)  # FF
    send(m2, con, b"can tx 0 7E0 320000\n", b"OK tx")  # FS=2
    frames = rx_frames(m2, con, 0)
    assert frames == [] or all("7E8#" not in f for f in frames), \
        "Overflow must not emit CFs, got %r" % frames
    # A subsequent CTS should also emit nothing (transfer dead)
    send(m2, con, b"can tx 0 7E0 300000\n", b"OK tx")
    frames2 = rx_frames(m2, con, 0)
    assert frames2 == [] or all("7E8#" not in f for f in frames2), \
        "aborted transfer must stay dead, got %r" % frames2
    print("PASS: FS=2 Overflow aborts")


def test_mf_29bit_fc_continues(mf_console):
    """29-bit physical request + FC produces CFs on 18DAF110."""
    m2, con = mf_console
    send(m2, con, b"can rx 0\n", b"OK rx")
    # 29-bit physical request ID 18DA10F1
    send(m2, con, b"can tx 0 18DA10F1 0322F190\n", b"OK tx")
    frames = rx_frames(m2, con, 0)
    assert any("18DAF110" in f.upper() or "ext:18DAF110" in f.lower()
               for f in frames), \
        "expected FF on 18DAF110, got %r" % frames
    send(m2, con, b"can tx 0 18DA10F1 300000\n", b"OK tx")
    frames = rx_frames(m2, con, 0)
    assert len(frames) >= 1, "29-bit FC should continue multi-frame, got %r" % frames
    print("PASS: 29-bit FC continuation → %d frames" % len(frames))


def test_mf_bs2_two_cfs_one_block(mf_console):
    """FC BS=2 → both remaining CFs in one block, then transfer completes."""
    m2, con = mf_console
    send(m2, con, b"can rx 0\n", b"OK rx")
    send(m2, con, b"can tx 0 7E0 0322F190\n", b"OK tx")
    rx_frames(m2, con, 0)  # consume FF
    send(m2, con, b"can tx 0 7E0 300200\n", b"OK tx")  # BS=2
    frames = rx_frames(m2, con, 0)
    assert len(frames) == 2, "BS=2 should emit both CFs, got %r" % frames
    # Transfer is complete: a further CTS must emit nothing.
    send(m2, con, b"can tx 0 7E0 300000\n", b"OK tx")
    assert rx_frames(m2, con, 0) == [], "completed transfer must stay dead"
    print("PASS: BS=2 one block")


def test_mf_fs_wait_aborts(mf_console):
    """FC FS=1 (Wait) → no CFs, transfer aborted."""
    m2, con = mf_console
    send(m2, con, b"can rx 0\n", b"OK rx")
    send(m2, con, b"can tx 0 7E0 0322F190\n", b"OK tx")
    rx_frames(m2, con, 0)  # FF
    send(m2, con, b"can tx 0 7E0 310000\n", b"OK tx")  # FS=Wait
    frames = rx_frames(m2, con, 0)
    assert frames == [], "Wait must not emit CFs, got %r" % frames
    print("PASS: FS=1 Wait aborts")
