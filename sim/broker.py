#!/usr/bin/env python3
"""Bee-link broker: bridges sim_m2 <-> sim_superb, logs bytes, injects faults.

Ports:
  14001  sim_m2 Bee Serial (side A)
  14002  sim_superb Bee Serial (side B)
  14003  fault-control console (text commands, one line each)

Fault control (connect to :14003, send a line, read reply):
  none                         clear all faults
  drop   <A|B|both> <0-100>    drop that % of *chunks* in the named direction(s)
  corrupt <A|B|both> <0-100>   flip 1 bit in that % of chunks
  dropn  <A|B> <n>             drop the next n chunks from that side (then clear)
  status                       print current policy
  help

Directions are source-side: `drop A 100` drops all A→B traffic (M2→SuperB).

Default policy is transparent (no faults) so existing smoke tests stay green.

Run from project root:  python3 sim/broker.py
"""
from __future__ import annotations

import os
import random
import selectors
import socket
import sys
import time

HOST = "127.0.0.1"
PORT_A = 14001
PORT_B = 14002
PORT_CTL = 14003
LOG_PATH = os.path.join("sim", "traffic.log")
MAX_LINE_BYTES = 64


def ts():
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()) + \
        ".%03d" % (int((time.time() % 1) * 1000),)


def fmt(data: bytes) -> str:
    n = len(data)
    shown = data[:MAX_LINE_BYTES]
    hexs = " ".join("%02x" % b for b in shown)
    asc = "".join(chr(b) if 32 <= b < 127 else "." for b in shown)
    more = "" if n <= MAX_LINE_BYTES else " (+%d more)" % (n - MAX_LINE_BYTES,)
    return "%d bytes: %s |%s|%s" % (n, hexs, asc, more)


class FaultPolicy:
    """Per-direction drop/corrupt probabilities and one-shot drop counters."""

    def __init__(self):
        self.drop_pct = {"A": 0, "B": 0}       # 0..100
        self.corrupt_pct = {"A": 0, "B": 0}
        self.dropn = {"A": 0, "B": 0}          # remaining chunks to drop
        self.stats = {
            "forwarded": 0, "dropped": 0, "corrupted": 0,
        }

    def clear(self):
        self.drop_pct = {"A": 0, "B": 0}
        self.corrupt_pct = {"A": 0, "B": 0}
        self.dropn = {"A": 0, "B": 0}

    def summary(self) -> str:
        return (
            "drop A=%d%% B=%d%% corrupt A=%d%% B=%d%% dropn A=%d B=%d "
            "stats fwd=%d drop=%d corr=%d"
            % (self.drop_pct["A"], self.drop_pct["B"],
               self.corrupt_pct["A"], self.corrupt_pct["B"],
               self.dropn["A"], self.dropn["B"],
               self.stats["forwarded"], self.stats["dropped"],
               self.stats["corrupted"])
        )

    def apply(self, side: str, data: bytes) -> bytes | None:
        """Return payload to forward, or None to drop the chunk."""
        if self.dropn[side] > 0:
            self.dropn[side] -= 1
            self.stats["dropped"] += 1
            return None
        if self.drop_pct[side] > 0 and random.randrange(100) < self.drop_pct[side]:
            self.stats["dropped"] += 1
            return None
        out = data
        if self.corrupt_pct[side] > 0 and random.randrange(100) < self.corrupt_pct[side]:
            ba = bytearray(out)
            if ba:
                i = random.randrange(len(ba))
                ba[i] ^= 0x01 << random.randrange(8)
            out = bytes(ba)
            self.stats["corrupted"] += 1
        self.stats["forwarded"] += 1
        return out


def main():
    os.makedirs(os.path.dirname(LOG_PATH) or ".", exist_ok=True)
    log = open(LOG_PATH, "a", buffering=1)
    log.write("%s broker starting (A=:%d B=:%d ctl=:%d)\n"
              % (ts(), PORT_A, PORT_B, PORT_CTL))

    sel = selectors.DefaultSelector()
    peers = {"A": None, "B": None}
    policy = FaultPolicy()
    ctl_bufs = {}  # sock -> bytearray

    def logline(msg):
        line = "%s %s" % (ts(), msg)
        log.write(line + "\n")
        print(line, flush=True)

    for port, side in ((PORT_A, "A"), (PORT_B, "B")):
        ls = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ls.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        ls.bind((HOST, port))
        ls.listen(2)
        ls.setblocking(False)
        sel.register(ls, selectors.EVENT_READ, data=("listen", side))

    ctl_ls = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ctl_ls.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    ctl_ls.bind((HOST, PORT_CTL))
    ctl_ls.listen(4)
    ctl_ls.setblocking(False)
    sel.register(ctl_ls, selectors.EVENT_READ, data=("ctl_listen", None))

    def handle_ctl_line(line: str) -> str:
        parts = line.strip().split()
        if not parts:
            return "ERR empty"
        cmd = parts[0].lower()
        if cmd in ("help", "?"):
            return ("OK commands: none | status | drop A|B|both N | "
                    "corrupt A|B|both N | dropn A|B N | help")
        if cmd in ("none", "clear", "off"):
            policy.clear()
            return "OK " + policy.summary()
        if cmd == "status":
            return "OK " + policy.summary()
        if cmd in ("drop", "corrupt") and len(parts) >= 3:
            who = parts[1].upper()
            try:
                pct = int(parts[2])
            except ValueError:
                return "ERR bad percent"
            if pct < 0 or pct > 100:
                return "ERR percent 0-100"
            sides = ["A", "B"] if who == "BOTH" else [who]
            if any(s not in ("A", "B") for s in sides):
                return "ERR side A|B|both"
            table = policy.drop_pct if cmd == "drop" else policy.corrupt_pct
            for s in sides:
                table[s] = pct
            return "OK " + policy.summary()
        if cmd == "dropn" and len(parts) >= 3:
            who = parts[1].upper()
            if who not in ("A", "B"):
                return "ERR side A|B"
            try:
                n = int(parts[2])
            except ValueError:
                return "ERR bad count"
            if n < 0 or n > 100000:
                return "ERR count range"
            policy.dropn[who] = n
            return "OK " + policy.summary()
        return "ERR unknown (try help)"

    logline("listening")
    try:
        while True:
            for key, _ in sel.select(timeout=1.0):
                kind = key.data[0]

                if kind == "listen":
                    side = key.data[1]
                    conn, addr = key.fileobj.accept()
                    conn.setblocking(False)
                    old = peers[side]
                    if old is not None:
                        try:
                            sel.unregister(old)
                        except Exception:
                            pass
                        old.close()
                    peers[side] = conn
                    sel.register(conn, selectors.EVENT_READ, data=("peer", side))
                    logline("%s connected from %s:%d" % (side, addr[0], addr[1]))

                elif kind == "ctl_listen":
                    conn, addr = key.fileobj.accept()
                    conn.setblocking(False)
                    ctl_bufs[conn] = bytearray()
                    sel.register(conn, selectors.EVENT_READ, data=("ctl", None))
                    logline("ctl connected from %s:%d" % (addr[0], addr[1]))

                elif kind == "ctl":
                    conn = key.fileobj
                    try:
                        data = conn.recv(1024)
                    except OSError:
                        data = b""
                    if not data:
                        sel.unregister(conn)
                        conn.close()
                        ctl_bufs.pop(conn, None)
                        continue
                    buf = ctl_bufs.setdefault(conn, bytearray())
                    buf.extend(data)
                    while True:
                        buf = ctl_bufs.get(conn, bytearray())
                        if b"\n" not in buf:
                            break
                        line_b, _, rest = bytes(buf).partition(b"\n")
                        ctl_bufs[conn] = bytearray(rest)
                        line = line_b.decode("utf-8", errors="replace").strip("\r")
                        reply = handle_ctl_line(line)
                        logline("ctl: %r -> %s" % (line, reply))
                        try:
                            conn.sendall((reply + "\n").encode("utf-8"))
                        except OSError:
                            pass

                else:  # peer
                    side = key.data[1]
                    conn = key.fileobj
                    try:
                        data = conn.recv(4096)
                    except OSError:
                        data = b""
                    if not data:
                        logline("%s disconnected" % side)
                        try:
                            sel.unregister(conn)
                        except Exception:
                            pass
                        conn.close()
                        peers[side] = None
                        continue
                    other = "B" if side == "A" else "A"
                    dest = peers[other]
                    out = policy.apply(side, data)
                    if out is None:
                        logline("%s->%s DROP %s" % (side, other, fmt(data)))
                        continue
                    if out is not data:
                        logline("%s->%s CORRUPT %s" % (side, other, fmt(out)))
                    else:
                        logline("%s->%s %s" % (side, other, fmt(out)))
                    if dest is not None:
                        try:
                            dest.sendall(out)
                        except OSError as e:
                            logline("forward %s->%s failed: %s" % (side, other, e))
    except KeyboardInterrupt:
        pass
    logline("broker stopping")
    log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
