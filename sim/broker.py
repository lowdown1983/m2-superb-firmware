#!/usr/bin/env python3
"""Bee-link broker: bridges sim_m2 <-> sim_superb and logs every byte.

Listens on 127.0.0.1:14001 (sim_m2's Bee Serial) and 127.0.0.1:14002
(sim_superb's Bee Serial). Forwards bytes both ways, verbatim, and appends
timestamped hex/ascii lines to sim/traffic.log — the primary debugging
artifact for the sim harness (docs/SIM.md).

Run from the project root:  python3 sim/broker.py
"""
import os
import selectors
import socket
import sys
import time

HOST = "127.0.0.1"
PORT_A = 14001  # sim_m2 Bee Serial
PORT_B = 14002  # sim_superb Bee Serial
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


def main():
    os.makedirs(os.path.dirname(LOG_PATH) or ".", exist_ok=True)
    log = open(LOG_PATH, "a", buffering=1)
    log.write("%s broker starting (A=:%d B=:%d)\n" % (ts(), PORT_A, PORT_B))

    sel = selectors.DefaultSelector()
    sides = {}  # listener socket -> 'A'/'B'
    peers = {"A": None, "B": None}  # side -> client socket

    for port, side in ((PORT_A, "A"), (PORT_B, "B")):
        ls = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ls.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        ls.bind((HOST, port))
        ls.listen(2)
        ls.setblocking(False)
        sel.register(ls, selectors.EVENT_READ, data=("listen", side))
        sides[ls.fileno()] = side

    def logline(msg):
        line = "%s %s" % (ts(), msg)
        log.write(line + "\n")
        print(line, flush=True)

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
                        sel.unregister(old)
                        old.close()
                    peers[side] = conn
                    sel.register(conn, selectors.EVENT_READ, data=("peer", side))
                    logline("%s connected from %s:%d" % (side, addr[0], addr[1]))
                else:
                    side = key.data[1]
                    conn = key.fileobj
                    try:
                        data = conn.recv(4096)
                    except OSError:
                        data = b""
                    if not data:
                        logline("%s disconnected" % side)
                        sel.unregister(conn)
                        conn.close()
                        peers[side] = None
                        continue
                    other = "B" if side == "A" else "A"
                    dest = peers[other]
                    logline("%s->%s %s" % (side, other, fmt(data)))
                    if dest is not None:
                        try:
                            dest.sendall(data)
                        except OSError as e:
                            logline("forward %s->%s failed: %s" % (side, other, e))
    except KeyboardInterrupt:
        pass
    logline("broker stopping")
    log.close()


if __name__ == "__main__":
    sys.exit(main())
