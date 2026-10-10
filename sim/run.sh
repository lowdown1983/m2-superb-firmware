#!/usr/bin/env bash
# sim/run.sh — interactive sim session (docs/SIM.md).
#
# Builds both sim envs, starts the Bee broker, starts the SuperB sim in the
# background, then attaches the M2 sim's SerialUSB console to your terminal.
# Type 'help' at the M2 console. Ctrl-C (or EOF) stops everything.
#
# What it proves (Marcus's definition of done):
#   1. M2 firmware is present and running  -> banner + 'status' on this console
#   2. SuperB firmware is present/running -> http://127.0.0.1:18080
#   3. M2<->SuperB talk over the Bee link -> SuperB web GUI shows the M2's
#      real mode via a live F1 7E CMD_GET_STATUS round-trip.
set -u

cd "$(dirname "$0")/.."   # project root
mkdir -p sim sim/nvs

echo "== building sim envs =="
if ! pio run -e sim_m2 -e sim_superb; then
    echo "BUILD FAILED — see output above. Bazzite setup: docs/SIM.md."
    exit 1
fi

M2_BIN=.pio/build/sim_m2/program
SB_BIN=.pio/build/sim_superb/program

echo "== starting Bee broker (14001/14002, log: sim/traffic.log) =="
python3 sim/broker.py > sim/broker.log 2>&1 &
BROKER_PID=$!

echo "== starting SuperB sim (web: http://127.0.0.1:18080) =="
"$SB_BIN" > sim/superb.log 2>&1 &
SB_PID=$!

cleanup() {
    kill "$BROKER_PID" "$SB_PID" 2>/dev/null
    wait 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "== waiting for sims to come up =="
python3 - <<'EOF'
import socket, sys, time
need = [(14001, "broker:A"), (14002, "broker:B"), (18080, "web GUI"),
        (14023, "TCP:23 bridge"), (14024, "BT SPP shim")]
deadline = time.time() + 30
ok = set()
while time.time() < deadline and len(ok) < len(need):
    for port, name in need:
        if port in ok:
            continue
        s = socket.socket()
        s.settimeout(0.5)
        try:
            s.connect(("127.0.0.1", port))
            ok.add(port)
            print("  up: %s (:%d)" % (name, port), flush=True)
        except OSError:
            pass
        finally:
            s.close()
    time.sleep(0.5)
missing = [n for p, n in need if p not in ok]
if missing:
    print("TIMEOUT waiting for: %s" % ", ".join(missing))
    print("Check sim/broker.log and sim/superb.log")
    sys.exit(1)
print("all sims up")
EOF

echo
echo "== M2 console attached — type 'help', try 'status', 'mode list' =="
echo "   (SuperB web GUI: http://127.0.0.1:18080  |  traffic: sim/traffic.log)"
echo
"$M2_BIN"
