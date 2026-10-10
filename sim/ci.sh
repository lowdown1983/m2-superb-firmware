#!/usr/bin/env bash
# Local mirror of .github/workflows/sim-ci.yml — same gate, no cloud required.
# Usage (from project root):  ./sim/ci.sh
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

if ! command -v pio >/dev/null 2>&1; then
  echo "pio not on PATH — install PlatformIO (pip install platformio)" >&2
  exit 1
fi
if ! python3 -c "import pytest" 2>/dev/null; then
  echo "pytest missing — pip install pytest" >&2
  exit 1
fi

echo "==> pio run -e sim_m2 -e sim_superb"
pio run -e sim_m2 -e sim_superb

echo "==> pytest sim/tests/"
python3 -m pytest sim/tests/ -v --tb=short

echo "OK: sim CI gate passed"
