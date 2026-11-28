#!/bin/sh
# Boot PebbleOS on the simulated Xteink X4. Usage: sim/run.sh [live|ble]  (default: live)
# Uses build-xteink_x4/ if present (pbl build -b build-xteink_x4), else the matching GitHub
# prerelease. Needs `sim` on PATH (or SIM=...), python3 and, for the release, gh.
set -e
cd "$(dirname "$0")/.."
SIM=${SIM:-sim}
MODE=${1:-live}
mkdir -p sim/out
B=build-xteink_x4
if [ ! -f "$B/pebbleos.bin" ]; then
  B=sim/out/release
  [ -f "$B/xteink_x4-pebbleos.bin" ] || {
    gh release download "${RELEASE:-xteink-preview-49be5ee10ba6}" -R ericmigi/PebbleOS -D "$B" -p 'xteink_x4-*' -p SHA256SUMS
    (cd "$B" && shasum -a 256 -c SHA256SUMS --ignore-missing)
  }
  cp "$B/xteink_x4-pebbleos.bin" "$B/pebbleos.bin"
  cp "$B/xteink_x4-system_resources.pbpack" "$B/system_resources.pbpack"
fi
# Same layout extras/xteink/flash.py writes: firmware at 0x0, resources at 0x300000.
python3 - "$B" <<'P'
import sys
b = bytearray(open(f"{sys.argv[1]}/pebbleos.bin", "rb").read())
r = open(f"{sys.argv[1]}/system_resources.pbpack", "rb").read()
b += b"\xff" * (0x300000 - len(b))
open("sim/out/flash.bin", "wb").write(b + r)
P
export SD_IMAGE=/dev/null EPD_DUMP=sim/out/epd
case "$MODE" in
  ble) "$SIM" run --scenario sim/ble.yaml --ascii | grep -E 'pebble>|RPT .*Pebble' | head -20 ;;
  *)
    "$SIM" run --scenario sim/live.yaml --ascii > sim/out/uart.log 2> sim/out/sim.log &
    PID=$!
    trap 'kill $PID 2>/dev/null' EXIT INT TERM
    URL="file://$PWD/sim/reader.html?port=21777"
    (command -v open >/dev/null && open "$URL") || echo "open $URL"
    echo "PebbleOS running (sim pid $PID), console in sim/out/uart.log. Ctrl-C stops."
    wait $PID ;;
esac
