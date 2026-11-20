#!/usr/bin/env bash
# Build PebbleOS (pinned commit) if out/ has no image, run the Pebble 2 Duo on pyrite for 34 virtual s,
# write the display's frames to out/frames/, then the GIF and MP4. SIM=/path/to/sim picks the sim binary.
# --trace-interrupts is aggregated live by scripts/irq_bins.py (1 ms bins) into out/irq_rates.json for the IRQ panel;
# the raw trace (~1 GB) is never written.
set -euo pipefail
cd "$(dirname "$0")"
SIM="${SIM:-sim}"
[ -f out/pebbleos.elf ] || scripts/build_pebbleos.sh
rm -rf out/frames && mkdir -p out/frames
"$SIM" run --scenario sim/scenario.yaml --ascii --trace-interrupts 2>&1 | python3 scripts/irq_bins.py out/irq_rates.json | tee out/run.log
ls out/frames | wc -l | xargs echo "frames written to out/frames:"
python3 scripts/demo_gif.py docs/screens/demo.gif mp4   # also docs/screens/demo.mp4 (needs ffmpeg)
