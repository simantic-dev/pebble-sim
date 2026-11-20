#!/usr/bin/env bash
# Build this PebbleOS tree for the Pebble 2 Duo (board "asterix") and fetch Core Devices' shipping asterix
# bootloader. Writes out/pebbleos.elf, out/resources.bin (system_resources.pbpack), out/boot.bin and
# out/log.dict.json (for scripts/dehash.py). Needs git, python3 and the PebbleOS SDK
# (https://github.com/coredevices/PebbleOS-SDK; set PEBBLEOS_SDK to its install dir).
set -euo pipefail
cd "$(dirname "$0")/.."
BOOT_COMMIT=334bc2f
mkdir -p out
git submodule update --init --recursive -q
. "${PEBBLEOS_SDK:?set PEBBLEOS_SDK to the PebbleOS SDK install dir}/env.sh"
[ -d .venv ] || python3 -m venv .venv
. .venv/bin/activate
pip install -q -r requirements.txt
pbl configure --board asterix
pbl build
cp build/pebbleos.elf out/pebbleos.elf
cp build/system_resources.pbpack out/resources.bin
cp build/log.dict.json out/log.dict.json
if [ ! -d out/lboot ]; then
  git clone -q https://github.com/coredevices/PebbleOS-legacy-bootloaders.git out/lboot
fi
git -C out/lboot checkout -q "$BOOT_COMMIT"
cp "out/lboot/bin/boot/boot_asterix@1753450013.bin" out/boot.bin
