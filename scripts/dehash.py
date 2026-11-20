#!/usr/bin/env python3
# Pipe sim output through PebbleOS's own log dehasher: dehash.py <PebbleOS checkout> <log.dict.json>
# e.g. python3 scripts/dehash.py . out/log.dict.json < out/run.log
import json, sys
sys.path.insert(0, sys.argv[1] + "/tools/libs/pebble-loghash")
from pebble.loghashing.newlogging import dehash_line
d = json.load(open(sys.argv[2]))
for line in sys.stdin:
    sys.stdout.write(dehash_line(line.rstrip("\n"), d) + "\n")
    sys.stdout.flush()
