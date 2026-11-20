#!/usr/bin/env python3
"""Filter: reads `sim run --trace-interrupts` output on stdin, counts exception entries per IRQ number in 1 ms bins of
virtual time, drops the (IRQ) lines (a full trace is ~1 GB) and passes every other line through. Writes the compact
table {"bin_s": 0.001, "bins": {irq: {bin: entries}}} to argv[1] at the end. A temporary input for demo_gif.py: not kept."""
import collections, json, re, sys

T = re.compile(r"^\[([\d.]+)s\] \(IRQ\) enter (\S+)")
bins = collections.defaultdict(lambda: collections.defaultdict(int))
for l in sys.stdin:
    if "(IRQ)" not in l:
        sys.stdout.write(l)
        continue
    m = T.match(l)
    if m:
        bins[m[2]][int(float(m[1]) * 1000)] += 1
json.dump({"bin_s": 0.001, "bins": bins}, open(sys.argv[1], "w"))
