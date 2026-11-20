#!/usr/bin/env python3
"""Headline GIF: the watch (display frames, pressed button lit) next to the firmware's traffic on one simulated clock, as in
simantic-dev/garmin-fr245-sim. Reads out/frames/*.png (named by virtual us), out/run.log (the phone peer's timestamped lines),
out/irq_rates.json (made by run.sh from --trace-interrupts, binned at 1 ms) and sim/overlay.board (the button script).
Under the traffic rows an IRQ panel: the top 8 interrupts in the trailing 0.5 s (bars, log scale) and total IRQ/s with a cursor.
Usage: python3 scripts/demo_gif.py [out_gif [mp4]]   (mp4: also <out>.mp4, 1920x1080, needs ffmpeg)"""
import glob, json, math, os, re, subprocess, sys
from PIL import Image, ImageChops, ImageDraw, ImageFont
sys.path.insert(0, os.path.dirname(__file__))
from watch_render import BUTTONS, watch

OUT = sys.argv[1] if len(sys.argv) > 1 else "docs/screens/demo.gif"
W, H, WATCH_W, ROWS = 900, 500, 320, 13  # design units of the traffic view: x 0..W, y 0..H; IRQ panel below it
PH = 170  # IRQ panel height under the traffic view
WIN = 0.5  # seconds of trailing window for the bar rates
COL = {"ble": (120, 230, 140), "btn": (255, 150, 40), "i2c": (255, 120, 190), "spi": (110, 200, 255)}
BG, FG, DIM = (18, 20, 24), (225, 225, 225), (130, 135, 145)
NCOL = 128  # GIF palette size (shared)
HOLD = 0.15  # the overlay's for_ms; the lit button follows it


ZOOM = 0.1  # s of history in the per-bin strip
PER = 1000  # bins per second; set from out/irq_rates.json bin_s


def events(irq):
    ev, last = [], {}  # (t, kind, text); last[kind] = time of the last kept row for the rate-limited kinds
    for l in open("out/run.log", errors="replace"):
        m = re.search(r"\[phone\] t=([\d.]+) (.*)", l)
        if not m:
            continue
        t, s = float(m[1]), m[2]
        if s.startswith("ADV_IND"):  # the watch advertises every ~20 ms: one row per 2 s
            if t - last.get("adv", -9) >= 2:
                last["adv"] = t
                ev.append((t, "ble", "air ADV_IND " + s.split(": ", 1)[-1][:46]))
        elif s.startswith("PPoGATT rx"):
            ev.append((t, "ble", "PPoGATT " + s[11:46]))
        else:
            ev.append((t, "ble", "phone: " + s.replace(" at %.3f s" % t, "")[:56]))
    for k, name, kind in (("IRQ3", "TWIM0 sensors bus (I2C)", "i2c"), ("IRQ4", "TWIM1 PMIC bus (I2C)", "i2c"),
                          ("IRQ47", "SPIM3 display refresh (SPI)", "spi"), ("IRQ41", "QSPI NOR flash", "spi")):
        for b in sorted(irq[0].get(k, {})):  # a row per active 1 ms bin, at most one per 0.25 s per bus
            if b / PER - last.get(k, -9) >= 0.25:
                last[k] = b / PER
                ev.append((b / PER, kind, f"{name}: {irq[0][k][b]} IRQ in 1 ms"))
    cfg = json.loads(re.search(r'buttons:.*?config: "(.*?)"\n', open("sim/overlay.board").read(), re.S)[1].replace('\\"', '"'))
    presses = []
    for e in cfg["holds"]:
        presses.append((e["at_ms"] / 1000, e["line"]))
        ev.append((e["at_ms"] / 1000, "btn", f"button {BUTTONS[e['line']][2]} pressed"))
    return sorted(ev, key=lambda e: e[0]), presses


# the chip's own NVIC wiring (pyrite crates/mcu/src/chips/nordic/nrf/nrf52/nrf52840.rs); twi0/twi1 are TWIM0/1, spim3 = SPIM3
IRQ_NAME = {"0": "CLOCK", "1": "RADIO", "2": "UART0", "3": "TWIM0 (sensors)", "4": "TWIM1 (PMIC)", "5": "NFCT", "6": "GPIOTE (buttons)",
            "7": "SAADC", "8": "TIMER0", "9": "TIMER1", "10": "TIMER2", "11": "RTC0", "12": "TEMP", "13": "RNG", "14": "ECB",
            "15": "AAR/CCM", "16": "WDT", "17": "RTC1 (tick)", "18": "QDEC", "19": "COMP", "20": "EGU0", "21": "EGU1", "22": "EGU2",
            "23": "EGU3", "24": "EGU4", "25": "EGU5", "26": "TIMER3", "27": "TIMER4", "28": "PWM0", "29": "PDM", "33": "PWM1",
            "34": "PWM2", "35": "SPIM2", "36": "RTC2", "37": "I2S", "39": "USBD", "40": "UART1", "41": "QSPI (NOR)", "42": "CRYPTOCELL",
            "45": "PWM3", "47": "SPIM3 (display)", "PendSV": "PendSV", "SysTick": "SysTick", "SVCall": "SVCall"}


def load_irq():
    global PER
    j = json.load(open("out/irq_rates.json"))
    PER = round(1 / j["bin_s"])
    b = {k: {int(i): c for i, c in v.items()} for k, v in j["bins"].items()}
    n = max(i for v in b.values() for i in v) + 1
    return b, n


def irq_name(k):
    return IRQ_NAME.get(k) or IRQ_NAME.get(k.removeprefix("IRQ"), k)


def irq_panel(d, irq, t, fnt, pw=W):
    bins, n = irq
    y0, x0 = H, 12
    d.line((0, y0, pw, y0), fill=(60, 64, 72))
    lo, hi = max(0, int((t - WIN) * PER)), int(t * PER) + 1
    rate = sorted(((sum(v.get(i, 0) for i in range(lo, hi)) / ((hi - lo) / PER), k) for k, v in bins.items()), reverse=True)[:8]
    rate = [r for r in rate if r[0]]
    d.text((x0, y0 + 6), f"IRQ entries/s, last {WIN:g} s (log bars)", font=fnt, fill=DIM)
    for r, (v, k) in enumerate(rate):
        y = y0 + 24 + r * 17
        d.text((x0, y), irq_name(k)[:18], font=fnt, fill=FG)
        d.rectangle((x0 + 150, y + 2, x0 + 150 + 250 * math.log10(1 + v) / 3.5, y + 11), fill=COL["btn"])
        d.text((x0 + 410, y), f"{v:6.0f}", font=fnt, fill=FG)
    sx, sw, sy, sh = 520, pw - 530, y0 + 40, PH - 56
    z = int(ZOOM * PER)  # one bar per bin over the last ZOOM s, so single-bin bursts stay visible
    end = min(int(t * PER), n - 1)
    cnt = [sum(v.get(i, 0) for v in bins.values()) for i in range(end - z + 1, end + 1)]
    top = max(4, max(cnt))
    d.text((sx, y0 + 6), f"IRQs per {1000 / PER:g} ms, last {ZOOM * 1000:g} ms", font=fnt, fill=DIM)
    d.rectangle((sx, sy, sx + sw, sy + sh), outline=(60, 64, 72))
    for k, c in enumerate(cnt):
        if c:
            x = sx + sw * k / z
            d.rectangle((x, sy + sh - sh * c / top, x + max(1, sw / z - 1), sy + sh), fill=(190, 150, 255))
    d.text((sx, sy + sh + 4), f"-{ZOOM * 1000:g} ms", font=fnt, fill=DIM); d.text((sx + sw - 30, sy + sh + 4), "now", font=fnt, fill=DIM)

class UD:
    """ImageDraw in design units: coordinates and widths x u, shifted by (ox, oy) pixels (HD output draws the same layout at u x)."""

    def __init__(self, d, u=1.0, ox=0, oy=0):
        self.d, self.u, self.ox, self.oy = d, u, ox, oy

    def p(self, x, y):
        return (self.ox + x * self.u, self.oy + y * self.u)

    def text(self, xy, s, **k):
        self.d.text(self.p(*xy), s, **k)

    def rectangle(self, b, width=1, **k):
        self.d.rectangle(self.p(b[0], b[1]) + self.p(b[2], b[3]), width=max(1, round(width * self.u)), **k)

    def line(self, pts, width=1, **k):
        pts = [pts[i:i + 2] for i in range(0, len(pts), 2)] if isinstance(pts[0], (int, float)) else pts
        self.d.line([self.p(*q) for q in pts], width=max(1, round(width * self.u)), **k)


def font(sz):
    for p in ("/System/Library/Fonts/Menlo.ttc", "/System/Library/Fonts/Monaco.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"):
        if os.path.exists(p):
            return ImageFont.truetype(p, sz)
    return ImageFont.load_default(sz)


def ticks_for(ev, presses, ft):
    """Candidate ticks: display frames, events, button edges; one per 0.35 s of simulated time, plus every button edge and the last display frame."""
    t1 = ft[-1] + 0.2
    edges = {p for p, _ in presses} | {p + HOLD for p, _ in presses}
    cand = sorted({t for t, _, _ in ev if 0 <= t <= t1} | set(ft) | edges | {0.0} | {float(x) for x in range(0, int(t1))})
    ticks, last = [], -9
    for t in cand:
        if t - last >= 0.35 or t in edges or t == ft[-1]:
            ticks.append(t); last = t
    return ticks, t1


def panel(d, u, x0, pw, t, shown, irq):
    """The traffic view and IRQ panel in design units on UD d: text starts at x0, right edge pw."""
    fnt, big = font(round(12 * u)), font(round(17 * u))
    d.text((x0, 10), f"t = {t:.2f} s (simulated)", font=big, fill=FG)
    d.text((x0, 34), "PebbleOS on the nRF52840: buttons, BLE, I2C, SPI", font=fnt, fill=DIM)
    d.line((x0, 54, pw - 8, 54), fill=(60, 64, 72))
    for i, (t_e, k, text) in enumerate(shown[-ROWS:]):
        y = 62 + i * 29
        newest = i == len(shown[-ROWS:]) - 1
        if newest:
            d.rectangle((x0 - 4, y - 3, pw - 6, y + 25), fill=(44, 50, 62))
        d.text((x0, y), f"{t_e:7.3f}s", font=fnt, fill=DIM)
        d.text((x0 + 62, y), text[:68], font=fnt, fill=COL[k])
        if newest:
            d.rectangle((x0 - 4, y - 3, x0 - 1, y + 25), fill=COL[k])
    lx = x0
    for k, name in (("btn", "button"), ("ble", "BLE"), ("i2c", "I2C"), ("spi", "SPI / QSPI")):
        d.rectangle((lx, H - 22, lx + 9, H - 13), fill=COL[k]); d.text((lx + 14, H - 24), name, font=fnt, fill=DIM)
        lx += 14 + 7 * len(name) + 14
    irq_panel(d, irq, t, fnt, pw)


def main():
    irq = load_irq()
    ev, presses = events(irq)
    full = H + PH
    fs = sorted(glob.glob("out/frames/*.png"))
    ft = [int(os.path.basename(f)[:-4]) / 1e6 for f in fs]
    ticks, t1 = ticks_for(ev, presses, ft)
    mp4 = len(sys.argv) > 2 and sys.argv[2] == "mp4"
    for hd in (False, True):
        if hd and not mp4:
            break
        frames, durs, fi, shown, prev_fi, ei, crop = [], [], 0, [], None, 0, None
        if hd:
            land, r = (1920, 1080), 1.9  # watch 798 x 1140, cropped to the height
            pl = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", "1920x1080", "-r", "30", "-i", "-",
                                   "-c:v", "libx264", "-preset", "slow", "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                                   os.path.splitext(OUT)[0] + ".mp4"], stdin=subprocess.PIPE)
        for k, t in enumerate([ticks[-1]] + ticks):  # tick 0: the final screen first, as a spoiler, then the run from 0 s
            if k == 1:
                fi, shown, prev_fi, ei = 0, [], None, 0
            while ei < len(ev) and ev[ei][0] <= t:
                shown.append(ev[ei]); ei += 1
            while fi + 1 < len(fs) and ft[fi + 1] <= t:
                fi += 1
            if t < ft[0]:
                fi = -1
            pressed = next((n for p, n in presses if p <= t < p + HOLD), None)
            frame = Image.open(fs[max(fi, 0)]) if fi >= 0 else Image.new("RGB", Image.open(fs[0]).size)
            changed = fi != prev_fi; prev_fi = fi
            dur = 2500 if k in (0, len(ticks)) else 300 if changed else 110  # hold the spoiler and the final screen
            if not hd:
                w = watch(frame, pressed, WATCH_W / 420)
                im = Image.new("RGB", (W, full), BG)
                im.paste(w, (0, (H - w.height) // 2))
                panel(UD(ImageDraw.Draw(im)), 1.0, WATCH_W + 12, W, t, shown, irq)
                frames.append(im); durs.append(dur)
                continue
            w = watch(frame, pressed, r)
            if crop is None:  # fixed from the unpressed body, so a button glow never moves the panel
                ref = watch(frame, None, r)
                x0, _, x1, _ = ImageChops.difference(ref, Image.new("RGB", ref.size, ref.getpixel((0, 0)))).getbbox()
                crop = (max(0, x0 - 24), 0, min(ref.width, x1 + 72), ref.height)
            w = w.crop(crop)
            u = min((land[0] - w.width) / 720, land[1] / (H + PH + 12))  # fit the 700-unit panel with margins
            im = Image.new("RGB", land, BG)
            im.paste(w.crop((0, (w.height - land[1]) // 2, w.width, (w.height - land[1]) // 2 + land[1])), (0, 0))
            panel(UD(ImageDraw.Draw(im), u, w.width, 0), u, 12, 700, t, shown, irq)
            pl.stdin.write(im.tobytes() * max(1, round(dur * 30 / 1000)))
        if hd:
            pl.stdin.close(); pl.wait()
            continue
        # one shared palette (sampled over the whole run) keeps the text colours stable between frames
        mosaic = Image.new("RGB", (W, full * 12))
        for k in range(12):
            mosaic.paste(frames[k * len(frames) // 12], (0, k * full))
        pal = mosaic.quantize(NCOL, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE)
        frames = [f.quantize(palette=pal, dither=Image.Dither.NONE) for f in frames]
        os.makedirs(os.path.dirname(OUT), exist_ok=True)
        frames[0].save(OUT, save_all=True, append_images=frames[1:], duration=durs, loop=0, optimize=True)
        print(f"{OUT}: {len(frames)} frames, {sum(durs) / 1000:.1f} s, {os.path.getsize(OUT) / 1e6:.2f} MB, sim 0.0-{t1:.1f} s")


if __name__ == "__main__":
    main()
