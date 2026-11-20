#!/usr/bin/env python3
"""An original, generic rectangular smartwatch body drawn with Pillow around the 144x168 display, one side button lit
when pressed. Pebble 2 Duo layout: Back on the left, Up / Select / Down on the right. No product artwork or logo.
watch(frame, pressed, r): `frame` is the PNG from out/frames, `pressed` a key of BUTTONS or None, r the output scale."""
from PIL import Image, ImageDraw, ImageFilter

W, H = 420, 600  # design units; the display is 2x its pixels (288 x 336) at r = 1
ACCENT = (255, 150, 40)
# name: (side, y centre, label); the key is the overlay.board `btn` line number 0..3 (P0.28..P0.31)
BUTTONS = {0: ("L", 300, "Back"), 1: ("R", 210, "Up"), 2: ("R", 300, "Select"), 3: ("R", 390, "Down")}
CASE = (45, 100, 375, 500)
_cache = {}


def _body(pressed, r):
    k = 2 * r  # draw at 2x the output and downsample
    S = lambda *v: tuple(x * k for x in v)
    im = Image.new("RGB", (round(W * k), round(H * k)), (236, 238, 241))
    d = ImageDraw.Draw(im)
    for box in ((120, 0, 300, 120), (120, 480, 300, 600)):  # straps
        d.rectangle(S(*box), fill=(52, 55, 61))
    for y in (540, 566):
        d.ellipse(S(204, y - 5, 216, y + 5), fill=(20, 21, 24))
    for n, (side, cy, _) in BUTTONS.items():
        on = n == pressed
        x0, x1 = (31, 49) if side == "L" else (371, 389)
        if on:  # pressed: sits 5 units deeper, lit with a soft glow
            x0, x1 = (x0 + 5, x1 + 5) if side == "L" else (x0 - 5, x1 - 5)
            glow = Image.new("L", im.size, 0)
            ImageDraw.Draw(glow).rounded_rectangle(S(x0 - 8, cy - 38, x1 + 8, cy + 38), round(12 * k), fill=255)
            im.paste(Image.new("RGB", im.size, ACCENT), mask=glow.filter(ImageFilter.GaussianBlur(9 * k)).point(lambda v: min(255, v * 3)))
            d = ImageDraw.Draw(im)
        d.rounded_rectangle(S(x0, cy - 36, x1, cy + 36), round(6 * k), fill=ACCENT if on else (84, 88, 96))
    d.rounded_rectangle(S(*CASE), round(44 * k), fill=(34, 36, 41), outline=(14, 15, 17), width=round(2 * k))
    d.rounded_rectangle(S(CASE[0] + 9, CASE[1] + 9, CASE[2] - 9, CASE[3] - 9), round(38 * k), fill=(14, 15, 17))
    return im.resize((round(W * r), round(H * r)), Image.LANCZOS)


def watch(frame, pressed=None, r=1):
    if (pressed, r) not in _cache:
        _cache[pressed, r] = _body(pressed, r)
    im = _cache[pressed, r].copy()
    sz = (round(288 * r), round(336 * r))
    im.paste(frame.convert("RGB").resize(sz, Image.NEAREST), (round(66 * r), round(132 * r)))
    return im


if __name__ == "__main__":
    for n in (None, 0, 2):
        watch(Image.new("RGB", (144, 168), (200, 205, 190)), n).save(f"/tmp/watch_{n}.png")
