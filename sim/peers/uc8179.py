# UltraChip UC8179/UC8279 e-paper controller (800x600 RAM, 1 bpp, 480 visible rows at gate 120..599) for the Xteink X4 Pro.
# One SPI slave on spi2 (one byte per transaction, as EpdBus does) that also answers the firmware's bit-banged
# controller probe on the SPI pads. Lines: 1 = DC (GPIO18, in), 2 = BUSY (GPIO6, out, high = idle),
# 3 = probe/SPI clock pad (GPIO12, in), 4 = probe data pad (GPIO11, in), 5 = probe data pad (GPIO11, out).
# Probe: after 8 command bits (DC low) and DC rising, reg 0x71 returns 1 byte (0x13) and reg 0x70 returns
# 00 00 <id> FF FF, MSB first, next bit on each clock falling edge. config {"id": 1} sets <id> (default = the real
# unit's answer, which PebbleOS rejects with -ENOTSUP; 0x41/0x42 are accepted).
# Panel: 0x10 old / 0x13 new data at the partial window (0x91 in, 0x90 window, 0x92 out; else full 800x600, x then y),
# 0x12 refresh dumps the NEW plane rows 120..599 to <EPD_DUMP>-NNN.pbm (800x480, RAM bit 1 = white, PBM inverted;
# same as ssd1677.py, convert with frame2png.py --transpose).
# BUSY (low = busy) is a declared constant: 1.6 s refresh, 150 ms power on/off.
# Not modelled: waveform/LUT banks, old-vs-new differential drive, grey, temperature/OTP, power state, 0x70/0x71 outside the probe.
import os
import sys

W, H, WB, ROW0, VIS = 800, 600, 100, 120, 480
DC, BUSY, CLK, SDA_IN, SDA_OUT = 1, 2, 3, 4, 5
FULL_US, POWER_US = 1_600_000, 150_000


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        self.id = int(getattr(ctx, "config", {}).get("id", 0x01))
        self.dc = True
        self.sda = False
        self.clk = False
        self.cmd = 0
        self.n = 0
        self.ram = {0x10: bytearray(b"\xff" * (WB * H)), 0x13: bytearray(b"\xff" * (WB * H))}
        self.partial = False
        self.win = [0, W - 1, 0, H - 1]
        self.x = self.y = 0
        self.frames = 0
        self.path = os.environ.get("EPD_DUMP", "sim/out/epd")
        self.pcmd = self.pbits = 0     # probe: command being shifted in / count of bits
        self.rx = []                   # probe: bits still to present
        ctx.gpio(BUSY).set(True)

    def _reset_window(self):
        self.x, self.y = self.win[0], self.win[2]

    def on_gpio(self, line, level):
        level = bool(level)
        if line == SDA_IN:
            self.sda = level
        elif line == CLK:
            rising = level and not self.clk
            self.clk = level
            if rising and not self.dc:
                self.pcmd = ((self.pcmd << 1) | self.sda) & 0xFF
                self.pbits += 1
            elif not level and self.rx:
                self.rx.pop(0)
                self._drive()
        elif line == DC:
            was, self.dc = self.dc, level
            if was and not level:
                self.pbits = 0
            elif level and not was and self.pbits >= 8 and self.pcmd in (0x70, 0x71):  # last 8 bits: CS frames it on glass
                data = [0x13] if self.pcmd == 0x71 else [0, 0, self.id, 0xFF, 0xFF]
                self.rx = [(b >> (7 - i)) & 1 for b in data for i in range(8)]
                self._drive()
            if level and not self.rx:
                self.ctx.gpio(SDA_OUT).release()

    def _drive(self):
        pin = self.ctx.gpio(SDA_OUT)
        if self.rx:
            pin.set(bool(self.rx[0]))
        else:
            pin.release()

    def on_transmit(self, v):
        if not self.dc:
            self.cmd, self.n = v, 0
            if v == 0x91: self.partial = True
            elif v == 0x92:
                self.partial = False
                self.win = [0, W - 1, 0, H - 1]
            elif v == 0x12: self._refresh()
            elif v in (0x02, 0x04): self._busy(POWER_US)
            elif v in (0x10, 0x13): self._reset_window()
            return 0
        c, n = self.cmd, self.n
        self.n += 1
        if c in self.ram:
            if 0 <= self.y < H and 0 <= self.x < WB * 8:
                self.ram[c][self.y * WB + self.x // 8] = v
            self.x += 8
            if self.x > self.win[1]:
                self.x = self.win[0]
                self.y += 1
                if self.y > self.win[3]:
                    self.y = self.win[2]
        elif c == 0x90 and n < 8:
            self.p = getattr(self, "p", [0] * 8)
            self.p[n] = v
            if n == 7:
                p = self.p
                self.win = [(p[0] << 8 | p[1]) & ~7, p[2] << 8 | p[3], p[4] << 8 | p[5], p[6] << 8 | p[7]]
                self._reset_window()
        return 0

    def _busy(self, us):
        self.ctx.gpio(BUSY).set(False)
        self.ctx.schedule_oneshot(us, self.on_timer)

    def _refresh(self):
        self.frames += 1
        print("EPD frame %d at %.3f s" % (self.frames, getattr(self.ctx, "now_us", 0) / 1e6), file=sys.stderr)
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        with open("%s-%03d.pbm" % (self.path, self.frames), "wb") as f:
            f.write(b"P4\n%d %d\n" % (W, VIS))
            f.write(bytes(b ^ 0xFF for b in self.ram[0x13][ROW0 * WB:(ROW0 + VIS) * WB]))
        self._busy(FULL_US)

    def on_timer(self):
        self.ctx.gpio(BUSY).set(True)
