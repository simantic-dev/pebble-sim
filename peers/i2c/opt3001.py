# TI OPT3001 ambient light sensor at 7-bit 0x44 (Pebble 2 Duo, TWIM0) as a scripted I2C peer.
# Source: OPT3001 datasheet SBOS681C (Nov 2017), "DS"; section and table numbers as printed.
# Registers (DS 7.6, Table 6), 16 bit, MSB first: 00h RESULT (E[15:12], R[11:0]; lux = 0.01 * 2^E * R, Tables 7-8,
# Equations 1-2), 01h CONFIGURATION (reset C810h, Table 10), 02h LOW LIMIT, 03h HIGH LIMIT (reset BFFFh), 7Eh MANUFACTURER ID
# 5449h, 7Fh DEVICE ID 3001h. The register pointer is kept between transactions (DS 7.5.1).
#
# Modelled
#  - Conversion time: integration 100 ms (CT = 0) or 800 ms (CT = 1) plus 3 ms until the data are ready (DS 6.5 note 5);
#    in automatic full-scale mode (RN = 1100b) a conversion started by a configuration write is preceded by the 10 ms
#    range-assessment measurement (DS 7.4.1).
#  - M[1:0] (Table 10): 00 shutdown, 01 single-shot (reads 01 while converting, returns to 00 when the conversion
#    completes), 10 / 11 continuous (back-to-back conversions). A configuration write aborts a conversion in progress and
#    starts a new one if M is not 00.
#  - RESULT holds the previous conversion until the new one completes, so the first read after a single-shot write returns
#    the old result (0 after reset), exactly as the silicon does; PebbleOS's single-shot read sees that value once.
#  - CRF (bit 7): set when a conversion completes, cleared by a configuration read or by a write with M != 00 (Table 10);
#    OVF (bit 8) when the light exceeds the programmed full scale (any range in manual mode, the whole device in auto mode).
#  - Range: automatic (RN = 1100b) picks the smallest range that holds the light; manual RN < 1100b uses that range and
#    clamps; ME masks the exponent to 0 in manual mode (Table 10). RESULT mantissa = round(lux / LSB), LSB from Table 8.
#  - General-call reset (address 0, byte 06h, DS 7.5.1.2) restores every register.
#
# Config (JSON): "lux" (default 120.0). The machine can also write the property lux at run time; each conversion samples it
# at its end.
#
# MODELED SIMPLIFICATION: the datasheet leaves the auto-range step heuristic (decrease by one or two settings, abort and
#   re-range on a fast transient) to the device; here the range that just fits is used on every conversion, with no extra
#   assessment time after the first. No noise, 100 ms low-range resolution loss, or spectral response.
# MODELED SIMPLIFICATION: the window comparison and INT pin (L, POL, FC, FH, FL, limit registers, end-of-conversion mode,
#   SMBus alert) are not modelled: the limit registers store, FH / FL read 0, INT is not driven.
# NOTE: DS 7.6.1.1.3 prints the low-limit reset as C0000h, which does not fit 16 bits; Table 11 gives 0 for both fields.
import math

RESULT, CONFIG, LOW, HIGH, MFG, DEV = 0x00, 0x01, 0x02, 0x03, 0x7E, 0x7F
CT, M_SHIFT, OVF, CRF, ME = 0x0800, 9, 0x0100, 0x0080, 0x0004
RESET = {RESULT: 0x0000, CONFIG: 0xC810, LOW: 0x0000, HIGH: 0xBFFF, MFG: 0x5449, DEV: 0x3001}
READ_ONLY = {RESULT, MFG, DEV}
AUTO = 0xC
INT_US = {0: 100_000, 1: 800_000}      # DS 6.5 total integration time, CT = 0 / 1
READY_US = 3_000                       # DS 6.5 note 5
ASSESS_US = 10_000                     # DS 7.4.1


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        cfg = getattr(ctx, "config", None) or {}
        self.lux = float(cfg.get("lux", 120.0))
        self._reset()

    def _reset(self):
        self.regs = dict(RESET)
        self.ptr, self.first, self.hi = 0, True, False
        self.conv_end = None
        self.ctx.cancel_timer()

    def set_property(self, name, value):
        self.lux = float(value)

    def get_property(self, name):
        return self.lux

    # ---- conversion -----------------------------------------------------------------------------------
    def _mode(self):
        return (self.regs[CONFIG] >> M_SHIFT) & 3

    def _conv_us(self):
        return INT_US[1 if self.regs[CONFIG] & CT else 0] + READY_US

    def _start(self, first):
        cfg = self.regs[CONFIG]
        assess = ASSESS_US if first and (cfg >> 12) == AUTO else 0
        self.conv_end = (self.ctx.now_us if first else self.conv_end) + assess + self._conv_us()
        self.ctx.schedule_oneshot(max(1, math.ceil(self.conv_end - self.ctx.now_us)))

    def _catch_up(self):
        while self.conv_end is not None and self.conv_end <= self.ctx.now_us:
            self._finish()

    def _finish(self):
        cfg, lux = self.regs[CONFIG], self.lux
        rn = cfg >> 12
        e = next((e for e in range(12) if lux <= 40.95 * 2 ** e), 11) if rn == AUTO else min(rn, 11)
        r = round(lux / (0.01 * 2 ** e))
        over = r > 0xFFF
        r = min(r, 0xFFF)
        shown_e = 0 if (cfg & ME and rn < AUTO) else e
        self.regs[RESULT] = (shown_e << 12) | r
        cfg = (cfg & ~OVF) | CRF | (OVF if over else 0)
        if self._mode() == 1:
            cfg &= ~(3 << M_SHIFT)                      # single-shot: back to shutdown
        self.regs[CONFIG] = cfg
        if self._mode() == 0:
            self.conv_end = None
            self.ctx.cancel_timer()
        else:
            self._start(first=False)

    def on_timer(self):
        self._catch_up()

    # ---- host interface -------------------------------------------------------------------------------
    def on_address(self, addr, is_read):
        self.first, self.hi = not is_read, False
        self.general_call = addr == 0
        return True

    def on_write(self, data):
        self._catch_up()
        if getattr(self, "general_call", False):
            if data[:1] == b"\x06":
                self._reset()
            return len(data)
        vals = []
        for b in data:
            if self.first:
                self.ptr, self.first = b, False
            else:
                vals.append(b)
        if len(vals) >= 2 and self.ptr in RESET and self.ptr not in READ_ONLY:
            self._write(self.ptr, (vals[0] << 8) | vals[1])
        return len(data)

    def on_read(self, count):
        self._catch_up()
        out = bytearray()
        for _ in range(count):
            v = self.regs.get(self.ptr, 0)
            out.append(v & 0xFF if self.hi else v >> 8)
            if self.hi and self.ptr == CONFIG:
                self.regs[CONFIG] &= ~CRF                # a configuration read clears CRF
            self.hi = not self.hi
        return bytes(out)

    def _write(self, a, v):
        if a == LOW or a == HIGH:
            self.regs[a] = v
            return
        keep = self.regs[CONFIG] & (OVF | CRF)           # OVF, CRF, FH, FL are read only
        self.regs[CONFIG] = (v & ~(OVF | CRF | 0x0060)) | (keep if (v >> M_SHIFT) & 3 == 0 else keep & ~CRF)
        self.conv_end = None                             # a write aborts a conversion in progress
        self.ctx.cancel_timer()
        if self._mode():
            self._start(first=True)
