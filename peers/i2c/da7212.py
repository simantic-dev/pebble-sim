# Dialog DA7212 ultra-low power stereo codec at 7-bit 0x1A as a scripted I2C peer.
# Source: DA7212 datasheet Revision 3.6 (Nov 1, 2024), "DS"; section and table numbers as printed.
# Registers: DS 14 register map; 8-bit address, auto-increment (CIF_CTRL.CIF_I2C_WRITE_MODE = 0, page mode).
#
# Modelled (the clock path the PebbleOS driver drives; no audio data path)
#  - SYSTEM_ACTIVE 0xFD (DS 13.40): 0 = standby (oscillator off), 1 = on. Any other access also asserts it (DS 13.40.2).
#    In standby the PLL is off: PLL_STATUS reads no lock.
#  - CIF_CTRL.CIF_REG_SOFT_RESET (0x1D bit 7, DS 14.1): returns every register to its default (including
#    SYSTEM_ACTIVE = 0, i.e. standby) and self-clears.
#  - PLL (DS 13.26, Tables 33-36): PLL_CTRL 0x27 PLL_EN / PLL_SRM_EN / PLL_INDIV, PLL_INTEGER 0x26 and
#    PLL_FRAC_TOP/BOT 0x24/0x25 (13 fractional bits). F_REF = MCLK / 2^(PLL_INDIV + 1) (DS 13.26.1), VCO = F_REF * FBDIV,
#    system clock = VCO / 8. The PLL locks (PLL_STATUS.PLL_LOCK, bit 0) once PLL_EN is set, the oscillator is on, MCLK is
#    present (PLL_MCLK_STATUS, bit 2: MCLK above 1 MHz) and F_REF is within 2-5 MHz (DS 13.26.1, Table 36 note); it drops
#    when any of those goes away, and a write to PLL_INTEGER / PLL_FRAC_TOP / PLL_FRAC_BOT restarts lock acquisition
#    (DS 13.27 "reset the PLL by re-writing PLL_INTEGER"). The `sysclk_hz` property reports the resulting system clock.
#  - PLL_STATUS 0x03 (DS 14.2): bit 3 PLL_BYPASS_ACTIVE (MCLK is the system clock: oscillator on, PLL_EN clear),
#    bit 2 PLL_MCLK_STATUS, bit 1 PLL_SRM_LOCK (SRM enabled, PLL locked, DAI in slave mode and a WCLK present; SRM is not
#    available with the DAI as master, Table 33), bit 0 PLL_LOCK.
#
# Config (JSON): "mclk_hz" (MCLK frequency at the pin; default 4000000, the 4 MHz master clock the Pebble 2 Duo I2S
# peripheral drives), "wclk_present" (default false: nothing drives WCLK into the codec when it is DAI master),
# "pll_lock_us" (default 1000).
#
# MODELED SIMPLIFICATION: the datasheet gives no PLL lock time, so lock takes the config value `pll_lock_us` (1 ms) after
#   the conditions above hold; it is a stand-in, not a datasheet number.
# MODELED SIMPLIFICATION: MCLK presence and frequency and WCLK presence are config values, not observed on the pins.
# MODELED SIMPLIFICATION: the 2-5 MHz MCLK TEST_UNLOCK / TEST_PLL procedure (DS 13.29) is stored, not enforced.
# MODELED SIMPLIFICATION: the first access out of standby never fails (DS 13.40.2 says it may); soft reset completes at
#   once; the system controllers finish at once (SYSTEM_STATUS reads 0 = complete) rather than after the Table 24
#   start-up times; audio paths, mixers, gains, DAI data, ADC and tone generator only store their registers; registers
#   other than those listed reset to 0 rather than their datasheet defaults.
import math

PLL_STATUS, CIF_CTRL, DIG_ROUTING_DAI, PLL_FRAC_TOP, PLL_FRAC_BOT, PLL_INTEGER, PLL_CTRL, DAI_CLK_MODE = (
    0x03, 0x1D, 0x21, 0x24, 0x25, 0x26, 0x27, 0x28)
SYSTEM_ACTIVE = 0xFD
SOFT_RESET = 0x80
PLL_EN, PLL_SRM_EN, DAI_CLK_EN = 0x80, 0x40, 0x80
LOCK, SRM_LOCK, MCLK_OK, BYPASS = 0x01, 0x02, 0x04, 0x08
DEFAULTS = {DIG_ROUTING_DAI: 0x10, PLL_INTEGER: 0x20, PLL_CTRL: 0x0C, DAI_CLK_MODE: 0x01}
READ_ONLY = {PLL_STATUS, 0x02, 0xE0}


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        cfg = getattr(ctx, "config", None) or {}
        self.mclk_hz = float(cfg.get("mclk_hz", 4_000_000))
        self.wclk = bool(cfg.get("wclk_present", False))
        self.lock_us = int(cfg.get("pll_lock_us", 1000))
        self._reset()

    def _reset(self):
        self.regs = bytearray(256)
        for a, v in DEFAULTS.items():
            self.regs[a] = v
        self.ptr, self.first = 0, True
        self.lock_at = None
        self.ctx.cancel_timer()

    def get_property(self, name):
        return {"sysclk_hz": self.sysclk_hz}[name]

    # ---- clock tree -----------------------------------------------------------------------------------
    @property
    def sysclk_hz(self):
        r = self.regs
        if not r[SYSTEM_ACTIVE] & 1:
            return 0
        if not r[PLL_CTRL] & PLL_EN:
            return self.mclk_hz
        fbdiv = r[PLL_INTEGER] & 0x7F
        fbdiv += (((r[PLL_FRAC_TOP] & 0x1F) << 8) | r[PLL_FRAC_BOT]) / 8192
        return self.mclk_hz / 2 ** (((r[PLL_CTRL] >> 2) & 3) + 1) * fbdiv / 8

    def _can_lock(self):
        r = self.regs
        fref = self.mclk_hz / 2 ** (((r[PLL_CTRL] >> 2) & 3) + 1)
        return bool(r[SYSTEM_ACTIVE] & 1 and r[PLL_CTRL] & PLL_EN and self.mclk_hz > 1e6 and 2e6 <= fref <= 5e6)

    def _relock(self, restart=False):
        if not self._can_lock():
            self.lock_at = None
        elif self.lock_at is None or restart:
            self.lock_at = self.ctx.now_us + self.lock_us
            self.ctx.schedule_oneshot(max(1, self.lock_us))

    def _locked(self):
        return self.lock_at is not None and self.ctx.now_us >= self.lock_at

    def on_timer(self):
        pass                                           # PLL_STATUS is computed from time on read

    def _status(self):
        r = self.regs
        on = r[SYSTEM_ACTIVE] & 1
        s = MCLK_OK if on and self.mclk_hz > 1e6 else 0
        if on and not r[PLL_CTRL] & PLL_EN:
            s |= BYPASS
        if self._locked():
            s |= LOCK
            if r[PLL_CTRL] & PLL_SRM_EN and not r[DAI_CLK_MODE] & DAI_CLK_EN and self.wclk:
                s |= SRM_LOCK
        return s

    # ---- host interface -------------------------------------------------------------------------------
    def on_address(self, addr, is_read):
        self.first = not is_read
        return True

    def _touch(self, a):
        if a != SYSTEM_ACTIVE and not self.regs[SYSTEM_ACTIVE] & 1:
            self.regs[SYSTEM_ACTIVE] = 1               # any access exits standby (DS 13.40.2)
            self._relock()

    def on_write(self, data):
        for b in data:
            if self.first:
                self.ptr, self.first = b, False
            else:
                self._touch(self.ptr)
                self._write(self.ptr, b)
                self.ptr = (self.ptr + 1) & 0xFF
        return len(data)

    def on_read(self, count):
        out = bytearray()
        for _ in range(count):
            self._touch(self.ptr)
            out.append(self._read(self.ptr))
            self.ptr = (self.ptr + 1) & 0xFF
        return bytes(out)

    def _read(self, a):
        return self._status() if a == PLL_STATUS else self.regs[a]

    def _write(self, a, v):
        if a in READ_ONLY:
            return
        if a == CIF_CTRL and v & SOFT_RESET:
            self._reset()
            return
        self.regs[a] = v
        if a == SYSTEM_ACTIVE:
            self.regs[a] = v & 1
            self._relock()
        elif a in (PLL_FRAC_TOP, PLL_FRAC_BOT, PLL_INTEGER):
            self._relock(restart=True)
        elif a == PLL_CTRL:
            self._relock()
