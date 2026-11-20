# TI DRV2604 haptic driver at 7-bit 0x5A as a scripted I2C peer.
# Source: DRV2604 datasheet SLOS824C (March 2018), "DS"; section and table numbers as printed.
# Wiring: ctx line 0 (declare numberOfGpios: 1) is the EN input, driven by the MCU (P0.2 on the Pebble 2 Duo).
# Registers: DS 7.6 Table 2 map, auto-increment on reads and writes.
#
# Modelled
#  - EN gates register access (DS 7.4.1.3): with EN low the device still ACKs but no read or write happens (writes are
#    dropped, reads return 0xFF, the idle bus). Registers are kept while EN is low. EN is low until the MCU drives it.
#    EN going low ends any process.
#  - Reset values (Table 2): STATUS 0x80 (DEVICE_ID = 4, DRV2604, Table 3), MODE 0x40 (STANDBY = 1, Table 4),
#    sequencer 0x04 = 0x01, RATED_VOLTAGE 0x3F, OD_CLAMP 0x89, A_CAL_COMP 0x0D, A_CAL_BEMF 0x6D, FBCTL 0x36,
#    CONTROL1 0x93, CONTROL2 0xF5, CONTROL3 0x80, CONTROL4 0x20 (AUTO_CAL_TIME = 2). MODE.DEV_RESET restores all of
#    these and self-clears (DS 7.4.1.5).
#  - STANDBY (DS 7.4.1.4): setting it acts at once and stops a running process (GO clears).
#  - GO (Table 8, DS 7.4.3) in internal-trigger mode (MODE 0) plays the waveform sequencer (Table 7): a WAIT entry
#    idles 10 ms * WAV_FRM_SEQ[6:0], the sequence ends at an identifier of zero or after eight entries; GO stays 1
#    until then and clears at the end; writing GO = 0 cancels. GO in calibration mode (MODE 7) runs for AUTO_CAL_TIME
#    (Table 21: 150-350, 250-450, 500-700, 1000-1200 ms; the middle of the printed range is used), then GO clears,
#    A_CAL_COMP / A_CAL_BEMF hold the result and STATUS.DIAG_RESULT = pass / fail from the config.
#  - RTP mode (MODE 5) drives while RTP_INPUT is non-zero and the part is out of standby; it does not touch GO (DS 7.4.3).
#    While driving, VBAT (0x21, VDD = VBAT * 5.6 V / 255) and LRA_PERIOD (0x22, LRA_PERIOD * 98.46 us) read live.
#  - STATUS.DIAG_RESULT clears on read (Table 3).
#
# Config (JSON): "vdd_mv" (default 3700), "lra_hz" (default 235), "cal_comp" / "cal_bemf" (auto-cal results, default
# the reset values), "cal_fail" (default false).
#
# MODELED SIMPLIFICATION: the RAM library (RAM_ADDR/RAM_DATA, waveform lengths) is not modelled, so a non-zero waveform
#   identifier plays for no time; only WAIT entries take time. ILLEGAL_ADDR is never set.
# MODELED SIMPLIFICATION: the actuator is not modelled: auto-cal results and the diagnostics outcome (MODE 6, which
#   completes at once because the datasheet gives no duration) come from the config; no back-EMF, braking, overdrive,
#   PWM / analog / external-trigger inputs, OTP programming, over-temperature or over-current.
# MODELED SIMPLIFICATION: DEV_RESET completes at once (the datasheet gives no reset time).
import math

STATUS, MODE, RTP, GO, A_CAL_COMP, A_CAL_BEMF, CTRL4, VBAT, LRA_PERIOD = 0x00, 0x01, 0x02, 0x0C, 0x18, 0x19, 0x1E, 0x21, 0x22
DEV_RESET, STANDBY = 0x80, 0x40
DIAG_RESULT = 0x08
DEFAULTS = {STATUS: 0x80, MODE: 0x40, 0x04: 0x01, 0x16: 0x3F, 0x17: 0x89, A_CAL_COMP: 0x0D, A_CAL_BEMF: 0x6D, 0x1A: 0x36,
            0x1B: 0x93, 0x1C: 0xF5, 0x1D: 0x80, CTRL4: 0x20}
READ_ONLY = {STATUS}
CAL_MS = ((150, 350), (250, 450), (500, 700), (1000, 1200))     # Table 21
EN = 0


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        cfg = getattr(ctx, "config", None) or {}
        self.vdd_mv = int(cfg.get("vdd_mv", 3700))
        self.lra_hz = float(cfg.get("lra_hz", 235))
        self.cal = (int(cfg.get("cal_comp", DEFAULTS[A_CAL_COMP])), int(cfg.get("cal_bemf", DEFAULTS[A_CAL_BEMF])))
        self.cal_fail = bool(cfg.get("cal_fail", False))
        self.en = False
        self.regs = bytearray(256)
        self._reset()

    def _reset(self):
        self.regs[:] = bytes(256)
        for a, v in DEFAULTS.items():
            self.regs[a] = v
        self.ptr, self.first = 0, True
        self.go_end = None
        self.ctx.cancel_timer()

    # ---- host interface -------------------------------------------------------------------------------
    def on_gpio(self, line, level):
        if line == EN:
            self.en = bool(level)
            if not self.en:
                self._stop()

    def on_address(self, addr, is_read):
        self.first = not is_read
        return True                                    # ACKs with EN low too (DS 7.4.1.3)

    def on_write(self, data):
        for b in data:
            if self.first:
                self.ptr, self.first = b, False
            else:
                if self.en:
                    self._write(self.ptr, b)
                self.ptr = (self.ptr + 1) & 0xFF
        return len(data)

    def on_read(self, count):
        out = bytearray()
        for _ in range(count):
            out.append(self._read(self.ptr) if self.en else 0xFF)
            self.ptr = (self.ptr + 1) & 0xFF
        return bytes(out)

    # ---- registers ------------------------------------------------------------------------------------
    def _driving(self):
        return (self.en and self.regs[MODE] & 7 == 5 and not self.regs[MODE] & STANDBY and self.regs[RTP] != 0)

    def _read(self, a):
        v = self.regs[a]
        if a == STATUS:
            self.regs[STATUS] &= ~DIAG_RESULT
        elif a == VBAT and self._driving():
            v = min(255, round(self.vdd_mv / 1000 * 255 / 5.6))
        elif a == LRA_PERIOD and self._driving():
            v = min(255, round(1e6 / self.lra_hz / 98.46))
        return v

    def _write(self, a, v):
        if a in READ_ONLY:
            return
        if a == MODE and v & DEV_RESET:
            self._reset()
            return
        if a == GO:
            self._go(v & 1)
            return
        self.regs[a] = v
        if a == MODE and v & STANDBY:
            self._stop()

    # ---- processes ------------------------------------------------------------------------------------
    def _go(self, bit):
        if not bit:
            self._stop()
            return
        mode = self.regs[MODE] & 7
        if self.go_end is not None or self.regs[MODE] & STANDBY or mode not in (0, 6, 7):
            return
        self.regs[GO] = 1
        self.go_kind = mode
        self.go_end = self.ctx.now_us + self._duration_us(mode)
        self.ctx.schedule_oneshot(max(1, math.ceil(self.go_end - self.ctx.now_us)))
        if self.go_end <= self.ctx.now_us:
            self.on_timer()

    def _duration_us(self, mode):
        if mode == 7:
            lo, hi = CAL_MS[(self.regs[CTRL4] >> 4) & 3]
            return (lo + hi) / 2 * 1000
        if mode == 6:
            return 0
        total = 0
        for a in range(0x04, 0x0C):
            e = self.regs[a]
            if e & 0x7F == 0:
                break
            if e & 0x80:
                total += (e & 0x7F) * 10_000
        return total

    def _stop(self):
        self.regs[GO] = 0
        self.go_end = None
        self.ctx.cancel_timer()

    def on_timer(self):
        if self.go_end is None or self.ctx.now_us < self.go_end:
            return
        if self.go_kind == 7:
            self.regs[A_CAL_COMP], self.regs[A_CAL_BEMF] = self.cal
        if self.go_kind in (6, 7):
            self.regs[STATUS] = (self.regs[STATUS] & ~DIAG_RESULT) | (DIAG_RESULT if self.cal_fail else 0)
        self._stop()
