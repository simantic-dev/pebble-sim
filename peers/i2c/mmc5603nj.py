# MEMSIC MMC5603NJ 3-axis magnetometer at 7-bit 0x30 as a scripted I2C peer.
# Source: MMC5603NJ datasheet Rev. B (formal release 1/17/2022), "DS". The datasheet has unnumbered sections;
# cited by name: "Specifications", "Register Map", "Register Details", "Operating Timing Diagram".
# Registers (Register Map / Register Details): Xout0..Zout1 00h-05h, Xout2..Zout2 06h-08h (bits 3:0 in the high
# nibble), Tout 09h, Status1 18h, ODR 1Ah, Control0 1Bh, Control1 1Ch, Control2 1Dh, ST_*_TH 1Eh-20h, ST_* 27h-29h,
# Product ID 39h (reset 0x10). Output words are unsigned, 20 bit, null field 524288 counts, 16384 counts/G
# (Specifications). A register read auto-increments the pointer.
#
# Modelled
#  - Power-on: no ACK for tOp = 5 ms (Operating Timing Diagram); OTP_read_done (Status1 bit 4) is set when the OTP
#    read finishes. Control1.Sw_reset clears every register and the part is unavailable for 20 ms (Control1
#    Sw_reset), then OTP_read_done is set again.
#  - Control0.Take_meas_M / Take_meas_T start one measurement; the bit self-clears at its end and Status1
#    Meas_m_done / Meas_t_done is set. Meas_m_done is cleared by a read of any magnetic data register 00h-08h,
#    Meas_t_done by a read of Tout (Status1 description).
#  - Measurement time tTM from Control1.BW (6.6 / 3.5 / 2.0 / 1.2 ms, Control1 table and Operating Timing Diagram)
#    for the three axes together, scaled by the number of axes not inhibited (X/Y/Z-inhibit; an inhibited axis
#    keeps its last output). With Auto_SR_en the part measures after a SET and after a RESET, so a measurement
#    takes 2 * tTM (this is what limits the ODR to 75 / 150 / 255 Hz per BW in the ODR table: 2 * 6.6 ms = 13.2 ms
#    against 1/75 Hz = 13.3 ms).
#  - Continuous mode: ODR (1Ah, write only) non-zero, Control0.Cmm_freq_en (self-clears) then Control2.Cmm_en;
#    ODR 255 with Control2.hpower is 1000 Hz. A measurement starts every 1/ODR; if the ODR is above what the
#    BW / Auto_SR setting can sustain (ODR table) measurements run back to back at the sustainable rate.
#    Clearing Cmm_en leaves continuous mode; a running measurement completes.
#  - Control0.Do_Set / Do_Reset self-clear (375 ns pulse, below this peer's 1 us timer resolution, so at once).
#  - Output = (field_gauss * 16384 + 524288) per axis plus seeded Gaussian noise at the typical total RMS noise per BW
#    (1.5 / 2.0 / 3.0 / 4.0 mG, Specifications). Tout = (temp_c + 75) / 0.8 (Tout register description).
#
# Motion input (config JSON): "seed", "field_gauss": [x,y,z] (default [0,0,0]), "temp_c" (default 25).
# The machine can also write the properties field_gauss and temp_c at run time.
#
# MODELED SIMPLIFICATION: no SET/RESET polarity or offset physics (the output is the null-field-compensated
#   field), so tSR (1 ms between SET/RESET and other operations) is not enforced; Prd_set periodic SET, auto
#   self-test, ST_*/ST_*_TH and the I3C features only store (self-test is not run, Sat_sensor stays 0).
# MODELED SIMPLIFICATION: the datasheet gives no temperature measurement time; Take_meas_T takes tTM of the BW.
# MODELED SIMPLIFICATION: the write-only registers (ODR, Control0-2, ST_*_TH) read back what was written (with
#   self-clearing bits cleared); the datasheet does not define their read value.
import math
import random

(XOUT0, TOUT, STATUS1, ODR, CTRL0, CTRL1, CTRL2, PRODUCT_ID) = 0x00, 0x09, 0x18, 0x1A, 0x1B, 0x1C, 0x1D, 0x39
PRODUCT_ID_VALUE = 0x10
MEAS_T_DONE, MEAS_M_DONE, OTP_READ_DONE = 0x80, 0x40, 0x10
TAKE_M, TAKE_T, DO_SET, DO_RESET, AUTO_SR_EN, CMM_FREQ_EN = 0x01, 0x02, 0x08, 0x10, 0x20, 0x80
SW_RESET, INHIBIT = 0x80, 0x1C                                  # Control1
HPOWER, CMM_EN = 0x80, 0x10                                     # Control2
POWER_ON_US, SW_RESET_US = 5_000, 20_000                        # tOp; Control1 Sw_reset "power on time is 20 mS"
TTM_US = (6600, 3500, 2000, 1200)                               # per BW, Control1 table
NOISE_MG = (1.5, 2.0, 3.0, 4.0)                                 # typical total RMS noise per BW, Specifications
COUNTS_PER_G, NULL = 16384, 524288
SELF_CLEAR = {CTRL0: TAKE_M | TAKE_T | DO_SET | DO_RESET | CMM_FREQ_EN}


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        cfg = getattr(ctx, "config", None) or {}
        self.rng = random.Random(cfg.get("seed", 1))
        self.base = {"field_gauss": cfg.get("field_gauss", [0.0, 0.0, 0.0]), "temp_c": cfg.get("temp_c", 25.0)}
        self.alive_at = POWER_ON_US
        self._reset(POWER_ON_US)

    def set_property(self, name, value):
        self.base[name] = value

    def get_property(self, name):
        return self.base[name]

    def _reset(self, otp_done_at):
        self.regs = bytearray(0x40)
        self.regs[PRODUCT_ID] = PRODUCT_ID_VALUE
        self.out = bytearray(10)
        self.ptr, self.first = 0, True
        self.meas_end = {}              # "m" / "t" -> time the running measurement completes
        self.next_cmm = None            # start time of the next continuous measurement
        self.cmm_period = None
        self.otp_done_at = otp_done_at
        self._arm()

    # ---- host interface -------------------------------------------------------------------------------
    def on_address(self, addr, is_read):
        self.first = not is_read
        return self.ctx.now_us >= self.alive_at

    def on_write(self, data):
        self._catch_up()
        for b in data:
            if self.first:
                self.ptr, self.first = b, False
            else:
                self._write(self.ptr, b)
                self.ptr = (self.ptr + 1) & 0xFF
        return len(data)

    def on_read(self, count):
        self._catch_up()
        out = bytearray()
        for _ in range(count):
            out.append(self._read(self.ptr))
            self.ptr = (self.ptr + 1) & 0xFF
        return bytes(out)

    # ---- registers ------------------------------------------------------------------------------------
    def _read(self, a):
        if a >= len(self.regs):
            return 0
        if a == STATUS1:
            return self.regs[a] | (OTP_READ_DONE if self.ctx.now_us >= self.otp_done_at else 0)
        if a <= 0x08:
            self.regs[STATUS1] &= ~MEAS_M_DONE
        elif a == TOUT:
            self.regs[STATUS1] &= ~MEAS_T_DONE
        return self.out[a] if a <= TOUT else self.regs[a]

    def _write(self, a, v):
        if a >= len(self.regs) or a <= TOUT or a in (STATUS1, PRODUCT_ID):
            return                                                  # read-only registers
        if a == CTRL1 and v & SW_RESET:
            self.alive_at = self.ctx.now_us + SW_RESET_US
            self._reset(self.alive_at)
            return
        self.regs[a] = v & ~SELF_CLEAR.get(a, 0)
        if a == CTRL0:
            if v & TAKE_M and "m" not in self.meas_end:
                self.regs[CTRL0] |= TAKE_M
                self.meas_end["m"] = self.ctx.now_us + self._duration()
            if v & TAKE_T and "t" not in self.meas_end:
                self.regs[CTRL0] |= TAKE_T
                self.meas_end["t"] = self.ctx.now_us + self._tm()
            if v & CMM_FREQ_EN:
                self.cmm_period = self._period()
        if a in (CTRL0, CTRL2, ODR):
            self._sync_cmm()
        self._arm()

    # ---- measurement ----------------------------------------------------------------------------------
    def _bw(self):
        return self.regs[CTRL1] & 3

    def _tm(self):
        return TTM_US[self._bw()]

    def _duration(self):
        axes = 3 - bin(self.regs[CTRL1] & INHIBIT).count("1")
        return self._tm() * axes / 3 * (2 if self.regs[CTRL0] & AUTO_SR_EN else 1)

    def _period(self):
        hz = 1000 if self.regs[ODR] == 255 and self.regs[CTRL2] & HPOWER else self.regs[ODR]
        return max(1e6 / hz, self._duration()) if hz else None

    def _sync_cmm(self):
        on = self.regs[CTRL2] & CMM_EN and self.regs[ODR] and self.cmm_period
        if not on:
            self.next_cmm = None
        elif self.next_cmm is None:
            self.next_cmm = self.ctx.now_us            # first measurement starts when continuous mode starts

    def _sample(self):
        sigma = NOISE_MG[self._bw()] / 1000 * COUNTS_PER_G
        for axis, g in enumerate(self.base["field_gauss"]):
            if self.regs[CTRL1] & (0x04 << axis):
                continue                               # inhibited axis keeps its last value
            raw = max(0, min(0xFFFFF, int(round(NULL + g * COUNTS_PER_G + self.rng.gauss(0, sigma)))))
            self.out[2 * axis], self.out[2 * axis + 1] = raw >> 12, (raw >> 4) & 0xFF
            self.out[6 + axis] = (raw & 0xF) << 4

    def _sample_t(self):
        self.out[TOUT] = max(0, min(255, int(round((self.base["temp_c"] + 75) / 0.8))))

    def _catch_up(self):
        """Run every event due at or before now, in time order."""
        now = self.ctx.now_us
        while True:
            ev = [(t, k) for k, t in self.meas_end.items() if t <= now]
            if self.next_cmm is not None and self.next_cmm <= now and "m" not in self.meas_end:
                ev.append((self.next_cmm, "start"))
            if not ev:
                return
            t, k = min(ev)
            if k == "start":
                self.meas_end["m"] = t + self._duration()
                self.next_cmm = t + self.cmm_period
            elif k == "m":
                del self.meas_end["m"]
                self._sample()
                self.regs[CTRL0] &= ~TAKE_M
                self.regs[STATUS1] |= MEAS_M_DONE
            else:
                del self.meas_end["t"]
                self._sample_t()
                self.regs[CTRL0] &= ~TAKE_T
                self.regs[STATUS1] |= MEAS_T_DONE

    def _arm(self):
        pend = list(self.meas_end.values()) + ([self.next_cmm] if self.next_cmm is not None else [])
        pend = [t for t in pend if t > self.ctx.now_us] or ([self.ctx.now_us + 1] if pend else [])
        if pend:
            self.ctx.schedule_oneshot(max(1, math.ceil(min(pend) - self.ctx.now_us)))
        else:
            self.ctx.cancel_timer()

    def on_timer(self):
        self._catch_up()
        self._arm()
