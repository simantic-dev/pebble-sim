# ST LSM6DSO 6-axis IMU at 7-bit 0x6A (SA0 low) as a scripted I2C peer.
# Sources: DS12140 Rev 2 (datasheet, "DS"), AN5192 Rev 6 ("AN"); table numbers as printed in them.
# Wiring: ctx.gpio(0) = INT1, ctx.gpio(1) = INT2 (declare numberOfGpios: 2). Active high unless
# CTRL3_C.H_LACTIVE (DS Table 50). Registers: DS Table 21 map. Output words are little-endian.
#
# Modelled
#  - Accelerometer, gyroscope and temperature sampled every 1/ODR from the ODR bits (DS Table 44, 48; the
#    high-performance column, XL_HM_MODE = 0). Temperature refreshes at 52 Hz while either sensor is on
#    (DS Table 2, TODR), 0 LSB at 25 C and 256 LSB/C. FS from CTRL1_XL / CTRL2_G (DS Table 2 sensitivities).
#  - STATUS_REG XLDA/GDA/TDA, cleared by the high-byte read of an output; BDU holds a word between its low and
#    high byte reads (DS Table 50); IF_INC auto-increment, with FIFO_DATA_OUT_Z_H (7Eh) rolling to 78h (AN 4.x).
#  - Data-ready on INT1/INT2: latched (the pin follows the status flag until the output is read) or pulsed
#    75 us (COUNTER_BDR_REG1.dataready_pulsed, DS Table 40).
#  - FIFO (DS 6.5): 512 words of tag + 6 data bytes (3 kB), watermark WTM[8:0], STOP_ON_WTM, bypass / FIFO /
#    continuous / continuous-to-FIFO / bypass-to-continuous / bypass-to-FIFO modes (trigger = wake-up),
#    batching rates BDR_XL / BDR_GY (FIFO_CTRL3) and ODR_T_BATCH (FIFO_CTRL4), tagged words with TAG_CNT and
#    parity (AN Table 79), FIFO_STATUS1/2 (DIFF_FIFO, WTM_IA, OVR_IA, FULL_IA, FIFO_OVR_LATCHED,
#    COUNTER_BDR_IA), COUNTER_BDR threshold, and INT1/INT2 routing of TH / OVR / FULL / CNT_BDR.
#  - Wake-up (AN 5.3): slope filter (acc(n) - acc(n-1)) / 2 against WK_THS * FS / 2^6 (or 2^8 with WAKE_THS_W),
#    WAKE_DUR consecutive samples, WAKE_UP_SRC (WU_IA, X/Y/Z_WU) and ALL_INT_SRC, LIR latch with
#    INT_CLR_ON_READ, INTERRUPTS_ENABLE gate, routing through MD1_CFG / MD2_CFG INT_WU.
#  - SW_RESET self-clears after 50 us, BOOT after 10 ms with INT1_BOOT high meanwhile (AN 3.x reset / reboot
#    flow); no I2C ACK for the first 10 ms after power-on (DS Vdd boot procedure).
#  - Noise: HP mode An (70 ug/rtHz at 2 g, DS Table 2) x sqrt(ODR/2); ULP mode (CTRL5_C.XL_ULP_EN) 5.5 mg RMS
#    (DS Table 2); gyro Rn 3.8 mdps/rtHz. Seeded, so a run is repeatable.
#
# Motion input (config JSON): "seed", "accel_g": [x,y,z] (default [0,0,1], level and still), "gyro_dps",
# "temp_c", and "profile": [{"t_ms": 0, "accel_g": [...], "gyro_dps": [...]}, ...] (piecewise constant).
# The machine can also write the properties accel_g, gyro_dps and temp_c at run time.
#
# Not modelled (registers store, nothing happens): tap, double tap, free-fall, 6D/4D, activity/inactivity,
# pedometer / significant motion / MLC / FSM (embedded function bank), sensor hub, OIS, timestamp counter and
# FIFO timestamp / CFG_change words, FIFO compression, the HPF and absolute-offset wake-up paths (the slope filter
# is used), low-power / normal modes (the high-performance ODR column is always used), LPF1/LPF2, the samples
# discarded after an ODR or mode change (DS/AN Table 12 settling), self-test, bias and cross-axis error,
# open-drain INT pins (PP_OD).
import math
import random
import struct
from collections import deque

INT1, INT2 = 0, 1
(FIFO_CTRL1, FIFO_CTRL2, FIFO_CTRL3, FIFO_CTRL4, COUNTER_BDR1, COUNTER_BDR2, INT1_CTRL, INT2_CTRL, WHO_AM_I,
 CTRL1_XL, CTRL2_G, CTRL3_C, CTRL5_C, CTRL9_XL, ALL_INT_SRC, WAKE_UP_SRC, STATUS_REG, OUT_TEMP_L,
 OUTX_L_G, OUTX_L_A, OUT_END, FIFO_STATUS1, FIFO_STATUS2, TAP_CFG0, TAP_CFG2, WAKE_UP_THS, WAKE_UP_DUR,
 MD1_CFG, MD2_CFG, FIFO_TAG) = (0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x14,
                                0x18, 0x1A, 0x1B, 0x1E, 0x20, 0x22, 0x28, 0x2E, 0x3A, 0x3B, 0x56, 0x58, 0x5B,
                                0x5C, 0x5E, 0x5F, 0x78)
WHO_AM_I_VALUE = 0x6C                                   # DS Table 41
DEFAULTS = {0x02: 0x3F, CTRL3_C: 0x04, CTRL9_XL: 0xE0}  # PIN_CTRL, CTRL3_C (IF_INC), CTRL9_XL; DS Table 21
READ_ONLY = {WHO_AM_I, STATUS_REG, ALL_INT_SRC, WAKE_UP_SRC, FIFO_STATUS1, FIFO_STATUS2, 0x40, 0x41, 0x42, 0x43,
             *range(OUT_TEMP_L, OUT_END), *range(0x78, 0x7F)}
BDU, H_LACTIVE, IF_INC, BOOT, SW_RESET = 0x40, 0x20, 0x04, 0x80, 0x01             # CTRL3_C, Table 50
ULP_EN = 0x80                                                                    # CTRL5_C, Table 54
XLDA, GDA, TDA = 0x01, 0x02, 0x04                                                # STATUS_REG
DRDY_PULSED = 0x80                                                               # COUNTER_BDR_REG1
RST_COUNTER_BDR, TRIG_COUNTER_BDR_G = 0x40, 0x20
WTM_IA, OVR_IA, FULL_IA, CNT_IA, OVR_LATCHED = 0x80, 0x40, 0x20, 0x10, 0x08       # FIFO_STATUS2, Table 113
STOP_ON_WTM = 0x80                                                               # FIFO_CTRL2
INTERRUPTS_ENABLE, LIR, INT_CLR_ON_READ, WAKE_THS_W = 0x80, 0x01, 0x40, 0x10      # TAP_CFG2/0, WAKE_UP_DUR
INTX_BOOT, INTX_FIFO_TH, INTX_FIFO_OVR, INTX_FIFO_FULL, INTX_CNT_BDR = 0x04, 0x08, 0x10, 0x20, 0x40
INT1_DRDY_XL, INT1_DRDY_G, INT2_DRDY_TEMP, INTX_WU = 0x01, 0x02, 0x04, 0x20
FIFO_WORDS = 512                  # 3 kB of data, DS 6.5
PULSE_US = 75                     # pulsed data-ready width, DS Table 40
RESET_US, BOOT_US, POWER_ON_US = 50, 10_000, 10_000   # AN reset / reboot flow, DS boot procedure
TEMP_ODR_HZ = 52
# ODR by code. Accel 1011 is 12.5 Hz in high performance (Table 44); gyro Table 48.
XL_ODR_HZ = {1: 12.5, 2: 26, 3: 52, 4: 104, 5: 208, 6: 416, 7: 833, 8: 1660, 9: 3330, 10: 6660, 11: 12.5}
G_ODR_HZ = {1: 12.5, 2: 26, 3: 52, 4: 104, 5: 208, 6: 416, 7: 833, 8: 1660, 9: 3330, 10: 6660}
# BDR by code (FIFO_CTRL3 / ODR_T_BATCH, DS Table 25-26).
BDR_HZ = {1: 12.5, 2: 26, 3: 52, 4: 104, 5: 208, 6: 417, 7: 833, 8: 1667, 9: 3333, 10: 6667, 11: 6.5}
T_BATCH_HZ = {1: 1.6, 2: 12.5, 3: 52}
ACCEL_MG_PER_LSB = (0.061, 0.488, 0.122, 0.244)   # CTRL1_XL.FS[1:0] = 2 g, 16 g, 4 g, 8 g (DS Table 2, 45)
ACCEL_FS_MG = (2000, 16000, 4000, 8000)
GYRO_MDPS_PER_LSB = (8.75, 17.5, 35.0, 70.0)      # 250, 500, 1000, 2000 dps (DS Table 2, 47)
GYRO_MDPS_125, GYRO_MDPS_4000 = 4.375, 140.0


def _clamp16(v):
    return max(-0x8000, min(0x7FFF, int(math.floor(v + 0.5))))


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        cfg = getattr(ctx, "config", None) or {}
        self.rng = random.Random(cfg.get("seed", 1))
        self.profile = sorted(cfg.get("profile", []), key=lambda p: p.get("t_ms", 0))
        self.base = {"accel_g": cfg.get("accel_g", [0.0, 0.0, 1.0]), "gyro_dps": cfg.get("gyro_dps", [0.0] * 3),
                     "temp_c": cfg.get("temp_c", 25.0)}
        self.level = [None, None]
        self.alive_at = POWER_ON_US
        self._reset()

    # ---- host interface -------------------------------------------------------------------------------
    def set_property(self, name, value):
        self.base[name] = value

    def get_property(self, name):
        return self.base[name]

    def on_address(self, addr, is_read):
        self.first = not is_read
        return self.ctx.now_us >= self.alive_at

    def on_write(self, data):
        for b in data:
            if self.first:
                self.ptr, self.first = b & 0x7F, False
            else:
                self._write(self.ptr, b)
                self._advance()
        return len(data)

    def on_read(self, count):
        out = bytearray()
        for _ in range(count):
            out.append(self._read(self.ptr))
            self._advance()
        return bytes(out)

    def _advance(self):
        if self.ptr == FIFO_TAG + 6:
            self.ptr = FIFO_TAG                       # AN: Z_H rolls back to the tag register
        elif self.regs[CTRL3_C] & IF_INC:
            self.ptr = (self.ptr + 1) & 0x7F

    # ---- state ----------------------------------------------------------------------------------------
    def _reset(self):
        self.regs = bytearray(0x80)
        for a, v in DEFAULTS.items():
            self.regs[a] = v
        self.ptr, self.first = 0, True
        self.out = bytearray(14)
        self.held = [False] * 7
        self.pend = [None] * 7
        self.status = 0
        self.fifo = deque()
        self.ovr_ia = self.ovr_latched = self.cnt_ia = False
        self.head = bytes(7)
        self.cnt = 0
        self.tag_cnt = 0
        self.seq = {"xl": 0, "g": 0, "t": 0}
        self.pushed = dict(self.seq)
        self.cur = {"xl": (0, 0, 0), "g": (0, 0, 0), "t": 0}
        self.prev_xl = None
        self.wu_run = 0
        self.wu_ia, self.wu_axes, self.clear_wu = False, 0, False
        self.booting = False
        self.due = {}          # event name -> next time, us
        self.pulse = [None, None]   # end of the data-ready pulse per line (INT1, INT2)
        self.pulse_hi = [False, False]
        self._sync()

    def _fs_index(self):
        return (self.regs[CTRL1_XL] >> 2) & 3

    def _rate(self, ev):
        r = self.regs
        if ev in ("xl", "bxl"):
            hz = XL_ODR_HZ.get(r[CTRL1_XL] >> 4) if ev == "xl" else BDR_HZ.get(r[FIFO_CTRL3] & 15)
        elif ev in ("g", "bg"):
            hz = G_ODR_HZ.get(r[CTRL2_G] >> 4) if ev == "g" else BDR_HZ.get(r[FIFO_CTRL3] >> 4)
        elif ev == "t":
            hz = TEMP_ODR_HZ if self._rate("xl") or self._rate("g") else None
        else:  # bt
            hz = T_BATCH_HZ.get((r[FIFO_CTRL4] >> 4) & 3)
        if ev[0] == "b" and (r[FIFO_CTRL4] & 7) == 0:
            return None                              # bypass: nothing is batched
        return hz

    def _sync(self):
        """Start or stop each periodic event to match the registers; a running one keeps its phase."""
        now = self.ctx.now_us
        for ev in ("xl", "g", "t", "bxl", "bg", "bt"):
            hz = self._rate(ev)
            if hz is None:
                self.due.pop(ev, None)
            elif ev not in self.due:
                self.due[ev] = now + 1e6 / hz
        self._pins()
        self._arm()

    def _arm(self):
        pend = list(self.due.values()) + [t for t in self.pulse if t is not None]
        if not pend:
            self.ctx.cancel_timer()
        else:
            self.ctx.schedule_oneshot(max(1, math.ceil(min(pend) - self.ctx.now_us)))

    def on_timer(self):
        now = self.ctx.now_us
        while True:
            ready = [(t, k) for k, t in self.due.items() if t <= now]
            if not ready:
                break
            t, k = min(ready, key=lambda e: (e[0], e[1][0] == "b"))   # a sample is taken before it is batched
            if k == "reset":
                del self.due[k]
                self.regs[CTRL3_C] &= ~SW_RESET
            elif k == "boot":
                del self.due[k]
                self.regs[CTRL3_C] &= ~BOOT
                self.booting = False
            else:
                hz = self._rate(k)
                if hz is None:
                    del self.due[k]
                    continue
                self.due[k] = t + 1e6 / hz
                getattr(self, "_ev_" + k)(t)
        for i in (0, 1):
            if self.pulse[i] is not None and self.pulse[i] <= now:
                self.pulse[i] = None
                self.pulse_hi[i] = False
        self._pins()
        self._arm()

    # ---- sensing --------------------------------------------------------------------------------------
    def _motion(self, t_us):
        m = dict(self.base)
        for p in self.profile:
            if p.get("t_ms", 0) * 1000 <= t_us:
                m.update({k: v for k, v in p.items() if k != "t_ms"})
        return m

    def _ev_xl(self, t):
        m = self._motion(t)
        lsb = ACCEL_MG_PER_LSB[self._fs_index()]
        sigma = 5.5 if self.regs[CTRL5_C] & ULP_EN else 0.070 * math.sqrt(self._rate("xl") / 2)
        raw = tuple(_clamp16((g * 1000 + self.rng.gauss(0, sigma)) / lsb) for g in m["accel_g"])
        self.cur["xl"] = raw
        self.seq["xl"] += 1
        self._publish(4, raw, XLDA, "xl")
        self._wake_up(raw, lsb)

    def _ev_g(self, t):
        c2 = self.regs[CTRL2_G]
        lsb = GYRO_MDPS_125 if c2 & 2 else GYRO_MDPS_4000 if c2 & 1 else GYRO_MDPS_PER_LSB[(c2 >> 2) & 3]
        sigma = 3.8 * math.sqrt(self._rate("g") / 2)
        raw = tuple(_clamp16((d * 1000 + self.rng.gauss(0, sigma)) / lsb) for d in self._motion(t)["gyro_dps"])
        self.cur["g"] = raw
        self.seq["g"] += 1
        self._publish(1, raw, GDA, "g")

    def _ev_t(self, t):
        raw = _clamp16((self._motion(t)["temp_c"] - 25.0) * 256)
        self.cur["t"] = raw
        self.seq["t"] += 1
        self._publish(0, (raw,), TDA, "t")

    def _publish(self, first_word, words, flag, sensor):
        for i, w in enumerate(words):
            idx = first_word + i
            if self.regs[CTRL3_C] & BDU and self.held[idx]:
                self.pend[idx] = w
            else:
                self._put(idx, w)
        self.status |= flag
        line_bit = {"xl": INT1_DRDY_XL, "g": INT1_DRDY_G}.get(sensor)
        if line_bit and self.regs[COUNTER_BDR1] & DRDY_PULSED:
            for i, ctl in ((0, INT1_CTRL), (1, INT2_CTRL)):
                if self.regs[ctl] & line_bit:
                    self.pulse_hi[i], self.pulse[i] = True, self.ctx.now_us + PULSE_US
        self._pins()

    def _put(self, idx, w):
        self.out[idx * 2:idx * 2 + 2] = struct.pack("<h", w)

    # ---- FIFO -----------------------------------------------------------------------------------------
    def _fifo_mode(self):
        m = self.regs[FIFO_CTRL4] & 7
        trig = self.wu_ia
        if m == 3:
            return 6 if not trig else 1       # continuous-to-FIFO
        if m == 4:
            return 6 if trig else 0           # bypass-to-continuous
        if m == 7:
            return 1 if trig else 0           # bypass-to-FIFO
        return m

    def _cap(self):
        wtm = ((self.regs[FIFO_CTRL2] & 1) << 8) | self.regs[FIFO_CTRL1]
        return wtm if self.regs[FIFO_CTRL2] & STOP_ON_WTM and wtm else FIFO_WORDS

    def _tag(self, sensor_tag):
        b = (sensor_tag << 3) | (self.tag_cnt << 1)
        return b | (bin(b).count("1") & 1)   # even number of ones (AN, TAG_PARITY)

    def _push(self, sensor_tag, data):
        mode = self._fifo_mode()
        if mode == 0:
            self.fifo.clear()
            self.ovr_ia = False
            return
        if len(self.fifo) >= self._cap():
            self.ovr_ia = True
            if mode == 1:
                return                          # FIFO mode stops when full
            self.fifo.popleft()                 # continuous: the oldest word is overwritten
            self.ovr_latched = True
        self.fifo.append(bytes([self._tag(sensor_tag)]) + data)

    def _batch(self, ev):
        s = {"bxl": "xl", "bg": "g", "bt": "t"}[ev]
        if self.pushed[s] == self.seq[s]:
            return
        self.pushed[s] = self.seq[s]
        c = self.cur[s]
        if s == "t":
            self._push(0x03, struct.pack("<h", c) + bytes(4))
        else:
            self._push(0x02 if s == "xl" else 0x01, struct.pack("<3h", *c))
        trig_g = bool(self.regs[COUNTER_BDR1] & TRIG_COUNTER_BDR_G)
        if s != "t" and (s == "g") == trig_g:
            self.cnt += 1
            th = (self.regs[COUNTER_BDR1] & 7) << 8 | self.regs[COUNTER_BDR2]
            if th and self.cnt >= th:
                self.cnt_ia = True
                self.cnt = 0

    def _ev_bxl(self, t):
        # TAG_CNT is the time slot of the fastest batched main sensor (AN, FIFO_DATA_OUT_TAG).
        if not self._rate("bg") or self._rate("bxl") >= self._rate("bg"):
            self.tag_cnt = (self.tag_cnt + 1) & 3
        self._batch("bxl")

    def _ev_bg(self, t):
        if not self._rate("bxl") or self._rate("bxl") < self._rate("bg"):
            self.tag_cnt = (self.tag_cnt + 1) & 3
        self._batch("bg")

    def _ev_bt(self, t):
        self._batch("bt")

    def _fifo_flags(self):
        n = len(self.fifo)
        wtm = ((self.regs[FIFO_CTRL2] & 1) << 8) | self.regs[FIFO_CTRL1]
        return {WTM_IA: wtm > 0 and n >= wtm, OVR_IA: self.ovr_ia, FULL_IA: n >= self._cap() - 1,
                CNT_IA: self.cnt_ia}

    # ---- wake-up (AN 5.3) -----------------------------------------------------------------------------
    def _wake_up(self, raw, lsb):
        r = self.regs
        prev, self.prev_xl = self.prev_xl, raw
        if self.clear_wu:
            self.wu_ia, self.wu_axes, self.clear_wu = False, 0, False
        if prev is None or not r[TAP_CFG2] & INTERRUPTS_ENABLE:
            return
        shift = 8 if r[WAKE_UP_DUR] & WAKE_THS_W else 6
        ths_mg = (r[WAKE_UP_THS] & 0x3F) * ACCEL_FS_MG[self._fs_index()] / (1 << shift)
        axes = 0
        for i in range(3):
            if abs((raw[i] - prev[i]) / 2 * lsb) > ths_mg:
                axes |= 4 >> i                    # X_WU = bit 2, Y_WU = bit 1, Z_WU = bit 0
        self.wu_run = self.wu_run + 1 if axes else 0
        if axes and self.wu_run > (r[WAKE_UP_DUR] >> 5) & 3:
            self.wu_ia, self.wu_axes = True, axes
        elif not r[TAP_CFG0] & LIR:
            self.wu_ia, self.wu_axes = False, 0

    def _wake_src(self):
        return (0x08 if self.wu_ia else 0) | self.wu_axes

    def _clear_on_read(self):
        if self.regs[TAP_CFG0] & LIR:
            if self.regs[TAP_CFG0] & INT_CLR_ON_READ:
                self.wu_ia, self.wu_axes = False, 0
            else:
                self.clear_wu = True              # cleared at the end of the next ODR period

    # ---- pins -----------------------------------------------------------------------------------------
    def _pins(self):
        r, f = self.regs, self._fifo_flags()
        for i, (ctl, md) in enumerate(((INT1_CTRL, MD1_CFG), (INT2_CTRL, MD2_CFG))):
            c = r[ctl]
            if r[COUNTER_BDR1] & DRDY_PULSED:
                drdy = self.pulse_hi[i]
            else:
                drdy = bool(c & INT1_DRDY_XL and self.status & XLDA or c & INT1_DRDY_G and self.status & GDA or
                            i == 1 and c & INT2_DRDY_TEMP and self.status & TDA)
            on = (drdy or (i == 0 and c & INTX_BOOT and self.booting)
                  or c & INTX_FIFO_TH and f[WTM_IA] or c & INTX_FIFO_OVR and f[OVR_IA]
                  or c & INTX_FIFO_FULL and f[FULL_IA] or c & INTX_CNT_BDR and f[CNT_IA]
                  or (r[md] & INTX_WU and self.wu_ia and r[TAP_CFG2] & INTERRUPTS_ENABLE))
            level = bool(on) != bool(r[CTRL3_C] & H_LACTIVE)
            if level != self.level[i]:
                self.level[i] = level
                self.ctx.gpio(i).set(level)

    # ---- registers ------------------------------------------------------------------------------------
    def _read(self, a):
        r = self.regs
        if a == WHO_AM_I:
            return WHO_AM_I_VALUE
        if a == STATUS_REG:
            return self.status & 7
        if OUT_TEMP_L <= a < OUT_END:
            i = (a - OUT_TEMP_L) // 2
            v = self.out[a - OUT_TEMP_L]
            if r[CTRL3_C] & BDU:
                if a & 1:
                    self.held[i] = False
                    if self.pend[i] is not None:
                        self._put(i, self.pend[i])
                        self.pend[i] = None
                else:
                    self.held[i] = True
            if a & 1:
                self.status &= ~(TDA if i == 0 else GDA if i < 4 else XLDA)
                self._pins()
            return v
        if a in (WAKE_UP_SRC, ALL_INT_SRC):
            v = self._wake_src() if a == WAKE_UP_SRC else (0x02 if self.wu_ia else 0)
            self._clear_on_read()
            self._pins()
            return v
        if a == FIFO_STATUS1:
            return len(self.fifo) & 0xFF
        if a == FIFO_STATUS2:
            f = self._fifo_flags()
            v = (len(self.fifo) >> 8) & 3 | (self.ovr_latched and OVR_LATCHED)
            for bit, on in f.items():
                v |= bit if on else 0
            self.ovr_latched = False
            return v
        if FIFO_TAG <= a <= FIFO_TAG + 6:
            if a == FIFO_TAG:
                self.head = self.fifo[0] if self.fifo else bytes(7)
            v = self.head[a - FIFO_TAG]
            if a == FIFO_TAG + 6 and self.fifo:
                self.fifo.popleft()
                self.ovr_ia = False
                self._pins()
            return v
        return r[a]

    def _write(self, a, v):
        if a in READ_ONLY:
            return
        r = self.regs
        old = r[a]
        r[a] = v
        if a == CTRL3_C:
            if v & SW_RESET:
                self._reset()
                self.regs[CTRL3_C] |= SW_RESET
                self.due["reset"] = self.ctx.now_us + RESET_US
                self._sync()
                return
            if v & BOOT and not old & BOOT:
                self.booting = True
                self.due["boot"] = self.ctx.now_us + BOOT_US
            elif old & BOOT:
                r[a] |= BOOT
        elif a == COUNTER_BDR1 and v & RST_COUNTER_BDR:
            r[a] &= ~RST_COUNTER_BDR
            self.cnt, self.cnt_ia = 0, False
        elif a == FIFO_CTRL4 and (v & 7) == 0:
            self.fifo.clear()
            self.ovr_ia = False
        if a == CTRL1_XL and v >> 4 != old >> 4 and not v >> 4:
            self.prev_xl, self.wu_run = None, 0
        self._sync()
