"""LSM6DSO peer self-tests (peers/i2c/lsm6dso.py): ODR timing, data-ready, BDU, FIFO, wake-up, reset.

    python3 -m pytest peers/tests/test_lsm6dso.py
"""
import struct

from i2c_harness import Dev

FIFO_CTRL1, FIFO_CTRL2, FIFO_CTRL3, FIFO_CTRL4, CNT1, INT1_CTRL, INT2_CTRL = 7, 8, 9, 10, 0x0B, 0x0D, 0x0E
CTRL1_XL, CTRL3_C, STATUS, OUTX_A, FST1, FST2, TAG = 0x10, 0x12, 0x1E, 0x28, 0x3A, 0x3B, 0x78
CFG = {"seed": 7}


def make(config=CFG):
    d = Dev("lsm6dso", 0x6A, config).start(boot_ms=11)
    d.wreg(CTRL3_C, 0x44)             # IF_INC | BDU, as PebbleOS writes
    return d


def accel(d):
    return struct.unpack("<3h", d.read(OUTX_A, 6))


def test_who_am_i_and_no_ack_during_boot():
    d = Dev("lsm6dso", 0x6A).start()
    assert d.write([0x0F]) is False               # first 10 ms after power-on: no ACK
    d.advance(11_000)
    assert d.rreg(0x0F) == 0x6C


def test_drdy_after_one_period_and_value():
    d = make()
    d.wreg(CTRL1_XL, 0x30)                        # 52 Hz, 2 g
    d.advance(18_000)
    assert d.rreg(STATUS) & 1 == 0                # period is 19.23 ms
    d.advance(1_500)
    assert d.rreg(STATUS) & 1 == 1
    x, y, z = accel(d)
    assert abs(z - 16393) < 120 and abs(x) < 120  # +1 g / 0.061 mg per LSB, ULP-free HP noise
    assert d.rreg(STATUS) & 1 == 0                # high-byte read clears XLDA


def test_odr_timing_208hz_sample_count_via_fifo():
    d = make()
    d.wreg(FIFO_CTRL3, 0x05)
    d.wreg(CTRL1_XL, 0x50)
    d.wreg(FIFO_CTRL4, 0x06)                      # continuous
    d.advance(100_000)
    n = d.rreg(FST1) | (d.rreg(FST2) & 3) << 8
    assert 20 <= n <= 21                          # 0.1 s at 208 Hz


def test_int1_latched_and_pulsed_drdy():
    d = make()
    d.wreg(INT1_CTRL, 0x01)
    d.wreg(CTRL1_XL, 0x20)                        # 26 Hz
    d.advance(40_000)
    assert d.ctx.pin[0] is True
    accel(d)
    assert d.ctx.pin[0] is False                  # latched until the output is read
    d.wreg(CNT1, 0x80)                            # pulsed
    d.ctx.levels.clear()
    d.advance(80_000)
    rises = [t for t, line, lvl in d.ctx.levels if line == 0 and lvl]
    falls = [t for t, line, lvl in d.ctx.levels if line == 0 and not lvl]
    assert len(rises) >= 2 and all(0 < f - r <= 80 for r, f in zip(rises, falls))


def test_h_lactive_inverts_int():
    d = make()
    d.wreg(CTRL3_C, 0x64)
    assert d.ctx.pin[0] is True                   # idle level is high
    d.wreg(INT1_CTRL, 0x01)
    d.wreg(CTRL1_XL, 0x20)
    d.advance(40_000)
    assert d.ctx.pin[0] is False


def test_bdu_holds_word_until_high_byte():
    d = make({"seed": 1, "profile": [{"t_ms": 0, "accel_g": [0, 0, 1]}, {"t_ms": 100, "accel_g": [0, 0, -1]}]})
    d.wreg(CTRL1_XL, 0x30)
    d.advance(60_000)
    lo = d.rreg(0x2C)                             # OUTZ_L starts the hold
    d.advance(60_000)                             # new samples (-1 g) arrive meanwhile
    hi = d.rreg(0x2D)
    assert struct.unpack("<h", bytes([lo, hi]))[0] > 10000
    d.advance(25_000)
    assert struct.unpack("<h", d.read(0x2C, 2))[0] < -10000


def test_fifo_watermark_tags_and_pop():
    d = make()
    d.wreg(FIFO_CTRL1, 4)
    d.wreg(INT1_CTRL, 0x08)                       # FIFO_TH
    d.wreg(FIFO_CTRL3, 0x03)                      # BDR 52 Hz
    d.wreg(FIFO_CTRL4, 0x06)
    d.wreg(CTRL1_XL, 0x30)
    d.advance(60_000)
    assert d.ctx.pin[0] is False
    assert d.rreg(FST2) & 0x80 == 0
    d.advance(25_000)                             # 4th word at 76.9 ms
    assert d.ctx.pin[0] is True
    assert d.rreg(FST2) & 0x80 and d.rreg(FST1) == 4
    words = d.read(TAG, 4 * 7)
    for i in range(4):
        w = words[i * 7:(i + 1) * 7]
        assert w[0] >> 3 == 0x02 and bin(w[0]).count("1") % 2 == 0   # accel NC tag, even parity
        assert abs(struct.unpack("<3h", w[1:])[2] - 16393) < 120
    assert (words[0] >> 1 & 3) != (words[7] >> 1 & 3)                  # TAG_CNT advances per slot
    assert d.rreg(FST1) == 0 and d.ctx.pin[0] is False


def test_fifo_overrun_continuous_and_fifo_mode_stops():
    d = make()
    d.wreg(FIFO_CTRL3, 0x06)                      # 417 Hz
    d.wreg(FIFO_CTRL4, 0x06)
    d.wreg(CTRL1_XL, 0x60)
    d.advance(1_300_000)                          # 512 words fill in 1.23 s
    st = d.read(FST1, 2)
    assert st[0] | (st[1] & 3) << 8 == 512 and st[1] & 0x40 and st[1] & 0x20
    assert d.rreg(FST2) & 0x08 == 0               # FIFO_OVR_LATCHED cleared by the previous read
    d.wreg(FIFO_CTRL4, 0x00)                      # bypass flushes
    assert d.read(FST1, 2) == bytes(2)
    d.wreg(FIFO_CTRL4, 0x01)                      # FIFO mode: stops when full, does not overwrite
    d.advance(1_300_000)
    first = d.read(TAG, 7)                        # popping one word makes room, the next sample fills it
    d.advance(100_000)
    st = d.read(FST1, 2)
    assert st[0] | (st[1] & 3) << 8 == 512 and first[0] >> 3 == 0x02


def test_stop_on_wtm_and_bypass_clears():
    d = make()
    d.wreg(FIFO_CTRL1, 3)
    d.wreg(FIFO_CTRL2, 0x80)
    d.wreg(FIFO_CTRL3, 0x04)
    d.wreg(FIFO_CTRL4, 0x01)
    d.wreg(CTRL1_XL, 0x40)
    d.advance(100_000)
    assert d.rreg(FST1) == 3 and d.rreg(FST2) & 0x40


def test_counter_bdr_flag():
    d = make()
    d.wreg(0x0C, 5)
    d.wreg(FIFO_CTRL3, 0x04)
    d.wreg(FIFO_CTRL4, 0x06)
    d.wreg(INT1_CTRL, 0x40)
    d.wreg(CTRL1_XL, 0x40)
    d.advance(60_000)
    assert d.ctx.pin[0] is True
    assert d.rreg(FST2) & 0x10


def test_wake_up_threshold_latch_and_clear_on_read():
    prof = [{"t_ms": 0, "accel_g": [0, 0, 1]}, {"t_ms": 200, "accel_g": [0.5, 0, 1]}, {"t_ms": 300, "accel_g": [0.5, 0, 1]}]
    d = make({"seed": 1, "profile": prof})
    d.wreg(0x56, 0x41)                            # LIR | INT_CLR_ON_READ
    d.wreg(0x58, 0x80)                            # INTERRUPTS_ENABLE
    d.wreg(0x5B, 0x06)                            # threshold 6 * 2g/64 = 187 mg; the 0.5 g step gives slope 250 mg
    d.wreg(0x5E, 0x20)                            # INT1_WU
    d.wreg(CTRL1_XL, 0x30)
    d.advance(190_000)
    assert d.ctx.pin[0] is False
    d.advance(40_000)
    assert d.ctx.pin[0] is True
    src = d.rreg(0x1B)
    assert src & 0x08 and src & 0x04              # WU_IA, X_WU
    assert d.ctx.pin[0] is False                  # cleared immediately on read
    d.advance(100_000)
    assert d.ctx.pin[0] is False                  # the step is over, slope is zero again


def test_wake_up_needs_interrupts_enable_and_duration():
    prof = [{"t_ms": 0, "accel_g": [0, 0, 1]}, {"t_ms": 100, "accel_g": [0.5, 0, 1]}, {"t_ms": 120, "accel_g": [0, 0, 1]}]
    d = make({"seed": 1, "profile": prof})
    d.wreg(0x5B, 0x04)
    d.wreg(0x5E, 0x20)
    d.wreg(CTRL1_XL, 0x40)                        # 104 Hz
    d.advance(200_000)
    assert d.rreg(0x1B) == 0                      # INTERRUPTS_ENABLE clear: nothing
    d.wreg(0x58, 0x80)
    d.wreg(0x5C, 0x40)                            # WAKE_DUR = 2: needs 3 samples; a 120 ms step is 2 slope hits
    d.advance(50_000)
    assert d.rreg(0x1B) == 0


def test_sw_reset_and_boot_timing():
    d = make()
    d.wreg(CTRL1_XL, 0x30)
    d.wreg(CTRL3_C, 0x01)
    assert d.rreg(CTRL3_C) & 1                    # still resetting
    d.advance(60)
    assert d.rreg(CTRL3_C) == 0x04                # defaults back, SW_RESET self-cleared
    assert d.rreg(CTRL1_XL) == 0
    d.wreg(INT1_CTRL, 0x04)
    d.wreg(CTRL3_C, 0x84)
    assert d.ctx.pin[0] is True                   # INT1_BOOT high while booting
    d.advance(10_100)
    assert d.rreg(CTRL3_C) & 0x80 == 0 and d.ctx.pin[0] is False


def test_peek_flow_like_pebbleos():
    """What the PebbleOS peek does: ODR on, poll STATUS every 5 ms, read, ODR off. Must beat 100 ms."""
    d = make()
    d.wreg(CTRL1_XL, 0x30)
    t = 0
    while not d.rreg(STATUS) & 1:
        d.advance(5_000)
        t += 5
        assert t < 100
    assert 15 <= t <= 25
    assert abs(accel(d)[2] - 16393) < 120


def test_scripted_motion_property():
    d = make()
    d.p.set_property("accel_g", [0, 1, 0])
    d.wreg(CTRL1_XL, 0x30)
    d.advance(25_000)
    assert abs(accel(d)[1] - 16393) < 120
