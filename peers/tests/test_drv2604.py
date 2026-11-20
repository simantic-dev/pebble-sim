"""DRV2604 peer self-tests (peers/i2c/drv2604.py): EN gating, reset values, GO self-clear, auto-cal, RTP.

    python3 -m pytest peers/tests/test_drv2604.py
"""
from i2c_harness import Dev

MODE, RTP, GO, CTRL4 = 0x01, 0x02, 0x0C, 0x1E


def make(config=None, en=True):
    d = Dev("drv2604", 0x5A, config).start()
    if en:
        d.p.on_gpio(0, True)
    return d


def test_reset_values():
    d = make()
    assert d.rreg(0x00) == 0x80                    # DEVICE_ID 4
    assert d.rreg(MODE) == 0x40                    # STANDBY
    assert (d.rreg(0x16), d.rreg(0x17), d.rreg(0x18), d.rreg(0x19)) == (0x3F, 0x89, 0x0D, 0x6D)
    assert d.rreg(CTRL4) == 0x20


def test_en_low_acks_but_no_access_and_registers_kept():
    d = make(en=False)
    assert d.write([0x16, 0x20]) is True
    assert d.rreg(0x16) == 0xFF
    d.p.on_gpio(0, True)
    assert d.rreg(0x16) == 0x3F                    # the write was dropped
    d.wreg(0x16, 0x20)
    d.p.on_gpio(0, False)
    d.p.on_gpio(0, True)
    assert d.rreg(0x16) == 0x20                    # registers are not reset by EN


def test_dev_reset_restores_defaults():
    d = make()
    d.wreg(0x16, 0x20)
    d.wreg(MODE, 0x80)
    assert d.rreg(0x16) == 0x3F and d.rreg(MODE) == 0x40


def test_go_plays_waits_then_self_clears():
    d = make()
    d.wreg(MODE, 0x00)                             # out of standby, internal trigger
    d.wreg(0x04, 0x80 | 5)                         # wait 50 ms
    d.wreg(0x05, 0x80 | 3)                         # wait 30 ms
    d.wreg(0x06, 0x00)
    d.wreg(GO, 1)
    assert d.rreg(GO) == 1
    d.advance(79_000)
    assert d.rreg(GO) == 1
    d.advance(2_000)
    assert d.rreg(GO) == 0


def test_go_cancel_and_standby_stop():
    d = make()
    d.wreg(MODE, 0x00)
    d.wreg(0x04, 0x80 | 50)
    d.wreg(GO, 1)
    d.wreg(GO, 0)
    assert d.rreg(GO) == 0
    d.wreg(GO, 1)
    d.wreg(MODE, 0x40)
    assert d.rreg(GO) == 0


def test_go_ignored_in_standby():
    d = make()
    d.wreg(0x04, 0x80 | 5)
    d.wreg(GO, 1)
    assert d.rreg(GO) == 0


def test_autocal_duration_per_auto_cal_time_and_results():
    for t, (lo, hi) in enumerate(((150, 350), (250, 450), (500, 700), (1000, 1200))):
        d = make({"cal_comp": 0x11, "cal_bemf": 0x77})
        d.wreg(MODE, 0x07)
        d.wreg(CTRL4, t << 4)
        d.wreg(GO, 1)
        mid = (lo + hi) // 2
        d.advance((mid - 1) * 1000)
        assert d.rreg(GO) == 1
        d.advance(2_000)
        assert d.rreg(GO) == 0
        assert d.rreg(0x18) == 0x11 and d.rreg(0x19) == 0x77
        assert d.rreg(0x00) & 0x08 == 0            # passed


def test_autocal_fail_flag_clears_on_read():
    d = make({"cal_fail": True})
    d.wreg(MODE, 0x07)
    d.wreg(GO, 1)
    d.advance(1_000_000)
    assert d.rreg(0x00) & 0x08
    assert d.rreg(0x00) & 0x08 == 0


def test_rtp_drives_and_reports_vbat_and_period():
    d = make()
    d.wreg(MODE, 0x05)
    assert d.rreg(0x21) == 0                       # not driving until RTP_INPUT is non-zero
    d.wreg(RTP, 0x40)
    assert d.rreg(GO) == 0                         # RTP does not touch register 0x0C
    assert d.rreg(0x21) == round(3.7 * 255 / 5.6)
    assert abs(d.rreg(0x22) * 98.46 - 1e6 / 235) < 100


def test_en_low_ends_process():
    d = make()
    d.wreg(MODE, 0x00)
    d.wreg(0x04, 0x80 | 50)
    d.wreg(GO, 1)
    d.p.on_gpio(0, False)
    d.p.on_gpio(0, True)
    assert d.rreg(GO) == 0
