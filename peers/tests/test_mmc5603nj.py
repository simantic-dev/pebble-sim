"""MMC5603NJ peer self-tests (peers/i2c/mmc5603nj.py): power-on, product ID, measurement timing and flags,
BW, Auto_SR, inhibit, continuous mode, SW reset.

    python3 -m pytest peers/tests/test_mmc5603nj.py
"""
from i2c_harness import Dev

STATUS, ODR, C0, C1, C2, TOUT = 0x18, 0x1A, 0x1B, 0x1C, 0x1D, 0x09
M_DONE, T_DONE = 0x40, 0x80


def make(config=None):
    return Dev("mmc5603nj", 0x30, config or {"seed": 3}).start(boot_ms=6)


def field(d):
    r = d.read(0, 9)
    out = []
    for i in range(3):
        raw = r[2 * i] << 12 | r[2 * i + 1] << 4 | r[6 + i] >> 4
        out.append((raw - 524288) / 16384)
    return out


def test_no_ack_before_top_and_product_id():
    d = Dev("mmc5603nj", 0x30).start()
    assert d.write([0x39]) is False                # tOp = 5 ms
    d.advance(5_000)
    assert d.rreg(0x39) == 0x10
    assert d.rreg(STATUS) & 0x10                   # OTP_read_done


def test_single_measurement_time_per_bw_and_done_clear_on_read():
    for bw, t_us in enumerate((6600, 3500, 2000, 1200)):
        d = make()
        d.wreg(C1, bw)
        d.wreg(C0, 0x01)                           # Take_meas_M
        d.advance(t_us - 100)
        assert d.rreg(STATUS) & M_DONE == 0
        d.advance(200)
        assert d.rreg(STATUS) & M_DONE
        assert d.rreg(C0) & 1 == 0                 # TM_M self-cleared
        d.read(0, 1)
        assert d.rreg(STATUS) & M_DONE == 0        # cleared by a data read


def test_auto_sr_doubles_measurement_time():
    d = make()
    d.wreg(C0, 0x21)                               # Auto_SR_en | Take_meas_M, BW 6.6 ms
    d.advance(13_000)
    assert d.rreg(STATUS) & M_DONE == 0
    d.advance(400)
    assert d.rreg(STATUS) & M_DONE


def test_field_scaling_and_null():
    d = make({"seed": 1, "field_gauss": [1.0, -0.5, 0.0]})
    d.wreg(C0, 0x01)
    d.advance(7_000)
    x, y, z = field(d)
    assert abs(x - 1.0) < 0.01 and abs(y + 0.5) < 0.01 and abs(z) < 0.01   # 1.5 mG rms noise


def test_inhibit_keeps_last_value():
    d = make({"seed": 1, "field_gauss": [1.0, 0.0, 0.0]})
    d.wreg(C0, 0x01)
    d.advance(7_000)
    d.p.set_property("field_gauss", [2.0, 0.0, 0.0])
    d.wreg(C1, 0x04)                               # X-inhibit
    d.wreg(C0, 0x01)
    d.advance(7_000)
    assert abs(field(d)[0] - 1.0) < 0.01


def test_temperature_measurement():
    d = make({"temp_c": 25.0})
    d.wreg(C0, 0x02)                               # Take_meas_T
    d.advance(7_000)
    assert d.rreg(STATUS) & T_DONE
    assert d.rreg(TOUT) == 125                     # (25 + 75) / 0.8
    assert d.rreg(STATUS) & T_DONE == 0


def test_set_reset_bits_self_clear():
    d = make()
    d.wreg(C0, 0x08)
    d.wreg(C0, 0x10)
    assert d.rreg(C0) == 0


def test_continuous_mode_rate():
    d = make()
    d.wreg(ODR, 100)
    d.wreg(C0, 0x80)                               # Cmm_freq_en
    d.wreg(C2, 0x10)                               # Cmm_en
    seen = 0
    for _ in range(100):                           # poll each ms for 100 ms, clearing the flag by reading
        d.advance(1_000)
        if d.rreg(STATUS) & M_DONE:
            seen += 1
            d.read(0, 1)
    assert 9 <= seen <= 11                         # 100 Hz
    d.wreg(C2, 0x00)
    d.advance(30_000)
    d.read(0, 1)
    d.advance(30_000)
    assert d.rreg(STATUS) & M_DONE == 0


def test_odr_above_sustainable_runs_back_to_back():
    d = make()
    d.wreg(C0, 0x20)                               # Auto_SR_en, BW 6.6 ms: 13.2 ms per sample
    d.wreg(ODR, 200)
    d.wreg(C0, 0xA0)
    d.wreg(C2, 0x10)
    d.advance(14_000)
    assert d.rreg(STATUS) & M_DONE
    d.read(0, 1)
    d.advance(5_000)                               # 5 ms = the 200 Hz period; not yet
    assert d.rreg(STATUS) & M_DONE == 0


def test_sw_reset_clears_registers_and_blocks_for_20ms():
    d = make()
    d.wreg(C1, 0x03)
    assert d.rreg(C1) == 0x03
    d.wreg(C1, 0x80)
    d.advance(19_000)
    assert d.write([0x39]) is False
    d.advance(1_500)
    assert d.rreg(C1) == 0 and d.rreg(0x39) == 0x10
