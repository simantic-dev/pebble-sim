"""OPT3001 peer self-tests (peers/i2c/opt3001.py): conversion time, single-shot / continuous, CRF, range, reset.

    python3 -m pytest peers/tests/test_opt3001.py
"""
from i2c_harness import Dev

CONFIG, RESULT = 0x01, 0x00
SINGLE_100, SINGLE_800, CONT_100 = 0xC200, 0xCA00, 0xC600     # RN auto, CT, M


def make(lux=120.0):
    return Dev("opt3001", 0x44, {"lux": lux}).start()


def r16(d, reg):
    b = d.read(reg, 2)
    return b[0] << 8 | b[1]


def w16(d, reg, v):
    d.wreg(reg, v >> 8, v & 0xFF)


def lux(d):
    v = r16(d, RESULT)
    return 0.01 * 2 ** (v >> 12) * (v & 0xFFF)


def test_ids_and_reset_values():
    d = make()
    assert (r16(d, 0x7E), r16(d, 0x7F), r16(d, CONFIG), r16(d, 3)) == (0x5449, 0x3001, 0xC810, 0xBFFF)
    assert r16(d, RESULT) == 0


def test_first_read_after_single_shot_returns_previous_result():
    d = make()
    w16(d, CONFIG, SINGLE_100)
    assert r16(d, RESULT) == 0                       # silicon behaviour PebbleOS sees: old (reset) result
    d.advance(114_000)                               # 10 ms assess + 100 ms + 3 ms
    assert r16(d, RESULT) != 0 and abs(lux(d) - 120) < 0.7


def test_single_shot_timing_100_and_800_ms():
    for cfg, ms in ((SINGLE_100, 113), (SINGLE_800, 813)):
        d = make()
        w16(d, CONFIG, cfg)
        d.advance((ms - 1) * 1000)
        assert r16(d, CONFIG) & 0x80 == 0 and (r16(d, CONFIG) >> 9) & 3 == 1   # still converting, reads 01
        d.advance(2_000)
        c = r16(d, CONFIG)
        assert c & 0x80 and (c >> 9) & 3 == 0        # CRF set, back to shutdown
        assert r16(d, CONFIG) & 0x80 == 0            # a config read clears CRF


def test_continuous_repeats_every_conversion_time():
    d = make()
    w16(d, CONFIG, CONT_100)
    d.advance(114_000)
    assert r16(d, CONFIG) & 0x80                     # CRF; reading cleared it
    d.advance(100_000)
    assert r16(d, CONFIG) & 0x80 == 0                # 103 ms per conversion, none yet
    d.advance(5_000)
    assert r16(d, CONFIG) & 0x80


def test_config_write_aborts_and_shutdown_write_keeps_crf():
    d = make()
    w16(d, CONFIG, SINGLE_100)
    d.advance(50_000)
    w16(d, CONFIG, 0xC010)                           # shutdown aborts
    d.advance(200_000)
    assert r16(d, CONFIG) & 0x80 == 0 and r16(d, RESULT) == 0
    w16(d, CONFIG, SINGLE_100)
    d.advance(114_000)
    w16(d, CONFIG, 0xC010)                           # writing shutdown does not clear CRF
    assert r16(d, CONFIG) & 0x80


def test_auto_range_and_manual_range_ovf_me():
    d = make(1000.0)
    w16(d, CONFIG, SINGLE_100)
    d.advance(114_000)
    assert r16(d, RESULT) >> 12 == 5                 # 1310.40 lux range (Table 8)
    assert abs(lux(d) - 1000) < 0.7
    w16(d, CONFIG, 0x0000 | 0x0200)                  # manual range 0 (40.95 lux), 100 ms, single-shot
    d.advance(103_000)
    c = r16(d, CONFIG)
    assert c & 0x100                                 # OVF
    assert r16(d, RESULT) == 0x0FFF
    w16(d, CONFIG, 0x4200 | 0x0004)                  # range 4, ME
    d.advance(103_000)
    assert r16(d, RESULT) >> 12 == 0


def test_general_call_reset():
    d = make()
    w16(d, CONFIG, CONT_100)
    d.p.on_address(0, False)
    d.p.on_write(b"\x06")
    assert r16(d, CONFIG) == 0xC810
