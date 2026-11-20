"""DA7212 peer self-tests (peers/i2c/da7212.py): standby, PLL lock conditions and timing, status bits, soft reset.

    python3 -m pytest peers/tests/test_da7212.py
"""
from i2c_harness import Dev

PLL_STATUS, CIF, FTOP, FBOT, INT, PLL_CTRL, DAI, ACTIVE = 0x03, 0x1D, 0x24, 0x25, 0x26, 0x27, 0x28, 0xFD
LOCK, SRM, MCLK = 1, 2, 4


def make(config=None):
    return Dev("da7212", 0x1A, config).start()


def pll_4mhz_to_12288(d):
    """The register values PebbleOS writes: 4 MHz MCLK, PLL_INDIV = 0, 49 + 0x4DD / 8192."""
    d.wreg(ACTIVE, 1)
    d.wreg(FTOP, 0x04)
    d.wreg(FBOT, 0xDD)
    d.wreg(INT, 0x31)
    d.wreg(PLL_CTRL, 0x80)


def test_standby_status_has_no_lock_and_access_wakes():
    d = make()
    d.wreg(ACTIVE, 1)
    assert d.rreg(PLL_STATUS) == MCLK | 8          # MCLK present, PLL bypassed (PLL_EN clear)
    d.wreg(ACTIVE, 0)
    assert d.rreg(0x40) == 0 and d.rreg(ACTIVE) == 1   # any access asserts SYSTEM_ACTIVE


def test_pll_lock_timing_and_status_bits():
    d = make()
    pll_4mhz_to_12288(d)
    assert d.rreg(PLL_STATUS) == MCLK              # PLL_LOCK is not instant
    d.advance(999)
    assert d.rreg(PLL_STATUS) & LOCK == 0
    d.advance(2)
    assert d.rreg(PLL_STATUS) == LOCK | MCLK       # PebbleOS requires LOCK | MCLK
    assert abs(d.p.get_property("sysclk_hz") - 12.288e6) < 1e3


def test_srm_lock_needs_srm_slave_and_wclk():
    d = make({"wclk_present": True})
    pll_4mhz_to_12288(d)
    d.wreg(PLL_CTRL, 0xC0)                         # PLL_EN | PLL_SRM_EN
    d.wreg(DAI, 0x00)                              # DAI slave
    d.advance(2_000)
    assert d.rreg(PLL_STATUS) & SRM
    d.wreg(DAI, 0x81)                              # DAI master: SRM not available (Table 33)
    assert d.rreg(PLL_STATUS) & SRM == 0
    d2 = make()
    pll_4mhz_to_12288(d2)
    d2.wreg(PLL_CTRL, 0xC0)
    d2.advance(2_000)
    assert d2.rreg(PLL_STATUS) & SRM == 0          # master with no WCLK: the driver's recovery path is not taken


def test_no_lock_when_mclk_missing_or_fref_out_of_range():
    d = make({"mclk_hz": 500_000})
    pll_4mhz_to_12288(d)
    d.advance(5_000)
    assert d.rreg(PLL_STATUS) & (LOCK | MCLK) == 0
    d = make({"mclk_hz": 12_288_000})
    pll_4mhz_to_12288(d)                           # INDIV = 0 gives F_REF 6.1 MHz: outside 2-5 MHz
    d.advance(5_000)
    assert d.rreg(PLL_STATUS) & LOCK == 0
    d.wreg(PLL_CTRL, 0x84)                         # INDIV = 01 (div 4): F_REF 3.07 MHz
    d.advance(2_000)
    assert d.rreg(PLL_STATUS) & LOCK


def test_rewriting_pll_integer_restarts_lock():
    d = make()
    pll_4mhz_to_12288(d)
    d.advance(2_000)
    assert d.rreg(PLL_STATUS) & LOCK
    d.wreg(INT, 0x31)
    assert d.rreg(PLL_STATUS) & LOCK == 0
    d.advance(1_001)
    assert d.rreg(PLL_STATUS) & LOCK


def test_standby_drops_lock():
    d = make()
    pll_4mhz_to_12288(d)
    d.advance(2_000)
    d.wreg(ACTIVE, 0)
    d.wreg(ACTIVE, 1)
    assert d.rreg(PLL_STATUS) & LOCK == 0          # lock restarts when the oscillator restarts
    d.advance(1_001)
    assert d.rreg(PLL_STATUS) & LOCK


def test_soft_reset_returns_defaults_and_standby():
    d = make()
    pll_4mhz_to_12288(d)
    d.wreg(CIF, 0x80)
    assert d.rreg(PLL_CTRL) == 0x0C and d.rreg(INT) == 0x20 and d.rreg(CIF) == 0
    assert d.rreg(PLL_STATUS) & LOCK == 0


def test_status_registers_read_only():
    d = make()
    d.wreg(PLL_STATUS, 0xFF)
    d.wreg(ACTIVE, 1)
    assert d.rreg(PLL_STATUS) == MCLK | 8
