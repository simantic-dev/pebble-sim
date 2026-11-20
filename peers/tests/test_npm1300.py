"""nPM1300 peer self-tests (peers/i2c/npm1300.py): ADC timing and results, events and IRQ pin, scripted VBUS/charge.

    python3 -m pytest peers/tests/test_npm1300.py
"""
from i2c_harness import Dev


def make(config=None):
    return Dev("npm1300", 0x6B, config or {}).start()


def rd(d, reg, n=1):
    d.write([reg >> 8, reg & 0xFF])
    d.p.on_address(d.addr, True)
    return d.p.on_read(n)


def wr(d, reg, *vals):
    d.write([reg >> 8, reg & 0xFF, *vals])


def test_adc_not_instant_and_vbat_result():
    d = make({"vbat_mv": 3900})
    wr(d, 0x0003, 0xFF)
    wr(d, 0x0500, 1)
    assert rd(d, 0x0003)[0] & 1 == 0
    d.advance(249)
    assert rd(d, 0x0003)[0] & 1 == 0
    d.advance(2)
    assert rd(d, 0x0003)[0] & 1
    raw = rd(d, 0x0511)[0] << 2 | rd(d, 0x0515)[0] & 3
    assert abs(raw * 5000 // 1023 - 3900) < 6


def test_tasks_queue_serially_and_ibat_follows_vbat():
    d = make()
    wr(d, 0x0524, 1)
    wr(d, 0x0500, 1)                       # VBAT then IBAT: 500 us
    wr(d, 0x0503, 1)                       # VSYS after them: 750 us
    d.advance(300)
    assert rd(d, 0x0003)[0] & 0x41 == 0x01
    d.advance(300)
    assert rd(d, 0x0003)[0] & 0x41 == 0x41 and rd(d, 0x0003)[0] & 0x08 == 0
    d.advance(200)
    assert rd(d, 0x0003)[0] & 0x08


def test_scripted_vbat_sampled_at_conversion_end():
    d = make({"profile": [{"t_ms": 10, "vbat_mv": 3500}], "vbat_mv": 4100})
    d.advance(9_900)
    wr(d, 0x0500, 1)
    d.advance(300)                         # conversion ends at 10.15 ms: the scripted 3.5 V
    raw = rd(d, 0x0511)[0] << 2 | rd(d, 0x0515)[0] & 3
    assert abs(raw * 5000 // 1023 - 3500) < 6


def test_vbus_event_irq_and_status():
    d = make({"profile": [{"t_ms": 5, "vbus": 1}]})
    wr(d, 0x0601, 5)                       # GPOIRQ
    wr(d, 0x0018, 0x03)                    # INTENEVENTSVBUSIN0SET
    d.advance(6_000)
    assert rd(d, 0x0207)[0] & 1
    assert rd(d, 0x0017)[0] & 1            # VBUSDETECTED pending
    assert d.ctx.pin[0] is True
    wr(d, 0x0017, 0x03)
    assert d.ctx.pin[0] is False


def test_irq_gated_by_inten_and_mode():
    d = make({"profile": [{"t_ms": 5, "vbus": 1}]})
    d.advance(6_000)
    assert d.ctx.pin.get(0) in (None, False)


def test_charge_status_and_complete_event():
    d = make({"vbus": 1, "charge": "cc", "profile": [{"t_ms": 20, "charge": "complete"}]})
    assert rd(d, 0x0334)[0] == 0x08
    wr(d, 0x000C, 0x10)
    wr(d, 0x0601, 5)
    d.advance(21_000)
    assert rd(d, 0x0334)[0] == 0x02
    assert rd(d, 0x000B)[0] & 0x10
    assert d.ctx.pin[0] is True
