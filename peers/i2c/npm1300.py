# Nordic nPM1300 PMIC at 7-bit 0x6b (Pebble 2 Duo, TWIM1) as a scripted I2C peer. Registers are 16 bit addressed: the first
# two bytes of a write are base (high) and offset (low), then data; reads auto-increment.
# SOURCE CAVEAT: the nPM1300 Product Specification could not be read (~/dev/daeta_sheets/npm1300_ps.pdf is a Cloudflare
# challenge page, not a PDF), so nothing here is cited to a PS table. Addresses, bit positions and transfer functions come
# from PebbleOS drivers/pmic/npm1300.c (pinned commit) and the ADC conversion time from Zephyr's npm13xx_charger.c
# (ADC_CONV_TIME_US = 250). Unverified against the PS: every value below.
# Wiring: ctx line 0 (numberOfGpios: 1) is the IRQ output (GPIO1.12 in asterix.dts, GPIOMODE1 = 5 "GPOIRQ").
#
# Modelled
#  - Event groups with SET / CLR / INTENSET / INTENCLR at base+0..3: ADC (0x0002), BCHARGER1 (0x000A), VBUSIN0 (0x0016).
#    EVENTS*CLR reads the pending events and a write of 1 clears them; INTEN* gates the IRQ. The IRQ pin is high while any
#    enabled event is pending and GPIOMODE1 is 5.
#  - ADC: TASKVBATMEASURE 0x0500, TASKNTCMEASURE 0x0501, TASKVSYSMEASURE 0x0503, TASKIBATMEASURE 0x0506 run one after
#    another, 250 us each; the result registers and the EVENTSADC ready bit appear when the conversion ends, from the
#    state at that moment. ADCIBATMEASEN bit 0 makes a VBAT task also measure IBAT. VBAT full scale 5.0 V, VSYS 6.375 V,
#    10 bit (MSB + 2 LSBs); IBAT full scale from the driver's formulas (discharge 1.12 * 200 mA limit, charge
#    1.25 * BCHGISET), ADCIBATMEASSTATUS mode 1 discharging / 3 charging.
#  - VBUSINSTATUS (0x0207 bit 0) and BCHGCHARGESTATUS (0x0334: bit 1 completed, 2 trickle, 3 constant current, 4 constant
#    voltage) follow the scripted state; a VBUS change sets VBUSDETECTED (bit 0) / VBUSREMOVED (bit 1), charge completion
#    sets CHGCOMPLETED (BCHARGER1 bit 4).
#  - Everything else is a stored register file.
# State (config JSON): vbat_mv (3900), vsys_mv (3900), ibat_ua (5000, magnitude), ntc_raw (512), vbus (0/1),
# charge ("off" | "trickle" | "cc" | "cv" | "complete"), and "profile": [{"t_ms": ..., any of those}, ...] (piecewise
# constant). The machine can write the same names as properties.
# MODELED SIMPLIFICATION: battery voltage is scripted, not a discharge curve; the charger state machine, NTC, die temperature,
#   regulators, ship mode and watchdog are not modelled; ADC timing is the vendor-driver 250 us, not a PS figure.
import math

EVENTSADCSET, EVENTSADCCLR = 0x0002, 0x0003
GROUPS = {0x0002: "adc", 0x000A: "chg", 0x0016: "vbus"}
VBATRDY, NTCRDY, VSYSRDY, IBATRDY = 0x01, 0x02, 0x08, 0x40
TASKS = {0x0500: VBATRDY, 0x0501: NTCRDY, 0x0503: VSYSRDY, 0x0506: IBATRDY}
VBUSINSTATUS, BCHGCHARGESTATUS, GPIOMODE1, ADCIBATMEASSTATUS, ADCIBATMEASEN = 0x0207, 0x0334, 0x0601, 0x0510, 0x0524
BCHGISETMSB, BCHGISETLSB = 0x0308, 0x0309
VBATMSB, NTCMSB, VSYSMSB, GP0LSBS, VBAT2MSB, GP1LSBS = 0x0511, 0x0512, 0x0514, 0x0515, 0x0518, 0x051A
CHARGE_BITS = {"off": 0, "trickle": 0x04, "cc": 0x08, "cv": 0x10, "complete": 0x02}
ADC_US = 250
GPOIRQ = 5
NAMES = ("vbat_mv", "vsys_mv", "ibat_ua", "ntc_raw", "vbus", "charge")


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        cfg = getattr(ctx, "config", None) or {}
        self.st = {"vbat_mv": 3900, "vsys_mv": 3900, "ibat_ua": 5000, "ntc_raw": 512, "vbus": 0, "charge": "off"}
        self.st.update({k: cfg[k] for k in NAMES if k in cfg})
        self.profile = sorted(cfg.get("profile", []), key=lambda p: p.get("t_ms", 0))
        self.regs = {}
        self.ev = {g: 0 for g in GROUPS.values()}
        self.inten = {g: 0 for g in GROUPS.values()}
        self.queue = []                      # [(done_us, ready_bit)]
        self.adc_free = 0
        self.ptr, self.pins = 0, False
        self.vbus_seen = int(bool(self.st["vbus"]))
        self.charge_seen = self.st["charge"]

    def set_property(self, name, value):
        self.st[name] = value

    def get_property(self, name):
        return self.st[name]

    def _state(self):
        now = self.ctx.now_us
        s = dict(self.st)
        for p in self.profile:
            if p.get("t_ms", 0) * 1000 <= now:
                s.update({k: v for k, v in p.items() if k in NAMES})
        return s

    # ---- time -----------------------------------------------------------------------------------------
    def _sync(self):
        now, s = self.ctx.now_us, self._state()
        for done, ev in [q for q in self.queue if q[0] <= now]:
            self.queue.remove((done, ev))
            self._measure(ev, s)
            self.ev["adc"] |= ev
        vbus = int(bool(s["vbus"]))
        if vbus != self.vbus_seen:
            self.ev["vbus"] |= 0x01 if vbus else 0x02
            self.vbus_seen = vbus
        if s["charge"] != self.charge_seen:
            if s["charge"] == "complete":
                self.ev["chg"] |= 0x10
            self.charge_seen = s["charge"]
        self.regs[VBUSINSTATUS] = vbus
        self.regs[BCHGCHARGESTATUS] = CHARGE_BITS[s["charge"]]
        self.regs[ADCIBATMEASSTATUS] = (3 if s["charge"] in ("trickle", "cc", "cv") else 1) << 2
        self._irq()
        pend = [q[0] for q in self.queue] + [int(p["t_ms"] * 1000) for p in self.profile if p.get("t_ms", 0) * 1000 > now]
        if pend:
            self.ctx.schedule_oneshot(max(1, math.ceil(min(pend) - now)))
        else:
            self.ctx.cancel_timer()

    def on_timer(self):
        self._sync()

    def _irq(self):
        level = self.regs.get(GPIOMODE1, 0) == GPOIRQ and any(self.ev[g] & self.inten[g] for g in self.ev)
        if level != self.pins:
            self.pins = level
            self.ctx.gpio(0).set(level)

    def _measure(self, ev, s):
        r = self.regs
        if ev == VBATRDY:
            raw = min(1023, s["vbat_mv"] * 1023 // 5000)
            r[VBATMSB] = raw >> 2
            r[GP0LSBS] = (r.get(GP0LSBS, 0) & ~0x03) | (raw & 3)
        elif ev == NTCRDY:
            r[NTCMSB] = s["ntc_raw"] >> 2
            r[GP0LSBS] = (r.get(GP0LSBS, 0) & ~0x0C) | ((s["ntc_raw"] & 3) << 2)
        elif ev == VSYSRDY:
            raw = min(1023, s["vsys_mv"] * 1023 // 6375)
            r[VSYSMSB] = raw >> 2
            r[GP0LSBS] = (r.get(GP0LSBS, 0) & ~0xC0) | ((raw & 3) << 6)
        elif ev == IBATRDY:
            if s["charge"] in ("trickle", "cc", "cv"):
                iset_ma = (((r.get(BCHGISETMSB, 0) << 1) | (r.get(BCHGISETLSB, 0) & 1)) * 2) or 1
                full = iset_ma * 1000 * 125 // 100
            else:
                full = 200_000 * 112 // 100          # the driver's 200 mA discharge limit
            raw = min(1023, s["ibat_ua"] * 1023 // full)
            r[VBAT2MSB] = raw >> 2
            r[GP1LSBS] = (r.get(GP1LSBS, 0) & ~0x30) | ((raw & 3) << 4)

    def _task(self, ev):
        start = max(self.ctx.now_us, self.adc_free)
        self.adc_free = start + ADC_US
        self.queue.append((self.adc_free, ev))

    # ---- host interface -------------------------------------------------------------------------------
    def on_address(self, addr, is_read):
        return True

    def on_write(self, data):
        self._sync()
        if len(data) < 2:
            return len(data)
        self.ptr = (data[0] << 8) | data[1]
        for b in data[2:]:
            self._write(self.ptr, b)
            self.ptr += 1
        self._sync()
        return len(data)

    def _write(self, a, b):
        for base, g in GROUPS.items():
            if base <= a <= base + 3:
                k = a - base
                if k == 1:
                    self.ev[g] &= ~b
                elif k == 2:
                    self.inten[g] |= b
                elif k == 3:
                    self.inten[g] &= ~b
                return
        if a in TASKS and b & 1:
            self._task(TASKS[a])
            if a == 0x0500 and self.regs.get(ADCIBATMEASEN, 0) & 1:
                self._task(IBATRDY)
        elif a in (VBUSINSTATUS, BCHGCHARGESTATUS, ADCIBATMEASSTATUS):
            pass                                     # read-only status
        else:
            self.regs[a] = b

    def on_read(self, count):
        self._sync()
        out = bytearray()
        for _ in range(count):
            a = self.ptr
            g = next((g for base, g in GROUPS.items() if a in (base, base + 1)), None)
            out.append((self.ev[g] if g else self.regs.get(a, 0)) & 0xFF)
            self.ptr += 1
        return bytes(out)
