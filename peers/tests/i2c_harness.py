"""Shared fake host for the I2C peer tests: the one-shot timer the real ctx has, recorded GPIO levels and
register-style transfers. No firmware, no simulator."""
import importlib.util
from pathlib import Path

PEERS = Path(__file__).resolve().parent.parent / "i2c"


class Gpio:
    def __init__(self, ctx, line):
        self.ctx, self.line = ctx, line

    def set(self, level=True):
        self.ctx.levels.append((self.ctx.now_us, self.line, bool(level)))
        self.ctx.pin[self.line] = bool(level)


class Ctx:
    def __init__(self, config=None):
        self.config = config or {}
        self.now_us = 0
        self.deadline = None
        self.levels = []
        self.pin = {}

    def gpio(self, line):
        return Gpio(self, line)

    def schedule_oneshot(self, delay_us, fn=None):
        self.deadline = self.now_us + delay_us

    def cancel_timer(self):
        self.deadline = None

    def info(self, msg):
        pass

    debug = warning = error = noisy = info


class Dev:
    """A peer plus its fake host: advance() runs virtual time and fires the peer's timer."""

    def __init__(self, name, addr, config=None):
        spec = importlib.util.spec_from_file_location(name, PEERS / f"{name}.py")
        self.mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.mod)
        self.ctx = Ctx(config)
        self.addr = addr
        self.p = None

    def start(self, boot_ms=0):
        self.p = self.mod.Slave(self.ctx)
        self.advance(boot_ms * 1000)
        return self

    def advance(self, us):
        end = self.ctx.now_us + us
        while self.ctx.deadline is not None and self.ctx.deadline <= end:
            self.ctx.now_us = self.ctx.deadline
            self.ctx.deadline = None
            self.p.on_timer()
        self.ctx.now_us = end

    def write(self, data):
        ack = self.p.on_address(self.addr, False)
        self.p.on_write(bytes(data))
        return ack

    def wreg(self, reg, *vals):
        return self.write([reg, *vals])

    def read(self, reg, n=1):
        self.write([reg])
        self.p.on_address(self.addr, True)
        return self.p.on_read(n)

    def rreg(self, reg):
        return self.read(reg, 1)[0]
