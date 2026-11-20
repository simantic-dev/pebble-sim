# The phone, as a ScriptedBlePeer in the central role (it defines on_adv). It scans, connects to the watch
# after CONNECT_AT_US, pairs (LE Secure Connections, Just Works), encrypts, finds the watch's PPoGATT service
# (reversed role: the watch is the GATT server), opens the PPoGATT session, then does what the Pebble phone app
# does: answers the watch's version request, sets the time (UTC + offset), asks which app runs, and only then
# pushes one notification as a BlobDB insert (Pebble Protocol endpoint 0xB1DB). Link layer, SMP and LL encryption are pyrite's
# ScriptedBlePeer; this file is only the phone's GATT client and Pebble Protocol.
import struct
import time

NAMES = {0x01: "flags", 0x02: "uuid16", 0x03: "uuid16", 0x06: "uuid128", 0x07: "uuid128",
         0x08: "name", 0x09: "name", 0x0A: "tx_power", 0x16: "svc_data16", 0xFF: "mfg"}


def ad_fields(data):
    i = 0
    while i + 1 < len(data) and data[i]:
        yield data[i + 1], data[i + 2:i + 1 + data[i]]
        i += 1 + data[i]


def describe(data):
    out = []
    for t, v in ad_fields(bytes(data)):
        if t in (0x08, 0x09):
            out.append('name="%s"' % v.decode(errors="replace"))
        else:
            out.append("%s=%s" % (NAMES.get(t, "ad%02x" % t), v.hex()))
    return " ".join(out)



PAIR_DELAY_US = 500_000
CONNECT_AT_US = int(23.0e6)   # after the button sequence of sim/scenario.yaml
BASE = bytes.fromhex("328E0FBBC6421AA6699BDADA")  # Pebble base UUID tail, as written in PebbleOS pebble_bt.h


def pebble_uuid(u32):                      # 128-bit UUID in ATT wire order (little endian)
    return (struct.pack(">I", u32) + BASE)[::-1]


PPOG_SVC, PPOG_NOTIFY, PPOG_WRITE = (pebble_uuid(u) for u in (0x40000000, 0x40000001, 0x40000003))
PHONE_NAME = b"Sim Phone"
# An X post as the Pebble app forwards it (coredevices/mobileapp BasicNotificationProcessor.kt: title = EXTRA_TITLE,
# body = EXTRA_TEXT; NotificationProperties.kt: com.twitter.android -> TimelineIcon.NotificationTwitter, PebbleOS TIMELINE_RESOURCE_NOTIFICATION_TWITTER = 0x80000006).
# Post: x.com/shong0504, 2026-11-19 12:37 PM local. X's exact EXTRA_TITLE wording is not verified.
NOTIFICATION = {1: "Seungmin Hong", 3: "Simulate a @Pebble", 4: struct.pack("<I", 0x8000_0006)}  # title, body, tiny icon (system resource: high bit set)


def blobdb_notification(attrs, uuid=bytes(range(1, 17))):
    """Pebble Protocol message: BlobDB INSERT of a TimelineItem notification (db 4), PebbleOS
    timeline/item.h SerializedTimelineItemHeader, attributes title=1, body=3, tiny icon=4 (uint32)."""
    vals = {i: v if isinstance(v, bytes) else v.encode() for i, v in attrs.items()}
    n, attrs = len(vals), b"".join(struct.pack("<BH", i, len(v)) + v for i, v in vals.items())
    item = uuid + bytes(16) + struct.pack("<iHBBBBHBB", SET_UTC, 0, 1, 1, 0, 4, len(attrs), n, 0) + attrs
    val = struct.pack("<BHBB", 1, 0x1234, 4, 16) + uuid + struct.pack("<H", len(item)) + item
    return struct.pack(">HH", len(val), 0xB1DB) + val


# Pebble Protocol endpoints, PebbleOS include/pbl/pp/protocol.h:40-68
EP = {"clock": 0x000B, "version": 0x0011, "sysmsg": 0x0012, "appmsg": 0x0031, "runstate": 0x0034, "blobdb": 0xB1DB}
EP_NAME = {v: k for k, v in EP.items()}
CAP_RUN_STATE = 1 << 0                     # protocol.h:77
PLATFORM_ANDROID = 2                       # protocol.h:161, platform_bitfield & PBL_PP_REMOTE_OS_MASK
# The time the phone sets: 2026-11-19 17:37:00 UTC, offset -300 min (local 12:37, when the post went out). A distinct time from the
# watch's boot clock so the change is visible on the display.
SET_UTC, SET_OFFSET_MIN = 1_795_109_820, -300


def pp_msg(endpoint, payload):
    """Pebble Protocol frame: length (BE16), endpoint (BE16), payload."""
    return struct.pack(">HH", len(payload), endpoint) + payload


def version_response():
    """PebbleOS subsys/pp/version.c:16-38,90-98: 0x01, then version_response_v3 = v1 (library version u32, session
    capabilities u32, platform_bitfield u32 BE) + response_version, major, minor, bugfix + protocol_capabilities u32
    (read as the host's little endian). Only platform and capabilities are acted on (version.c:63-76); the app
    version numbers are only logged."""
    v1 = struct.pack(">III", 0, 0, PLATFORM_ANDROID)
    return pp_msg(EP["version"], b"\x01" + v1 + bytes([2, 4, 0, 0]) + struct.pack("<I", CAP_RUN_STATE))


def set_time(utc, offset_min):
    """PebbleOS pebble/services/clock/service.c:65-71,404-419: clock endpoint sub-command 0x03 = utc_time (BE32),
    utc_offset_min (BE16), region_name_len (u8), region_name. Length must be 1 + 7 + region_name_len; with no
    region name the watch applies the fixed offset (service.c:274-281)."""
    return pp_msg(EP["clock"], b"\x03" + struct.pack(">IhB", utc, offset_min, 0))


def run_state_status_request():
    """PebbleOS pebble/process_management/app_run_state.c:89-107, app_run_state.h:25: command 0x03 (STATUS, phone to
    watch) followed by a UUID (zero = none, read by the handler at app_run_state.c:107). The watch answers with
    the running app's UUID because the version response announced PBL_PP_CAP_RUN_STATE (app_run_state.c:30)."""
    return pp_msg(EP["runstate"], b"\x03" + bytes(16))


class Peer:
    # PebbleOS's NimBLE config (subsys/bluetooth/Kconfig.defconfig): LE Secure Connections only, MITM, bonding,
    # DisplayYesNo. A phone that offers less is refused with "Authentication Requirements".
    io_capabilities = "DisplayYesNo"
    sc = True
    mitm = True
    bonding = True

    def __init__(self, ctx):
        self.ctx = ctx
        self.seen = 0
        self.step = "idle"
        self.notify_h = self.write_h = 0
        self.sn = 0
        self.svc_end = 0
        self.rx = b""
        self.queue = []                                    # handshake messages still to send: (label, frame)

    def log(self, msg):                                    # prefixed with the virtual time, for scripts/demo_gif.py
        self.ctx.warning("t=%.3f %s" % (self.ctx.now_us / 1e6, msg))

    # --- scanning and security -------------------------------------------------
    def on_adv(self, addr, data):
        self.seen += 1
        if self.seen <= 3 or self.seen % 50 == 0:
            self.log("ADV_IND #%d from %s at %.3f s: %s" % (self.seen, addr, self.ctx.now_us / 1e6, describe(data)))
        return self.ctx.now_us >= CONNECT_AT_US and b"Pebble" in bytes(data)

    def on_connect(self):
        # A phone does LL feature/version exchange and MTU before it pairs; pairing in the very first
        # connection event races PebbleOS's host (see SIMULATION.md, Open).
        self.log("connected at %.3f s" % (self.ctx.now_us / 1e6))
        self.step = "wait_pair"
        self.ctx.schedule_oneshot(PAIR_DELAY_US)

    def on_numeric_compare(self, value):
        self.log("numeric comparison %06d: confirmed" % value)
        return True

    def on_paired(self, info):
        self.log("paired: sc=%s authenticated=%s bonded=%s key_size=%d" % (
            info["sc"], info["authenticated"], info["bonded"], info["key_size"]))

    def on_pairing_failed(self, reason):
        self.log("pairing failed: %s" % reason)

    def on_encrypted(self):
        self.log("link encrypted at %.3f s" % (self.ctx.now_us / 1e6))
        self.step = "find_svc"
        self.ctx.send(bytes([0x06]) + struct.pack("<HH", 1, 0xFFFF) + struct.pack("<H", 0x2800) + PPOG_SVC)

    # --- GATT client: discover the service, subscribe, then PPoGATT ------------
    def on_att(self, pdu):
        pdu = bytes(pdu)
        op = pdu[0]
        if op == 0x02:                                     # the watch's Exchange MTU Request: stay at 23
            self.ctx.send(bytes([0x03, 23, 0]))
        elif op == 0x08:                                   # the watch reads our Device Name (0x2A00)
            self.ctx.send(bytes([0x09, 2 + len(PHONE_NAME), 3, 0]) + PHONE_NAME)
        elif op == 0x01:
            self.log("ATT error rsp op=%02x handle=%04x code=%02x in step %s" % (pdu[1], *struct.unpack("<HB", pdu[2:5]), self.step))
        elif op == 0x1B:
            if struct.unpack("<H", pdu[1:3])[0] == self.notify_h:
                self.ppog_rx(pdu[3:])
        elif self.step == "find_svc" and op == 0x07:
            self.svc_end = struct.unpack("<H", pdu[3:5])[0]
            self.step = "find_chr"
            self.ctx.send(bytes([0x08]) + struct.pack("<HHH", struct.unpack("<H", pdu[1:3])[0], self.svc_end, 0x2803))
        elif self.step == "find_chr" and op == 0x09:
            n, last = pdu[1], 0
            for i in range(2, len(pdu), n):
                last, vh = struct.unpack("<H", pdu[i:i + 2])[0], struct.unpack("<H", pdu[i + 3:i + 5])[0]
                uuid = pdu[i + 5:i + n]
                if uuid == PPOG_NOTIFY:
                    self.notify_h = vh
                elif uuid == PPOG_WRITE:
                    self.write_h = vh
            if not (self.notify_h and self.write_h) and last < self.svc_end:   # MTU 23: one entry per response
                self.ctx.send(bytes([0x08]) + struct.pack("<HHH", last + 1, self.svc_end, 0x2803))
                return
            self.step = "subscribe"
            # MODELED SIMPLIFICATION: the CCCD is taken to sit right after the characteristic value
            # (true of NimBLE's generated table) instead of running Find Information.
            self.ctx.send(bytes([0x12]) + struct.pack("<HH", self.notify_h + 1, 1))
        elif self.step == "subscribe" and op == 0x13:
            self.log("subscribed to PPoGATT data (notify %04x, write %04x)" % (self.notify_h, self.write_h))
            self.step = "ppog_reset"
            self.ppog_tx(b"\x02" + b"\x01" + b"PHONESIM0001")   # Reset Request: version 1, serial

    # --- PPoGATT (PebbleOS ppogatt_internal.h): header = type (3 bits) | sn << 3 ------------
    def ppog_tx(self, pkt):
        self.ctx.send(bytes([0x52]) + struct.pack("<H", self.write_h) + pkt)

    def ppog_rx(self, pkt):
        t, sn = pkt[0] & 7, pkt[0] >> 3
        self.log("PPoGATT rx type=%d sn=%d %s" % (t, sn, pkt[1:].hex()))
        if t == 3 and self.step == "ppog_reset":          # Reset Complete: answer, session is open
            self.ppog_tx(b"\x03\x04\x04")                   # max rx / tx window 4
            self.step = "open"                              # now wait for the watch's version request
        elif t == 0:                                       # Data: ack it
            self.ppog_tx(bytes([1 | sn << 3]))
            self.rx += pkt[1:]
            while len(self.rx) >= 4 and len(self.rx) >= 4 + struct.unpack(">H", self.rx[:2])[0]:
                n, ep = struct.unpack(">HH", self.rx[:4])
                self.pp_rx(ep, self.rx[4:4 + n])
                self.rx = self.rx[4 + n:]

    def pp_rx(self, ep, payload):
        self.log("Pebble Protocol rx %s (0x%04x) %s" % (EP_NAME.get(ep, "endpoint"), ep, payload.hex()))
        if ep == EP["version"] and payload[:1] == b"\x00" and self.step == "open":   # version.c:41: request = 0x00
            self.pp_send("version response (Android, run-state capability)", version_response())
            self.queue = [("set time %s UTC, offset %+d min" % (time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(SET_UTC)), SET_OFFSET_MIN),
                           set_time(SET_UTC, SET_OFFSET_MIN)),
                          ("app run state status request", run_state_status_request())]
            self.step = "handshake"
            self.ctx.schedule_oneshot(300_000)

    def pp_send(self, label, msg):
        self.log("Pebble Protocol tx: %s (%d bytes)" % (label, len(msg)))
        for i in range(0, len(msg), 19):                   # default ATT MTU 23: 19 payload bytes per packet
            self.ppog_tx(bytes([self.sn << 3]) + msg[i:i + 19])
            self.sn = (self.sn + 1) % 32

    def on_timer(self):
        if self.step == "wait_pair":
            self.step = "pairing"
            self.ctx.pair()
        elif self.step == "handshake":
            if self.queue:
                self.pp_send(*self.queue.pop(0))
                self.ctx.schedule_oneshot(500_000 if self.queue else 1_500_000)
            else:
                self.pp_send("BlobDB insert: notification", blobdb_notification(NOTIFICATION))
                self.step = "sent"


if __name__ == "__main__":
    d = describe(bytes.fromhex("0201060c09") + b"Pebble 1A2B")
    assert d == 'flags=06 name="Pebble 1A2B"', d
    m = blobdb_notification("T", "B")
    assert struct.unpack(">HH", m[:4]) == (len(m) - 4, 0xB1DB), m.hex()
    assert pebble_uuid(0x40000000)[-4:] == bytes([0, 0, 0, 0x40])
    assert len(set_time(SET_UTC, SET_OFFSET_MIN)) == 4 + 1 + 7 and len(version_response()) == 4 + 1 + 12 + 4 + 4
    print("ok")
