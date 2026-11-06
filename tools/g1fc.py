#!/usr/bin/env python3
"""Faceclaw's display path (../g1_firmware/docs/VM.md §6.11) from a host: records, packets and draw calls as the phone
sends them, and its acceptance test, on an arm or on the emulator (emulator/tests/v5_tests.py, test `faceclaw`). The
test draws on the panel and checks every step by PANEL_READ; it leaves a session reset behind.

  python3 -I g1fc.py --ble ADDR [--hold S]     (--hold: keep the last picture up for S seconds)
"""
import argparse
import json
import random
import struct
import sys
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import g1host as g  # noqa: E402

W, H, STRIDE = 640, 480, 320
OPT_START, OPT_END = 0x80, 0x40
BOTH, COMPRESSED, RESET = 3, 4, 8
MSG_DRAW, MSG_PRESENT, MSG_BRIGHTNESS = 26, 28, 30
KIND_ACK, KIND_NACK = 1, 3
MAX_MESSAGE = 16384


def crc16(data):
    """CRC-16/CCITT-FALSE, the record's."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xFFFF
    return crc


class Frame:
    """The picture as it should be: 4 bpp rows, the left pixel in the high nibble."""

    def __init__(self):
        self.b = bytearray(STRIDE * H)

    def px(self, x, y):
        v = self.b[y * STRIDE + x // 2]
        return v & 15 if x & 1 else v >> 4

    def set(self, x, y, c):
        i = y * STRIDE + x // 2
        self.b[i] = (self.b[i] & 0xF0) | c if x & 1 else (self.b[i] & 0x0F) | (c << 4)

    def rect(self, x, y, w, h, fn):
        for yy in range(y, y + h):
            for xx in range(x, x + w):
                self.set(xx, yy, fn(xx, yy) & 15)

    def crc(self):
        return zlib.crc32(bytes(self.b))


def rle(pixels):
    """The RLE of a bbox: a run per token, [n|c] for 1..15, [c][n8], or [c][0][n16]."""
    out = bytearray()
    i, n = 0, len(pixels)
    while i < n:
        c = pixels[i]
        j = i + 1
        while j < n and pixels[j] == c and j - i < 65535:
            j += 1
        run = j - i
        if run <= 15:
            out.append(run << 4 | c)
        elif run <= 255:
            out += bytes([c, run])
        else:
            out += bytes([c, 0, run & 255, run >> 8])
        i = j
    return bytes(out)


def bbox(frame, x, y, w, h):
    """The draw call that puts frame's rectangle on the arm."""
    px = [frame.px(xx, yy) for yy in range(y, y + h) for xx in range(x, x + w)]
    if x % 4 == 0 and y % 2 == 0 and w % 4 == 0 and h % 2 == 0 and w / 4 < 256 and h / 2 < 256:
        head = bytes([0, x // 4, y // 2, w // 4, h // 2])
    else:
        head = bytes([1]) + struct.pack("<HHHH", x, y, w, h)
    return bytes([1, 0]) + head + rle(px)


def draw_message(calls):
    return bytes([MSG_DRAW]) + struct.pack("<H", len(calls)) + b"".join(struct.pack("<H", len(c)) + c for c in calls)


PRESENT = bytes([MSG_PRESENT])


def brightness(target, visible):
    return bytes([MSG_BRIGHTNESS, 1, target, 1 if visible else 0, 0, 0])


class FcLink:
    """One arm's display path: a persistent zlib stream, as the phone keeps per arm."""

    def __init__(self, arm):
        self.arm = arm
        self.stream = 1
        self.z = None
        arm.fc_acks = []

    def reset_context(self):
        self.z = None

    def record(self, message, compress=True):
        reset = self.z is None
        flags = BOTH
        if compress:
            if self.z is None:
                self.z = zlib.compressobj(6, zlib.DEFLATED, 15)
            body = self.z.compress(message) + self.z.flush(zlib.Z_SYNC_FLUSH)
            flags |= COMPRESSED
        else:
            body = message
        if reset:
            flags |= RESET
        return bytes([flags]) + struct.pack("<HH", len(body), crc16(message)) + body

    def write_record(self, rec, stream=None):
        """The packets of one record; returns the stream ID."""
        s = self.stream if stream is None else stream
        if stream is None:
            self.stream = self.stream % 250 + 1
        chunk = min(self.arm.tr.mtu - 3, 244) - 2
        for off in range(0, len(rec), chunk):
            part = rec[off:off + chunk]
            opts = (OPT_START if off == 0 else 0) | (OPT_END if off + len(part) == len(rec) else 0)
            self.arm.tr.write("G1_FC", bytes([opts, s]) + part)
            self.arm.pump()
            if isinstance(self.arm.tr, g.BleTransport):
                # bleak does not wait for CoreBluetooth's canSendWriteWithoutResponse: faster than the link, macOS
                # silently drops packets. The phone's writes wait for each completion.
                self.arm.wait(0.02)
        return s

    def wait(self, stream, timeout=20.0):
        """The answer to one stream: (kind, size, crc)."""
        found = []

        def seen():
            for a in self.arm.fc_acks:
                if a["stream"] == stream:
                    found.append(a)
                    return True
            return False
        self.arm.run_until(seen, timeout, f"FC answer to stream {stream}")
        self.arm.fc_acks.remove(found[0])
        return found[0]

    def send(self, message, compress=True, want=KIND_ACK, timeout=20.0):
        rec = self.record(message, compress)
        s = self.write_record(rec)
        a = self.wait(s, timeout)
        assert a["kind"] == want, (a, want, len(message))
        if want == KIND_ACK:
            assert a["size"] == len(message) and a["crc"] == crc16(message), (a, len(message))
        return {"wire": len(rec), "size": len(message)}


def panel_crc(arm):
    rep = arm.ctrl("PANEL_READ", struct.pack("<HH", 0, H), timeout=8.0, tries=1)
    assert rep["status"] == 0, rep
    return struct.unpack_from("<I", rep["body"])[0]


def bands(frame, x, y, w, h, band=16):
    """Rectangles of `band` rows, messages under the limit, as the phone cuts a big update."""
    msgs, calls, size = [], [], 3
    for top in range(y, y + h, band):
        c = bbox(frame, x, top, w, min(band, y + h - top))
        assert len(c) + 5 <= MAX_MESSAGE
        if size + len(c) + 2 > MAX_MESSAGE:
            msgs.append(draw_message(calls))
            calls, size = [], 3
        calls.append(c)
        size += len(c) + 2
    if calls:
        msgs.append(draw_message(calls))
    return msgs


def acceptance(arm, res, emulated=False, hold=None):
    """Every step ends with the panel's CRC against the expected frame."""
    rnd = random.Random(0xFC)
    arm.fc_acks = []
    rep = arm.ctrl("FC_INFO")
    assert rep["status"] == 0, rep
    version, flags, max_message, max_wire, rx_size, history = struct.unpack_from("<BBHHII", rep["body"])
    res["fc_info"] = {"version": version, "max_message": max_message, "max_wire": max_wire, "rx_size": rx_size,
                      "history": history}
    assert version == 1 and max_message == MAX_MESSAGE and history == 32768, res["fc_info"]

    link = FcLink(arm)
    f = Frame()
    steps = res["fc_steps"] = {}

    def checked(name, extra=None):
        got = panel_crc(arm)
        assert got == f.crc(), (name, f"{got:08x}", f"{f.crc():08x}")
        steps[name] = extra or True

    # A full frame: every grey level, in vertical bars and bands.
    f.rect(0, 0, W, H, lambda x, y: (x // 40 + y // 120))
    wire = size = 0
    for m in bands(f, 0, 0, W, H):
        r = link.send(m)
        wire += r["wire"]
        size += r["size"]
    link.send(PRESENT)
    checked("full_frame", {"decoded": size, "wire": wire})

    # A rectangle at odd coordinates and of odd size: byte-rounded on the panel, exact in the frame.
    f.rect(101, 51, 33, 17, lambda x, y: 15 - ((x + y) & 3))
    link.send(draw_message([bbox(f, 101, 51, 33, 17)]))
    link.send(PRESENT)
    checked("odd_rect")

    # Even width at an even x, one column, one row, the panel's corners.
    for x, y, w, h, c in ((10, 10, 2, 2, 9), (639, 0, 1, 480, 7), (0, 479, 640, 1, 5), (0, 0, 1, 1, 15), (638, 478, 2, 2, 3)):
        f.rect(x, y, w, h, lambda xx, yy, c=c: c)
        link.send(draw_message([bbox(f, x, y, w, h)]))
    link.send(PRESENT)
    checked("edges")

    # Several rectangles in a message, then more than the dirty list holds.
    calls = []
    for i in range(6):
        x, y = 20 + i * 90, 200 + i * 7
        f.rect(x, y, 40, 10, lambda xx, yy, i=i: i + 3)
        calls.append(bbox(f, x, y, 40, 10))
    link.send(draw_message(calls))
    link.send(PRESENT)
    checked("multi_rect")
    calls = []
    for i in range(40):
        x, y = 8 * i, 300 + (i % 5) * 10
        f.rect(x, y, 6, 6, lambda xx, yy, i=i: (i % 15) + 1)
        calls.append(bbox(f, x, y, 6, 6))
    link.send(draw_message(calls))
    link.send(PRESENT)
    checked("dirty_overflow")

    # The stream's history: the same content again costs next to nothing on the wire.
    f.rect(0, 400, 320, 20, lambda x, y: rnd.randrange(16))
    m = draw_message([bbox(f, 0, 400, 320, 20)])
    first = link.send(m)
    second = link.send(m)
    assert second["wire"] < first["wire"] / 4, (first, second)
    link.send(PRESENT)
    checked("history", {"first": first["wire"], "repeat": second["wire"]})

    # Records in flight together, and a raw record among them: order holds.
    streams = []
    for i in range(5):
        f.rect(400, 20 + i * 12, 60, 10, lambda x, y, i=i: 15 - i)
        streams.append(link.write_record(link.record(draw_message([bbox(f, 400, 20 + i * 12, 60, 10)]))))
    f.rect(480, 20, 60, 10, lambda x, y: 8)
    streams.append(link.write_record(link.record(draw_message([bbox(f, 480, 20, 60, 10)]), compress=False)))
    streams.append(link.write_record(link.record(PRESENT)))
    for s in streams:
        assert link.wait(s)["kind"] == KIND_ACK
    checked("pipelined")

    # Clear, and a repeat of the same record (a replay after a lost ACK).
    link.send(draw_message([bytes([9, 0, 2])]))
    f.b[:] = bytes([0x22]) * len(f.b)
    link.send(PRESENT)
    checked("clear")

    # Hidden: the panel waits in standby, draws go to the frame; shown: it comes back with PRESENT.
    link.send(brightness(50, False))
    assert arm.props("PANEL")["PANEL"] == 2, arm.props("PANEL")
    f.rect(100, 100, 200, 100, lambda x, y: 11)
    link.send(draw_message([bbox(f, 100, 100, 200, 100)]))
    link.send(PRESENT)
    assert arm.props("PANEL")["PANEL"] == 2
    link.send(brightness(50, True))
    link.send(PRESENT)
    assert arm.props("PANEL")["PANEL"] == 1
    checked("hide_show")
    assert arm.props("BRIGHTNESS")["BRIGHTNESS"] == 21, arm.props("BRIGHTNESS")

    # PROP_SET: what a host without instructions has for properties.
    rep = arm.ctrl("PROP_SET", struct.pack("<BI", g.PROPS["BRIGHTNESS_MAX"], 30))
    assert rep["status"] == 0, rep
    arm.wait(0.1)
    assert arm.props("BRIGHTNESS_MAX")["BRIGHTNESS_MAX"] == 30, arm.props("BRIGHTNESS_MAX")
    arm.ctrl("PROP_SET", struct.pack("<BIBI", g.PROPS["BRIGHTNESS_MAX"], 42, g.PROPS["IMU_RATE"], 26))
    arm.wait(0.1)
    p = arm.props("BRIGHTNESS_MAX", "IMU_RATE")
    assert p["BRIGHTNESS_MAX"] == 42 and p["IMU_RATE"] == 26, p
    arm.ctrl("PROP_SET", struct.pack("<BI", g.PROPS["IMU_RATE"], 13))
    assert arm.ctrl("PROP_SET", b"abc")["status"] == 1
    steps["prop_set"] = True

    # The microphone (right arm, on an encrypted link or the emulator's test build): its indicator over the frame while on,
    # drawn again over a present, and the frame back in its corner when it stops; the lease lasts as long as packets come.
    if res.get("mic", True):
        frames = []
        arm.on_mic = lambda first, n, data, t_dev, t_host: frames.append((first, n, len(data)))
        rep = arm.mic(True)
        assert rep["status"] == 0, rep
        arm.wait(0.5)
        assert frames and all(n == 5 and size == 200 for _, n, size in frames), frames[:3]
        withblock = Frame()
        withblock.b[:] = f.b

        def block(x0=620 if arm.hello()["role"] == "R" else 4):
            withblock.b[:] = f.b
            withblock.rect(x0, 4, 16, 16, lambda x, y: 15)
            return withblock.crc()
        assert panel_crc(arm) == block(), "the indicator is on the panel"
        f.rect(600, 0, 40, 30, lambda x, y: 4)   # a draw right over it
        link.send(draw_message([bbox(f, 600, 0, 40, 30)]))
        link.send(PRESENT)
        assert panel_crc(arm) == block(), "and stays through a present over it"
        rep = arm.mic(False)
        assert rep["status"] == 0, rep
        arm.wait(0.5)
        arm.on_mic = None
        res["mic_events"] = len(frames)
        assert panel_crc(arm) == f.crc(), "the frame is back under the indicator"
        steps["microphone"] = True

    # Malformed messages: refused, the picture not presented, the stream alive.
    good = draw_message([bbox(f, 0, 0, 4, 2)])
    link.send(draw_message([bytes([1, 0, 1]) + struct.pack("<HHHH", 636, 0, 8, 2) + bytes([0x8F, 0x8F])]), want=KIND_NACK)
    link.send(draw_message([bytes([1, 0, 1]) + struct.pack("<HHHH", 0, 0, 4, 2) + bytes([0x1F])]), want=KIND_NACK)   # short
    link.send(draw_message([bytes([1, 0, 1]) + struct.pack("<HHHH", 0, 0, 4, 2) + bytes([0xFF])]), want=KIND_NACK)   # long
    link.send(draw_message([bytes([1, 0, 1]) + struct.pack("<HHHH", 0, 0, 0, 2)]), want=KIND_NACK)   # empty
    link.send(draw_message([bytes([1, 1, 0, 0, 0, 0, 0, 0])]), want=KIND_NACK)   # a target: not on this path
    link.send(draw_message([bytes([7, 0, 0, 0])]), want=KIND_NACK)   # a call of the display lists
    link.send(bytes([MSG_DRAW, 2, 0, 200, 0, 1, 0]), want=KIND_NACK)   # calls cut off
    link.send(bytes([MSG_BRIGHTNESS, 1, 101, 1, 0, 0]), want=KIND_NACK)
    link.send(bytes([127, 1, 2, 3]))   # unknown to this firmware: taken and ignored
    link.send(good)
    link.send(PRESENT)
    checked("malformed_messages")

    # Damaged records: NACK, and nothing works until a record resets the stream, as the phone's recovery does.
    rec = bytearray(link.record(good))
    rec[3] ^= 1   # the CRC
    s = link.write_record(bytes(rec))
    assert link.wait(s)["kind"] == KIND_NACK
    rec = link.record(good)
    s = link.write_record(rec)
    assert link.wait(s)["kind"] == KIND_NACK, "the stream should be lost"
    link.reset_context()
    link.send(good)
    for bad in (b"\x78\x9c\xff\xff\xff\xff", bytes(rnd.randrange(256) for _ in range(300)), b"\x00"):
        link.reset_context()
        body = bad
        rec = bytes([BOTH | COMPRESSED | RESET]) + struct.pack("<HH", len(body), 0x1234) + body
        s = link.write_record(rec)
        assert link.wait(s)["kind"] == KIND_NACK
    # A header that lies about the length; a first packet too short; a record too big; a continuation out of nowhere; a
    # record whose last packet never came, which the next one's start ends.
    s = link.write_record(bytes([BOTH | RESET]) + struct.pack("<HH", 50, 0) + b"xx")
    assert link.wait(s)["kind"] == KIND_NACK
    arm.tr.write("G1_FC", bytes([OPT_START, 204, BOTH | RESET]) + struct.pack("<HH", 50, 0) + b"never ends")
    arm.tr.write("G1_FC", bytes([OPT_START | OPT_END, 205, BOTH | RESET]) + struct.pack("<HH", len(PRESENT), crc16(PRESENT)) + PRESENT)
    assert link.wait(204)["kind"] == KIND_NACK
    assert link.wait(205)["kind"] == KIND_ACK
    arm.tr.write("G1_FC", bytes([OPT_START | OPT_END, 201, BOTH]))
    assert link.wait(201)["kind"] == KIND_NACK
    arm.tr.write("G1_FC", bytes([OPT_START, 202, BOTH]) + struct.pack("<HH", 60000, 0) + b"z" * 20)
    assert link.wait(202)["kind"] == KIND_NACK
    arm.tr.write("G1_FC", bytes([OPT_END, 203]) + b"nobody started this")
    # A decoded message over the limit
    link.reset_context()
    link.send(bytes([MSG_DRAW]) + bytes(MAX_MESSAGE + 1), want=KIND_NACK)
    link.reset_context()
    link.send(good)
    link.send(PRESENT)
    checked("damaged_records")

    # More than the ring holds, sent without waiting: some records refused, then the phone's recovery works.
    noise = [draw_message([bbox_noise(f, rnd, 0, 16 * i, 640, 16)]) for i in range(6)]
    sent = [link.write_record(link.record(m)) for m in noise]
    answers = [link.wait(s, 60.0)["kind"] for s in sent]
    res["ring_overflow_answers"] = answers
    link.reset_context()
    for i in range(6):
        link.send(draw_message([bbox(f, 0, 16 * i, 640, 16)]))
    link.send(PRESENT)
    checked("ring_recovery")

    rep = arm.ctrl("FC_INFO")
    records, nacks, presents = struct.unpack_from("<III", rep["body"], 14)
    res["fc_counters"] = {"records": records, "nacks": nacks, "presents": presents}
    assert arm.ctrl("HELLO")["epoch"] == arm.epoch, "the arm rebooted"
    if hold:
        arm.wait(hold)
    # The arena goes back to the VM, and the path opens again afterwards.
    arm.reset_session(clear_arena=True)
    arm.wait_idle(10)
    rep = arm.ctrl("FC_INFO")
    assert rep["status"] == 0 and rep["body"][1] == 0, rep
    link = FcLink(arm)
    f = Frame()
    f.rect(0, 0, W, 8, lambda x, y: 6)
    link.send(draw_message([bbox(f, 0, 0, W, 8)]))
    link.send(PRESENT)
    checked("after_reset_session")
    arm.reset_session(clear_arena=True)
    arm.set_panel(0)
    arm.wait_idle(10)


def bbox_noise(frame, rnd, x, y, w, h):
    frame.rect(x, y, w, h, lambda xx, yy: rnd.randrange(16))
    return bbox(frame, x, y, w, h)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ble", required=True)
    ap.add_argument("--hold", type=float)
    args = ap.parse_args()
    arm = g.Arm(g.BleTransport(args.ble), name="arm")
    arm.setup()
    hello = arm.hello()
    res = {"hello": {k: v for k, v in hello.items() if k in ("version", "role", "in_case", "encrypted", "safe_level")},
           "mic": hello["mic"] and hello["encrypted"]}
    acceptance(arm, res, hold=args.hold)
    res["pass"] = True
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
