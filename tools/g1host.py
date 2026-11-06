#!/usr/bin/env python3
"""g1host: the reference host client of the scouter firmware g1v0 0.5.x, protocol 5
(../g1_firmware/docs/VM.md). With arms on an older protocol (g1v0 0.4.x, 0.3.x) or on Even's firmware only the
smp-* commands work.

Two transports with the same interface:
  --emu PATH   the Renode virtual phone bridge (../g1_firmware/emulator/models/G1_VirtualPhone.cs). Time is the
               emulator's virtual time: every number measured this way is emulated.
  --ble ADDR   BlueZ through bleak, for the nRF5340 DK or an arm running g1v0. After a lost link it
               reconnects by itself and resyncs on the protocol's epoch (VM.md §6.2).

The host sends host code (Code: VM instructions) in sequenced packets and keeps a mirror of the arm: the
VM core itself (../src/vm, built with g1mirror.c into a shared library) runs every packet
as the arm does, so the arena and the panel can be compared byte for byte (ARENA_CRC, PANEL_READ, the
emulator's panel dump). The drawing helpers take word addresses (aw = byte address / 4) as in protocol 4.
"""
import argparse
import asyncio
import hashlib
import json
import os
import queue
import socket
import struct
import subprocess
import sys
import threading
import time
import zlib
from collections import OrderedDict
from pathlib import Path

UUIDS = {
    "G1_CMD": "f3da4d59-277a-4fc9-a1ed-41c9c22d4401",
    "G1_CTRL": "f3da4d59-277a-4fc9-a1ed-41c9c22d4402",
    "G1_EVT": "f3da4d59-277a-4fc9-a1ed-41c9c22d4403",
    "G1_PAIR": "f3da4d59-277a-4fc9-a1ed-41c9c22d4404",
    "G1_FC": "f3da4d59-277a-4fc9-a1ed-41c9c22d4405",
    "SMP": "da2e7828-fbce-4e01-ae9e-261174997c48",
    "NUS_RX": "6e400002-b5a3-f393-e0a9-e50e24dcca9e",
    "NUS_TX": "6e400003-b5a3-f393-e0a9-e50e24dcca9e",
}
PANEL_W, PANEL_H = 640, 480
M32 = 0xFFFFFFFF
ERRORS = {1: "malformed", 2: "invalid op", 3: "arena", 4: "format", 5: "argument", 6: "window",
          7: "safe mode", 8: "font", 9: "stack", 10: "budget", 11: "panel", 12: "mic", 13: "asset"}


def seq_before(a, b):
    return ((a - b) & 0xFFFF) >= 0x8000


def signed32(v):
    v &= M32
    return v - (1 << 32) if v >= 1 << 31 else v


class EmuTransport:
    """Lock-step client of the virtual phone bridge. One step = one connection event of the
    emulated link (or 10 ms of virtual time while no link is up)."""

    def __init__(self, path, wait_s=180.0):
        deadline = time.monotonic() + wait_s
        while True:
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            try:
                s.connect(path)
                break
            except OSError:
                s.close()
                if time.monotonic() > deadline:
                    raise
                time.sleep(0.2)
        self.sock = s
        self.rf = s.makefile("rb")
        self.now_us = 0
        self.connected = False
        self.mtu = 23
        self.steps_done = self.steps_total = 0
        self.handles = {}
        self.names = {}
        self.pending = []
        self.inbox = self._read_batch()

    def now(self):
        return self.now_us / 1e6

    def _read_batch(self):
        events = []
        while True:
            raw = self.rf.readline()
            if not raw:
                raise ConnectionError("emulator bridge closed")
            parts = raw.decode("ascii").rstrip("\n").split(" ")
            kind = parts[0]
            if kind == "T":
                self.now_us = int(parts[1])
                self.connected = parts[2] == "1"
                self.mtu = int(parts[3])
                self.steps_done, self.steps_total = int(parts[4]), int(parts[5])
            elif kind == "H":
                self.handles[parts[1]] = int(parts[2], 16)
                self.names[int(parts[2], 16)] = parts[1]
            elif kind == "N":
                name = self.names.get(int(parts[1], 16), parts[1])
                events.append(("notify", name, bytes.fromhex(parts[3]), int(parts[2]) / 1e6))
            elif kind == "L":
                text = " ".join(parts[1:])
                if text.startswith("lost"):
                    self.handles.clear()
                    self.names.clear()
                events.append(("link", text, None, self.now()))
            elif kind == "E":
                return events

    def write(self, name, data):
        self.pending.append(f"W {self.handles[name]:X} {data.hex()}")

    def has(self, name):
        return name in self.handles

    def script(self, line):
        self.pending.append(f"X {line}")

    def set_model(self, path, prop, value):
        self.pending.append(f"F {path} {prop} {value}")

    def call_model(self, path, method, arg=None):
        self.pending.append(f"C {path} {method}" + (f" {arg}" if arg is not None else ""))

    def go_silent(self, ms):
        self.pending.append(f"D {ms}")

    def reset_machine(self):
        self.pending.append("Z")

    def save_memory(self, address, length, path):
        self.pending.append(f"M {address:X} {length:X} {path}")

    def step(self):
        events, self.inbox = self.inbox, []
        if events:
            return events
        lines = self.pending + ["G"]
        self.pending = []
        self.sock.sendall(("\n".join(lines) + "\n").encode("ascii"))
        return self._read_batch()

    def scripts_idle(self):
        return not self.pending and self.steps_done >= self.steps_total

    def close(self):
        self.sock.close()


class BleTransport:
    """BlueZ via bleak in a background event loop. Writes are Write Without Response, in order.
    A lost link is reconnected in the background (the arm advertises again after a disconnect or a
    reboot) and the notifications are subscribed again; the Arm then resyncs on the epoch. Writes
    while the link is down are dropped: the protocol resends them after the resync."""

    def __init__(self, address, wait_s=20.0, reconnect=True):
        self.address = address
        self.wait_s = wait_s
        self.reconnect = reconnect
        self.closing = False
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.loop.run_forever, daemon=True).start()
        self.events = queue.Queue()
        self.client = None
        self.connected = False
        self.reconnecting = False
        self.reconnects = 0
        self.chars = {}
        self.mtu = 23
        self.steps_done = self.steps_total = 0
        self.run(self._connect(), timeout=wait_s + 30)

    def run(self, coro, timeout=60):
        return asyncio.run_coroutine_threadsafe(coro, self.loop).result(timeout)

    async def _connect(self):
        from bleak import BleakClient
        client = BleakClient(self.address, timeout=self.wait_s, disconnected_callback=self._lost)
        self.client = client
        await client.connect()
        chars = {}
        for name, uuid in UUIDS.items():
            ch = client.services.get_characteristic(uuid)
            if ch is not None:
                chars[name] = ch
        for name in ("G1_EVT", "SMP", "NUS_TX"):
            if name in chars:
                await client.start_notify(chars[name], self._notify_cb(name))
        backend = getattr(client, "_backend", None)
        if hasattr(backend, "_acquire_mtu"):
            try:
                await backend._acquire_mtu()
            except Exception:
                pass
        mtu = client.mtu_size
        ch = chars.get("G1_CMD")
        for _ in range(20):
            if mtu > 23 or ch is None:
                break
            if ch.max_write_without_response_size > 20:
                mtu = ch.max_write_without_response_size + 3
                break
            await asyncio.sleep(0.1)
        self.chars, self.mtu, self.connected = chars, mtu, True
        self.events.put(("link", "connected", None, self.now()))

    def _notify_cb(self, name):
        return lambda _c, data: self.events.put(("notify", name, bytes(data), self.now()))

    def _lost(self, client):
        if client is not self.client:
            return
        self.connected = False
        self.chars = {}
        self.events.put(("link", "lost", None, self.now()))
        if self.reconnect and not self.closing and not self.reconnecting:
            self.reconnecting = True
            asyncio.ensure_future(self._reconnect(), loop=self.loop)

    async def _reconnect(self):
        delay = 0.5
        try:
            while not self.closing and not self.connected:
                await asyncio.sleep(delay)
                try:
                    await self._connect()
                    self.reconnects += 1
                except Exception as ex:
                    self.events.put(("link", f"reconnect-failed {type(ex).__name__}", None, self.now()))
                    try:
                        await self.client.disconnect()
                    except Exception:
                        pass
                    delay = min(delay * 2, 4.0)
        finally:
            self.reconnecting = False

    def now(self):
        return time.monotonic()

    def has(self, name):
        return name in self.chars

    def write(self, name, data):
        ch = self.chars.get(name)
        if not self.connected or ch is None:
            return False
        try:
            self.run(self.client.write_gatt_char(ch, data, response=False), timeout=10)
            return True
        except Exception:
            return False

    def script(self, line):
        pass

    def step(self, wait_s=0.003):
        out = []
        try:
            out.append(self.events.get(timeout=wait_s))
            while True:
                out.append(self.events.get_nowait())
        except queue.Empty:
            pass
        return out

    def scripts_idle(self):
        return True

    def close(self):
        self.closing = True
        try:
            self.run(self.client.disconnect(), timeout=10)
        except Exception:
            pass


# ---------------------------------------------------------------- CBOR subset + MCUmgr SMP

def cbor_enc(v):
    def head(major, n):
        if n < 24:
            return bytes([major << 5 | n])
        for ai, size in ((24, 1), (25, 2), (26, 4), (27, 8)):
            if n < 1 << (8 * size):
                return bytes([major << 5 | ai]) + n.to_bytes(size, "big")
        raise ValueError("integer too large")
    if isinstance(v, bool):
        return b"\xf5" if v else b"\xf4"
    if isinstance(v, int):
        return head(0, v) if v >= 0 else head(1, -1 - v)
    if isinstance(v, (bytes, bytearray)):
        return head(2, len(v)) + bytes(v)
    if isinstance(v, str):
        b = v.encode()
        return head(3, len(b)) + b
    if isinstance(v, (list, tuple)):
        return head(4, len(v)) + b"".join(cbor_enc(x) for x in v)
    if isinstance(v, dict):
        return head(5, len(v)) + b"".join(cbor_enc(k) + cbor_enc(x) for k, x in v.items())
    raise TypeError(type(v))


def cbor_dec(b, i=0):
    ib = b[i]
    major, ai = ib >> 5, ib & 31
    i += 1
    if ai < 24:
        n = ai
    elif ai in (24, 25, 26, 27):
        size = 1 << (ai - 24)
        n = int.from_bytes(b[i:i + size], "big")
        i += size
    elif ai == 31:
        n = None
    else:
        raise ValueError("bad CBOR header")
    if major == 0:
        return n, i
    if major == 1:
        return -1 - n, i
    if major in (2, 3):
        v = b[i:i + n]
        return (bytes(v) if major == 2 else v.decode()), i + n
    if major in (4, 5):
        out = [] if major == 4 else {}
        count = 0
        while (n is None and b[i] != 0xFF) or (n is not None and count < n):
            k, i = cbor_dec(b, i)
            if major == 4:
                out.append(k)
            else:
                out[k], i = cbor_dec(b, i)
            count += 1
        return out, (i + 1 if n is None else i)
    if major == 7:
        return {20: False, 21: True, 22: None}.get(ai), i
    raise ValueError(f"unsupported CBOR major {major}")


def image_hash(img):
    hdr_size = struct.unpack_from("<H", img, 8)[0]
    img_size = struct.unpack_from("<I", img, 12)[0]
    off = hdr_size + img_size
    _magic, total = struct.unpack_from("<HH", img, off)
    end, off = off + total, off + 4
    while off < end:
        t, n = struct.unpack_from("<HH", img, off)
        if t == 0x10:
            return bytes(img[off + 4:off + 4 + n])
        off += 4 + n
    raise ValueError("image has no SHA-256 TLV")


class Smp:
    """MCUmgr SMP over the SMP characteristic (image and OS groups), on any transport."""

    def __init__(self, link):
        self.link = link
        self.seq = 0
        self.buf = b""
        self.responses = {}
        link.listeners["SMP"] = self._on_data

    def _on_data(self, data, t):
        self.buf += data
        while len(self.buf) >= 8:
            n = struct.unpack_from(">H", self.buf, 2)[0]
            if len(self.buf) < 8 + n:
                return
            frame, self.buf = self.buf[:8 + n], self.buf[8 + n:]
            try:
                body = cbor_dec(frame, 8)[0] if n else {}
            except Exception:
                self.buf = b""   # a frame cut by a lost link: drop it, the request is sent again
                return
            self.responses[frame[6]] = body

    def request(self, op, group, cmd, payload, timeout=20.0):
        seq = self.seq & 0xFF
        self.seq += 1
        body = cbor_enc(payload)
        frame = struct.pack(">BBHHBB", op, 0, len(body), group, seq, cmd) + body
        size = self.link.tr.mtu - 3
        losses = self.link.losses()
        for off in range(0, len(frame), size):
            self.link.tr.write("SMP", frame[off:off + size])
        # a lost link loses the request: give up at once instead of waiting for the timeout
        self.link.run_until(lambda: seq in self.responses or self.link.losses() > losses, timeout,
                            f"SMP {group}/{cmd}")
        if seq not in self.responses:
            raise TimeoutError(f"SMP {group}/{cmd}: link lost")
        return self.responses.pop(seq)

    def image_list(self):
        return self.request(0, 1, 0, {}).get("images", [])

    def upload(self, img, chunk=2048, progress=None, retries=40):
        """Upload in chunks; after a timeout or a lost link, set the link up again and resend at the
        host's offset (MCUmgr answers with the offset it expects, so the upload resumes)."""
        off = 0
        t0 = self.link.tr.now()
        failures = 0
        while off < len(img):
            req = {"off": off, "data": img[off:off + chunk]}
            if off == 0:
                req = {"image": 0, "len": len(img), "off": 0, "sha": hashlib.sha256(img).digest(),
                       "data": img[:chunk], "upgrade": False}
            losses = self.link.losses()
            try:
                rsp = self.request(2, 1, 1, req, timeout=30.0)
            except (TimeoutError, KeyError):
                failures += 1
                if failures > retries:
                    raise
                if self.link.losses() > losses or not self.link.tr.has("SMP"):
                    self.link.run_until(lambda: self.link.tr.connected, 60, "reconnect")
                    self.link.setup()
                self.buf = b""
                continue
            if rsp.get("rc", 0) != 0 or "off" not in rsp:
                raise RuntimeError(f"SMP upload refused at {off}: {rsp}")
            off = rsp["off"]
            if progress:
                progress(off, len(img))
        self.resumed = failures
        return self.link.tr.now() - t0

    def confirm(self, img):
        return self.request(2, 1, 0, {"hash": image_hash(img), "confirm": True})

    def reset(self):
        return self.request(2, 0, 5, {}, timeout=5.0)


def smp_images(smp):
    """Image list as short dicts (image, slot, version, hash, flags)."""
    out = []
    for e in smp.image_list():
        flags = [k for k in ("bootable", "pending", "confirmed", "active", "permanent") if e.get(k)]
        out.append({"image": e.get("image", 0), "slot": e.get("slot"), "version": e.get("version"),
                    "hash": e["hash"].hex() if isinstance(e.get("hash"), bytes) else e.get("hash"),
                    "flags": flags})
    return out


def smp_slot1(smp, img):
    """The image-0 secondary-slot entry if it holds exactly `img` (MCUboot SHA-256 TLV), else None."""
    want = image_hash(img).hex()
    for e in smp_images(smp):
        if e["image"] == 0 and e["slot"] == 1 and e["hash"] == want:
            return e
    return None


def smp_retry(arm, fn, tries=10):
    """fn(), again after a lost link (the link is set up again first)."""
    for k in range(tries):
        try:
            return fn()
        except (TimeoutError, KeyError):
            if k == tries - 1:
                raise
            if not arm.tr.has("SMP"):
                arm.run_until(lambda: arm.tr.connected, 60, "reconnect")
                arm.setup()


def smp_upload_verified(arm, img, progress=None):
    """Plain SMP upload of an app image to image 0's secondary slot, then an image list that must show
    that slot holding exactly this image. Nothing is marked for install."""
    before = smp_retry(arm, lambda: smp_images(arm.smp))
    secs = arm.smp.upload(img, progress=progress)
    after = smp_retry(arm, lambda: smp_images(arm.smp))
    entry = smp_retry(arm, lambda: smp_slot1(arm.smp, img))
    if entry is None:
        raise RuntimeError(f"slot 1 does not hold the uploaded image: {after}")
    return {"images_before": before, "upload_s": round(secs, 2), "kBps": round(len(img) / secs / 1000, 2),
            "resumed": getattr(arm.smp, "resumed", 0), "images_after": after, "slot1": entry}


def smp_commit(arm, img, reset=True):
    """Check slot 1 again, mark it for a permanent install (image state 'confirm'), then reset."""
    if smp_slot1(arm.smp, img) is None:
        raise RuntimeError("slot 1 no longer holds the image: nothing confirmed")
    rsp = arm.smp.confirm(img)
    out = {"confirm_rc": rsp.get("rc", 0), "images": [
        {"slot": e.get("slot"), "version": e.get("version"),
         "flags": [k for k in ("pending", "confirmed", "active", "permanent") if e.get(k)]}
        for e in rsp.get("images", [])]}
    if out["confirm_rc"]:
        raise RuntimeError(f"confirm refused: {rsp}")
    if reset:
        out["reset_t"] = arm.tr.now()
        try:
            arm.smp.reset()
            out["reset_reply"] = True
        except (TimeoutError, KeyError):
            out["reset_reply"] = False   # the arm may go down before its reply leaves
    return out


# ---------------------------------------------------------------- mirror of the arm

def pack_rows(levels, w, h, fmt):
    """levels: w*h values 0..(2^fmt-1), row-major -> packed bitmap (fmt 1 or 4), stride bytes per row."""
    stride = (w * fmt + 7) // 8
    out = bytearray(stride * h)
    for y in range(h):
        for x in range(w):
            v = levels[y * w + x]
            bit = x * fmt
            out[y * stride + bit // 8] |= v << (8 - fmt - bit % 8)
    return bytes(out), stride


def version_tuple(v):
    return tuple(int(x) for x in str(v).split(".")[:3])


def draw_exact(version):
    """g1v0 0.3.10+ draws exactly the DRAW's pixels, at any x and width (None: assume so)."""
    return version is None or version_tuple(version) >= (0, 3, 10)


def varint(v):
    out = bytearray()
    while True:
        b, v = v & 0x7F, v >> 7
        if not v:
            return bytes(out + bytes([b]))
        out.append(b | 0x80)


def next_varint(data, i):
    """(value, next i) of the LEB128 value at data[i:], at most 3 bytes; (None, i) if cut off or longer."""
    val = 0
    for k in range(3):
        if i >= len(data):
            return None, i
        b = data[i]
        i += 1
        val |= (b & 0x7F) << (7 * k)
        if not b & 0x80:
            return val, i
    return None, i


def unpackbits(data, out_len):
    """PackBits as the arm decodes it: None unless the runs use exactly `data` and give out_len bytes."""
    out = bytearray()
    i, n = 0, len(data)
    while i < n:
        c = data[i]
        i += 1
        if c < 128:
            if i + c + 1 > n:
                return None
            out += data[i:i + c + 1]
            i += c + 1
        elif c != 128:
            if i >= n:
                return None
            out += bytes([data[i]]) * (257 - c)
            i += 1
        if len(out) > out_len:
            return None
    return out if len(out) == out_len else None


def packbits(data):
    """PackBits (Apple) RLE: runs of >=2 equal bytes -> (257-count, byte); literals -> (count-1, data)."""
    out = bytearray()
    i, n = 0, len(data)
    while i < n:
        run = 1
        while i + run < n and run < 128 and data[i + run] == data[i]:
            run += 1
        if run >= 2:
            out += bytes([257 - run, data[i]])
            i += run
        else:
            j = i
            lit = bytearray()
            while j < n and len(lit) < 128:
                r = 1
                while j + r < n and r < 3 and data[j + r] == data[j]:
                    r += 1
                if r >= 2:
                    break
                lit.append(data[j])
                j += 1
            out += bytes([len(lit) - 1]) + lit
            i = j
    return bytes(out)


# ---------------------------------------------------------------- font blob (PROTOCOL.md §5)

FIRST_GLYPHS = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"


def glyph_order(glyphs):
    """VM.md §4.1: 0-9, A-Z, a-z first (an empty glyph, codepoint 0, for a missing one; the list ends after
    the last one present), then the others by codepoint."""
    by_cp = {g[0]: g for g in glyphs}
    first = [ord(c) for c in FIRST_GLYPHS]
    last = max((i for i, cp in enumerate(first) if cp in by_cp), default=-1)
    head = [by_cp.get(cp, (0, 0, 0, 0, 0, 0, [])) for cp in first[:last + 1]]
    return head + sorted(g for g in glyphs if g[0] not in first)


def make_font_blob(glyphs, ascent, descent, line_spacing, kerning=None):
    """glyphs: list of (codepoint, w, h, x_off, y_off, advance, bits) with bits = w*h values 0/1,
    row-major; kerning: {(left_cp, right_cp): adjust}. Returns the blob (font format 1, PROTOCOL.md §5,
    in the glyph order of VM.md §4.1)."""
    glyphs = glyph_order(glyphs)
    index = {g[0]: i for i, g in enumerate(glyphs) if g[0]}
    pairs = sorted((index[l] << 16 | index[r], adj) for (l, r), adj in (kerning or {}).items()
                   if adj and l in index and r in index)
    n, k = len(glyphs), len(pairs)
    bm = 16 + 16 * n + 5 * k
    table, cps, stream = bytearray(), bytearray(), []
    for cp, w, h, xo, yo, adv, bits in glyphs:
        if not (0 <= w <= 255 and 0 <= h <= 255 and 0 <= adv <= 255 and -32768 <= xo < 32768 and -32768 <= yo < 32768):
            raise ValueError(f"glyph U+{cp:04X} out of the blob's ranges")
        off = bm * 8 + len(stream) if w and h else 0
        stream += list(bits) if w and h else []
        table += struct.pack("<IBBBBhh", off, w, h, adv, 0, xo, yo)
        cps += struct.pack("<I", cp)
    packed = bytearray((len(stream) + 7) // 8)
    for i, b in enumerate(stream):
        if b:
            packed[i >> 3] |= 1 << (7 - (i & 7))
    kern = b"".join(struct.pack("<I", key) for key, _ in pairs) + bytes(adj & 0xFF for _, adj in pairs)
    size = bm + len(packed)
    header = b"SKF\x01" + struct.pack("<IHHBBBB", size, n, k, min(ascent, 255), min(descent, 255),
                                      min(line_spacing, 255), 0)
    return header + table + cps + kern + packed


def build_font_blob(path, size, chars=None, kerning=True):
    """Render a TrueType font with Pillow into a font blob: Pillow's 8-bit mask thresholded at 128,
    offsets from the left end of the baseline, advances rounded, kerning = the pair's advance minus
    the two single advances, rounded (pairs of `chars`, default printable ASCII)."""
    from PIL import ImageFont
    font = ImageFont.truetype(path, size)
    chars = sorted(set(chars or "".join(chr(c) for c in range(32, 127))))
    ascent, descent = font.getmetrics()
    glyphs = []
    for ch in chars:
        mask, (ox, oy) = font.getmask2(ch, mode="L", anchor="ls")
        gw, gh = mask.size
        bits = [1 if v >= 128 else 0 for v in mask] if gw and gh else []
        if not gw or not gh:
            gw = gh = ox = oy = 0
        glyphs.append((ord(ch), gw, gh, ox, oy, round(font.getlength(ch)), bits))
    kern = {}
    if kerning:
        single = {ch: font.getlength(ch) for ch in chars}
        for l in chars:
            for r in chars:
                adj = round(font.getlength(l + r) - single[l] - single[r])
                if adj:
                    kern[(ord(l), ord(r))] = max(-128, min(127, adj))
    return make_font_blob(glyphs, ascent, descent, ascent + descent, kern)


class Font:
    """Host view of a font blob: codepoint -> glyph index, advances and kerning, for layout."""

    def __init__(self, blob):
        if blob[:4] != b"SKF\x01":
            raise ValueError("not a font blob (SKF version 1)")
        self.blob = bytes(blob)
        self.size, self.n, self.k, self.ascent, self.descent, self.line_spacing = struct.unpack_from("<IHHBBB", blob, 4)
        self.entries = [struct.unpack_from("<IBBBBhh", blob, 16 + 12 * i) for i in range(self.n)]
        cps = struct.unpack_from(f"<{self.n}I", blob, 16 + 12 * self.n)
        self.index = {cp: i for i, cp in enumerate(cps)}
        keys = struct.unpack_from(f"<{self.k}I", blob, 16 + 16 * self.n)
        adjs = struct.unpack_from(f"<{self.k}b", blob, 16 + 16 * self.n + 4 * self.k)
        self.kerning = dict(zip(keys, adjs))

    def indices(self, s):
        out = []
        for ch in s:
            g = self.index.get(ord(ch), self.index.get(0xFFFD))
            if g is not None:
                out.append(g)
        return out

    def kern(self, left, right):
        return self.kerning.get(left << 16 | right, 0)

    def advance(self, indices):
        """Pen advance of a run of glyph indices: advances plus kerning between neighbours."""
        return sum(self.entries[g][3] for g in indices) + sum(self.kern(a, b) for a, b in zip(indices, indices[1:]))

    def width(self, s):
        return self.advance(self.indices(s))


# ---------------------------------------------------------------- the VM (VM.md §2, §3)

ARENA_SIZE = 262144
UNKNOWN = 0x80000000
COPY, MAX, MASK, ERASE = 0, 1, 2, 3   # drawing modes; BLIT mode | ONE_BPP for a 1 bpp source
ONE_BPP = 4

# name: (opcode, operand count, literal bytes); WRITE and WRITE_RLE are built by Code.write*
OPS = {
    "COPY": (0x03, 3, 0), "UNPACK": (0x04, 4, 0), "MOV": (0x10, 2, 0),
    "ADD": (0x11, 3, 0), "SUB": (0x12, 3, 0), "MUL": (0x13, 3, 0), "DIVU": (0x14, 3, 0), "MODU": (0x15, 3, 0),
    "DIVS": (0x16, 3, 0), "MODS": (0x17, 3, 0), "AND": (0x18, 3, 0), "OR": (0x19, 3, 0), "XOR": (0x1A, 3, 0),
    "SHL": (0x1B, 3, 0), "SHR": (0x1C, 3, 0), "SAR": (0x1D, 3, 0),
    "LD32": (0x20, 3, 0), "LD16": (0x21, 3, 0), "LD8": (0x22, 3, 0), "LD4": (0x23, 3, 0),
    "ST32": (0x24, 3, 0), "ST16": (0x25, 3, 0), "ST8": (0x26, 3, 0), "ST4": (0x27, 3, 0),
    "JMP": (0x30, 0, 3), "CALL": (0x31, 0, 3), "RET": (0x32, 0, 0), "END": (0x33, 0, 0), "HALT": (0x34, 0, 0),
    "BEQ": (0x35, 2, 3), "BNE": (0x36, 2, 3), "BLTU": (0x37, 2, 3), "BGEU": (0x38, 2, 3),
    "BLTS": (0x39, 2, 3), "BGES": (0x3A, 2, 3),
    "SLEEP_UNTIL": (0x40, 1, 0), "WAIT_EVENT": (0x41, 2, 0), "RAISE": (0x42, 1, 0), "NOTIFY": (0x43, 2, 0),
    "SYS_READ": (0x44, 2, 0), "SYS_WRITE": (0x45, 2, 0),
    "FILL": (0x50, 7, 0), "BLIT": (0x51, 12, 0), "GLYPHS": (0x52, 9, 0), "DRAW": (0x53, 8, 0), "LINE": (0x54, 7, 0),
    "VIDEO": (0x55, 10, 0), "ASSET_LOAD": (0x56, 4, 0),
}
OP_NAMES = {v[0]: k for k, v in OPS.items()} | {0x01: "WRITE", 0x02: "WRITE_RLE"}

PROPS = {"TIME_US": 0x00, "TRIGGER": 0x01, "PENDING": 0x02, "ARM": 0x03, "LINK": 0x04, "LINK_INTERVAL": 0x05,
         "BATT_MV": 0x10, "BATT_PCT": 0x11, "CHARGE": 0x12, "IN_CASE": 0x13, "WORN": 0x14, "BATT_MA": 0x15,
         "BATT_TEMP": 0x16, "ACC_X": 0x18, "ACC_Y": 0x19, "ACC_Z": 0x1A, "PITCH": 0x1B, "LIGHT": 0x1C,
         "ROLL": 0x1D, "BRIGHTNESS": 0x20, "PANEL": 0x21, "TIMER": 0x22, "MOTION_MG": 0x23, "PITCH_LO": 0x24,
         "PITCH_HI": 0x25, "LIGHT_LO": 0x26, "LIGHT_HI": 0x27, "IMU_RATE": 0x28, "BRIGHTNESS_MAX": 0x29,
         "BRIGHTNESS_CAP": 0x2A, "GYR_X": 0x30, "GYR_Y": 0x31,
         "GYR_Z": 0x32, "GYRO_RATE": 0x33, "TOUCH_GESTURE": 0x34, "CASE_LID": 0x35, "CASE_CHARGING": 0x36,
         "CASE_BATT_PCT": 0x37, "AUTO_BRIGHTNESS": 0x38, "PEER_LINK": 0x39, "PEER_SEND": 0x3A, "PEER_MSG": 0x3B,
         "PEER_T_US": 0x3C}
OWNED_PROPS = {PROPS[n] for n in ("GYRO_RATE", "AUTO_BRIGHTNESS", "PEER_LINK", "PEER_SEND")}
SIGNED_PROPS = {"BATT_MA", "BATT_TEMP", "ACC_X", "ACC_Y", "ACC_Z", "PITCH", "ROLL", "PITCH_LO", "PITCH_HI",
                "GYR_X", "GYR_Y", "GYR_Z"}
TRIGGERS = {0: "START", 1: "TIMER", 2: "MOTION", 3: "TILT", 4: "LIGHT", 5: "BATTERY", 6: "CASE", 7: "WEAR",
            8: "TOUCH", 9: "PEER", 12: "USER0", 13: "USER1", 14: "USER2", 15: "USER3"}


class Code:
    """Instructions as bytes (VM.md §2.1). Operands are arena word indices: 0-14 are the registers
    r0-r14, any other word is an escape (a u16 word index after the instruction). Literal fields: the
    u24 target of JMP, CALL and the branches. The same bytes run as host code (in a packet) or, written
    into the arena, as a program."""

    def __init__(self):
        self.b = bytearray()

    def __len__(self):
        return len(self.b)

    def ins(self, name, *words, target=None):
        op, k, lit = OPS[name]
        if len(words) != k:
            raise ValueError(f"{name} takes {k} operands")
        b = bytearray([op])
        for i in range(0, k, 2):
            hi = min(words[i], 15)
            lo = min(words[i + 1], 15) if i + 1 < k else 0
            b.append(hi << 4 | lo)
        if lit:
            b += struct.pack("<I", target or 0)[:3]
        for w in words:
            if not 0 <= w <= 0xFFFF:
                raise ValueError(f"word {w} outside the arena")
            if w >= 15:
                b += struct.pack("<H", w)
        self.b += b
        return self

    def write(self, addr, data):
        data = bytes(data)
        if len(data) > 255:
            raise ValueError("WRITE carries at most 255 bytes")
        self.b += bytes([0x01]) + struct.pack("<I", addr)[:3] + bytes([len(data)]) + data
        return self

    def write_rle(self, addr, length, packed):
        if len(packed) > 255:
            raise ValueError("WRITE_RLE carries at most 255 bytes")
        self.b += bytes([0x02]) + struct.pack("<I", addr)[:3] + struct.pack("<HB", length, len(packed)) + bytes(packed)
        return self


def _mirror_lib():
    """The VM core plus g1mirror.c as a shared library, built once per source version."""
    import ctypes
    here = Path(__file__).resolve().parent
    vm_dir = here.parent / "src/vm"
    srcs = [here / "g1mirror.c", vm_dir / "vm.c", vm_dir / "vm_gfx.c"]
    digest = hashlib.sha256(b"".join(p.read_bytes() for p in srcs + [vm_dir / "vm.h", vm_dir / "vm_gfx.h"]))
    out = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "g1host" / f"g1mirror-{digest.hexdigest()[:16]}.so"
    if not out.exists():
        out.parent.mkdir(parents=True, exist_ok=True)
        tmp = out.with_suffix(f".{os.getpid()}.tmp")
        subprocess.run(["cc", "-std=c11", "-O2", "-shared", "-fPIC", f"-I{vm_dir}", "-o", str(tmp)] + [str(p) for p in srcs],
                       check=True)
        tmp.rename(out)
    lib = ctypes.CDLL(str(out))
    P, U8P, U32 = ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32
    for name, res, args in (("mirror_new", P, [ctypes.c_int, ctypes.c_int]), ("mirror_free", None, [P]),
                            ("mirror_arena", U8P, [P]), ("mirror_panel", U8P, [P]), ("mirror_panel_on", ctypes.c_int, [P]),
                            ("mirror_set_panel", None, [P, ctypes.c_int]), ("mirror_set_time", None, [P, U32]),
                            ("mirror_set_prop", None, [P, U32, U32]), ("mirror_set_triggers", None, [P, U32]),
                            ("mirror_check_write", ctypes.c_int, [P, U32, U32]),
                            ("mirror_packet", ctypes.c_int, [P, ctypes.c_char_p, U32]),
                            ("mirror_event", ctypes.c_int, [P, ctypes.c_int, ctypes.c_char_p]),
                            ("mirror_reset", None, [P, ctypes.c_int]),
                            ("mirror_set_asset", ctypes.c_int, [P, ctypes.c_char_p, ctypes.c_char_p, U32])):
        fn = getattr(lib, name)
        fn.restype = res
        fn.argtypes = args
    return lib


def _lc3_lib():
    """liblc3 (scouter_kit/third_party/liblc3, the firmware's encoder) as a shared library, built once."""
    import ctypes
    here = Path(__file__).resolve().parent
    lc3_dir = here.parent / "third_party/liblc3"
    srcs = sorted((lc3_dir / "src").glob("*.c"))
    digest = hashlib.sha256(b"".join(p.read_bytes() for p in srcs + sorted((lc3_dir / "include").glob("*.h"))))
    out = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "g1host" / f"liblc3-{digest.hexdigest()[:16]}.so"
    if not out.exists():
        out.parent.mkdir(parents=True, exist_ok=True)
        tmp = out.with_suffix(f".{os.getpid()}.tmp")
        subprocess.run(["cc", "-std=c11", "-O2", "-ffast-math", "-shared", "-fPIC", f"-I{lc3_dir / 'include'}",
                        "-DLC3_PLUS=0", "-DLC3_PLUS_HR=0", "-o", str(tmp)] + [str(p) for p in srcs] + ["-lm"], check=True)
        tmp.rename(out)
    lib = ctypes.CDLL(str(out))
    lib.lc3_decoder_size.restype = ctypes.c_uint
    lib.lc3_decoder_size.argtypes = [ctypes.c_int, ctypes.c_int]
    lib.lc3_setup_decoder.restype = ctypes.c_void_p
    lib.lc3_setup_decoder.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_void_p]
    lib.lc3_decode.restype = ctypes.c_int
    lib.lc3_decode.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_void_p, ctypes.c_int]
    return lib


class Lc3Decoder:
    """MIC_DATA frames (VM.md §6.5: 16 kHz, 10 ms) to 160 s16 samples each; a missing frame (None) is
    concealed by the codec (PLC). decode() returns (samples, status): 0 a valid frame, 1 concealed."""

    def __init__(self):
        import ctypes
        self.ct = ctypes
        self.lib = _lc3_lib()
        self.mem = ctypes.create_string_buffer(self.lib.lc3_decoder_size(10000, 16000))
        self.dec = self.lib.lc3_setup_decoder(10000, 16000, 0, self.mem)
        self.pcm = (ctypes.c_int16 * 160)()

    def decode(self, frame):
        if frame is None:
            st = self.lib.lc3_decode(self.dec, None, 0, 0, self.pcm, 1)
        else:
            st = self.lib.lc3_decode(self.dec, frame, len(frame), 0, self.pcm, 1)
        return list(self.pcm), st


class Mirror:
    """The arm as the host expects it: the same VM core runs every host packet (VM.md §5.6; programs are
    not mirrored, so what they write and draw is unknown here). arena and panel are writable views of
    the core's memory (panel: one level per pixel, 640 x 480)."""

    def __init__(self, arm_role=0, has_panel=True):
        import ctypes
        self.lib = _mirror_lib()
        self.m = self.lib.mirror_new(arm_role, 1 if has_panel else 0)
        self.arena = (ctypes.c_uint8 * ARENA_SIZE).from_address(ctypes.addressof(self.lib.mirror_arena(self.m).contents))
        self.panel = (ctypes.c_uint8 * (PANEL_W * PANEL_H)).from_address(
            ctypes.addressof(self.lib.mirror_panel(self.m).contents))
        self.events = []

    def __del__(self):
        if getattr(self, "m", None):
            self.lib.mirror_free(self.m)
            self.m = None

    def packet(self, data, dev_time=None):
        """Run one packet (u16 seq + host code); returns its events as (type, body) pairs."""
        if dev_time is not None:
            self.lib.mirror_set_time(self.m, dev_time & M32)
        n = self.lib.mirror_packet(self.m, bytes(data), len(data))
        buf = bytes(64)
        out = []
        for i in range(n):
            k = self.lib.mirror_event(self.m, i, buf)
            out.append((buf[0], bytes(buf[1:k])))
        self.events += out
        return out

    def panel_on(self):
        return bool(self.lib.mirror_panel_on(self.m))

    def set_panel(self, v):
        self.lib.mirror_set_panel(self.m, int(v))

    def set_prop(self, pid, value):
        self.lib.mirror_set_prop(self.m, pid, value & M32)

    def set_triggers(self, triggers):
        self.lib.mirror_set_triggers(self.m, triggers)

    def check_write(self, pid, value):
        return self.lib.mirror_check_write(self.m, pid, value & M32)

    def reset(self, clear_arena):
        self.lib.mirror_reset(self.m, 1 if clear_arena else 0)

    def set_asset(self, aid, data):
        """The bytes of a stored asset, for the mirror's ASSET_LOAD (empty data forgets them)."""
        self.lib.mirror_set_asset(self.m, bytes(aid), bytes(data), len(data))

    def word(self, i):
        return struct.unpack_from("<I", self.arena, 4 * i)[0]

    def panel_rows(self, y0=0, rows=PANEL_H):
        p = bytes(self.panel)
        return b"".join(bytes((p[y * PANEL_W + i] << 4) | p[y * PANEL_W + i + 1] for i in range(0, PANEL_W, 2))
                        for y in range(y0, y0 + rows))


# ---------------------------------------------------------------- protocol session (VM.md §6)

PROTO = 5
HELLO_FMT = "<BBBBBBBHHHIBHHHBIIBB"
HELLO_KEYS = ("proto", "_v0", "_v1", "_v2", "role", "flags", "board", "triggers", "panel_w", "panel_h", "arena",
              "queue", "max_pkt", "rx_next", "exec_next", "credits", "boot_count", "resetreas", "safe_level",
              "unhealthy")
PAIR_COMPANY = 0xFFFF
CTRL = {"HELLO": 0x01, "PING": 0x02, "STATUS": 0x03, "RESET_SESSION": 0x04, "LINK": 0x05, "LOG_READ": 0x06,
        "HEALTH_OK": 0x07, "REBOOT": 0x08, "DIAG": 0x09, "PROPS": 0x0A, "PANEL_READ": 0x0B, "ARENA_CRC": 0x0C,
        "VM_START": 0x10, "VM_STOP": 0x11, "VM_STATUS": 0x12, "MIC": 0x13, "EVENTS": 0x14,
        "PANEL_GUARD": 0x16, "ASSET_INFO": 0x17, "ASSET_LIST": 0x18, "ASSET_PUT": 0x19, "ASSET_DELETE": 0x1A,
        "ASSET_FORMAT": 0x1B, "FLASH_CRC": 0x1C, "FC_INFO": 0x1D, "PROP_SET": 0x1E, "FAULT": 0x3F}
ASSET_INFO_KEYS = ("data", "free", "largest", "entries", "state", "error", "flags", "seq")
ASSET_STATES = {0: "idle", 1: "erasing", 2: "writing", 3: "verifying", 4: "done", 5: "failed"}
FORMAT_MAGIC, WIPE_MAGIC = 0x534B4153, 0x50570000
# the flash the store never writes (docs/design-flash-assets.md items 1, 27): the update slots, stock's settings
PROTECTED = ((0x0, 0x130000), (0x130000, 0x10000))
ASSET_ID_AT = 384   # in SCRATCH: the ID of an ASSET_LOAD in host code
VM_STATUS_FMT = "<BBHIIBBBBIIHHBHB"
VM_STATUS_KEYS = ("state", "trigger", "pending", "table", "next", "task", "jumps_left", "depth", "halt_code",
                  "timer", "tasks", "coalesced", "notify_dropped", "flags", "cap_pauses", "busy_ms")
VM_STATES = {0: "stopped", 1: "running", 2: "halted"}


def ad_structures(payload):
    """Advertising or scan response payload -> {AD type: bytes} (VM.md §6.1)."""
    out, i = {}, 0
    while i + 1 < len(payload) and payload[i]:
        n = payload[i]
        out[payload[i + 1]] = bytes(payload[i + 2:i + 1 + n])
        i += 1 + n
    return out


def pair_record(data):
    """The pair record: manufacturer data for company 0xFFFF without the company bytes (as bleak and
    BlueZ present it). None if it is not a format-1 record."""
    if len(data) < 3 or data[0] != 1:
        return None
    fl = data[1]
    return dict(role="LR"[0 if fl & 1 else 1], no_panel=bool(fl & 2), from_serial=bool(fl & 4),
                pair_id=bytes(data[2:]).decode("ascii", "replace"))


STATUS_FMT = "<HHBBBBBIIHIHHHHHHHBHII"
STATUS_KEYS = ("vbat_mv", "ntc_code", "charge_status", "vbus_status", "vterm_code", "pmic_flags",
               "charge_refused", "packets", "instructions", "errors", "draws", "late", "duplicates",
               "window_errors", "event_drops", "interval", "latency", "timeout", "phy", "mtu",
               "uptime_ms", "i2c_errors")

POWER_FMT = "<HhhhiiHHH8BHHHIBBBBBHB"
POWER_KEYS = ("batt_mv", "batt_ma", "batt_temp_dc", "die_temp_dc", "batt_ua", "ibat_ua", "vbus_mv", "vsys_mv",
              "vrect_mv", "charge_status", "charge_error", "vbus_status", "ldsw_status", "ibat_stat", "case_status",
              "case_level", "case_lid", "case_requests", "case_age_s", "cpu_permille", "samples", "flags", "state",
              "imu_id", "imu_ctrl1_xl", "imu_ctrl2_g", "invalid", "gpo_events")
POWER_STATE_FMT = "<IIqiiQQ"
POWER_STATE_KEYS = ("samples", "ua_samples", "sum_ua", "sum_ntc_dc", "sum_die_dc", "busy_cycles", "all_cycles")
POWER_STATE_NAMES = {0: "advertising, panel off", 1: "advertising, panel on", 2: "connected, panel off",
                     3: "connected, panel on", 4: "VBUS, advertising, panel off", 5: "VBUS, advertising, panel on",
                     6: "VBUS, connected, panel off", 7: "VBUS, connected, panel on"}


def power_state_delta(before, after):
    """Per-state averages between two power_states() reads: mA, degC and CPU % of the samples taken in
    between (the arm samples once a second)."""
    out = []
    for b, a in zip(before, after):
        n = a["samples"] - b["samples"]
        if n <= 0:
            continue
        nu = a["ua_samples"] - b["ua_samples"]
        cyc = a["all_cycles"] - b["all_cycles"]
        out.append({"state": a["state"], "name": POWER_STATE_NAMES[a["state"]], "samples": n, "ua_samples": nu,
                    "avg_ma": round((a["sum_ua"] - b["sum_ua"]) / nu / 1000.0, 3) if nu > 0 else None,
                    "ntc_c": round((a["sum_ntc_dc"] - b["sum_ntc_dc"]) / n / 10.0, 2),
                    "die_c": round((a["sum_die_dc"] - b["sum_die_dc"]) / n / 10.0, 2),
                    "cpu_pct": round(100.0 * (a["busy_cycles"] - b["busy_cycles"]) / cyc, 3) if cyc else None})
    return out


class ProtocolMismatch(RuntimeError):
    """The arm does not speak protocol 5 (HELLO's first byte)."""


class EpochLost(RuntimeError):
    """The arm rebooted (new epoch) while this draw was in flight: its arena is gone, re-upload."""


class Refused(ValueError):
    """A write the arm would refuse (VM.md §2.5: a setting of a part it lacks), not sent."""


class ControlError(RuntimeError):
    """A control request answered with an error status."""


# the client's own arena words: constants (the pool) at the top of the arena, below them a scratch area
# for glyph indices and draw markers; programs must not write there (VM.md §5.6)
POOL_WORDS = 1024
POOL_BASE = ARENA_SIZE // 4 - POOL_WORDS
SCRATCH = 4 * POOL_BASE - 512
HOST_TOP = SCRATCH


class Arm:
    def __init__(self, tr, name="arm", log=None, auto_resync=None):
        self.tr = tr
        self.auto_resync = isinstance(tr, BleTransport) if auto_resync is None else auto_resync
        self.resync_due = False
        self._resyncing = False
        self.name = name
        self.log = log or (lambda s: print(s, flush=True))
        self.listeners = {"G1_EVT": self._on_evt}
        self.fc_acks = []
        self.smp = Smp(self)
        self.epoch = None
        self.mirror = None
        self.info = {}
        self.tag = 0
        self.replies = {}
        self.samples = []
        self.offset = None
        self.offset_err_us = None
        self.errors = []
        self.halts = []
        self.notifies = []
        self.inputs = []
        self.notices = []
        self.on_mic = None      # MIC_DATA: called as on_mic(first, n, frames bytes, t_dev, t_host); never stored here
        self.mic_events = 0
        self.events = []        # every ERROR, HALT and NOTIFY as (type, body after offset 9), in order
        self.fences = {}
        self.sent_at = {}
        self.acked_at = {}
        self.hb_every = 0.0     # > 0: send the stock NUS heartbeat 25 06 00 seq 04 seq this often (stock phase)
        self.hb_next = 0.0
        self.hb_sent = 0
        self.stats = {"packets": 0, "resent": 0, "fast_resent": 0, "epoch_changes": 0, "resyncs": 0, "hellos": 0}
        self.link_events = []
        self.adv = {}           # emulator with the vphone's ScanFirst: last "adv" and "scanrsp" payloads
        self.window = 12
        self.case_hold = None
        self.heard_t = 0.0
        self.interval_s = 0.0075
        self._clear_window()

    def _clear_window(self):
        self.next_seq = self.rx_next = self.exec_next = 0
        self.credits = 0
        self.sack = 0
        self.unacked = OrderedDict()
        self.outq = []
        self.cur = Code()
        self.max_pkt = 20
        self.frame_start_seq = None
        self.draws = []
        self.pool = {}
        self.pool_pending = set()
        self.pool_written = set()
        self.arena_top = 0x400
        self.srtt = None
        self.rttvar = None
        self.backoff = 1
        self.progress_t = 0.0

    # -- driving the transport
    def pump(self):
        if self.hb_every and self.tr.connected and self.tr.has("NUS_RX") and self.tr.now() >= self.hb_next:
            seq = self.hb_sent % 0xFF
            self.tr.write("NUS_RX", bytes([0x25, 0x06, 0x00, seq, 0x04, seq]))
            self.hb_sent += 1
            self.hb_next = self.tr.now() + self.hb_every
        if self.resync_due and not self._resyncing and self.tr.connected:
            self._resync()
        if self.tr.connected and self.tr.has("G1_CMD") and self.epoch is not None and not self.resync_due:
            self._send_packets()
        for kind, name, data, t in self.tr.step():
            if kind == "notify":
                fn = self.listeners.get(name)
                if fn:
                    fn(data, t)
            elif name.startswith(("adv ", "scanrsp ")):
                kind, _, hexdata = name.partition(" ")
                self.adv[kind] = bytes.fromhex(hexdata)
            else:
                self.link_events.append((name.split(" ")[0], t))
                self.log(f"[{self.name}] {t:9.3f}s link {name}")
                if self.auto_resync and name == "connected" and self.epoch is not None:
                    self.resync_due = True

    def _resync(self):
        """After a reconnect, HELLO tells whether the arm kept its state (VM.md §6.2). Same epoch: resend
        from rx_next. New epoch: the arm rebooted, its arena is gone; draws in flight fail with EpochLost
        and hello() resets the mirror and the window, so the caller uploads again."""
        self._resyncing = True
        try:
            old = self.epoch
            open_draws = [d for d in self.draws if not d["done"]]
            self.hello()
            if self.epoch != old:
                for d in open_draws:
                    d.update(done=True, epoch_lost=True)
                self.log(f"[{self.name}] resync: new epoch, arm memory lost ({len(open_draws)} draws failed)")
                return
            now = self.tr.now()
            self.sack = 0
            for seq in list(self.unacked):
                if seq_before(seq, self.rx_next):
                    del self.unacked[seq]
            for seq, ent in list(self.unacked.items()):
                self._send(seq, ent["data"], now, resend=True)
            self.progress_t = now
            for d in open_draws:
                if seq_before(d["send_seq"], self.exec_next):
                    d.update(done=True, event_lost=True)
            self.stats["resyncs"] += 1
            self.log(f"[{self.name}] resync: same epoch, rx_next {self.rx_next}, resent {len(self.unacked)}")
        finally:
            self._resyncing = False
            self.resync_due = False

    def run_until(self, cond, timeout, what="condition"):
        end = self.tr.now() + timeout
        while not cond():
            if self.tr.now() > end:
                raise TimeoutError(f"{self.name}: {what} not reached within {timeout} s")
            self.pump()

    def wait(self, seconds):
        end = self.tr.now() + seconds
        while self.tr.now() < end:
            self.pump()

    def losses(self):
        return sum(1 for kind, _t in self.link_events if kind == "lost")

    def setup(self, timeout=60.0):
        """Wait for a link, then (emulator) MTU exchange, discovery and subscriptions."""
        self.run_until(lambda: self.tr.connected, timeout, "connection")
        if isinstance(self.tr, EmuTransport):
            self.tr.script("mtu 247")
            self.tr.script("discover")
            self.pump()
            self.run_until(self.tr.scripts_idle, timeout, "discovery")
            for _ in range(3):   # the protocol needs ATT MTU >= 65; retry a lost exchange
                if self.tr.mtu >= 65 or not self.tr.connected:
                    break
                self.log(f"[{self.name}] ATT MTU still {self.tr.mtu}: repeating the exchange")
                self.tr.script("mtu 247")
                self.pump()
                self.run_until(self.tr.scripts_idle, timeout, "MTU exchange")
            for name in ("G1_EVT", "SMP", "NUS_TX"):
                if self.tr.has(name):
                    self.tr.script(f"subscribe {name}")
            self.pump()
            self.run_until(self.tr.scripts_idle, timeout, "subscriptions")

    # -- control requests (VM.md §6.4; unsequenced, a lost one is retried with a new tag)
    def ctrl(self, op, body=b"", timeout=3.0, tries=3):
        op = CTRL.get(op, op)
        for attempt in range(tries):
            self.tag = (self.tag + 1) & 0xFF
            tag = self.tag
            self.tr.write("G1_CTRL", bytes([op, tag]) + bytes(body))
            try:
                self.run_until(lambda: tag in self.replies, timeout, f"reply to control 0x{op:02x}")
                return self.replies.pop(tag)
            except TimeoutError:
                if attempt == tries - 1:
                    raise

    def ctrl_ok(self, op, body=b"", **kw):
        rep = self.ctrl(op, body, **kw)
        if rep["status"]:
            raise ControlError(f"{self.name}: control {op} status {rep['status']} ({ERRORS.get(rep['status'], '?')})")
        return rep["body"]

    def hello(self):
        rep = self.ctrl("HELLO")
        self.stats["hellos"] += 1
        return self._apply_hello(rep["body"], rep["epoch"])

    def panel_guard(self, on=None):
        """PANEL_GUARD (0.5.52+, VM.md §6.6): False switches the panel guard off (BRIGHTNESS_CAP 42, no NOTICE 7)
        until True or a reboot, a disconnection keeps it; None only asks. None from older firmware."""
        rep = self.ctrl("PANEL_GUARD", b"" if on is None else bytes([1 if on else 0]))
        return bool(rep["body"][0]) if rep["status"] == 0 and rep["body"] else None

    def _apply_hello(self, body, epoch):
        if not body or body[0] != PROTO or len(body) < struct.calcsize(HELLO_FMT):
            v = f"{body[1]}.{body[2]}.{body[3]}" if len(body) >= 4 else "?"
            raise ProtocolMismatch(f"{self.name}: arm speaks protocol {body[0] if body else '?'} (firmware {v}), "
                                   f"this client speaks {PROTO}; smp-* commands still work")
        f = dict(zip(HELLO_KEYS, struct.unpack_from(HELLO_FMT, body)))
        fl, bd = f["flags"], f["board"]
        info = dict(proto=f["proto"], version=f"{f['_v0']}.{f['_v1']}.{f['_v2']}", role="RL"[f["role"]],
                    flags=fl, safe_mode=bool(fl & 1), panel_ok=bool(fl & 2), pmic_ok=bool(fl & 4),
                    panel_on=bool(fl & 8), charging=bool(fl & 16), test_build=bool(fl & 32),
                    encrypted=bool(fl & 64), in_case=bool(fl & 128),
                    board=bd, no_panel=bool(bd & 1), bondable=bool(bd & 2), mic=bool(bd & 4),
                    triggers=f["triggers"], trigger_names=[v for b, v in TRIGGERS.items() if f["triggers"] >> b & 1],
                    panel=(f["panel_w"], f["panel_h"]), arena=f["arena"], queue=f["queue"],
                    max_pkt=f["max_pkt"], rx_next=f["rx_next"], exec_next=f["exec_next"],
                    credits=f["credits"], boot_count=f["boot_count"], resetreas=f["resetreas"],
                    safe_level=f["safe_level"], unhealthy=f["unhealthy"], epoch=epoch)
        if epoch != self.epoch:
            if self.epoch is not None:
                self.stats["epoch_changes"] += 1
                self.log(f"[{self.name}] epoch changed {self.epoch:08x} -> {epoch:08x}: arm memory reset")
            self.epoch = epoch
            self.samples = []
            self.offset = None
            self._clear_window()
            self.next_seq = info["rx_next"]
            self.mirror = Mirror(f["role"], not info["no_panel"])
            # the arm refuses the settings of parts it lacks (GYRO_RATE without the IMU, AUTO_BRIGHTNESS without
            # the light sensor unless a left arm follows the right one over the arm-to-arm link, PEER_*)
            self.mirror.set_triggers(f["triggers"])
        self.rx_next, self.exec_next, self.credits = info["rx_next"], info["exec_next"], info["credits"]
        self.max_pkt = min(info["max_pkt"], self.tr.mtu - 3)
        if not info["panel_on"] and not self.unacked and not self.outq and self.mirror.panel_on():
            self.mirror.set_panel(0)   # off at boot and 30 s after the last DRAW with no host
        self.heard_t = self.tr.now()
        self.info = info
        return info

    def status(self):
        return dict(zip(STATUS_KEYS, struct.unpack_from(STATUS_FMT, self.ctrl_ok("STATUS"))))

    def props(self, *names):
        """PROPS: the named properties (VM.md §2.5) as a dict; signed ones as Python ints, UNKNOWN as None."""
        ids = [PROPS[n] if isinstance(n, str) else n for n in names]
        out = {}
        for i in range(0, len(ids), 12):
            part = ids[i:i + 12]
            vals = struct.unpack(f"<{len(part)}I", self.ctrl_ok("PROPS", bytes(part)))
            for pid, v in zip(part, vals):
                name = next((k for k, x in PROPS.items() if x == pid), pid)
                out[name] = None if v == UNKNOWN else signed32(v) if name in SIGNED_PROPS else v
        return out

    def battery(self):
        return self.props("BATT_MV", "BATT_PCT", "CHARGE", "IN_CASE", "BATT_MA", "BATT_TEMP")

    def power(self, page=0):
        """DIAG POWER (0x09 sub-op 1): power telemetry (g1_firmware/README.md). Page 0: live values;
        pages 1..8: the accumulators of state page-1 (bit 0 panel on, bit 1 host connected, bit 2 VBUS)."""
        body = self.ctrl_ok("DIAG", bytes([1, page]))
        if page:
            v = dict(zip(POWER_STATE_KEYS, struct.unpack_from(POWER_STATE_FMT, body)))
            v["state"] = page - 1
            return v
        return dict(zip(POWER_KEYS, struct.unpack_from(POWER_FMT, body)))

    def power_states(self):
        return [self.power(k) for k in range(1, 9)]

    def imu(self):
        """The accelerometer properties (VM.md §2.5): ACC_X/Y/Z mg, PITCH, ROLL degrees, IMU_RATE Hz."""
        return self.props("ACC_X", "ACC_Y", "ACC_Z", "PITCH", "ROLL", "IMU_RATE")

    def pin_probe(self, p0=None, p1=None):
        """DIAG PIN_PROBE (sub-op 3): the IN registers while the LSM6DSO pulls INT1, then INT2 low (open drain,
        the candidate pins pulled up); a pin that follows one of them is that line. p0/p1: only these pins
        (masks) of the candidates."""
        req = bytes([3]) if p0 is None else struct.pack("<BII", 3, p0, p1)
        body = self.ctrl_ok("DIAG", req, timeout=5.0)
        masks = struct.unpack_from("<II", body)
        steps = [struct.unpack_from("<II", body, 8 + 8 * i) for i in range(5)]
        pins, found = {}, {}
        for port in (0, 1):
            for pin in range(32):
                if masks[port] >> pin & 1:
                    levels = "".join(str(st[port] >> pin & 1) for st in steps)
                    pins[f"P{port}.{pin:02d}"] = levels
                    if levels == "10111":
                        found["INT1"] = f"P{port}.{pin:02d}"
                    elif levels == "11101":
                        found["INT2"] = f"P{port}.{pin:02d}"
        return {"pins": pins, "found": found}

    REG4 = ("UARTE0.ENABLE", "TWIM1.ENABLE", "TWIM2.ENABLE", "SPIM4.ENABLE", "SAADC.ENABLE", "PDM0.ENABLE",
            "QSPI.ENABLE", "CLOCK.HFCLKSTAT", "CLOCK.HFCLK192MSTAT", "CLOCK.LFCLKSTAT", "CLOCK.HFCLKCTRL",
            "VREGMAIN.DCDCEN", "VREGRADIO.DCDCEN", "VREGH.DCDCEN", "RESET.NETWORK.FORCEOFF", "WDT0.RUNSTATUS",
            "GPIOTE0.INTENSET", "P0.LATCH", "P1.LATCH", "P0.DETECTMODE")
    REG5 = ("BUCK1NORMVOUT", "BUCK2NORMVOUT", "BUCKENCTRL", "BUCKPWMCTRL", "BUCKSWCTRLSEL", "BUCKCTRL0",
            "BUCKSTATUS", "LDSWSTATUS", "LDSW1GPISEL", "LDSW2GPISEL", "LDSWCONFIG", "LDSW1LDOSEL", "LDSW2LDOSEL",
            "LDSW1VOUTSEL", "LDSW2VOUTSEL", "GPIOMODE0", "GPIOMODE1", "GPIOMODE2", "GPIOMODE3", "GPIOMODE4",
            "GPIOPUEN0", "GPIOPUEN1", "GPIOPUEN2", "GPIOPUEN3", "GPIOPUEN4", "GPIOPDEN0", "GPIOPDEN1",
            "GPIOPDEN2", "GPIOPDEN3", "GPIOPDEN4", "GPIOSTATUS", "LEDDRV0MODESEL", "LEDDRV1MODESEL",
            "LEDDRV2MODESEL", "TIMERSET", "TIMERCONFIG", "POFCONFIG", "SHIP_0B04", "BCHGENABLESET",
            "BCHGCHARGESTATUS", "VBUSINSTATUS")

    def regs(self, page):
        """DIAG REGS (sub-op 4), read only: 0-2 PIN_CNF of P0.00-15, P0.16-31, P1.00-15; 3 OUT/IN/DIR and
        GPIOTE CONFIG; 4 peripheral ENABLEs, clocks, regulators; 5 nPM1300 rails, GPIOs, LEDs."""
        body = self.ctrl_ok("DIAG", bytes([4, page]))[1:]
        if page <= 2:
            port, base = page // 2, (page & 1) * 16
            return {f"P{port}.{base + i:02d}": f"0x{v:08x}" for i, v in enumerate(struct.unpack("<16I", body[:64]))}
        if page == 3:
            v = struct.unpack("<6I8I", body[:56])
            out = {f"P{p}.{k}": f"0x{v[3 * p + i]:08x}" for p in (0, 1) for i, k in enumerate(("OUT", "IN", "DIR"))}
            out.update({f"GPIOTE.CONFIG{i}": f"0x{c:08x}" for i, c in enumerate(v[6:])})
            return out
        if page == 4:
            return {k: f"0x{x:08x}" for k, x in zip(self.REG4, struct.unpack(f"<{len(body) // 4}I", body))}
        return {k: f"0x{x:02x}" for k, x in zip(self.REG5, body)}

    def diag_set(self, item, value):
        """DIAG SET (sub-op 5, test builds): 1 touch rail P0.30, 2 LDSW2 (IMU + microphone), 3 touch controller
        mode (0 read, 1 active, 2 sleep, 3 deep sleep: chip ID and REG_WST before/after)."""
        body = self.ctrl_ok("DIAG", bytes([5, item, value]), timeout=5.0)
        out = {}
        if item == 3:
            out = dict(zip(("chip_id", "wst_before", "wst_after"), (f"0x{x:08x}" for x in struct.unpack_from("<3I", body))))
            body = body[12:]
        out.update(touch_rail=body[0], imu=body[1])
        return out

    def imu_stats(self):
        """DIAG IMU (sub-op 8): samples read by the imu thread, polls without a new sample, I2C errors, whether
        every sample is being read (MOTION or TILT can fire), whether the IMU answers."""
        body = self.ctrl_ok("DIAG", bytes([8]))
        n, miss, err, on, present = struct.unpack_from("<IIIBB", body)
        irqs = struct.unpack_from("<I", body, 14)[0] if len(body) >= 18 else None   # 0.5.4+: INT1 wake-ups
        out = dict(samples=n, misses=miss, i2c_errors=err, streaming=bool(on), present=bool(present), irqs=irqs)
        if len(body) >= 40:   # 0.5.8+: stock's factory pitch base (centidegrees; None: no record) and the head map
            base = struct.unpack_from("<i", body, 18)[0]
            m = struct.unpack_from("<9h", body, 22)
            out.update(factory_pitch_base=None if base == -0x80000000 else base,
                       head_map=[[m[3 * i + j] / 16384 for j in range(3)] for i in range(3)])
        return out

    def touch_stats(self):
        """DIAG TOUCH (sub-op 11, 0.5.18+): the TouchBar driver's state and counters, the controller's chip ID,
        mode (REG_WST: 0x01000000 scanning, 0x03000000 asleep) and event enable, TOUCH_GESTURE, WORN and the
        interrupts taken."""
        body = self.ctrl_ok("DIAG", bytes([11]))
        present, scanning = struct.unpack_from("<2B", body)
        keys = ("events", "i2c_errors", "loads", "chip_id", "wst", "irqen", "gesture", "worn", "interrupts")
        v = dict(zip(keys, struct.unpack_from("<9I", body, 2)))
        v.update(present=bool(present), scanning=bool(scanning))
        return v

    def light_stats(self):
        """DIAG LIGHT (sub-op 12, 0.5.18+): the light sensor's state (present, converting, AUTO_BRIGHTNESS), its
        readings and I2C errors, and LIGHT (0.01 lux; None while it does not convert)."""
        body = self.ctrl_ok("DIAG", bytes([12]))
        present, converting, auto = struct.unpack_from("<3B", body)
        readings, errors, light = struct.unpack_from("<3I", body, 3)
        return dict(present=bool(present), converting=bool(converting), auto=bool(auto), readings=readings,
                    i2c_errors=errors, light=None if light == 0x80000000 else light)

    def peer_stats(self):
        """DIAG PEER (sub-op 13, 0.5.24+): the arm-to-arm link (available, PEER_LINK, the left's AUTO_BRIGHTNESS),
        the net core's last link report (its signal value, which decides WORN, and its ESB counters), the link's
        own counters, the clock offset (left minus right, µs; None until known) and the time from a frame's queueing
        to the net core's report that it went out (µs: last, longest, total)."""
        body = self.ctrl_ok("DIAG", bytes([13]))
        available, link, follow, value = struct.unpack_from("<4B", body)
        keys = ("reports", "net_acked", "net_failed", "net_received", "report_t", "frames_out", "reports_sent",
                "report_timeouts", "send_errors", "frames_in", "delivered", "inbox_drops", "levels_applied",
                "last_arrival_t", "other_msgs")
        v = dict(zip(keys, struct.unpack_from("<15I", body, 4)))
        offset = struct.unpack_from("<i", body, 64)[0]
        v.update(available=bool(available), link=link, follow=bool(follow), link_value=value,
                 offset_us=None if offset == -0x80000000 else offset)
        v.update(zip(("delay_last_us", "delay_max_us", "delay_total_us"), struct.unpack_from("<3I", body, 68)))
        v["delay_avg_us"] = v["delay_total_us"] // v["reports_sent"] if v["reports_sent"] else None
        return v

    def adc_record(self, n=64, wait_us=1000):
        """DIAG ADC_BURST (sub-op 16, 0.5.54+): n VBAT+IBAT conversions wait_us apart, as fast as the bus allows
        (the panel's pulse, docs/review-dimming.md point 12): [(t_us since the first, mA or None, mV)]."""
        out, first = [], 0
        while True:
            body = self.ctrl_ok("DIAG", struct.pack("<BBHB", 16, n, wait_us, first), timeout=5.0)
            taken = body[0]
            for i in range(2, len(body) - 5, 6):
                t, ma, mv = struct.unpack_from("<HhH", body, i)
                out.append((t * 10, None if ma == 0x7FFF else ma, mv))
            first = len(out)
            if first >= taken or len(body) <= 2:
                return out

    def guard_state(self):
        """DIAG GUARD (sub-op 17, 0.5.54+): the panel guard's last burst (mean mA, lowest mV, valid conversions),
        I_avg (mA, -1 none), the cap, the level shown and written, on, the release count, the attacks and the budget
        (mA; 0.5.56+, else None)."""
        body = self.ctrl_ok("DIAG", bytes([17]))
        ma, avg, mv, n, cap, shown, written, on, calm, attacks = struct.unpack_from("<hhH6BI", body)
        return dict(burst_ma=ma, avg_ma=avg, burst_min_mv=mv, burst_n=n, cap=cap, shown=shown, written=written,
                    on=bool(on), calm_s=calm, attacks=attacks,
                    budget_ma=struct.unpack_from("<H", body, 16)[0] if len(body) >= 18 else None)

    def bonds(self):
        """DIAG BONDS (sub-op 19, 0.5.58+, read only): the arm's bonds, oldest first by Zephyr's aging counter (a new
        pairing with all slots used replaces the first one not connected now): address, key types, flags (1
        authenticated, 0x10 LE Secure Connections), key size, the counter, whether that peer is connected now."""
        b = self.ctrl_ok("DIAG", bytes([19]))
        out = []
        for i in range(b[0]):
            e = b[5 + 16 * i:21 + 16 * i]
            addr = ":".join(f"{x:02X}" for x in reversed(e[1:7]))
            keys, flags, size, counter, live = struct.unpack_from("<HBBIB", e, 7)
            out.append(dict(addr=addr, type=("public", "random")[e[0]] if e[0] < 2 else e[0], keys=keys, flags=flags,
                            key_size=size, aging=counter, connected=bool(live)))
        return sorted(out, key=lambda x: x["aging"])

    def case_stats(self):
        """DIAG CASE (sub-op 14, 0.5.26+): IN_CASE and its inputs (VBUS, the tag's field, motion since the last of
        them, how long ago that was), the case link's counters and last bytes, the wear classification (-1 none)
        and whether the arm was in the case since, and how often the charger was enabled after a battery reply."""
        body = self.ctrl_ok("DIAG", bytes([14]))
        in_case, vbus, field, moved, since = struct.unpack_from("<4BI", body)
        v = dict(zip(("messages", "requests", "replies"), struct.unpack_from("<3I", body, 8)))
        v.update(zip(("busy", "unknown", "long_trip", "restarts"), struct.unpack_from("<4H", body, 20)))
        status, battery, lid, gpo, field_age, wear, after = struct.unpack_from("<4BIbB", body, 28)
        v.update(in_case=bool(in_case), vbus=bool(vbus), field=bool(field), moved=bool(moved), since_ms=since,
                 status=status, case_battery_byte=battery, lid_byte=lid, gpo_config=gpo,
                 field_age_ms=None if field_age == M32 else field_age, wear=wear, since_case=bool(after),
                 charger_enables=struct.unpack_from("<I", body, 38)[0])
        if len(body) >= 110:   # 0.5.32+: the reply times and the tag's interrupt bits
            v.update(zip(("gpo_events", "i2c_errors", "found_by_look", "replies_timed", "replies_late",
                          "reply_max_us", "reply_avg_us"), struct.unpack_from("<7I", body, 42)))
            v["it_sts_counts"] = dict(zip(("rf_user", "rf_activity", "rf_interrupt", "field_falling", "field_rising",
                                           "rf_put_msg", "rf_get_msg", "rf_write"), struct.unpack_from("<8I", body, 70)))
            v["vrect_mv"] = struct.unpack_from("<H", body, 108)[0]
        if len(body) >= 123:   # the LEDs off while the case talks to the arm, its panel on
            v.update(zip(("blanks", "blank_pulses", "blanked_s"), struct.unpack_from("<3I", body, 110)))
            v["blanked"] = bool(body[122])
        return v

    def case_trace(self):
        """DIAG CASE_TRACE (sub-op 15, 0.5.32+): the case link's last events as (ms of uptime, kind, value): edge (value
        1: the first of a poll window), it_sts (the tag's interrupt bits), reply (its status), blank, unblank."""
        body = self.ctrl_ok("DIAG", bytes([15]))
        kinds = {1: "edge", 2: "it_sts", 3: "reply", 4: "blank", 5: "unblank"}
        return [(ms, kinds.get(k, k), v) for ms, k, v in struct.iter_unpack("<IBB", body[1:1 + 6 * body[0]])]

    def mic_stats(self):
        """DIAG MIC (sub-op 9, 0.5.8+): the capture's counters and levels since the last MIC 1 (no audio)."""
        body = self.ctrl_ok("DIAG", bytes([9]))
        on, rate, fpe, avail = struct.unpack_from("<4B", body)
        keys = ("starts", "frames", "lost", "encoded", "events", "event_drops", "faults", "encode_us")
        v = dict(zip(keys, struct.unpack_from("<8I", body, 4)))
        v.update(zip(("encode_max_us", "peak", "rms"), struct.unpack_from("<3H", body, 36)))
        v.update(on=bool(on), rate=rate, per_event=fpe, available=bool(avail))
        v["encode_avg_us"] = round(v["encode_us"] / v["encoded"], 1) if v["encoded"] else None
        return v

    IDLE_SWITCHES = ("qspi-dpd", "qspi-pulls", "uart-tx", "lows", "panel-en", "sensors", "twim2")

    def net_off(self, seconds, phases, masks=None):
        """DIAG NET_OFF (sub-op 6, test builds): the arm reboots into a measurement with the network core held
        off (no Bluetooth), phases of `seconds` each, then reboots normally; read it with net_off_result. masks
        (0.5.32+): one idle-switch mask per phase (IDLE_SWITCHES bits) instead of the fixed phases."""
        body = struct.pack("<BHB", 6, seconds, phases) + (bytes(masks) if masks else b"")
        return self.ctrl_ok("DIAG", body)

    def net_off_result(self, masks=None):
        body = self.ctrl_ok("DIAG", bytes([7]))
        done, seconds, fails_at_test, fails_now = struct.unpack_from("<BHBB", body)
        names = ("net core off", "+ touch rail off", "+ LDSW2 off", "net core off, all on")
        out = []
        for i in range(done):
            n, nb, ua, v0, v1, ntc, cpu = struct.unpack_from("<IIiHHhH", body, 5 + 20 * i)
            name = ("+".join(x for b, x in enumerate(self.IDLE_SWITCHES) if masks[i] >> b & 1) or "net core off"
                    if masks else names[i] if i < len(names) else str(i))
            out.append(dict(phase=i, name=name, samples=n, battery_samples=nb, avg_ma=round(ua / 1000, 3),
                            vbat_first=v0, vbat_last=v1, ntc_c=None if ntc == -32768 else ntc / 10,
                            cpu_pct=cpu / 10))
        return {"seconds": seconds, "net_fails_at_test": fails_at_test, "net_fails_now": fails_now, "phases": out}

    def idle_switches(self, mask):
        """DIAG SET 7 (0.5.32+, test builds): exactly the idle switches of `mask` on; returns the mask in force."""
        return self.ctrl_ok("DIAG", bytes([5, 7, mask & 0xFF]))[0]

    # ------------------------------------------------ the asset store (VM.md §6.10)

    def asset_info(self):
        info = dict(zip(ASSET_INFO_KEYS, struct.unpack_from("<IIIBBBBI", self.ctrl_ok("ASSET_INFO"))))
        info["state_name"] = ASSET_STATES.get(info["state"], "?")
        return info

    def asset_list(self):
        out, first = [], 0
        while True:
            b = self.ctrl_ok("ASSET_LIST", bytes([first]))
            got = (len(b) - 2) // 32
            for i in range(got):
                e = b[2 + 32 * i:34 + 32 * i]
                offset, size, crc = struct.unpack_from("<III", e, 16)
                out.append({"id": e[:16].hex(), "offset": offset, "size": size, "crc": crc, "type": e[28]})
            first += got
            if got == 0 or first >= b[0]:
                return out

    def asset_wait(self, job, start, timeout=120.0):
        """The end of a store job (1 put, 2 delete, 3 format, 4 crc) started after notice `start`: its NOTICE 8,
        or ASSET_INFO state 4/5 when the notice was lost. Returns (result, the notice's 16 bytes or None)."""
        def found():
            for n in self.notices[start:]:
                b = bytes.fromhex(n["body"])
                if n["kind"] == 8 and len(b) >= 18 and b[0] == job:
                    return b[1], b[2:18]
            return None
        end = self.tr.now() + timeout
        while True:
            try:
                self.run_until(found, min(2.0, max(0.1, end - self.tr.now())), "NOTICE 8")
                return found()
            except TimeoutError:
                if job != 4:
                    info = self.asset_info()
                    if info["state"] in (4, 5) and not found():
                        return info["error"], None
                if self.tr.now() > end:
                    raise

    def asset_put(self, data, typ=4, addr=None, timeout=120.0, wait=True):
        """Stages data at arena address addr (default: allocated here), then ASSET_PUT and its job. Returns the
        result (0 stored, or the error) and the ID; the mirror learns the bytes."""
        data = bytes(data)
        aid = hashlib.sha256(data).digest()[:16]
        if addr is None:
            addr = 4 * self.alloc(len(data))
        self.write(addr, data)
        self.flush()
        self.wait_idle(timeout)
        start = len(self.notices)
        rep = self.ctrl("ASSET_PUT", aid + struct.pack("<IIB", addr, len(data), typ))
        if rep["status"]:
            return rep["status"], aid
        if rep["body"][0] == 1 or not wait:
            self.mirror.set_asset(aid, data)
            return (0 if rep["body"][0] == 1 else None), aid
        result, _ = self.asset_wait(1, start, timeout)
        if result == 0:
            self.mirror.set_asset(aid, data)
        return result, aid

    def asset_delete(self, aid, timeout=60.0):
        start = len(self.notices)
        rep = self.ctrl("ASSET_DELETE", bytes(aid))
        if rep["status"]:
            return rep["status"]
        return self.asset_wait(2, start, timeout)[0]

    def asset_format(self, wipe=False, timeout=60.0):
        """Frees everything; wipe also erases every sector of the store (minutes over stock's old data)."""
        start = len(self.notices)
        rep = self.ctrl("ASSET_FORMAT", struct.pack("<II", FORMAT_MAGIC, WIPE_MAGIC) if wipe else
                        struct.pack("<I", FORMAT_MAGIC))
        if rep["status"]:
            return rep["status"]
        return self.asset_wait(3, start, 1800.0 if wipe else timeout)[0]

    def asset_load(self, aid, addr, off=0, n=None, data=None):
        """ASSET_LOAD in host code: bytes off..off+n-1 of the asset to arena address addr (n: its size, from
        data or the arm's list). With data the mirror knows the bytes; else it logs an error for this load."""
        if data is not None:
            self.mirror.set_asset(aid, data)
        if n is None:
            n = len(data) - off if data is not None else next(
                e["size"] for e in self.asset_list() if e["id"] == bytes(aid).hex()) - off
        self.add(Code().write(SCRATCH + ASSET_ID_AT, bytes(aid)))
        self.ins("ASSET_LOAD", SCRATCH + ASSET_ID_AT, addr, off, n)
        self.flush()

    def flash_crc(self, addr, n, timeout=120.0):
        """FLASH_CRC: crc32 of flash bytes addr..addr+n-1 and how many are not 0xFF (read only). Only the notice
        that names this range counts: a retried request runs its job twice, and the second notice of an earlier
        range may arrive after this request."""
        start = len(self.notices)
        self.ctrl_ok("FLASH_CRC", struct.pack("<II", addr, n))

        def mine():
            for x in self.notices[start:]:
                b = bytes.fromhex(x["body"])
                if x["kind"] == 8 and len(b) >= 18 and b[0] == 4 and struct.unpack_from("<II", b, 2) == (addr, n):
                    return b[1], b[2:18]
            return None
        self.run_until(mine, timeout, f"FLASH_CRC's NOTICE 8 for 0x{addr:x}+0x{n:x}")
        result, b = mine()
        if result:
            raise ControlError(f"{self.name}: FLASH_CRC failed ({result})")
        a, m, crc, set_ = struct.unpack("<IIII", b)
        return {"addr": a, "n": m, "crc": crc, "not_ff": set_}

    def fingerprint(self):
        """The crc32 of each region the store never writes (docs/design-flash-assets.md item 27)."""
        return {f"0x{a:x}+0x{n:x}": self.flash_crc(a, n)["crc"] for a, n in PROTECTED}

    def asset_step(self, value):
        """DIAG SET 10 (test builds): 1 the store pauses after every flash call that changes flash, 2 one step,
        3 abort the job, 0 off."""
        return self.diag_set(10, value)

    def asset_reset_at(self, k):
        """DIAG SET 9 (test builds): the next put resets the arm at checkpoint k (1 after the erase, 2
        mid-program, 3 after the verification, 4 in the directory write), as a power failure would."""
        return self.diag_set(9, k)

    def asset_stats(self):
        """DIAG 18: the last put's step times (µs), counters, the single-step state."""
        b = self.ctrl_ok("DIAG", bytes([18]))
        keys = ("erase_us", "program_us", "verify_us", "dir_us", "longest_erase_us", "erased", "commits", "loads",
                "load_bytes", "check_us", "steps", "wiped")
        out = dict(zip(keys, struct.unpack_from("<12I", b)))
        out.update(paused=b[48], step_mode=b[49], reset_at=b[50], copy=b[51], stack_unused=struct.unpack_from("<I", b, 52)[0])
        return out

    def asset_put_stepped(self, data, typ=4, addr=None, timeout=600.0, log=print):
        """The acceptance run's first put (design item 28): single-stepped, the fingerprints re-read after every
        flash call; any change aborts the job and raises."""
        before = self.fingerprint()
        self.asset_step(1)
        try:
            data = bytes(data)
            aid = hashlib.sha256(data).digest()[:16]
            if addr is None:
                addr = 4 * self.alloc(len(data))
            self.write(addr, data)
            self.flush()
            self.wait_idle(timeout)
            start = len(self.notices)
            rep = self.ctrl("ASSET_PUT", aid + struct.pack("<IIB", addr, len(data), typ))
            if rep["status"] or rep["body"][0] == 1:
                return rep["status"], aid
            last = -1
            while True:
                done = [n for n in self.notices[start:] if n["kind"] == 8 and bytes.fromhex(n["body"])[0] == 1]
                if done:
                    result = bytes.fromhex(done[0]["body"])[1]
                    break
                st = self.asset_stats()
                if not st["paused"] or st["steps"] == last:
                    self.wait(0.05)
                    continue
                last = st["steps"]
                now = self.fingerprint()
                if now != before:
                    self.asset_step(3)
                    raise ControlError(f"{self.name}: a protected region changed after flash call {st['steps']}: "
                                       f"{before} -> {now}")
                log(f"step {st['steps']}: protected regions unchanged")
                self.asset_step(2)
            if result == 0:
                self.mirror.set_asset(aid, data)
            return result, aid
        finally:
            self.asset_step(0)

    def read_log(self):
        out, off, total = bytearray(), 0, None
        while total is None or off < total:
            body = self.ctrl_ok("LOG_READ", struct.pack("<H", off))
            total, _off = struct.unpack_from("<HH", body)
            chunk = body[4:]
            if not chunk:
                break
            out += chunk
            off += len(chunk)
        return out.decode("ascii", "replace")

    def reset_session(self, clear_arena=False):
        """RESET_SESSION: a new epoch, the window back to 0, the program stopped (VM.md §6.4). Returns
        the number of packets the arm dropped."""
        rep = self.ctrl("RESET_SESSION", bytes([1 if clear_arena else 0]), timeout=8.0)
        if rep["status"]:
            raise ControlError(f"RESET_SESSION status {rep['status']}")
        flushed = struct.unpack_from("<H", rep["body"])[0]
        old, clock = self.mirror, (self.samples, self.offset, self.offset_err_us)
        self.epoch = None
        self._apply_hello(rep["body"][2:], rep["epoch"])
        self.samples, self.offset, self.offset_err_us = clock   # a new session, the same device clock
        if old is not None:
            if not clear_arena:
                self.mirror.arena[:] = bytes(old.arena)
            self.mirror.set_panel(old.panel_on())
            self.mirror.panel[:] = bytes(old.panel)
        return flushed

    def link(self, min_int, max_int, latency=0, timeout=400, phy=0):
        body = self.ctrl_ok("LINK", struct.pack("<HHHHB", min_int, max_int, latency, timeout, phy))
        return struct.unpack_from("<bb", body)

    def health_ok(self):
        return self.ctrl_ok("HEALTH_OK")[0]

    def reboot(self, mode=0):
        self.ctrl("REBOOT", bytes([mode]), tries=1)

    def panel_probe(self, mhz, flags=0, delay_ms=16):
        """DIAG PANEL_PROBE (0x09 sub-op 2): rails on as stock, then stock's ID read 9F 00 00 00 in SPI
        modes 0-3 x MISO sample delay 0-7 at `mhz`. flags: 1 CS active high, 2 P0.24 first, 4 LDSW2 on,
        8 stock reset 66/99 first, 16 AB (release deep power-down) first."""
        rep = self.ctrl("DIAG", struct.pack("<BBBH", 2, mhz, flags, delay_ms), timeout=5.0, tries=1)
        b = rep["body"]
        out = {"mhz": mhz, "flags": flags, "delay_ms": delay_ms, "status": rep["status"]}
        if len(b) < 10 + 128:
            return out
        miso = b[4]
        out.update(miso_cs_idle={"pullup": miso & 1, "pulldown": miso >> 1 & 1},
                   miso_cs_active={"pullup": miso >> 2 & 1, "pulldown": miso >> 3 & 1},
                   errors=b[5], ab=b[6:10].hex() if flags & 16 else None, ids={})
        for k in range(32):
            rx = b[10 + 4 * k:14 + 4 * k].hex()
            out["ids"].setdefault(rx, []).append(f"m{k // 8}d{k % 8}")
        return out

    def panel_read(self, y0=0, rows=PANEL_H):
        """PANEL_READ: CRC-32 of panel rows read back over SPI, compared with the mirror's panel."""
        rep = self.ctrl("PANEL_READ", struct.pack("<HH", y0, rows), timeout=5.0, tries=1)
        b = rep["body"]
        out = {"status": rep["status"], "y0": y0}
        if len(b) >= 22:
            crc, n = struct.unpack_from("<IH", b)
            out.update(crc=f"{crc:08x}", rows=n, first16=b[6:22].hex())
        data = self.mirror.panel_rows(y0, rows)
        out.update(mirror_crc=f"{zlib.crc32(data):08x}", mirror_first16=data[:16].hex(),
                   mirror_on=self.mirror.panel_on())
        out["match"] = out.get("crc") == out["mirror_crc"] if self.mirror.panel_on() else rep["status"] == 11
        return out

    def arena_crc(self, addr=0, length=0):
        """ARENA_CRC: CRC-32 of the arm's arena bytes from addr (length 0 = to the end), compared with the
        mirror. Valid once every packet has run (wait_idle first)."""
        rep = self.ctrl("ARENA_CRC", struct.pack("<II", addr, length), timeout=5.0)
        b = rep["body"]
        end = ARENA_SIZE if length == 0 else addr + length
        mcrc = f"{zlib.crc32(bytes(self.mirror.arena[addr:end])):08x}"
        out = {"status": rep["status"], "addr": addr, "bytes": end - addr, "mirror_crc": mcrc}
        if len(b) >= 20:
            out.update(crc=f"{struct.unpack_from('<I', b)[0]:08x}", first16=b[4:20].hex())
        out["match"] = out.get("crc") == mcrc
        return out

    def vm_start(self, table=0x40):
        """VM_START with the trigger table at `table` (send it once the program's packets have run)."""
        return self.ctrl("VM_START", struct.pack("<I", table))["status"]

    def vm_stop(self):
        return self.ctrl("VM_STOP")["status"]

    def vm_status(self):
        body = self.ctrl_ok("VM_STATUS")
        v = dict(zip(VM_STATUS_KEYS, struct.unpack_from(VM_STATUS_FMT, body)))
        v["state_name"] = VM_STATES.get(v["state"], "?")
        v["raw"] = bytes(body[:32])
        return v

    def mic(self, on, rate=0):
        """MIC (VM.md §6.5): returns the reply (status, and the frame format when on)."""
        rep = self.ctrl("MIC", bytes([1 if on else 0, rate]))
        out = {"status": rep["status"]}
        if not rep["status"] and len(rep["body"]) >= 6:
            on_, fb, fpe, rate_hz, ms = struct.unpack_from("<BBBHB", rep["body"])
            out.update(on=bool(on_), frame_bytes=fb, frames_per_event=fpe, sample_rate=rate_hz, frame_ms=ms)
            if len(rep["body"]) >= 8:   # 0.5.12+: the lease a MIC 1 gives (VM.md §6.5)
                out["lease_ms"] = struct.unpack_from("<H", rep["body"], 6)[0]
        return out

    def mic_hold(self, seconds, rate=0):
        """Keep the microphone on for `seconds`: MIC 1 again every 0.8 s renews its lease (VM.md §6.5)."""
        end = self.tr.now() + seconds
        while True:
            left = end - self.tr.now()
            if left <= 0:
                return
            self.wait(min(left, 0.8))
            if end - self.tr.now() > 0:
                self.mic(True, rate)

    def events_mask(self, mask=None):
        body = self.ctrl_ok("EVENTS", b"" if mask is None else struct.pack("<H", mask))
        return struct.unpack_from("<H", body)[0]

    def fault(self, kind):
        """Test builds only. The arm may die before its reply gets out, so do not wait for it."""
        self.tag = (self.tag + 1) & 0xFF
        self.tr.write("G1_CTRL", bytes([0x3F, self.tag, kind]))
        self.pump()

    # -- clock offset (device minus host, microseconds modulo 2^32)
    def ping(self):
        s_us = int(self.tr.now() * 1e6)
        rep = self.ctrl("PING", struct.pack("<I", s_us & M32))
        _host, t_rx, t_tx = struct.unpack_from("<III", rep["body"])
        r_us = int(rep["t_host"] * 1e6)
        d1 = (t_rx - s_us) & M32
        d2 = (t_tx - r_us) & M32
        off = (d1 + signed32(d2 - d1) / 2.0) % (1 << 32)
        delay = (r_us - s_us) - ((t_tx - t_rx) & M32)
        self.samples = (self.samples + [(delay, off)])[-16:]
        best = min(self.samples)
        self.offset, self.offset_err_us = best[1], best[0] / 2.0
        return delay, off

    def sync_clock(self, n=8):
        for _ in range(n):
            self.ping()
        return self.offset_err_us

    def to_device(self, host_s):
        return int(round(host_s * 1e6 + self.offset)) & M32

    def to_host(self, dev_us):
        now_us = self.tr.now() * 1e6
        return (now_us + signed32(int(dev_us - self.offset - now_us))) / 1e6

    def device_now(self):
        return self.to_device(self.tr.now()) if self.offset is not None else 0

    # -- host code in sequenced packets (VM.md §6.2)
    def code_room(self):
        return self.max_pkt - 2

    def add(self, code):
        """Queue host code; an instruction group (a Code) never spans packets: if it does not fit the
        current packet, that one is closed first."""
        b = bytes(code.b if isinstance(code, Code) else code)
        if len(b) > self.code_room():
            raise ValueError(f"{len(b)} bytes of host code do not fit a {self.max_pkt}-byte packet")
        if len(self.cur) + len(b) > self.code_room():
            self.flush()
        self.cur.b += b

    def raw_packet(self, code):
        """Queue a packet of host code as given (tests of the arm's checks); the mirror runs it too."""
        self.flush()
        self._queue_packet(bytes(code))

    def flush(self):
        if len(self.cur):
            self._queue_packet(bytes(self.cur.b))
            self.cur = Code()
        self.pool_pending = set()

    def _queue_packet(self, code):
        seq = self.next_seq
        self.next_seq = (seq + 1) & 0xFFFF
        if self.frame_start_seq is None:
            self.frame_start_seq = seq
        data = struct.pack("<H", seq) + code
        self.outq.append((seq, data))
        self.stats["packets"] += 1
        evs = self.mirror.packet(data, self.device_now())
        for typ, body in evs:
            if typ == 0x02:
                self.log(f"[{self.name}] mirror ERROR {self._error(body)}")
        return seq

    def _sacked(self, seq):
        i = (seq - self.rx_next - 1) & 0xFFFF
        return i < 32 and (self.sack >> i) & 1

    def rto(self):
        """RFC 6298 (VM.md §6.2): SRTT + 4 RTTVAR, at least 4 connection intervals and 100 ms, 1 s before
        the first sample, doubled per resend up to x8."""
        if self.srtt is None:
            base = 1.0
        else:
            base = max(self.srtt + 4 * self.rttvar, max(4 * self.interval_s, 0.1))
        return min(base * self.backoff, 8.0)

    def _send(self, seq, data, now, resend=False):
        self.tr.write("G1_CMD", data)
        ent = self.unacked.get(seq)
        if ent is None:
            self.unacked[seq] = {"data": data, "first": now, "last": now, "resent": False}
            self.sent_at.setdefault(seq, now)
        else:
            ent["last"] = now
            ent["resent"] = ent["resent"] or resend

    def _send_packets(self):
        now = self.tr.now()
        if self.unacked and now - self.progress_t > self.rto():
            seq, ent = next(iter(self.unacked.items()))
            if not self._sacked(seq):
                self.stats["resent"] += 1
                self._send(seq, ent["data"], now, resend=True)
                self.backoff = min(self.backoff * 2, 8)
            self.progress_t = now
        waiting = self.outq or self.exec_next != self.next_seq
        if waiting and not self.unacked and now - self.heard_t > 0.3 and not self._resyncing:
            self.heard_t = now
            self._resyncing = True
            try:
                self.hello()
            finally:
                self._resyncing = False
        limit = (self.rx_next + min(self.credits, self.window)) & 0xFFFF
        sent = 0
        while self.outq and seq_before(self.outq[0][0], limit) and sent < 12:
            seq, data = self.outq.pop(0)
            if not self.unacked:
                self.progress_t = now
            self._send(seq, data, now)
            sent += 1

    def idle(self):
        return not len(self.cur) and not self.outq and not self.unacked and self.exec_next == self.next_seq

    def wait_idle(self, timeout=30.0):
        """Send everything queued and wait until the arm has executed it."""
        self.flush()
        self.run_until(self.idle, timeout, "all packets executed")
        self.frame_start_seq = None

    # -- events from the arm (VM.md §6.3)
    @staticmethod
    def _error(body):
        seq, off, op, code, detail, addr = struct.unpack_from("<HBBBII", body)
        return dict(seq=seq, offset=off, op=op, op_name=OP_NAMES.get(op, "?"), code=code,
                    error=ERRORS.get(code, "?"), detail=detail, address=addr)

    def _on_evt(self, d, t):
        typ = d[0]
        epoch, t_dev = struct.unpack_from("<II", d, 1)
        self.heard_t = t
        if typ & 0x80:
            tag, status = d[9], d[10]
            if typ == 0x81 and len(d) > 11 and d[11] == 4:
                tag = d[1]   # a protocol 4 HELLO reply (tag at byte 1): hello() reports the mismatch
            self.replies[tag] = {"op": typ & 0x7F, "epoch": epoch, "t_dev": t_dev, "status": status,
                                 "body": bytes(d[11:]), "t_host": t}
            return
        if typ == 0x01:
            rx, ex, credits, sack = struct.unpack_from("<HHBI", d, 9)
            if epoch != self.epoch:
                return
            if rx != self.rx_next:
                self.progress_t = t
                self.backoff = 1
            self.rx_next, self.exec_next, self.credits, self.sack = rx, ex, credits, sack
            for seq in list(self.unacked):
                if seq_before(seq, rx):
                    ent = self.unacked.pop(seq)
                    self.acked_at.setdefault(seq, t)
                    if not ent["resent"]:
                        r = t - ent["first"]
                        if self.srtt is None:
                            self.srtt, self.rttvar = r, r / 2
                        else:
                            self.rttvar = 0.75 * self.rttvar + 0.25 * abs(self.srtt - r)
                            self.srtt = 0.875 * self.srtt + 0.125 * r
            if sack:
                top = sack.bit_length()
                for i in range(top):
                    seq = (rx + i) & 0xFFFF
                    hole = i == 0 or not (sack >> (i - 1)) & 1
                    ent = self.unacked.get(seq)
                    if hole and ent and t - ent["last"] > max(0.02, (self.srtt or 0.02)):
                        self.stats["fast_resent"] += 1
                        self._send(seq, ent["data"], t, resend=True)
            for dr in self.draws:
                if not dr["done"] and seq_before(dr["send_seq"], ex):
                    dr.update(done=True, event_lost=dr["t_end"] is None)
            return
        body = bytes(d[9:])
        if typ == 0x08:   # EV_FC_ACK, the display path's (g1fc.py)
            kind, stream, _msg, lens, size, crc = struct.unpack_from("<BBHBHH", body)
            self.fc_acks.append({"kind": kind, "stream": stream, "lens": lens, "size": size, "crc": crc, "t": t})
            return
        if typ == 0x02:
            err = self._error(body)
            err["t"] = t
            self.errors.append(err)
            self.events.append((typ, body))
            self.log(f"[{self.name}] arm ERROR {err}")
        elif typ == 0x03:
            trig, addr, op, code, detail = struct.unpack_from("<BIBBI", body)
            h = dict(trigger=trig, address=addr, op=op, op_name=OP_NAMES.get(op, "?"), code=code,
                     error=ERRORS.get(code, "ok" if code == 0 else "?"), detail=detail, t=t, t_dev=t_dev)
            self.halts.append(h)
            self.events.append((typ, body))
            self.log(f"[{self.name}] program HALT {h}")
        elif typ == 0x04:
            count, src, a, b = struct.unpack_from("<HBII", body)
            n = dict(count=count, source=src, a=a, b=b, t=t, t_dev=t_dev)
            self.notifies.append(n)
            self.events.append((typ, body))
            self._on_notify(n)
        elif typ == 0x05:
            kind = body[0]
            self.notices.append({"kind": kind, "body": body[1:].hex(), "t": t, "t_dev": t_dev})
            self.log(f"[{self.name}] notice {kind} {body[1:].hex()}")
            if kind == 1 and len(body) >= 3:
                self.interval_s = struct.unpack_from("<H", body, 1)[0] * 0.00125
            elif kind == 6 and len(body) >= 2:
                # the case hold (VM.md §6.6, 0.5.32–0.5.58): the panel went off, its image is gone; the mirror's is only right again
                # after the next full redraw (DRAWs during the hold run on the mirror but not on the arm)
                self.case_hold = bool(body[1])
                if self.case_hold and self.mirror:
                    self.mirror.panel[:] = bytes(PANEL_W * PANEL_H)
        elif typ == 0x06:
            first, n = struct.unpack_from("<HB", body)
            self.mic_events += 1
            if self.on_mic:
                self.on_mic(first, n, body[3:], t_dev, t)
        elif typ == 0x07:
            count, trig, value = struct.unpack_from("<HBI", body)
            self.inputs.append(dict(count=count, trigger=trig, value=value, t=t, t_dev=t_dev))

    # -- arena helpers: constants in the pool, word-addressed drawing in the style of protocol v4
    def k(self, value):
        """The pool word that holds `value` (VM.md §2.1: constants are arena words); written into the
        arena first, as its own small group, unless this session wrote it already (the current packet, or
        an earlier one that the mirror shows unchanged). A new session on an arm that kept its epoch does
        not know the arm's arena, so every constant is written once per session."""
        value &= M32
        w = self.pool.get(value)
        if w is None:
            if len(self.pool) >= POOL_WORDS:
                self.pool.clear()
                self.pool_pending.clear()
            w = POOL_BASE + len(self.pool)
            self.pool[value] = w
        if w not in self.pool_pending and (w not in self.pool_written or self.mirror.word(w) != value):
            self.add(Code().write(4 * w, struct.pack("<I", value)))
            self.pool_pending.add(w)
            self.pool_written.add(w)
        return w

    def ins(self, name, *values, target=None):
        """One instruction whose operands are the given values (constants from the pool). A SYS_WRITE of a
        setting the arm owns (GYRO_RATE, AUTO_BRIGHTNESS, PEER_LINK, PEER_SEND) that the arm would refuse, by its
        HELLO.triggers and the PEER_LINK written so far, raises Refused and sends nothing."""
        if name == "SYS_WRITE" and len(values) == 2 and values[0] in OWNED_PROPS and self.mirror is not None:
            self.flush()
            if self.mirror.check_write(values[0], values[1]):
                pname = next(k for k, x in PROPS.items() if x == values[0])
                raise Refused(f"{self.name}: {pname} = {values[1]} refused: this arm does not take it "
                              f"(HELLO.triggers 0x{self.info.get('triggers', 0):04x})")
        words = [self.k(v) for v in values]
        self.add(Code().ins(name, *words, target=target))
        return self

    def alloc(self, nbytes):
        """Bump-allocate a word-aligned arena region from 0x400 up; returns its word address."""
        off = self.arena_top
        self.arena_top = (off + nbytes + 3) & ~3
        if self.arena_top > HOST_TOP:
            raise ValueError("arena exhausted in the host allocator")
        return off // 4

    def write(self, addr, data):
        data = bytes(data)
        step = min(255, self.code_room() - 5)
        for i in range(0, len(data), step):
            self.add(Code().write(addr + i, data[i:i + step]))

    def write_arena(self, aw, data):
        self.write(4 * aw, data)

    def write_arena_rle(self, aw, data):
        packed = packbits(data)
        if len(packed) <= min(255, self.code_room() - 7) and len(data) <= 0xFFFF:
            self.add(Code().write_rle(4 * aw, len(data), packed))
        else:
            self.write_arena(aw, data)

    def write_arena_packbits(self, aw, out_len, packed):
        """One WRITE_RLE of data the caller already PackBits-encoded and sized to one packet."""
        self.add(Code().write_rle(4 * aw, out_len, packed))

    def copy_arena(self, src_aw, dst_aw, nwords):
        self.ins("COPY", 4 * dst_aw, 4 * src_aw, 4 * nwords)

    def fill(self, dst_aw, stride, x, y, w, h, level):
        self.ins("FILL", 4 * dst_aw, stride, x, y, w, h, level)

    def blit(self, src_aw, ss, sx, sy, dst_aw, ds, dx, dy, w, h, fmt=4, mode=COPY, level=15):
        """BLIT into a 4 bpp bitmap; fmt=1: the source is 1 bpp (ink x level)."""
        self.ins("BLIT", 4 * src_aw, ss, sx, sy, 4 * dst_aw, ds, dx, dy, w, h, mode | (ONE_BPP if fmt == 1 else 0), level)

    def line(self, dst_aw, stride, x0, y0, x1, y1, level):
        self.ins("LINE", 4 * dst_aw, stride, x0, y0, x1, y1, level)

    def glyph_run_raw(self, font_aw, dst_aw, ds, x, y, data, mode=MASK, level=15):
        """GLYPHS over the varint bytes given, written to the client's scratch area just before."""
        words = [self.k(v) for v in (4 * font_aw, 4 * dst_aw, ds, x, y, mode, level, SCRATCH, len(data))]
        self.add(Code().write(SCRATCH, data).ins("GLYPHS", *words))

    def glyph_run(self, font_aw, dst_aw, ds, x, y, indices, mode=MASK, level=15):
        self.glyph_run_raw(font_aw, dst_aw, ds, x, y, b"".join(varint(g) for g in indices), mode, level)

    def set_panel(self, v):
        """PANEL: 0 off, 1 on, 2 standby (VM.md §2.5)"""
        self.ins("SYS_WRITE", PROPS["PANEL"], int(v))

    def set_brightness(self, level):
        self.ins("SYS_WRITE", PROPS["BRIGHTNESS"], level)

    def fence(self, token):
        """NOTIFY a = token: arrives once every packet before it has run."""
        self.ins("NOTIFY", token, 0)

    def wait_until(self, at_host):
        self.add(Code().write(SCRATCH + 256, struct.pack("<I", self.to_device(at_host))).ins("SLEEP_UNTIL", SCRATCH // 4 + 64))

    def draw(self, src_aw, stride, dx, dy, w, h, at_host=None, sx=0, sy=0):
        """DRAW the w x h region at pixel (sx, sy) of the 4 bpp bitmap at word address src_aw to the panel at
        (dx, dy), in a packet of its own, between two NOTIFYs (a = 0xD7A00000 + draw number, b = 0 before,
        1 after) whose event times give the DRAW's start and end on the arm."""
        idx = len(self.draws)
        words = [self.k(v) for v in (4 * src_aw, stride, sx, sy, dx, dy, w, h, 0, 1)]
        self.flush()
        c = Code().write(SCRATCH + 260, struct.pack("<I", 0xD7A00000 + (idx & 0xFFFFF)))
        if at_host is not None:
            c.write(SCRATCH + 256, struct.pack("<I", self.to_device(at_host)))
            c.ins("SLEEP_UNTIL", SCRATCH // 4 + 64)
        mark = SCRATCH // 4 + 65
        c.ins("NOTIFY", mark, words[8]).ins("DRAW", *words[:8]).ins("NOTIFY", mark, words[9])
        self.add(c)
        seq = self.next_seq
        self.flush()
        visible = dx < PANEL_W and dx + w > 0
        rows = max(0, min(dy + h, PANEL_H) - max(dy, 0)) if visible else 0
        self.draws.append({"send_seq": seq, "done": False, "panel": bytes(self.mirror.panel), "at_host": at_host,
                           "n_err": len(self.errors), "t_start": None, "t_end": None, "notify": False, "rows": rows,
                           "at_dev": self.to_device(at_host) if at_host is not None else None})
        return idx

    def _on_notify(self, n):
        if n["a"] >> 20 != 0xD7A:
            self.fences[n["a"]] = n
            return
        idx = n["a"] & 0xFFFFF
        if idx < len(self.draws):
            dr = self.draws[idx]
            dr["notify"] = True
            if n["b"] == 0:
                dr["t_start"] = n["t_dev"]
            else:
                dr.update(t_end=n["t_dev"], t_rx=n["t"], done=True)

    def _draw_error(self, dr):
        return next((e for e in self.errors[dr["n_err"]:] if e["seq"] == dr["send_seq"]), None)

    def wait_draw(self, idx, timeout=60.0):
        """Wait until draw idx has run. A DRAW the arm refuses (safe mode, bad arguments) raises."""
        dr = self.draws[idx]
        self.run_until(lambda: dr["done"] or self._draw_error(dr) is not None, timeout, f"draw {idx} done")
        if dr.get("epoch_lost"):
            raise EpochLost(f"draw {idx}: the arm rebooted (new epoch {self.epoch:08x})")
        err = self._draw_error(dr)
        if err:
            raise RuntimeError(f"draw {idx} refused by the arm: {err}")
        return self.draw_timing(idx)

    def draw_timing(self, idx):
        dr = self.draws[idx]
        if dr.get("event_lost") or dr["t_end"] is None or dr["t_start"] is None:
            return {"draw": idx, "rows": dr["rows"], "panel_write_ms": None, "send_to_shown_ms": None,
                    "lateness_ms": None, "event_lost": True}
        t_send = self.sent_at.get(dr["send_seq"])
        t_end = self.to_host(dr["t_end"]) if self.offset is not None else None
        late = None if dr["at_dev"] is None else signed32(dr["t_start"] - dr["at_dev"]) / 1000.0
        t_ack = self.acked_at.get(dr["send_seq"])
        return {"draw": idx, "rows": dr["rows"],
                "send_to_ack_ms": None if (t_ack is None or t_send is None) else (t_ack - t_send) * 1000.0,
                "send_to_done_rx_ms": None if t_send is None else (dr["t_rx"] - t_send) * 1000.0,
                "panel_write_ms": signed32(dr["t_end"] - dr["t_start"]) / 1000.0,
                "send_to_shown_ms": None if (t_end is None or t_send is None) else (t_end - t_send) * 1000.0,
                "lateness_ms": late}

    # -- text on top of glyph runs: font is the Font of the blob at font_aw; returns the pen advance
    def text(self, font_aw, font, dst_aw, ds, x, y, s, level=15, mode=MASK):
        idx, pen, start = font.indices(s), x, 0
        per_run = max(1, (self.code_room() - 5 - 30 - 6 * 9) // 3)
        while start < len(idx):
            run = idx[start:start + per_run]
            if start:
                pen += font.kern(idx[start - 1], run[0])
            self.glyph_run(font_aw, dst_aw, ds, pen, y, run, mode=mode, level=level)
            pen += font.advance(run)
            start += len(run)
        return pen - x


# ---------------------------------------------------------------- emulator checks and pictures

def read_ppm_levels(path):
    data = Path(path).read_bytes()
    fields, i = [], 0
    while len(fields) < 4:
        while data[i] in b" \t\r\n":
            i += 1
        j = i
        while data[j] not in b" \t\r\n":
            j += 1
        fields.append(data[i:j])
        i = j
    i += 1
    w, h = int(fields[1]), int(fields[2])
    if fields[0] != b"P6" or (w, h) != (PANEL_W, PANEL_H):
        raise ValueError("unexpected panel dump")
    rgb = data[i:i + w * h * 3]
    return bytes(rgb[k * 3] // 17 for k in range(w * h))


def save_png(levels, w, h, path, scale=1):
    from PIL import Image
    img = Image.frombytes("L", (w, h), bytes(v * 17 for v in levels))
    if scale != 1:
        img = img.resize((w * scale, h * scale), Image.NEAREST)
    img.save(path)


def emu_check(arm, draw_idx, out_dir, label):
    """Dump the emulated panel, compare it pixel-for-pixel with the mirror's panel at that draw."""
    out_dir = Path(out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    ppm = out_dir / f"{label}.ppm"
    arm.tr.call_model("sysbus.spim_a.jbd", "DumpFrame", str(ppm))
    arm.pump()
    arm.pump()
    got = read_ppm_levels(ppm)
    want = arm.draws[draw_idx]["panel"]
    bad = sum(1 for a, b in zip(got, want) if a != b)
    save_png(got, PANEL_W, PANEL_H, out_dir / f"{label}_panel.png")
    ppm.unlink()
    return bad


# ---------------------------------------------------------------- demo content



def dejavu(name):
    """DejaVu's file: Debian's fonts-dejavu-core, else a font folder of macOS (Homebrew's font-dejavu)."""
    for d in ("/usr/share/fonts/truetype/dejavu", os.path.expanduser("~/Library/Fonts"), "/Library/Fonts"):
        if os.path.exists(os.path.join(d, name)):
            return os.path.join(d, name)
    return os.path.join("/usr/share/fonts/truetype/dejavu", name)


FONT_PATH = dejavu("DejaVuSans.ttf")
FRAME_W, FRAME_H, FRAME_STRIDE = 640, 200, 320   # a 4 bpp frame bitmap the host builds in the arena
PANEL_Y = 160                                      # where the host places it on the 640x480 panel


class Fonts:
    """Upload font blobs (PROTOCOL.md §5) into the arena; the host (bump allocator) decides placement.
    load() returns (word address, Font) for Arm.text()."""

    def __init__(self, arm):
        self.arm = arm
        self.loaded = {}

    def load(self, size, path=FONT_PATH, chars=None):
        return self.load_blob(build_font_blob(path, size, chars), (path, size))

    def load_blob(self, blob, key=None):
        aw = self.arm.alloc(len(blob))
        self.arm.write_arena(aw, blob)
        self.loaded[key or aw] = (aw, Font(blob))
        return self.loaded[key or aw]


def demo_frames(arm, out_dir=None, at_lead=None):
    """Build frames in an arena region with every primitive, draw each to the panel, check pixels."""
    from PIL import Image, ImageDraw
    fonts = Fonts(arm)
    big = fonts.load(28)
    small = fonts.load(16)
    fr = arm.alloc(FRAME_H * FRAME_STRIDE)       # the frame bitmap
    # a 1 bpp image sprite in the arena (byte-strided)
    pic = Image.new("1", (160, 120), 0)
    dd = ImageDraw.Draw(pic)
    for r in range(8, 60, 8):
        dd.ellipse([80 - r, 60 - r, 80 + r, 60 + r], outline=1)
    dd.line([0, 0, 159, 119], fill=1)
    tex, sstride = pack_rows([1 if v else 0 for v in pic.convert("L").tobytes()], 160, 120, 1)
    img = arm.alloc(len(tex))
    arm.write_arena(img, tex)
    arm.flush()
    out = []

    def show():
        at = arm.tr.now() + at_lead if at_lead else None
        idx = arm.draw(fr, FRAME_STRIDE, 0, PANEL_Y, FRAME_W, FRAME_H, at_host=at)
        timing = arm.wait_draw(idx)
        if out_dir and isinstance(arm.tr, EmuTransport):
            timing["pixel_mismatches"] = emu_check(arm, idx, out_dir, f"frame{idx + 1:03d}")
        out.append(timing)

    arm.fill(fr, FRAME_STRIDE, 0, 0, FRAME_W, FRAME_H, 0)
    arm.fill(fr, FRAME_STRIDE, 0, 0, FRAME_W, 2, 15)
    arm.fill(fr, FRAME_STRIDE, 0, FRAME_H - 2, FRAME_W, 2, 15)
    arm.text(*big, fr, FRAME_STRIDE, 20, 40, "G1 protocol 5", level=15)
    arm.text(*small, fr, FRAME_STRIDE, 20, 70, "arena bitmaps -> draw() to the panel", level=12, mode=0)
    show()
    arm.blit(img, sstride, 0, 0, fr, FRAME_STRIDE, 460, 70, 160, 120, fmt=1, mode=1, level=9)
    show()
    arm.copy_arena(fr + 30 * FRAME_STRIDE // 4, fr, (FRAME_H - 30) * FRAME_STRIDE // 4)
    arm.fill(fr, FRAME_STRIDE, 0, FRAME_H - 30, FRAME_W, 30, 0)
    arm.text(*big, fr, FRAME_STRIDE, 20, FRAME_H - 8, "scrolled up 30 rows (copy_arena)", level=10)
    show()
    for k in range(3):
        arm.fill(fr, FRAME_STRIDE, 20, 100, 300, 34, 0)
        arm.text(*big, fr, FRAME_STRIDE, 20, 128, f"counter {k:03d}", level=15)
        show()
    return out


BOLD_PATH = dejavu("DejaVuSans-Bold.ttf")


def text_width(arm, font, s):
    return font[1].width(s)


def grid_frame(arm, label="LEFT", size=15):
    """A full 640x480 frame in the arena: 2 px border (level 15), lines every 40 px (level 7; centre
    lines 11), ticks every 10 px along the edges, x coordinates of the vertical lines along the top and
    bottom, y coordinates of the horizontal lines along the left and right, solid 12x12 corner squares
    and a label in the centre. Composed on the arm with FILL and GLYPH_RUN. Returns its word address."""
    W, H, S = PANEL_W, PANEL_H, PANEL_W // 2
    font = Fonts(arm).load(size, BOLD_PATH)
    fr = arm.alloc(H * S)
    arm.fill(fr, S, 0, 0, W, H, 0)
    for x in range(40, W, 40):
        arm.fill(fr, S, x, 0, 1, H, 11 if x == W // 2 else 7)
    for y in range(40, H, 40):
        arm.fill(fr, S, 0, y, W, 1, 11 if y == H // 2 else 7)
    for v in range(10, W, 10):
        n = 8 if v % 40 == 0 else 4
        arm.fill(fr, S, v, 2, 1, n, 15)
        arm.fill(fr, S, v, H - 2 - n, 1, n, 15)
    for v in range(10, H, 10):
        if v % 40 == 10 or v in (20, H - 50, H - 20):
            continue                                # beside a row label: would read as a minus sign
        n = 8 if v % 40 == 0 else 4
        arm.fill(fr, S, 2, v, n, 1, 15)
        arm.fill(fr, S, W - 2 - n, v, n, 1, 15)
    arm.fill(fr, S, 0, 0, W, 2, 15)
    arm.fill(fr, S, 0, H - 2, W, 2, 15)
    arm.fill(fr, S, 0, 0, 2, H, 15)
    arm.fill(fr, S, W - 2, 0, 2, H, 15)
    for x, y in ((0, 0), (W - 12, 0), (0, H - 12), (W - 12, H - 12)):
        arm.fill(fr, S, x, y, 12, 12, 15)
    for x in range(0, W, 40):
        s = str(x)
        tx = 16 if x == 0 else min(x + 3, W - 4 - text_width(arm, font, s))
        arm.text(*font, fr, S, tx, 28, s)
        arm.text(*font, fr, S, tx, H - 16, s)
    for y in range(40, H, 40):
        s = str(y)
        base = y + 15 if y < H - 40 else y - 3     # the last one above its line, clear of the bottom labels
        arm.text(*font, fr, S, 8, base, s)
        arm.text(*font, fr, S, W - 8 - text_width(arm, font, s), base, s)
    s = f"{label} 640x480"
    arm.fill(fr, S, W // 2 - 70, H // 2 - 26, 140, 22, 0)
    arm.text(*font, fr, S, W // 2 - text_width(arm, font, s) // 2, H // 2 - 9, s)
    return fr


def draw_cases():
    """DRAW cases (source, sx, sy, dx, dy, w, h): odd and even widths, panel and source positions, the
    right and bottom panel edges, clipping on every side, full-width rows. Sources: N 4 bpp noise
    160x120, F the 640x480 frame, W a 656x6 frame (left-clipped full-width rows), S a 1 bpp bitmap
    BLITted into a 4 bpp one at odd x."""
    return [
        ("N", 0, 0, 10, 10, 1, 5), ("N", 1, 0, 21, 10, 2, 5), ("N", 0, 1, 33, 11, 3, 7),
        ("N", 3, 2, 40, 12, 4, 9), ("N", 5, 0, 51, 13, 7, 9), ("N", 0, 0, 64, 20, 8, 8),
        ("N", 2, 3, 77, 21, 33, 17), ("N", 1, 1, 120, 30, 64, 40), ("N", 0, 0, 201, 40, 159, 100),
        ("N", 1, 0, 400, 50, 159, 119),
        ("N", 0, 0, 639, 100, 1, 10), ("N", 1, 0, 638, 120, 2, 10), ("N", 0, 0, 637, 140, 3, 10),
        ("N", 3, 0, 560, 160, 80, 20), ("N", 2, 0, 561, 190, 79, 20), ("N", 0, 0, 600, 220, 60, 20),
        ("N", 1, 0, 597, 250, 61, 20),
        ("N", 0, 0, 300, 470, 30, 10), ("N", 0, 5, 333, 475, 31, 20), ("N", 0, 0, 0, 479, 50, 1),
        ("N", 2, 2, 590, 460, 50, 20), ("N", 1, 1, 601, 465, 51, 30),
        ("N", 0, 0, -1, 300, 20, 10), ("N", 1, 0, -2, 320, 21, 10), ("N", 0, 0, -3, -2, 30, 12),
        ("N", 0, 0, 100, -5, 20, 10),
        ("F", 0, 200, 0, 200, 640, 40), ("F", 0, 479, 0, 479, 640, 1), ("F", 0, 0, 0, 0, 640, 1),
        ("F", 1, 300, 0, 300, 639, 5), ("F", 0, 400, 1, 400, 639, 4), ("F", 0, 410, 2, 410, 638, 4),
        ("F", 0, 50, 0, 470, 640, 10), ("F", 3, 120, 7, 121, 600, 9),
        ("W", 0, 0, -3, 140, 643, 6), ("W", 0, 0, -2, 150, 642, 6), ("W", 1, 0, -16, 160, 655, 6),
        ("S", 1, 0, 151, 330, 77, 33), ("S", 0, 1, 250, 331, 100, 50), ("S", 3, 2, 549, 380, 91, 45),
    ]


def draw_test(arm, out_dir=None, seed=7):
    """The draw_cases() over a background without 0 pixels, each checked against the mirror right after
    it: an emulated panel pixel by pixel, a real one by PANEL_READ (CRC of all 480 rows)."""
    import random
    rnd = random.Random(seed)
    emu = isinstance(arm.tr, EmuTransport)
    arm.arena_top = 0x400
    S2 = PANEL_W // 2
    F, N, M, S, W = (arm.alloc(n) for n in (S2 * PANEL_H, 80 * 120, 15 * 64, 64 * 64, 328 * 6))
    src = {"F": (F, S2), "N": (N, 80), "S": (S, 64), "W": (W, 328)}
    for k in range(80):
        arm.fill(F, S2, 0, k * 6, PANEL_W, 6, k % 15 + 1)
    for k in range(64):
        arm.fill(F, S2, k * 10 + 3, 0, 5, PANEL_H, (k * 7) % 15 + 1)
    arm.write_arena(N, bytes(rnd.randrange(256) for _ in range(80 * 120)))
    arm.write_arena(M, bytes(rnd.randrange(256) for _ in range(15 * 64)))
    arm.fill(S, 64, 0, 0, 128, 64, 5)
    arm.blit(M, 15, 3, 1, S, 64, 1, 0, 117, 60, fmt=1, mode=0, level=12)
    for k in range(41):
        arm.fill(W, 328, k * 16, 0, 16, 6, (k * 4) % 15 + 1)
    arm.fill(W, 328, 1, 0, 3, 6, 15)
    out = {"version": arm.info["version"], "cases": []}

    def check(idx, label):
        if emu:
            ppm = Path(out_dir or ".").resolve() / f"{label}.ppm"
            ppm.parent.mkdir(parents=True, exist_ok=True)
            arm.tr.call_model("sysbus.spim_a.jbd", "DumpFrame", str(ppm))
            arm.pump()
            arm.pump()
            got = read_ppm_levels(ppm)
            ppm.unlink()
            want = arm.draws[idx]["panel"]
            bad = [i for i, (a, b) in enumerate(zip(got, want)) if a != b]
            if bad and out_dir:
                save_png(got, PANEL_W, PANEL_H, Path(out_dir) / f"{label}_panel.png")
            if bad:
                arm.mirror.panel[:] = got          # the next case starts from what the panel shows
            return {"mismatches": len(bad), "first": [(i % PANEL_W, i // PANEL_W) for i in bad[:4]]}
        r = arm.panel_read(0, PANEL_H)
        return {"read_match": r.get("match"), "crc": r.get("crc"), "mirror_crc": r.get("mirror_crc")}

    idx = arm.draw(F, S2, 0, 0, PANEL_W, PANEL_H)
    out["background"] = dict(arm.wait_draw(idx), **check(idx, "background"))
    for k in range(60):
        arm.fill(F, S2, 0, k * 8, PANEL_W, 8, (k * 2 + 3) % 15 + 1)
    for k in range(91):
        arm.fill(F, S2, k * 7 + 2, 0, 3, PANEL_H, (k * 11) % 15 + 1)
    for i, (name, sx, sy, dx, dy, w, h) in enumerate(draw_cases()):
        aw, stride = src[name]
        case = {"case": i, "src": name, "sx": sx, "sy": sy, "dx": dx, "dy": dy, "w": w, "h": h}
        idx = arm.draw(aw, stride, dx, dy, w, h, sx=sx, sy=sy)
        try:
            t = arm.wait_draw(idx)
            case.update(write_ms=t["panel_write_ms"])
        except RuntimeError as ex:
            case["refused"] = str(ex)
        case.update(check(idx, f"case{i:02d}"))
        out["cases"].append(case)
    done = out["cases"]
    out["checked"] = len(done)
    if emu:
        out["mismatches"] = sum(c["mismatches"] for c in done) + out["background"]["mismatches"]
        out["bad_cases"] = [c["case"] for c in done if c["mismatches"]]
        if out_dir:
            save_png(arm.mirror.panel, PANEL_W, PANEL_H, Path(out_dir) / "drawtest_final.png")
    else:
        out["bad_cases"] = [c["case"] for c in done if not c["read_match"]]
        out["all_match"] = out["background"]["read_match"] and not out["bad_cases"]
    return out


def bench(arm, n=40, lead=None):
    """n one-packet text updates: clear a line in the arena frame, draw the glyphs, then draw() only
    that 400x30 band (source pixel (20, 40) of the frame) to the panel. If the arm reboots on the way
    (new epoch after a reconnect), the font and frame are uploaded again and the run goes on."""
    def prepare():
        arm.sync_clock()
        fa, first = Fonts(arm).load(24)
        fr = arm.alloc(FRAME_H * FRAME_STRIDE)
        arm.fill(fr, FRAME_STRIDE, 0, 0, FRAME_W, FRAME_H, 0)
        arm.wait_idle(120)
        return fa, first, fr

    fa, first, fr = prepare()
    epoch = arm.epoch
    out = []
    for k in range(n):
        if arm.epoch != epoch:
            fa, first, fr = prepare()
            epoch = arm.epoch
        arm.fill(fr, FRAME_STRIDE, 20, 40, 400, 30, 0)
        arm.text(fa, first, fr, FRAME_STRIDE, 20, 65, f"update {k:03d}")
        idx = arm.draw(fr, FRAME_STRIDE, 20, PANEL_Y + 40, 400, 30, sx=20, sy=40,
                       at_host=arm.tr.now() + lead if lead else None)
        try:
            out.append(arm.wait_draw(idx))
        except EpochLost as ex:
            out.append({"draw": idx, "epoch_lost": str(ex)})
        arm.wait(0.05)
    return out


def link_test(arm, kb=64, rounds=2):
    """Writes of kb KB of random data at 0x1000: time, KB/s, resends and link losses per round, and the
    region's ARENA_CRC against the mirror."""
    out = []
    for r in range(rounds):
        arm.arena_top = 0x1000
        aw = arm.alloc(kb * 1024)
        t0, l0, s0 = arm.tr.now(), arm.losses(), dict(arm.stats)
        arm.write_arena(aw, os.urandom(kb * 1024))
        arm.wait_idle(600)
        dt = arm.tr.now() - t0
        out.append({"round": r, "s": round(dt, 2), "KBps": round(kb / dt, 1), "losses": arm.losses() - l0,
                    "resent": arm.stats["resent"] - s0["resent"], "fast_resent": arm.stats["fast_resent"] - s0["fast_resent"],
                    "crc_match": arm.arena_crc(4 * aw, kb * 1024)["match"]})
        print(json.dumps(out[-1]), file=sys.stderr, flush=True)
    return out


# ---------------------------------------------------------------- command line

def open_arm(args):
    if args.emu:
        tr = EmuTransport(args.emu)
    else:
        tr = BleTransport(args.ble)
    arm = Arm(tr, name=args.name)
    arm.setup()
    return arm


def assets_cmd(arm, args):
    a = args.args
    if args.action == "info":
        return arm.asset_info()
    if args.action == "list":
        return arm.asset_list()
    if args.action == "fingerprint":
        return arm.fingerprint()
    if args.action == "put":
        data = Path(a[0]).read_bytes()
        if args.step:
            result, aid = arm.asset_put_stepped(data, args.type, args.addr, log=lambda s: print(s, file=sys.stderr))
        else:
            result, aid = arm.asset_put(data, args.type, args.addr)
        return {"id": aid.hex(), "result": result, "error": ERRORS.get(result), "stats": arm.asset_stats()}
    if args.action == "delete":
        return {"result": arm.asset_delete(bytes.fromhex(a[0]))}
    if args.action in ("format", "wipe"):
        return {"result": arm.asset_format(wipe=args.action == "wipe")}
    data = Path(args.file).read_bytes() if args.file else None
    aid, addr = bytes.fromhex(a[0]), int(a[1], 0)
    off = int(a[2], 0) if len(a) > 2 else 0
    n = int(a[3], 0) if len(a) > 3 else None
    arm.asset_load(aid, addr, off, n, data)
    arm.wait_idle(30)
    return {"events": [(t, b.hex()) for t, b in arm.events][-5:]}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    where = ap.add_mutually_exclusive_group(required=True)
    where.add_argument("--emu", help="path of the emulator bridge socket")
    where.add_argument("--ble", help="BLE address of the arm or DK")
    ap.add_argument("--name", default="arm")
    ap.add_argument("--out", default=None, help="directory for PNGs and JSON results")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("hello")
    sub.add_parser("status")
    sub.add_parser("battery", help="the battery properties (BATT_MV, BATT_PCT, CHARGE, IN_CASE, BATT_MA, BATT_TEMP)")
    p = sub.add_parser("props", help="PROPS: read properties by name (VM.md §2.5)")
    p.add_argument("names", nargs="+")
    sub.add_parser("vm-status")
    p = sub.add_parser("vm-start")
    p.add_argument("--table", type=lambda v: int(v, 0), default=0x40)
    sub.add_parser("vm-stop")
    p = sub.add_parser("run", help="one packet of host code given in hex (the mirror runs it too)")
    p.add_argument("hex")
    p = sub.add_parser("mic", help="MIC on/off (needs an encrypted link; the indicator until M5)")
    p.add_argument("state", choices=("on", "off"))
    p = sub.add_parser("linktest", help="link check: write --kb KB of random data --rounds times, KB/s, resends, losses, CRC")
    p.add_argument("--kb", type=int, default=64)
    p.add_argument("--rounds", type=int, default=2)
    sub.add_parser("power", help="power telemetry (DIAG), live values and per-state accumulators")
    sub.add_parser("imu", help="ACC_X/Y/Z, PITCH, ROLL, IMU_RATE")
    sub.add_parser("peer-stats", help="0.5.24+: the arm-to-arm link (DIAG PEER: its counters, the net core's link value) and WORN")
    sub.add_parser("bonds", help="0.5.58+: the arm's bonds, oldest first (DIAG BONDS: address, key flags, aging counter)")
    p = sub.add_parser("touch-watch", help="0.5.18+: print the TouchBar's TOUCH and WEAR INPUT events for a while")
    p.add_argument("seconds", type=float)
    p = sub.add_parser("adc-record", help="0.5.54+: DIAG ADC_BURST: VBAT+IBAT conversions with their times, the "
                       "panel white at --level (the panel's pulse); the guard off meanwhile")
    p.add_argument("--n", type=int, default=64)
    p.add_argument("--wait-us", type=int, default=1000)
    p.add_argument("--level", type=int, action="append", help="levels to record at (default 12 and 42)")
    p = sub.add_parser("panel-guard", help="0.5.52+: PANEL_GUARD: 0 the panel guard off (for measurements; kept "
                       "until 1 or a reboot), 1 on; no value: query; with the cap, the levels and the battery")
    p.add_argument("state", nargs="?", type=int, choices=(0, 1))
    p = sub.add_parser("case-watch", help="0.5.26+: print CASE and WEAR INPUT events (IN_CASE, the lid, the case's charger "
                       "and battery, WORN) for a while")
    p.add_argument("seconds", type=float)
    p = sub.add_parser("gyro-watch", help="0.5.18+: GYRO_RATE 104, then GYR_X/Y/Z (dps) and PITCH/ROLL 5 times a second "
                                          "for a while, GYRO_RATE 0 at the end")
    p.add_argument("seconds", type=float)
    p = sub.add_parser("light-watch", help="0.5.18+: panel on, LIGHT and BRIGHTNESS every second for a while")
    p.add_argument("seconds", type=float)
    p.add_argument("--auto", action="store_true", help="AUTO_BRIGHTNESS 1 meanwhile")
    p = sub.add_parser("pin-probe", help="DIAG PIN_PROBE: which pins the LSM6DSO's INT1/INT2 reach")
    p.add_argument("--pins", help="only these, e.g. P1.04,P1.15")
    p = sub.add_parser("regs", help="DIAG REGS: register pages 0-5 (read only)")
    p.add_argument("pages", nargs="*", type=int, default=[0, 1, 2, 3, 4, 5])
    p = sub.add_parser("diag-set", help="DIAG SET (test builds): touch-rail|ldsw2 0|1, touch-mode 0-3, gpo-pullup 0|1")
    p.add_argument("item", choices=("touch-rail", "ldsw2", "touch-mode", "gpo-pullup"))
    p.add_argument("value", type=int)
    p = sub.add_parser("net-off", help="DIAG NET_OFF (test builds): reboot into a net-core-off measurement")
    p.add_argument("seconds", type=int)
    p.add_argument("phases", type=int)
    sub.add_parser("net-off-result", help="DIAG NET_OFF_RESULT: the stored net-off measurement")
    sub.add_parser("arena-crc", help="CRC of the whole arena on the arm (a fresh session's mirror is zero)")
    sub.add_parser("log")
    p = sub.add_parser("demo")
    p.add_argument("--lead-ms", type=float, default=None, help="present at now + lead (else at once)")
    p = sub.add_parser("text")
    p.add_argument("text")
    p = sub.add_parser("bench", help="latency of one-packet text updates")
    p.add_argument("--n", type=int, default=40)
    p.add_argument("--lead-ms", type=float, default=None)
    p = sub.add_parser("fault", help="test builds: 1 crash, 2 hang (WDT0), 3 panic, 4 thread hang, 5 stop PMIC kicks")
    p.add_argument("kind", type=int)
    p = sub.add_parser("reboot")
    p.add_argument("--mode", type=int, default=0, help="0 normal, 1 clear boot counter, 2 safe mode")
    p = sub.add_parser("smp-list")
    p.add_argument("--hb", type=float, default=0.0, help="stock: NUS heartbeat every HB s")
    p = sub.add_parser("smp-upload", help="upload to slot 1 and verify it with an image list")
    p.add_argument("image")
    p.add_argument("--hb", type=float, default=0.0, help="stock: NUS heartbeat every HB s")
    p.add_argument("--confirm", action="store_true", help="also mark it for install (no reset)")
    p.add_argument("--reset", action="store_true", help="with --confirm: reset into the install")
    p = sub.add_parser("smp-commit", help="slot 1 must hold IMAGE: confirm it, then reset")
    p.add_argument("image")
    p.add_argument("--hb", type=float, default=0.0)
    p.add_argument("--no-reset", action="store_true")
    p = sub.add_parser("panel-probe", help="diagnostic image: ID reads over SPI clock x flags x delay")
    p.add_argument("--mhz", default="32,16,8")
    p.add_argument("--flags", default="0")
    p.add_argument("--delays", default="16")
    p = sub.add_parser("grid", help="full 640x480 grid, drawn at (0,0), then N more full-frame DRAWs")
    p.add_argument("--label", default="LEFT")
    p.add_argument("--redraw", type=int, default=0)
    p.add_argument("--verify", action="store_true", help="0.3.8+: read the panel cache back and compare")
    p = sub.add_parser("drawtest", help="tricky DRAW pattern (odd/even sizes and positions, panel edges, "
                                        "clipping), each DRAW checked by PANEL_READ (emulator: pixel by pixel)")
    p.add_argument("--seed", type=int, default=7)
    sub.add_parser("pair", help="0.5.61+, --ble: read PAIR, which makes CoreBluetooth pair (in the case, see VM.md 6.1)")
    p = sub.add_parser("brightness", help="BRIGHTNESS: stock's level 0..42 (no level: query)")
    p.add_argument("level", type=int, nargs="?")
    p = sub.add_parser("panel", help="PANEL: the panel on (all pixels 0), off, or in standby (fast wake)")
    p.add_argument("state", choices=("on", "off", "standby"))
    p = sub.add_parser("assets", help="0.5.56+: the asset store (VM.md §6.10): info | list | put FILE [--type N] "
                       "[--addr A] [--step] | delete ID | format | wipe | load ID ADDR [OFF N] [--file FILE] | "
                       "fingerprint (the crc32 of 0x0+0x130000 and 0x130000+0x10000, which the store never writes)")
    p.add_argument("action", choices=("info", "list", "put", "delete", "format", "wipe", "load", "fingerprint"))
    p.add_argument("args", nargs="*")
    p.add_argument("--type", type=int, default=4, help="put: 1 font, 2 4-bpp bitmap, 3 1-bpp bitmap, 4 raw")
    p.add_argument("--addr", type=lambda x: int(x, 0), default=None, help="put: the arena address to stage at")
    p.add_argument("--file", help="load: the asset's bytes, so the mirror knows them")
    p.add_argument("--step", action="store_true",
                   help="put, test builds: pause after every flash call and check the fingerprints each time")
    p = sub.add_parser("flash-crc", help="0.5.56+: FLASH_CRC: crc32 of flash bytes ADDR..ADDR+N-1 (read only)")
    p.add_argument("addr", type=lambda x: int(x, 0))
    p.add_argument("n", type=lambda x: int(x, 0))
    p = sub.add_parser("link")
    p.add_argument("min_ms", type=float)
    p.add_argument("max_ms", type=float)
    p.add_argument("--latency", type=int, default=0)
    p.add_argument("--timeout-ms", type=int, default=4000)
    p.add_argument("--phy", type=int, default=2)
    args = ap.parse_args()

    arm = open_arm(args)
    out = {}
    if not args.cmd.startswith("smp-"):
        out["hello"] = arm.hello()
    if args.cmd == "status":
        out["status"] = arm.status()
    elif args.cmd == "battery":
        out["battery"] = arm.battery()
    elif args.cmd == "props":
        out["props"] = arm.props(*[n.upper() if not n.isdigit() else int(n) for n in args.names])
    elif args.cmd == "vm-status":
        out["vm_status"] = arm.vm_status()
    elif args.cmd == "vm-start":
        out["vm_start"] = arm.vm_start(args.table)
    elif args.cmd == "vm-stop":
        out["vm_stop"] = arm.vm_stop()
    elif args.cmd == "run":
        arm.raw_packet(bytes.fromhex(args.hex))
        arm.wait_idle(10)
        arm.wait(0.3)
        out["events"] = [(t, b.hex()) for t, b in arm.events]
    elif args.cmd == "mic":
        out["mic"] = arm.mic(args.state == "on")
    elif args.cmd == "linktest":
        out["rounds"] = link_test(arm, args.kb, args.rounds)
    elif args.cmd == "power":
        out["power"] = arm.power()
        out["power_states"] = arm.power_states()
    elif args.cmd == "imu":
        out["imu"] = arm.imu()
        out["imu_stats"] = arm.imu_stats()
    elif args.cmd == "bonds":
        out["bonds"] = arm.bonds()
    elif args.cmd == "peer-stats":
        out["peer"] = arm.peer_stats()
        out["props"] = arm.props("PEER_LINK", "WORN", "IN_CASE")
    elif args.cmd == "touch-watch":
        out["before"] = arm.touch_stats()
        arm.events_mask(1 << 7 | 1 << 8)
        n0, t0 = len(arm.inputs), arm.tr.now()
        while arm.tr.now() - t0 < args.seconds:
            arm.wait(0.2)
            for x in arm.inputs[n0:]:
                name = {7: "WEAR", 8: "TOUCH"}.get(x["trigger"], x["trigger"])
                value = {1: "tap", 2: "double tap", 3: "triple tap", 4: "long press start", 5: "long press end"}.get(
                    x["value"], x["value"]) if x["trigger"] == 8 else x["value"]
                print(f"{arm.tr.now() - t0:7.2f} s  {name} {value}", flush=True)
            n0 = len(arm.inputs)
        arm.events_mask(0)
        out["after"] = arm.touch_stats()
    elif args.cmd == "adc-record":
        was = arm.panel_guard(None)
        arm.panel_guard(False)
        try:
            buf = arm.alloc(320 * 480)
            arm.fill(buf, 320, 0, 0, 640, 480, 15)
            out["records"] = {}
            for level in args.level or [12, 42]:
                arm.set_brightness(level)
                arm.wait_draw(arm.draw(buf, 320, 0, 0, 640, 480))
                arm.wait(1.5)
                out["records"][level] = arm.adc_record(args.n, args.wait_us)
        finally:
            arm.set_panel(0)
            arm.set_brightness(21)
            arm.flush()
            arm.wait_idle(10)
            arm.panel_guard(was is not False)
    elif args.cmd == "panel-guard":
        out["panel_guard"] = arm.panel_guard(None if args.state is None else bool(args.state))
        out.update(arm.props("BRIGHTNESS", "BRIGHTNESS_MAX", "BRIGHTNESS_CAP", "BATT_MV", "BATT_PCT", "BATT_TEMP"))
    elif args.cmd == "case-watch":
        out["before"] = arm.case_stats()
        arm.events_mask(1 << 6 | 1 << 7)
        n0, t0 = len(arm.inputs), arm.tr.now()
        while arm.tr.now() - t0 < args.seconds:
            arm.wait(0.2)
            for x in arm.inputs[n0:]:
                v = x["value"]
                if x["trigger"] == 6:
                    pct = v >> 8 & 0xFF
                    text = (f"CASE in_case {v & 1} lid_closed {v >> 1 & 1} case_charging {v >> 2 & 1} "
                            f"case_battery {'unknown' if pct == 0xFF else pct}")
                else:
                    text = f"WEAR {v}"
                print(f"{arm.tr.now() - t0:7.2f} s  {text}", flush=True)
            n0 = len(arm.inputs)
        arm.events_mask(0)
        out["after"] = arm.case_stats()
    elif args.cmd == "gyro-watch":
        arm.ins("SYS_WRITE", PROPS["GYRO_RATE"], 104)
        arm.wait_idle(10)
        t0 = arm.tr.now()
        while arm.tr.now() - t0 < args.seconds:
            arm.wait(0.2)
            v = arm.props("GYR_X", "GYR_Y", "GYR_Z", "PITCH", "ROLL")
            dps = " ".join("-" if v[k] is None else f"{v[k] / 1000:.1f}" for k in ("GYR_X", "GYR_Y", "GYR_Z"))
            print(f"{arm.tr.now() - t0:7.1f} s  GYR {dps} dps  PITCH {v['PITCH']}  ROLL {v['ROLL']}", flush=True)
        arm.ins("SYS_WRITE", PROPS["GYRO_RATE"], 0)
        arm.wait_idle(10)
    elif args.cmd == "light-watch":
        arm.set_panel(1)
        if args.auto:
            arm.ins("SYS_WRITE", PROPS["AUTO_BRIGHTNESS"], 1)
        arm.wait_idle(10)
        arm.events_mask(1 << 4)
        t0 = arm.tr.now()
        while arm.tr.now() - t0 < args.seconds:
            arm.wait(1.0)
            v = arm.props("LIGHT", "BRIGHTNESS")
            lux = None if v["LIGHT"] is None else v["LIGHT"] / 100
            print(f"{arm.tr.now() - t0:7.1f} s  {lux} lux  BRIGHTNESS {v['BRIGHTNESS']}", flush=True)
        arm.events_mask(0)
        arm.ins("SYS_WRITE", PROPS["AUTO_BRIGHTNESS"], 0)
        arm.set_panel(0)
        arm.wait_idle(10)
        out["light_stats"] = arm.light_stats()
    elif args.cmd == "pin-probe":
        masks = [0, 0]
        for pin in (args.pins.split(",") if args.pins else []):
            port, num = pin[1:].split(".")
            masks[int(port)] |= 1 << int(num)
        out["pin_probe"] = arm.pin_probe(*masks) if args.pins else arm.pin_probe()
    elif args.cmd == "regs":
        out["regs"] = {p: arm.regs(p) for p in args.pages}
    elif args.cmd == "diag-set":
        out["diag_set"] = arm.diag_set(("touch-rail", "ldsw2", "touch-mode", "gpo-pullup").index(args.item) + 1, args.value)
    elif args.cmd == "net-off":
        out["net_off"] = arm.net_off(args.seconds, args.phases).hex()
    elif args.cmd == "net-off-result":
        out["net_off_result"] = arm.net_off_result()
    elif args.cmd == "panel":
        arm.set_panel({"off": 0, "on": 1, "standby": 2}[args.state])
        arm.wait_idle(10)
        out["hello_after"] = arm.hello()
    elif args.cmd == "arena-crc":
        out["arena_crc"] = arm.arena_crc()
    elif args.cmd == "log":
        print(arm.read_log())
    elif args.cmd == "demo":
        arm.sync_clock()
        lead = args.lead_ms / 1000.0 if args.lead_ms else None
        out["draws"] = demo_frames(arm, args.out, at_lead=lead)
    elif args.cmd == "text":
        fonts = Fonts(arm)
        fa, first = fonts.load(28)
        fr = arm.alloc(FRAME_H * FRAME_STRIDE)
        arm.fill(fr, FRAME_STRIDE, 0, 0, FRAME_W, FRAME_H, 0)
        arm.text(fa, first, fr, FRAME_STRIDE, 20, 60, args.text)
        idx = arm.draw(fr, FRAME_STRIDE, 0, PANEL_Y, FRAME_W, FRAME_H)
        out["draw"] = arm.wait_draw(idx)
    elif args.cmd == "bench":
        out["draws"] = bench(arm, args.n, args.lead_ms / 1000.0 if args.lead_ms else None)
        for key in ("send_to_ack_ms", "send_to_shown_ms", "send_to_done_rx_ms"):
            vals = sorted(f[key] for f in out["draws"] if f.get(key) is not None)
            if vals:
                out[key] = {"min": round(vals[0], 1), "median": round(vals[len(vals) // 2], 1),
                            "max": round(vals[-1], 1), "n": len(vals)}
    elif args.cmd == "panel-probe":
        out["probes"] = []
        for mhz in (int(x) for x in args.mhz.split(",")):
            for flags in (int(x, 0) for x in args.flags.split(",")):
                for delay in (int(x) for x in args.delays.split(",")):
                    r = arm.panel_probe(mhz, flags, delay)
                    out["probes"].append(r)
                    ids = {k: (v if len(v) < 32 else "all") for k, v in r.get("ids", {}).items()}
                    print(f"{mhz:2d} MHz flags {flags:#04x} delay {delay:4d} ms status {r['status']} "
                          f"err {r.get('errors')} miso idle {r.get('miso_cs_idle')} active {r.get('miso_cs_active')} "
                          f"ab {r.get('ab')} ids {ids}", file=sys.stderr, flush=True)
        out["hello_after"] = arm.hello()
    elif args.cmd == "grid":
        arm.sync_clock()
        fr = grid_frame(arm, args.label)
        out["draws"] = []
        for _ in range(1 + args.redraw):
            idx = arm.draw(fr, PANEL_W // 2, 0, 0, PANEL_W, PANEL_H)
            out["draws"].append(arm.wait_draw(idx))
            arm.wait(0.3)
        if args.out and isinstance(arm.tr, EmuTransport):
            out["pixel_mismatches"] = emu_check(arm, idx, args.out, "grid")
        if args.verify:
            out["panel_read"] = arm.panel_read(0, PANEL_H)
        ws = [d["panel_write_ms"] for d in out["draws"] if d.get("panel_write_ms") is not None]
        if ws:
            out["panel_write_ms"] = {"min": min(ws), "max": max(ws), "n": len(ws)}
    elif args.cmd == "drawtest":
        r = draw_test(arm, args.out, seed=args.seed)
        out["drawtest"] = r
        print(f"drawtest {r['version']}: {r['checked']} DRAWs checked, bad cases {r['bad_cases']}"
              + (f", mismatches {r['mismatches']}" if "mismatches" in r else f", all match {r['all_match']}"),
              file=sys.stderr, flush=True)
    elif args.cmd == "pair":
        if not arm.tr.has("G1_PAIR"):
            raise SystemExit("no PAIR characteristic: g1v0 0.5.61+ over --ble (or a stale GATT cache)")
        out["pair_read"] = arm.tr.run(arm.tr.client.read_gatt_char(arm.tr.chars["G1_PAIR"]), timeout=90).hex()
        arm.wait(1.0)
        out["hello_after"] = arm.hello()
    elif args.cmd == "brightness":
        if args.level is not None:
            arm.set_brightness(args.level)
            arm.wait_idle(10)
        out["brightness"] = arm.props("BRIGHTNESS", "PANEL")
    elif args.cmd == "fault":
        arm.fault(args.kind)
        arm.wait(1.0)
    elif args.cmd == "reboot":
        arm.reboot(args.mode)
    elif args.cmd == "smp-list":
        arm.hb_every = args.hb
        out["images"] = smp_images(arm.smp)
    elif args.cmd == "smp-upload":
        arm.hb_every = args.hb
        img = Path(args.image).read_bytes()
        out["image"] = {"file": args.image, "sha256": hashlib.sha256(img).hexdigest(), "bytes": len(img),
                        "image_hash": image_hash(img).hex()}
        last = [0.0]

        def prog(off, total):
            if arm.tr.now() - last[0] > 15 or off == total:
                last[0] = arm.tr.now()
                print(f"upload {off}/{total}", file=sys.stderr, flush=True)
        out["upload"] = smp_upload_verified(arm, img, progress=prog)
        if args.confirm:
            out["commit"] = smp_commit(arm, img, reset=args.reset)
    elif args.cmd == "smp-commit":
        arm.hb_every = args.hb
        img = Path(args.image).read_bytes()
        out["commit"] = smp_commit(arm, img, reset=not args.no_reset)
    elif args.cmd == "assets":
        out["assets"] = assets_cmd(arm, args)
    elif args.cmd == "flash-crc":
        out["flash_crc"] = arm.flash_crc(args.addr, args.n)
    elif args.cmd == "link":
        out["link"] = arm.link(round(args.min_ms / 1.25), round(args.max_ms / 1.25), args.latency,
                               args.timeout_ms // 10, args.phy)
        arm.wait(2.0)
    if args.cmd in ("bench", "grid"):
        out["clock_offset_err_ms"] = None if arm.offset_err_us is None else arm.offset_err_us / 1000.0
        out["link_at_end"] = {k: v for k, v in arm.status().items() if k in ("interval", "latency", "timeout", "phy", "mtu")}
    out["session"] = dict(arm.stats, losses=arm.losses(), reconnects=getattr(arm.tr, "reconnects", None))
    print(json.dumps(out, indent=1, default=lambda o: o.hex() if isinstance(o, bytes) else str(o)))
    if args.out:
        Path(args.out).mkdir(parents=True, exist_ok=True)
        Path(args.out, f"{args.cmd}.json").write_text(json.dumps(out, indent=1, default=str))
    arm.tr.close()


if __name__ == "__main__":
    main()

