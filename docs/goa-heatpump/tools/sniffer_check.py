"""Python port of the modbus_sniffer framing and pairing logic, run over IO Ninja captures.

Usage: python sniffer_check.py CAPTURE.txt [...]

Bytes are fed in random-sized chunks (as UART reads would deliver them) to check that framing
does not depend on chunk boundaries. Reports frames, skipped bytes, per-address stats and blocks.
"""

import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from busmap import crc_ok, read_bytes  # noqa: E402

MAX_FRAME = 256


class Sniffer:
    def __init__(self):
        self.buf = bytearray()
        self.pending = None
        self.blocks = {}
        self.addr = {}
        self.frames = 0
        self.skipped = 0
        self.orphans = 0

    def stats(self, a):
        return self.addr.setdefault(a, {"req": 0, "rep": 0, "unans": 0, "exc": 0})

    def frame_length(self, flush):
        b = self.buf
        size = len(b)
        if size < 4:
            return -1 if flush else 0
        if b[0] > 247:
            return -1
        fc = b[1]
        base = fc & 0x7F
        cands = []
        unknown = False
        if fc & 0x80:
            cands = [5]
        elif 1 <= base <= 4:
            cands = [8]
            r = 5 + b[2]
            if r != 8:
                cands.append(r)
        elif base in (5, 6):
            cands = [8]
        elif base in (0x0F, 0x10):
            cands = [8]
            if size >= 7:
                cands.append(9 + b[6])
            else:
                unknown = True
        else:
            return -1
        cands.sort()
        for n in cands:
            if n > MAX_FRAME:
                continue
            if size < n:
                if flush:
                    continue
                return 0
            if crc_ok(bytes(b[:n])):
                return n
        if unknown and not flush:
            return 0
        return -1

    def parse(self, flush):
        while self.buf:
            n = self.frame_length(flush)
            if n == 0:
                break
            if n < 0:
                del self.buf[0]
                self.skipped += 1
                continue
            self.handle(bytes(self.buf[:n]))
            del self.buf[:n]

    def handle(self, f):
        self.frames += 1
        a, fc = f[0], f[1]
        base = fc & 0x7F
        p = self.pending
        match = p is not None and p["a"] == a and p["fc"] == base
        if fc & 0x80:
            reply = True
        elif 1 <= base <= 4:
            req_ok = len(f) == 8
            rep_ok = len(f) == 5 + f[2]
            reply = match if (req_ok and rep_ok) else rep_ok
        elif base in (5, 6):
            reply = match and p["start"] == (f[2] << 8 | f[3])
        else:
            reply = len(f) == 8
        (self.reply if reply else self.request)(f)

    def request(self, f):
        a, fc = f[0], f[1]
        start = f[2] << 8 | f[3]
        if self.pending is not None:
            self.stats(self.pending["a"])["unans"] += 1
            self.pending = None
        self.stats(a)["req"] += 1
        if fc == 0x10:
            self.update(a, fc, start, f[7:7 + f[6]])
        elif fc == 0x06:
            self.update(a, fc, start, f[4:6])
        if a != 0:
            self.pending = {"a": a, "fc": fc, "start": start}

    def reply(self, f):
        a, fc = f[0], f[1]
        base = fc & 0x7F
        p = self.pending
        if p is None or p["a"] != a or p["fc"] != base:
            self.orphans += 1
            return
        self.pending = None
        s = self.stats(a)
        s["rep"] += 1
        if fc & 0x80:
            s["exc"] += 1
            return
        if base in (3, 4):
            self.update(a, base, p["start"], f[3:3 + f[2]])

    def update(self, a, fc, start, data):
        regs = [data[i] << 8 | data[i + 1] for i in range(0, len(data) - 1, 2)]
        blk = self.blocks.setdefault((a, fc, start), {"regs": [], "updates": 0})
        if len(blk["regs"]) < len(regs):
            blk["regs"] += [0] * (len(regs) - len(blk["regs"]))
        blk["regs"][: len(regs)] = regs
        blk["updates"] += 1


def run(path):
    data, _ = read_bytes(path)
    rng = random.Random(1)
    s = Sniffer()
    i = 0
    while i < len(data):
        n = rng.randint(1, 64)
        s.buf += data[i:i + n]
        i += n
        s.parse(False)
    s.parse(True)
    print(f"== {path.rsplit(chr(92), 1)[-1]}: {len(data)} bytes, {s.frames} frames, "
          f"{s.skipped} skipped bytes, {s.orphans} orphan replies")
    for a, st in sorted(s.addr.items()):
        print(f"   addr {a}: {st}")
    for (a, fc, start), blk in sorted(s.blocks.items()):
        r = blk["regs"]
        serial = "".join(chr(v >> 8) + chr(v & 0xFF) for v in r[:6])
        print(f"   block addr={a} fc=0x{fc:02x} start=0x{start:04x} regs={len(r)} updates={blk['updates']} "
              f"[9]=0x{r[9]:04x} serial={serial!r}" if len(r) > 9 else
              f"   block addr={a} fc=0x{fc:02x} start=0x{start:04x} regs={len(r)} updates={blk['updates']}")


for p in sys.argv[1:]:
    run(p)

