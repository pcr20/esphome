"""Python port of the modbus_sniffer framing, pairing and statistics logic, run over IO Ninja captures.

Usage:
    python sniffer_check.py CAPTURE.txt [...]   run the parser over captures and print its statistics
    python sniffer_check.py --selftest CAPTURE.txt
        damage a clean capture in known ways (bit flips, lost bytes, frames with unknown function
        codes, noise) and check that the statistics count exactly that damage

Bytes are fed in random-sized chunks (as UART reads would deliver them) to check that framing
does not depend on chunk boundaries. Keep this in step with modbus_sniffer.cpp.
"""

import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from busmap import crc16, crc_ok, read_bytes  # noqa: E402

MAX_FRAME = 256
KNOWN = {1, 2, 3, 4, 5, 6, 0x0F, 0x10}


def crc_frame_length(data, start, size):
    """Length of the shortest CRC-valid frame (address <= 247) at data[start:start+size], else 0."""
    if size < 4 or data[start] > 247:
        return 0
    for n in range(4, min(size, MAX_FRAME) + 1):
        if crc_ok(bytes(data[start:start + n])):
            return n
    return 0


def split_crc_frames(data):
    count = 0
    off = 0
    while off < len(data):
        n = crc_frame_length(data, off, len(data) - off)
        if n == 0:
            return 0
        off += n
        count += 1
    return count


class Sniffer:
    def __init__(self):
        self.buf = bytearray()
        self.pending = None
        self.blocks = {}
        self.addr = {}
        self.seen = {}  # address -> function codes seen in valid frames
        self.rx = 0
        self.frames = 0
        self.skipped = 0
        self.orphans = 0
        self.crc_errors = 0
        self.unknown = 0
        self.unknown_list = []
        self.noise_runs = 0
        self.runs = []  # (bytes, presumed frames, cut short) for every bad run
        self.run = bytearray()
        self.run_bytes = 0
        self.run_frames = 0
        self.walk = 0

    def stats(self, a):
        return self.addr.setdefault(a, {"req": 0, "rep": 0, "unans": 0, "exc": 0})

    def feed(self, data):
        self.rx += len(data)
        self.buf += data
        self.parse(False)

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

    def presumed_length(self):
        b = self.buf
        size = len(b)
        if size < 2 or b[0] > 247:
            return 0
        fc = b[1]
        base = fc & 0x7F
        # Only an address and function already seen together in a valid frame make a convincing header.
        if base not in KNOWN or base not in self.seen.get(b[0], ()):
            return 0
        if fc & 0x80:
            return 5
        p = self.pending
        reply = p is not None and p["a"] == b[0] and p["fc"] == base
        n = 8
        if base <= 4:
            if reply and size >= 3:
                n = 5 + b[2]
        elif base in (0x0F, 0x10):
            if not reply and size >= 7:
                n = 9 + b[6]
        return min(n, MAX_FRAME)

    def skip_byte(self):
        if self.walk > 0:
            self.walk -= 1
        else:
            n = self.presumed_length()
            if n > 0:
                self.run_frames += 1
                self.walk = n - 1
        if len(self.run) < 2 * MAX_FRAME:
            self.run.append(self.buf[0])
        self.run_bytes += 1
        self.skipped += 1
        del self.buf[0]

    def end_run(self):
        if self.run_bytes == 0:
            return
        unknown = split_crc_frames(self.run) if len(self.run) == self.run_bytes else 0
        if unknown:
            self.unknown += unknown
            off = 0
            while off < len(self.run):
                n = crc_frame_length(self.run, off, len(self.run) - off)
                self.unknown_list.append(bytes(self.run[off:off + n]))
                off += n
        else:
            self.crc_errors += self.run_frames
            if self.run_frames == 0:
                self.noise_runs += 1
            self.runs.append((self.run_bytes, self.run_frames, self.walk > 0))
        self.run = bytearray()
        self.run_bytes = self.run_frames = self.walk = 0

    def parse(self, flush):
        while self.buf:
            n = self.frame_length(flush)
            if n == 0:
                break
            if n < 0:
                self.skip_byte()
                continue
            self.end_run()
            self.handle(bytes(self.buf[:n]))
            del self.buf[:n]
        if flush:
            self.end_run()

    def handle(self, f):
        self.frames += 1
        a, fc = f[0], f[1]
        base = fc & 0x7F
        self.stats(a)
        self.seen.setdefault(a, set()).add(base)
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

    def summary(self):
        bad = self.crc_errors + self.unknown
        good = 100 * (self.rx - self.skipped) / self.rx if self.rx else 100
        rate = 100 * bad / (self.frames + bad) if self.frames + bad else 0
        return (f"{self.rx} bytes, {good:.3f}% in {self.frames} valid frames; {self.crc_errors} CRC errors, "
                f"{self.unknown} unknown frames ({rate:.3f}% of frames), {self.skipped} skipped bytes, "
                f"{self.orphans} orphan replies, {self.noise_runs} runs of noise")


def run_stream(data, seed=1):
    rng = random.Random(seed)
    s = Sniffer()
    i = 0
    while i < len(data):
        n = rng.randint(1, 64)
        s.feed(data[i:i + n])
        i += n
    s.parse(True)
    return s


def report(path):
    data, _ = read_bytes(path)
    s = run_stream(data)
    print(f"== {os.path.basename(path)}")
    print(f"   {s.summary()}")
    for a, st in sorted(s.addr.items()):
        print(f"   addr {a}: {st}")
    for (a, fc, start), blk in sorted(s.blocks.items()):
        print(f"   block addr={a} fc=0x{fc:02x} start=0x{start:04x} regs={len(blk['regs'])} updates={blk['updates']}")
    for n, frames, cut in s.runs:
        print(f"   bad run: {n} bytes, {frames} frames with CRC errors{', last cut short' if cut else ''}")
    for f in s.unknown_list[:20]:
        print(f"   unknown frame: {f.hex(' ')}")


def frame_spans(data):
    """Offsets and lengths of the valid frames of a clean capture."""
    s = Sniffer()
    s.buf = bytearray(data)
    spans = []
    off = 0
    while s.buf:
        n = s.frame_length(True)
        assert n > 0, f"capture is not clean at byte {off}"
        spans.append((off, n))
        s.handle(bytes(s.buf[:n]))
        del s.buf[:n]
        off += n
    return spans


def with_crc(body):
    c = crc16(body)
    return bytes(body) + bytes([c & 0xFF, c >> 8])


def selftest(path):
    data, _ = read_bytes(path)
    spans = frame_spans(data)
    rng = random.Random(42)
    total = len(spans)
    # Choose distinct frames to damage, never two neighbours (each damage must stay its own run).
    chosen = sorted(rng.sample(range(1, total - 1), 400))
    picked = []
    for i in chosen:
        if not picked or i - picked[-1] >= 3:
            picked.append(i)
    kinds = ["flip", "drop", "unknown", "noise", "unknown_pair"]
    plan = {i: kinds[k % len(kinds)] for k, i in enumerate(picked)}
    out = bytearray()
    expect = {"flip": 0, "drop": 0, "unknown": 0, "noise": 0}
    expected_unknown_frames = []
    for idx, (off, n) in enumerate(spans):
        frame = bytearray(data[off:off + n])
        kind = plan.get(idx)
        if kind == "flip":
            # Flip one bit outside the header bytes that set the length, so the walk keeps its length.
            pos = rng.randrange(7, n) if n > 8 else rng.randrange(2, n)
            frame[pos] ^= 1 << rng.randrange(8)
            expect["flip"] += 1
        elif kind == "drop":
            # Lose bytes from the middle of the frame, as a UART overrun would.
            pos = rng.randrange(3, n - 2)
            del frame[pos:pos + rng.randint(1, min(4, n - pos - 2))]
            expect["drop"] += 1
        elif kind in ("unknown", "unknown_pair"):
            # A valid frame of an unknown function code (0x2B, 0x08, 0x41..) goes before the real frame.
            for _ in range(2 if kind == "unknown_pair" else 1):
                while True:
                    fc = rng.choice([0x08, 0x11, 0x14, 0x17, 0x2B, 0x41, 0x64])
                    body = bytes([rng.randint(1, 247), fc]) + bytes(rng.randrange(256) for _ in range(rng.randint(2, 30)))
                    unk = with_crc(body)
                    # A frame with a shorter CRC-valid prefix (odds 1 in 65 536 per length) cannot be
                    # told apart from two frames, by the sniffer or anyone; leave those out of the test.
                    if crc_frame_length(unk, 0, len(unk)) == len(unk):
                        break
                expected_unknown_frames.append(unk)
                out += unk
        elif kind == "noise":
            # Line noise: bytes that cannot start a frame (address > 247), before the real frame.
            out += bytes(rng.randint(248, 255) for _ in range(rng.randint(1, 6)))
            expect["noise"] += 1
        out += frame
    s = run_stream(bytes(out), seed=7)
    bad_frames = expect["flip"] + expect["drop"]
    print(f"== selftest on {os.path.basename(path)}: {total} frames; damaged {expect['flip']} by a bit flip, "
          f"{expect['drop']} by lost bytes; added {len(expected_unknown_frames)} unknown frames and "
          f"{expect['noise']} runs of noise")
    print(f"   {s.summary()}")
    checks = [
        ("valid frames", s.frames, total - bad_frames),
        ("CRC errors", s.crc_errors, bad_frames),
        ("unknown frames", s.unknown, len(expected_unknown_frames)),
        ("unknown frames are the injected ones", s.unknown_list == expected_unknown_frames, True),
        ("runs of noise", s.noise_runs, expect["noise"]),
        ("byte accounting: rx = frame bytes + skipped", s.rx,
         sum(n for i, (_, n) in enumerate(spans) if plan.get(i) not in ("flip", "drop")) + s.skipped),
    ]
    ok = True
    for name, got, want in checks:
        status = "ok" if got == want else "FAIL"
        ok &= got == want
        print(f"   {status:4s} {name}: got {got}, expected {want}")
    return ok


if __name__ == "__main__":
    if sys.argv[1:2] == ["--selftest"]:
        sys.exit(0 if all([selftest(p) for p in sys.argv[2:]]) else 1)
    for p in sys.argv[1:]:
        report(p)
