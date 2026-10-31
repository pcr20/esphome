"""Parse IO Ninja text exports of the Goa heat-pump RS-485 bus into Modbus frames and summarise them.

Framing ignores IO Ninja's own chunking: all bytes are concatenated and frames are found by
function-code length rules plus CRC, so address 0 and merged/split chunks are handled.
"""

from __future__ import annotations

import collections
import json
import re
import sys

TS_LINE = re.compile(r"^(\d\d):(\d\d):(\d\d) \+(\d+):(\d+\.\d+) > ([0-9a-f]{4})  ((?:[0-9a-f]{2} )+)")
CONT_LINE = re.compile(r"^\s+> ([0-9a-f]{4})  ((?:[0-9a-f]{2} )+)")
PLAIN_LINE = re.compile(r"^((?:[0-9a-f]{2} )+)\s")


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def crc_ok(frame: bytes) -> bool:
    return len(frame) >= 4 and crc16(frame[:-2]) == (frame[-2] | frame[-1] << 8)


def read_bytes(path: str) -> tuple[bytes, list[float | None]]:
    """Return the byte stream and a per-byte timestamp (seconds since capture start, or None)."""
    data = bytearray()
    times: list[float | None] = []
    t: float | None = None
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if m := TS_LINE.match(line):
                t = int(m.group(4)) * 60 + float(m.group(5))
                hexpart = m.group(7)
            elif m := CONT_LINE.match(line):
                hexpart = m.group(2)
            elif m := PLAIN_LINE.match(line):
                hexpart = m.group(1)
            else:
                continue
            chunk = bytes.fromhex(hexpart)
            data += chunk
            times += [t] * len(chunk)
    return bytes(data), times


def frame_candidates(buf: bytes, i: int) -> list[tuple[int, str]]:
    if i + 4 > len(buf):
        return []
    fc = buf[i + 1]
    c: list[tuple[int, str]] = []
    if fc in (3, 4):
        c.append((8, "req"))
        if i + 2 < len(buf):
            c.append((5 + buf[i + 2], "resp"))
    elif fc in (0x10, 0x0F):
        if i + 6 < len(buf):
            c.append((9 + buf[i + 6], "req"))
        c.append((8, "resp"))
    elif fc in (5, 6):
        c.append((8, "req/resp"))
    elif fc & 0x80:
        c.append((5, "exc"))
    return c


def split_frames(buf: bytes, times: list[float | None]):
    frames = []
    junk = 0
    i = 0
    while i < len(buf):
        for length, kind in frame_candidates(buf, i):
            fr = buf[i : i + length]
            if len(fr) == length and crc_ok(fr):
                frames.append({"t": times[i], "raw": fr, "kind": kind})
                i += length
                break
        else:
            junk += 1
            i += 1
    return frames, junk


def regs(payload: bytes) -> list[int]:
    return [payload[k] << 8 | payload[k + 1] for k in range(0, len(payload) - 1, 2)]


def analyse(path: str) -> dict:
    buf, times = read_bytes(path)
    frames, junk = split_frames(buf, times)
    out: dict = {"file": path, "bytes": len(buf), "frames": len(frames), "unframed_bytes": junk}

    # Pair requests with responses.
    transactions = []
    pending = None
    for fr in frames:
        raw, kind = fr["raw"], fr["kind"]
        addr, fc = raw[0], raw[1]
        if kind == "req":
            if pending is not None:
                transactions.append({**pending, "reply": None})
            start = raw[2] << 8 | raw[3]
            count = raw[4] << 8 | raw[5]
            values = regs(raw[7:-2]) if fc == 0x10 else None
            pending = {"t": fr["t"], "addr": addr, "fc": fc, "start": start, "count": count, "write_values": values}
            if addr == 0:  # broadcast: never answered
                transactions.append({**pending, "reply": None})
                pending = None
        else:
            if pending is not None and pending["addr"] == addr and (pending["fc"] == fc or fc == pending["fc"] | 0x80):
                reply = {"t": fr["t"], "exception": raw[2] if fc & 0x80 else None}
                if fc in (3, 4):
                    reply["values"] = regs(raw[3:-2])
                transactions.append({**pending, "reply": reply})
                pending = None
            else:
                transactions.append({"t": fr["t"], "addr": addr, "fc": fc, "orphan_reply": raw.hex(" ")})
    if pending is not None:
        transactions.append({**pending, "reply": None})

    # Exchange types and counts.
    kinds = collections.Counter()
    for tr in transactions:
        if "orphan_reply" in tr:
            kinds[f"ORPHAN reply addr={tr['addr']} fc=0x{tr['fc']:02x}"] += 1
            continue
        answered = "answered" if tr["reply"] else ("broadcast" if tr["addr"] == 0 else "NO REPLY")
        kinds[f"addr={tr['addr']} fc=0x{tr['fc']:02x} start=0x{tr['start']:04x} count={tr['count']} {answered}"] += 1
    out["exchange_types"] = dict(kinds.most_common())

    # Timing (only when timestamps exist).
    timed = [tr for tr in transactions if tr.get("t") is not None and "start" in tr]
    if timed:
        out["duration_s"] = round(timed[-1]["t"] - timed[0]["t"], 1)
        polls = [tr["t"] for tr in timed if tr["addr"] == 1 and tr["fc"] == 3 and tr["start"] == 0x0BB9]
        gaps = [b - a for a, b in zip(polls, polls[1:])]
        if gaps:
            gaps.sort()
            out["cycle_period_s"] = {"min": round(gaps[0], 3), "median": round(gaps[len(gaps) // 2], 3), "max": round(gaps[-1], 3)}
        lat = sorted(tr["reply"]["t"] - tr["t"] for tr in timed if tr["reply"] and tr["reply"].get("t") is not None)
        if lat:
            out["reply_latency_s"] = {"min": round(lat[0], 3), "median": round(lat[len(lat) // 2], 3), "max": round(lat[-1], 3)}

    # Register variability per block.
    blocks: dict[str, list[list[int]]] = collections.defaultdict(list)
    for tr in transactions:
        if "start" not in tr:
            continue
        key = f"addr{tr['addr']}_fc{tr['fc']:02x}_0x{tr['start']:04x}"
        if tr["write_values"]:
            blocks[key + "_write"].append(tr["write_values"])
        if tr["reply"] and tr["reply"].get("values"):
            blocks[key + "_reply"].append(tr["reply"]["values"])
    var = {}
    for key, rows in blocks.items():
        n = max(len(r) for r in rows)
        cols = []
        for k in range(n):
            vals = [r[k] for r in rows if k < len(r)]
            distinct = sorted(set(vals))
            cols.append({
                "i": k,
                "first": f"0x{vals[0]:04x}",
                "distinct": len(distinct),
                "min": f"0x{min(vals):04x}",
                "max": f"0x{max(vals):04x}",
                "values": [f"0x{v:04x}" for v in distinct[:8]],
            })
        var[key] = {"samples": len(rows), "registers": cols}
    out["blocks"] = var

    # Rare exchanges with context: everything that is not the regular 3-step cycle.
    regular = {(1, 3, 0x0BB9), (2, 3, 0x0BB9), (0, 0x10, 0x07D1)}
    rare = []
    prev_poll = None
    for idx, tr in enumerate(transactions):
        if "start" in tr and (tr["addr"], tr["fc"], tr["start"]) == (1, 3, 0x0BB9) and tr["reply"]:
            prev_poll = tr["reply"]["values"]
        if "start" not in tr or (tr["addr"], tr["fc"], tr["start"]) not in regular:
            rare.append({
                "index": idx,
                "t": tr.get("t"),
                "addr": tr["addr"],
                "fc": tr["fc"],
                "start": f"0x{tr['start']:04x}" if "start" in tr else None,
                "count": tr.get("count"),
                "write_values": [f"0x{v:04x}" for v in tr["write_values"]] if tr.get("write_values") else None,
                "reply": [f"0x{v:04x}" for v in tr["reply"]["values"]] if tr.get("reply") and tr["reply"].get("values") else (tr.get("reply") and "ack"),
                "preceding_dev1_poll_regs_8_to_16": [f"0x{v:04x}" for v in prev_poll[8:17]] if prev_poll else None,
                "orphan": tr.get("orphan_reply"),
            })
    out["rare_exchanges"] = rare
    return out


if __name__ == "__main__":
    result = [analyse(p) for p in sys.argv[2:]]
    with open(sys.argv[1], "w", encoding="utf-8") as f:
        json.dump(result, f, indent=1)
    for r in result:
        print(f"== {r['file']}")
        for k in ("bytes", "frames", "unframed_bytes", "duration_s", "cycle_period_s", "reply_latency_s"):
            if k in r:
                print(f"  {k}: {r[k]}")
        for k, v in r["exchange_types"].items():
            print(f"  {v:6d}  {k}")
        print(f"  rare exchanges: {len(r['rare_exchanges'])}")
