# Goa heat pump — RS-485 Modbus bus map

Status: reverse-engineered from passive captures. Everything here is observed behaviour, not a
manufacturer specification. Items marked **(hypothesis)** are unconfirmed.

## Sources

| Source | What it is |
|---|---|
| `IO Ninja/sessions/goa serial/goa serial_turnon.txt` | 37 min timestamped capture (2023-11), includes a power-on from the display. 4 906 frames, 0 unframed bytes. |
| `IO Ninja/sessions/goa serial/log 100ms timeout.txt` | Longer untimestamped capture (2023-10), ~2 190 polling cycles, two parameter-block exchanges. |
| `IO Ninja/sessions/goa serial/state progress 00 01.txt`, `... 00 10.txt` | Annotated excerpts of the two "change" sequences. |
| `OneDrive/Documents/goa pompe a chaleur.xlsx` | Register labels (T01–T15, O06–O09, R01–R03, timers) from manual comparison with the display. |
| ESP32 `esphome-web-910e58` live logs (2026-10-31) | Confirms the same traffic today (unknown address 0x02 every ~1.8 s). |

Frames were extracted by `tools/busmap.py` (in this folder): all bytes are concatenated and split
using Modbus RTU length rules plus CRC, independently of how the capture tool chunked them.

## Physical layer

- RS-485, 9600 baud, 8N1 (no parity). Wiring (heat-pump connector): orange = A, orange/white = B,
  brown = +12 V, brown/white = GND.
- Bus utilisation ≈ 400 bytes per 1.8 s cycle (~23 %). There are two idle gaps of ~0.6 s per cycle.

## Roles

| Address | Role | Evidence |
|---|---|---|
| — | **Master = heat-pump main board** | Issues every request; broadcasts live sensor data that only the main board can measure. |
| 1 | **Touchscreen display (slave)** | Answers reads of 0x0bb9 / 0x03e9 / 0x0443 and the 0x0bb9 acknowledge write. |
| 2 | **Polled, never answers** | Master reads 0x0bb9 from it every cycle and waits ~245 ms timeout. Probably an optional second display/remote slot. **(hypothesis)** |
| 0 | **Broadcast** (master → everyone) | Function 0x10 writes, never answered (Modbus broadcast). |

No other addresses or function codes appear in any capture.

## The regular cycle (every 1.45–2.58 s, median 1.81 s)

| Step | Frame | Size | Notes |
|---|---|---|---|
| 1 | Master → Dev1: `01 03 0b b9 00 5a` read 90 regs @ 0x0bb9 | 8 B | |
| 2 | Dev1 → Master: 90 regs | 185 B | Reply latency ≈ 15–65 ms. Contains clock + change flag. |
| — | idle ≈ 0.6 s | | |
| 3 | Master → Dev2: `02 03 0b b9 00 5a` | 8 B | |
| 4 | (no reply — master times out ≈ 245 ms) | | |
| 5 | Master → broadcast: `00 10 07 d1 00 5a b4 …` write 90 regs @ 0x07d1 | 189 B | **Live status block** (temperatures, compressor…). |
| — | idle ≈ 0.6 s | | |

In ~3 400 cycles the display missed a poll 5 times; the master simply carries on.

## The change sequence (the "less frequent", more detailed traffic)

The display raises a flag in its 0x0bb9 block; the master then fetches a settings block from the
display, acknowledges, and re-broadcasts the block to everyone:

1. Dev1's 0x0bb9 reply has register **[10] ≠ 0**:
   - `0x0001` → master reads **0x03e9** (seen on power-on from the display)
   - `0x0010` → master reads **0x0443** (seen after a parameter change)
2. Master → Dev1: read 90 regs @ 0x03e9 or 0x0443; display replies with the block.
3. Master → Dev1: **write 11 regs @ 0x0bb9** = the 10-register header + `[10] = 0x0000`
   (clears the flag; the display echoes the standard 0x10 reply). If unanswered, the master repeats it.
4. Flag reads back as 0 on the next polls.
5. A few cycles later (≈ 6.5 s after step 2) the master **broadcasts the same block** to address 0
   (write 90 regs @ 0x03e9 or 0x0443). On power-on the broadcast copy of 0x03e9 differed from the
   display's copy in one register ([13]: 0 → 1), i.e. the master broadcasts the state it applied.

Consequence for the sniffer: every block of interest eventually appears as an **address-0 broadcast**,
so decoding broadcasts plus Dev1's 0x0bb9 reply captures the complete state.

## Block format (all blocks)

Every 90-register block — read reply or broadcast write — starts with the same self-describing header:

| Reg | Value | Meaning |
|---|---|---|
| [0]–[5] | `5746 3231 3035 3132 3031 3033` | ASCII serial "WF2105120103" |
| [6]–[7] | `0000 0000` | |
| [8] | `0101` | constant — protocol/format version **(hypothesis)** |
| [9] | block start address (`0bb9`, `07d1`, `03e9`, `0443`) | the block names itself |
| [10]… | block data | |

Register `[i]` therefore corresponds to Modbus address `start + i`. Values are big-endian 16-bit;
temperatures are signed, ×0.1 °C.

### Block 0x0bb9 — display status (Dev1 reply to every poll)

| Reg | Meaning | Observed |
|---|---|---|
| [10] | change flag: 0 = none, `0001` = 0x03e9 changed, `0010` = 0x0443 changed | 0, 1, 0x10 |
| [14] | clock hour (**BCD**) | 0x09, 0x10, 0x14–0x16, 0x17, 0x20, 0x22 |
| [15] | clock minute (BCD) | 0x00–0x59 |
| [16] | clock second (BCD) | 0x00–0x59 |
| others | 0 | |

### Block 0x07d1 — live status (broadcast every cycle)

Labels from the spreadsheet; "range" is what the two captures saw.

| Reg | Label | Scale | Range seen | Current YAML decode |
|---|---|---|---|---|
| [10] | on/off | bool | 0/1 | `goa_pac_10_OnOff` |
| [11] | ? | | constant 1 | `goa_pac_12_` |
| [12] | ? | | 0 | `goa_pac_13_` |
| [18] | ? (mode/state?) | | 0, 0x0a, 0x0b | — |
| [19] | O06 expansion valve | ×0.1 (YAML) | 0x12c–0x181 | ✔ |
| [20] | O07 compressor output frequency | Hz | 0–85 | ✔ |
| [21] | O08 compressor current | ×0.1 A | 0–10.7 | ✔ |
| [22] | O09 IPM temp | ×0.1 °C | 18–36 | ✔ |
| [33] | S01–S06 bitfield? | bits | constant 0x0030 | — |
| [44] | T01 suction temp | ×0.1 °C | | ✔ |
| [45] | T02 inlet water temp | ×0.1 °C | | ✔ |
| [46] | T03 outlet water temp | ×0.1 °C | | ✔ |
| [47] | T04 coil 1 temp | ×0.1 °C | | ✔ |
| [48] | T05 ambient temp | ×0.1 °C | | ✔ |
| [49] | T06 exhaust temp | ×0.1 °C | | ✔ |
| [50] | T07 compressor current detect | ×0.1 A | | ✔ |
| [51] | T08 AC fan output | | 0 | ✔ |
| [52] | T09 flow rate input | | 0 | ✔ |
| [53] | T10 pressure sensor | ×0.1 bar | | ✔ |
| [54] | ? | | 0, 0x20, 0x2b–0x2d | — |
| [55] | constant marker | | `5aa5` | — |
| [56] | ? slowly varying counter | | 0x0518–0x077b | — |
| [58] | ? flag | | 0/1 | — |
| [59] | T11 superheat | ×0.1 °C | | ✔ |
| [60] | T12 target fan motor speed | rpm | 0–740 | ✔ |
| [61] | T13 overheat after compensation | ×0.1 °C | | ✔ |
| [62] | T14 inverter plate AC voltage | V | 198–223 | ✔ |
| [64] | T15 antifreeze temp | ×0.1 °C | | ✔ |
| [66] | ? | | 0–0x398 | — |
| [82] | main controller software version | | 27 | — |
| [83] | main controller software code | | 569 | — |

Register index = YAML HEXBYTES offset ÷ 4 (e.g. `x.substr(176,4)` → [44]).

### Block 0x0443 — parameters (Dev1 reply on flag `0010`, then broadcast)

| Reg | Label (spreadsheet) | Value seen | Decoded |
|---|---|---|---|
| [13] | defrost time | 0x2d / 0x2e | 45 / 46 min |
| [44] | R01 cooling setpoint | 0x010e | 27.0 °C |
| [45] | R02 heating setpoint | 0x012c | 30.0 °C |
| [46] | R03 auto setpoint | 0x015e | 35.0 °C |
| [59] | timer start hour | 0x000a | 10 |
| [60] | timer start minute | 0 | |
| [61] | timer stop hour | 0x0015 | 21 |
| [62] | timer stop minute | 0 | |
| others | unlabelled parameters (many non-zero) | | |

### Block 0x03e9 — user state/mode (Dev1 reply on flag `0001`, then broadcast) — mostly unlabelled

Seen once, at power-on: `[10]=1 [11]=1 [12]=0x012c (300 → 30.0 °C setpoint?) [13]=0→1 (power?) …`
followed by ~60 non-zero parameters. **(hypothesis: power / mode / active setpoint)** — needs captures
around specific display actions (on/off, mode change, setpoint change) to label.

The existing ESPHome YAML's `reg_vals3e9` global was intended to hold this block; the copy code never
fires (it checks for start address 0xFFF0), so it only ever shows its initial value.

## Implications for the `modbus_sniffer` component

1. **Address 0 is broadcast data, not a device** — the decoder must accept it (stock ESPHome 2026.10
   deliberately treats address 0 as never-answered and drops it as a peer response).
2. **Track blocks, not devices**: the natural key is `(address, function, start)`; each block is 90
   registers with a self-identifying header, so a fixed `uint16_t[90]` image per block is enough.
   Blocks to keep: broadcast 0x07d1 / 0x03e9 / 0x0443 and Dev1's 0x0bb9.
3. **Record every frame** regardless of address (requirement): unknown blocks get an image created
   on first sight and a log line; unanswered requests (Dev2) are counted, not warned about.
4. **Pair requests with replies** — replies carry no start address; only pairing gives it.
5. **Change detection**: publish entities only when their block is refreshed; expose the display's
   change flag and the BCD clock.

## Phase-2 control options (not yet tested)

- **Emulate Dev2 (hypothesis, preferred)**: the master already polls address 2 every cycle and
  gets no answer. An ESP answering as Dev2 with the display's protocol — raise flag `0010` or
  `0001` in its 0x0bb9 reply, then serve a modified 0x0443 / 0x03e9 block when the master reads it —
  would change settings through the master's own path, without bus collisions. Unknown: whether the
  master accepts settings from Dev2, and what Dev2 is meant to be.
- **Inject in idle gaps**: ~0.6 s idle windows twice per cycle; risky because the master does not
  expect a second master.
- **Inline proxy** (two UARTs between display and main board): full control, but a hardware change
  and a single point of failure.
