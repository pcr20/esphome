# `modbus_sniffer` component

A passive Modbus RTU monitor for ESPHome. It never transmits. It is an external component on
the stock `uart` and does not use or override `modbus` / `modbus_controller`. The bus it was written
for is described in [bus-map.md](bus-map.md); see [goa-sniffer-example.yaml](goa-sniffer-example.yaml)
for a complete configuration.

```yaml
external_components:
  - source: github://pcr20/esphome@modbus_sniffer_v2
    components: [modbus_sniffer]

uart:
  id: uart_modbus
  rx_pin: GPIO16
  baud_rate: 9600
  rx_buffer_size: 1024   # room for a 189-byte frame while the main loop is busy

modbus_sniffer:
  id: goa
  uart_id: uart_modbus
```

## How it works

- **Framing by content, not timing.** The byte stream is split into frames with the function-code
  length rules (read request 8 bytes, read reply 5 + byte count, write-multiple request
  9 + byte count, write reply 8, exception 5) and the CRC. UART buffering on the ESP32 makes
  3.5-character gaps unreliable, so gaps are only used to discard an incomplete tail after the bus
  goes quiet (`resync_timeout`). Bytes that do not form a valid frame are skipped and counted.
- **Every address.** Address 0 (broadcast) and addresses nothing is configured for are handled
  like any other; a new address or block is logged at INFO the first time it is seen.
- **Request/reply pairing.** A read reply carries no start address, so it is attributed to the
  last request with the same address and function. A request that is followed by another request
  is counted as unanswered (Dev2 on the Goa bus).
- **Block images.** Each `(address, function, start)` gets a register image that is updated in
  place. Writes (0x10, 0x06) are keyed by the address written to; read replies (0x03, 0x04) by the
  replying slave and the request's start address.

- **Every skipped byte is accounted for.** When a run of skipped bytes ends (a valid frame follows,
  or the bus goes quiet), it is classified:
  - If it splits exactly into frames with valid CRCs, those are **unknown frames**: a function code
    or length the parser does not handle. Each is logged at INFO with its hex and counted.
  - Otherwise it is walked frame by frame. Each header whose address and function code have already
    been seen together in a valid frame starts a presumed frame of the length its header implies.
    Each presumed frame counts as a **CRC error**, including frames that were cut short by lost bytes.
    A run with no such header is noise.
- **Lost bytes.** The ESP-IDF UART driver drops bytes silently when its buffer is full. The sniffer
  records the most bytes ever waiting in the buffer (`rx_buffer_peak`), and warns once it reaches
  75% of `rx_buffer_size`. A peak well below the buffer size means no bytes were lost before the
  sniffer read them.

Checked with a Python port of the parser (`tools/sniffer_check.py`):
- **Turn-on capture:** all 4 906 frames, 0 skipped bytes, and every block image's register [9]
  equal to its own start address.
- **`--selftest`:** damages that capture in known ways and expects the counts to match exactly. It
  flips bits in 69 frames, drops bytes from 69, inserts 204 unknown-function frames and adds 68 runs
  of noise. Valid frames, CRC errors, unknown frames, noise runs and the byte totals all match.
- **Second capture (`log 100ms timeout.txt`):** the 24 408 skipped bytes are 7 bad runs, with no
  unknown frames:
  - 5 short runs, each one damaged frame.
  - 2 long runs (15 202 and 8 580 bytes) with correct frame lengths but the top bit of most bytes
    cleared. The capture PC's port evidently ran with 7 data bits for a while.
  - The parser counts 211 CRC errors there, against 248 frames counted by hand. It undercounts the
    7-bit stretch because the stripped length bytes mislead the walk.

## Hub options

| Option | Default | |
|---|---|---|
| `uart_id` | | UART to listen on; only RX is needed. |
| `resync_timeout` | `50ms` | Quiet time after which an incomplete buffered frame is dropped. |
| `log_frames` | `false` | Log every frame (hex, 32 bytes per line) at DEBUG. |
| `stats_interval` | `60s` | Log a summary at INFO (bytes, % in valid frames, CRC errors, unknown frames, UART buffer peak), and per-address / per-block counters at DEBUG; `0s` disables. The same counters and the address/block inventory are part of the config dump, which is logged again whenever a log client connects, but only with `logger: level: CONFIG` or more verbose. |
| `on_block` | | Automation run for every block update, with `address`, `function`, `start`, `registers` (`std::vector<uint16_t>`). |
| `on_frame` | | Automation run for every valid frame, with `frame` (`std::vector<uint8_t>`, including CRC). |

From a lambda, `id(goa).get_block(address, function, start)` returns the latest `Block` (or
`nullptr`), with `registers`, `updates` and `last_update` (millis).

## Entities

All platforms locate a register the same way:

| Option | Default | |
|---|---|---|
| `address` | required | Bus address of the block; 0 = broadcast. |
| `function_code` | `0x10` for address 0, else `0x03` | Function of the frames carrying the block. |
| `block` | required | Start address of the block. |
| `index` | `0` | Register within the block (Modbus address `block + index`), as in the bus-map tables. |

- `sensor`: `value_type` one of `U_WORD` (default), `S_WORD`, `U_DWORD`, `S_DWORD`, `U_DWORD_R`,
  `S_DWORD_R`, `FP32`, `FP32_R`, `BCD` (each nibble a decimal digit, e.g. the display clock), or
  `bitmask` to publish `register & mask`. Scale with filters (`multiply: 0.1`). Publishes on every
  block update; add `delta`/`throttle` filters to reduce traffic.
- `sensor` with `statistic:` instead of `address`/`block`: one of the sniffer's own counters, published
  every `update_interval` (default `60s`) as a diagnostic entity. The available statistics are:
  - `rx_bytes`
  - `frames`
  - `skipped_bytes`
  - `crc_errors`
  - `unknown_frames`
  - `orphan_replies`
  - `rx_buffer_peak`
  - `requests`, `replies`, `unanswered` and `exceptions`, which take an optional `address`; without
    it they sum over all addresses.

  Counters are `total_increasing` and restart at 0 on reboot; `rx_buffer_peak` is a measurement.
  They are floats in Home Assistant, so `rx_bytes` stops being exact past 16.7 M (about 5 hours on
  the Goa bus).

  ```yaml
  sensor:
    - platform: modbus_sniffer
      statistic: crc_errors
      name: Modbus CRC errors
    - platform: modbus_sniffer
      statistic: unanswered
      address: 1
      name: Modbus unanswered display polls
  ```
- `binary_sensor`: `bitmask` (default `0xFFFF`); on when `register & bitmask` is non-zero.
- `text_sensor`: `format: HEX` (default; 4 lowercase hex digits per register, the same layout as
  `modbus_controller`'s `HEXBYTES`, so existing `substr(4 * n, 4)` lambdas keep working) or
  `ASCII`; `count` registers from `index` (0 = to the end of the block). Publishes only when the
  text changes. Home Assistant rejects states longer than 255 characters, so keep HEX sensors to
  at most 63 registers and split a 90-register block into two (`count: 60`, then `index: 60`).

## Migrating from the `dev_uart_update` fork

| Old | New |
|---|---|
| `external_components` with `modbus`, `modbus_controller`, `uart` | only `modbus_sniffer` |
| `modbus:` + `modbus_controller:` with `disable_send` and dummy `custom_command` | `modbus_sniffer:` |
| `HEXBYTES` text sensor + `substr()` lambdas | `sensor` per value with `index` / `value_type`, or the `HEX` text sensor |
| metadata stamped onto sensors (`address_in`, `start_reg_in`, …) | `on_block` arguments or `get_block()` |
