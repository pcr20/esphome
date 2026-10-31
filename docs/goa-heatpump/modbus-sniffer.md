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

Checked against the IO Ninja captures with a Python port of the parser
(`tools/sniffer_check.py`): all 4 906 frames of the turn-on capture, 0 skipped bytes, every block
image's register [9] equal to its own start address.

## Hub options

| Option | Default | |
|---|---|---|
| `uart_id` | | UART to listen on; only RX is needed. |
| `resync_timeout` | `50ms` | Quiet time after which an incomplete buffered frame is dropped. |
| `log_frames` | `false` | Log every frame (hex, 32 bytes per line) at DEBUG. |
| `stats_interval` | `60s` | Log frame / per-address / per-block counters at DEBUG; `0s` disables. |
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
- `binary_sensor`: `bitmask` (default `0xFFFF`); on when `register & bitmask` is non-zero.
- `text_sensor`: `format: HEX` (default; 4 lowercase hex digits per register, the same layout as
  `modbus_controller`'s `HEXBYTES`, so existing `substr(4 * n, 4)` lambdas keep working) or
  `ASCII`; `count` registers from `index` (0 = to the end of the block). Publishes only when the
  text changes.

## Migrating from the `dev_uart_update` fork

| Old | New |
|---|---|
| `external_components` with `modbus`, `modbus_controller`, `uart` | only `modbus_sniffer` |
| `modbus:` + `modbus_controller:` with `disable_send` and dummy `custom_command` | `modbus_sniffer:` |
| `HEXBYTES` text sensor + `substr()` lambdas | `sensor` per value with `index` / `value_type`, or the `HEX` text sensor |
| metadata stamped onto sensors (`address_in`, `start_reg_in`, …) | `on_block` arguments or `get_block()` |
