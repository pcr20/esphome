#pragma once

#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include <vector>

namespace esphome::modbus_sniffer {

/// Longest Modbus RTU frame (address + PDU + CRC).
static constexpr size_t MAX_FRAME_LENGTH = 256;

enum class ValueType : uint8_t {
  U_WORD,
  S_WORD,
  U_DWORD,
  S_DWORD,
  U_DWORD_R,
  S_DWORD_R,
  FP32,
  FP32_R,
  BCD,
};

/// Number of 16-bit registers a value of this type occupies.
size_t value_type_words(ValueType type);
/// Decode a value starting at registers[index]; returns false if the block is too short.
bool decode_value(const std::vector<uint16_t> &registers, size_t index, ValueType type, float &value);

/// Latest image of one register block, keyed by (address, function, start).
///
/// Writes (0x06/0x10) are keyed by the address the master wrote to (0 = broadcast);
/// read replies (0x03/0x04) by the replying slave and the start address of the paired request.
struct Block {
  uint8_t address;
  uint8_t function;
  uint16_t start;
  std::vector<uint16_t> registers;
  uint32_t updates{0};
  uint32_t last_update{0};
};

/// Counters a `statistic` sensor can publish. REQUESTS to EXCEPTIONS can be limited to one address.
enum class Statistic : uint8_t {
  RX_BYTES,        ///< bytes read from the UART
  FRAMES,          ///< valid frames
  SKIPPED_BYTES,   ///< bytes outside valid frames (bad frames, unknown frames, noise)
  CRC_ERRORS,      ///< frames with a plausible header whose CRC failed (or that were cut short)
  UNKNOWN_FRAMES,  ///< CRC-valid frames the parser does not understand (function code or length)
  ORPHAN_REPLIES,  ///< replies without a matching request
  RX_BUFFER_PEAK,  ///< most bytes ever waiting in the UART buffer
  REQUESTS,
  REPLIES,
  UNANSWERED,
  EXCEPTIONS,
};

/// Statistics for one bus address, created the first time the address is seen.
struct AddressStats {
  uint8_t address;
  uint32_t requests{0};
  uint32_t replies{0};
  uint32_t unanswered{0};
  uint32_t exceptions{0};
  /// Function codes seen in valid frames from or to this address, one bit per known code.
  uint8_t functions{0};
};

class BlockListener {
 public:
  BlockListener(uint8_t address, uint8_t function, uint16_t start)
      : address_(address), function_(function), start_(start) {}
  bool matches(const Block &block) const {
    return block.address == this->address_ && block.function == this->function_ && block.start == this->start_;
  }
  virtual void on_block(const Block &block) = 0;

 protected:
  uint8_t address_;
  uint8_t function_;
  uint16_t start_;
};

/// Passive Modbus RTU bus monitor.
///
/// Splits the byte stream into frames using the function-code length rules plus CRC (not
/// inter-frame timing, which UART buffering makes unreliable), pairs requests with replies,
/// and keeps a register image for every block seen on the bus, for every address including
/// broadcast address 0. It never transmits.
class ModbusSniffer : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_resync_timeout(uint32_t resync_timeout) { this->resync_timeout_ = resync_timeout; }
  void set_log_frames(bool log_frames) { this->log_frames_ = log_frames; }
  void set_stats_interval(uint32_t stats_interval) { this->stats_interval_ = stats_interval; }
  void register_listener(BlockListener *listener) { this->listeners_.push_back(listener); }

  /// Latest image of a block, or nullptr if it has not been seen yet. For use in lambdas.
  const Block *get_block(uint8_t address, uint8_t function, uint16_t start) const;
  const std::vector<Block> &get_blocks() const { return this->blocks_; }
  const std::vector<AddressStats> &get_address_stats() const { return this->addresses_; }
  /// Value of a counter; @p address (0-247) limits the per-address counters to one address, -1 sums all.
  float get_statistic(Statistic statistic, int16_t address = -1) const;

  template<typename F> void add_on_block_callback(F &&callback) { this->block_callbacks_.add(std::forward<F>(callback)); }
  template<typename F> void add_on_frame_callback(F &&callback) { this->frame_callbacks_.add(std::forward<F>(callback)); }

 protected:
  struct Pending {
    bool active{false};
    uint8_t address;
    uint8_t function;
    uint16_t start;
    uint16_t count;
  };

  void parse_buffer_(bool flush);
  int frame_length_(bool flush) const;
  void handle_frame_(const uint8_t *frame, size_t len);
  void handle_request_(const uint8_t *frame, size_t len);
  void handle_reply_(const uint8_t *frame, size_t len);
  void update_block_(uint8_t address, uint8_t function, uint16_t start, const uint8_t *data, uint16_t count);
  void close_pending_unanswered_();
  void skip_byte_();
  size_t presumed_length_() const;
  void end_run_();
  void log_hex_(int level, const char *kind, const uint8_t *frame, size_t len);
  void log_frame_(const uint8_t *frame, size_t len, const char *kind);
  void log_stats_();
  AddressStats &stats_for_(uint8_t address);

  std::vector<uint8_t> buffer_;
  /// The current run of skipped bytes (first 2 * MAX_FRAME_LENGTH bytes), classified when it ends.
  std::vector<uint8_t> run_buf_;
  uint32_t run_bytes_{0};
  /// Frames with a plausible header found in the current run, and bytes left of the last one.
  uint32_t run_frames_{0};
  size_t walk_remaining_{0};
  uint32_t last_rx_{0};
  uint32_t resync_timeout_{50};
  uint32_t stats_interval_{60000};
  bool log_frames_{false};

  Pending pending_;
  std::vector<Block> blocks_;
  std::vector<AddressStats> addresses_;
  std::vector<BlockListener *> listeners_;

  uint64_t rx_bytes_{0};
  uint32_t frames_{0};
  uint32_t skipped_bytes_{0};
  uint32_t crc_errors_{0};
  uint32_t unknown_frames_{0};
  uint32_t noise_runs_{0};
  uint32_t orphan_replies_{0};
  size_t rx_buffer_peak_{0};
  bool rx_buffer_warned_{false};

  LazyCallbackManager<void(uint8_t, uint8_t, uint16_t, const std::vector<uint16_t> &)> block_callbacks_;
  LazyCallbackManager<void(const std::vector<uint8_t> &)> frame_callbacks_;
};

}  // namespace esphome::modbus_sniffer
