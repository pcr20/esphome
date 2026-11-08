#include "modbus_sniffer.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cinttypes>
#include <cstring>

namespace esphome::modbus_sniffer {

static const char *const TAG = "modbus_sniffer";

static constexpr uint8_t MAX_ADDRESS = 247;
static constexpr uint8_t EXCEPTION_FLAG = 0x80;
static constexpr uint8_t FC_READ_COILS = 0x01;
static constexpr uint8_t FC_READ_DISCRETE_INPUTS = 0x02;
static constexpr uint8_t FC_READ_HOLDING_REGISTERS = 0x03;
static constexpr uint8_t FC_READ_INPUT_REGISTERS = 0x04;
static constexpr uint8_t FC_WRITE_SINGLE_COIL = 0x05;
static constexpr uint8_t FC_WRITE_SINGLE_REGISTER = 0x06;
static constexpr uint8_t FC_WRITE_MULTIPLE_COILS = 0x0F;
static constexpr uint8_t FC_WRITE_MULTIPLE_REGISTERS = 0x10;
/// Bytes per frame logged on one line.
static constexpr size_t LOG_CHUNK = 32;

/// Warn once the UART buffer has been this full (percent): beyond it, bytes may be dropped unseen.
static constexpr size_t RX_BUFFER_WARN_PERCENT = 75;

static bool crc_ok(const uint8_t *frame, size_t len) {
  uint16_t crc = crc16(frame, len - 2);
  return crc == (frame[len - 2] | (frame[len - 1] << 8));
}

/// Bit for a known function code in AddressStats::functions, or 0 for an unknown one.
static uint8_t function_bit(uint8_t base) {
  if (base >= FC_READ_COILS && base <= FC_WRITE_SINGLE_REGISTER)
    return 1 << (base - FC_READ_COILS);
  if (base == FC_WRITE_MULTIPLE_COILS)
    return 1 << 6;
  if (base == FC_WRITE_MULTIPLE_REGISTERS)
    return 1 << 7;
  return 0;
}

/// Length of the shortest CRC-valid frame (address <= 247) at the start of @p data, or 0 if there is none.
static size_t crc_frame_length(const uint8_t *data, size_t size) {
  if (size < 4 || data[0] > MAX_ADDRESS)
    return 0;
  const size_t limit = std::min(size, MAX_FRAME_LENGTH);
  uint16_t crc = crc16(data, 2);  // CRC of data[0, len - 2)
  for (size_t len = 4; len <= limit; len++) {
    if (crc == (data[len - 2] | (data[len - 1] << 8)))
      return len;
    crc = crc16(data + len - 2, 1, crc);
  }
  return 0;
}

/// Number of CRC-valid frames that exactly make up @p data, or 0 if it does not split into whole frames.
static size_t split_crc_frames(const uint8_t *data, size_t size) {
  size_t count = 0;
  for (size_t offset = 0; offset < size; count++) {
    size_t len = crc_frame_length(data + offset, size - offset);
    if (len == 0)
      return 0;
    offset += len;
  }
  return count;
}

size_t value_type_words(ValueType type) {
  switch (type) {
    case ValueType::U_DWORD:
    case ValueType::S_DWORD:
    case ValueType::U_DWORD_R:
    case ValueType::S_DWORD_R:
    case ValueType::FP32:
    case ValueType::FP32_R:
      return 2;
    default:
      return 1;
  }
}

bool decode_value(const std::vector<uint16_t> &registers, size_t index, ValueType type, float &value) {
  size_t words = value_type_words(type);
  if (index + words > registers.size())
    return false;
  uint16_t w0 = registers[index];
  uint16_t w1 = words > 1 ? registers[index + 1] : 0;
  uint32_t dword = (uint32_t(w0) << 16) | w1;
  uint32_t dword_r = (uint32_t(w1) << 16) | w0;
  switch (type) {
    case ValueType::U_WORD:
      value = w0;
      return true;
    case ValueType::S_WORD:
      value = static_cast<int16_t>(w0);
      return true;
    case ValueType::U_DWORD:
      value = dword;
      return true;
    case ValueType::S_DWORD:
      value = static_cast<int32_t>(dword);
      return true;
    case ValueType::U_DWORD_R:
      value = dword_r;
      return true;
    case ValueType::S_DWORD_R:
      value = static_cast<int32_t>(dword_r);
      return true;
    case ValueType::FP32:
      memcpy(&value, &dword, sizeof(value));
      return true;
    case ValueType::FP32_R:
      memcpy(&value, &dword_r, sizeof(value));
      return true;
    case ValueType::BCD: {
      uint32_t result = 0;
      for (int shift = 12; shift >= 0; shift -= 4) {
        uint8_t digit = (w0 >> shift) & 0x0F;
        if (digit > 9)
          return false;
        result = result * 10 + digit;
      }
      value = result;
      return true;
    }
  }
  return false;
}

void ModbusSniffer::setup() {
  this->buffer_.reserve(2 * MAX_FRAME_LENGTH);
  this->run_buf_.reserve(2 * MAX_FRAME_LENGTH);
  if (this->stats_interval_ > 0)
    this->set_interval("stats", this->stats_interval_, [this]() { this->log_stats_(); });
}

void ModbusSniffer::loop() {
  const uint32_t now = millis();
  size_t avail = this->available();
  if (avail > this->rx_buffer_peak_) {
    // The buffer only fills between reads, so its level here is its peak. If it ever fills up, the
    // UART driver drops bytes without telling anyone; a peak well below its size rules that out.
    this->rx_buffer_peak_ = avail;
    const size_t size = this->parent_->get_rx_buffer_size();
    if (!this->rx_buffer_warned_ && avail * 100 >= size * RX_BUFFER_WARN_PERCENT) {
      this->rx_buffer_warned_ = true;
      ESP_LOGW(TAG, "UART buffer reached %u of %u bytes; bytes may be lost. Increase rx_buffer_size",
               (unsigned) avail, (unsigned) size);
    }
  }
  if (avail > 0) {
    uint8_t chunk[64];
    size_t total = 0;
    // Bound the work per loop() call; the rest stays in the UART buffer.
    while (avail > 0 && total < 2 * MAX_FRAME_LENGTH) {
      size_t n = std::min(avail, sizeof(chunk));
      if (!this->read_array(chunk, n))
        break;
      this->buffer_.insert(this->buffer_.end(), chunk, chunk + n);
      this->rx_bytes_ += n;
      total += n;
      avail = this->available();
    }
    this->last_rx_ = now;
    this->parse_buffer_(false);
  } else if (!this->buffer_.empty() && now - this->last_rx_ > this->resync_timeout_) {
    // The bus went quiet with an incomplete frame buffered: consume or discard what is left.
    this->parse_buffer_(true);
  }
}

void ModbusSniffer::parse_buffer_(bool flush) {
  while (!this->buffer_.empty()) {
    int len = this->frame_length_(flush);
    if (len == 0)
      break;  // need more bytes
    if (len < 0) {
      this->skip_byte_();
      continue;
    }
    this->end_run_();
    this->handle_frame_(this->buffer_.data(), len);
    this->buffer_.erase(this->buffer_.begin(), this->buffer_.begin() + len);
  }
  if (flush)
    this->end_run_();  // the bus went quiet, so the run cannot continue
}

/// Drop the first buffered byte, which starts no valid frame, into the current run of skipped bytes.
///
/// Resyncing goes byte by byte, but for the statistics the run is also walked frame by frame: a
/// plausible header (address, known function code) starts a presumed frame of the length that
/// header implies, and bytes up to its end belong to it. Each presumed frame is a CRC error.
void ModbusSniffer::skip_byte_() {
  if (this->walk_remaining_ > 0) {
    this->walk_remaining_--;
  } else {
    size_t len = this->presumed_length_();
    if (len > 0) {
      this->run_frames_++;
      this->walk_remaining_ = len - 1;
    }
  }
  if (this->run_buf_.size() < 2 * MAX_FRAME_LENGTH)
    this->run_buf_.push_back(this->buffer_.front());
  this->run_bytes_++;
  this->skipped_bytes_++;
  this->buffer_.erase(this->buffer_.begin());
}

/// Length of the frame the header at the start of the buffer implies, or 0 if it is not a plausible header.
size_t ModbusSniffer::presumed_length_() const {
  const size_t size = this->buffer_.size();
  const uint8_t *buf = this->buffer_.data();
  if (size < 2 || buf[0] > MAX_ADDRESS)
    return 0;
  const uint8_t fc = buf[1];
  const uint8_t base = fc & ~EXCEPTION_FLAG;
  // Only an address and function already seen together in a valid frame make a convincing header;
  // anything else in a run is more likely data than the start of a frame.
  const uint8_t bit = function_bit(base);
  bool seen = false;
  for (const auto &stats : this->addresses_) {
    if (stats.address == buf[0]) {
      seen = (stats.functions & bit) != 0;
      break;
    }
  }
  if (!seen)
    return 0;
  if (fc & EXCEPTION_FLAG)
    return 5;
  // Where a request and a reply share a function code, a pending request from this address means a reply.
  const bool reply = this->pending_.active && this->pending_.address == buf[0] && this->pending_.function == base;
  size_t len = 8;
  if (base <= FC_READ_INPUT_REGISTERS) {
    if (reply && size >= 3)
      len = 5 + size_t(buf[2]);
  } else if (base == FC_WRITE_MULTIPLE_COILS || base == FC_WRITE_MULTIPLE_REGISTERS) {
    if (!reply && size >= 7)
      len = 9 + size_t(buf[6]);
  }
  return std::min(len, MAX_FRAME_LENGTH);
}

/// Classify the run of skipped bytes that just ended, when a valid frame follows or the bus goes quiet.
void ModbusSniffer::end_run_() {
  if (this->run_bytes_ == 0)
    return;
  const uint8_t *run = this->run_buf_.data();
  const size_t size = this->run_buf_.size();
  size_t unknown = size == this->run_bytes_ ? split_crc_frames(run, size) : 0;
  if (unknown > 0) {
    // Whole frames with valid CRCs that the length rules do not know: an unhandled function code or length.
    this->unknown_frames_ += unknown;
    for (size_t offset = 0; offset < size;) {
      size_t len = crc_frame_length(run + offset, size - offset);
      this->log_hex_(ESPHOME_LOG_LEVEL_INFO, "Unknown frame", run + offset, len);
      offset += len;
    }
  } else {
    this->crc_errors_ += this->run_frames_;
    if (this->run_frames_ == 0)
      this->noise_runs_++;
    ESP_LOGD(TAG, "Skipped %" PRIu32 " bytes: %" PRIu32 " frames with CRC errors%s", this->run_bytes_,
             this->run_frames_, this->walk_remaining_ > 0 ? ", the last cut short" : "");
    this->log_hex_(ESPHOME_LOG_LEVEL_VERBOSE, "Skipped", run, std::min(size, LOG_CHUNK * 2));
  }
  this->run_buf_.clear();
  this->run_bytes_ = 0;
  this->run_frames_ = 0;
  this->walk_remaining_ = 0;
}

/// Length of the valid frame at the start of the buffer, 0 if more bytes are needed,
/// or -1 if no valid frame starts there. With @p flush no more bytes will come.
int ModbusSniffer::frame_length_(bool flush) const {
  const size_t size = this->buffer_.size();
  if (size < 4)
    return flush ? -1 : 0;
  const uint8_t *buf = this->buffer_.data();
  if (buf[0] > MAX_ADDRESS)
    return -1;
  const uint8_t fc = buf[1];
  const uint8_t base = fc & ~EXCEPTION_FLAG;

  // Candidate lengths: a request and a reply can share a function code but differ in length.
  size_t candidates[2];
  size_t count = 0;
  bool length_unknown = false;
  if (fc & EXCEPTION_FLAG) {
    candidates[count++] = 5;
  } else if (base >= FC_READ_COILS && base <= FC_READ_INPUT_REGISTERS) {
    candidates[count++] = 8;              // request
    size_t reply = 5 + size_t(buf[2]);    // reply: address, fc, byte count, data, CRC
    if (reply != 8)
      candidates[count++] = reply;
  } else if (base == FC_WRITE_SINGLE_COIL || base == FC_WRITE_SINGLE_REGISTER) {
    candidates[count++] = 8;  // request and echo reply
  } else if (base == FC_WRITE_MULTIPLE_COILS || base == FC_WRITE_MULTIPLE_REGISTERS) {
    candidates[count++] = 8;  // reply
    if (size >= 7) {
      candidates[count++] = 9 + size_t(buf[6]);  // request: header, byte count, data, CRC
    } else {
      length_unknown = true;
    }
  } else {
    return -1;
  }
  if (count == 2 && candidates[1] < candidates[0])
    std::swap(candidates[0], candidates[1]);

  for (size_t i = 0; i < count; i++) {
    size_t len = candidates[i];
    if (len > MAX_FRAME_LENGTH)
      continue;
    if (size < len) {
      if (flush)
        continue;
      return 0;
    }
    if (crc_ok(buf, len))
      return static_cast<int>(len);
  }
  if (length_unknown && !flush)
    return 0;
  return -1;
}

void ModbusSniffer::handle_frame_(const uint8_t *frame, size_t len) {
  this->frames_++;
  if (!this->frame_callbacks_.empty()) {
    std::vector<uint8_t> data(frame, frame + len);
    this->frame_callbacks_.call(data);
  }

  const uint8_t address = frame[0];
  const uint8_t fc = frame[1];
  const uint8_t base = fc & ~EXCEPTION_FLAG;
  const bool pending_match =
      this->pending_.active && this->pending_.address == address && this->pending_.function == base;
  this->stats_for_(address).functions |= function_bit(base);

  bool reply;
  if (fc & EXCEPTION_FLAG) {
    reply = true;
  } else if (base >= FC_READ_COILS && base <= FC_READ_INPUT_REGISTERS) {
    bool can_be_request = len == 8;
    bool can_be_reply = len == 5 + size_t(frame[2]);
    reply = can_be_request && can_be_reply ? pending_match : can_be_reply;
  } else if (base == FC_WRITE_SINGLE_COIL || base == FC_WRITE_SINGLE_REGISTER) {
    // The reply echoes the request.
    reply = pending_match && this->pending_.start == ((frame[2] << 8) | frame[3]);
  } else {
    reply = len == 8;
  }

  if (reply) {
    this->handle_reply_(frame, len);
  } else {
    this->handle_request_(frame, len);
  }
}

void ModbusSniffer::handle_request_(const uint8_t *frame, size_t len) {
  const uint8_t address = frame[0];
  const uint8_t fc = frame[1];
  const uint16_t start = (frame[2] << 8) | frame[3];
  const uint16_t count = (frame[4] << 8) | frame[5];
  this->log_frame_(frame, len, address == 0 ? "broadcast" : "request");

  this->close_pending_unanswered_();
  this->stats_for_(address).requests++;

  if (fc == FC_WRITE_MULTIPLE_REGISTERS) {
    this->update_block_(address, fc, start, frame + 7, frame[6] / 2);
  } else if (fc == FC_WRITE_SINGLE_REGISTER) {
    this->update_block_(address, fc, start, frame + 4, 1);
  }

  // Broadcasts are never answered.
  if (address != 0) {
    this->pending_.active = true;
    this->pending_.address = address;
    this->pending_.function = fc;
    this->pending_.start = start;
    this->pending_.count = count;
  }
}

void ModbusSniffer::handle_reply_(const uint8_t *frame, size_t len) {
  const uint8_t address = frame[0];
  const uint8_t fc = frame[1];
  const uint8_t base = fc & ~EXCEPTION_FLAG;
  this->log_frame_(frame, len, "reply");

  if (!this->pending_.active || this->pending_.address != address || this->pending_.function != base) {
    // A reply carries no start address; without its request it cannot be placed in a block.
    this->orphan_replies_++;
    ESP_LOGD(TAG, "Reply from 0x%02X fc 0x%02X without a matching request", address, fc);
    return;
  }
  this->pending_.active = false;
  AddressStats &stats = this->stats_for_(address);
  stats.replies++;

  if (fc & EXCEPTION_FLAG) {
    stats.exceptions++;
    ESP_LOGD(TAG, "Exception 0x%02X from 0x%02X for fc 0x%02X start 0x%04X", frame[2], address, base,
             this->pending_.start);
    return;
  }
  if (base == FC_READ_HOLDING_REGISTERS || base == FC_READ_INPUT_REGISTERS)
    this->update_block_(address, base, this->pending_.start, frame + 3, frame[2] / 2);
}

void ModbusSniffer::close_pending_unanswered_() {
  if (!this->pending_.active)
    return;
  this->stats_for_(this->pending_.address).unanswered++;
  ESP_LOGV(TAG, "No reply from 0x%02X to fc 0x%02X start 0x%04X", this->pending_.address, this->pending_.function,
           this->pending_.start);
  this->pending_.active = false;
}

void ModbusSniffer::update_block_(uint8_t address, uint8_t function, uint16_t start, const uint8_t *data,
                                  uint16_t count) {
  Block *block = nullptr;
  for (auto &candidate : this->blocks_) {
    if (candidate.address == address && candidate.function == function && candidate.start == start) {
      block = &candidate;
      break;
    }
  }
  if (block == nullptr) {
    Block created;
    created.address = address;
    created.function = function;
    created.start = start;
    this->blocks_.push_back(std::move(created));
    block = &this->blocks_.back();
    ESP_LOGI(TAG, "New block: address 0x%02X fc 0x%02X start 0x%04X, %u registers", address, function, start,
             count);
  }
  if (block->registers.size() < count)
    block->registers.resize(count);
  for (uint16_t i = 0; i < count; i++)
    block->registers[i] = (data[2 * i] << 8) | data[2 * i + 1];
  block->updates++;
  block->last_update = millis();

  this->block_callbacks_.call(address, function, start, block->registers);
  for (auto *listener : this->listeners_) {
    if (listener->matches(*block))
      listener->on_block(*block);
  }
}

const Block *ModbusSniffer::get_block(uint8_t address, uint8_t function, uint16_t start) const {
  for (const auto &block : this->blocks_) {
    if (block.address == address && block.function == function && block.start == start)
      return &block;
  }
  return nullptr;
}

AddressStats &ModbusSniffer::stats_for_(uint8_t address) {
  for (auto &stats : this->addresses_) {
    if (stats.address == address)
      return stats;
  }
  ESP_LOGI(TAG, "New bus address 0x%02X%s", address, address == 0 ? " (broadcast)" : "");
  AddressStats stats;
  stats.address = address;
  this->addresses_.push_back(stats);
  return this->addresses_.back();
}

/// Log a frame as hex, 32 bytes per line, at INFO, DEBUG or VERBOSE.
void ModbusSniffer::log_hex_(int level, const char *kind, const uint8_t *frame, size_t len) {
  char hex[format_hex_pretty_size(LOG_CHUNK)];
  for (size_t offset = 0; offset < len; offset += LOG_CHUNK) {
    size_t n = std::min(LOG_CHUNK, len - offset);
    format_hex_pretty_to(hex, frame + offset, n, ' ');
#define SNIFFER_LOG_HEX(log) \
  if (offset == 0) { \
    log(TAG, "%s 0x%02X fc 0x%02X (%u bytes): %s", kind, frame[0], len > 1 ? frame[1] : 0, (unsigned) len, hex); \
  } else { \
    log(TAG, "  +%03u: %s", (unsigned) offset, hex); \
  }
    if (level == ESPHOME_LOG_LEVEL_INFO) {
      SNIFFER_LOG_HEX(ESP_LOGI)
    } else if (level == ESPHOME_LOG_LEVEL_DEBUG) {
      SNIFFER_LOG_HEX(ESP_LOGD)
    } else {
      SNIFFER_LOG_HEX(ESP_LOGV)
    }
#undef SNIFFER_LOG_HEX
  }
}

void ModbusSniffer::log_frame_(const uint8_t *frame, size_t len, const char *kind) {
  if (this->log_frames_)
    this->log_hex_(ESPHOME_LOG_LEVEL_DEBUG, kind, frame, len);
}

float ModbusSniffer::get_statistic(Statistic statistic, int16_t address) const {
  switch (statistic) {
    case Statistic::RX_BYTES:
      return this->rx_bytes_;
    case Statistic::FRAMES:
      return this->frames_;
    case Statistic::SKIPPED_BYTES:
      return this->skipped_bytes_;
    case Statistic::CRC_ERRORS:
      return this->crc_errors_;
    case Statistic::UNKNOWN_FRAMES:
      return this->unknown_frames_;
    case Statistic::ORPHAN_REPLIES:
      return this->orphan_replies_;
    case Statistic::RX_BUFFER_PEAK:
      return this->rx_buffer_peak_;
    default:
      break;
  }
  uint32_t sum = 0;
  for (const auto &stats : this->addresses_) {
    if (address >= 0 && stats.address != address)
      continue;
    switch (statistic) {
      case Statistic::REQUESTS:
        sum += stats.requests;
        break;
      case Statistic::REPLIES:
        sum += stats.replies;
        break;
      case Statistic::UNANSWERED:
        sum += stats.unanswered;
        break;
      case Statistic::EXCEPTIONS:
        sum += stats.exceptions;
        break;
      default:
        break;
    }
  }
  return sum;
}

void ModbusSniffer::log_stats_() {
  // One INFO line answers "was everything processed": bytes in valid frames out of all bytes received,
  // and every frame that was not (CRC errors, unknown frames), plus how close the UART buffer came to full.
  const uint32_t bad = this->crc_errors_ + this->unknown_frames_;
  const uint64_t settled = this->rx_bytes_ - this->buffer_.size();  // bytes already framed or skipped
  const float good_bytes =
      settled > 0 ? 100.0f * float(settled - this->skipped_bytes_) / float(settled) : 100.0f;
  const float error_rate = this->frames_ + bad > 0 ? 100.0f * float(bad) / float(this->frames_ + bad) : 0.0f;
  ESP_LOGI(TAG,
           "%" PRIu64 " bytes, %.3f%% in %" PRIu32 " valid frames; %" PRIu32 " CRC errors, %" PRIu32
           " unknown frames (%.3f%% of frames), %" PRIu32 " skipped bytes; UART buffer peak %u/%u",
           this->rx_bytes_, good_bytes, this->frames_, this->crc_errors_, this->unknown_frames_, error_rate,
           this->skipped_bytes_, (unsigned) this->rx_buffer_peak_, (unsigned) this->parent_->get_rx_buffer_size());
  ESP_LOGD(TAG, "  Orphan replies: %" PRIu32 ", runs of noise: %" PRIu32, this->orphan_replies_, this->noise_runs_);
  for (const auto &stats : this->addresses_) {
    ESP_LOGD(TAG, "  Address 0x%02X: %" PRIu32 " requests, %" PRIu32 " replies, %" PRIu32 " unanswered, %" PRIu32
             " exceptions",
             stats.address, stats.requests, stats.replies, stats.unanswered, stats.exceptions);
  }
  const uint32_t now = millis();
  for (const auto &block : this->blocks_) {
    ESP_LOGD(TAG, "  Block 0x%02X fc 0x%02X start 0x%04X: %u registers, %" PRIu32 " updates, last %" PRIu32 " ms ago",
             block.address, block.function, block.start, (unsigned) block.registers.size(), block.updates,
             now - block.last_update);
  }
}

void ModbusSniffer::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Modbus Sniffer:\n"
                "  Resync timeout: %" PRIu32 " ms\n"
                "  Log frames: %s\n"
                "  Stats interval: %" PRIu32 " ms\n"
                "  Listeners: %u",
                this->resync_timeout_, YESNO(this->log_frames_), this->stats_interval_,
                (unsigned) this->listeners_.size());
  // dump_config runs again whenever a log client connects, so the inventory is not only seen at boot.
  ESP_LOGCONFIG(TAG,
                "  Received: %" PRIu64 " bytes, %" PRIu32 " frames, %" PRIu32 " CRC errors, %" PRIu32
                " unknown frames, %" PRIu32 " skipped bytes, UART buffer peak %u",
                this->rx_bytes_, this->frames_, this->crc_errors_, this->unknown_frames_, this->skipped_bytes_,
                (unsigned) this->rx_buffer_peak_);
  for (const auto &stats : this->addresses_) {
    ESP_LOGCONFIG(TAG, "  Address 0x%02X: %" PRIu32 " requests, %" PRIu32 " replies, %" PRIu32 " unanswered",
                  stats.address, stats.requests, stats.replies, stats.unanswered);
  }
  for (const auto &block : this->blocks_) {
    ESP_LOGCONFIG(TAG, "  Block 0x%02X fc 0x%02X start 0x%04X: %u registers, %" PRIu32 " updates", block.address,
                  block.function, block.start, (unsigned) block.registers.size(), block.updates);
  }
}

}  // namespace esphome::modbus_sniffer
