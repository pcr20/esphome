#pragma once

#include "esphome/components/modbus_sniffer/modbus_sniffer.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include <algorithm>
#include <cstdio>
#include <string>

namespace esphome::modbus_sniffer {

enum class TextFormat : uint8_t {
  HEX,    ///< 4 lowercase hex digits per register, no separators (same as modbus_controller HEXBYTES)
  ASCII,  ///< 2 characters per register, high byte first; NUL bytes end the string
};

class ModbusSnifferTextSensor : public text_sensor::TextSensor, public BlockListener {
 public:
  ModbusSnifferTextSensor(uint8_t address, uint8_t function, uint16_t start)
      : BlockListener(address, function, start) {}
  void set_index(uint8_t index) { this->index_ = index; }
  /// Number of registers to include; 0 = to the end of the block.
  void set_count(uint8_t count) { this->count_ = count; }
  void set_format(TextFormat format) { this->format_ = format; }

  void on_block(const Block &block) override {
    const size_t size = block.registers.size();
    if (this->index_ >= size)
      return;
    size_t end = this->count_ == 0 ? size : std::min(size, size_t(this->index_) + this->count_);
    std::string text;
    if (this->format_ == TextFormat::HEX) {
      text.reserve((end - this->index_) * 4);
      char hex[5];
      for (size_t i = this->index_; i < end; i++) {
        snprintf(hex, sizeof(hex), "%04x", block.registers[i]);
        text += hex;
      }
    } else {
      for (size_t i = this->index_; i < end; i++) {
        char high = block.registers[i] >> 8;
        char low = block.registers[i] & 0xFF;
        if (high == 0)
          break;
        text += high;
        if (low == 0)
          break;
        text += low;
      }
    }
    // The bus repeats most blocks every cycle; only publish changes.
    if (this->has_published_ && text == this->last_)
      return;
    this->has_published_ = true;
    this->last_ = text;
    this->publish_state(text);
  }

 protected:
  uint8_t index_{0};
  uint8_t count_{0};
  TextFormat format_{TextFormat::HEX};
  bool has_published_{false};
  std::string last_;
};

}  // namespace esphome::modbus_sniffer
