#pragma once

#include "esphome/components/modbus_sniffer/modbus_sniffer.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome::modbus_sniffer {

class ModbusSnifferSensor : public sensor::Sensor, public BlockListener {
 public:
  ModbusSnifferSensor(uint8_t address, uint8_t function, uint16_t start) : BlockListener(address, function, start) {}
  void set_index(uint8_t index) { this->index_ = index; }
  void set_value_type(ValueType value_type) { this->value_type_ = value_type; }
  void set_bitmask(uint32_t bitmask) { this->bitmask_ = bitmask; }

  void on_block(const Block &block) override {
    float value;
    if (this->bitmask_ != 0) {
      if (this->index_ >= block.registers.size())
        return;
      value = block.registers[this->index_] & this->bitmask_;
    } else if (!decode_value(block.registers, this->index_, this->value_type_, value)) {
      return;
    }
    this->publish_state(value);
  }

 protected:
  uint8_t index_{0};
  ValueType value_type_{ValueType::U_WORD};
  uint32_t bitmask_{0};
};

}  // namespace esphome::modbus_sniffer
