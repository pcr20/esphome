#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/modbus_sniffer/modbus_sniffer.h"

namespace esphome::modbus_sniffer {

class ModbusSnifferBinarySensor : public binary_sensor::BinarySensor, public BlockListener {
 public:
  ModbusSnifferBinarySensor(uint8_t address, uint8_t function, uint16_t start)
      : BlockListener(address, function, start) {}
  void set_index(uint8_t index) { this->index_ = index; }
  void set_bitmask(uint16_t bitmask) { this->bitmask_ = bitmask; }

  void on_block(const Block &block) override {
    if (this->index_ >= block.registers.size())
      return;
    this->publish_state((block.registers[this->index_] & this->bitmask_) != 0);
  }

 protected:
  uint8_t index_{0};
  uint16_t bitmask_{0xFFFF};
};

}  // namespace esphome::modbus_sniffer
