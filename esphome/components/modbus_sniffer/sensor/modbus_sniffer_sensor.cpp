#include "modbus_sniffer_sensor.h"

#include "esphome/core/log.h"

namespace esphome::modbus_sniffer {

static const char *const TAG = "modbus_sniffer.sensor";

void ModbusSnifferStatisticSensor::dump_config() { LOG_SENSOR("", "Modbus Sniffer Statistic", this); }

}  // namespace esphome::modbus_sniffer
