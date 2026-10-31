import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import CONF_INDEX
from esphome.types import ConfigType

from .. import (
    CONF_BITMASK,
    listener_args,
    modbus_sniffer_ns,
    register_listener,
    register_schema,
)

DEPENDENCIES = ["modbus_sniffer"]

ModbusSnifferBinarySensor = modbus_sniffer_ns.class_(
    "ModbusSnifferBinarySensor", binary_sensor.BinarySensor
)

CONFIG_SCHEMA = register_schema(
    binary_sensor.binary_sensor_schema(ModbusSnifferBinarySensor),
    {cv.Optional(CONF_BITMASK, default=0xFFFF): cv.hex_uint16_t},
)


async def to_code(config: ConfigType) -> None:
    var = await binary_sensor.new_binary_sensor(config, *listener_args(config))
    cg.add(var.set_index(config[CONF_INDEX]))
    cg.add(var.set_bitmask(config[CONF_BITMASK]))
    await register_listener(var, config)
