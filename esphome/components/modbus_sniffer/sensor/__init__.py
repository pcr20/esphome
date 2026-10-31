import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import CONF_INDEX
from esphome.types import ConfigType

from .. import (
    CONF_BITMASK,
    CONF_VALUE_TYPE,
    VALUE_TYPES,
    listener_args,
    modbus_sniffer_ns,
    register_listener,
    register_schema,
)

DEPENDENCIES = ["modbus_sniffer"]

ModbusSnifferSensor = modbus_sniffer_ns.class_("ModbusSnifferSensor", sensor.Sensor)

CONFIG_SCHEMA = register_schema(
    sensor.sensor_schema(ModbusSnifferSensor),
    {
        cv.Optional(CONF_VALUE_TYPE, default="U_WORD"): cv.enum(VALUE_TYPES, upper=True),
        # Publishes `register & bitmask` instead of a decoded value.
        cv.Optional(CONF_BITMASK): cv.hex_uint16_t,
    },
)


async def to_code(config: ConfigType) -> None:
    var = await sensor.new_sensor(config, *listener_args(config))
    cg.add(var.set_index(config[CONF_INDEX]))
    cg.add(var.set_value_type(config[CONF_VALUE_TYPE]))
    if CONF_BITMASK in config:
        cg.add(var.set_bitmask(config[CONF_BITMASK]))
    await register_listener(var, config)
