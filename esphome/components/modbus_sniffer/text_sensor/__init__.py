import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import CONF_COUNT, CONF_FORMAT, CONF_INDEX
from esphome.types import ConfigType

from .. import listener_args, modbus_sniffer_ns, register_listener, register_schema

DEPENDENCIES = ["modbus_sniffer"]

ModbusSnifferTextSensor = modbus_sniffer_ns.class_(
    "ModbusSnifferTextSensor", text_sensor.TextSensor
)
TextFormat = modbus_sniffer_ns.enum("TextFormat", is_class=True)
TEXT_FORMATS = {
    "HEX": TextFormat.HEX,
    "ASCII": TextFormat.ASCII,
}

CONFIG_SCHEMA = register_schema(
    text_sensor.text_sensor_schema(ModbusSnifferTextSensor),
    {
        cv.Optional(CONF_FORMAT, default="HEX"): cv.enum(TEXT_FORMATS, upper=True),
        # Registers to include from `index`; 0 = to the end of the block.
        cv.Optional(CONF_COUNT, default=0): cv.int_range(min=0, max=125),
    },
)


async def to_code(config: ConfigType) -> None:
    var = await text_sensor.new_text_sensor(config, *listener_args(config))
    cg.add(var.set_index(config[CONF_INDEX]))
    cg.add(var.set_count(config[CONF_COUNT]))
    cg.add(var.set_format(config[CONF_FORMAT]))
    await register_listener(var, config)
