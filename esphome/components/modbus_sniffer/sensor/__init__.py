import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ADDRESS,
    CONF_INDEX,
    CONF_STATE_CLASS,
    CONF_UNIT_OF_MEASUREMENT,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
)
from esphome.types import ConfigType

from .. import (
    CONF_BITMASK,
    CONF_MODBUS_SNIFFER_ID,
    CONF_VALUE_TYPE,
    VALUE_TYPES,
    ModbusSniffer,
    listener_args,
    modbus_sniffer_ns,
    register_listener,
    register_schema,
)

DEPENDENCIES = ["modbus_sniffer"]

CONF_STATISTIC = "statistic"

ModbusSnifferSensor = modbus_sniffer_ns.class_("ModbusSnifferSensor", sensor.Sensor)
ModbusSnifferStatisticSensor = modbus_sniffer_ns.class_(
    "ModbusSnifferStatisticSensor", sensor.Sensor, cg.PollingComponent
)
Statistic = modbus_sniffer_ns.enum("Statistic", is_class=True)

STATISTICS = {
    "rx_bytes": Statistic.RX_BYTES,
    "frames": Statistic.FRAMES,
    "skipped_bytes": Statistic.SKIPPED_BYTES,
    "crc_errors": Statistic.CRC_ERRORS,
    "unknown_frames": Statistic.UNKNOWN_FRAMES,
    "orphan_replies": Statistic.ORPHAN_REPLIES,
    "rx_buffer_peak": Statistic.RX_BUFFER_PEAK,
    "requests": Statistic.REQUESTS,
    "replies": Statistic.REPLIES,
    "unanswered": Statistic.UNANSWERED,
    "exceptions": Statistic.EXCEPTIONS,
}
# Statistics that can be limited to one bus address.
PER_ADDRESS = {"requests", "replies", "unanswered", "exceptions"}
BYTE_STATISTICS = {"rx_bytes", "skipped_bytes", "rx_buffer_peak"}

REGISTER_SENSOR_SCHEMA = register_schema(
    sensor.sensor_schema(ModbusSnifferSensor),
    {
        cv.Optional(CONF_VALUE_TYPE, default="U_WORD"): cv.enum(VALUE_TYPES, upper=True),
        # Publishes `register & bitmask` instead of a decoded value.
        cv.Optional(CONF_BITMASK): cv.hex_uint16_t,
    },
)

_STATISTIC_SCHEMA = (
    sensor.sensor_schema(
        ModbusSnifferStatisticSensor,
        accuracy_decimals=0,
        state_class=STATE_CLASS_TOTAL_INCREASING,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    )
    .extend(
        {
            cv.GenerateID(CONF_MODBUS_SNIFFER_ID): cv.use_id(ModbusSniffer),
            cv.Required(CONF_STATISTIC): cv.enum(STATISTICS, lower=True),
            # Limits a per-address statistic to one address; all addresses by default.
            cv.Optional(CONF_ADDRESS): cv.int_range(min=0, max=247),
        }
    )
    .extend(cv.polling_component_schema("60s"))
)


def _statistic_schema(config: ConfigType) -> ConfigType:
    config = dict(config)
    statistic = str(config.get(CONF_STATISTIC, "")).lower()
    if statistic in BYTE_STATISTICS:
        config.setdefault(CONF_UNIT_OF_MEASUREMENT, "B")
    if statistic == "rx_buffer_peak":
        config.setdefault(CONF_STATE_CLASS, STATE_CLASS_MEASUREMENT)
    config = _STATISTIC_SCHEMA(config)
    if CONF_ADDRESS in config and str(config[CONF_STATISTIC]) not in PER_ADDRESS:
        raise cv.Invalid(
            f"'{CONF_ADDRESS}' only applies to {', '.join(sorted(PER_ADDRESS))}",
            path=[CONF_ADDRESS],
        )
    return config


def _sensor_schema(config):
    # `statistic:` selects a counter of the sniffer itself; otherwise the sensor reads a register.
    if isinstance(config, dict) and CONF_STATISTIC in config:
        return _statistic_schema(config)
    return REGISTER_SENSOR_SCHEMA(config)


CONFIG_SCHEMA = _sensor_schema


async def to_code(config: ConfigType) -> None:
    if CONF_STATISTIC in config:
        var = await sensor.new_sensor(config)
        await cg.register_component(var, config)
        hub = await cg.get_variable(config[CONF_MODBUS_SNIFFER_ID])
        cg.add(var.set_parent(hub))
        cg.add(var.set_statistic(config[CONF_STATISTIC]))
        if CONF_ADDRESS in config:
            cg.add(var.set_address(config[CONF_ADDRESS]))
        return
    var = await sensor.new_sensor(config, *listener_args(config))
    cg.add(var.set_index(config[CONF_INDEX]))
    cg.add(var.set_value_type(config[CONF_VALUE_TYPE]))
    if CONF_BITMASK in config:
        cg.add(var.set_bitmask(config[CONF_BITMASK]))
    await register_listener(var, config)
