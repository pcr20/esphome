from esphome import automation
import esphome.codegen as cg
from esphome.components import uart
import esphome.config_validation as cv
from esphome.const import CONF_ADDRESS, CONF_BLOCK, CONF_ID, CONF_INDEX
from esphome.types import ConfigType

CODEOWNERS = ["@pcr20"]
DEPENDENCIES = ["uart"]
MULTI_CONF = True

modbus_sniffer_ns = cg.esphome_ns.namespace("modbus_sniffer")
ModbusSniffer = modbus_sniffer_ns.class_("ModbusSniffer", cg.Component, uart.UARTDevice)
ValueType = modbus_sniffer_ns.enum("ValueType", is_class=True)

CONF_MODBUS_SNIFFER_ID = "modbus_sniffer_id"
CONF_FUNCTION_CODE = "function_code"
CONF_VALUE_TYPE = "value_type"
CONF_BITMASK = "bitmask"
CONF_RESYNC_TIMEOUT = "resync_timeout"
CONF_LOG_FRAMES = "log_frames"
CONF_STATS_INTERVAL = "stats_interval"
CONF_ON_BLOCK = "on_block"
CONF_ON_FRAME = "on_frame"

VALUE_TYPES = {
    "U_WORD": ValueType.U_WORD,
    "S_WORD": ValueType.S_WORD,
    "U_DWORD": ValueType.U_DWORD,
    "S_DWORD": ValueType.S_DWORD,
    "U_DWORD_R": ValueType.U_DWORD_R,
    "S_DWORD_R": ValueType.S_DWORD_R,
    "FP32": ValueType.FP32,
    "FP32_R": ValueType.FP32_R,
    "BCD": ValueType.BCD,
}

FUNCTION_WRITE_MULTIPLE_REGISTERS = 0x10
FUNCTION_READ_HOLDING_REGISTERS = 0x03

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ModbusSniffer),
            cv.Optional(
                CONF_RESYNC_TIMEOUT, default="50ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_LOG_FRAMES, default=False): cv.boolean,
            cv.Optional(
                CONF_STATS_INTERVAL, default="60s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_ON_BLOCK): automation.validate_automation({}),
            cv.Optional(CONF_ON_FRAME): automation.validate_automation({}),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "modbus_sniffer", require_rx=True
)


def _default_function_code(config: ConfigType) -> ConfigType:
    # Broadcasts (address 0) are always writes; other addresses are usually read replies.
    if CONF_FUNCTION_CODE not in config:
        config[CONF_FUNCTION_CODE] = (
            FUNCTION_WRITE_MULTIPLE_REGISTERS
            if config[CONF_ADDRESS] == 0
            else FUNCTION_READ_HOLDING_REGISTERS
        )
    return config


# Locates one register in a block: register `index` is Modbus address `block + index`.
REGISTER_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MODBUS_SNIFFER_ID): cv.use_id(ModbusSniffer),
        cv.Required(CONF_ADDRESS): cv.int_range(min=0, max=247),
        cv.Optional(CONF_FUNCTION_CODE): cv.hex_int_range(min=1, max=0x7F),
        cv.Required(CONF_BLOCK): cv.hex_uint16_t,
        cv.Optional(CONF_INDEX, default=0): cv.int_range(min=0, max=124),
    }
)


def register_schema(base: cv.Schema, extra: dict | None = None):
    return cv.All(base.extend(REGISTER_SCHEMA).extend(extra or {}), _default_function_code)


async def register_listener(var, config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_MODBUS_SNIFFER_ID])
    cg.add(hub.register_listener(var))


def listener_args(config: ConfigType) -> list:
    return [config[CONF_ADDRESS], config[CONF_FUNCTION_CODE], config[CONF_BLOCK]]


_CALLBACK_AUTOMATIONS = (
    automation.CallbackAutomation(
        CONF_ON_BLOCK,
        "add_on_block_callback",
        [
            (cg.uint8, "address"),
            (cg.uint8, "function"),
            (cg.uint16, "start"),
            (cg.std_vector.template(cg.uint16), "registers"),
        ],
    ),
    automation.CallbackAutomation(
        CONF_ON_FRAME,
        "add_on_frame_callback",
        [(cg.std_vector.template(cg.uint8), "frame")],
    ),
)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_resync_timeout(config[CONF_RESYNC_TIMEOUT]))
    cg.add(var.set_log_frames(config[CONF_LOG_FRAMES]))
    cg.add(var.set_stats_interval(config[CONF_STATS_INTERVAL]))
    await automation.build_callback_automations(var, config, _CALLBACK_AUTOMATIONS)
