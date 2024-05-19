import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import (
    CONF_ID,
    CONF_INPUT,
    CONF_NUMBER,
    CONF_MODE,
    CONF_INVERTED,
    CONF_INTERRUPT,
    CONF_OPEN_DRAIN_INTERRUPT,
    CONF_OUTPUT,
    CONF_PULLUP,
)
from esphome.core import coroutine

CODEOWNERS = ["@msz"]


CONF_MSZT200_DEV = "msz_t200_device"
CONF_UNIT_ID = "unit_id"
CONF_MODULE_ID = "module_id"
CONF_INST_ID = "channel_id"


msz_t200_gpio_ns = cg.esphome_ns.namespace("msz_t200_gpio")

msz_t200GPIOPin = msz_t200_gpio_ns.class_("MszT200GPIOPin", cg.GPIOPin)
msz_t200InterruptMode = msz_t200_gpio_ns.enum("MszT200InterruptMode")

msz_t200_device_ns = cg.esphome_ns.namespace('msz_t200_device')
MszT200Base = msz_t200_device_ns.class_('MszT200Base', cg.Component)

MSZ_T200_INTERRUPT_MODES = {
    "NO_INTERRUPT": msz_t200InterruptMode.MszT200_NO_INTERRUPT,
    "CHANGE": msz_t200InterruptMode.MszT200_CHANGE,
    "RISING": msz_t200InterruptMode.MszT200_RISING,
    "FALLING": msz_t200InterruptMode.MszT200_FALLING,
}

MSZ_T200_CONFIG_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_OPEN_DRAIN_INTERRUPT, default=False): cv.boolean,
    }
).extend(cv.COMPONENT_SCHEMA)


@coroutine
async def register_msz_t200(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_open_drain_ints(config[CONF_OPEN_DRAIN_INTERRUPT]))
    return var


def validate_mode(value):
    if not (value[CONF_INPUT] or value[CONF_OUTPUT]):
        raise cv.Invalid("Mode must be either input or output")
    if value[CONF_INPUT] and value[CONF_OUTPUT]:
        raise cv.Invalid("Mode must be either input or output")
    if value[CONF_PULLUP] and not value[CONF_INPUT]:
        raise cv.Invalid("Pullup only available with input")
    return value




MSZT200_PIN_SCHEMA = pins.gpio_base_schema(
    msz_t200GPIOPin,
    cv.int_range(min=1, max=128),
    modes=[CONF_INPUT, CONF_OUTPUT, CONF_PULLUP],
    mode_validator=validate_mode,
    invertable=True,
).extend(
    {
        cv.Required(CONF_MSZT200_DEV): cv.use_id(MszT200Base),
        cv.Required(CONF_UNIT_ID): cv.int_range(min=1, max=4),
        cv.Required(CONF_MODULE_ID): cv.int_range(min=1, max=4),
        cv.Required(CONF_INST_ID): cv.int_range(min=1, max=8),
        cv.Optional(CONF_INTERRUPT, default="NO_INTERRUPT"): cv.enum(
            MSZ_T200_INTERRUPT_MODES, upper=True
        ),
    }
)

@pins.PIN_SCHEMA_REGISTRY.register(CONF_MSZT200_DEV, MSZT200_PIN_SCHEMA)
async def msz_t200_pin_to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    parent = await cg.get_variable(config[CONF_MSZT200_DEV])

    cg.add(var.set_parent(parent))

    unit = config[CONF_UNIT_ID]
    module = config[CONF_MODULE_ID]
    num = config[CONF_INST_ID]
    cg.add(var.set_inst_ident(unit, module, num))
    cg.add(var.set_inverted(config[CONF_INVERTED]))
    cg.add(var.set_flags(pins.gpio_flags_expr(config[CONF_MODE])))
    cg.add(var.set_interrupt_mode(config[CONF_INTERRUPT]))
    return var
