import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_HUMIDITY,
    CONF_ID,
    CONF_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_HUMIDITY,
)

CONF_MSZT200_DEV = "msz_t200_device"
CONF_UNIT_ID = "unit_id"
CONF_MODULE_ID = "module_id"
CONF_BUS_ID = "bus_id"

msz_t200_dallas_ns = cg.esphome_ns.namespace("msz_t200_dallas")
MszT200Dallas = msz_t200_dallas_ns.class_("MszT200Dallas", cg.PollingComponent)

msz_t200_device_ns = cg.esphome_ns.namespace('msz_t200_device')
MszT200Base = msz_t200_device_ns.class_('MszT200Base', cg.Component)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MszT200Dallas),
            cv.Required(CONF_MSZT200_DEV): cv.use_id(MszT200Base),
            cv.Required(CONF_UNIT_ID): cv.int_range(min=1, max=4),
			cv.Required(CONF_MODULE_ID): cv.int_range(min=1, max=4),
			cv.Required(CONF_BUS_ID): cv.int_range(min=1, max=4),
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_HUMIDITY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    )
    .extend(cv.polling_component_schema("3s"))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    parent = await cg.get_variable(config[CONF_MSZT200_DEV])
    cg.add(var.set_parent(parent))

    unit = config[CONF_UNIT_ID]
    module = config[CONF_MODULE_ID]
    bus = config[CONF_BUS_ID]
    cg.add(var.set_inst_ident(unit, module, bus))
    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))

    if CONF_HUMIDITY in config:
        sens = await sensor.new_sensor(config[CONF_HUMIDITY])
        cg.add(var.set_humidity_sensor(sens))
