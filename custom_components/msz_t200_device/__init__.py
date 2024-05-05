import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import spi
from esphome.components import text_sensor
from esphome.const import CONF_ID, CONF_TEXT_SENSORS


AUTO_LOAD = ["msz_t200_gpio", "msz_t200_sens2", "msz_t200_dallas"]

DEPENDENCIES = ['spi']

CONF_UNIT_MODULE_1_1 = "unit_module_1.1"
CONF_UNIT_MODULE_1_2 = "unit_module_1.2"
CONF_UNIT_MODULE_1_3 = "unit_module_1.3"
CONF_UNIT_MODULE_1_4 = "unit_module_1.4"

msz_t200_device_ns = cg.esphome_ns.namespace('msz_t200_device')
MszT200Base = msz_t200_device_ns.class_('MszT200Base', cg.Component)
MszT200Device = msz_t200_device_ns.class_('MszT200Device', MszT200Base, spi.SPIDevice)
MszT200ModuleType = msz_t200_device_ns.enum("MszT200ModuleType")


MSZ_T200_MODULE_TYPE = {
    "none": MszT200ModuleType.NoneEmpty,
    "input8": MszT200ModuleType.Input8,
    "output8": MszT200ModuleType.Output8,
    "dallas4": MszT200ModuleType.Dallas4,
}

CONFIG_SCHEMA = ( 
	cv.Schema(
		{
			cv.GenerateID(): cv.declare_id(MszT200Device),
			cv.Optional(CONF_TEXT_SENSORS): cv.ensure_list(
				text_sensor.text_sensor_schema()
			),
			cv.Optional(CONF_UNIT_MODULE_1_1, default="none"): cv.enum(MSZ_T200_MODULE_TYPE),
			cv.Optional(CONF_UNIT_MODULE_1_2, default="none"): cv.enum(MSZ_T200_MODULE_TYPE),
			cv.Optional(CONF_UNIT_MODULE_1_3, default="none"): cv.enum(MSZ_T200_MODULE_TYPE),
			cv.Optional(CONF_UNIT_MODULE_1_4, default="none"): cv.enum(MSZ_T200_MODULE_TYPE),
		}
	)
	.extend(cv.COMPONENT_SCHEMA)
	.extend(spi.spi_device_schema(cs_pin_required=True))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)

    cg.add(var.set_conf_mod1(1,1,config[CONF_UNIT_MODULE_1_1]))
    cg.add(var.set_conf_mod1(1,2,config[CONF_UNIT_MODULE_1_2]))
    cg.add(var.set_conf_mod1(1,3,config[CONF_UNIT_MODULE_1_3]))
    cg.add(var.set_conf_mod1(1,4,config[CONF_UNIT_MODULE_1_4]))
    
    for i, conf in enumerate(config[CONF_TEXT_SENSORS]):
        text = cg.Pvariable(conf[CONF_ID], var.get_new_text_sensor(i))
        await text_sensor.register_text_sensor(text, conf)
