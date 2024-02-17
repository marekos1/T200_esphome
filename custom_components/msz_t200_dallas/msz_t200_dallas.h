#ifndef __MSZ_T200_DALLAS__
#define __MSZ_T200_DALLAS__

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"

#include "esphome/components/msz_t200_device/msz_t200_device.h"


namespace esphome {
namespace msz_t200_dallas {


class MszT200Dallas : public PollingComponent {
 public:
	void setup() override;
	void dump_config() override;
	float get_setup_priority() const override;
	void update() override;
  
	void set_parent(msz_t200_device::MszT200Base *parent);
	void set_inst_ident(uint8_t unit_id, uint8_t module_id, uint8_t channel) { 
		this->instance_ident_.unit_id = unit_id; 
		this->instance_ident_.module_id = module_id; 
		this->instance_ident_.channel = channel; }

	void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
	void set_humidity_sensor(sensor::Sensor *humidity_sensor) { humidity_sensor_ = humidity_sensor; }

 protected:
	bool read_data_(uint8_t *data);
  
	msz_t200_device::MszT200Base *parent_;
	msz_t200_device::MszT200InstanceIdent instance_ident_;
   
	sensor::Sensor *temperature_sensor_{nullptr};
	sensor::Sensor *humidity_sensor_{nullptr};
};


}  // namespace msz_t200_dallas
}  // namespace esphome

#endif /* __MSZ_T200_DALLAS__ */
