#ifndef __MSZ_T200_SENS2__
#define __MSZ_T200_SENS2__

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"

#include "esphome/components/msz_t200_device/msz_t200_device.h"


namespace esphome {
namespace msz_t200_sens2 {



class MszT200Sens2 : public PollingComponent {
 public:
  
  MszT200Sens2() : PollingComponent(5000) { }
 
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;
  
  void set_parent(msz_t200_device::MszT200Base *parent);
  void set_bus(uint8_t unit_id, uint8_t module_id, uint8_t bus_id) { unit_id_ = unit_id; module_id_ = module_id; bus_id_ = bus_id; }
  
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
  void set_humidity_sensor(sensor::Sensor *humidity_sensor) { humidity_sensor_ = humidity_sensor; }

 protected:
  bool read_data_(uint8_t *data);
  
   msz_t200_device::MszT200Base *parent_;
  
  uint8_t unit_id_;
  uint8_t module_id_;
  uint8_t bus_id_;
   
   
   

  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *humidity_sensor_{nullptr};
};

}  // namespace msz_t200_sens2
}  // namespace esphome

#endif /* __MSZ_T200_SENS2__ */





