// Implementation based on:
//  - ESPEasy: https://github.com/letscontrolit/ESPEasy/blob/mega/src/_P034_DHT12.ino
//  - DHT12_sensor_library: https://github.com/xreef/DHT12_sensor_library/blob/master/DHT12.cpp

#include "msz_t200_dallas.h"
#include "esphome/core/log.h"

#include "esphome/components/msz_t200_device/msz_t200_device.h"


namespace esphome {
namespace msz_t200_dallas {
	

static const char *const TAG = "msz_t200_dallas";


static bool set_parent_was = false;

void MszT200Dallas::set_parent(msz_t200_device::MszT200Base *parent) { 
	
	this->parent_ = parent; 
	set_parent_was = true;
}

float temperature = 10.0f;

void MszT200Dallas::update() {
	
	//ESP_LOGCONFIG(TAG, "set_parent_was: %u parent_: %p" , set_parent_was ? 1 : 0, this->parent_);
	
	
	uint32_t update_interval = get_update_interval();
//	ESP_LOGCONFIG(TAG, "update_interval: %u" , update_interval);
	

#if 0
	temperature += 0.05f;
	if (this->temperature_sensor_ != nullptr) {
	   ESP_LOGCONFIG(TAG, "publish temperature: %f" , temperature);
      this->temperature_sensor_->publish_state(temperature);
	}
#else
	uint32_t data = this->parent_->get_data(instance_ident_);
	if (this->temperature_sensor_ != nullptr) {
//	   ESP_LOGCONFIG(TAG, "publish data: %f  instance_ident_.channelL: %u " , data, instance_ident_.channel);
      this->temperature_sensor_->publish_state(data);
	}
#endif
	
	this->status_clear_warning();
}

void MszT200Dallas::setup() {
  ESP_LOGCONFIG(TAG, "Setting up DHT12...");
  
  
	
  

}

void MszT200Dallas::dump_config() {
  ESP_LOGD(TAG, "DHT12:");
//ms  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with DHT12 failed!");
  }
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
}

float MszT200Dallas::get_setup_priority() const { 
	return setup_priority::DATA; 
}
	
bool MszT200Dallas::read_data_(uint8_t *data) {
 /*ms
  if (!this->read_bytes(0, data, 5)) {
    ESP_LOGW(TAG, "Updating DHT12 failed!");
    return false;
  }
  uint8_t checksum = data[0] + data[1] + data[2] + data[3];
  if (data[4] != checksum) {
    ESP_LOGW(TAG, "DHT12 Checksum invalid!");
    return false;
  }
*/
  return true;
}

void MszT200Dallas::set_inst_ident(uint8_t unit_id, uint8_t module_id, uint8_t channel_id) {
	 
	ESP_LOGCONFIG(TAG, "Enter uint_id: %u module_id: %u channel_id: %u" , unit_id, module_id, channel_id);
	this->instance_ident_.unit_id = unit_id; 
	this->instance_ident_.module_id = module_id; 
	this->instance_ident_.channel_id = channel_id; 
}

}  // namespace msz_t200_dallas
}  // namespace esphome
