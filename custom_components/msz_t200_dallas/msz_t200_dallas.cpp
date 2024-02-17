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
	
	ESP_LOGCONFIG(TAG, "set_parent_was: %u parent_: %p" , set_parent_was ? 1 : 0, this->parent_);
	
	
	uint32_t update_interval = get_update_interval();
	ESP_LOGCONFIG(TAG, "update_interval: %u" , update_interval);
	
	
	temperature += 0.05f;
	if (this->temperature_sensor_ != nullptr) {
	   ESP_LOGCONFIG(TAG, "publish temperature: %f" , temperature);
      this->temperature_sensor_->publish_state(temperature);
	}
	this->status_clear_warning();
#if 0
  uint8_t data[5];
  if (!this->read_data_(data)) {
    this->status_set_warning();
    return;
  }
  const uint16_t raw_temperature = uint16_t(data[2]) * 10 + (data[3] & 0x7F);
  float temperature = raw_temperature / 10.0f;
  if ((data[3] & 0x80) != 0) {
    // negative
    temperature *= -1;
  }

  const uint16_t raw_humidity = uint16_t(data[0]) * 10 + data[1];
  float humidity = raw_humidity / 10.0f;

  ESP_LOGD(TAG, "Got temperature=%.2f°C humidity=%.2f%%", temperature, humidity);
  if (this->temperature_sensor_ != nullptr)
    this->temperature_sensor_->publish_state(temperature);
  if (this->humidity_sensor_ != nullptr)
    this->humidity_sensor_->publish_state(humidity);
  this->status_clear_warning();
#endif
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

}  // namespace msz_t200_dallas
}  // namespace esphome
