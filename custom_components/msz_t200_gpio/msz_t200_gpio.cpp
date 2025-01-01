#include "msz_t200_gpio.h"
#include "esphome/core/log.h"

#include "esphome/components/msz_t200_device/msz_t200_device.h"


namespace esphome {
namespace msz_t200_gpio {

	
static const char *TAG = "msz_t200_gpio";


void MszT200GPIOPin::set_ha_user_conf_inst_ident(const uint8_t unit_id, const uint8_t module_id, const uint8_t channel_id) {
	
	msz_t200_device::MszRc					rc;
		
	inst_ident_.unit_no = MSZ_T200_USER_UNIT_TO_UNIT_NO(unit_id); 
	inst_ident_.module_no = MSZ_T200_USER_MODULE_TO_MODULE_NO(module_id); 
	inst_ident_.channel_no = MSZ_T200_USER_CHANNEL_TO_CHANNEL_NO(channel_id); 
	if (parent_) {
		rc = parent_->validate_inst_ident(inst_ident_);
		if (rc != msz_t200_device::MszRc::OK) {
			ESP_LOGCONFIG(TAG, "New instance ident validation fail rc: %d", rc);
		}
	}
}

void MszT200GPIOPin::set_ha_user_conf_parent(msz_t200_device::MszT200Base *parent) { 
	
	parent_ = parent; 
}

void MszT200GPIOPin::set_ha_user_conf_inverted(const bool inverted) { 
	
	inverted_ = inverted; 
}

void MszT200GPIOPin::set_ha_user_conf_flags(gpio::Flags flags) { 
	
	flags_ = flags; 
}

void MszT200GPIOPin::set_ha_user_conf_interrupt_mode(MszT200InterruptMode interrupt_mode) { 
	
	interrupt_mode_ = interrupt_mode; 
}

MszT200GPIOPin::MszT200GPIOPin() : parent_{NULL}, inverted_{false}, flags_{gpio::Flags::FLAG_NONE}, interrupt_mode_{MszT200_NO_INTERRUPT} {
	
	inst_ident_.unit_no = MSZ_T200_UNITS_PER_DEVICE;
	inst_ident_.module_no = MSZ_T200_MODULES_PER_UNIT;
	inst_ident_.channel_no = MSZ_T200_CHANNELS_PER_MODULE;
}

void MszT200GPIOPin::setup() { 
	
	pin_mode(flags_); 
}

void MszT200GPIOPin::pin_mode(gpio::Flags flags) { 
	

}

bool MszT200GPIOPin::digital_read() { 
	
	bool ret_val;

	if (parent_) {
		parent_->msz_t200_gpio_read(this->inst_ident_, ret_val);
	} else {
		ret_val = false;
	}
	
	return ret_val;
}

void MszT200GPIOPin::digital_write(bool value) { 
	
	if (parent_) {
		parent_->msz_t200_gpio_write(this->inst_ident_, value != this->inverted_);
	}
}

std::string MszT200GPIOPin::dump_summary() const {
	
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%u via MszT200", this->inst_ident_.channel_no);
  return buffer;
  
}

}  // namespace msz_t200_gpio
}  // namespace esphome
