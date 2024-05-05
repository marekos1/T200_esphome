#include "msz_t200_gpio.h"
#include "esphome/core/log.h"

#include "esphome/components/msz_t200_device/msz_t200_device.h"


namespace esphome {
namespace msz_t200_gpio {

	
static const char *TAG = "msz_t200_gpio";

static bool set_parent_was = false;

static bool pin_state[5];


void MszT200GPIOPin::set_parent(msz_t200_device::MszT200Base *parent) { 
	
	this->parent_ = parent; 
	set_parent_was = true;
}

void add_us(struct timeval& tv, const uint32_t us) {
	
	struct timeval	enter_val = tv;
	uint64_t 		add_us;

	add_us = tv.tv_usec;
	add_us += us;
	 
	tv.tv_sec += add_us / 1000000;
	tv.tv_usec = add_us % 1000000;	
//	ESP_LOGCONFIG(TAG, "add_us: us: %u enter_val %u.%u tv %u.%u", us, enter_val.tv_sec, enter_val.tv_usec, tv.tv_sec, tv.tv_usec);
}

bool comapre_time(const struct timeval& current, const struct timeval& next) {
	
	bool ret_val = false;
		
	if (current.tv_sec > next.tv_sec) {
		ret_val = true;
	} else if (((current.tv_sec == next.tv_sec)) && (current.tv_usec > next.tv_usec)) {
		ret_val = true;
	}
//	ESP_LOGCONFIG(TAG, "comapre_time: current %u.%u next %u.%u ret_val: %u", current->tv_sec, current->tv_usec, next->tv_sec, next->tv_usec, ret_val ? 1 : 0);
	
	return ret_val;
}

bool MszT200GPIOPin::digital_read_prv(uint8_t unit_id, uint8_t module_id, uint8_t pin) {
	
	bool ret_val = false;
	struct timeval tv;
	static struct timeval pin2_change = { .tv_sec = 0, .tv_usec = 0};
	static struct timeval pin3_change = { .tv_sec = 0, .tv_usec = 0};
	
    gettimeofday(&tv, NULL);
	//ESP_LOGCONFIG(TAG, "Enter digital_read pin: %u time %u.%u", pin, tv.tv_sec, tv.tv_usec);
	switch (pin) {
	case 2:
		if ((pin2_change.tv_sec == 0) && (pin2_change.tv_usec == 0)) {
			pin2_change = tv;
			add_us(pin2_change, 1750000);
			ESP_LOGCONFIG(TAG, "digital_read pin: 2 set pin2_change %u.%u", pin2_change.tv_sec, pin2_change.tv_usec);
		} else {
			if (comapre_time(tv, pin2_change)) {
				pin2_change = tv;
				add_us(pin2_change, 1750000);
				if (pin_state[2]) {
					pin_state[2] = false;
				} else {
					pin_state[2] = true;
				}
				ESP_LOGCONFIG(TAG, "digital_read pin: pin %u state change to %u at time %u.%u", pin, pin_state[2] ? 1 : 0, tv.tv_sec, tv.tv_usec);
			} else {
			//	ESP_LOGCONFIG(TAG, "digital_read pin:compare false tv: %u.%u pin2_change %u.%u", tv.tv_sec, tv.tv_usec, pin2_change.tv_sec, pin2_change.tv_usec);
			}
		}
		ret_val = pin_state[2];
		break;
	case 3:
		if ((pin3_change.tv_sec == 0) && (pin3_change.tv_usec == 0)) {
			pin3_change = tv;
			add_us(pin3_change, 1950000);
			ESP_LOGCONFIG(TAG, "digital_read pin: 2 set pin3_change %u.%u", pin3_change.tv_sec, pin3_change.tv_usec);
		} else {
			if (comapre_time(tv, pin3_change)) {
				pin3_change = tv;
				add_us(pin3_change, 1950000);
				if (pin_state[3]) {
					pin_state[3] = false;
				} else {
					pin_state[3] = true;
				}
				ESP_LOGCONFIG(TAG, "digital_read pin: pin %u state change to %u at time %u.%u", pin, pin_state[3] ? 1 : 0, tv.tv_sec, tv.tv_usec);
				ESP_LOGCONFIG(TAG, "set_parent_was: %u parent_: %p" , set_parent_was ? 1 : 0, this->parent_);
				
				uint8_t reg_value = (uint8_t)tv.tv_usec;
			//	this->parent_->read_reg(tv.tv_sec, &reg_value);
			} else {
			//	ESP_LOGCONFIG(TAG, "digital_read pin:compare false tv: %u.%u pin3_change %u.%u", tv.tv_sec, tv.tv_usec, pin3_change.tv_sec, pin3_change.tv_usec);
			}
		}
		ret_val = pin_state[3];
		break;
	default:
		ESP_LOGCONFIG(TAG, "digital_read pin: Unknonw pin %u", pin);
		break;
	}

	return ret_val;
}

void MszT200GPIOPin::digital_write_prv(uint8_t pin, bool value) {

}

void MszT200GPIOPin::pin_mode_prv(uint8_t pin, gpio::Flags flags) {

}

void MszT200GPIOPin::pin_interrupt_mode_prv(uint8_t pin, msz_t200_gpio::MszT200InterruptMode interrupt_mode) {

}


void MszT200GPIOPin::setup() { 
	
	pin_mode(flags_); 
}

void MszT200GPIOPin::pin_mode(gpio::Flags flags) { 
	
	this->pin_mode_prv(this->instance_ident_.channel, flags);
}

bool MszT200GPIOPin::digital_read() { 
	
	bool ret_val;
	uint8_t reg_value;
	
//	ESP_LOGCONFIG(TAG, "Enter digital_read pin_ %d", this->pin_);
//	ret_val = this->parent_->read_reg(6, &reg_value);
	
//	return this->digital_read_prv(this->instance_ident_.unit_id, this->instance_ident_.module_id, this->instance_ident_.channel) != this->inverted_; 
	
	this->parent_->gpio_read(this->instance_ident_, ret_val);
	if (ret_val == true) {
		ESP_LOGCONFIG(TAG, "Read TRUE");
	}
	
	return ret_val;
}
void MszT200GPIOPin::digital_write(bool value) { 
	
	ESP_LOGCONFIG(TAG, "Enter digital_write channel %d value: %d", this->instance_ident_.channel, value);

#if 0	
	this->parent_->digital_write(this->instance_ident_.channel, value != this->inverted_);
#else
	this->digital_write_prv(this->instance_ident_.channel, value != this->inverted_);
#endif
}

std::string MszT200GPIOPin::dump_summary() const {
	
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%u via MszT200", this->instance_ident_.channel);
  return buffer;
  
}

}  // namespace msz_t200_gpio
}  // namespace esphome
