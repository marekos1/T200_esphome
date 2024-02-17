#include "esphome/core/log.h"
#include "msz_t200_device.h"


namespace esphome {
namespace msz_t200_device {
	

static const char *TAG = "msz_t200_device";


float MszT200Base::get_setup_priority() const { 
	
	return setup_priority::IO;
}

MszRc MszT200Base::gpio_read(const MszT200InstanceIdent& inst_ident, bool& gpio_state) {
		
	MszRc								rc = MszRc::OK;
		
	if (inst_ident.channel < 32) {
		gpio_state = gpio_state_[inst_ident.channel];
	} else {
		rc = MszRc::Inv_arg;
	}
	
	return rc;
}

static struct timeval gpio_change_test[gpio_total]; // = { .tv_sec = 0, .tv_usec = 0};
static const uint32_t gpio_change_interval[gpio_total] = {
	3, 5, 7, 9, 11, 13, 15, 17,
	1, 2, 1, 3, 1, 4, 1, 5,
	1, 3, 2, 6, 2, 1, 1, 1,
	10, 5, 7, 9, 8, 2, 16, 32
};

static bool msz_t200_device_digital_read_test(const struct timeval& current_time, const uint32_t gpio_change_interval, 
											  const bool prev_state, struct timeval& prev_change_time) {
	
	bool ret_val = false;
	
	if (current_time.tv_sec > (prev_change_time.tv_sec + gpio_change_interval)) {
		prev_change_time.tv_sec = current_time.tv_sec;
		ret_val = !prev_state;
	} else {
		ret_val = prev_state;
	}

	return ret_val;
}

void MszT200Device::loop()  {
#if 0
	struct timeval tv;
	static struct timeval last = { .tv_sec = 0, .tv_usec = 0};
//	static struct timeval spi_last = { .tv_sec = 0, .tv_usec = 0};
	
    gettimeofday(&tv, NULL);
    
    if (tv.tv_sec > (last.tv_sec + 3)) {
		last = tv;
		ESP_LOGCONFIG(TAG, "unit1_module_type: %u %u %u %u", this->unit1_module_type[0], this->unit1_module_type[1], this->unit1_module_type[2], this->unit1_module_type[3]);
	}
#endif
	static uint32_t ctr = 0;
#if 0
//	if (tv.tv_sec > last.tv_sec) {
//		spi_last = tv;
		this->enable();
		this->transfer_byte(0xAA);
		this->transfer_byte(0x55);
//		this->transfer_byte(tv.tv_sec);
		this->transfer_byte(ctr++);
		this->transfer_byte(static_cast<uint8_t>(this->unit1_module_type[0]));
		this->disable();
//}
#else

	write_reg(0x00456789, ctr++);
#endif

	uint32_t					gpio_no;
	struct timeval tv;
	
    gettimeofday(&tv, NULL);
	for (gpio_no = 0; gpio_no < gpio_total; gpio_no++) {
		this->gpio_state_[gpio_no] = msz_t200_device_digital_read_test(tv, gpio_change_interval[gpio_no], this->gpio_state_[gpio_no], this->change_time_[gpio_no]);
	}
	
}

void MszT200Device::setup() {
    
    ESP_LOGCONFIG(TAG, "setup ENTER msz_t200_device");
    this->spi_setup();
}

void MszT200Device::dump_config() {
	
    ESP_LOGCONFIG(TAG, "msz t200 component");
    LOG_PIN("  CS Pin:", this->cs_);
}

void MszT200Device::set_conf_mod1(uint8_t unit, uint8_t module, MszT200ModuleType mode_type) {

    ESP_LOGCONFIG(TAG, "Enter unit module: %u.%u mode_type: %u", unit, module, mode_type);
    if (unit == 1) {
		if (module && (module < 5)) {
			this->unit1_module_type[module - 1] = mode_type;
		}
	}
}
	
bool MszT200Device::read_reg(uint8_t reg, uint8_t *value) {

    ESP_LOGCONFIG(TAG, "setup ENTER read_reg reg: %u value: %u", reg, *value);
    
	return true;
}

uint32_t msz_t200_crc32_calc(uint8_t *data, const uint32_t data_len) {
		
	uint32_t								crc32 = 0;	
	uint32_t								byte_no;
	
	for (byte_no = 0; byte_no < data_len; byte_no++) {
		crc32 += *(data + byte_no);
	}	
	
	return crc32;
}

MszRc MszT200Device::write_reg(uint32_t reg_addr, uint32_t value) {

	MszRc									rc = MszRc::OK;
	uint8_t									data_wr[20];
	uint32_t								crc32;
	
	data_wr[0] = 0x4D;
	data_wr[1] = 0xD3;
	data_wr[2] = 0x00;
	data_wr[3] = (uint8_t)(reg_addr >> 16);
	data_wr[4] = (uint8_t)(reg_addr >> 8);
	data_wr[5] = (uint8_t)reg_addr;
	data_wr[6] = (uint8_t)(1 << 7);
	data_wr[7] = 0x01;
	
	data_wr[8] = 0xFF;
	data_wr[9] = 0xFF;
	data_wr[10] = 0xFF;
	data_wr[11] = 0xFF;
	
	data_wr[12] = (uint8_t)(value >> 24);
	data_wr[13] = (uint8_t)(value >> 16);
	data_wr[14] = (uint8_t)(value >> 8);
	data_wr[15] = (uint8_t)(value >> 0);
	
	crc32 = msz_t200_crc32_calc(data_wr, 16);
	memcpy(&data_wr[16], &crc32, 4);
	
	this->enable();
	this->write_array(data_wr, 20); 
	this->disable();
	
	return rc;
}

void MszT200Device::update_reg(uint8_t pin, bool pin_value, uint8_t reg_addr) {

}


}  // namespace msz_t200_device
}  // namespace esphome
