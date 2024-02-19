#include "esphome/core/log.h"
#include "msz_t200_device.h"


namespace esphome {
namespace msz_t200_device {
	

static const char *TAG = "msz_t200_device";

static const uint8_t msz_t200_header_first = 0x4D;
static const uint8_t msz_t200_header_second = 0xD3;
static const uint32_t msz_t200_register_number = 0x01FFFFFF;
static const uint8_t msz_t200_single_operation_word_length_max = 128;

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

	uint32_t test_recv_val = 0x00000000;

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
	
	uint32_t reg_read_value;
	MszRc rc;
	
	rc = read_reg(0x00456789, &reg_read_value);
	if (rc == MszRc::OK) {
		#if 0
		if (test_recv_val == 0) {
			test_recv_val = reg_read_value;
		} else {
			if (test_recv_val != reg_read_value) {
				ESP_LOGCONFIG(TAG, "Read register ERROR not expected value: 0x%08X should 0x%08X", test_recv_val, reg_read_value);
				test_recv_val = 0;
			}
		}
		test_recv_val++;
		#endif
	} else {
		ESP_LOGCONFIG(TAG, "Read register ERROR rc: %d", rc);
		test_recv_val = 0;
	}
	
	
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
	




uint32_t msz_t200_crc32_calc(uint8_t *data, const uint32_t data_len) {
		
	uint32_t								crc32 = 0;	
	uint32_t								byte_no;
	
	for (byte_no = 0; byte_no < data_len; byte_no++) {
		crc32 += *(data + byte_no);
	}	
//	ESP_LOGCONFIG(TAG, "crc32: %u", crc32);
	    
	return crc32;
}

void MszT200Device::set_32bdata_value(uint8_t *data, uint32_t value) {
	
	*(data + 0) = (uint8_t)(value >> 24);
	*(data + 1) = (uint8_t)(value >> 16);
	*(data + 2) = (uint8_t)(value >> 8);
	*(data + 3) = (uint8_t)(value >> 0);
}

uint32_t MszT200Device::get_32bdata_value(const uint8_t *data) {

	uint32_t								_32bdata_value = 0;

	_32bdata_value |= (uint32_t)(*(data + 0) << 24);
	_32bdata_value |= (uint32_t)(*(data + 1) << 16);
	_32bdata_value |= (uint32_t)(*(data + 2) << 8);
	_32bdata_value |= (uint32_t)(*(data + 3) << 0);

	return _32bdata_value;
}

void MszT200Device::create_header(uint8_t *data, const uint32_t reg_addr, const bool write_operation, const uint32_t burst_length) {
	
	*(data + 0) = msz_t200_header_first;
	*(data + 1) = msz_t200_header_second;
	this->set_32bdata_value((data + 2), (reg_addr & 0x00FFFFFF));
	*(data + 6) = write_operation ? (uint8_t)(1 << 7) : 0x00;
	*(data + 7) = burst_length;
}

MszRc MszT200Device::write_reg(const uint32_t reg_addr, const uint32_t reg_value) {

	MszRc									rc = MszRc::OK;
	uint8_t									data_wr[20];
	uint32_t								crc32;
	
	if (reg_addr < msz_t200_register_number) {
		this->create_header(data_wr, reg_addr, true, 1);
				
		this->enable();
		this->write_array(data_wr, 2); 
		this->set_32bdata_value(&data_wr[8], reg_value);
		this->write_array(&data_wr[2], 6);
		crc32 = msz_t200_crc32_calc(data_wr, 12);
		this->set_32bdata_value(&data_wr[12], crc32);
		this->write_array(&data_wr[8], 8);

		this->disable();
	} else {
		rc = MszRc::Inv_arg;
	}

	return rc;
}

MszRc MszT200Device::read_reg(const uint32_t reg_addr, uint32_t *reg_value) {

	MszRc									rc = MszRc::OK;
	uint8_t									data[20];
	uint32_t								crc32_recv, crc32_calc, byte_no;
	
	ESP_LOGCONFIG(TAG, "setup ENTER read_reg reg_addr: %u", reg_addr);
	
	if (reg_addr < msz_t200_register_number) {
		
		this->create_header(data, reg_addr, false, 1);
		this->enable();
		this->write_array(data, 2); 
		this->write_array(&data[2], 6);
		this->read_array(&data[8], 8);
		this->disable();
		ESP_LOGCONFIG(TAG, "%02X %02X %02X %02X %02X %02X %02X %02X ", data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);
		crc32_recv = get_32bdata_value(&data[12]);
		crc32_calc = msz_t200_crc32_calc(data, 12);
		if (crc32_recv != crc32_calc) {
			rc = MszRc::Inv_crc;
		}
	} else {
		rc = MszRc::Inv_arg;
	}

	return rc;
}


void MszT200Device::update_reg(uint8_t pin, bool pin_value, uint8_t reg_addr) {

}


}  // namespace msz_t200_device
}  // namespace esphome
