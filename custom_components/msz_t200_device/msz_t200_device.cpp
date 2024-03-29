#include "esphome/core/log.h"
#include "msz_t200_device.h"


namespace esphome {
namespace msz_t200_device {
	

static const char *TAG = "msz_t200_device";

static const uint8_t msz_t200_header_first = 0x4D;
static const uint8_t msz_t200_header_second = 0xD3;
static const uint32_t msz_t200_register_number = 0x01FFFFFF;


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

bool timeval_compare(const struct timeval& lhs, const struct timeval& rhs) {
	
	bool ret_val = false;
	
    if (lhs.tv_sec == rhs.tv_sec) {
        ret_val = (lhs.tv_usec > rhs.tv_usec);
    } else {
        ret_val = (lhs.tv_sec > rhs.tv_sec);
	}
//    ESP_LOGCONFIG(TAG, "timeval_compare lhs: %u.%u rhs: %u.%u ret_val: %u", lhs.tv_sec, lhs.tv_usec, rhs.tv_sec, rhs.tv_usec, ret_val);
      
	return ret_val;
}

void timeval_add_us(struct timeval& tv, const uint32_t us) {
	
	uint32_t								add_sec;
	
	tv.tv_usec += us;
	if (tv.tv_usec >= 1000000) {
		add_sec = tv.tv_usec / 1000000;
		tv.tv_sec += add_sec;
		tv.tv_usec -= (add_sec * 1000000);
	}
}

void MszT200Device::test1()  {
	
	static uint32_t 						test_value = 0, read_value_ok = 0, read_value_err = 0;
	static bool								read = false;
	static  timeval							last_operation, last_stats_show;					
	uint32_t								reg_read_value;
	MszRc									rc;
	struct timeval 							current_tv;
	
    gettimeofday(&current_tv, NULL);
    if (timeval_compare(current_tv, this->startup_tv)) {
		
		if (timeval_compare(current_tv, last_operation)) {
			last_operation = current_tv;
			timeval_add_us(last_operation, 2000);
		}
		
		if (read) {
			rc = read_reg(0x00456789, &reg_read_value);
			if (rc == MszRc::OK) {
				if (reg_read_value == test_value) {
					read_value_ok++;
				} else {
					read_value_err++;
					ESP_LOGCONFIG(TAG, "ERROR read value: 0x%08X should 0x%08X read_ok_ctr: %u read_err_ctr: %u", &reg_read_value, test_value, this->stats.read_ok_ctr, this->stats.read_err_ctr);
				}
			} else {
				ESP_LOGCONFIG(TAG, "Read error rc: %d", rc);
			}
			test_value++;
			read = false;
		} else {
			rc = write_reg(0x00456789, test_value);
			if (rc == MszRc::OK) {
				this->stats.write_ok_ctr++;
			} else {
				this->stats.write_err_ctr++;
				ESP_LOGCONFIG(TAG, "Write error rc: %d", rc);
			}
			read = true;
		}
		
	}

	if (timeval_compare(current_tv, last_stats_show)) {
		last_stats_show = current_tv;
		last_stats_show.tv_sec += 3;
		ESP_LOGCONFIG(TAG, "Stats: Write ok: %8u err: %8u Read ok: %8u err: %8u",  this->stats.write_ok_ctr,  this->stats.write_err_ctr,  this->stats.read_ok_ctr,  this->stats.read_err_ctr);
	//	ESP_LOGCONFIG(TAG, "startup_tv %u.%u", this->startup_tv.tv_sec, this->startup_tv.tv_usec);
	}
	
}

void MszT200Device::loop()  {
	
	test1();
}

void MszT200Device::setup() {
    
    ESP_LOGCONFIG(TAG, "setup ENTER msz_t200_device");
    this->spi_setup();
    
    struct timeval 							current_tv;
	
    gettimeofday(&this->startup_tv, NULL);
    this->startup_tv.tv_sec += 10;
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
	    
	return crc32;
}


uint32_t MszT200Device::set_32bdata_value(uint8_t *data, const uint32_t value) {
	
	*(data + 0) = (uint8_t)(value >> 24);
	*(data + 1) = (uint8_t)(value >> 16);
	*(data + 2) = (uint8_t)(value >> 8);
	*(data + 3) = (uint8_t)(value >> 0);
	
	return 4;
}

uint32_t MszT200Device::get_32bdata_value(const uint8_t *data, uint32_t& value) {
				
	value = 0;
	value |= (uint32_t)(*(data + 0) << 24);
	value |= (uint32_t)(*(data + 1) << 16);
	value |= (uint32_t)(*(data + 2) << 8);
	value |= (uint32_t)(*(data + 3) << 0);

	return 4;
}

uint32_t MszT200Device::send_header(uint8_t *data, const uint32_t reg_addr, const bool write_operation, const uint32_t burst_length) {
	
	uint32_t									header_idx = 0;
	
	*(data + header_idx) = msz_t200_header_first;
	this->write_array(data + header_idx, 1);
	header_idx++;
	delayMicroseconds(20);
	
	*(data + header_idx) = msz_t200_header_second;
	this->write_array(data + header_idx, 1);
	header_idx++;
	delayMicroseconds(20);
			
	header_idx += this->set_32bdata_value((data + header_idx), (reg_addr & 0x00FFFFFF));
	*(data + header_idx++) = write_operation ? 0x80 : 0x00;
	*(data + header_idx++) = burst_length;
	this->write_array(data + 2, 6);
	if (write_operation) {
		delayMicroseconds(20);
	} else {
		delayMicroseconds(100);
	}
	
	return header_idx;
}

uint32_t MszT200Device::send_register_value(uint8_t *data, const uint32_t reg_value) {
	
	uint32_t									data_idx = 0;
	
	data_idx += this->set_32bdata_value(data, reg_value);
	this->write_array(data, data_idx);
	
	return data_idx;
}

uint32_t MszT200Device::recv_register_value(uint8_t *data, uint32_t& reg_value) {
	
	uint32_t									data_idx = 0;
	
	this->read_array(data, 4);
	data_idx += get_32bdata_value(data, reg_value);
	
	return data_idx;
}

MszRc MszT200Device::write_reg(const uint32_t reg_addr, const uint32_t reg_value) {

	MszRc									rc = MszRc::OK;
	uint8_t									data[2 + 6 + 4 + 4];
	uint32_t								data_idx, crc32;
	
//	ESP_LOGCONFIG(TAG, "setup ENTER write_reg reg_addr: %u reg_value: %u", reg_addr, reg_value);
	if (reg_addr < msz_t200_register_number) {
		data_idx = 0;
		this->enable();
		data_idx += this->send_header(data, reg_addr, true, 1);
		data_idx += this->send_register_value(data + data_idx, reg_value);
		crc32 = msz_t200_crc32_calc(data, data_idx);
		data_idx += this->send_register_value(data + data_idx, crc32);
		this->disable();
	} else {
		rc = MszRc::Inv_arg;
	}

	return rc;
}

MszRc MszT200Device::read_reg(const uint32_t reg_addr, uint32_t *reg_data) {

	MszRc									rc = MszRc::OK;
	uint8_t									data[2 + 6 + 4 + 4];
	uint32_t								data_idx, reg_val;
	uint32_t								crc32_recv, crc32_calc;
	
//	ESP_LOGCONFIG(TAG, "setup ENTER read_reg reg_addr: %u", reg_addr);
	if (reg_addr < msz_t200_register_number) {
		data_idx = 0;
		this->enable();
		data_idx += this->send_header(data + data_idx, reg_addr, false, 1);
		data_idx += this->recv_register_value(data + data_idx, reg_val);
		this->recv_register_value(data + data_idx, crc32_recv);
		this->disable();
		crc32_calc = msz_t200_crc32_calc(data, data_idx);
		if (crc32_recv == crc32_calc) {
			*reg_data = reg_val;
			this->stats.read_ok_ctr++;
		} else {
		//	ESP_LOGCONFIG(TAG, "Invalid CRC recv: 0x%08X expect: 0x%08X", crc32_recv, crc32_calc);
			rc = MszRc::Inv_crc;
			this->stats.read_err_ctr++;
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
