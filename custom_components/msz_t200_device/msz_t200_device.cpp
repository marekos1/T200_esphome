#include "esphome/core/log.h"
#include "msz_t200_device.h"


namespace esphome {
namespace msz_t200_device {
	

static const char *TAG = "msz_t200_device";

static const uint8_t msz_t200_header_first = 0x4D;
static const uint8_t msz_t200_header_second = 0xD3;
static const uint32_t msz_t200_register_number = 0x01FFFFFF;
static const uint32_t msz_t200_register_access_in_single_operation = 128;

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

void MszT200Device::test1(bool& read, const uint32_t reg_addr, uint32_t *reg_test_value, const uint32_t reg_test_count)  {
	
	static  timeval							last_operation, last_stats_show;					
	uint32_t								reg_read_value[msz_t200_register_access_in_single_operation], reg_no, reg_value;
	MszRc									rc;
	struct timeval 							current_tv;
	
    gettimeofday(&current_tv, NULL);
    if (timeval_compare(current_tv, this->startup_tv)) {
		
		if (timeval_compare(current_tv, last_operation)) {
			last_operation = current_tv;
			timeval_add_us(last_operation, 2000);
		}
		
		if (read) {
			rc = read_registers(reg_addr, reg_read_value, reg_test_count);
			if (rc == MszRc::OK) {
				for (reg_no = 0; reg_no < reg_test_count; reg_no++) {
					reg_value = *(reg_test_value + reg_no);
					if (reg_read_value[reg_no] != reg_value) {
						ESP_LOGCONFIG(TAG, "ERROR test1 addr: 0x%08X read value: 0x%08X should 0x%08X", reg_addr + reg_no, reg_read_value[reg_no], reg_value);
						rc = MszRc::Error;
					}
					reg_value++;
					*(reg_test_value + reg_no) = reg_value;
				}
			} else {
				ESP_LOGCONFIG(TAG, "Read error rc: %d", rc);
			}
			read = false;
		} else {
			rc = write_registers(reg_addr, reg_test_value, reg_test_count);
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
		ESP_LOGCONFIG(TAG, "Stats: Write ok: %8u err: %8u",  this->stats.write_ok_ctr, this->stats.write_err_ctr);
		ESP_LOGCONFIG(TAG, "Stats: Read  ok: %8u err: %8u",  this->stats.read_ok_ctr, this->stats.read_err_ctr);
	}
}

void MszT200Device::loop() {
	
	static uint32_t 						reg_test1_value[1] = {	0x33333333};
	static uint32_t 						reg_test2_value[128] = { 	0x11111111, 0x88888888, 0x55555555, 0xAAAAAAAA,
																	0x18181818, 0x5A5A5A5A, 0x78787878, 0x185A185A};
	static bool								reg_test1_read = false, reg_test2_read = false;
	static uint32_t							test_ctr = 0;
	
	if (test_ctr == 0) {
		test_ctr++;
		test1(reg_test1_read, 0x00456789, reg_test1_value, 1);
	} else if (test_ctr == 1) {
		test_ctr = 0;
		test1(reg_test2_read, 0x00888888, reg_test2_value, 128);
	} else {
		
	}
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
		delayMicroseconds(1000);
	}
	
	return header_idx;
}

uint32_t MszT200Device::send_register_value(uint8_t *data, const uint32_t reg_value) {
	
	uint32_t									data_idx = 0;
	
	data_idx += this->set_32bdata_value(data, reg_value);
	this->write_array(data, data_idx);
	
	return data_idx;
}

uint32_t MszT200Device::send_registers_value(uint8_t *data, const uint32_t *reg_data, const uint32_t regs_count) {
	
	uint32_t									data_idx = 0;
	uint32_t									reg_no, reg_value;
	
	for (reg_no = 0; reg_no < regs_count; reg_no++) {
		reg_value = *(reg_data + reg_no);
		data_idx += this->set_32bdata_value(data + (reg_no * 4), reg_value);
	}
	this->write_array(data, data_idx);
	
	return data_idx;
}

uint32_t MszT200Device::recv_register_value(uint8_t *data, uint32_t& reg_value) {
	
	uint32_t									data_idx = 0;
	
	this->read_array(data, 4);
	data_idx += get_32bdata_value(data, reg_value);
	
	return data_idx;
}

uint32_t MszT200Device::recv_registers_value(uint8_t *data, uint32_t *reg_data, const uint32_t regs_count) {
	
	uint32_t									data_idx = 0;
	uint32_t									reg_no, reg_value;
	
	this->read_array(data, regs_count * 4);
//	ESP_LOGCONFIG(TAG, "data 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", 
//								*(data + 0), *(data + 1), *(data + 2), *(data + 3), 
//								*(data + 4), *(data + 5), *(data + 6), *(data + 7));
	for (reg_no = 0; reg_no < regs_count; reg_no++) {
		data_idx += get_32bdata_value(data + (reg_no * 4), reg_value);
		*(reg_data + reg_no) = reg_value;
		if ((reg_no == 0) || (reg_no == 1)) {
//			ESP_LOGCONFIG(TAG, "reg_no: %u data 0x%02X 0x%02X 0x%02X 0x%02X reg_value: 0x%08X 0x%08X", reg_no, 
//								*(data + (reg_no * 4) + 0), *(data + (reg_no * 4) + 1), *(data + (reg_no * 4) + 2), *(data + (reg_no * 4) + 3),
//								reg_value, *(reg_data + reg_no));
		}
	}

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

MszRc MszT200Device::write_registers(const uint32_t reg_addr, const uint32_t *reg_data, const uint32_t regs_count) {

	MszRc									rc = MszRc::OK;
	uint8_t									data[2 + 6 + (4 * msz_t200_register_access_in_single_operation) + 4];
	uint32_t								data_idx, crc32;
	
//	ESP_LOGCONFIG(TAG, "ENTER write_reg reg_addr: %u reg_value: %u regs_count: %u", reg_addr, *reg_data, regs_count);
	if ((reg_addr < msz_t200_register_number) && (regs_count <= msz_t200_register_access_in_single_operation) && (reg_addr + regs_count) <= msz_t200_register_number) {
		data_idx = 0;
		this->enable();
		data_idx += this->send_header(data, reg_addr, true, regs_count);
		data_idx += this->send_registers_value(data + data_idx, reg_data, regs_count);
		crc32 = msz_t200_crc32_calc(data, data_idx);
		data_idx += this->send_register_value(data + data_idx, crc32);
		this->disable();
	} else {
		rc = MszRc::Inv_arg;
	}
//	ESP_LOGCONFIG(TAG, "EXIT write_reg rc: %u", rc);

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

MszRc MszT200Device::read_registers(const uint32_t reg_addr, uint32_t *reg_data, const uint32_t regs_count) {

	MszRc									rc = MszRc::OK;
	uint8_t									data[2 + 6 + (4 * msz_t200_register_access_in_single_operation) + 4];
	uint32_t								data_idx, crc32_recv, crc32_calc;
	
//	ESP_LOGCONFIG(TAG, "ENTER reg_addr: %u regs_count: %u", reg_addr, regs_count);
	if ((reg_addr < msz_t200_register_number) && (regs_count <= msz_t200_register_access_in_single_operation) && (reg_addr + regs_count) <= msz_t200_register_number) {
		data_idx = 0;
		this->enable();
		data_idx += this->send_header(data + data_idx, reg_addr, false, regs_count);
		data_idx += this->recv_registers_value(data + data_idx, reg_data, regs_count);
		this->recv_register_value(data + data_idx, crc32_recv);
		this->disable();
		crc32_calc = msz_t200_crc32_calc(data, data_idx);
		if (crc32_recv == crc32_calc) {
			this->stats.read_ok_ctr++;
		} else {
		//	ESP_LOGCONFIG(TAG, "Invalid CRC recv: 0x%08X expect: 0x%08X", crc32_recv, crc32_calc);
			rc = MszRc::Inv_crc;
			this->stats.read_err_ctr++;
		}
	} else {
		rc = MszRc::Inv_arg;
	}
//	ESP_LOGCONFIG(TAG, "EXIT reg_addr rc: %u data 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", rc, 
//													*(data + 8), *(data + 9), *(data + 10), *(data + 11), 
//													*(data + 12), *(data + 13), *(data + 14), *(data + 15));
//	ESP_LOGCONFIG(TAG, "EXIT reg_addr rc: %u reg 0x%08X 0x%08X 0x%08X 0x%08X", rc, *(reg_data + 0), *(reg_data + 1), *(reg_data + 2), *(reg_data + 3));
//	ESP_LOGCONFIG(TAG, "EXIT reg_addr rc: %u", rc);

	return rc;
}

void MszT200Device::update_reg(uint8_t pin, bool pin_value, uint8_t reg_addr) {

}

}  // namespace msz_t200_device
}  // namespace esphome
