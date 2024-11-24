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

uint32_t MszT200Base::get_inst_idx(const uint32_t unit_no, const uint32_t module_no, const uint32_t inst_no) {
	
	uint32_t								inst_idx;
	
	inst_idx = unit_no * 4 + module_no * 8 + inst_no;
	
	return inst_idx;
}

MszRc MszT200Base::gpio_read(const MszT200InstanceIdent& inst_ident, bool& gpio_state) {
		
	MszRc									rc = MszRc::OK;
		
//	ESP_LOGCONFIG(TAG, "Enter unit_id: %u module_id: %u channel: %u", inst_ident.unit_id, inst_ident.module_id, inst_ident.channel);
	if ((inst_ident.unit_id == 0) && (inst_ident.module_id == 0) && (inst_ident.channel_id == 0)) {
		gpio_state = gpio_state_[0];
	} else if ((inst_ident.unit_id == 0) && (inst_ident.module_id == 0) && (inst_ident.channel_id == 1)) {
		gpio_state = gpio_state_[1];
	} else {
		rc = MszRc::Inv_arg;
	}
	
	return rc;
}

MszRc MszT200Base::gpio_write(const MszT200InstanceIdent& inst_ident, const bool gpio_state) {
		
	MszRc									rc = MszRc::OK;
	uint32_t								inst_idx;
	//[msz_t200_device:051]: Enter unit_id: 0 module_id: 2 channel_id: 0 gpio_state: 0
	ESP_LOGCONFIG(TAG, "Enter unit_id: %u module_id: %u channel_id: %u gpio_state: %d", inst_ident.unit_id, inst_ident.module_id, inst_ident.channel_id, gpio_state);
	inst_idx = get_inst_idx(inst_ident.unit_id, inst_ident.module_id, inst_ident.channel_id);
	if (gpio_state != gpio_output_set_state[inst_idx]) {
		gpio_output_set_state[inst_idx] = gpio_state;
		gpio_output_change_state[inst_ident.module_id] = true;	
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
			if (rc != MszRc::OK) {
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
	
#if MSZ_T200_SW_OPTION_TEXT_SENSOR   
	    uint32_t i;
	    static uint32_t sensor_ctr[8] = {466, 798, 133213, 8979, 32156, 48949, 78999, 1111};
	    char text[128];
	    
		for (i = 0; i < 2; i++) {
			if (this->text_sensors_ptr[i]) {
				snprintf(text, 128, " Sensor %u value %u name: %s", i + 1, sensor_ctr[i], this->text_sensors_ptr[i]->get_name().c_str());
				this->text_sensors_ptr[i]->publish_state(text);
			}
			sensor_ctr[i]++;
		}
		
		text_sensor::TextSensor* ts_spi_stat;
		
		ts_spi_stat = get_text_sensor_by_name("SpiStat");
		
		if (ts_spi_stat) {
			stats.print_stats(text, 128);
			ts_spi_stat->publish_state(text);
		}
		
#endif /* MSZ_T200_SW_OPTION_TEXT_SENSOR */		
	}
}

void MszT200Device::get_unit_conf(const uint32_t unit_conf_reg, MszT200DeviceSlaveModuleConf& conf) {
	
	conf.clear();
	conf.unit_module_type[0] = static_cast<MszT200ModuleType>((unit_conf_reg >> 0) & 0x000000FF);
	conf.unit_module_type[1] = static_cast<MszT200ModuleType>((unit_conf_reg >> 8) & 0x000000FF);
	conf.unit_module_type[2] = static_cast<MszT200ModuleType>((unit_conf_reg >> 16) & 0x000000FF);
	conf.unit_module_type[3] = static_cast<MszT200ModuleType>((unit_conf_reg >> 24) & 0x000000FF);	
}

bool MszT200Device::detect_slave(MszT200DeviceSlaveStatusData& status) {
	
	bool									detected = false;
	MszRc									rc;
	uint32_t								reg_read_value[14], reg_no;
	
	rc = read_registers(0x00000000, reg_read_value, 8);
	if (rc == MszRc::OK) {
		if (reg_read_value[0] & (1 << 0)) {
			if (reg_read_value[1] == 0x00000082) {
			 	
			 	detected = true;
			 	status.clear();
			 	status.hw_rev = reg_read_value[2];
			 	status.sw_ver = reg_read_value[3];
			 	ESP_LOGCONFIG(TAG, "Found slave HW rev: %u SW ver: %u", status.hw_rev, status.sw_ver);
			 	get_unit_conf(reg_read_value[6], status.module_conf);				
			 	ESP_LOGCONFIG(TAG, "Module 0 config: %u %u %u %u", status.module_conf.unit_module_type[0], status.module_conf.unit_module_type[1], status.module_conf.unit_module_type[2], status.module_conf.unit_module_type[3]);
			 	
			} else {
				ESP_LOGCONFIG(TAG, "Slave unknown ID: 0x%08X", reg_read_value[1]);
			}
		} else {
			ESP_LOGCONFIG(TAG, "Slave NOT read!");
		}
	} else {
		ESP_LOGCONFIG(TAG, "Read error rc: %d 0x%08X 0x%08X 0x%08X 0x%08X   0x%08X 0x%08X 0x%08X 0x%08X", rc,
											 reg_read_value[0], reg_read_value[1], reg_read_value[2], reg_read_value[3],
											 reg_read_value[4], reg_read_value[5], reg_read_value[6], reg_read_value[7]);
	}
			
	return detected;
}

MszRc MszT200Device::check_module_configuration(const MszT200DeviceSlaveModuleConf& read_module_conf) {
	
	MszRc									rc = MszRc::OK;
	uint32_t								module_no;
	
	module_no = 0;
	for (auto& type : read_module_conf.unit_module_type) {
		if (type == this->config.unit_module_type[module_no]) {
			module_no++;
		} else {
			rc = MszRc::Inv_conf;
			break;
		}
	}
	
	return rc;
}

MszRc MszT200Device::apply_module_configuration(const MszT200DeviceSlaveModuleConf& module_conf) {
	
	MszRc									rc = MszRc::OK;
	uint32_t								reg_value;
	
	reg_value = 0;
	reg_value |= static_cast<uint32_t>(module_conf.unit_module_type[0]) << 0;
	reg_value |= static_cast<uint32_t>(module_conf.unit_module_type[1]) << 8;
	reg_value |= static_cast<uint32_t>(module_conf.unit_module_type[2]) << 16;
	reg_value |= static_cast<uint32_t>(module_conf.unit_module_type[3]) << 24;
	rc = write_registers(6, &reg_value, 1);
	
	return rc;
}

MszRc MszT200Device::read_module_status_by_type(const uint32_t unit_no, const uint32_t module_no, const MszT200ModuleType module_type, 
												const uint8_t status_reg_value) {
	
	MszRc									rc = MszRc::OK;
	uint32_t								inst_no, inst_idx;
	bool									new_state;
	
	switch (module_type) {
	case MszT200ModuleType::Input8:
		for (inst_no = 0; inst_no < 8; inst_no++) {
			new_state = status_reg_value & (1 << inst_no);
			inst_idx = get_inst_idx(unit_no, module_no, inst_no);
			if (new_state != this->gpio_state_[inst_idx]) {
				ESP_LOGCONFIG(TAG, "Input8 unit_no: %u module_no: %u inst_no: %u inst_idx: %u State change: %u", 
													unit_no, module_no, inst_no, inst_idx, new_state);
				this->gpio_state_[inst_idx] = new_state;
			}
		}
		break;
	default:
		rc = MszRc::Inv_arg;
		break;
	}
		
	return rc;	
}

MszRc MszT200Device::read_module_status() {
	
	MszRc									rc = MszRc::OK;
	uint32_t								reg_read_value[8], unit_no, module_no;
	MszT200DeviceSlaveModuleConf			unit_module_conf;
	uint8_t									unit_module_status_reg_value;
	
	rc = read_registers(0x00000006, reg_read_value, 8);
	if (rc == MszRc::OK) {
		#if 0
		for (unit_no = 0; unit_no < 4; unit_no++) {
			if (reg_read_value[unit_no]) {
				get_unit_conf(reg_read_value[unit_no], unit_module_conf);	
				rc = check_module_configuration(unit_module_conf);
				if (rc == MszRc::OK) {
					for (module_no = 0; module_no < 4; module_no++) {
						unit_module_status_reg_value = (uint8_t)((reg_read_value[4] >> (8 * module_no)) & 0x000000FF);
						read_module_status_by_type(unit_no, module_no, unit_module_conf.unit_module_type[module_no], unit_module_status_reg_value);
					}
				} else {
					break;
				}
			}
		}
		#endif
	} else {
		ESP_LOGCONFIG(TAG, "Faield rc: %d - ", rc);
	}

	return rc;
}

MszRc MszT200Device::write_module_state() {
	
	MszRc									rc = MszRc::OK;
	uint32_t								unit_no, module_no, reg_value, inst_idx;
		
//	ESP_LOGCONFIG(TAG, "Enter");
	for (unit_no = 0; unit_no < 4; unit_no++) {
		for (module_no = 0; module_no < 4; module_no++) {
			if (this->config.unit_module_type[module_no] == MszT200ModuleType::Output8) {
//				ESP_LOGCONFIG(TAG, "Unit %u module %u is Output", unit_no, module_no);
				if (gpio_output_change_state[unit_no]) {
					ESP_LOGCONFIG(TAG, "Change state on unit_no: %u", unit_no);
					reg_value = 0;
					for (inst_idx = 0; inst_idx < 32; inst_idx++) {
						if (gpio_output_set_state[inst_idx]) {
							reg_value |= 1 << inst_idx;
						}
					} 
					ESP_LOGCONFIG(TAG, "Write register 14 to reg_value: %u", reg_value);
					rc = write_registers(14, &reg_value, 1);
					if (rc == MszRc::OK) {
						gpio_output_change_state[unit_no] = false;
					}
				}
				break;
			} 
		}
	}
	
	return rc;
}


void MszT200Device::loop() {

	struct timeval 							current_tv;
	static struct timeval					last_stats_show;
	MszT200DeviceSlaveStatusData			slave_status_data;
	MszRc									rc;
	char									text[128];
	text_sensor::TextSensor*				ts;
	bool									status_change;
	static uint32_t 						ctr = 0;
	
    gettimeofday(&current_tv, NULL);
    
    if (++ctr > 1) {
		ctr = 0;
		this->slave_status.get(slave_status_data);
		if ((slave_status_data.detected) && (slave_status_data.configured)) {
			rc = read_module_status();
			if (rc == MszRc::OK) {
				write_module_state();
			} else {
				ESP_LOGCONFIG(TAG, "read_module_status FAILED!!! detect again");
				slave_status_data.configured = false;
				slave_status_data.detected = false;
			}
		} else {
			if (timeval_compare(current_tv, this->startup_tv)) {
				
				slave_status_data.clear();
				slave_status_data.detected = detect_slave(slave_status_data);
				
				if (slave_status_data.detected) {
					rc = this->check_module_configuration(slave_status_data.module_conf);
					if (rc == MszRc::OK) {
						slave_status_data.configured = true;
					} else {
						ESP_LOGCONFIG(TAG, "Slave not configured properly");
						slave_status_data.configured = false;
						delayMicroseconds(10000);
						this->apply_module_configuration(this->config);
					}
				}
				
				status_change = this->slave_status.set(slave_status_data);
				if (status_change) {
					ts = get_text_sensor_by_name("Status");
					if (ts) {
						this->slave_status.print_status(text, 128);
						ts->publish_state(text);
					}
				}
			}
		}

		if (timeval_compare(current_tv, last_stats_show)) {
			last_stats_show = current_tv;
			last_stats_show.tv_sec += 3;
			ts = get_text_sensor_by_name("SpiStat");
			if (ts) {
				stats.print_stats(text, 128);
				ts->publish_state(text);
			}
		}
	}
}

void MszT200Device::setup() {
    
    ESP_LOGCONFIG(TAG, "setup ENTER msz_t200_device");
    this->spi_setup();
    
    struct timeval 							current_tv;
	this->slave_status.clear();
    gettimeofday(&this->startup_tv, NULL);
    this->startup_tv.tv_sec += 10;
}

void MszT200Device::dump_config() {
	
    ESP_LOGCONFIG(TAG, "msz t200 component get_component_source: %s", get_component_source());
 //   ESP_LOGCONFIG(TAG, "msz t200 component get_object_id: %s", this->get_object_id().c_str());
 //   ESP_LOGCONFIG(TAG, "msz t200 component get_name: %s", this->get_name().c_str());   
    LOG_PIN("  CS Pin:", this->cs_);
}

void MszT200Device::set_conf_mod1(uint8_t unit, uint8_t module, MszT200ModuleType mode_type) {

    ESP_LOGCONFIG(TAG, "Enter unit module: %u.%u mode_type: %u", unit, module, mode_type);
    if (unit == 1) {
		if (module && (module < 5)) {
			this->config.unit_module_type[module - 1] = mode_type;
		}
	}
}


#if MSZ_T200_SW_OPTION_TEXT_SENSOR   

text_sensor::TextSensor* MszT200Device::get_new_text_sensor(int i) { 
	
	ESP_LOGCONFIG(TAG, "Enter i: %d", i);
	this->text_sensors_ptr[i] = new text_sensor::TextSensor();
	
	return this->text_sensors_ptr[i];
}

text_sensor::TextSensor* MszT200Device::get_text_sensor_by_name(const char *name) { 
	
	text_sensor::TextSensor					*ts = NULL;
	uint32_t								i;
	    
	for (i = 0; i < 8; i++) {
		if (this->text_sensors_ptr[i]) {
			if (this->text_sensors_ptr[i]->get_name().str().compare(name) == 0) {
				ts = this->text_sensors_ptr[i];
				break;
			}
		}
	}
	
	return ts;
}

#endif /* MSZ_T200_SW_OPTION_TEXT_SENSOR */

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
	uint8_t										recv_header_data[2];


	*(data + header_idx) = msz_t200_header_first;
	recv_header_data[0] = this->transfer_byte(*(data + header_idx));
	header_idx++;
	delayMicroseconds(10);
	
	*(data + header_idx) = msz_t200_header_second;
	recv_header_data[1] = this->transfer_byte(*(data + header_idx));
	header_idx++;
	delayMicroseconds(10);
	
	if ((recv_header_data[0] == 0xA0) && (recv_header_data[1] == 0xA1)) {
		header_idx += this->set_32bdata_value((data + header_idx), (reg_addr & 0x00FFFFFF));
		*(data + header_idx++) = write_operation ? 0x80 : 0x00;
		*(data + header_idx++) = burst_length;
		this->write_array(data + 2, 6);
		if (write_operation) {
			delayMicroseconds(20);
		} else {
			delayMicroseconds(200);
		}
	} else {
		header_idx = 0;
		ESP_LOGCONFIG(TAG, "Recv header data invalid rx 0x%02X 0x%02X but should recv 0xA0 0xA1", recv_header_data[0], recv_header_data[1]);
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

MszRc MszT200Device::write_registers(const uint32_t reg_addr, const uint32_t *reg_data, const uint32_t regs_count) {

	MszRc									rc = MszRc::OK;
	uint8_t									data[2 + 6 + (4 * msz_t200_register_access_in_single_operation) + 4];
	uint32_t								data_idx, crc32, write_try_ctr;
	
	ESP_LOGCONFIG(TAG, "ENTER write_reg reg_addr: %u reg_value: %u regs_count: %u", reg_addr, *reg_data, regs_count);
	if ((reg_addr < msz_t200_register_number) && (regs_count <= msz_t200_register_access_in_single_operation) && (reg_addr + regs_count) <= msz_t200_register_number) {
		write_try_ctr = 0;
		do {
			if (rc != MszRc::OK) {
				delayMicroseconds(300);
			}
			data_idx = 0;
			this->enable();
			delayMicroseconds(100);
			data_idx += this->send_header(data, reg_addr, true, regs_count);
			if (data_idx) {
				data_idx += this->send_registers_value(data + data_idx, reg_data, regs_count);
				crc32 = msz_t200_crc32_calc(data, data_idx);
				data_idx += this->send_register_value(data + data_idx, crc32);
				this->stats.write_ok_ctr++;
			} else {	
				this->stats.write_err_ctr++;
				rc = MszRc::Error;
				if (++write_try_ctr >= 3) {
					this->disable();
					break;
				}
			}
			this->disable();
		} while (rc != MszRc::OK);
	} else {
		rc = MszRc::Inv_arg;
		this->stats.write_err_ctr++;
	}
	ESP_LOGCONFIG(TAG, "EXIT write_reg rc: %u", rc);

	return rc;
}

MszRc MszT200Device::read_registers(const uint32_t reg_addr, uint32_t *reg_data, const uint32_t regs_count) {

	MszRc									rc = MszRc::OK;
	uint8_t									data[2 + 6 + (4 * msz_t200_register_access_in_single_operation) + 4];
	uint32_t								data_idx, crc32_recv, crc32_calc, read_try_ctr;
	
//	ESP_LOGCONFIG(TAG, "ENTER reg_addr: %u regs_count: %u", reg_addr, regs_count);
	if ((reg_addr < msz_t200_register_number) && (regs_count <= msz_t200_register_access_in_single_operation) && (reg_addr + regs_count) <= msz_t200_register_number) {
		do {
			if (rc != MszRc::OK) {
				delayMicroseconds(300);
			}
			data_idx = 0;
			this->enable();
			delayMicroseconds(100);
			data_idx += this->send_header(data + data_idx, reg_addr, false, regs_count);
			if (data_idx) {
				data_idx += this->recv_registers_value(data + data_idx, reg_data, regs_count);
				this->recv_register_value(data + data_idx, crc32_recv);
				this->disable();
				crc32_calc = msz_t200_crc32_calc(data, data_idx);
				if (crc32_recv == crc32_calc) {
					this->stats.read_ok_ctr++;
				} else {
					ESP_LOGCONFIG(TAG, "Invalid CRC recv: 0x%08X expect: 0x%08X", crc32_recv, crc32_calc);
					rc = MszRc::Inv_crc;
					this->stats.read_err_ctr++;
					if (++read_try_ctr >= 3) {
						this->disable();
						break;
					}
				}
			} else {
				rc = MszRc::Error;
				this->stats.read_err_ctr++;
				if (++read_try_ctr >= 3) {
					this->disable();
					break;
				}
			}
			this->disable();
		} while (rc != MszRc::OK);
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
