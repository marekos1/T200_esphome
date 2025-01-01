#include "esphome/core/log.h"
#include "msz_t200_device.h"
#include "driver/gpio.h"

namespace esphome {
namespace msz_t200_device {
	

static const char *TAG = "msz_t200_device";

static const uint8_t msz_t200_header_first = 0x4D;
static const uint8_t msz_t200_header_second = 0xD3;
static const uint32_t msz_t200_register_number = 0x01FFFFFF;
static const uint32_t msz_t200_register_access_in_single_operation = 128;




bool timeval_compare(const struct timeval& lhs, const struct timeval& rhs) {
	
	bool ret_val = false;
	
    if (lhs.tv_sec == rhs.tv_sec) {
        ret_val = (lhs.tv_usec > rhs.tv_usec);
    } else {
        ret_val = (lhs.tv_sec > rhs.tv_sec);
	}
      
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

void timeval_add(struct timeval& tv, const struct timeval& tv_to_add) {
	
	timeval_add_us(tv, tv_to_add.tv_usec);
	tv.tv_sec += tv_to_add.tv_sec;
}

void timeval_get_avg(struct timeval& avg_tv, const struct timeval& tv_cum, const uint32_t tv_cum_ctr) {
	
	uint64_t 								usec;
	
	usec = tv_cum.tv_sec * 1000000;
	usec += tv_cum.tv_usec;
	usec = usec / tv_cum_ctr;
	avg_tv.tv_sec = usec / 1000000;
	avg_tv.tv_usec = usec % 1000000;
}

int timeval_subtract(struct timeval& result, const struct timeval& x, const struct timeval& y) {
	
	struct timeval							tv;
	
	tv = y;
	/* Perform the carry for the later subtraction by updating y. */
	if (x.tv_usec < tv.tv_usec) {
		int nsec = (tv.tv_usec - x.tv_usec) / 1000000 + 1;
		tv.tv_usec -= 1000000 * nsec;
		tv.tv_sec += nsec;
	}
	if (x.tv_usec - tv.tv_usec > 1000000) {
		int nsec = (x.tv_usec - tv.tv_usec) / 1000000;
		tv.tv_usec += 1000000 * nsec;
		tv.tv_sec -= nsec;
	}

	/* Compute the time remaining to wait. tv_usec is certainly positive. */
	result.tv_sec = x.tv_sec - tv.tv_sec;
	result.tv_usec = x.tv_usec - tv.tv_usec;

	/* Return 1 if result is negative. */
	return x.tv_sec < tv.tv_sec;
}


/**********************************************************
 * MSZ T200 SPI stats
 **********************************************************/

void MszT200DeviceSpiStats::step() {

	struct timeval							current_tv;
	
	if (txts) {
		 gettimeofday(&current_tv, NULL);
		if (current_tv.tv_sec >= publish_ctr) {
			publish_ctr = current_tv.tv_sec + 3;
			publish();

		}
	}
}

void MszT200DeviceSpiStats::event(const bool wr_operation, const MszRc rc, const uint32_t data_length) {
	
	if (wr_operation) {
		if (rc == MszRc::OK) {
			write_ok_ctr++;
			write_data_ctr+= data_length;
		} else {
			write_err_ctr++;
		}
	} else {
		if (rc == MszRc::OK) {
			read_ok_ctr++;
			read_data_ctr+= data_length;
		} else {
			read_err_ctr++;
		}
	}
}

const char* MszT200DeviceSpiStats::print_stats(char *txt, uint32_t text_length) {
		
	snprintf(txt, text_length, "Write ok: %u err: %u Read ok: %u err: %u", write_ok_ctr, write_err_ctr, read_ok_ctr, read_err_ctr);
		
	return txt;
}

void MszT200DeviceSpiStats::publish() {
	
	char								text[128];

	if (txts) {
		print_stats(text, 128);
		txts->publish_state(text);
	}
}

void MszT200DeviceSpiStats::set_txt_sensor(text_sensor::TextSensor *txt_sensor) {

	txts = txt_sensor;
}

MszT200DeviceSpiStats::MszT200DeviceSpiStats() : write_ok_ctr{0}, write_err_ctr{0}, 
												 read_ok_ctr{0}, read_err_ctr{0}, 
												 publish_ctr{0} {
			
	txts = NULL;
}

/**********************************************************
 * MSZ T200 LOOP stats
 **********************************************************/
 
void MszT200DeviceLoopStats::start() {
	
	if (enable_) {
		start_time = millis();
	} else {
		if (millis() > 10000) {
			enable_stats(true);
		}
	}
}

void MszT200DeviceLoopStats::end() {
	
	if (enable_) {
		end_time = millis();
		loop_time = end_time - start_time;
		if (loop_time < loop_time_min) {
			loop_time_min = loop_time;
		}
		if (loop_time > loop_time_max) {
			loop_time_max = loop_time;
		}
		loop_time_cum += loop_time;
		loop_time_total++;
		
		loop_rate_ctr++;
		if (end_time >= loop_rate_last_sec) {
			loop_rate_last_sec = end_time + 1000;
			loop_rate = loop_rate_ctr;
			loop_rate_ctr = 0;
			
			if (loop_rate < loop_rate_min) {
				loop_rate_min = loop_rate;
			}
			if (loop_rate > loop_rate_max) {
				loop_rate_max = loop_rate;
			}
			loop_rate_cum += loop_rate;
			loop_rate_total++;
			loop_rate_avg = loop_rate_cum / loop_rate_total;
		}
		
		if(end_time >= publish_next_ctr) {
			publish_next_ctr = end_time + 3000;
			loop_time_avg = loop_time_cum / loop_time_total;
			
			if (txts) {
				publish();	
			}	
		}
	}	
}

void MszT200DeviceLoopStats::enable_stats(const bool enable) {
	
	enable_ = enable;
}

const char* MszT200DeviceLoopStats::print_stats(char *txt, uint32_t text_length) {

	snprintf(txt, text_length, "Loop rate current %u min: %u max: %u avg: %u Loop Time current %u min: %u max: %u avg: %u", 
												    loop_rate, loop_rate_min, loop_rate_max, loop_rate_avg,
													loop_time, loop_time_min, loop_time_max, loop_time_avg);

	return txt;
}

void MszT200DeviceLoopStats::publish() {
	
	char								text[128];
		
	if (txts) {
		print_stats(text, 128);
		txts->publish_state(text);
	}
}

void MszT200DeviceLoopStats::set_txt_sensor(text_sensor::TextSensor *txt_sensor) {
		
	txts = txt_sensor;
} 

MszT200DeviceLoopStats::MszT200DeviceLoopStats() :  enable_{false}, start_time{0}, end_time{0},
													loop_time{0}, loop_time_min{0xFFFFFFFF}, loop_time_max{0}, loop_time_cum{0}, loop_time_total{0}, loop_time_avg{0},
													loop_rate{0}, loop_rate_min{0xFFFFFFFF}, loop_rate_max{0}, loop_rate_cum{0}, loop_rate_avg{0},
													loop_rate_ctr{0}, loop_rate_last_sec{0}, loop_rate_total{0}, publish_ctr{0}, publish_next_ctr{0} {
	
	txts = NULL;
}



/**********************************************************
 * MSZ T200 Base 
 **********************************************************/

float MszT200Base::get_setup_priority() const { 
	
	return setup_priority::IO;
}

msz_t200_inst_no_t MszT200Base::get_inst_no(const msz_t200_unit_no_t unit_no, const msz_t200_module_no_t module_no, const msz_t200_channel_no_t channel_no) {
	
	msz_t200_inst_no_t						inst_no = 0;
	
	inst_no += unit_no * MSZ_T200_MODULES_PER_UNIT * MSZ_T200_CHANNELS_PER_MODULE;
	inst_no += module_no * MSZ_T200_CHANNELS_PER_MODULE;
	inst_no += channel_no;

	return inst_no;
}

msz_t200_inst_no_t MszT200Base::get_inst_no(const MszT200InstanceIdent& inst_ident) {
	
	msz_t200_inst_no_t						inst_no;
	
	inst_no = get_inst_no(inst_ident.unit_no, inst_ident.module_no, inst_ident.channel_no);

	return inst_no;
}

MszRc MszT200Base::validate_inst_ident(const MszT200InstanceIdent& inst_ident) {
	
	MszRc									rc = MszRc::OK;
	
	if (inst_ident.unit_no >= MSZ_T200_UNITS_PER_DEVICE) {
		rc = MszRc::Inv_unit_no;
	} else if (inst_ident.module_no >= MSZ_T200_MODULES_PER_UNIT) {
		rc = MszRc::Inv_module_no;
	} else if (inst_ident.channel_no >= MSZ_T200_CHANNELS_PER_MODULE) {	
		rc = MszRc::Inv_channel_no;
	}
		
	return rc;
}

MszRc MszT200Base::msz_t200_gpio_read(const MszT200InstanceIdent& inst_ident, bool& gpio_state) {
		
	MszRc									rc = MszRc::OK;
	msz_t200_inst_no_t						inst_no;
	
	rc = validate_inst_ident(inst_ident);
	if (rc == MszRc::OK) {
		if ((ha_user_config.unit_module_type[inst_ident.unit_no][inst_ident.module_no] == MszT200ModuleType::Input8) ||
			(ha_user_config.unit_module_type[inst_ident.unit_no][inst_ident.module_no] == MszT200ModuleType::Output8)) {
			inst_no = get_inst_no(inst_ident);
			if (inst_no < MSZ_T200_TOTAL_INST) {
				gpio_state = gpio_state_[inst_no];
			} else {
				ESP_LOGW(TAG, "Something goes wrong inst_no: %u is higher then expected value %u unit_no: %u module_no: %u channel_no: %u ", 
													inst_no, MSZ_T200_TOTAL_INST, inst_ident.unit_no, inst_ident.module_no, inst_ident.channel_no);
				rc = MszRc::Inv_state;
			}
		} else {
			rc = MszRc::Inv_conf;
		}
	}
	
	return rc;
}

MszRc MszT200Base::msz_t200_gpio_write(const MszT200InstanceIdent& inst_ident, const bool gpio_state) {
		
	MszRc									rc = MszRc::OK;
	msz_t200_inst_no_t						inst_no;
	
	rc = validate_inst_ident(inst_ident);
	if (rc == MszRc::OK) {
		if ((ha_user_config.unit_module_type[inst_ident.unit_no][inst_ident.module_no] == MszT200ModuleType::Input8) ||
			(ha_user_config.unit_module_type[inst_ident.unit_no][inst_ident.module_no] == MszT200ModuleType::Output8)) {
			inst_no = get_inst_no(inst_ident);
			if (inst_no < MSZ_T200_TOTAL_INST) {
				ESP_LOGD(TAG, "gpio write unit_no: %u inst_no: %u gpio_state: %u current state: %u", inst_ident.unit_no, inst_no, gpio_state, gpio_output_set_state[inst_no]); 
				if (gpio_state != gpio_output_set_state[inst_no]) {
					gpio_output_set_state[inst_no] = gpio_state;
					gpio_output_change_state[inst_ident.unit_no] = true;	
				}
			} else {
				ESP_LOGW(TAG, "Something goes wrong inst_no: %u is higher then expected value %u unit_no: %u module_no: %u channel_no: %u ", 
													inst_no, MSZ_T200_TOTAL_INST, inst_ident.unit_no, inst_ident.module_no, inst_ident.channel_no);
				rc = MszRc::Inv_state;													
			}
		} else {
			rc = MszRc::Inv_conf;
		}
	}
	
	return rc;
}

MszRc MszT200Base::msz_t200_dallas_get_data(const MszT200InstanceIdent& inst_ident, uint32_t& data) {	//to jest do dallas i wymaga zmiany nazwy
		
	MszRc									rc = MszRc::OK;
	
	rc = validate_inst_ident(inst_ident);
	if (rc == MszRc::OK) {
		if (ha_user_config.unit_module_type[inst_ident.unit_no][inst_ident.module_no] == MszT200ModuleType::Dallas4) {
			data = 1111;
		} else {
			rc = MszRc::Inv_conf;
		}
	}
		
	return rc;
}

/**********************************************************
 * MSZ T200 Slave Status 
 **********************************************************/

bool MszT200DeviceSlaveStatus::get_init_done() {
		
	return init_done_;
}
	
uint32_t MszT200DeviceSlaveStatus::get_firmware_version() {
		
	return detect_.sw_ver;
}

void MszT200DeviceSlaveStatus::set_txt_sensor(text_sensor::TextSensor *txt_sensor) {
		
	txts = txt_sensor;
}

uint32_t MszT200DeviceSlaveStatus::get_poll_ctr() {
		
	return poll_ctr;
}
	
void MszT200DeviceSlaveStatus::set_poll_ctr(const uint32_t new_value) {
		
	poll_ctr = new_value;
}
		
bool MszT200DeviceSlaveStatus::get_next_poll() {
		
	bool									next_poll = false;
		
	if (poll_ctr) {
		if (--poll_ctr == 0) {
			next_poll = true;
			if (init_done_) {
				poll_ctr = poll_ctr_1sec_factor;		// ~1 second
			} else {
				poll_ctr = poll_ctr_1sec_factor / 10;
			}
		}
	} else {
		poll_ctr = poll_ctr_1sec_factor;		// ~1 second
	}
		
	return next_poll;
}

void MszT200DeviceSlaveStatus::access_regs_event(const MszRc rc) {
	
	MszT200DeviceSlaveStatusDetect 		detect;
	MszT200DeviceSlaveStatusConfigured	conf;
	
	if (rc == MszRc::OK) {
		read_err_ctr = 0;
		set_poll_ctr(poll_ctr_1sec_factor);
	} else {
		read_err_ctr++;
		if (read_err_ctr > 5) {
			read_err_ctr = 0;
			set_configured(conf);
			set_detected(detect);
		}
		set_poll_ctr(poll_ctr_1sec_factor / 5);
	}
}

void MszT200DeviceSlaveStatus::detect_fail_event(const MszRc rc) {
	
	MszT200DeviceSlaveStatusDetect 		detect;
	
	if (rc != MszRc::OK) {
		detect_err_ctr++;
		if (detect_err_ctr > 3) {
			detect_err_ctr = 0;
			set_detected(detect);
		}
	}
	set_poll_ctr(poll_ctr_1sec_factor / 5);
}

void MszT200DeviceSlaveStatus::apply_conf_event(const MszRc rc) {
	
	MszT200DeviceSlaveStatusDetect 		detect;
	MszT200DeviceSlaveStatusConfigured	conf;
	
	if (rc == MszRc::OK) {
		apply_conf_ok_ctr++;
		if (apply_conf_err_ctr > 3) {
			set_configured(conf);
			set_detected(detect);
		}
	} else {
		apply_conf_err_ctr++;
		if (apply_conf_err_ctr > 3) {
			apply_conf_err_ctr = 0;
			set_configured(conf);
		}
	}
	set_poll_ctr(poll_ctr_1sec_factor / 5);
}

void MszT200DeviceSlaveStatus::detect_ok_event(const MszRc rc, const MszT200DeviceSlaveStatusDetect& detect) {
	
	if (rc == MszRc::OK) {
		set_detected(detect);
		detect_ok_ctr++;
		detect_err_ctr = 0;
	}
	set_poll_ctr(poll_ctr_1sec_factor);
}

void MszT200DeviceSlaveStatus::conf_ok_event(const MszRc rc, const MszT200DeviceSlaveStatusConfigured& conf) {
	
	if (rc == MszRc::OK) {
		set_configured(conf);
		conf_ok_ctr++;
		apply_conf_err_ctr = 0;
	}
	set_poll_ctr(poll_ctr_1sec_factor);
}

MszT200DeviceSlaveStatus::MszT200DeviceSlaveStatus() : poll_ctr{2 * poll_ctr_1sec_factor}, detect_err_ctr{detect_err_ctr},
													   read_err_ctr{0}, apply_conf_err_ctr{0}, apply_conf_ok_ctr{0}, detect_ok_ctr{0}, 
													   conf_ok_ctr{0}, init_done_{false} {
	
	txts = NULL;
}

const char* MszT200DeviceSlaveStatus::print_status(char *txt, uint32_t text_length) {
	
	int 								ct;
	
	ct = 0;
	ct += snprintf(txt + ct, text_length - ct, "Detect: %u Configured: %u", detect_.detected, conf_.configured);
	if (detect_.detected) {
		ct += snprintf(txt + ct, text_length - ct, " HW rev: %u Sw Ver: %u", detect_.hw_rev, detect_.sw_ver);
	}
	
	return txt;
}

void MszT200DeviceSlaveStatus::publish() {

	char								text[128];
		
	if (txts) {
		print_status(text, 128);
		txts->publish_state(text);
	}
}

void MszT200DeviceSlaveStatus::set_detected(const MszT200DeviceSlaveStatusDetect& detect) {
	
	if (detect.detected != detect_.detected) {
		detect_ = detect;
		set_init_done();
		publish();
	}
}

void MszT200DeviceSlaveStatus::set_configured(const MszT200DeviceSlaveStatusConfigured& conf) {
	
	if (conf.configured != conf_.configured) {
		conf_ = conf;
		set_init_done();
		publish();
	}
}

void MszT200DeviceSlaveStatus::set_init_done() {
	
	bool									init_done;
	
	init_done = (detect_.detected && conf_.configured);
	if (init_done != init_done_) {
		init_done_ = init_done;
		if (init_done_) {
			ESP_LOGI(TAG, "Connect to slave HW rev: %u FW ver: %u", detect_.hw_rev, detect_.sw_ver);
		} else {
			ESP_LOGW(TAG, "Lost Connection to slave");
		}
	}
}


/**********************************************************
 * MSZ T200 Device 
 **********************************************************/

MszRc MszT200Device::detect_slave_device(uint32_t *regs_value, MszT200DeviceSlaveStatusDetect& status) {
	
	MszRc									rc = MszRc::OK;
	uint32_t								reg_no;
	msz_t200_unit_no_t 						unit_no;
	
	ESP_LOGD(TAG, "Enter 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X", regs_value[0], regs_value[1], regs_value[2], regs_value[3], regs_value[4], regs_value[5]);
	if (regs_value[0] & (1 << 0)) {
		if (regs_value[1] == 0x00000082) {
			status.hw_rev = regs_value[2];
			status.sw_ver = regs_value[3];
			if (status.hw_rev == 1) {
				if ((status.sw_ver == 1) || (status.sw_ver == 2)) {
					status.detected = true;
				} else {
					ESP_LOGI(TAG, "Slave unsupported firmware version: %u expected: %u or %u", status.sw_ver, 1, 2);
					rc = MszRc::Unsupported_fw_ver;
				}
			} else {
				ESP_LOGI(TAG, "Slave unsupported hardware revision: %u expected: %u", status.hw_rev, 1);
				rc = MszRc::Unsupported_hw_rev;
			}
					
		} else {
			ESP_LOGI(TAG, "Slave unknown ID: 0x%08X", regs_value[1]);
			rc = MszRc::Slave_unknown_ID;
		}
	} else {
		ESP_LOGI(TAG, "Slave NOT read!");
		rc = MszRc::Slave_not_yet_ready;
	}
	
	return rc;
}

void MszT200Device::parse_slave_conf(uint32_t *regs_value, MszT200DeviceSlaveConf& conf) {
	
	msz_t200_unit_no_t						unit_no;
	msz_t200_module_no_t					module_no;

	for (unit_no = 0; unit_no < MSZ_T200_UNITS_PER_DEVICE; unit_no++) {
		for (module_no = 0; module_no < MSZ_T200_MODULES_PER_UNIT; module_no++) {
			conf.unit_module_type[unit_no][module_no] = static_cast<MszT200ModuleType>((regs_value[unit_no] >> (module_no * 8)) & 0x000000FF);
		}
	}
}

MszRc MszT200Device::apply_slave_conf(const MszT200DeviceSlaveConf& user_conf) {
	
	MszRc									rc = MszRc::OK;
	msz_t200_unit_no_t						unit_no;
	msz_t200_module_no_t					module_no;
	uint32_t								reg_value[MSZ_T200_UNITS_PER_DEVICE];
	
	for (unit_no = 0; unit_no < MSZ_T200_UNITS_PER_DEVICE; unit_no++) {
		reg_value[unit_no] = 0;
		for (module_no = 0; module_no < MSZ_T200_MODULES_PER_UNIT; module_no++) {
			reg_value[unit_no] |= static_cast<uint32_t>(user_conf.unit_module_type[unit_no][module_no]) << (module_no * 8);			
		}
	}
	ESP_LOGD(TAG, "Write 0x%X", reg_value[0]);
	rc = write_registers(6, reg_value, MSZ_T200_UNITS_PER_DEVICE);
	
	return rc;
}

MszRc MszT200Device::slave_conf_poll(uint32_t *regs_value, const MszT200DeviceSlaveConf& user_conf) {
	
	MszRc									rc = MszRc::OK;
	MszT200DeviceSlaveConf					read_conf;
	
	ESP_LOGD(TAG, "Enter 0x%X 0x%X 0x%X 0x%X", regs_value[0], regs_value[1], regs_value[2], regs_value[3]);
	parse_slave_conf(regs_value, read_conf);
	if (read_conf.comapre(user_conf)) {
		rc = MszRc::Inv_conf;
	}
	
	return rc;
}

MszRc MszT200Device::parse_module_status_by_type(const msz_t200_unit_no_t unit_no, const msz_t200_module_no_t module_no, const MszT200ModuleType module_type, 
												const uint8_t status_reg_value) {
	
	MszRc									rc = MszRc::OK;
	msz_t200_channel_no_t					channel_no;
	msz_t200_inst_no_t						inst_no;
	bool									new_state;
	
	switch (module_type) {
	case MszT200ModuleType::Input8:
		for (channel_no = 0; channel_no < MSZ_T200_CHANNELS_PER_MODULE; channel_no++) {
			inst_no = get_inst_no(unit_no, module_no, channel_no);
			if (inst_no < MSZ_T200_TOTAL_INST) {
				new_state = status_reg_value & (1 << ((module_no * 8) + channel_no));
				if (new_state != this->gpio_state_[inst_no]) {
					ESP_LOGD(TAG, "Input8 unit_no: %u module_no: %u channel_no: %u inst_no: %u State change: %u", 
													unit_no, module_no, channel_no, inst_no, new_state);
					this->gpio_state_[inst_no] = new_state;
				}
			} else {
				ESP_LOGW(TAG, "Something goes wrong inst_no: %u is higher then expected value %u unit_no: %u module_no: %u channel_no: %u ", 
													inst_no, MSZ_T200_TOTAL_INST, unit_no, module_no, channel_no);
				rc = MszRc::Inv_state;
			}
		}
		break;
	default:
		rc = MszRc::Inv_arg;
		break;
	}
		
	return rc;	
}

MszRc MszT200Device::slave_status_poll(const uint32_t *regs_value, const MszT200DeviceSlaveConf& conf) {
	
	MszRc									rc = MszRc::OK;
	msz_t200_unit_no_t						unit_no;
	msz_t200_module_no_t					module_no;
	
	ESP_LOGD(TAG, "Enter 0x%X 0x%X 0x%X 0x%X", regs_value[0], regs_value[1], regs_value[2], regs_value[3]);
	for (unit_no = 0; unit_no < MSZ_T200_UNITS_PER_DEVICE; unit_no++) {
		for (module_no = 0; module_no < MSZ_T200_MODULES_PER_UNIT; module_no++) {
			parse_module_status_by_type(unit_no, module_no, conf.unit_module_type[unit_no][module_no], regs_value[unit_no]);
		}
	}
	
	return rc;
}

MszRc MszT200Device::read_device_status(const MszT200DeviceSlaveConf& user_conf) {
	
	MszRc									rc = MszRc::OK;
	uint32_t								reg_read_value[8];
	
	rc = read_registers(0x00000006, reg_read_value, 8);
	if (rc == MszRc::OK) {
		rc = slave_conf_poll(&reg_read_value[0], user_conf);
		if (rc == MszRc::OK) {
			slave_status_poll(&reg_read_value[4], ha_user_config);
		}
	}

	return rc;
}

MszRc MszT200Device::write_unit_new_state(const msz_t200_unit_no_t	unit_no, const uint32_t new_state) {
	
	MszRc									rc = MszRc::OK;
	uint32_t								slave_cpu_firmware_version = slave_status.get_firmware_version();
	uint32_t								reg_value;
		
	if (slave_cpu_firmware_version == 1) {
		reg_value = new_state;
		rc = write_registers(14 + unit_no, &reg_value, 1);
	} else if (slave_cpu_firmware_version == 2) {
		reg_value = new_state;
		rc = write_registers(2000 + unit_no, &reg_value, 1);
	} else {
		rc = MszRc::Unsupported_fw_ver;
	}
		
	return rc;
}

MszRc MszT200Device::write_new_state(uint32_t *new_state, const uint32_t new_state_length) {
	
	MszRc									rc = MszRc::OK;
	uint32_t								slave_cpu_firmware_version = slave_status.get_firmware_version();
		
	if (slave_cpu_firmware_version == 1) {
		rc = write_registers(14, new_state, new_state_length);
	} else if (slave_cpu_firmware_version == 2) {
		rc = write_registers(2000, new_state, new_state_length);
	} else {
		rc = MszRc::Unsupported_fw_ver;
	}
		
	return rc;
}

MszRc MszT200Device::write_state(const bool force, bool& was_write) {
	
	MszRc									rc = MszRc::OK;
	msz_t200_unit_no_t						unit_no;
	msz_t200_module_no_t					module_no;
	msz_t200_channel_no_t					channel_no;
	msz_t200_inst_no_t						inst_no;
	uint32_t								reg_value[MSZ_T200_UNITS_PER_DEVICE];
	
	was_write = false;
	for (unit_no = 0; unit_no < MSZ_T200_UNITS_PER_DEVICE; unit_no++) {
		reg_value[unit_no] = 0;
		if (gpio_output_change_state[unit_no] || force) {
			for (module_no = 0; module_no < MSZ_T200_MODULES_PER_UNIT; module_no++) {
				if (ha_user_config.unit_module_type[unit_no][module_no] == MszT200ModuleType::Output8) {
					for (channel_no = 0; channel_no < MSZ_T200_CHANNELS_PER_MODULE; channel_no++) {
						inst_no = module_no * MSZ_T200_CHANNELS_PER_MODULE + channel_no;
						if (gpio_output_set_state[inst_no]) {
							reg_value[unit_no] |= 1 << inst_no;
						}
					}
				}				
			}
			if (force == false) {
				rc = write_unit_new_state(unit_no, reg_value[unit_no]);
				if (rc == MszRc::OK) {
					gpio_output_change_state[unit_no] = false;
					was_write = true;
				}
				break;
			}
		}
	}
	if (force) {
		rc = write_new_state(reg_value, MSZ_T200_UNITS_PER_DEVICE);
		was_write = true;
	}
	
	return rc;
}

MszRc MszT200Device::init_slave(uint32_t *reg_read_value) {
	
	MszRc									rc = MszRc::OK;
	MszT200DeviceSlaveStatusDetect			detect_stat;
	MszT200DeviceSlaveStatusConfigured		configured_stat;
	
	rc = detect_slave_device(reg_read_value, detect_stat);
	if (rc == MszRc::OK) {
		slave_status.detect_ok_event(rc, detect_stat);
		rc = slave_conf_poll(&reg_read_value[6], ha_user_config);
		if (rc == MszRc::OK) {
			configured_stat.conf_apply = ha_user_config;
			configured_stat.configured = true;					
			slave_status.conf_ok_event(rc, configured_stat);
		} else {
			ESP_LOGD(TAG, "Slave configuration invalid! Apply conf");
			delayMicroseconds(1000);
			rc = apply_slave_conf(ha_user_config);
			slave_status.apply_conf_event(rc);
			if (rc != MszRc::OK) {
				ESP_LOGW(TAG, "Apply configuration to slave fail!");
			}
		}
	} else {
		slave_status.detect_fail_event(rc);
	}
	
	return rc;
}

MszRc MszT200Device::poll() {
	
	MszRc									rc = MszRc::OK;	
	bool									init_done, was_write;
	uint32_t								reg_read_value[14];

	if (slave_status.get_init_done()) {
		if ((this->irq_pin_->digital_read() == false)) {
			ESP_LOGD(TAG, "Irq");
			rc = read_device_status(ha_user_config);
			slave_status.access_regs_event(rc);
		}
		if (rc == MszRc::OK) {
			rc = write_state(false, was_write);
			if (was_write) {
				slave_status.access_regs_event(rc);
			}
		}
	}
	if (slave_status.get_next_poll()) {
		ESP_LOGD(TAG, "Poll");	
		rc = read_registers(0x00000000, reg_read_value, 14);
		slave_status.access_regs_event(rc);
		if (rc == MszRc::OK) {
			init_done = slave_status.get_init_done();
			rc = init_slave(reg_read_value);
			if (rc == MszRc::OK) {
				if (init_done) {
					rc = slave_status_poll(&reg_read_value[10], ha_user_config);
				} else {
					delayMicroseconds(1000);
					rc = write_state(true, was_write);
					slave_status.access_regs_event(rc);
				}
			}
		}
	}
	
	return rc;
}

void MszT200Device::loop() {

    loop_stats.start();
	poll();
	loop_stats.end();
	spi_stats.step();
}

void MszT200Device::setup() {
	
	text_sensor::TextSensor					*txt_sensor;
    
    ESP_LOGI(TAG, "setup ENTER msz_t200_device");
    this->spi_setup();
	ESP_LOGD(TAG, "Config unit_no 0: %u %u %u %u", this->ha_user_config.unit_module_type[0][0], 
												   this->ha_user_config.unit_module_type[0][1], 
												   this->ha_user_config.unit_module_type[0][2], 
												   this->ha_user_config.unit_module_type[0][3]);

	if (this->irq_pin_ != nullptr) {
		this->irq_pin_->setup();
	}
	
	if (this->test_pin_ != nullptr) {
		this->test_pin_->digital_write(false);
		this->test_pin_->pin_mode(gpio::Flags::FLAG_OUTPUT);
	}
 
	txt_sensor = get_text_sensor_by_name("Statistics");
	this->loop_stats.set_txt_sensor(txt_sensor);
	if (txt_sensor == NULL) {
		ESP_LOGW(TAG, "Not found txt sensor: Statistics");
	}
	
	txt_sensor = get_text_sensor_by_name("SpiStat");
	this->spi_stats.set_txt_sensor(txt_sensor);
	if (txt_sensor == NULL) {
		ESP_LOGW(TAG, "Not found txt sensor: SpiStat");
	}
	
	txt_sensor = get_text_sensor_by_name("Status");
	this->slave_status.set_txt_sensor(txt_sensor);
	if (txt_sensor == NULL) {
		ESP_LOGW(TAG, "Not found txt sensor: Status");
	}
}

void MszT200Device::dump_config() {
	
    ESP_LOGCONFIG(TAG, "msz t200 component get_component_source: %s", get_component_source()); 
    LOG_PIN("  CS Pin:", this->cs_);
    LOG_PIN("  IRQ Pin: ", irq_pin_);
}

/**********************************************************
 * MSZ T200 Device Home Assitant set configuration
 **********************************************************/

void MszT200Device::set_ha_user_conf_unit_module(const uint8_t u_unit, const uint8_t u_module, const MszT200ModuleType mode_type) {
	
	msz_t200_unit_no_t						unit_no = MSZ_T200_USER_UNIT_TO_UNIT_NO(u_unit);
	msz_t200_module_no_t					module_no = MSZ_T200_USER_MODULE_TO_MODULE_NO(u_module);
	
    ESP_LOGD(TAG, "Enter unit_no module_no: %u.%u mode_type: %u", unit_no, module_no, mode_type);
    if ((unit_no < MSZ_T200_UNITS_PER_DEVICE) && (module_no < MSZ_T200_MODULES_PER_UNIT)) {
		ha_user_config.unit_module_type[unit_no][module_no] = mode_type;
	} else {
		ESP_LOGW(TAG, "Invalid HA USER CONFIGURATION !!!");
	}
}

void MszT200Device::set_ha_user_conf_irq_pin(InternalGPIOPin *irq_pin) { 
	
	irq_pin_ = irq_pin; 
}

void MszT200Device::set_ha_user_conf_test_pin(InternalGPIOPin *test_pin) { 
	
	test_pin_ = test_pin; 
}


/**********************************************************
 * MSZ T200 Device Status and statistics Text Sensors
 **********************************************************/

text_sensor::TextSensor* MszT200Device::get_new_text_sensor_by_id(const int i) { 
	
	ESP_LOGD(TAG, "Enter i: %d", i);
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


/**********************************************************
 * MSZ T200 SPI maintenece and protocol
 **********************************************************/

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
		ESP_LOGD(TAG, "Recv header data invalid rx 0x%02X 0x%02X but should recv 0xA0 0xA1 write_operation: %u", recv_header_data[0], recv_header_data[1], write_operation);
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
	uint32_t								data_idx, crc32;
	
	this->test_pin_->digital_write(false);
	ESP_LOGD(TAG, "ENTER write_reg reg_addr: %u reg_value: %u regs_count: %u", reg_addr, *reg_data, regs_count);
	if ((reg_addr < msz_t200_register_number) && (regs_count <= msz_t200_register_access_in_single_operation) && (reg_addr + regs_count) <= msz_t200_register_number) {
		data_idx = 0;
		this->enable();
		delayMicroseconds(1000);
		data_idx += this->send_header(data, reg_addr, true, regs_count);
		if (data_idx) {
			data_idx += this->send_registers_value(data + data_idx, reg_data, regs_count);
			crc32 = msz_t200_crc32_calc(data, data_idx);
			data_idx += this->send_register_value(data + data_idx, crc32);
		} else {	
			ESP_LOGI(TAG, "Inv header");
			rc = MszRc::Error;
		}
		this->disable();
	} else {
		rc = MszRc::Inv_arg;
	}
	this->spi_stats.event(true, rc, regs_count);
	if (rc != MszRc::OK) {
		this->test_pin_->digital_write(true);
	}
//	ESP_LOGD(TAG, "EXIT write_reg rc: %u", rc);

	return rc;
}

MszRc MszT200Device::read_registers(const uint32_t reg_addr, uint32_t *reg_data, const uint32_t regs_count) {

	MszRc									rc = MszRc::OK;
	uint8_t									data[2 + 6 + (4 * msz_t200_register_access_in_single_operation) + 4];
	uint32_t								data_idx, crc32_recv, crc32_calc;
	
	this->test_pin_->digital_write(false);
//	ESP_LOGCONFIG(TAG, "ENTER reg_addr: %u regs_count: %u", reg_addr, regs_count);
	if ((reg_addr < msz_t200_register_number) && (regs_count <= msz_t200_register_access_in_single_operation) && 
		(reg_addr + regs_count) <= msz_t200_register_number) {
		data_idx = 0;
		this->enable();
		delayMicroseconds(1000);
		data_idx += this->send_header(data + data_idx, reg_addr, false, regs_count);
		if (data_idx) {
			data_idx += this->recv_registers_value(data + data_idx, reg_data, regs_count);
			this->recv_register_value(data + data_idx, crc32_recv);
			this->disable();			
			crc32_calc = msz_t200_crc32_calc(data, data_idx);
			if (crc32_recv != crc32_calc) {
				ESP_LOGI(TAG, "Invalid CRC recv: 0x%08X expect: 0x%08X", crc32_recv, crc32_calc);
				rc = MszRc::Inv_crc;
			}
		} else {
			this->disable();
			ESP_LOGD(TAG, "Inv header 1");
			rc = MszRc::Error;
		}
	} else {
		rc = MszRc::Inv_arg;
	}
	this->spi_stats.event(false, rc, regs_count);
	if (rc != MszRc::OK) {
		this->test_pin_->digital_write(true);
	}
//	ESP_LOGCONFIG(TAG, "EXIT reg_addr rc: %u data 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", rc, 
//													*(data + 8), *(data + 9), *(data + 10), *(data + 11), 
//													*(data + 12), *(data + 13), *(data + 14), *(data + 15));
//	ESP_LOGCONFIG(TAG, "EXIT reg_addr rc: %u reg 0x%08X 0x%08X 0x%08X 0x%08X", rc, *(reg_data + 0), *(reg_data + 1), *(reg_data + 2), *(reg_data + 3));
//	ESP_LOGCONFIG(TAG, "EXIT reg_addr rc: %u", rc);

	return rc;
}

}  // namespace msz_t200_device
}  // namespace esphome
