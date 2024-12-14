#ifndef __MSZ_T200_COMPNENT__
#define __MSZ_T200_COMPNENT__


#include "esphome/components/spi/spi.h"
#include "esphome/core/component.h"
#include "esphome/components/text_sensor/text_sensor.h"


namespace esphome {
namespace msz_t200_device {
	
#define MSZ_T200_SW_OPTION_TEXT_SENSOR	1
	
	
static const uint32_t gpio_total = 128;	
	
enum class MszRc {
	OK = 0,
	Error = -1,
	Inv_arg = -2,
	Inv_crc = -3,
	Inv_conf = -4,
};
	
enum MszT200ModuleType : uint8_t { 
	NoneEmpty = 0, 
	Input8,
	Output8,
	Dallas4
};

class MszT200InstanceIdent {
 public:
	uint8_t unit_id;
	uint8_t module_id;
	uint32_t channel_id;
};

class MszT200DeviceSpiStats {
	
 public:
	uint32_t 						write_ok_ctr;
	uint32_t						write_err_ctr;
	uint32_t 						read_ok_ctr;
	uint32_t						read_err_ctr;
	
	MszT200DeviceSpiStats() : write_ok_ctr{0}, write_err_ctr{0}, read_ok_ctr{0}, read_err_ctr{0}, publish_ctr{0} {
#if MSZ_T200_SW_OPTION_TEXT_SENSOR 		
		txts = NULL;
#endif /* MSZ_T200_SW_OPTION_TEXT_SENSOR */
	}
	
	const char* print_stats(char *txt, uint32_t text_length) {
		
		snprintf(txt, text_length, "Write ok: %u err: %u Read ok: %u err: %u", write_ok_ctr, write_err_ctr, read_ok_ctr, read_err_ctr);
		
		return txt;
	}
	
	void step(const struct timeval& current_tv);
	
#if MSZ_T200_SW_OPTION_TEXT_SENSOR   
	void set_txt_sensor(text_sensor::TextSensor *txt_sensor) {
		
		txts = txt_sensor;
	} 
#endif /* MSZ_T200_SW_OPTION_TEXT_SENSOR */	
	
 private:
#if MSZ_T200_SW_OPTION_TEXT_SENSOR 		
 	text_sensor::TextSensor					*txts;
#endif /* MSZ_T200_SW_OPTION_TEXT_SENSOR */
 	uint32_t								publish_ctr;
};

class MszT200DeviceLoopStats {
	
 public:
 
	MszT200DeviceLoopStats() : publish_ctr{0}, loop_tv_min{.tv_sec = 9999, .tv_usec = 999999} {
#if MSZ_T200_SW_OPTION_TEXT_SENSOR 		
		txts = NULL;
#endif /* MSZ_T200_SW_OPTION_TEXT_SENSOR */
	}

	const char* print_stats(char *txt, uint32_t text_length) {
		
		snprintf(txt, text_length, "Loop rate: %u curr: %u.%06u min: %u.%06u max: %u.%06u avg: %u.%06u", 
													loop_rate, loop_tv.tv_sec, loop_tv.tv_usec, 
													loop_tv_min.tv_sec, loop_tv_min.tv_usec, 
													loop_tv_max.tv_sec, loop_tv_max.tv_usec,
													loop_tv_avg.tv_sec, loop_tv_avg.tv_usec);

		return txt;
	}
	
	void clear() {
		publish_ctr = 0;
		loop_tv_min = {.tv_sec = 9999, .tv_usec = 999999};
	}
	
	void step(const struct timeval& current_tv);
	
	
#if MSZ_T200_SW_OPTION_TEXT_SENSOR   
	void set_txt_sensor(text_sensor::TextSensor *txt_sensor) {
		
		txts = txt_sensor;
	} 
#endif /* MSZ_T200_SW_OPTION_TEXT_SENSOR */	
		
 private:
	uint32_t								publish_ctr;
	struct timeval 							loop_tv, loop_tv_min, loop_tv_max, loop_tv_cum, loop_tv_avg;
	uint32_t 								loop_cum_ctr, loop_rate_last_sec, loop_rate_ctr, loop_rate;
#if MSZ_T200_SW_OPTION_TEXT_SENSOR 		
 	text_sensor::TextSensor					*txts;
#endif /* MSZ_T200_SW_OPTION_TEXT_SENSOR */
};


class MszT200DeviceSlaveModuleConf {
 public:
	MszT200ModuleType				unit_module_type[4];
	
	void clear() {
		for (auto& type : unit_module_type) {
			type = MszT200ModuleType::NoneEmpty;
		}
	}
	
	bool comapre(const MszT200DeviceSlaveModuleConf& data) {
		
		bool								diff = false;
		uint32_t							module_no;
		
		for (module_no = 0; module_no < 4; module_no++) {
			if (data.unit_module_type != unit_module_type) {
				diff = true;
				break;
			}
		}
		
		return diff;
	}
};

class MszT200DeviceSlaveStatusData {
 public:
 	bool							detected;
	bool							configured;
	bool							init_done;
	uint32_t						hw_rev;
	uint32_t						sw_ver;
		
	void set_detected(const bool detect_state) {
		
		if (detect_state != this->detected) {
			this->detected = detect_state;
			init_done = (this->detected && this->configured);
		}
	}
	
	void set_configured(const bool configure_state) {
		
		if (configure_state != this->configured) {
			this->configured = configure_state;
			init_done = (this->detected && this->configured);
		}
	}
	
	MszT200DeviceSlaveModuleConf	module_conf;
	
	void clear() {
		detected = false;
		configured = false;
		hw_rev = 0;
		sw_ver = 0;
		init_done = false;
		module_conf.clear();
	}
	
	bool comapre(const MszT200DeviceSlaveStatusData& data) {
		
		bool								diff = false;
		
		if ((data.detected != detected) || (data.configured != configured) || 
			(data.hw_rev != hw_rev) || (data.sw_ver != sw_ver)) {
			diff = true;
		}
		
		return diff;
	}
};

class MszT200DeviceSlaveStatus {
	
 public:
 
#if MSZ_T200_SW_OPTION_TEXT_SENSOR  
	text_sensor::TextSensor					*txts;
#endif /* MSZ_T200_SW_OPTION_TEXT_SENSOR */

	MszT200DeviceSlaveStatus() {
#if MSZ_T200_SW_OPTION_TEXT_SENSOR 		
		txts = NULL;
#endif /* MSZ_T200_SW_OPTION_TEXT_SENSOR */
	}
 
	void clear() {
		poll_ctrl = 0;
		data_m.clear();
	}
	
	void get(MszT200DeviceSlaveStatusData& data) {
		
		data = data_m;
	}
	
	bool set(const MszT200DeviceSlaveStatusData& data) {
		
		bool								diff = false;
		
		diff = data_m.comapre(data);
		if (diff) {
			data_m = data;
		}
		
		return diff;
	}
	
	bool get_poll_timeout() {
		
		bool 								timeout = false;
		
		if (++poll_ctrl > 300) {
			poll_ctrl = 0;
			timeout = true;
		}
		
		return timeout;
	}
	
	bool get_init_done() {
		
		return data_m.init_done;
	}
	
	void clear_init_done() {
		
		data_m.clear();
	}
		
	const char* print_status(char *txt, uint32_t text_length) {
		
		int 								ct;
		
		ct = 0;
		ct += snprintf(txt + ct, text_length - ct, "Detect: %u Configured: %u", data_m.detected, data_m.configured);
		if (data_m.detected) {
			ct += snprintf(txt + ct, text_length - ct, " HW rev: %u Sw Ver: %u", data_m.hw_rev, data_m.sw_ver);
		}
		
		return txt;
	}
	
#if MSZ_T200_SW_OPTION_TEXT_SENSOR   
	void set_txt_sensor(text_sensor::TextSensor *txt_sensor) {
		
		txts = txt_sensor;
	} 
#endif /* MSZ_T200_SW_OPTION_TEXT_SENSOR */	
	
 private:
	MszT200DeviceSlaveStatusData	data_m;
	uint32_t						poll_ctrl;
};


class MszT200Base : public Component, public text_sensor::TextSensor {
 public:
	void set_open_drain_ints(const bool value) { this->open_drain_ints_ = value; }
	float get_setup_priority() const override;
	
	uint32_t get_inst_idx(const uint32_t unit_no, const uint32_t module_no, const uint32_t inst_no);
	
	MszRc gpio_read(const MszT200InstanceIdent& inst_ident, bool& gpio_state);
	MszRc gpio_write(const MszT200InstanceIdent& inst_ident, const bool gpio_state);
	
	uint32_t rd_ok_ctr = 0;
    uint32_t rd_err_ctr = 0;
	
	bool gpio_state_[gpio_total];
	bool gpio_output_set_state[gpio_total];
	bool gpio_output_change_state[4];
	
	struct timeval change_time_[gpio_total];
	
	
	
	uint32_t get_data(const MszT200InstanceIdent& ident) {
		uint32_t data = 0;
		
		if (ident.channel_id == 1) {
			data = rd_ok_ctr;
		} else if (ident.channel_id == 2) {
			data = rd_err_ctr;
		}
		return data;
	}
	
 protected:
	// update registers with given pin value.
	virtual void update_reg(uint8_t pin, bool pin_value, uint8_t reg_a);

	bool open_drain_ints_;
};

class MszT200Device : public MszT200Base, 
					  public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST,spi::CLOCK_POLARITY_LOW, 
                                spi::CLOCK_PHASE_LEADING,spi::DATA_RATE_200KHZ> {
 public:
    void setup() override;
    void loop() override;
    void dump_config() override;
       
	MszRc write_registers(const uint32_t reg_addr, const uint32_t *reg_data, const uint32_t regs_count);
	MszRc read_registers(const uint32_t reg_addr, uint32_t *reg_data, const uint32_t regs_count);
    
    void update_reg(uint8_t pin, bool pin_value, uint8_t reg_a) override;
    void set_conf_mod1(uint8_t unit, uint8_t module, MszT200ModuleType mode_type);
    void set_irq_pin(InternalGPIOPin *irq_pin) { irq_pin_ = irq_pin; }

	MszT200DeviceSlaveModuleConf config[4] = {MszT200ModuleType::NoneEmpty, MszT200ModuleType::NoneEmpty, MszT200ModuleType::NoneEmpty, MszT200ModuleType::NoneEmpty};
    text_sensor::TextSensor* get_new_text_sensor_by_id(const int i);    

 private:
	MszT200DeviceSpiStats					spi_stats;
	MszT200DeviceLoopStats					loop_stats;
	MszT200DeviceSlaveStatus				slave_status;
	
	struct timeval 							startup_tv;

 
	uint32_t set_32bdata_value(uint8_t *data, const uint32_t value);
	uint32_t get_32bdata_value(const uint8_t *data, uint32_t& value);
	
	uint32_t send_header(uint8_t *data, const uint32_t reg_addr, const bool write_operation, const uint32_t burst_length);
	uint32_t send_register_value(uint8_t *data, const uint32_t reg_value);
	uint32_t send_registers_value(uint8_t *data, const uint32_t *reg_data, const uint32_t regs_count);
	uint32_t recv_register_value(uint8_t *data, uint32_t& reg_value);
	uint32_t recv_registers_value(uint8_t *data, uint32_t *reg_data, const uint32_t regs_count);
	
	MszRc init();
	void get_unit_conf(const uint32_t unit_conf_reg, MszT200DeviceSlaveModuleConf& conf);
	bool detect_slave(MszT200DeviceSlaveStatusData& status);
	MszRc check_module_configuration(const uint32_t unit_no, const MszT200DeviceSlaveModuleConf& read_module_conf);
	MszRc apply_module_configuration(const uint32_t unit_no, const MszT200DeviceSlaveModuleConf& module_conf);
	MszRc read_module_status_by_type(const uint32_t unit_no, const uint32_t module_no, const MszT200ModuleType module_type, const uint8_t status_reg_value);
	MszRc read_module_status();
	MszRc write_module_state();
    
#if MSZ_T200_SW_OPTION_TEXT_SENSOR   
    text_sensor::TextSensor* text_sensors_ptr[8];
    
    text_sensor::TextSensor* get_text_sensor_by_name(const char *name);
#else /* MSZ_T200_SW_OPTION_TEXT_SENSOR */
	text_sensor::TextSensor* text_sensors_x;
#endif /* MSZ_T200_SW_OPTION_TEXT_SENSOR */	
	 
 protected:
   InternalGPIOPin *irq_pin_{nullptr};
};

}  // namespace msz_t200_device
}  // namespace esphome

#endif /* __MSZ_T200_COMPNENT__ */
