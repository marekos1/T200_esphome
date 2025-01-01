#ifndef __MSZ_T200_COMPNENT__
#define __MSZ_T200_COMPNENT__


#include "esphome/components/spi/spi.h"
#include "esphome/core/component.h"
#include "esphome/components/text_sensor/text_sensor.h"


namespace esphome {
namespace msz_t200_device {
	
	
#define MSZ_T200_UNITS_PER_DEVICE 			1
#define MSZ_T200_MODULES_PER_UNIT 			4
#define MSZ_T200_CHANNELS_PER_MODULE		8

#define MSZ_T200_TOTAL_INST 				(MSZ_T200_UNITS_PER_DEVICE * MSZ_T200_MODULES_PER_UNIT * MSZ_T200_CHANNELS_PER_MODULE)

#define MSZ_T200_SW_OPTION_TEXT_SENSOR		1



typedef uint8_t msz_t200_unit_no_t;
#define MSZ_T200_USER_UNIT_TO_UNIT_NO(user_unit) (user_unit - 1);

typedef uint8_t msz_t200_module_no_t;
#define MSZ_T200_USER_MODULE_TO_MODULE_NO(user_module) (user_module - 1);

typedef uint8_t msz_t200_channel_no_t;
#define MSZ_T200_USER_CHANNEL_TO_CHANNEL_NO(user_module) (user_module - 1);

typedef uint8_t msz_t200_inst_no_t;


	
enum class MszRc {
	OK = 0,
	Error = -1,
	Inv_arg = -2,
	Inv_crc = -3,
	Inv_conf = -4,
	Inv_state = -5,
	
	Inv_unit_no = - 10,
	Inv_module_no = - 11,
	Inv_channel_no = - 12,
	
	Slave_not_yet_ready = -100,
	Slave_unknown_ID = -101,
	Unsupported_hw_rev = -102,
	Unsupported_fw_ver = -109,
};
	
enum MszT200ModuleType : uint8_t { 
	NoneEmpty = 0, 
	Input8,
	Output8,
	Dallas4
};

class MszT200InstanceIdent {
 public:
	msz_t200_unit_no_t 						unit_no;
	msz_t200_module_no_t 					module_no;
	msz_t200_channel_no_t 					channel_no;
};



/**********************************************************
 * MSZ T200 SPI stats
 **********************************************************/

class MszT200DeviceSpiStats {
	
 public:
	void step();
	void event(const bool wr_operation, const MszRc rc, const uint32_t data_length);
 	void publish();
 	void set_txt_sensor(text_sensor::TextSensor *txt_sensor);
	MszT200DeviceSpiStats();
	
 private:
 	text_sensor::TextSensor					*txts;
 	uint32_t								publish_ctr;
 	uint32_t 								write_ok_ctr;
	uint32_t								write_err_ctr;
	uint32_t 								write_data_ctr;
	uint32_t 								read_ok_ctr;
	uint32_t								read_err_ctr;
	uint32_t 								read_data_ctr;
 	
 	const char* print_stats(char *txt, uint32_t text_length);
};


/**********************************************************
 * MSZ T200 LOOP stats
 **********************************************************/

class MszT200DeviceLoopStats {
	
 public:
	void start();
	void end();
	void enable_stats(const bool enable);
	void publish();
	void set_txt_sensor(text_sensor::TextSensor *txt_sensor);
	MszT200DeviceLoopStats();
		
 private:
	bool									enable_;
	uint32_t								start_time, end_time;
	uint32_t 								loop_time, loop_time_min, loop_time_max, loop_time_cum, loop_time_total, loop_time_avg;
	uint32_t 								loop_rate, loop_rate_min, loop_rate_max, loop_rate_cum, loop_rate_avg;
	uint32_t								loop_rate_ctr, loop_rate_last_sec, loop_rate_total;
	uint32_t								publish_ctr, publish_next_ctr;
 	text_sensor::TextSensor					*txts;

	const char* print_stats(char *txt, uint32_t text_length);
};

class MszT200DeviceSlaveConf {
 public:
	MszT200ModuleType				unit_module_type[MSZ_T200_UNITS_PER_DEVICE][MSZ_T200_MODULES_PER_UNIT];
	
	void clear() {
		
		msz_t200_unit_no_t			unit_no;
		msz_t200_module_no_t		module_no;
		
		for (unit_no = 0; unit_no < MSZ_T200_UNITS_PER_DEVICE; unit_no++) {
			for (module_no = 0; module_no < MSZ_T200_MODULES_PER_UNIT; module_no++) {
				unit_module_type[unit_no][module_no] = MszT200ModuleType::NoneEmpty;
			}
		}
	}
	
	bool comapre(const MszT200DeviceSlaveConf& data) {
		
		bool								diff = false;
		msz_t200_unit_no_t					unit_no;
		msz_t200_module_no_t				module_no;
		
		for (unit_no = 0; unit_no < MSZ_T200_UNITS_PER_DEVICE; unit_no++) {
			for (module_no = 0; module_no < MSZ_T200_MODULES_PER_UNIT; module_no++) {
				if (data.unit_module_type[unit_no][module_no] != unit_module_type[unit_no][module_no]) {
					diff = true;
					break;
				}
			}
		}
				
		return diff;
	}
	
	MszT200DeviceSlaveConf() {
		
		clear();
	}
};

class MszT200DeviceSlaveStatusDetect {
 public:
 	bool							detected;
	uint32_t						hw_rev;
	uint32_t						sw_ver;

	MszT200DeviceSlaveStatusDetect() : detected{false}, hw_rev{0}, sw_ver{0} {
		
	}
};

class MszT200DeviceSlaveStatusConfigured {
 public:
	bool							configured;
	MszT200DeviceSlaveConf			conf_apply;
	
			
	MszT200DeviceSlaveStatusConfigured() : configured{false} {
		
		conf_apply.clear();
	}
};

class MszT200DeviceSlaveStatus {
	
 public:
	bool get_init_done();
	uint32_t get_firmware_version();	
	void set_txt_sensor(text_sensor::TextSensor *txt_sensor);
	
	uint32_t get_poll_ctr();
	void set_poll_ctr(const uint32_t new_value);
	bool get_next_poll();
	
	void access_regs_event(const MszRc rc);
	void detect_fail_event(const MszRc rc);
	void apply_conf_event(const MszRc rc);
	void detect_ok_event(const MszRc rc, const MszT200DeviceSlaveStatusDetect& detect);
	void conf_ok_event(const MszRc rc, const MszT200DeviceSlaveStatusConfigured& conf);
	
	MszT200DeviceSlaveStatus();
	
 private:
	const uint32_t							poll_ctr_1sec_factor = 124;
	uint32_t								poll_ctr;
	
	uint32_t								read_err_ctr;
	uint32_t								detect_err_ctr;
	uint32_t								apply_conf_err_ctr;
	uint32_t								apply_conf_ok_ctr;
	uint32_t								detect_ok_ctr;
	uint32_t								conf_ok_ctr;
	
	bool									init_done;
	MszT200DeviceSlaveStatusDetect			detect_;
	MszT200DeviceSlaveStatusConfigured		conf_;
	text_sensor::TextSensor					*txts;
	
	const char* print_status(char *txt, uint32_t text_length);
	void publish();
	void set_detected(const MszT200DeviceSlaveStatusDetect& detect);
	void set_configured(const MszT200DeviceSlaveStatusConfigured& conf);
};


class MszT200Base : public Component, public text_sensor::TextSensor {
 public:
 
	MszT200DeviceSlaveConf 					ha_user_config;
 
	float get_setup_priority() const override;
	
	msz_t200_inst_no_t get_inst_no(const msz_t200_unit_no_t unit_no, const msz_t200_module_no_t module_no, const msz_t200_channel_no_t channel_no);
	MszRc validate_inst_ident(const MszT200InstanceIdent& inst_ident);
	
	MszRc msz_t200_gpio_read(const MszT200InstanceIdent& inst_ident, bool& gpio_state);
	MszRc msz_t200_gpio_write(const MszT200InstanceIdent& inst_ident, const bool gpio_state);
	
	bool gpio_state_[MSZ_T200_TOTAL_INST];
	bool gpio_output_set_state[MSZ_T200_TOTAL_INST];
	bool gpio_output_change_state[MSZ_T200_UNITS_PER_DEVICE];

	MszRc msz_t200_dallas_get_data(const MszT200InstanceIdent& inst_ident, uint32_t& data) ; 	//to jest do dallas i wymaga zmiany nazwy
 private:
    msz_t200_inst_no_t get_inst_no(const MszT200InstanceIdent& inst_ident); 
};

class MszT200Device : public MszT200Base, 
					  public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST,spi::CLOCK_POLARITY_LOW, 
                                spi::CLOCK_PHASE_LEADING,spi::DATA_RATE_200KHZ> {
 public:
    void setup() override;
    void loop() override;
    void dump_config() override;
    
    void set_ha_user_conf_unit_module(const uint8_t u_unit, const uint8_t u_module, const MszT200ModuleType mode_type);
    void set_ha_user_conf_irq_pin(InternalGPIOPin *irq_pin);
    void set_ha_user_conf_test_pin(InternalGPIOPin *test_pin);
    text_sensor::TextSensor* get_new_text_sensor_by_id(const int i);

 private:
	MszT200DeviceSlaveStatus				slave_status;
	MszT200DeviceSpiStats					spi_stats;
	MszT200DeviceLoopStats					loop_stats;

	uint32_t set_32bdata_value(uint8_t *data, const uint32_t value);
	uint32_t get_32bdata_value(const uint8_t *data, uint32_t& value);
	uint32_t send_header(uint8_t *data, const uint32_t reg_addr, const bool write_operation, const uint32_t burst_length);
	uint32_t send_register_value(uint8_t *data, const uint32_t reg_value);
	uint32_t send_registers_value(uint8_t *data, const uint32_t *reg_data, const uint32_t regs_count);
	uint32_t recv_register_value(uint8_t *data, uint32_t& reg_value);
	uint32_t recv_registers_value(uint8_t *data, uint32_t *reg_data, const uint32_t regs_count);
	MszRc write_registers(const uint32_t reg_addr, const uint32_t *reg_data, const uint32_t regs_count);
	MszRc read_registers(const uint32_t reg_addr, uint32_t *reg_data, const uint32_t regs_count);
	
	MszRc poll();
	MszRc init_slave(uint32_t *reg_read_value);
	MszRc detect_slave_device(uint32_t *regs_value, MszT200DeviceSlaveStatusDetect& status);
	void parse_slave_conf(uint32_t *regs_value, MszT200DeviceSlaveConf& conf);
	MszRc apply_slave_conf(const MszT200DeviceSlaveConf& user_conf);
	MszRc slave_conf_poll(uint32_t *regs_value, const MszT200DeviceSlaveConf& user_conf);
	MszRc parse_module_status_by_type(const msz_t200_unit_no_t unit_no, const msz_t200_module_no_t module_no, const MszT200ModuleType module_type, 
												const uint8_t status_reg_value);
	MszRc slave_status_poll(const uint32_t *regs_value, const MszT200DeviceSlaveConf& conf);
	MszRc read_device_status(const MszT200DeviceSlaveConf& user_conf);
	
	MszRc write_unit_new_state(const msz_t200_unit_no_t	unit_no, const uint32_t new_state);
	MszRc write_new_state(uint32_t *new_state, const uint32_t new_state_length);
	MszRc write_state(const bool force, bool& was_write);
    
    text_sensor::TextSensor* text_sensors_ptr[8];
    text_sensor::TextSensor* get_text_sensor_by_name(const char *name);
    
 protected:
   InternalGPIOPin *irq_pin_{nullptr};
   InternalGPIOPin *test_pin_{nullptr};
};

}  // namespace msz_t200_device
}  // namespace esphome

#endif /* __MSZ_T200_COMPNENT__ */
