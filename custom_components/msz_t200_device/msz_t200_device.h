#ifndef __MSZ_T200_COMPNENT__
#define __MSZ_T200_COMPNENT__

#include "esphome/core/component.h"
#include "esphome/components/spi/spi.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace msz_t200_device {
	
	
static const uint32_t gpio_total = 32;	
	
enum class MszRc {
	OK = 0,
	Error = -1,
	Inv_arg = -2,
	Inv_crc = -3,
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
	uint32_t channel;
};

class MszT200DeviceStats {
	
	public:
	uint32_t 						write_ok_ctr;
	uint32_t						write_err_ctr;
	uint32_t 						read_ok_ctr;
	uint32_t						read_err_ctr;
	
	MszT200DeviceStats() : write_ok_ctr{0}, write_err_ctr{0}, read_ok_ctr{0}, read_err_ctr{0} {
		
	}
};

class MszT200Base : public Component {
 public:
	void set_open_drain_ints(const bool value) { this->open_drain_ints_ = value; }
	float get_setup_priority() const override;
	MszRc gpio_read(const MszT200InstanceIdent& inst_ident, bool& gpio_state);
	
	uint32_t rd_ok_ctr = 0;
    uint32_t rd_err_ctr = 0;
	
	bool gpio_state_[gpio_total];
	struct timeval change_time_[gpio_total];
	
	uint32_t get_data(const MszT200InstanceIdent& ident) {
		uint32_t data = 0;
		
		if (ident.channel == 1) {
			data = rd_ok_ctr;
		} else if (ident.channel == 2) {
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
       
    MszRc write_reg(const uint32_t reg_addr, const uint32_t reg_value);
	MszRc write_registers(const uint32_t reg_addr, const uint32_t *reg_data, const uint32_t regs_count);
	
    MszRc read_reg(const uint32_t reg_addr, uint32_t *reg_data);
	MszRc read_registers(const uint32_t reg_addr, uint32_t *reg_data, const uint32_t regs_count);
    
    void update_reg(uint8_t pin, bool pin_value, uint8_t reg_a) override;
    void set_conf_mod1(uint8_t unit, uint8_t module, MszT200ModuleType mode_type);

    MszT200ModuleType unit1_module_type[4] = {MszT200ModuleType::NoneEmpty};
  
 private:
	MszT200DeviceStats	stats;
	struct timeval 							startup_tv;
 
	uint32_t set_32bdata_value(uint8_t *data, const uint32_t value);
	uint32_t get_32bdata_value(const uint8_t *data, uint32_t& value);
	
	uint32_t send_header(uint8_t *data, const uint32_t reg_addr, const bool write_operation, const uint32_t burst_length);
	uint32_t send_register_value(uint8_t *data, const uint32_t reg_value);
	uint32_t send_registers_value(uint8_t *data, const uint32_t *reg_data, const uint32_t regs_count);
	uint32_t recv_register_value(uint8_t *data, uint32_t& reg_value);
	uint32_t recv_registers_value(uint8_t *data, uint32_t *reg_data, const uint32_t regs_count);
	
	void test1(bool& read, const uint32_t reg_addr, uint32_t *reg_test_value, const uint32_t reg_test_count);
};

}  // namespace msz_t200_device
}  // namespace esphome

#endif /* __MSZ_T200_COMPNENT__ */
