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


	
class MszT200Base : public Component {
 public:
	void set_open_drain_ints(const bool value) { this->open_drain_ints_ = value; }
	float get_setup_priority() const override;
	MszRc gpio_read(const MszT200InstanceIdent& inst_ident, bool& gpio_state);
	
	// read a given register
	virtual bool read_reg(uint8_t reg, uint8_t *value);
	
	bool gpio_state_[gpio_total];
	struct timeval change_time_[gpio_total];
	
 protected:
	// update registers with given pin value.
	virtual void update_reg(uint8_t pin, bool pin_value, uint8_t reg_a);

	bool open_drain_ints_;
};

class MszT200Device : public MszT200Base, 
					  public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST,spi::CLOCK_POLARITY_LOW, 
                                spi::CLOCK_PHASE_LEADING,spi::DATA_RATE_1MHZ> {
 public:
    void setup() override;
    void loop() override;
    void dump_config() override;
    
    MszRc write_reg(uint32_t reg_addr, uint32_t value);
    	
    	
    
    bool read_reg(uint8_t reg, uint8_t *value) override;

    void update_reg(uint8_t pin, bool pin_value, uint8_t reg_a) override;
    
    void set_conf_mod1(uint8_t unit, uint8_t module, MszT200ModuleType mode_type);

    
    int test_val;
    MszT200ModuleType unit1_module_type[4] = {MszT200ModuleType::NoneEmpty};
};

}  // namespace msz_t200_device
}  // namespace esphome

#endif /* __MSZ_T200_COMPNENT__ */
