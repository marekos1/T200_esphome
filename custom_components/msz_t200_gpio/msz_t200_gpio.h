#ifndef __MSZ_T200_GPIO__
#define __MSZ_T200_GPIO__

#include "esphome/core/component.h"
#include "esphome/core/hal.h"

#include "esphome/components/msz_t200_device/msz_t200_device.h"

namespace esphome {
namespace msz_t200_gpio {

enum MszT200InterruptMode : uint8_t { MszT200_NO_INTERRUPT = 0, MszT200_CHANGE, MszT200_RISING, MszT200_FALLING };

class MszT200GPIOPin : public GPIOPin {
 public:
	void setup() override;
	void pin_mode(gpio::Flags flags) override;
	bool digital_read() override;
	void digital_write(bool value) override;

    
	std::string dump_summary() const override;

	void set_parent(msz_t200_device::MszT200Base *parent);
	
	void set_inst_ident(uint8_t unit_id, uint8_t module_id, uint8_t channel_id) { 
		this->instance_ident_.unit_id = unit_id - 1; 
		this->instance_ident_.module_id = module_id - 1; 
		this->instance_ident_.channel_id = channel_id - 1; 
	}
	
	
	
	void set_inverted(bool inverted) { inverted_ = inverted; }
	void set_flags(gpio::Flags flags) { flags_ = flags; }
	void set_interrupt_mode(MszT200InterruptMode interrupt_mode) { interrupt_mode_ = interrupt_mode; }

 protected:
	msz_t200_device::MszT200Base *parent_;
	msz_t200_device::MszT200InstanceIdent instance_ident_;

	bool inverted_;
	gpio::Flags flags_;
	MszT200InterruptMode interrupt_mode_;
  
	bool digital_read_prv(uint8_t unit_id, uint8_t module_id, uint8_t pin);
	void digital_write_prv(uint8_t pin, bool value);
  
	void pin_mode_prv(uint8_t pin, gpio::Flags flags);
	void pin_interrupt_mode_prv(uint8_t pin, MszT200InterruptMode interrupt_mode);
};

}  // namespace msz_t200_gpio
}  // namespace esphome

#endif /* __MSZ_T200_GPIO__ */
