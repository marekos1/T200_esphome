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
 	void set_ha_user_conf_inst_ident(const uint8_t unit_id, const uint8_t module_id, const uint8_t channel_id);
	void set_ha_user_conf_parent(msz_t200_device::MszT200Base *parent);
	void set_ha_user_conf_inverted(const bool inverted);
	void set_ha_user_conf_flags(gpio::Flags flags);
	void set_ha_user_conf_interrupt_mode(MszT200InterruptMode interrupt_mode);
	
	void setup() override;
	void pin_mode(gpio::Flags flags) override;
	bool digital_read() override;
	void digital_write(bool value) override;
	std::string dump_summary() const override;
	
	MszT200GPIOPin();

 protected:
 	msz_t200_device::MszT200InstanceIdent 	inst_ident_;
	msz_t200_device::MszT200Base 			*parent_;
	bool 									inverted_;
	gpio::Flags								flags_;
	MszT200InterruptMode					interrupt_mode_;
};

}  // namespace msz_t200_gpio
}  // namespace esphome

#endif /* __MSZ_T200_GPIO__ */
