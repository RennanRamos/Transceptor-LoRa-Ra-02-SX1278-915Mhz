#include "inc/Gpio.hpp"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Gpio::Gpio(){
	config_.edge = 0;
	config_.invert = 0;
	config_.ioc = 0;
	config_.pin = 0;
	config_.port = 0;
	config_.strength = 0;
	config_.pintype = 0;
}


bool Gpio::operator==(const Gpio& other) {
	return (&config_ == &other.config_);
}

GpioConfig Gpio::getGpioConfig(void) {
	return config_;
}

void Gpio::gpioUnlock(void) {
	GPIOUnlockPin(config_.port, config_.pin);
}

void Gpio::padConfig(void) {
	GPIOPadConfigSet(config_.port, config_.pin, config_.strength, config_.pintype);
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
