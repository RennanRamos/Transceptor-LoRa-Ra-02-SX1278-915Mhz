#include "inc/Gpio.hpp"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

GpioIn::GpioIn(){
	callback_ = nullptr;
}

bool GpioIn::read(void) {
  uint32_t state;
  
  /* Read the pin value */
  state = GPIOPinRead(config_.port, config_.pin);
  
  return (bool)state;
}

void GpioIn::enable(GpioConfig& config) {
	config_ = config;
	GPIOPinTypeGPIOInput(config_.port, config_.pin);

	
	GPIOIntTypeSet(config_.port, config_.pin, config_.edge);


}

void GpioIn::setCallback(Callback* callback) {
  
  callback_ = callback;

  
  InterruptHandler& interruptHandler = InterruptHandler::getInstance();

  
  interruptHandler.setInterruptHandler(*this);
}

void GpioIn::clearCallback(void) {
  
  callback_ = nullptr;
}

void GpioIn::enableInterrupts(void) {

	GPIOIntClear(config_.port, config_.pin);


	GPIOIntEnable(config_.port, config_.pin);
}

void GpioIn::disableInterrupts(void) {

	GPIOIntDisable(config_.port, config_.pin);
}


void GpioIn::interruptHandler(void){

  if (callback_ != nullptr) {
      callback_->execute();
  }
}


/*=============================== protected =================================*/

/*================================ private ==================================*/
