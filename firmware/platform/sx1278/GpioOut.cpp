/*================================ include ==================================*/

#include "inc/Gpio.hpp"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

GpioOut::GpioOut(){}


void GpioOut::on(void) {
  if (config_.invert) {
    HWREG(config_.port + (config_.pin << 2)) =  ~(config_.pin);

  }
  else {
    HWREG(config_.port + (config_.pin << 2)) =  config_.pin;

  }
}

void GpioOut::off(void) {
  if (config_.invert) {

    HWREG(config_.port + (config_.pin << 2)) =  config_.pin;
  }
  else {

    HWREG(config_.port + (config_.pin << 2))  =  ~(config_.pin);
  }
}

void GpioOut::toggle(void) {
//	status() ? high() : low();
  uint32_t status = HWREG(config_.port + (config_.pin << 2));
	HWREG(config_.port + (config_.pin << 2)) = status^config_.pin;

	uint32_t state = status();

	state = (~state) & config_.pin;

	GPIOPinWrite(config_.port, config_.pin, state);

}

void GpioOut::high(void){

  HWREG(config_.port + (config_.pin << 2)) =  config_.pin;
}

void GpioOut::low(void) {

  HWREG(config_.port + (config_.pin << 2)) =  ~(config_.pin);

}

uint32_t GpioOut::status(void) {

  uint32_t status = GPIOPinRead(config_.port, config_.pin);


  return (status & config_.pin);
}

void GpioOut::enable(GpioConfig& config)  {
	config_ = config;
	off();
  HWREG(config_.port + GPIO_O_AMSEL)  &= ~(config_.pin); //desabilita função analógica
  HWREG(config_.port + GPIO_O_DIR) |= config_.pin; // seta direção do pino
  HWREG(config_.port + GPIO_O_AFSEL) &= ~(config_.pin); //desabilita função alternativa
  HWREG(config_.port + GPIO_O_DEN) |= config_.pin; //habilita e/s digital
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
