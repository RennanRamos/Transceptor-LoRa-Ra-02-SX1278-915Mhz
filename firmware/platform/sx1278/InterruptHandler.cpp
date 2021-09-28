/*================================ include ==================================*/

#include <InterruptHandler.hpp>

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/* Single class instance */
InterruptHandler InterruptHandler::instance_;

/* Vector containing the 8 objects refs to pins that causes interruptions in any GPIO port */
GpioIn* InterruptHandler::GPIOA_interruptVector_[8];
GpioIn* InterruptHandler::GPIOB_interruptVector_[8];
GpioIn* InterruptHandler::GPIOC_interruptVector_[8];
GpioIn* InterruptHandler::GPIOD_interruptVector_[8];
GpioIn* InterruptHandler::GPIOE_interruptVector_[8];
GpioIn* InterruptHandler::GPIOF_interruptVector_[8];

/* Vector containing the 2 objects refs to timers that causes interruptions */
Timer* InterruptHandler::TIMER0_interruptVector_[2];
Timer* InterruptHandler::TIMER1_interruptVector_[2];
Timer* InterruptHandler::TIMER2_interruptVector_[2];
Timer* InterruptHandler::TIMER3_interruptVector_[2];
Timer* InterruptHandler::TIMER4_interruptVector_[2];
Timer* InterruptHandler::TIMER5_interruptVector_[2];

/* Vector objects refs to SPI that causes interruptions */
Spi* InterruptHandler::SPI0_interruptVector_;
Spi* InterruptHandler::SPI1_interruptVector_;
Spi* InterruptHandler::SPI2_interruptVector_;
Spi* InterruptHandler::SPI3_interruptVector_;

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

InterruptHandler &InterruptHandler::getInstance(void)
{
  /* Returns the only instance of the InterruptHandler */
  return instance_;
}

void InterruptHandler::setInterruptHandler(GpioIn& gpio)
{
  /* Get the GPIO port and pin */
  uint32_t port = gpio.getGpioConfig().port;
  uint8_t pin   = gpio.getGpioConfig().pin;

  /* Select the pin number */
  if (pin == GPIO_PIN_0) 	  pin = 0;
  else if (pin == GPIO_PIN_1) pin = 1;
  else if (pin == GPIO_PIN_2) pin = 2;
  else if (pin == GPIO_PIN_3) pin = 3;
  else if (pin == GPIO_PIN_4) pin = 4;
  else if (pin == GPIO_PIN_5) pin = 5;
  else if (pin == GPIO_PIN_6) pin = 6;
  else if (pin == GPIO_PIN_7) pin = 7;

  /* Store a pointer to the GPIO object in the interrupt vector */
  if (port == GPIO_PORTA_BASE) {
	IntPrioritySet(INT_GPIOA, 0xE0); //menor nível de prioridade (0 é o maior)
	GPIOA_interruptVector_[pin] = &gpio;
  }
  else if (port == GPIO_PORTB_BASE) {
	IntPrioritySet(INT_GPIOB, 0xE0);
	GPIOB_interruptVector_[pin] = &gpio;
  }
  else if (port == GPIO_PORTC_BASE) {
	IntPrioritySet(INT_GPIOC, 0xE0);
	GPIOC_interruptVector_[pin] = &gpio;
  }
  else if (port == GPIO_PORTD_BASE) {
	IntPrioritySet(INT_GPIOD, 0xE0);
	GPIOD_interruptVector_[pin] = &gpio;
  }
  else if (port == GPIO_PORTE_BASE) {
	IntPrioritySet(INT_GPIOE, 0xE0);
	GPIOE_interruptVector_[pin] = &gpio;
  }
  else if (port == GPIO_PORTF_BASE){
	IntPrioritySet(INT_GPIOF, 0xE0);
	GPIOF_interruptVector_[pin] = &gpio;
  }
}

void InterruptHandler::clearInterruptHandler(GpioIn& gpio)
{
  /* Get the GPIO port and pin */
  uint32_t port = gpio.getGpioConfig().port;
  uint8_t pin   = gpio.getGpioConfig().pin;

  /* Select the pin number */
  if (pin == GPIO_PIN_0) 	  pin = 0;
  else if (pin == GPIO_PIN_1) pin = 1;
  else if (pin == GPIO_PIN_2) pin = 2;
  else if (pin == GPIO_PIN_3) pin = 3;
  else if (pin == GPIO_PIN_4) pin = 4;
  else if (pin == GPIO_PIN_5) pin = 5;
  else if (pin == GPIO_PIN_6) pin = 6;
  else if (pin == GPIO_PIN_7) pin = 7;

  /* Remove the pointer to the GPIO object in the interrupt vector */
  if (port == GPIO_PORTA_BASE) GPIOA_interruptVector_[pin] = nullptr;
  else if (port == GPIO_PORTB_BASE) GPIOB_interruptVector_[pin] = nullptr;
  else if (port == GPIO_PORTC_BASE) GPIOC_interruptVector_[pin] = nullptr;
  else if (port == GPIO_PORTD_BASE) GPIOD_interruptVector_[pin] = nullptr;
  else if (port == GPIO_PORTE_BASE) GPIOE_interruptVector_[pin] = nullptr;
  else if (port == GPIO_PORTF_BASE) GPIOF_interruptVector_[pin] = nullptr;

}

void InterruptHandler::setInterruptHandler(Timer& timer_) {
  /* Get the TIMER base */
  uint32_t base = timer_.getConfig().base;
  uint32_t source = timer_.getConfig().source;

  if (base == TIMER0_BASE) {
	  if ( (source == TIMER_A) || (source == TIMER_BOTH) ) TIMER0_interruptVector_[0] = &timer_;
	  else if (source == TIMER_B) TIMER0_interruptVector_[1] = &timer_;
  }
  else if (base == TIMER1_BASE) {
	  if ((source == TIMER_A) || (source == TIMER_BOTH)) TIMER1_interruptVector_[0] = &timer_;
	  else if (source == TIMER_B) TIMER1_interruptVector_[1] = &timer_;
  }
  else if (base == TIMER2_BASE) {
	  if ((source == TIMER_A) || (source == TIMER_BOTH)) TIMER2_interruptVector_[0] = &timer_;
	  else if (source == TIMER_B) TIMER2_interruptVector_[1] = &timer_;
  }
  else if (base == TIMER3_BASE) {
	  if ((source == TIMER_A) || (source == TIMER_BOTH)) TIMER3_interruptVector_[0] = &timer_;
	  else if (source == TIMER_B) TIMER3_interruptVector_[1] = &timer_;
  }
  else if (base == TIMER4_BASE) {
	  if ((source == TIMER_A) || (source == TIMER_BOTH)) TIMER4_interruptVector_[0] = &timer_;
	  else if (source == TIMER_B) TIMER4_interruptVector_[1] = &timer_;
  }
  else if (base == TIMER5_BASE) {
	  if ((source == TIMER_A) || (source == TIMER_BOTH)) TIMER5_interruptVector_[0] = &timer_;
	  else if (source == TIMER_B) TIMER5_interruptVector_[1] = &timer_;
  }
}

void InterruptHandler::clearInterruptHandler(Timer& timer_)
{
  /* Get the TIMER base */
  uint32_t base = timer_.getConfig().base;
  uint32_t source = timer_.getConfig().source;

  if (base == TIMER0_BASE) {
	  if ( (source == TIMER_A) || (source == TIMER_BOTH)) TIMER0_interruptVector_[0] = nullptr;
	  else if (source == TIMER_B) TIMER0_interruptVector_[1] = nullptr;
  }
  else if (base == TIMER1_BASE) {
	  if ((source == TIMER_A) || (source == TIMER_BOTH)) TIMER1_interruptVector_[0] = nullptr;
	  else if (source == TIMER_B) TIMER1_interruptVector_[1] = nullptr;
  }
  else if (base == TIMER2_BASE) {
	  if ((source == TIMER_A) || (source == TIMER_BOTH)) TIMER2_interruptVector_[0] = nullptr;
	  else if (source == TIMER_B) TIMER2_interruptVector_[1] = nullptr;
  }
  else if (base == TIMER3_BASE) {
	  if ((source == TIMER_A) || (source == TIMER_BOTH)) TIMER3_interruptVector_[0] = nullptr;
	  else if (source == TIMER_B) TIMER3_interruptVector_[1] = nullptr;
  }
  else if (base == TIMER4_BASE) {
	  if ((source == TIMER_A) || (source == TIMER_BOTH)) TIMER4_interruptVector_[0] = nullptr;
	  else if (source == TIMER_B) TIMER4_interruptVector_[1] = nullptr;
  }
  else if (base == TIMER5_BASE) {
	  if ((source == TIMER_A) || (source == TIMER_BOTH)) TIMER5_interruptVector_[0] = nullptr;
	  else if (source == TIMER_B) TIMER5_interruptVector_[1] = nullptr;
  }
}


void InterruptHandler::setInterruptHandler(Spi& spi_)
{
  /* Get the SPI base */
  uint32_t base = spi_.getConfig().base;

  /* Store a pointer to the SPI object in the interrupt vector */
  if (base == SSI0_BASE)
  {
    SPI0_interruptVector_ = &spi_;
  }
  else if (base == SSI1_BASE)
  {
    SPI1_interruptVector_ = &spi_;
  }
  else if (base == SSI2_BASE) {
	SPI2_interruptVector_ = &spi_;
  }
  else if (base == SSI3_BASE) {
	SPI3_interruptVector_ = &spi_;
  }
}

void InterruptHandler::clearInterruptHandler(Spi& spi_)
{
  /* Get the SPI base */
  uint32_t base = spi_.getConfig().base;

  /* Remove the pointer to the SPI object in the interrupt vector */
  if (base == SSI0_BASE)
  {
    SPI0_interruptVector_ = nullptr;
  }
  else if (base == SSI1_BASE)
  {
    SPI1_interruptVector_ = nullptr;
  }
  else if (base == SSI2_BASE) {
	SPI2_interruptVector_ = nullptr;
  }
  else if (base == SSI3_BASE) {
	SPI3_interruptVector_ = nullptr;
  }
}
/*=============================== protected =================================*/

/*================================ private ==================================*/

InterruptHandler::InterruptHandler()
{
}

void InterruptHandler::init() {
	/* Register the GPIO interrupt handlers */
	GPIOIntRegister(GPIO_PORTA_BASE, GPIOA_InterruptHandler);
	GPIOIntRegister(GPIO_PORTB_BASE, GPIOB_InterruptHandler);
	GPIOIntRegister(GPIO_PORTC_BASE, GPIOC_InterruptHandler);
	GPIOIntRegister(GPIO_PORTD_BASE, GPIOD_InterruptHandler);
	GPIOIntRegister(GPIO_PORTE_BASE, GPIOE_InterruptHandler);
	GPIOIntRegister(GPIO_PORTF_BASE, GPIOF_InterruptHandler);

	/* Register the Timer interrupt handlers */
	TimerIntRegister(TIMER0_BASE, TIMER_A, TIMER0_InterruptHandler);
	TimerIntRegister(TIMER0_BASE, TIMER_B, TIMER0_InterruptHandler);
	TimerIntRegister(TIMER0_BASE, TIMER_BOTH, TIMER0_InterruptHandler);

	TimerIntRegister(TIMER1_BASE, TIMER_A, TIMER1_InterruptHandler);
	TimerIntRegister(TIMER1_BASE, TIMER_B, TIMER1_InterruptHandler);
	TimerIntRegister(TIMER1_BASE, TIMER_BOTH, TIMER1_InterruptHandler);

	TimerIntRegister(TIMER2_BASE, TIMER_A, TIMER2_InterruptHandler);
	TimerIntRegister(TIMER2_BASE, TIMER_B, TIMER2_InterruptHandler);
	TimerIntRegister(TIMER2_BASE, TIMER_BOTH, TIMER2_InterruptHandler);

	TimerIntRegister(TIMER3_BASE, TIMER_A, TIMER3_InterruptHandler);
	TimerIntRegister(TIMER3_BASE, TIMER_B, TIMER3_InterruptHandler);
	TimerIntRegister(TIMER3_BASE, TIMER_BOTH, TIMER3_InterruptHandler);

	TimerIntRegister(TIMER4_BASE, TIMER_A, TIMER4_InterruptHandler);
	TimerIntRegister(TIMER4_BASE, TIMER_B, TIMER4_InterruptHandler);
	TimerIntRegister(TIMER4_BASE, TIMER_BOTH, TIMER4_InterruptHandler);

	TimerIntRegister(TIMER5_BASE, TIMER_A, TIMER5_InterruptHandler);
	TimerIntRegister(TIMER5_BASE, TIMER_B, TIMER5_InterruptHandler);
	TimerIntRegister(TIMER5_BASE, TIMER_BOTH, TIMER5_InterruptHandler);

	SSIIntRegister(SSI0_BASE, SPI0_InterruptHandler);
	SSIIntRegister(SSI1_BASE, SPI1_InterruptHandler);
	SSIIntRegister(SSI2_BASE, SPI2_InterruptHandler);
	SSIIntRegister(SSI3_BASE, SPI3_InterruptHandler);

}

void InterruptHandler::GPIO_InterruptHandler(uint32_t base, GpioIn* vector[8]) {

  /* Read the GPIO interrupt status */
  uint32_t status = GPIOIntStatus(base, true);

  /* Clear the GPIO interrupt status */
  GPIOIntClear(base, status);

  /* Call all the GPIO interrupt handlers */
  /*obs: é possível que mais de um pino tenha gerado interrupção nessa porta. As ISRs são executadas na ordem dos pinos (0 -- 7) */
  if ((status & GPIO_PIN_0) && (vector[0] != nullptr)) vector[0]->interruptHandler();
  if ((status & GPIO_PIN_1) && (vector[1] != nullptr)) vector[1]->interruptHandler();
  if ((status & GPIO_PIN_2) && (vector[2] != nullptr)) vector[2]->interruptHandler();
  if ((status & GPIO_PIN_3) && (vector[3] != nullptr)) vector[3]->interruptHandler();
  if ((status & GPIO_PIN_4) && (vector[4] != nullptr)) vector[4]->interruptHandler();
  if ((status & GPIO_PIN_5) && (vector[5] != nullptr)) vector[5]->interruptHandler();
  if ((status & GPIO_PIN_6) && (vector[6] != nullptr)) vector[6]->interruptHandler();
  if ((status & GPIO_PIN_7) && (vector[7] != nullptr)) vector[7]->interruptHandler();

}

void InterruptHandler::TIMER_InterruptHandler(uint32_t base, Timer* vector[2]) {
	uint32_t status;

	status = TimerIntStatus(base, true);
	TimerIntClear(base, status);

	if (base == TIMER0_BASE) {
		if ((TIMER0_interruptVector_[0] != nullptr) && \
			((status & TIMER_TIMA_MATCH) || \
			(status & TIMER_CAPA_EVENT) || \
			(status & TIMER_TIMA_TIMEOUT) || \
			(status & TIMER_CAPA_MATCH)))
		{
			TIMER0_interruptVector_[0]->interruptHandler();
		}

		if ((TIMER0_interruptVector_[1] != nullptr) && \
			((status & TIMER_TIMB_MATCH) || \
			(status & TIMER_CAPB_EVENT) || \
			(status & TIMER_TIMB_TIMEOUT) || \
			(status & TIMER_CAPB_MATCH)))
		{
			TIMER0_interruptVector_[1]->interruptHandler();
		}
	}

	if (base == TIMER1_BASE) {
		if ((TIMER1_interruptVector_[0] != nullptr) && \
			((status & TIMER_TIMA_MATCH) || \
			(status & TIMER_CAPA_EVENT) || \
			(status & TIMER_TIMA_TIMEOUT) || \
			(status & TIMER_CAPA_MATCH)))
		{
			TIMER1_interruptVector_[0]->interruptHandler();
		}

		if ((TIMER1_interruptVector_[1] != nullptr) && \
			((status & TIMER_TIMB_MATCH) || \
			(status & TIMER_CAPB_EVENT) || \
			(status & TIMER_TIMB_TIMEOUT) || \
			(status & TIMER_CAPB_MATCH)))
		{
			TIMER1_interruptVector_[1]->interruptHandler();
		}
	}

	if (base == TIMER2_BASE) {
		if ((TIMER2_interruptVector_[0] != nullptr) && \
			((status & TIMER_TIMA_MATCH) || \
			(status & TIMER_CAPA_EVENT) || \
			(status & TIMER_TIMA_TIMEOUT) || \
			(status & TIMER_CAPA_MATCH)))
		{
			TIMER2_interruptVector_[0]->interruptHandler();
		}

		if ((TIMER2_interruptVector_[1] != nullptr) && \
			((status & TIMER_TIMB_MATCH) || \
			(status & TIMER_CAPB_EVENT) || \
			(status & TIMER_TIMB_TIMEOUT) || \
			(status & TIMER_CAPB_MATCH)))
		{
			TIMER2_interruptVector_[1]->interruptHandler();
		}
	}

	if (base == TIMER3_BASE) {
		if ((TIMER3_interruptVector_[0] != nullptr) && \
			((status & TIMER_TIMA_MATCH) || \
			(status & TIMER_CAPA_EVENT) || \
			(status & TIMER_TIMA_TIMEOUT) || \
			(status & TIMER_CAPA_MATCH)))
		{
			TIMER3_interruptVector_[0]->interruptHandler();
		}

		if ((TIMER3_interruptVector_[1] != nullptr) && \
			((status & TIMER_TIMB_MATCH) || \
			(status & TIMER_CAPB_EVENT) || \
			(status & TIMER_TIMB_TIMEOUT) || \
			(status & TIMER_CAPB_MATCH)))
		{
			TIMER3_interruptVector_[1]->interruptHandler();
		}
	}

	if (base == TIMER4_BASE) {
		if ((TIMER4_interruptVector_[0] != nullptr) && \
			((status & TIMER_TIMA_MATCH) || \
			(status & TIMER_CAPA_EVENT) || \
			(status & TIMER_TIMA_TIMEOUT) || \
			(status & TIMER_CAPA_MATCH)))
		{
			TIMER4_interruptVector_[0]->interruptHandler();
		}

		if ((TIMER4_interruptVector_[1] != nullptr) && \
			((status & TIMER_TIMB_MATCH) || \
			(status & TIMER_CAPB_EVENT) || \
			(status & TIMER_TIMB_TIMEOUT) || \
			(status & TIMER_CAPB_MATCH)))
		{
			TIMER4_interruptVector_[1]->interruptHandler();
		}
	}

	if (base == TIMER5_BASE) {
		if ((TIMER5_interruptVector_[0] != nullptr) && \
			((status & TIMER_TIMA_MATCH) || \
			(status & TIMER_CAPA_EVENT) || \
			(status & TIMER_TIMA_TIMEOUT) || \
			(status & TIMER_CAPA_MATCH)))
		{
			TIMER5_interruptVector_[0]->interruptHandler();
		}

		if ((TIMER5_interruptVector_[1] != nullptr) && \
			((status & TIMER_TIMB_MATCH) || \
			(status & TIMER_CAPB_EVENT) || \
			(status & TIMER_TIMB_TIMEOUT) || \
			(status & TIMER_CAPB_MATCH)))
		{
			TIMER5_interruptVector_[1]->interruptHandler();
		}
	}
}

//void InterruptHandler::SPI_InterruptHandler(uint32_t base, Spi* vector) {
//	uint32_t status;
//
//	status = SSIIntStatus(base, true);
//	SSIIntClear(base, status);
//
//	if (base == SSI0_BASE) {
//		if ((SPI0_interruptVector_ != nullptr) && \
//			((status & SSI_TXEOT) || \
//			(status & SSI_DMATX) || \
//			(status & SSI_DMARX) || \
//			(status & SSI_TXFF) || \
//			(status & SSI_RXFF) || \
//			(status & SSI_RXTO) || \
//			(status & SSI_RXOR)))
//		{
//			SPI0_interruptVector_->interruptHandler();
//		}
//	}
//	else if (base == SSI1_BASE) {
//		if ((SPI1_interruptVector_ != nullptr) && \
//			((status & SSI_TXEOT) || \
//			(status & SSI_DMATX) || \
//			(status & SSI_DMARX) || \
//			(status & SSI_TXFF) || \
//			(status & SSI_RXFF) || \
//			(status & SSI_RXTO) || \
//			(status & SSI_RXOR)))
//		{
//			SPI1_interruptVector_->interruptHandler();
//		}
//	}
//	else if (base == SSI2_BASE) {
//		if ((SPI2_interruptVector_ != nullptr) && \
//			((status & SSI_TXEOT) || \
//			(status & SSI_DMATX) || \
//			(status & SSI_DMARX) || \
//			(status & SSI_TXFF) || \
//			(status & SSI_RXFF) || \
//			(status & SSI_RXTO) || \
//			(status & SSI_RXOR)))
//		{
//			SPI2_interruptVector_->interruptHandler();
//		}
//	}
//	else if (base == SSI3_BASE) {
//		if ((SPI3_interruptVector_ != nullptr) && \
//			((status & SSI_TXEOT) || \
//			(status & SSI_DMATX) || \
//			(status & SSI_DMARX) || \
//			(status & SSI_TXFF) || \
//			(status & SSI_RXFF) || \
//			(status & SSI_RXTO) || \
//			(status & SSI_RXOR)))
//		{
//			SPI3_interruptVector_->interruptHandler();
//		}
//	}
//}

/**************************** GPIO ****************************/
void InterruptHandler::GPIOA_InterruptHandler(void) {
	GPIO_InterruptHandler(GPIO_PORTA_BASE, GPIOA_interruptVector_);
}

void InterruptHandler::GPIOB_InterruptHandler(void) {
	GPIO_InterruptHandler(GPIO_PORTB_BASE, GPIOB_interruptVector_);
}

void InterruptHandler::GPIOC_InterruptHandler(void) {
	GPIO_InterruptHandler(GPIO_PORTC_BASE, GPIOC_interruptVector_);
}

void InterruptHandler::GPIOD_InterruptHandler(void) {
	GPIO_InterruptHandler(GPIO_PORTD_BASE, GPIOD_interruptVector_);
}

void InterruptHandler::GPIOE_InterruptHandler(void) {
	GPIO_InterruptHandler(GPIO_PORTE_BASE, GPIOE_interruptVector_);
}

void InterruptHandler::GPIOF_InterruptHandler(void) {
	GPIO_InterruptHandler(GPIO_PORTF_BASE, GPIOF_interruptVector_);
}
/**************************** GPIO end ****************************/

/**************************** Timers ****************************/
void InterruptHandler::TIMER0_InterruptHandler(void){
  TIMER_InterruptHandler(TIMER0_BASE, TIMER0_interruptVector_);
}

void InterruptHandler::TIMER1_InterruptHandler(void){
  TIMER_InterruptHandler(TIMER1_BASE, TIMER1_interruptVector_);
}

void InterruptHandler::TIMER2_InterruptHandler(void){
  TIMER_InterruptHandler(TIMER2_BASE, TIMER2_interruptVector_);
}

void InterruptHandler::TIMER3_InterruptHandler(void){
  TIMER_InterruptHandler(TIMER3_BASE, TIMER3_interruptVector_);
}

void InterruptHandler::TIMER4_InterruptHandler(void){
  TIMER_InterruptHandler(TIMER4_BASE, TIMER4_interruptVector_);
}

void InterruptHandler::TIMER5_InterruptHandler(void){
  TIMER_InterruptHandler(TIMER5_BASE, TIMER5_interruptVector_);
}
/**************************** Timers end ****************************/

/**************************** SPI ****************************/

void InterruptHandler::SPI0_InterruptHandler(void)
{
  if (SPI0_interruptVector_ != nullptr)
  {
    /* Call the SPI interrupt handler */
    SPI0_interruptVector_->interruptHandler();
  }
}

void InterruptHandler::SPI1_InterruptHandler(void)
{
  if (SPI1_interruptVector_ != nullptr)
  {
    /* Call the SPI interrupt handler */
    SPI1_interruptVector_->interruptHandler();
  }
}

void InterruptHandler::SPI2_InterruptHandler(void)
{
  if (SPI2_interruptVector_ != nullptr)
  {
    /* Call the SPI interrupt handler */
    SPI2_interruptVector_->interruptHandler();
  }
}

void InterruptHandler::SPI3_InterruptHandler(void)
{
  if (SPI3_interruptVector_ != nullptr)
  {
    /* Call the SPI interrupt handler */
    SPI3_interruptVector_->interruptHandler();
  }
}


/**************************** SPI end ****************************/
