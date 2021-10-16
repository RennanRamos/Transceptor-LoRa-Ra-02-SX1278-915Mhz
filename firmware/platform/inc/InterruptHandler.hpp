#ifndef INTERRUPT_HANDLER_HPP_
#define INTERRUPT_HANDLER_HPP_

#include <stdint.h>
#include "inc/Gpio.hpp"
#include "inc/Timer.hpp"
#include "inc/Spi.hpp"
#include "inc/hw_memmap.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"

class GpioIn;
class Spi;

class InterruptHandler {

public:
    static InterruptHandler& getInstance(void);

    //Interrupt para GpioIn
    static void setInterruptHandler(GpioIn& gpio);
    static void clearInterruptHandler(GpioIn& gpio);

    //Interrupt para timer
    static void setInterruptHandler(Timer& timer);
	static void clearInterruptHandler(Timer& timer);
	//Interrupt para SPI
	static void setInterruptHandler(Spi& spi);
	static void clearInterruptHandler(Spi& spi);

    void init(void);
private:
    InterruptHandler();
    // GpioIn ports
    static void GPIO_InterruptHandler(uint32_t base, GpioIn* vector[8]);
    static void GPIOA_InterruptHandler(void); //ISR da porta A
    static void GPIOB_InterruptHandler(void); //ISR da porta B
    static void GPIOC_InterruptHandler(void); //ISR da porta C
    static void GPIOD_InterruptHandler(void); //ISR da porta D
    static void GPIOE_InterruptHandler(void); //ISR da porta E
    static void GPIOF_InterruptHandler(void); //ISR da porta F

    // Timers
    static void TIMER_InterruptHandler(uint32_t base, Timer* vector[2]);
	static void TIMER0_InterruptHandler(void);
	static void TIMER1_InterruptHandler(void);
	static void TIMER2_InterruptHandler(void);
	static void TIMER3_InterruptHandler(void);
	static void TIMER4_InterruptHandler(void);
	static void TIMER5_InterruptHandler(void);

	// SPI
	static void SPI0_InterruptHandler(void);
	static void SPI1_InterruptHandler(void);
	static void SPI2_InterruptHandler(void);
	static void SPI3_InterruptHandler(void);
private:
    static InterruptHandler instance_;
    // Gpio ports
    static GpioIn* GPIOA_interruptVector_[8]; //array com refs de cada pino
    static GpioIn* GPIOB_interruptVector_[8]; //array com refs de cada pino
    static GpioIn* GPIOC_interruptVector_[8]; //array com refs de cada pino
    static GpioIn* GPIOD_interruptVector_[8]; //array com refs de cada pino
    static GpioIn* GPIOE_interruptVector_[8]; //array com refs de cada pino
    static GpioIn* GPIOF_interruptVector_[8]; //array com refs de cada pino

    // Timers
    static Timer* TIMER0_interruptVector_[2];
    static Timer* TIMER1_interruptVector_[2];
    static Timer* TIMER2_interruptVector_[2];
    static Timer* TIMER3_interruptVector_[2];
    static Timer* TIMER4_interruptVector_[2];
    static Timer* TIMER5_interruptVector_[2];

    // SSI
	static Spi* SPI0_interruptVector_;
	static Spi* SPI1_interruptVector_;
	static Spi* SPI2_interruptVector_;
	static Spi* SPI3_interruptVector_;
	static void SPI_InterruptHandler(uint32_t base, Spi* vector);
};

#endif /* INTERRUPT_HANDLER_HPP_ */
