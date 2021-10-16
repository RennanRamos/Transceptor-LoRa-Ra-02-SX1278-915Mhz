#ifndef GPIO_HPP_
#define GPIO_HPP_

#include <stdint.h>

class GpioConfig {
public:
	bool operator==(const GpioConfig& other) {return (port == other.port) && (pin == other.pin);}
public:
	uint32_t port;
	uint32_t pin;
	uint32_t ioc;
	uint32_t edge;
	uint32_t invert;
	uint32_t strength;
	uint32_t pintype;
};

class Gpio {
	public:
	  Gpio();
	  GpioConfig getGpioConfig(void);
	  void gpioUnlock(void);
	  void padConfig(void);
	public:
	  bool operator==(const Gpio& other);
	protected:
	  GpioConfig config_;
};

/*****************************************************************************/

class GpioIn: public Gpio {
	friend class InterruptHandler; //permite acessar atributos privados/protegidos em InterruptHandler
	public:
	  GpioIn();
	  bool read(void);
	  void enable(GpioConfig& config);
	  void setCallback(Callback* callback);
	  void clearCallback(void);
	  void enableInterrupts(void);
	  void disableInterrupts(void);
	protected:
	  Callback* callback_;
	  void interruptHandler(void);
};


/*****************************************************************************/

class GpioOut: public Gpio {
	public:
	  GpioOut();
	  void on(void);
	  void off(void);
	  void toggle(void);
	  void high(void);
	  void low(void);
	  uint32_t status(void);
	  void enable(GpioConfig& config);
};

/*****************************************************************************/

class GpioSpi: public Gpio {
	public:
		GpioSpi();
		void enable(GpioConfig& config);
		void disable();
};


#endif /* GPIO_HPP_ */
