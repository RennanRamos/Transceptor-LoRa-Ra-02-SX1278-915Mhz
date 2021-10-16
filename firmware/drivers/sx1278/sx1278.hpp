#ifndef SX1278_HPP_
#define SX1278_HPP_

#include "sx1278_regs.h"
#include "sx1278_conf.h"
#include "inc/platform_defines.h"
#include "src/rp2_common/hardware_spi/include/hardware/spi.h"
#include "src/rp2040/hardware_structs/include/hardware/structs/spi.h"

class sx1278
{

friend class InterruptHandler;

public:

typedef enum
{
  CORE_RFLORA = 0x00,
} RadioCore;

typedef enum
{
  STATE_TRXOFF = RF_STATE_TRXOFF,
  STATE_TXPREP = RF_STATE_TXPREP,
  STATE_TX     = RF_STATE_TX,
  STATE_RX     = RF_STATE_RX,
  STATE_TRANS  = RF_STATE_TRANS,
  STATE_RESET  = RF_STATE_RESET
} RadioState;

typedef enum
{
  CMD_NOP    = RF_CMD_NOP,
  CMD_SLEEP  = RF_CMD_SLEEP,
  CMD_TRXOFF = RF_CMD_TRXOFF, 
  CMD_TXPREP = RF_CMD_TXPREP,
  CMD_TX     = RF_CMD_TX,
  CMD_RX     = RF_CMD_RX,
  CMD_RESET  = RF_CMD_RESET
} RadioCommand;

typedef enum
{
  TX_POWER_MIN = 0, 
  TX_POWER_0 = 0, TX_POWER_1, TX_POWER_2, TX_POWER_3, TX_POWER_4, TX_POWER_5,
  TX_POWER_6,  TX_POWER_7,  TX_POWER_8,  TX_POWER_9,  TX_POWER_10,
  TX_POWER_11, TX_POWER_12, TX_POWER_13, TX_POWER_14, TX_POWER_15,
  TX_POWER_16, TX_POWER_17, TX_POWER_18, TX_POWER_19, TX_POWER_20,
  TX_POWER_21, TX_POWER_22, TX_POWER_23, TX_POWER_24, TX_POWER_25,
  TX_POWER_26, TX_POWER_27, TX_POWER_28, TX_POWER_29, TX_POWER_30,
  TX_POWER_31 = 31, TX_POWER_MAX = 31
} TransmitPower;

typedef enum
{
  Error       = -1,
  Success     =  0
} RadioResult;

public:
  sx1278();
  void enable(GpioOut* pwr, GpioOut* rst, GpioIn* irq);
  
  void on(void);
  void off(void);
  
  void hardReset(void);
  void softReset(RadioCore rc);

  bool check(void);
  
  void configure(RadioCore rc, const radio_settings_t* radio_settings, const frequency_settings_t* frequency_settings, uint16_t channel);
  
  void sleep(RadioCore rc);
  void wakeup(RadioCore rc);
  void transmit(RadioCore rc);
  void receive(RadioCore rc);
  
  bool cca(RadioCore rc, int8_t threshold, int8_t* rssi, bool* rssi_valid);
  bool csma(RadioCore rc, int8_t cca_threshold, uint8_t* retries, int8_t* rssi);

  void setRxCallbacks(RadioCore rc, Callback* rxInit, Callback* rxDone);
  void setTxCallbacks(RadioCore rc, Callback* txInit, Callback* txDone);
  void enableInterrupts(void);
  void disableInterrupts(void);
  
  RadioResult getRSSI(RadioCore rc, int8_t* rssi);
  RadioResult getED(RadioCore rc, int8_t* edv);
  RadioResult setTransmitPower(RadioCore rc, TransmitPower power);
  
  RadioResult loadPacket(RadioCore rc, uint8_t* data, uint16_t length);
  RadioResult getPacket(RadioCore rc, uint8_t* buffer, uint16_t* length, int8_t* rssi, int8_t* lqi, bool* crc);

public:
  void setContinuousTransmission(RadioCore rc, bool enable);
protected:
  void interruptHandler(void);
  void interruptHandler_rf24(uint8_t rf24_irqs, uint8_t bbc24_irqs); 
  void interruptHandler_rfLORA(uint8_t rfLORA_irqs, uint8_t bbcLORA_irqs);
private:
  RadioState getState(RadioCore rc);
  void goToState(RadioCore rc, RadioCommand cmd, RadioState target);
  void writeCmd(RadioCore rc, RadioCommand cmd);
private:
  void singleAccessRead(uint16_t address, uint8_t* value);
  void singleAccessWrite(uint16_t address, uint8_t value);
  void blockAccessRead(uint16_t address, uint8_t* values, uint16_t length);
  void blockAccessWrite(uint16_t address, uint8_t* values, uint16_t length);
private:
  uint16_t getCORERegisterAddress(RadioCore rc);
  uint16_t getRFRegisterAddress(RadioCore rc, uint16_t address);
  uint16_t getBBCRegisterAddress(RadioCore rc, uint16_t address);
  uint16_t getFBRegisterAddress(RadioCore rc, uint16_t address);
  uint16_t getCRCLength(RadioCore rc);
private:  
  void enableBitFromRegister(uint16_t address, uint32_t mask);
  void disableBitFromRegister(uint16_t address, uint32_t mask);
private:

  spi_inst_t spi_;

  uint8_t rfLORA_irqm;
  uint8_t bbc0_irqm, bbc1_irqm;

  uint8_t crc_length;
  
};

#endif /* SX1278_HPP_ */
