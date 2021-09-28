/*================================ include ==================================*/

#include <stdlib.h>

#include "sx1278.hpp"
#include "sx1278_regs.h"
#include "sx1278_conf.h"

#include "BoardImplementation.hpp"

#include "platform_types.h"

/*================================ define ===================================*/

#define sx1278_RSSI_MAX_VALUE        ( 127 )

#define sx1278_ED_MIN_VALUE          ( -127 )
#define sx1278_ED_MAX_VALUE          ( 4 )

#define sx1278_DELAY_MS              ( 1 )
#define sx1278_DELAY_US              ( 10 )

#define sx1278_RFn_IRQM              ( 0x00 )
#define sx1278_BBCn_IRQM             ( 0x13 )

#define sx1278_RF_CFG_DEFAULT        ( 0x08 )
#define sx1278_RF_CLK0_DEFAULT       ( 0x00 ) 

#define sx1278_RFn_PAC_PACUR_SHIFT   ( 4 )
#define sx1278_RFn_PAC_PACUR_MASK    ( 0x06 << sx1278_RFn_PAC_PACUR_SHIFT)
#define sx1278_RFn_PAC_PACUR_3dB     ( 0x00 )
#define sx1278_RFn_PAC_PACUR_2dB     ( 0x01 )
#define sx1278_RFn_PAC_PACUR_1dB     ( 0x02 )
#define sx1278_RFn_PAC_PACUR_0dB     ( 0x03 )

#define sx1278_RFn_PAC_TXPWR_MASK    ( 0x1F )

#define sx1278_BBCn_PC_CTX_MASK      ( 0x80 )
#define sx1278_BBCn_PC_FCST_MASK     ( 0x08 )
#define sx1278_BBCn_PC_FCSOK_MASK    ( 0x20 )

#define sx1278_BBCn_IRQS_RXFS_MASK   ( 0x01 )
#define sx1278_BBCn_IRQS_RXFE_MASK   ( 0x02 )
#define sx1278_BBCn_IRQS_TXFE_MASK   ( 0x10 )

#define sx1278_RFn_IRQS_EDC_MASK     ( 0x02 )

#define sx1278_BBCn_PC_BBEN_MASK     ( 0x01 << 2 )

#define sx1278_RFn_EDC_EDM_AUTO      ( 0x00 << 0 )
#define sx1278_RFn_EDC_EDM_SINGLE    ( 0x01 << 0 )
#define sx1278_RFn_EDC_EDM_CONT      ( 0x02 << 0 )
#define sx1278_RFn_EDC_EDM_OFF       ( 0x03 << 0 )

#define sx1278_RFn_EDD_DF_(x)        ( ( x & 0x3F ) << 2 )
#define sx1278_RFn_EDD_DTB_2_US      ( 0x00 << 0 )
#define sx1278_RFn_EDD_DTB_8_US      ( 0x01 << 0 )
#define sx1278_RFn_EDD_DTB_32_US     ( 0x02 << 0 )
#define sx1278_RFn_EDD_DTB_128_US    ( 0x03 << 0 )

#define sx1278_BBCn_PC_FCST_16_BIT   ( 2 )
#define sx1278_BBCn_PC_FCST_32_BIT   ( 4 )

#define sx1278_EDV_READ_RETRIES      ( 4 )
#define sx1278_EDV_INVALID           ( 127 )

#define sx1278_CCA_RETRIES           ( 2 )
#define sx1278_CCA_DELAY_US          ( 200 )

#define sx1278_CSMA_RETRIES          ( 3 )
#define sx1278_CSMA_DELAY_US         ( 100 )
#define sx1278_CSMA_MODULUS          ( 100 )

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

extern BoardImplementation board;
extern NumberGenerator prng;

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

sx1278::sx1278() {
}


void sx1278::enable(Spi *spi, GpioOut *pwr, GpioOut *rst, GpioOut *cs,
    GpioIn *irq) {


  rxLORAInit_ = nullptr;
  rxLORADone_ = nullptr;
  txLORAInit_ = nullptr;
  txLORADone_ = nullptr;
  

  rfLORA_irqm = 0;
  bbc1_irqm = 0;

  dma_ = false;

  cs_ = cs;
  spi_ = spi;
  pwr_ = pwr;
  rst_ = rst;
  irq_ = irq;


  off();


  irq_->setCallback(&callback_);
}

void sx1278::on(void) {
  /* While on, all pins are high and wait */
  cs_->high();
  rst_->high();
  pwr_->high();

  board.delayMilliseconds(sx1278_DELAY_MS);
}

void sx1278::off(void) {
  /* While off, all pins are low */
  cs_->low();
  rst_->low();
  pwr_->low();

  board.delayMilliseconds(sx1278_DELAY_MS);
}

void sx1278::hardReset(void) {

  rst_->low();
  board.delayMilliseconds(sx1278_DELAY_MS);


  rst_->high();
  board.delayMilliseconds(sx1278_DELAY_MS);
}

void sx1278::softReset(RadioCore rc) {
  
  goToState(rc, RadioCommand::CMD_RESET, RadioState::STATE_RESET);
}

bool sx1278::check(void) {
  bool status = false;

  uint8_t pn = 0;
  uint8_t ver = 0;

  /* Read part number register */
  singleAccessRead(RF_PN, &pn);


  if (pn == PN_215) {


    singleAccessRead(RF_VN, &ver);


    if (ver == VN_1_1 || ver == VN_1_3) {
      status = true;
    }
  }
  return status;
}

void sx1278::configure(RadioCore rc, const radio_settings_t *radio_settings,
    const frequency_settings_t *frequency_settings, uint16_t channel) {
  uint16_t rf_base, bbc_base;
  uint32_t frequency;
  uint16_t address;
  uint8_t value;

  
  if (rc == RadioCore::CORE_RFLORA) {
    frequency = frequency_settings->frequency0;
  }
  else {
    frequency = frequency_settings->frequency0 - 1500000;
  }

  
  if (channel > frequency_settings->channel_max) {
    channel = frequency_settings->channel_max;
  }

  register_t2 channel_settings[] = { { RFn_CCF0L, (uint8_t) ((frequency / 25)
      % 256) }, { RFn_CCF0H, (uint8_t) ((frequency / 25) / 256) }, { RFn_CS,
      (uint8_t) (frequency_settings->channel_spacing / 25) }, { RFn_CNL,
      (uint8_t) (channel % 256) }, { RFn_CNM, (uint8_t) (channel / 256) }, };

  
  rf_base = getRFRegisterAddress(rc, 0x00);
  bbc_base = getBBCRegisterAddress(rc, 0x00);

  singleAccessWrite(RF_CFG, sx1278_RF_CFG_DEFAULT);

  
  singleAccessWrite(RF_CLKO, sx1278_RF_CLK0_DEFAULT);

  
  for (uint16_t i = 0; i < sizeof(irq_settings) / sizeof(irq_settings[0]);
      i++) {
    
    address = irq_settings[i].address;
    value = irq_settings[i].value;

    
    singleAccessWrite(address, value);

    
    *irq_mask[i] = value;
  }

  
  for (uint16_t i = 0; i < radio_settings->rf_elements; i++) {
    
    address = rf_base + radio_settings->rf_registers[i].address;
    value = radio_settings->rf_registers[i].value;

    
    singleAccessWrite(address, value);
  }

  
  for (uint16_t i = 0; i < radio_settings->bbc_elements; i++) {
    
    address = bbc_base + radio_settings->bbc_registers[i].address;
    value = radio_settings->bbc_registers[i].value;

   
    singleAccessWrite(address, value);
  }

  
  for (uint16_t i = 0;
      i < sizeof(channel_settings) / sizeof(channel_settings[0]); i++) {
    
    address = rf_base + channel_settings[i].address;
    value = channel_settings[i].value;

    
    singleAccessWrite(address, value);
  }

  
  crc_length = getCRCLength(rc);
}

void sx1278::sleep(RadioCore rc) {
  goToState(rc, RadioCommand::CMD_TRXOFF, RadioState::STATE_TRXOFF);
}

void sx1278::wakeup(RadioCore rc) {
  goToState(rc, RadioCommand::CMD_TXPREP, RadioState::STATE_TXPREP);
}

void sx1278::transmit(RadioCore rc) {
  goToState(rc, RadioCommand::CMD_TX, RadioState::STATE_TX);

  if (rc == CORE_RFLORA && txLORAInit_ != nullptr) {
    txLORAInit_->execute();
  }
}

void sx1278::receive(RadioCore rc) {
  goToState(rc, RadioCommand::CMD_RX, RadioState::STATE_RX);
}

bool sx1278::cca(RadioCore rc, int8_t cca_threshold, int8_t *rssi,
    bool *rssi_valid) {
  uint16_t bbcn_pc_address;
  uint16_t rfn_edv_address, rfn_edc_address;
  uint16_t irqs_address;
  bool cca_status = false;
  int8_t edv;


  irqs_address = sx1278::getCORERegisterAddress(rc);
  bbcn_pc_address = sx1278::getBBCRegisterAddress(rc, BBCn_PC);
  rfn_edc_address = sx1278::getRFRegisterAddress(rc, RFn_EDC);
  rfn_edv_address = sx1278::getRFRegisterAddress(rc, RFn_RSSI);


  disableBitFromRegister(bbcn_pc_address, sx1278_BBCn_PC_BBEN_MASK);


  receive(rc);


  singleAccessWrite(rfn_edc_address, sx1278_RFn_EDC_EDM_SINGLE);


  bool status;
  do {
    uint8_t rf_irqs;

    singleAccessRead(irqs_address, &rf_irqs);

    status = ((rf_irqs & sx1278_RFn_IRQS_EDC_MASK)
        == sx1278_RFn_IRQS_EDC_MASK);
  }
  while (!status);


  singleAccessRead(rfn_edv_address, (uint8_t*) &edv);


  uint8_t read_retries = sx1278_EDV_READ_RETRIES;
  do {

    singleAccessRead(rfn_edv_address, (uint8_t*) &edv);

    read_retries--;

  }
  while ((edv == sx1278_EDV_INVALID) && (read_retries > 0));


  if (edv != sx1278_EDV_INVALID) {

    *rssi = edv;
    *rssi_valid = true;


    if (edv < cca_threshold) {

      cca_status = true;
    }
  }
  else {
    *rssi_valid = false;
  }


  enableBitFromRegister(bbcn_pc_address, sx1278_BBCn_PC_BBEN_MASK);


  wakeup(rc);

  return cca_status;
}

bool sx1278::csma(RadioCore rc, int8_t cca_threshold, uint8_t *retries,
    int8_t *rssi) {
  int8_t rssi_values[sx1278_CSMA_RETRIES] = { 0 };
  uint8_t rssi_position = 0;
  int8_t rssi_mean;


  uint8_t csma_retries = sx1278_CSMA_RETRIES;
  bool cca_status = false;


  do {
    int8_t rssi;
    bool rssi_valid;

    cca_status = cca(rc, cca_threshold, &rssi, &rssi_valid);

    if (rssi_valid) {

      rssi_values[rssi_position] = rssi;
      uart.write(&rssi, sizeof(rssi));

      rssi_position++;
    }


    csma_retries--;


    if (!cca_status && csma_retries > 0) {
      uint32_t delay_us;
      uint16_t slots;


      slots = prng.get() % sx1278_CSMA_MODULUS;

      delay_us = slots * sx1278_CSMA_DELAY_US;

      board.delayMicroseconds(delay_us);
    }
  }
  while (!cca_status && csma_retries > 0);


  if (cca_status && csma_retries > 0) {

    int32_t accumul = 0;
    for (uint8_t i = 0; i < rssi_position; i++) {
      accumul += rssi_values[i];
    }
    rssi_mean = accumul / rssi_position;

    *retries = rssi_position;
    *rssi = rssi_mean;
  }

  return cca_status;
}

void sx1278::setRxCallbacks(RadioCore rc, Callback *rxInit,
    Callback *rxDone) {
  if (rc == RadioCore::CORE_RFLORA) {
    rxLORAInit_ = rxInit;
    rxLORADone_ = rxDone;
  }
}

void sx1278::setTxCallbacks(RadioCore rc, Callback *txInit,
    Callback *txDone) {
  if (rc == RadioCore::CORE_RFLORA) {
    txLORAInit_ = txInit;
    txLORADone_ = txDone;
  }
}

void sx1278::enableInterrupts(void) {
  irq_->enableInterrupts();
}

void sx1278::disableInterrupts(void) {
  irq_->disableInterrupts();
}

sx1278::RadioResult sx1278::getRSSI(RadioCore rc, int8_t *rssi) {
  uint16_t address;
  int8_t value;


  address = getRFRegisterAddress(rc, RFn_RSSI);


  singleAccessRead(address, (uint8_t*) &value);

 
  if (value == sx1278_RSSI_MAX_VALUE) {
    *rssi = 0;
    return RadioResult::Error;
  }
  else {
    *rssi = value;
    return RadioResult::Success;
  }
}

sx1278::RadioResult sx1278::getED(RadioCore rc, int8_t *ed) {
  uint16_t address;
  int8_t value;

 
  address = getRFRegisterAddress(rc, RFn_EDV);

 
  singleAccessRead(address, (uint8_t*) &value);

 
  if (value < sx1278_ED_MIN_VALUE || value > sx1278_ED_MAX_VALUE) {
    *ed = 0;
    return RadioResult::Error;
  }
  else {
    *ed = value;
    return RadioResult::Success;
  }
}

sx1278::RadioResult sx1278::setTransmitPower(RadioCore rc,
    TransmitPower power) {
  uint16_t address;
  int8_t value;

  
  address = getRFRegisterAddress(rc, RFn_PAC);

  
  singleAccessRead(address, (uint8_t*) &value);

  
  value &= (~sx1278_RFn_PAC_PACUR_MASK);

  
  value &= (~sx1278_RFn_PAC_TXPWR_MASK);

  
  value |= (sx1278_RFn_PAC_PACUR_3dB << sx1278_RFn_PAC_PACUR_SHIFT);

  
  value |= (power & sx1278_RFn_PAC_TXPWR_MASK);

  
  singleAccessWrite(address, value);

  return RadioResult::Success;
}

sx1278::RadioResult sx1278::loadPacket(RadioCore rc, uint8_t *data,
    uint16_t length) {
  uint16_t bbc_txfll;
  uint16_t bbc_fbtxs;
  uint8_t scratch[2];

  
  bbc_txfll = getBBCRegisterAddress(rc, BBCn_TXFLL);
  bbc_fbtxs = getFBRegisterAddress(rc, BBCn_FBTXS);

  
  length += crc_length;

  
  scratch[0] = (uint8_t) ((length >> 0) & 0xFF); /* low byte */
  scratch[1] = (uint8_t) ((length >> 8) & 0x07); /* high byte */

  
  blockAccessWrite(bbc_txfll, scratch, 2);

  
  blockAccessWrite(bbc_fbtxs, data, length);

  return RadioResult::Success;
}

sx1278::RadioResult sx1278::getPacket(RadioCore rc, uint8_t *buffer,
    uint16_t *length, int8_t *rssi, int8_t *lqi, bool *crc) {
  uint16_t bbc_rxfll, bbc_pc;
  uint16_t rf_rssi, rf_edv;
  uint16_t bbc_fbrxs;
  uint16_t scratch;
  uint8_t scratch_buffer[2];
  uint8_t byte;

  
  bbc_rxfll = getBBCRegisterAddress(rc, BBCn_RXFLL);
  bbc_pc = getBBCRegisterAddress(rc, BBCn_PC);
  rf_rssi = getRFRegisterAddress(rc, RFn_RSSI);
  rf_edv = getRFRegisterAddress(rc, RFn_EDV);
  bbc_fbrxs = getFBRegisterAddress(rc, BBCn_FBRXS);

  
  blockAccessRead(bbc_rxfll, scratch_buffer, 2);

  
  scratch = (scratch_buffer[0] << 0);
  scratch |= (scratch_buffer[1] << 8);

  
  scratch -= crc_length;

  
  if (scratch > *length) {
    return RadioResult::Error;
  }
  else {

    *length = scratch;
  }


  blockAccessRead(bbc_fbrxs, buffer, scratch);


  singleAccessRead(rf_rssi, &byte);
  *rssi = byte;

  singleAccessRead(rf_edv, &byte);
  *lqi = byte;


  singleAccessRead(bbc_pc, &byte);
  *crc = (bool) ((byte & sx1278_BBCn_PC_FCSOK_MASK)
      == sx1278_BBCn_PC_FCSOK_MASK);

  return RadioResult::Success;
}

void sx1278::setContinuousTransmission(RadioCore rc, bool enable) {
  uint16_t bbc_pc;
  uint8_t value;


  bbc_pc = getBBCRegisterAddress(rc, BBCn_PC);


  singleAccessRead(bbc_pc, &value);


  if (enable) {
    value |= sx1278_BBCn_PC_CTX_MASK;
  }
  else {
    value ^= sx1278_BBCn_PC_CTX_MASK;
  }


  singleAccessWrite(bbc_pc, value);
}

/*=============================== protected =================================*/

void sx1278::interruptHandler(void) {

  uint8_t rfLORA_irqs;
  uint8_t bbc0_irqs;


  singleAccessRead(RFLORA_IRQS, &rfLORA_irqs);
  singleAccessRead(BBC0_IRQS, &bbc0_irqs);


  if ((rfLORA_irqs & rfLORA_irqm) || (bbc0_irqs & bbc0_irqm)) {
    interruptHandler_rfLORA(rfLORA_irqs, bbc0_irqs);
  }

}

void sx1278::interruptHandler_rfLORA(uint8_t rf_irqs, uint8_t bbc_irqs) {

  if ((bbc_irqs & sx1278_BBCn_IRQS_RXFS_MASK) && (rxLORAInit_ != nullptr)) {
    rxLORAInit_->execute();
  }


  if ((bbc_irqs & sx1278_BBCn_IRQS_RXFE_MASK) && (rxLORADone_ != nullptr)) {
    rxLORADone_->execute();
  }

  if (txLORAInit_ != nullptr) {

  }

  
  if ((bbc_irqs & sx1278_BBCn_IRQS_TXFE_MASK) && (txLORADone_ != nullptr)) {
    txLORADone_->execute();
  }
}


/*================================ private ==================================*/

sx1278::RadioState sx1278::getState(RadioCore rc) {
  RadioState state;
  uint16_t address;

  
  address = getRFRegisterAddress(rc, RFn_STATE);

  
  singleAccessRead(address, (uint8_t*) &state);


  return state;
}

void sx1278::goToState(RadioCore rc, RadioCommand cmd, RadioState target) {
  RadioState current;

 
  writeCmd(rc, cmd);

  
  do {

    current = getState(rc);
    if (target != current) {
      board.delayMicroseconds(sx1278_DELAY_US);
    }
  }
  while (target != current);
}

void sx1278::writeCmd(RadioCore rc, RadioCommand cmd) {
  uint16_t address;

  
  address = getRFRegisterAddress(rc, RFn_CMD);

  
  singleAccessWrite(address, (uint8_t) cmd);
}

void sx1278::singleAccessRead(uint16_t address, uint8_t *value) {
  uint8_t spi_tx_transaction[3];
  uint8_t spi_rx_transaction[3];
  uint8_t address_hi, address_lo;

  
  address_hi = (uint8_t) ((address & 0xFF00) >> 8);
  address_lo = (uint8_t) ((address & 0x00FF) >> 0);

  
  spi_tx_transaction[0] = READ_CMD | address_hi;
  spi_tx_transaction[1] = address_lo;
  spi_tx_transaction[2] = 0x00;

  
  cs_->low();

  
  spi_->rwByte(spi_tx_transaction, 3, spi_rx_transaction, 3, dma_);

  
  cs_->high();

  
  *value = spi_rx_transaction[2];
}

void sx1278::singleAccessWrite(uint16_t address, uint8_t value) {
  uint8_t spi_tx_transaction[3];
  uint8_t spi_rx_transaction[3];
  uint8_t address_hi, address_lo;

  
  address_hi = (uint8_t) ((address & 0xFF00) >> 8);
  address_lo = (uint8_t) ((address & 0x00FF) >> 0);

  
  spi_tx_transaction[0] = WRITE_CMD | address_hi;
  spi_tx_transaction[1] = address_lo;
  spi_tx_transaction[2] = value;

  
  cs_->low();

  
  spi_->rwByte(spi_tx_transaction, 3, spi_rx_transaction, 3, dma_);


  cs_->high();
}

void sx1278::blockAccessRead(uint16_t address, uint8_t *values,
    uint16_t length) {
  uint8_t spi_tx_transaction[2];
  uint8_t spi_rx_transaction[2];
  uint8_t address_hi, address_lo;


  address_hi = (uint8_t) ((address & 0xFF00) >> 8);
  address_lo = (uint8_t) ((address & 0x00FF) >> 0);


  spi_tx_transaction[0] = READ_CMD | address_hi;
  spi_tx_transaction[1] = address_lo;


  cs_->low();


  spi_->rwByte(spi_tx_transaction, 2, spi_rx_transaction, 2, dma_);


  spi_->rwByte(NULL, 0, values, length, dma_);


  cs_->high();
}

void sx1278::blockAccessWrite(uint16_t address, uint8_t *values,
    uint16_t length) {
  uint8_t spi_tx_transaction[2];
  uint8_t spi_rx_transaction[2];
  uint8_t address_hi, address_lo;


  address_hi = (uint8_t) ((address & 0xFF00) >> 8);
  address_lo = (uint8_t) ((address & 0x00FF) >> 0);


  spi_tx_transaction[0] = WRITE_CMD | address_hi;
  spi_tx_transaction[1] = address_lo;


  cs_->low();


  spi_->rwByte(spi_tx_transaction, 2, spi_rx_transaction, 2, dma_);


  spi_->rwByte(values, length, NULL, 0, dma_);


  cs_->high();
}

inline uint16_t sx1278::getCORERegisterAddress(RadioCore rc) {
  uint16_t address;


  switch (rc) {
  case RadioCore::CORE_RFLORA:
    address = RFLORA_IRQS;
    break;
  default:
    address = 0;
    break;
  }

  return address;
}

inline uint16_t sx1278::getRFRegisterAddress(RadioCore rc,
    uint16_t address) {

  switch (rc) {
  case RadioCore::CORE_RFLORA:
    address = RFLORA_BASE + address;
    break;
  default:
    address = 0;
    break;
  }

  return address;
}

inline uint16_t sx1278::getBBCRegisterAddress(RadioCore rc,
    uint16_t address) {

  switch (rc) {
  case RadioCore::CORE_RFLORA:
    address = BBC0_BASE + address;
    break;
  default:
    return 0;
    break;
  }

  return address;
}

inline uint16_t sx1278::getFBRegisterAddress(RadioCore rc,
    uint16_t address) {

  switch (rc) {
  case RadioCore::CORE_RFLORA:
    address = BBC0_FB_BASE + address;
    break;
  default:
    return 0;
    break;
  }

  return address;
}

inline uint16_t sx1278::getCRCLength(RadioCore rc) {
  uint16_t address;
  uint8_t value;


  address = getBBCRegisterAddress(rc, BBCn_PC);


  singleAccessRead(address, &value);


  if ((value & sx1278_BBCn_PC_FCST_MASK) == sx1278_BBCn_PC_FCST_MASK) {
    return sx1278_BBCn_PC_FCST_16_BIT;
  }
  else {
    return sx1278_BBCn_PC_FCST_32_BIT;
  }
}

inline void sx1278::enableBitFromRegister(uint16_t address, uint32_t mask) {
  uint8_t scratch;
  singleAccessRead(address, &scratch);
  scratch |= mask;
  singleAccessWrite(address, scratch);
}

inline void sx1278::disableBitFromRegister(uint16_t address, uint32_t mask) {
  uint8_t scratch;
  singleAccessRead(address, &scratch);
  scratch &= ~mask;
  singleAccessWrite(address, scratch);
}