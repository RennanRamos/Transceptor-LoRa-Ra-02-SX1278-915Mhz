#ifndef SX1278_CONF_HPP_
#define SX1278_CONF_HPP_

#include <stdint.h>

typedef struct
{
  uint16_t address;
  uint8_t value;
} register_t2;

typedef struct
{
  const register_t2* rf_registers;
  uint16_t rf_elements;
  const register_t2* bbc_registers;
  uint16_t bbc_elements;
} radio_settings_t;

typedef struct
{
  uint32_t frequency0;
  uint16_t channel_spacing;
  uint16_t channel_min;
  uint16_t channel_max;
} frequency_settings_t;


enum {
  CONFIG_FSK_OPTION1 = 0, /* 2-FSK,  50 kbps */
  CONFIG_FSK_OPTION2,     /* 2-FSK, 100 kbps */
  CONFIG_FSK_OPTION3,     /* 4-FSK, 200 kbps */
  CONFIG_OQPSK_RATE1,     /* OQPSK-DSSS,  100 kchips/s,   6.25 kbps */
  CONFIG_OQPSK_RATE2,     /* OQPSK-DSSS,  100 kchips/s,  12.50 kbps */
  CONFIG_OQPSK_RATE3,     /* OQPSK-DSSS,  100 kchips/s,  25.00 kbps */
  CONFIG_OQPSK_RATE4,     /* OQPSK-DSSS,  100 kchips/s,  50.00 kbps */
  CONFIG_OQPSK_RATE5,     /* OQPSK-DSSS, 2000 kchips/s, 250 kbps, legacy */
  CONFIG_OFDM1_MCS0,      /* BPSK,   rate 1/2, 4x repetition,  100 kbps */
  CONFIG_OFDM1_MCS1,      /* BPSK,   rate 1/2, 2x repetition,  200 kbps */
  CONFIG_OFDM1_MCS2,      /* QPSK,   rate 1/2, 2x repetition,  400 kbps */
  CONFIG_OFDM1_MCS3,      /* QPSK,   rate 1/2, 0x repetition,  800 kbps */
  CONFIG_OFDM1_MCS4,      /* QPSK,   rate 3/4, 0x repetition, 1200 kbps */
  CONFIG_OFDM1_MCS5,      /* 16-QAM, rate 1/2, 0x repetition, 1600 kbps */
  CONFIG_OFDM1_MCS6,      /* 16-QAM, rate 3/4, 0x repetition, 2400 kbps */
  CONFIG_OFDM2_MCS0,      /* BPSK,   rate 1/2, 4x repetition,   50 kbps */
  CONFIG_OFDM2_MCS1,      /* BPSK,   rate 1/2, 2x repetition,  100 kbps */
  CONFIG_OFDM2_MCS2,      /* QPSK,   rate 1/2, 2x repetition,  200 kbps */
  CONFIG_OFDM2_MCS3,      /* QPSK,   rate 1/2, 0x repetition,  400 kbps */
  CONFIG_OFDM2_MCS4,      /* QPSK,   rate 3/4, 0x repetition,  600 kbps */
  CONFIG_OFDM2_MCS5,      /* 16-QAM, rate 1/2, 0x repetition,  800 kbps */
  CONFIG_OFDM2_MCS6,      /* 16-QAM, rate 3/4, 0x repetition, 1200 kbps */
  CONFIG_OFDM3_MCS1,      /* BPSK,   rate 1/2, 2x repetition,   50 kbps */
  CONFIG_OFDM3_MCS2,      /* QPSK,   rate 1/2, 2x repetition,  100 kbps */
  CONFIG_OFDM3_MCS3,      /* QPSK,   rate 1/2, 0x repetition,  200 kbps */
  CONFIG_OFDM3_MCS4,      /* QPSK,   rate 3/4, 0x repetition,  300 kbps */
  CONFIG_OFDM3_MCS5,      /* 16-QAM, rate 1/2, 0x repetition,  400 kbps */
  CONFIG_OFDM3_MCS6,      /* 16-QAM, rate 3/4, 0x repetition,  600 kbps */
  CONFIG_OFDM4_MCS2,      /* QPSK,   rate 1/2, 2x repetition,   50 kbps */
  CONFIG_OFDM4_MCS3,      /* QPSK,   rate 1/2, 0x repetition,  100 kbps */
  CONFIG_OFDM4_MCS4,      /* QPSK,   rate 3/4, 0x repetition,  150 kbps */
  CONFIG_OFDM4_MCS5,      /* 16-QAM, rate 1/2, 0x repetition,  200 kbps */
  CONFIG_OFDM4_MCS6,      /* 16-QAM, rate 3/4, 0x repetition,  300 kbps */
  CONFIG_ELEMENTS
};

enum {
  FREQUENCY_LORA_FSK1 = 0, /* FSK Mode 1,   200 kHz */
  FREQUENCY_LORA_FSK2,     /* FSK Mode 2,   400 kHz */
  FREQUENCY_LORA_FSK3,     /* FSK Mode 3,   400 kHz */
  FREQUENCY_LORA_OQPSK,    /* OQPSK,        600 kHz */
  FREQUENCY_LORA_OFDM1,    /* OFDM Mode 1, 1200 kHz */
  FREQUENCY_LORA_OFDM2,    /* OFDM Mode 2,  800 kHz */
  FREQUENCY_LORA_OFDM3,    /* OFDM Mode 3,  400 kHz */
  FREQUENCY_LORA_OFDM4,    /* OFDM Mode 4,  200 kHz */
  FREQUENCY_LORA_ELEMENTS
};

extern const radio_settings_t radio_settings[CONFIG_ELEMENTS];
extern const frequency_settings_t frequency_settings_lora[FREQUENCY_LORA_ELEMENTS];

#endif /* SX1278_CONF_HPP_ */
