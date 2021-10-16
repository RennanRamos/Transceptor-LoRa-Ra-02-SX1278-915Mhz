/*================================ include ==================================*/
#include "drivers/sx10278.hpp"
#include "drivers/sx10278_conf.h"

#include "host/hardware_timer/include/hardware/timer.h"
#include "rp2_common/hardware_spi/include/hardware/spi.h"
#include "src/rp2040/hardware_structs/include/hardware/structs/spi.h"

#include <string.h>
/*================================ define ===================================*/

#define HEARTBEAT_TASK_PRIORITY (tskIDLE_PRIORITY + 0)
#define TRANSMIT_TASK_PRIORITY (tskIDLE_PRIORITY + 1)

#define HEARTBEAT_TASK_STACK_SIZE (128)
#define TRANSMIT_TASK_STACK_SIZE (1024)

#define SPI_BAUDRATE (8000000)

#define TX_BUFFER_LENGTH (127)
#define EUI48_ADDDRESS_LENGTH (6)

#define RADIO_CORE (sx1278_::CORE_RF09)

#define OFDM_SETTINGS (&radio_settings[CONFIG_OFDM3_MCS1])           /* X BPSK,   rate 1/2, 2x repetition,   50 kbps */
#define OFDM_FREQUENCY (&frequency_settings_09[FREQUENCY_09_OFDM2])  /* OFDM Mode 2,  800 kHz */
#define FSK_SETTINGS (&radio_settings[CONFIG_FSK_OPTION1])           /* X 2-FSK,  50 kbps */
#define FSK_FREQUENCY (&frequency_settings_09[FREQUENCY_09_FSK1])    /* FSK Mode 1,   200 kHz */
#define OQPSK_SETTINGS (&radio_settings[CONFIG_OQPSK_RATE4])         /* X OQPSK-DSSS,  100 kchips/s,  50.00 kbps */
#define OQPSK_FREQUENCY (&frequency_settings_09[FREQUENCY_09_OQPSK]) /* OQPSK,        600 kHz */

#define RADIO_CHANNEL (0)
#define RADIO_TX_POWER (sx1278_::TransmitPower::TX_POWER_MAX)

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/
spi_inst_t spi_SX1278;
sx1278 sx1278_;
/*=============================== variables =================================*/

/*================================= public ==================================*/

int main(void)
{
  /* Enable the sx1278 interface */
  sx1278_.enable(&spi_SX1278, &sx1278__irq);
  uint64_t packet_counter = 0;
  uint8_t tx_mode = 0;
  uint8_t cycle = 0;
  int8_t cca_threshold = 0;
  uint8_t csma_retries = 0;
  int8_t csma_rssi = 0;
  bool csma_check = false;

  /* Forever */
  while (true)
  {
    uint16_t tx_buffer_len;

    bool sent;

    for (cycle = 0; cycle < 3; cycle++)
    {

      if (cycle == 1)
      {
        busy_wait_us_32(10000)
      }
      if (cycle == 2)
      {
        busy_wait_us_32(10000);
      }

      for (tx_mode = 0; tx_mode < 3; tx_mode++)
      {
        /* Turn sx1278_ radio off */
        sx1278_.on();

        /* Wake up and configure radio */
        sx1278_.wakeup(RADIO_CORE);

        // Run through 3 pre configured radio settings
        switch (tx_mode)
        {
        case 0:
          // Configure FSK Radio
          sx1278_.configure(RADIO_CORE, FSK_SETTINGS, FSK_FREQUENCY, RADIO_CHANNEL);
          cca_threshold = -94;

          break;
        case 1:
          // Rádio OQPSK
          sx1278_.configure(RADIO_CORE, OQPSK_SETTINGS, OQPSK_FREQUENCY, RADIO_CHANNEL);
          cca_threshold = -93;

          break;
        case 2:
          // Configure OFDM Radio
          sx1278_.configure(RADIO_CORE, OFDM_SETTINGS, OFDM_FREQUENCY, RADIO_CHANNEL);
          cca_threshold = -91;

          break;
        default:
          sx1278_.configure(RADIO_CORE, OFDM_SETTINGS, OFDM_FREQUENCY, RADIO_CHANNEL);
          break;
        }

        /* Set Tx Power to the maximum */
        sx1278_.setTransmitPower(RADIO_CORE, RADIO_TX_POWER);

        // Check if channel is busy
        csma_check = sx1278_.csma(RADIO_CORE, cca_threshold, &csma_retries, &csma_rssi);

        /* Prepare radio packet */
        tx_buffer_len = prepare_packet(radio_buffer, eui48_address, packet_counter, tx_mode, cycle, csma_retries, csma_rssi);

        /* Load packet to radio */
        sx1278_.loadPacket(RADIO_CORE, radio_buffer, tx_buffer_len);

        /* Transmit packet if the channel is free */
        if (csma_check)
        {
          sx1278_.transmit(RADIO_CORE);
        }
        busy_wait_us_32(1500);

        /* Turn sx1278_ radio off */
        sx1278_.off();
      }
    }

    /* Increment packet counter */
    packet_counter++;

    // Delay
    busy_wait_us_32(10000);
  }
  return 0;
}
/*================================ private ==================================*/


// INSERT RANDOM DATA TO THE PACKET!
static uint16_t prepare_packet(uint8_t *packet_ptr, uint8_t *eui48_address, uint64_t packet_counter, uint8_t tx_mode, uint8_t tx_counter, uint8_t csma_retries, int8_t csma_rssi)
{
 uint16_t packet_length = 0;

 /* Copy MAC address */
 for (packet_length = 0; packet_length < EUI48_ADDDRESS_LENGTH; packet_length++)
 {
   packet_ptr[packet_length] = 'x';
 }

 /* Copy packet counter */
 packet_ptr[packet_length++] = (uint8_t)((packet_counter & 0xFF00000000000000) >> 56);
 packet_ptr[packet_length++] = (uint8_t)((packet_counter & 0x00FF000000000000) >> 48);
 packet_ptr[packet_length++] = (uint8_t)((packet_counter & 0x0000FF0000000000) >> 40);
 packet_ptr[packet_length++] = (uint8_t)((packet_counter & 0x000000FF00000000) >> 32);
 packet_ptr[packet_length++] = (uint8_t)((packet_counter & 0x00000000FF000000) >> 24);
 packet_ptr[packet_length++] = (uint8_t)((packet_counter & 0x0000000000FF0000) >> 16);
 packet_ptr[packet_length++] = (uint8_t)((packet_counter & 0x000000000000FF00) >> 8);
 packet_ptr[packet_length++] = (uint8_t)((packet_counter & 0x00000000000000FF) >> 0);

 // Tx info
 packet_ptr[packet_length++] = tx_mode;
 packet_ptr[packet_length++] = tx_counter;

 // CSMA info
 packet_ptr[packet_length++] = csma_retries;
 packet_ptr[packet_length++] = csma_rssi;

 // Fill 32 bytes
 packet_ptr[packet_length++] = 0; // 19;
 packet_ptr[packet_length++] = 0; // 20;
 packet_ptr[packet_length++] = 0; // 21;
 packet_ptr[packet_length++] = 0; // 22;
 packet_ptr[packet_length++] = 0; // 23;
 packet_ptr[packet_length++] = 0; // 24;
 packet_ptr[packet_length++] = 0; // 25;
 packet_ptr[packet_length++] = 0; // 26;
 packet_ptr[packet_length++] = 0; // 27;
 packet_ptr[packet_length++] = 0; // 28;
 packet_ptr[packet_length++] = 0; // 29;
 packet_ptr[packet_length++] = 0; // 30;
 packet_ptr[packet_length++] = 0; // 31;
 packet_ptr[packet_length++] = 0; // 32;

 return packet_length;
}
