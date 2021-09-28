
/*================================ include ==================================*/

#include "BoardImplementation.hpp"

#include "Gpio.hpp"

#include "sx1278.hpp"
#include "sx1278_conf.h"

/*================================ define ===================================*/
#define UART_BAUDRATE						( 115200 )
#define SPI_BAUDRATE                        ( 16000000 )

#define RADIO_BUFFER_LENGTH                 ( 1024 )
#define SERIAL_BUFFER_LENGTH                ( 1024 )

#define RADIO_CORE                          ( sx1278::CORE_RF09 )
#define RADIO_SETTINGS                      ( &radio_settings[CONFIG_OFDM3_MCS1] )
#define RADIO_FREQUENCY                     ( &frequency_settings_LORA[FREQUENCY_LORA_OFDM2] )
#define RADIO_CHANNEL                       ( 0 )
#define RADIO_TX_POWER                      ( sx1278::TransmitPower::TX_POWER_MIN )

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

/*=============================== variables =================================*/

static uint8_t radio_buffer[RADIO_BUFFER_LENGTH];
static uint16_t radio_buffer_len = sizeof(radio_buffer);

/*================================= public ==================================*/

sx1278 sx1278;

int main(void)
{
  //sx1278.enable(&spi, &sx1278_pwr, &sx1278_rst, &sx1278_cs, &sx1278_irq);
  /* Forever */
 // while (true)
  //{
    //bool status;

    /* Turn sx1278 radio on */
    //sx1278.on();

    /* Wake up and configure radio */
    //sx1278.wakeup(RADIO_CORE);
    //sx1278.configure(RADIO_CORE, RADIO_SETTINGS, RADIO_FREQUENCY, RADIO_CHANNEL);
    //sx1278.setTransmitPower(RADIO_CORE, RADIO_TX_POWER);
	    /* Check sx1278 radio */
    //status = sx1278.check();
  //  if (!status)
  //  {
	  /* Blink red LED */
      //board.error();
//	}
    //sx1278::RadioResult result;
    //int8_t rssi, lqi;
    //bool crc;
   // bool received;

    /* Initialize packet pointer and length */
    //uint8_t* packet_ptr = radio_buffer;
    //uint16_t packet_len = radio_buffer_len;

    /* Try to receive a packet */

    //sx1278.receive(RADIO_CORE);
    //board.delayMilliseconds(100);

    /* If we have received a packet */

      /* Load packet to radio */
      //result = sx1278.getPacket(RADIO_CORE, packet_ptr, &packet_len, &rssi, &lqi, &crc);

      /* Check packet has been received successfully */
     // if (result == sx1278::RadioResult::Success && crc == true)
     // {
//        uint16_t length;
       // uart.write(radio_buffer, 13);
        /* Turn on yellow LED */
       // led_blue.on();
    //  }
     // led_blue.off();
      /* Turn sx1278 radio off */
//      sx1278.off();
    //  board.delayMilliseconds(900);
 // }


  return 0;
}

/*================================ private ==================================*/
