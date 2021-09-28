/**
 * @file       platform_prng.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       November, 2018
 * @brief
 *
 * @copyright  Copyright 2018, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include <stdlib.h>

#include "Board.hpp"
#include "RandomNumberGenerator.hpp"

#include "platform_includes.h"
#include "platform_defines.h"

/*================================ define ===================================*/

#define SOC_ADC_ADCCON1_RCTRL0                  ( 0x00000004 )
#define SOC_ADC_ADCCON1_RCTRL1                  ( 0x00000008 )

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

extern BoardImplementation board;

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

NumberGenerator::NumberGenerator()
{
}

void NumberGenerator::init(void)
{
  return this->init();
}

uint16_t NumberGenerator::get(void)
{
  return this->get();
}


DeterministicNumberGenerator::DeterministicNumberGenerator()
{
}

void DeterministicNumberGenerator::init(void)
{
  uint32_t scratch = 0;
  uint8_t eui64[8];
  
//  board.getEUI64(eui64);
  
  scratch |= eui64[4] << 24;
  scratch |= eui64[5] << 16;
  scratch |= eui64[6] <<  8;
  scratch |= eui64[7] <<  0;
  
  srand(scratch);
}

uint16_t DeterministicNumberGenerator::get(void)
{
  uint32_t random;
  
  random = rand();
  
  return random;
}



/*=============================== protected =================================*/

/*================================ private ==================================*/
