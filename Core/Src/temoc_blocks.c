/*
 * temoc_blocks.c
 *
 * Compile options for board bring-up code and software assisted debugging
 * NOT FOR LAUNCH!
 */

#include "main.h"
#include "custom_bus.h"
#include "lps22hh.h"
#include "lsm6dsr.h"
#include "m95p32.h"

extern __IO uint32_t uwTick;
extern SPI_HandleTypeDef hspi1;
extern LPS22HH_Object_t hlps22;
extern LSM6DSR_Object_t hlsm6d;
extern M95_Object_t     hm95p32;
extern volatile uint32_t currTarAddr;
extern uint16_t PRESS_PIN;
extern uint16_t IMU_PIN;

void temocTests(void)
{
#if 0 // mem dump
  currTarAddr = 0x000000;
  uint8_t phw_ID = 0;
  uint8_t xhw_ID = 0;
  while (1) {
	  debugSector = 0;

	  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_RESET);
	  Single_Read(&hm95p32, sector, currTarAddr, 4096U); //get page with init data
	  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_SET);

	  while(!debugSector){};
	  currTarAddr += 4096U;
  }

#endif

#if 0 // sensor tests
  uint8_t phw_ID = 0;
  uint8_t xhw_ID = 0;
  while (1) {
	  // pressure test
	  HAL_Delay(100);
	  phw_ID = 0;
	  LPS22HH_ReadID(&hlps22, &phw_ID);

  	  // xcel test
	  HAL_Delay(100);
	  xhw_ID = 0;
	  LSM6DSR_ReadID(&hlsm6d, &xhw_ID);
  }

#endif

#if 0 // pressure sensor reading test (checks for MEMS damage)
  uint8_t tx_dump[6];
  uint8_t rx_dump[6];
  memset(tx_dump, 0, 6);
  tx_dump[0] = 0xa8;
  while (1) {
	  HAL_GPIO_WritePin(GPIOC, PRESS_PIN, GPIO_PIN_RESET);
	  BSP_SPI1_SendRecv(tx_dump, rx_dump, 6);
	  HAL_GPIO_WritePin(GPIOC, PRESS_PIN, GPIO_PIN_SET);
	  HAL_Delay(1000);
  }
#endif

#if 0
  extern void M95p32_Reformat();
  M95p32_Reformat(); //clear memory
  while(1){}
#endif
}
