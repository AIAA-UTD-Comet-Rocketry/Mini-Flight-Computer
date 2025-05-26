/*
 * Functions to handle interfacing with the external EEPROM.
 * All the operations are collected together although not all sections
 * of the memory can be accessed at any time. It is up to the
 * programmer to avoid hazards with calibration values and
 * file pointers
 */
#include "memops_ex.h"
#include "m95p32.h"

M95_Object_t     hm95p32;
extern SPI_HandleTypeDef hspi1;
uint32_t mem_InitBlock[128] __attribute__((aligned (4))); //512 bytes
uint32_t gNumOpenFd;
volatile uint32_t currTarAddr, currDbgAddr;


/*
 * Adds the the m95p32 chip to the spi 1 bus.
 * Also loads in available blob pointers as form
 * of file system.
 */
void M95p32_Init(void)
{
 /*
  * Memory Architecture:
  * - 4 194 304 bytes
  * - 512 bytes per page
  * - 8 pages per sector
  * - 16 sectors per block
  * - 64 blocks total
  *
  * Erase operations must stay within a block, sector, or page
  * Write operations must stay within the page, a new call is needed to span a boundary
  * Reads are unlimited
  *
  * page 0 (0x000 - 0x1FF) stores init data and file locations
  * pages 1 and up contain 1 stagedMem array per page, with file boundaries defined in page 0
  *
  */

  hm95p32.IO.Init = (M95_Init_Func)BSP_SPI1_Init;
  hm95p32.IO.DeInit = (M95_DeInit_Func)BSP_SPI1_DeInit;
  hm95p32.IO.Delay = (M95_Delay)HAL_Delay;
  hm95p32.IO.Read = BSP_SPI1_Recv;
  hm95p32.IO.Write = BSP_SPI1_Send;

  memset(mem_InitBlock, 0, 512U);
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_RESET);
  Single_Read(&hm95p32, (uint8_t*)mem_InitBlock, 0x000000, 512U); //get page with init data
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_SET);

  if(mem_InitBlock[0] != 0xa5a5a5a5)
  {
    //the eeprom has not been formatted
    M95p32_Reformat();
  }

  mem_InitBlock[1] += 1; //start another file
  currTarAddr = mem_InitBlock[mem_InitBlock[1]]; //start where previous file ends
  if(currTarAddr == 1) //new file system
  {
    currTarAddr = RUN_DATA_ADDR;
  }
  //we do not save in new file count
  //error saving at end of flight results in potential loss of a file
  //if saved now instead, errors would remove formatting page, losing all files
  currDbgAddr = DEBUG_PAGE_ADDR;
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_RESET);
  Page_Erase(&hm95p32, DEBUG_PAGE_ADDR);
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_SET);
}


/*
 * Reformat EEPROM by removing data references
 * Sets blob pointers to zero
 * Sets file count to zero
 */
void M95p32_Reformat(void)
{
 /*
  * 512 page 0 format:
  * *-------*--------------*
  * | bytes | description  |
  * *-------*--------------*
  * |  0-3  | format ID    |
  * *-------*--------------*
  * |  4-7  |# Files(0-126)|
  * *-------*--------------*
  * |   8   | File 2 start |
  * |   *   |      *       |
  * |   *   |      *       |
  * |   *   |      *       |
  * |  508  |File 127 start|
  * *-------*--------------*
  *
  *	file count held in bytes 4-7 only needs 1 byte, the MSB 3 are reserved for alignment
  *
  *	file 1 always starts at 0x000200
  *	if there are n files, mem_InitBlock[n+1] holds the address for file n+1
  */

  memset(mem_InitBlock, 0, 512U);
  mem_InitBlock[0] = 0xa5a5a5a5;

  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_RESET);
  WRITE_ENABLE(&hm95p32);
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_SET);
  __NOP();
  __NOP();
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_RESET);
  Page_Write(&hm95p32, (uint8_t*)mem_InitBlock, 0x000000, 512U); //get page with init data
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_SET);
}


/*
 * Increment file count and start next file pointer
 * FLush data if needed (not used)
 */
void M95p32_Close(void)
{
  //saves end of file telling next file where to start
  mem_InitBlock[mem_InitBlock[1] + 1] = currTarAddr;

  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_RESET);
  WRITE_ENABLE(&hm95p32);
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_SET);
  __NOP();
  __NOP();
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_RESET);
  Page_Write(&hm95p32, (uint8_t*)mem_InitBlock, 0x000000, PAGE_SIZE);
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_SET);
}


/*
 * Printf is not available during flight, post-flight failure
 * analysis must use non-volatile memory.
 * 512 bytes are available in this debug page
 */
void M95p32_DebugPrint(uint8_t* data, uint32_t size)
{
  if(currDbgAddr + size >= RUN_DATA_ADDR) { return; } // page overrun overflow

  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_RESET);
  WRITE_ENABLE(&hm95p32);
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_SET);
  __NOP();
  __NOP();
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_RESET);
  Page_Prog(&hm95p32, data, currDbgAddr, size);
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_SET);
  currDbgAddr += size;
}


void M95p32_DebugSectorDump(uint8_t* sector, uint32_t sectorNum)
{
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_RESET);
  Single_Read(&hm95p32, sector, sectorNum * SECTOR_SIZE, SECTOR_SIZE); //get page with init data
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_SET);
}


/*
 * Saves a page of data to open file
 * data: pointer to data buffer
 * size: number of bytes in buffer to save
 * Return: status, 0 on success, 1 on failure
 */
uint32_t M95p32_SavePage(uint8_t* data, uint32_t size)
{
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_RESET);
  WRITE_ENABLE(&hm95p32);
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_SET);

  if(size > PAGE_SIZE) // Page writes must stay within the page size
    size = PAGE_SIZE;

  //this takes 0.5ms at minimum, can caused missed sensor cycle each 700msec
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_RESET);
  Page_Write(&hm95p32, data, currTarAddr, size);
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_SET);
  currTarAddr += 0x200; //move address to next page

  return currTarAddr >= 0x400000; // only show fail on memory overflow
}


/*
 * These functions need to be implemented but should not be called; the accelerometer calibration library decides
 * when to call these functions. They may be implemented as empty (always return 0) if saving and loading
 * calibration coefficients is not needed.
 * Currently these functions are implemented as NVM storage is used to store the acceleration calibration value
 * even when the chip is powered OFF.
 * More info: https://www.st.com/resource/en/user_manual/dm00373531-getting-started-with-motionac-accelerometer-calibration-library-in-xcubemems1-expansion-for-stm32cube-stmicroelectronics.pdf
 */
uint8_t MotionAC_SaveCalInNVM(uint16_t dataSize, uint32_t *data)
{
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_RESET);
  WRITE_ENABLE(&hm95p32);
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_SET);

  // Prepend magic number and version
  uint32_t buffer[dataSize/sizeof(uint32_t) + 2];
  buffer[0] = MOTIONAC_CAL_MAGIC;
  buffer[1] = 0x01; // Data format version
  memcpy(&buffer[2], data, dataSize);

  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_RESET);
  Page_Write(&hm95p32, (uint8_t*)buffer, MOTIONAC_CAL_ADDR, sizeof(buffer));
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_SET);

  //printf("Acc coef in NVM: %d\n", data);

  return 0;
}


uint8_t MotionAC_LoadCalFromNVM(uint16_t dataSize, uint32_t *data)
{
  uint32_t header[2];

  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_RESET);
  Single_Read(&hm95p32, (uint8_t*)header, MOTIONAC_CAL_ADDR, sizeof(header));
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_SET);

  if(header[0] != MOTIONAC_CAL_MAGIC || header[1] != 0x01) {
    return 1; // Invalid calibration data
  }

  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_RESET);
  Single_Read(&hm95p32, (uint8_t*)data, MOTIONAC_CAL_ADDR + 8, dataSize);
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_SET);

  return 0;
}
