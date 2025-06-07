/*
 * External non-volatile memory API for the flight computer.
 * Used for sensor data, calibration, and post-flight debugging
 */
#include "main.h"

#define MOTIONAC_CAL_MAGIC 0xACCA1B1E
#define MOTIONAC_CAL_ADDR  0x000200  // Reserved page after init block
#define DEBUG_PAGE_ADDR    0x000400
#define RUN_DATA_ADDR      0x000600

#define PAGE_SIZE          512U
#define SECTOR_SIZE        4096U

void M95p32_Init(void);
void M95p32_Close(void);

void M95p32_Reformat(void);
void M95p32_DebugPrint(uint8_t*, uint32_t);
void M95p32_DebugSectorDump(uint8_t*, uint32_t);

uint32_t M95p32_SavePage(uint8_t*, uint32_t);

uint8_t MotionAC_LoadCalFromNVM(uint16_t dataSize, uint32_t *data);
uint8_t MotionAC_SaveCalInNVM(uint16_t dataSize, uint32_t *data);
