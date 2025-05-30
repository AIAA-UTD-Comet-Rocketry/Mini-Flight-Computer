/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "custom_bus.h"
#include "lps22hh.h"
#include "lsm6dsr.h"
#include "m95p32.h"
#include "FlightState.h"
#include "motion_gc.h"
#include "motion_ac.h"
#include "motion_fx.h"
#include "stdlib.h" // For malloc/free
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_FREQ 	    104.0f //100Hz
#define MOTIONAC_CAL_MAGIC	0xACCA1B1E
#define GBIAS_GYRO_TH_SC    (2.0f*0.002f)
#define GBIAS_ACC_TH_SC     (2.0f*0.000765f)
#define GYRO_FILTER_SIZE 	512

#define MOTIONAC_CAL_ADDR  0x000200  // Reserved page after init block
#define DEBUG_PAGE_ADDR    0x000400
#define RUN_DATA_ADDR      0x000600

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint16_t PRESS_PIN = PRESS_CS_Pin;
uint16_t IMU_PIN = IMU_CS_Pin;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

/* USER CODE BEGIN PV */
extern __IO uint32_t uwTick;
extern SPI_HandleTypeDef hspi1;
LPS22HH_Object_t hlps22; //pressure + barometer
LSM6DSR_Object_t hlsm6d; //acc + gyro
M95_Object_t     hm95p32;
FlightState      fs = {0};
volatile uint32_t currTarAddr;
volatile uint32_t currDbgAddr;

float gAltitude;
float gTotalAcc;
float gDegOffVert;

const float SEALEVELPRESSURE_HPA = 1013.25f; //Change it according to location

/* MotionGC/AC variables */
unsigned int cal_data[MAC_ACC_CAL_DATA_SIZE / sizeof(unsigned int)];
MGC_mcu_type_t mcu_type = MGC_MCU_STM32;
MGC_knobs_t gc_knobs;
MGC_output_t gc_data_out;
MAC_knobs_t ac_knobs;
MAC_output_t ac_data_out;
uint8_t calibration_loaded = 0;

/* MotionFX */
MFXState_t *mfx_state; // MotionFX engine state
MFX_input_t motion_input;
MFX_output_t motion_output;

/* Rolling average gyro filtering */
static LSM6DSR_Axes_t gyroBuffer[GYRO_FILTER_SIZE];
static uint32_t gyroIndex = 0;
static uint8_t gyroBufferFilled = 0;
static int64_t sumX = 0, sumY = 0, sumZ = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
static void Lps22_Init(void);
static void Lsm6D_Init(void);
static void M95p32_Init(void);
static void M95p32_Close(void);
void M95p32_Reformat(void);
void M95p32_DebugPrint(const uint8_t*, uint32_t size);

extern void temocTests();

static void MotionGC_Init(void);
static int CalibrateGyroData(void);
static void ApplyGyroCalibration(LSM6DSR_Axes_t *);

static void MotionAC_Init(void);
static uint8_t CalibrateAccData(void);
static void ApplyAccCalibration(LSM6DSR_Axes_t *);

char MotionAC_LoadCalFromNVM(unsigned short int dataSize, unsigned int *data);
char MotionAC_SaveCalInNVM(unsigned short int dataSize, unsigned int *data);

static void MotionFX_Init(void);
static void GetAltitude(float *);
static void ProcessSensorData(LSM6DSR_Axes_t, LSM6DSR_Axes_t);
static void GetGyroRollingAverage(LSM6DSR_Axes_t *);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct
{
	float 			currPress; 	   // hPa
	float			currTemp; 	   // °C
	LSM6DSR_Axes_t	currAcc; 	   // mg
	LSM6DSR_Axes_t	currGyro; 	   // mdps
	uint32_t        currTick;      // ms
	float           roll;          // °
	float           pitch;         // °
	float           yaw;           // °
	float           linAcc[3];     // g (X,Y,Z)
	float           gravity[3];    // g vector
	float			altitude;      // feet
} dataframe_t;

dataframe_t nextFrame __attribute__((aligned (4))); //aligned for faster data transfer
dataframe_t stagedMem[14] __attribute__((aligned (4)));
uint32_t mem_InitBlock[128] __attribute__((aligned (4))); //512 bytes
uint8_t memIndex = 0;
volatile uint8_t debugSector = 0;
uint8_t sector[4096] __attribute__((aligned (4))) = {0};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	memset(&nextFrame,0,sizeof(dataframe_t));
	memset(&stagedMem,0,sizeof(stagedMem));
	memIndex = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  HAL_Delay(10000); //10 sec for save interrupt

  MX_GPIO_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  BSP_SPI1_Init();
  Lps22_Init();
  Lsm6D_Init();
  M95p32_Init();
  initFlightState(&fs);
  MotionGC_Init();
  MotionAC_Init();
  MotionFX_Init();

  uint8_t dataFlag = 0;
  uint8_t memFlag = 0;

  temocTests(); // Compile option test programs

  /*Calibrate Gyroscope*/
  bool calibrated = false;
  uint32_t nextTick = uwTick;
  while(!calibrated)
  {
	  if(uwTick >= nextTick) // 10ms passed
	  {
		  nextTick = uwTick + 10;
		  calibrated = CalibrateGyroData();
	  }
  }

  /*Calibrate Accelerometer*/
  calibrated = false;
  nextTick = uwTick;
  if (MotionAC_LoadCalFromNVM(sizeof(cal_data), cal_data) == 0)
  {
	memcpy(cal_data, &ac_data_out, sizeof(ac_data_out)); // Serialize
	MotionAC_GetCalParams(&ac_data_out);
	calibration_loaded = 1;
	printf("calibration loaded: %d\n", calibration_loaded);

  } else {
	while(!calibrated)
	{
	  if(uwTick >= nextTick) // 10ms passed
	  {
		  nextTick = uwTick + 10;
		  calibrated = CalibrateAccData();
	  }
	}
  }
  /* USER CODE END calibrate */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    dataFlag = 0;
    LPS22HH_PRESS_Get_DRDY_Status(&hlps22, &dataFlag);
    //LSM6DSR_ACC_Get_DRDY_Status(&hlsm6d, &dataFlag);
    if(dataFlag != 0)
    {
		//get sensor data
		LPS22HH_PRESS_GetPressure(&hlps22, &nextFrame.currPress);
		LPS22HH_TEMP_GetTemperature(&hlps22, &nextFrame.currTemp);

		GetAltitude(&nextFrame.altitude);

		LSM6DSR_GYRO_GetAxes(&hlsm6d, &nextFrame.currGyro);
		LSM6DSR_ACC_GetAxes(&hlsm6d, &nextFrame.currAcc);

		//apply calibration bias to raw values:
		ApplyGyroCalibration(&nextFrame.currGyro);
		GetGyroRollingAverage(&nextFrame.currGyro);
		ApplyAccCalibration(&nextFrame.currAcc);

		gAltitude = nextFrame.altitude;
		gTotalAcc = nextFrame.currAcc.y;
		gDegOffVert = nextFrame.pitch;

		ProcessSensorData(nextFrame.currGyro, nextFrame.currAcc);
    }

    //updateState(&fs);


    if(uwTick >= nextTick)
    {
      nextTick = uwTick + 500; //0.05s delay

      //perform a memory store
      nextFrame.currTick = uwTick;
      memcpy(&stagedMem[memIndex], &nextFrame, sizeof(dataframe_t));
      ++memIndex;

      if(memIndex >= 14) //mem block full
      {
        memIndex = 0;
        memFlag = 1;
      }
    }

    if(memFlag != 0)
    {
      memFlag = 0;
      //block is ready to be stored
      HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_RESET);
      WRITE_ENABLE(&hm95p32);
      HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_SET);

      //this takes 0.5ms at minimum, can caused missed sensor cycle each 700msec
      HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_RESET);
      Page_Write(&hm95p32, (uint8_t*)stagedMem, currTarAddr, sizeof(stagedMem));
      HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_SET);
      currTarAddr += 0x200; //move address to next page
      if(currTarAddr >= 0x400000) {
        // max data size reached
        __disable_irq();
        while(1)
        {
        }
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DROGUE1_Pin|DROGUE2_Pin|STATUS_LED_Pin|IMU_CS_Pin
                          |PRESS_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MAIN1_Pin|MAIN2_Pin|EEPROM_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DROGUE1_Pin DROGUE2_Pin STATUS_LED_Pin */
  GPIO_InitStruct.Pin = DROGUE1_Pin|DROGUE2_Pin|STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MAIN1_Pin MAIN2_Pin EEPROM_CS_Pin */
  GPIO_InitStruct.Pin = MAIN1_Pin|MAIN2_Pin|EEPROM_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU_CS_Pin PRESS_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin|PRESS_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USERSENSE_Pin */
  GPIO_InitStruct.Pin = USERSENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USERSENSE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_GPIO_WritePin(GPIOC, IMU_CS_Pin|PRESS_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_SET);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void Lps22_Init(void)
{
  hlps22.IO.Init = (LPS22HH_Init_Func)BSP_SPI1_Init;
  hlps22.IO.DeInit = (LPS22HH_DeInit_Func)BSP_SPI1_DeInit;
  hlps22.IO.GetTick = (LPS22HH_GetTick_Func)HAL_GetTick;
  hlps22.IO.Delay = (LPS22HH_Delay_Func)HAL_Delay;
  hlps22.IO.BusType = 1U;
  hlps22.Ctx.read_reg = ReadRegWrap;
  hlps22.Ctx.write_reg = WriteRegWrap;
  hlps22.Ctx.mdelay = HAL_Delay;
  hlps22.Ctx.handle = &PRESS_PIN;

  LPS22HH_Init(&hlps22);

  LPS22HH_PRESS_Enable(&hlps22);
  LPS22HH_TEMP_Enable(&hlps22);
  LPS22HH_PRESS_SetOutputDataRate(&hlps22, 100.0f);
  LPS22HH_TEMP_SetOutputDataRate(&hlps22, 100.0f);
}

static void Lsm6D_Init(void)
{
  hlsm6d.IO.Init = (LSM6DSR_Init_Func)BSP_SPI1_Init;
  hlsm6d.IO.DeInit = (LSM6DSR_DeInit_Func)BSP_SPI1_DeInit;
  hlsm6d.IO.GetTick = (LSM6DSR_GetTick_Func)HAL_GetTick;
  hlsm6d.IO.Delay = (LSM6DSR_Delay_Func)HAL_Delay;
  hlsm6d.IO.BusType = 1U;
  hlsm6d.Ctx.read_reg = ReadRegWrap;
  hlsm6d.Ctx.write_reg = WriteRegWrap;
  hlsm6d.Ctx.mdelay = HAL_Delay;
  hlsm6d.Ctx.handle = &IMU_PIN;

  LSM6DSR_Init(&hlsm6d);

  LSM6DSR_ACC_Enable(&hlsm6d);
  LSM6DSR_ACC_SetOutputDataRate(&hlsm6d, 104.0f);
  LSM6DSR_ACC_SetFullScale(&hlsm6d, 16U);

  LSM6DSR_GYRO_Enable(&hlsm6d);
  LSM6DSR_GYRO_SetOutputDataRate(&hlsm6d, 104.0f);
  LSM6DSR_GYRO_SetFullScale(&hlsm6d, 2000U); //changed from 2000 to 250
}

static void M95p32_Init(void)
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

  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_RESET);
  WRITE_ENABLE(&hm95p32);
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_SET);

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
	Page_Write(&hm95p32, (uint8_t*)mem_InitBlock, 0x000000, 512U); //get page with init data
	HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_SET);
}

static void M95p32_Close(void)
{
  //saves end of file telling next file where to start
  mem_InitBlock[mem_InitBlock[1] + 1] = currTarAddr;

  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_RESET);
  WRITE_ENABLE(&hm95p32);
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_SET);
  __NOP();
  __NOP();
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_RESET);
  Page_Write(&hm95p32, (uint8_t*)mem_InitBlock, 0x000000, 512U);
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_SET);
}

void M95p32_DebugPrint(const uint8_t* data, uint32_t size)
{
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

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == USERSENSE_Pin)
  {
    __disable_irq();
    currTarAddr = 0x000000;
    uint8_t phw_ID = 0;
    uint8_t xhw_ID = 0;
    while (1) {
      HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_RESET);
      Single_Read(&hm95p32, sector, currTarAddr, 4096U); //get page with init data
      HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_SET);
    }
  //debugSector = 1;
  }
}


/*Calculate Altitude from Pressure and Temperature Data*/
static void GetAltitude(float *altitude)
{
    const float R = 287.05f;   // Specific gas constant for dry air (J/(kg·K))
    const float g = 9.80665f;  // Gravity (m/s²)
    float T = nextFrame.currTemp;
    float P = nextFrame.currPress;

    // Convert temperature to Kelvin
    T += 273.15f;

    // Calculate altitude using the hypsometric equation
    *altitude = (R * T / g) * log(SEALEVELPRESSURE_HPA / P);

    //meter to feet
    *altitude *= 3.28f;

    nextFrame.altitude = *altitude;

    // For debugging
    static uint32_t alcounter = 0;
    if (alcounter++ % 1000 == 0) {
    	char msg[100];
		int len = sprintf(msg, "Alt: %.2f ft\n", nextFrame.altitude);
		_write(0, msg, len);
    }
}


/* Initialize the MotionGC library */
static void MotionGC_Init(void)
{
	float sample_freq = SAMPLE_FREQ;
	MotionGC_Initialize(mcu_type, &sample_freq); // Initialize with sample frequency
	MotionGC_GetKnobs(&gc_knobs); // Get default knobs

	// Configure for your sensor
	gc_knobs.AccThr = 0.05f;    // 8 mg threshold for stillness detection
	gc_knobs.GyroThr = 0.15f;    // 0.15 dps threshold

	MotionGC_SetKnobs(&gc_knobs);
}


/* Calibrate gyroscope sensor (LSM6DSR) */
static int CalibrateGyroData(void)
{
  MGC_input_t data_in;
  LSM6DSR_Axes_t gyro_data;
  LSM6DSR_Axes_t acc_data;
  int bias_updated;

  LSM6DSR_ACC_GetAxes(&hlsm6d, &acc_data);
  LSM6DSR_GYRO_GetAxes(&hlsm6d, &gyro_data); // Get angular rate from LSM6DSR

  // data cast and copy - mdps -> dps & mg -> g
  data_in.Acc[0]  = ((float)acc_data.x)/1000.0f;
  data_in.Acc[1]  = ((float)acc_data.y)/1000.0f;
  data_in.Acc[2]  = ((float)acc_data.z)/1000.0f;
  data_in.Gyro[0] = ((float)gyro_data.x)/1000.0f;
  data_in.Gyro[1] = ((float)gyro_data.y)/1000.0f;
  data_in.Gyro[2] = ((float)gyro_data.z)/1000.0f;

  // Calculate the compensated gyroscope data
  MotionGC_Update(&data_in, &gc_data_out, &bias_updated);

  if (bias_updated) {
	  printf("Gyro calibration done!\n");
	  printf("Gyro offset:- X: %d Y: %d Z: %d\n", (int)(gc_data_out.GyroBiasX*1000), (int)(gc_data_out.GyroBiasY*1000), (int)(gc_data_out.GyroBiasZ*1000));
  }

  return bias_updated;
}


/* Main loop for filtering  */
static void ApplyGyroCalibration(LSM6DSR_Axes_t *currGyro)
{
	static uint32_t gccounter = 0;
	uint32_t len;
	char msg[100] = {};

	if(!(gccounter++%1000)){
		//printf("Gyro before Cal mdps:- X: %d Y: %d Z: %d\n", (int)currGyro->x, (int)currGyro->y, (int)currGyro->z);
		//printf("Gyro before Cal mdps:- X: Y: Z: \n");
		len = sprintf(msg, "Gyro B mdps:- X: %d Y: %d Z: %d\n", (int)currGyro->x, (int)currGyro->y, (int)currGyro->z);
		_write(0, msg, len);
	}

	currGyro->x = currGyro->x - (int32_t)(gc_data_out.GyroBiasX*1000);
	currGyro->y = currGyro->y - (int32_t)(gc_data_out.GyroBiasY*1000);
	currGyro->z = currGyro->z - (int32_t)(gc_data_out.GyroBiasZ*1000);

	//float gbias[3];
	//MotionFX_getGbias(mfx_state, (float *) &currGyro);

	if(!(gccounter%1000)) {
		//printf("Gyro after Cal mdps:- X: %d Y: %d Z: %d\n\n", (int)currGyro->x, (int)currGyro->y, (int)currGyro->z);
		//printf("Gyro after Cal mdps:- X: Y: Z:\n\n");
		len = sprintf(msg, "Gyro A mdps:- X: %d Y: %d Z: %d\n", (int)currGyro->x, (int)currGyro->y, (int)currGyro->z);
		_write(0, msg, len);
	}
}


static void GetGyroRollingAverage(LSM6DSR_Axes_t *avgGyro)
{
	uint32_t count = 0;
	uint32_t len = 0;
	uint32_t gccounter = 0;
	char msg[100] = {};

	// Remove oldest value from sum
	sumX -= gyroBuffer[gyroIndex].x;
	sumY -= gyroBuffer[gyroIndex].y;
	sumZ -= gyroBuffer[gyroIndex].z;

	// Store new calibrated value in buffer
	gyroBuffer[gyroIndex].x = avgGyro->x;
	gyroBuffer[gyroIndex].y = avgGyro->y;
	gyroBuffer[gyroIndex].z = avgGyro->z;

	// Add new value to sum
	sumX += avgGyro->x;
	sumY += avgGyro->y;
	sumZ += avgGyro->z;

	// Advance buffer index
	gyroIndex = (gyroIndex + 1) % GYRO_FILTER_SIZE;

	if (gyroIndex == 0) {
	    gyroBufferFilled = 1;
	}

	if (gyroBufferFilled) { //Buffer is full
		count = GYRO_FILTER_SIZE;
	} else {
		count = gyroIndex;
	}

	// Compute average
	avgGyro->x = (int32_t)(sumX / count);
	avgGyro->y = (int32_t)(sumY / count);
	avgGyro->z = (int32_t)(sumZ / count);

	if(!(++gccounter % 1000)) {
		len = sprintf(msg, "Gyro Avg mdps:- X: %d Y: %d Z: %d\n", (int)avgGyro->x, (int)avgGyro->y, (int)avgGyro->z);
		_write(0, msg, len);
	}
}


/* Initialize the MotionAC library */
static void MotionAC_Init(void)
{
	MotionAC_Initialize(true); // Initialize with sample frequency
	MotionAC_GetKnobs(&ac_knobs);

	//ac_knobs.Sample_ms = 1;
	// Configure for 6-point calibration
	ac_knobs.Run6PointCal = 1;       // Enable 6-point calibration
	ac_knobs.Sample_ms = 10;         // Match your 100Hz sensor ODR
	ac_knobs.MoveThresh_g = 0.15f;   // Critical for detecting stillness

	MotionAC_SetKnobs(&ac_knobs);
}


/* Calibrate accelerometer sensor (LSM6DSR)*/
static uint8_t CalibrateAccData(void)
{
  MAC_input_t data_in;
  LSM6DSR_Axes_t acc_data;
  LSM6DSR_ACC_GetAxes(&hlsm6d, &acc_data);

  // data cast and copy - mdps -> dps & mg -> g
  data_in.Acc[0]  = ((float)acc_data.x)/1000.0f;
  data_in.Acc[1]  = ((float)acc_data.y)/1000.0f;
  data_in.Acc[2]  = ((float)acc_data.z)/1000.0f;
  data_in.TimeStamp = HAL_GetTick(); // Use direct timestamp

  uint8_t is_calibrated = 0;
  // Calculate the compensated accelerometer data
  MotionAC_Update(&data_in, &is_calibrated);

  MotionAC_GetCalParams(&ac_data_out);

  static uint8_t counter = 0;
  if (counter++ % 100 == 0) {
	  printf("Time: %d ", counter);
	  int status = ac_data_out.CalQuality;
	  switch(status) {
	  	  case 1:
			printf("MAC_CALQSTATUS: POOR\n");
			break;
	  	case 2:
			printf("MAC_CALQSTATUS: OK\n");
			break;
	  	case 3:
			printf("MAC_CALQSTATUS: GOOD!!!!!\n");
			break;
	  	default:
	  		printf("MAC_CALQSTATUS: UNKNOWN!\n");
	  }
	  //printf("Status: %d\n", ac_data_out.CalQuality);
	  printf("Acc Bias:- X: %d Y: %d Z: %d\n", (int)(ac_data_out.AccBias[0]*1000), (int)(ac_data_out.AccBias[1]*1000), (int)(ac_data_out.AccBias[2]*1000));
  }

  if (is_calibrated) {
	  memcpy(cal_data, &ac_data_out, sizeof(ac_data_out)); // Serialize
	  MotionAC_SaveCalInNVM(sizeof(cal_data), cal_data);

	  printf("Acc cal done!!\n");
	  printf("Cal Q: %d | Acc Bias:- X: %d Y: %d Z: %d\n", (int)ac_data_out.CalQuality, (int)(ac_data_out.AccBias[0]*1000), (int)(ac_data_out.AccBias[1]*1000), (int)(ac_data_out.AccBias[2]*1000));
  }

  return is_calibrated;
}


/* Main loop for filtering  */
static void ApplyAccCalibration(LSM6DSR_Axes_t *currAcc)
{
	MAC_output_t data_out;
	uint32_t len;
	char msg[100] = {};

	/* Get Calibration coeficients */
	MotionAC_GetCalParams(&data_out);

	static uint32_t accounter = 0;

	if(!(accounter++%1000)) {
		//printf("Acc after Cal:- X: %d Y: %d Z: %d\n", currAcc->x, currAcc->y, currAcc->z);
		len = sprintf(msg, "Acc B:- X: %d Y: %d Z: %d\n", currAcc->x, currAcc->y, currAcc->z);
		_write(0, msg, len);
	}

	//int-mg        int-mg           fl-g                   fl-g
	currAcc->x = (currAcc->x - (int32_t)(data_out.AccBias[0]*1000)) * data_out.SF_Matrix[0][0];
	currAcc->y = (currAcc->y - (int32_t)(data_out.AccBias[1]*1000)) * data_out.SF_Matrix[1][1];
	currAcc->z = (currAcc->z - (int32_t)(data_out.AccBias[2]*1000)) * data_out.SF_Matrix[2][2];

	if(!(accounter%1000)) {
		//printf("Acc before Cal:- X: %d Y: %d Z: %d\n\n", currAcc->x, currAcc->y, currAcc->z);
		len = sprintf(msg, "Acc A:- X: %d Y: %d Z: %d\n", currAcc->x, currAcc->y, currAcc->z);
		_write(0, msg, len);

		float gravity = sqrt(pow((currAcc->x),2) + pow((currAcc->y),2) + pow((currAcc->z),2));
		len = sprintf(msg, "Gravity: %d mg\n", (int)gravity);
		_write(0, msg, len);
	}
}

static void MotionFX_Init(void)
{
	MFX_knobs_t knobs;

	// Allocate memory for MotionFX state
	size_t state_size = MotionFX_GetStateSize();
	mfx_state = (MFXState_t *)malloc(state_size);
	if(mfx_state == NULL) {
		Error_Handler();
	}

	// Initialize MotionFX with state pointer
	MotionFX_initialize(mfx_state);

	knobs.acc_orientation[0] = 'n';  // East
	knobs.acc_orientation[1] = 'w';  // North
	knobs.acc_orientation[2] = 'u';  // Up
	knobs.gyro_orientation[0] = 'n';
	knobs.gyro_orientation[1] = 'w';
	knobs.gyro_orientation[2] = 'u';

	MotionFX_getKnobs(mfx_state, &knobs);

	// Configure parameters
	knobs.modx = 1; // Update every propagate (Set 1 for STM32F4)
	knobs.LMode = 1;
	knobs.start_automatic_gbias_calculation = 1; // Auto Gyro Bias Calibraton
	knobs.output_type = MFX_ENGINE_OUTPUT_ENU; // Set orientation format
	knobs.gbias_gyro_th_sc = GBIAS_GYRO_TH_SC;
	knobs.gbias_acc_th_sc = GBIAS_ACC_TH_SC;

	MotionFX_setKnobs(mfx_state, &knobs);
	MotionFX_enable_6X(mfx_state, MFX_ENGINE_ENABLE); // Enable 6-axis fusion (gyro + acc)
	MotionFX_enable_9X(mfx_state, MFX_ENGINE_DISABLE);

	// print library version
	char msg[100] = {};
	char version[35] = {};
	MotionFX_GetLibVersion(version);
	uint32_t len = sprintf(msg, "MotionFX version: %s\n", version);
	_write(0, msg, len);
}

static void ProcessSensorData(LSM6DSR_Axes_t currGyro, LSM6DSR_Axes_t currAcc)
{
	float deltaTime = 1.0f / SAMPLE_FREQ; //0.01f or 10ms
	uint32_t len = 0;
	char msg[100] = {};
	MFX_input_t motion_input;
	MFX_output_t motion_output;

	motion_input.gyro[0] = currGyro.x / 1000.0f; // mdps -> dps
	motion_input.gyro[1] = currGyro.y / 1000.0f;
	motion_input.gyro[2] = currGyro.z / 1000.0f;

	motion_input.acc[0] = currAcc.x / 1000.0f; // mg -> g
	motion_input.acc[1] = currAcc.y / 1000.0f;
	motion_input.acc[2] = currAcc.z / 1000.0f;

	motion_input.mag[0] = 0.0f; // Not used in 6X
	motion_input.mag[1] = 0.0f;
	motion_input.mag[2] = 0.0f;

	/* Run Sensor Fusion algorithm */
	//MotionFX_propagate(mfx_state, &motion_output, &motion_input, &deltaTime); //predictive
	MotionFX_update(mfx_state, &motion_output, &motion_input, &deltaTime, NULL); //corrective

	// Store formatted outputs
	nextFrame.roll = motion_output.rotation[0];
	nextFrame.pitch = motion_output.rotation[1];
	nextFrame.yaw = motion_output.rotation[2];
	memcpy(nextFrame.linAcc, motion_output.linear_acceleration, sizeof(float)*3);
	memcpy(nextFrame.gravity, motion_output.gravity, sizeof(float)*3);

	static uint32_t fxcounter = 0;

	if((++fxcounter % 1000 == 0))
	{
		len = sprintf(msg, "Roll: %d deg\n", (int)nextFrame.roll);
		_write(0, msg, len);
		len = sprintf(msg, "Pitch: %d deg\n", (int)nextFrame.pitch);
		_write(0, msg, len);
		len = sprintf(msg, "Yaw: %d deg\n", (int)nextFrame.yaw);
		_write(0, msg, len);
		len = sprintf(msg, "X_Acc: %d mg\n", (int)(nextFrame.linAcc[0]*1000));
		_write(0, msg, len);
		len = sprintf(msg, "Y_Acc: %d mg\n", (int)(nextFrame.linAcc[1]*1000));
		_write(0, msg, len);
		len = sprintf(msg, "Z_Acc: %d mg\n\n", (int)(nextFrame.linAcc[2]*1000));
		_write(0, msg, len);
//		printf("Roll: %d deg\n", (int)nextFrame.roll);
//		printf("Pitch: %d deg\n", (int)nextFrame.pitch);
//		printf("Yaw: %d deg\n", (int)nextFrame.yaw);
//		printf("X_Gravity: %d g\n", nextFrame.gravity[0]);
//		printf("Y_Gravity: %d g\n", nextFrame.gravity[1]);
//		printf("Z_Gravity: %d g\n\n", nextFrame.gravity[2]);

		// Gravity - converted to fixed-point for embedded safety
//		len = sprintf(msg, "X_Gravity: %d mg\n", (int)(nextFrame.gravity[0]*1000));
//		_write(0, msg, len);
//		len = sprintf(msg, "Y_Gravity: %d mg\n", (int)(nextFrame.gravity[1]*1000));
//		_write(0, msg, len);
//		len = sprintf(msg, "Z_Gravity: %d mg\n\n", (int)(nextFrame.gravity[2]*1000));
//		_write(0, msg, len);
	}
}

/**
 *  These functions need to be implemented but should not be called; the accelerometer calibration library decides
	when to call these functions. They may be implemented as empty (always return 0) if saving and loading
	calibration coefficients is not needed.
	Currently these functions are implemented as NVM storage is used to store the acceleration calibration value
	even when the chip is powered OFF.
	More info: https://www.st.com/resource/en/user_manual/dm00373531-getting-started-with-motionac-accelerometer-calibration-library-in-xcubemems1-expansion-for-stm32cube-stmicroelectronics.pdf
 */
char MotionAC_SaveCalInNVM(unsigned short int dataSize, unsigned int *data)
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

char MotionAC_LoadCalFromNVM(unsigned short int dataSize, unsigned int *data)
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

  calibration_loaded = 1;

  return 0;
}


int _write(int le, char *ptr, int len)
{
	int DataIdx;
	for(DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
