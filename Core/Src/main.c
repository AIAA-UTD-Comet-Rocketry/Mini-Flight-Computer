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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint16_t PRESS_PIN = PRESS_CS_Pin;
uint16_t IMU_PIN = IMU_CS_Pin;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern __IO uint32_t uwTick;
extern SPI_HandleTypeDef hspi1;
LPS22HH_Object_t hlps22;
LSM6DSR_Object_t hlsm6d;
M95_Object_t     hm95p32;
FlightState      fs = {0};
volatile uint32_t currTarAddr;
volatile uint32_t currDbgAddr;

float gAltitude;
float gTotalAcc;
float gDegOffVert;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static void Lps22_Init(void);
static void Lsm6D_Init(void);
static void M95p32_Init(void);
static void M95p32_Close(void);
void M95p32_Reformat(void);
void M95p32_DebugPrint(const uint8_t*, uint32_t size);


extern void temocTests();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct
{
	float 			currPress;
	float			currTemp;
	LSM6DSR_Axes_t	currAcc;
	LSM6DSR_Axes_t	currGyro;
	uint32_t		currTick;
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
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  BSP_SPI1_Init();
  Lps22_Init();
  Lsm6D_Init();
  M95p32_Init();
  initFlightState(&fs);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t dataFlag = 0;
  uint32_t nextTick = 0;
  uint8_t memFlag = 0;

  temocTests(); // Compile option test programs

  HAL_Delay(10000); //10 sec for save interrupt
  while (1)
  {
    dataFlag = 0;
    LPS22HH_PRESS_Get_DRDY_Status(&hlps22, &dataFlag);
    //LSM6DSR_ACC_Get_DRDY_Status(&hlsm6d, &dataFlag);
    if(dataFlag != 0)
    {
      //get pressure data
      LPS22HH_PRESS_GetPressure(&hlps22, &nextFrame.currPress);
      LPS22HH_TEMP_GetTemperature(&hlps22, &nextFrame.currTemp);
      //get IMU data
      LSM6DSR_ACC_GetAxes(&hlsm6d, &nextFrame.currAcc);
      LSM6DSR_GYRO_GetAxes(&hlsm6d, &nextFrame.currGyro);

      //todo process acc, alt, attitude
      gTotalAcc = ((float)nextFrame.currAcc.z)/1000.0;
    }

    updateState(&fs);

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
      if(currTarAddr >= 0x400000){
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
  LPS22HH_PRESS_SetOutputDataRate(&hlps22, 200.0f);
  LPS22HH_TEMP_SetOutputDataRate(&hlps22, 200.0f);
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
  LSM6DSR_ACC_SetOutputDataRate(&hlsm6d, 208.0f);
  LSM6DSR_ACC_SetFullScale(&hlsm6d, 16U);

  LSM6DSR_GYRO_Enable(&hlsm6d);
  LSM6DSR_GYRO_SetOutputDataRate(&hlsm6d, 208.0f);
  LSM6DSR_GYRO_SetFullScale(&hlsm6d, 2000U);
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
    currTarAddr = 0x000400;
  }
  //we do not save in new file count
  //error saving at end of flight results in potential loss of a file
  //if saved now instead, errors would remove formatting page, losing all files
  currDbgAddr = 0x000200;
  HAL_GPIO_WritePin(GPIOA, EEPROM_CS_Pin, GPIO_PIN_RESET);
  Page_Erase(&hm95p32, 0x000200);
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
