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
#include "FlightState.h"
#include "sensor_filter_ops.h"
#include "memops_ex.h"
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
CRC_HandleTypeDef hcrc;

/* USER CODE BEGIN PV */
extern __IO uint32_t uwTick;
extern SPI_HandleTypeDef hspi1;
LPS22HH_Object_t hlps22; //pressure + barometer
LSM6DSR_Object_t hlsm6d; //acc + gyro
FlightState      fs;

float gAltitude, gTotalAcc, gDegOffVert;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
static void Lps22_Init(void);
static void Lsm6D_Init(void);

extern void temocTests();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
dataframe_t nextFrame __attribute__((aligned (4))); //aligned for faster data transfer
dataframe_t stagedMem[6] __attribute__((aligned (4)));
uint8_t memIndex = 0;
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
		  calibrated = CalibrateGyroData(&hlsm6d);
	  }
  }

  /*Calibrate Accelerometer*/
  calibrated = false;
  nextTick = uwTick;

  if (MotionAC_LoadCalFromNVM(sizeof(cal_data), cal_data) == 0) {
    extern MAC_output_t ac_data_out;
    memcpy(&ac_data_out, cal_data, sizeof(ac_data_out));
	printf("Calibration loaded\n");
  } else {
	while(!calibrated)
	{
	  if(uwTick >= nextTick) // 10ms passed
	  {
		  nextTick = uwTick + 10;
		  calibrated = CalibrateAccData(&hlsm6d);
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

		GetAltitude(&nextFrame);

		LSM6DSR_GYRO_GetAxes(&hlsm6d, &nextFrame.currGyro);
		LSM6DSR_ACC_GetAxes(&hlsm6d, &nextFrame.currAcc);

		//apply calibration bias to raw values:
		ApplyGyroCalibration(&nextFrame.currGyro, &nextFrame.currAcc);
		ApplyAccCalibration(&nextFrame.currAcc, &nextFrame.totalAcc);

		gAltitude = nextFrame.altitude;
		gTotalAcc = nextFrame.totalAcc;
		gDegOffVert = nextFrame.pitch;

		ProcessSensorData(&nextFrame);
    }

    updateState(&fs);

    if(uwTick >= nextTick)
    {
      nextTick = uwTick + 500; //0.05s delay

      //perform a memory store
      nextFrame.currTick = uwTick;
      memcpy(&stagedMem[memIndex], &nextFrame, sizeof(dataframe_t));
      ++memIndex;

      if(memIndex >= 6) //mem block full
      {
        memIndex = 0;
        memFlag = 1;
      }
    }

    if(memFlag != 0)
    {
      memFlag = 0;
      //block is ready to be stored

      uint32_t ret = M95p32_SavePage((uint8_t*)stagedMem, sizeof(stagedMem));

      if(ret) {
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
  LSM6DSR_GYRO_SetFullScale(&hlsm6d, 250U); //changed from 2000 to 250
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
    while (1) {
      M95p32_DebugSectorDump((uint8_t*)sector, 0U);
      (void)sector;
    }
  }
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
