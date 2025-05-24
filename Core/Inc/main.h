/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lps22hh.h"
#include "lsm6dsr.h"
#include "m95p32.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
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
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void M95p32_Reformat(void);
void M95p32_DebugPrint(uint8_t*, uint32_t size);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DROGUE1_Pin GPIO_PIN_13
#define DROGUE1_GPIO_Port GPIOC
#define DROGUE2_Pin GPIO_PIN_14
#define DROGUE2_GPIO_Port GPIOC
#define STATUS_LED_Pin GPIO_PIN_15
#define STATUS_LED_GPIO_Port GPIOC
#define MAIN1_Pin GPIO_PIN_1
#define MAIN1_GPIO_Port GPIOA
#define MAIN2_Pin GPIO_PIN_2
#define MAIN2_GPIO_Port GPIOA
#define EEPROM_CS_Pin GPIO_PIN_3
#define EEPROM_CS_GPIO_Port GPIOA
#define IMU_CS_Pin GPIO_PIN_4
#define IMU_CS_GPIO_Port GPIOC
#define PRESS_CS_Pin GPIO_PIN_5
#define PRESS_CS_GPIO_Port GPIOC
#define USERSENSE_Pin GPIO_PIN_5
#define USERSENSE_GPIO_Port GPIOB
#define USERSENSE_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
