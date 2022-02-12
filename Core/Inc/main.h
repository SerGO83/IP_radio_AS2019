/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define COL_13_Pin GPIO_PIN_2
#define COL_13_GPIO_Port GPIOE
#define COL_09_Pin GPIO_PIN_3
#define COL_09_GPIO_Port GPIOE
#define HC138_C_Pin GPIO_PIN_4
#define HC138_C_GPIO_Port GPIOE
#define COL_11_Pin GPIO_PIN_6
#define COL_11_GPIO_Port GPIOE
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define USER_Btn_EXTI_IRQn EXTI15_10_IRQn
#define COL_08_Pin GPIO_PIN_0
#define COL_08_GPIO_Port GPIOF
#define COL_10_Pin GPIO_PIN_1
#define COL_10_GPIO_Port GPIOF
#define COL_12_Pin GPIO_PIN_2
#define COL_12_GPIO_Port GPIOF
#define HC138_A_Pin GPIO_PIN_3
#define HC138_A_GPIO_Port GPIOF
#define HC138_B_Pin GPIO_PIN_5
#define HC138_B_GPIO_Port GPIOF
#define COL_06_Pin GPIO_PIN_7
#define COL_06_GPIO_Port GPIOF
#define COL_07_Pin GPIO_PIN_8
#define COL_07_GPIO_Port GPIOF
#define COL_04_Pin GPIO_PIN_9
#define COL_04_GPIO_Port GPIOF
#define SSD1351_DC_Pin GPIO_PIN_10
#define SSD1351_DC_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define LGHT_SNS_POT_Pin GPIO_PIN_2
#define LGHT_SNS_POT_GPIO_Port GPIOC
#define COL_18_Pin GPIO_PIN_0
#define COL_18_GPIO_Port GPIOA
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_4
#define SPI3_CS_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define COL_19_Pin GPIO_PIN_0
#define COL_19_GPIO_Port GPIOB
#define LIN_POT_Pin GPIO_PIN_1
#define LIN_POT_GPIO_Port GPIOB
#define COL_02_Pin GPIO_PIN_0
#define COL_02_GPIO_Port GPIOG
#define COL_01_Pin GPIO_PIN_1
#define COL_01_GPIO_Port GPIOG
#define SW3_BTN_Pin GPIO_PIN_7
#define SW3_BTN_GPIO_Port GPIOE
#define SW4_BTN_Pin GPIO_PIN_8
#define SW4_BTN_GPIO_Port GPIOE
#define COL_15_Pin GPIO_PIN_10
#define COL_15_GPIO_Port GPIOE
#define COL_16_Pin GPIO_PIN_12
#define COL_16_GPIO_Port GPIOE
#define COL_17_Pin GPIO_PIN_14
#define COL_17_GPIO_Port GPIOE
#define ENC_BTN_Pin GPIO_PIN_15
#define ENC_BTN_GPIO_Port GPIOE
#define SW5_BTN_Pin GPIO_PIN_11
#define SW5_BTN_GPIO_Port GPIOB
#define SPI3_DREQ_Pin GPIO_PIN_12
#define SPI3_DREQ_GPIO_Port GPIOB
#define SPI3_DREQ_EXTI_IRQn EXTI15_10_IRQn
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define SPI3_DCS_Pin GPIO_PIN_15
#define SPI3_DCS_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define COL_14_Pin GPIO_PIN_11
#define COL_14_GPIO_Port GPIOD
#define SPI3_RESET_Pin GPIO_PIN_14
#define SPI3_RESET_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define COL_05_Pin GPIO_PIN_0
#define COL_05_GPIO_Port GPIOD
#define COL_03_Pin GPIO_PIN_1
#define COL_03_GPIO_Port GPIOD
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define SSD1351_CS_Pin GPIO_PIN_4
#define SSD1351_CS_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define COL_20_Pin GPIO_PIN_0
#define COL_20_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
