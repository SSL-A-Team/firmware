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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define SPI1_INT1_Pin GPIO_PIN_1
#define SPI1_INT1_GPIO_Port GPIOF
#define SPI1_INT2_Pin GPIO_PIN_2
#define SPI1_INT2_GPIO_Port GPIOF
#define SPI1_INT3_Pin GPIO_PIN_3
#define SPI1_INT3_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define BATT_MON_Pin GPIO_PIN_0
#define BATT_MON_GPIO_Port GPIOA
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define UART7_BOOT0_Pin GPIO_PIN_0
#define UART7_BOOT0_GPIO_Port GPIOG
#define UART7_RST_Pin GPIO_PIN_1
#define UART7_RST_GPIO_Port GPIOG
#define USART3_BOOT0_Pin GPIO_PIN_14
#define USART3_BOOT0_GPIO_Port GPIOE
#define USART3_RST_Pin GPIO_PIN_15
#define USART3_RST_GPIO_Port GPIOE
#define USART3_TX_Pin GPIO_PIN_10
#define USART3_TX_GPIO_Port GPIOB
#define USART3_RX_Pin GPIO_PIN_11
#define USART3_RX_GPIO_Port GPIOB
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_15
#define LED_GREEN_GPIO_Port GPIOB
#define DIP_3_Pin GPIO_PIN_14
#define DIP_3_GPIO_Port GPIOD
#define DIP_2_Pin GPIO_PIN_15
#define DIP_2_GPIO_Port GPIOD
#define DIP_1_Pin GPIO_PIN_2
#define DIP_1_GPIO_Port GPIOG
#define DIP_0_Pin GPIO_PIN_3
#define DIP_0_GPIO_Port GPIOG
#define BTN_1_Pin GPIO_PIN_4
#define BTN_1_GPIO_Port GPIOG
#define BTN_0_Pin GPIO_PIN_5
#define BTN_0_GPIO_Port GPIOG
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
#define SDIO_DET_Pin GPIO_PIN_0
#define SDIO_DET_GPIO_Port GPIOD
#define USART2_BOOT0_Pin GPIO_PIN_4
#define USART2_BOOT0_GPIO_Port GPIOD
#define USART2_RST_Pin GPIO_PIN_7
#define USART2_RST_GPIO_Port GPIOD
#define USART6_BOOT0_Pin GPIO_PIN_10
#define USART6_BOOT0_GPIO_Port GPIOG
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define USART6_RST_Pin GPIO_PIN_12
#define USART6_RST_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define SPI1_NSS1_Pin GPIO_PIN_15
#define SPI1_NSS1_GPIO_Port GPIOG
#define SPI1_NSS0_Pin GPIO_PIN_6
#define SPI1_NSS0_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define UART8_BOOT0_Pin GPIO_PIN_8
#define UART8_BOOT0_GPIO_Port GPIOB
#define UART8_RST_Pin GPIO_PIN_9
#define UART8_RST_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
