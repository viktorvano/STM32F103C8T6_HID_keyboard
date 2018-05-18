/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
// USB keyboard codes
#define USB_HID_MODIFIER_LEFT_CTRL   0x01
#define USB_HID_MODIFIER_LEFT_SHIFT  0x02
#define USB_HID_MODIFIER_LEFT_ALT    0x04
#define USB_HID_MODIFIER_LEFT_GUI    0x08 // (Win/Apple/Meta)
#define USB_HID_MODIFIER_RIGHT_CTRL  0x10
#define USB_HID_MODIFIER_RIGHT_SHIFT 0x20
#define USB_HID_MODIFIER_RIGHT_ALT   0x40
#define USB_HID_MODIFIER_RIGHT_GUI   0x80

#include "usbd_hid.h"
#include "usb_hid_keys.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
struct keyboardHID_t
{
	uint8_t id;
	uint8_t modifiers;
	uint8_t key1;
	uint8_t key2;
	uint8_t key3;
};
struct keyboardHID_t keyboardHID;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  keyboardHID.id = 1;
  keyboardHID.modifiers = 0;
  keyboardHID.key1 = 0;
  keyboardHID.key2 = 0;
  keyboardHID.key3 = 0;
  HAL_Delay(800);
  HAL_GPIO_WritePin(USB_Enable_GPIO_Port, USB_Enable_Pin, SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  if(!HAL_GPIO_ReadPin(Button_RESUME_GPIO_Port, Button_RESUME_Pin))
	  {
			keyboardHID.key1 = KEY_F8;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
			HAL_Delay(20);
			while(!HAL_GPIO_ReadPin(Button_RESUME_GPIO_Port, Button_RESUME_Pin));
			keyboardHID.key1 = 0;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
			HAL_Delay(20);
	  }

	  if(!HAL_GPIO_ReadPin(Button_SUSPEND_GPIO_Port, Button_SUSPEND_Pin))
	  {
			keyboardHID.key1 = KEY_F9;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
			HAL_Delay(20);
			while(!HAL_GPIO_ReadPin(Button_SUSPEND_GPIO_Port, Button_SUSPEND_Pin));
			keyboardHID.key1 = 0;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
			HAL_Delay(20);
	  }

	  if(!HAL_GPIO_ReadPin(Button_STEP_INTO_GPIO_Port, Button_STEP_INTO_Pin))
	  {
			keyboardHID.key1 = KEY_F5;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
			HAL_Delay(20);
			while(!HAL_GPIO_ReadPin(Button_STEP_INTO_GPIO_Port, Button_STEP_INTO_Pin));
			keyboardHID.key1 = 0;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
			HAL_Delay(20);
	  }

	  if(!HAL_GPIO_ReadPin(Button_STEP_OVER_GPIO_Port, Button_STEP_OVER_Pin))
	  {
			keyboardHID.key1 = KEY_F6;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
			HAL_Delay(20);
			while(!HAL_GPIO_ReadPin(Button_STEP_OVER_GPIO_Port, Button_STEP_OVER_Pin));
			keyboardHID.key1 = 0;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
			HAL_Delay(20);
	  }

	  if(!HAL_GPIO_ReadPin(Button_STEP_RETURN_GPIO_Port, Button_STEP_RETURN_Pin))
	  {
			keyboardHID.key1 = KEY_F7;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
			HAL_Delay(20);
			while(!HAL_GPIO_ReadPin(Button_STEP_RETURN_GPIO_Port, Button_STEP_RETURN_Pin));
			keyboardHID.key1 = 0;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
			HAL_Delay(20);
	  }

	  if(!HAL_GPIO_ReadPin(Button_RUN_TO_LINE_GPIO_Port, Button_RUN_TO_LINE_Pin))
	  {
			keyboardHID.modifiers = USB_HID_MODIFIER_LEFT_CTRL;
			keyboardHID.key1 = KEY_R;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
			HAL_Delay(20);
			while(!HAL_GPIO_ReadPin(Button_RUN_TO_LINE_GPIO_Port, Button_RUN_TO_LINE_Pin));
			keyboardHID.modifiers = 0;
			keyboardHID.key1 = 0;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
			HAL_Delay(20);
	  }

	  if(!HAL_GPIO_ReadPin(Button_RESTART_GPIO_Port, Button_RESTART_Pin))
	  {
			keyboardHID.key1 = KEY_F10;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
			HAL_Delay(20);
			while(!HAL_GPIO_ReadPin(Button_RESTART_GPIO_Port, Button_RESTART_Pin));
			keyboardHID.key1 = 0;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
			HAL_Delay(20);
	  }

	  if(!HAL_GPIO_ReadPin(Button_TERMINATE_AND_RELAUNCH_GPIO_Port, Button_TERMINATE_AND_RELAUNCH_Pin))
	  {
			keyboardHID.modifiers = USB_HID_MODIFIER_LEFT_CTRL;
			keyboardHID.key1 = KEY_F8;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
			HAL_Delay(20);
			while(!HAL_GPIO_ReadPin(Button_TERMINATE_AND_RELAUNCH_GPIO_Port, Button_TERMINATE_AND_RELAUNCH_Pin));
			keyboardHID.modifiers = 0;
			keyboardHID.key1 = 0;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
			HAL_Delay(20);
	  }

	  if(!HAL_GPIO_ReadPin(Button_TERMINATE_GPIO_Port, Button_TERMINATE_Pin))
	  {
			keyboardHID.modifiers = USB_HID_MODIFIER_LEFT_CTRL;
			keyboardHID.key1 = KEY_F2;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
			HAL_Delay(20);
			while(!HAL_GPIO_ReadPin(Button_TERMINATE_GPIO_Port, Button_TERMINATE_Pin));
			keyboardHID.modifiers = 0;
			keyboardHID.key1 = 0;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
			HAL_Delay(20);
	  }

	  if(!HAL_GPIO_ReadPin(Button_RUN_GPIO_Port, Button_RUN_Pin))
	  {
			keyboardHID.modifiers = USB_HID_MODIFIER_LEFT_CTRL;
			keyboardHID.key1 = KEY_F11;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
			HAL_Delay(20);
			while(!HAL_GPIO_ReadPin(Button_RUN_GPIO_Port, Button_RUN_Pin));
			keyboardHID.modifiers = 0;
			keyboardHID.key1 = 0;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
			HAL_Delay(20);
	  }

	  if(!HAL_GPIO_ReadPin(Button_DEBUG_GPIO_Port, Button_DEBUG_Pin))
	  {
			keyboardHID.key1 = KEY_F11;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
			HAL_Delay(20);
			while(!HAL_GPIO_ReadPin(Button_DEBUG_GPIO_Port, Button_DEBUG_Pin));
			keyboardHID.key1 = 0;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
			HAL_Delay(20);
	  }

	  if(!HAL_GPIO_ReadPin(Button_BUILD_GPIO_Port, Button_BUILD_Pin))
	  {
			keyboardHID.modifiers = USB_HID_MODIFIER_LEFT_CTRL;
			keyboardHID.key1 = KEY_B;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
			HAL_Delay(20);
			while(!HAL_GPIO_ReadPin(Button_BUILD_GPIO_Port, Button_BUILD_Pin));
			keyboardHID.modifiers = 0;
			keyboardHID.key1 = 0;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
			HAL_Delay(20);
	  }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_Enable_GPIO_Port, USB_Enable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Button_RESUME_Pin Button_SUSPEND_Pin Button_STEP_INTO_Pin Button_STEP_OVER_Pin 
                           Button_STEP_RETURN_Pin Button_RUN_TO_LINE_Pin Button_RESTART_Pin Button_TERMINATE_AND_RELAUNCH_Pin */
  GPIO_InitStruct.Pin = Button_RESUME_Pin|Button_SUSPEND_Pin|Button_STEP_INTO_Pin|Button_STEP_OVER_Pin 
                          |Button_STEP_RETURN_Pin|Button_RUN_TO_LINE_Pin|Button_RESTART_Pin|Button_TERMINATE_AND_RELAUNCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_TERMINATE_Pin Button_RUN_Pin Button_DEBUG_Pin Button_BUILD_Pin */
  GPIO_InitStruct.Pin = Button_TERMINATE_Pin|Button_RUN_Pin|Button_DEBUG_Pin|Button_BUILD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_Enable_Pin */
  GPIO_InitStruct.Pin = USB_Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(USB_Enable_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
