/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void Channel_A_Config(void)
//{
//  uint32_t *pAHB1ClkCtrlReg =   (uint32_t*)0x40023830; // address of AHB1 clock control register
//  uint32_t *pAFRLPortAReg   =   (uint32_t*)0x40020024; // address of alternate function register of port A
//  uint32_t *pPortAModeReg   =   (uint32_t*)0x40020000; // address of port A mode register
//  uint32_t *pPortAPullUpReg =   (uint32_t*)0x4002000C; // address of port A pull up register
//
//  *pAHB1ClkCtrlReg |= 0x1;        // port A clock enable
//  *pAFRLPortAReg   |= 0x00000001; // alternate function of Timer 1 Channel 1 enabled
//  *pPortAModeReg   &= 0xFFFCFFFF;
//  *pPortAModeReg   |= 0x00020000; // port A Pin 8 configured for 'Alternate function'
//  *pPortAPullUpReg |= 0x10000;    // port A Pin 8 pulled up
//}
//
//void Channel_B_Config(void)
//{
//  uint32_t *pAHB1ClkCtrlReg =   (uint32_t*)0x40023830; // address of AHB1 clock control register
//  uint32_t *pAFRLPortAReg   =   (uint32_t*)0x40020024; // address of alternate function register of port A
//  uint32_t *pPortAModeReg   =   (uint32_t*)0x40020000; // address of port A mode register
//  uint32_t *pPortAPullUpReg =   (uint32_t*)0x4002000C; // address of port A pull up register
//
//  *pAHB1ClkCtrlReg |= 0x1;        // port A clock enable
//  *pAFRLPortAReg   |= 0x00000010; // alternate function of Timer 1 Channel 2 enabled
//  *pPortAModeReg   &= 0xFFF3FFFF;
//  *pPortAModeReg   |= 0x00080000; // port A Pin 9 configured for 'Alternate function'
//  *pPortAPullUpReg |= 0x40000;    // port A Pin 9 pulled up
//}
//
//void Channel_A_Config1(void)
//{
//  uint32_t *pAHB1ClkCtrlReg =   (uint32_t*)0x40023830; // address of AHB1 clock control register
//  uint32_t *pAFRLPortAReg   =   (uint32_t*)0x40020024; // address of alternate function register of port A
//  uint32_t *pPortAModeReg   =   (uint32_t*)0x40020000; // address of port A mode register
//  uint32_t *pPortAPullUpReg =   (uint32_t*)0x4002000C; // address of port A pull up register
//
//  *pAHB1ClkCtrlReg |= 0x1;        // port A clock enable
//  *pAFRLPortAReg   |= 0x00010000; // alternate function of Timer 2 Channel 1 enabled
//  *pPortAModeReg   &= 0x3FFFFFFF;
//  *pPortAModeReg   |= 0x80000000; // port A Pin 15 configured for 'Alternate function'
//  *pPortAPullUpReg |= 0x40000000;    // port A Pin 15 pulled up
//}
//
//void Channel_B_Config1(void)
//{
//  uint32_t *pAHB1ClkCtrlReg =   (uint32_t*)0x40023830; // address of AHB1 clock control register
//  uint32_t *pAFRLPortBReg   =   (uint32_t*)0x40020420; // address of alternate function register of port B
//  uint32_t *pPortBModeReg   =   (uint32_t*)0x40020400; // address of port B mode register
//  uint32_t *pPortBPullUpReg =   (uint32_t*)0x4002040C; // address of port B pull up register
//
//  *pAHB1ClkCtrlReg |= 0x2;        // port B clock enable
//  *pAFRLPortBReg   |= 0x00100000; // alternate function of Timer 2 Channel 2 enabled
//  *pPortBModeReg   &= 0xFFFFFF3F;
//  *pPortBModeReg   |= 0x00000080; // port B Pin 3 configured for 'Alternate function'
//  *pPortBPullUpReg |= 0x0040;     // port B Pin 3 pulled up
//}
//
//
//void Channel_A_Config2(void)
//{
//  uint32_t *pAHB1ClkCtrlReg =   (uint32_t*)0x40023830; // address of AHB1 clock control register
//  uint32_t *pAFRLPortBReg   =   (uint32_t*)0x40020420; // address of alternate function register of port B
//  uint32_t *pPortBModeReg   =   (uint32_t*)0x40020400; // address of port B mode register
//  uint32_t *pPortBPullUpReg =   (uint32_t*)0x4002040C; // address of port B pull up register
//
//  *pAHB1ClkCtrlReg |= 0x2;        // port B clock enable
//  *pAFRLPortBReg   |= 0x00001000; // alternate function of Timer 3 Channel 1 enabled
//  *pPortBModeReg   &= 0xFFFFFCFF;
//  *pPortBModeReg   |= 0x00000200; // port B Pin 4 configured for 'Alternate function'
//  *pPortBPullUpReg |= 0x00100;     // port B Pin 4 pulled up
//}
//
//
//void Channel_B_Config2(void)
//{
//  uint32_t *pAHB1ClkCtrlReg =   (uint32_t*)0x40023830; // address of AHB1 clock control register
//  uint32_t *pAFRLPortBReg   =   (uint32_t*)0x40020420; // address of alternate function register of port B
//  uint32_t *pPortBModeReg   =   (uint32_t*)0x40020400; // address of port B mode register
//  uint32_t *pPortBPullUpReg =   (uint32_t*)0x4002040C; // address of port B pull up register
//
//  *pAHB1ClkCtrlReg |= 0x2;        // port B clock enable
//  *pAFRLPortBReg   |= 0x00001000; // alternate function of Timer 3 Channel 2 enabled
//  *pPortBModeReg   &= 0xFFFFF3FF;
//  *pPortBModeReg   |= 0x00000800; // port B Pin 5 configured for 'Alternate function'
//  *pPortBPullUpReg |= 0x00400;     // port B Pin 5 pulled up
//}
//
//void Encoder_Init(uint16_t ARR_Value)
//{
//	 uint32_t *pAPB2ClkCtrlReg =   (uint32_t*)0x40023844; // address of APB2 clock control register
//	 uint32_t *pTimer1ArrReg   =   (uint32_t*)0x4001002C; // address of timer 1 auto reload register
//	 uint32_t *pTimer1CCMR1Reg =   (uint32_t*)0x40010018; // address of timer 1 capture/compare register
//	 uint32_t *pTimer1CcerReg  =   (uint32_t*)0x40010020; // address of timer 1 capture/compare enable register
//     uint32_t *pTimer1SmcrReg  =   (uint32_t*)0x40010008; // address of timer 1 slave mode control register
//     uint32_t *pTimer1CR1Reg   =   (uint32_t*)0x40010000; // address of timer 1 control register 1
//
//    *pAPB2ClkCtrlReg |= 0x1;           //timer 1 clock enable
//    *pTimer1ArrReg   =  ARR_Value - 1; // maximum count value for counter
//    *pTimer1CCMR1Reg |= 0x1;   // mapping 'Input capture 1' mode in timer 1 Channel 1 and configuring it as input
//    *pTimer1CCMR1Reg |= 0x100; // mapping 'Input capture 2' mode in timer 1 Channel 2 and configuring it as input
//    *pTimer1CcerReg  |= 0x2;   // channel 1 triggered at rising edge
//    *pTimer1CcerReg  |= 0x20;  // channel 2 triggered at rising edge
//    *pTimer1SmcrReg  |= 0x3;   // counter counts up/down on both channel 1 and channel 2 edges depending on the level of the other input.
//    *pTimer1CR1Reg   |= 0x1;   // counter enable
//}
//
//void Encoder_Init1(uint16_t ARR_Value)
//{
//	 uint32_t *pAPB1ClkCtrlReg =   (uint32_t*)0x40023840; // address of APB2 clock control register
//	 uint32_t *pTimer2ArrReg   =   (uint32_t*)0x4000002C; // address of timer 2 auto reload register
//	 uint32_t *pTimer2CCMR1Reg =   (uint32_t*)0x40000018; // address of timer 2 capture/compare register
//	 uint32_t *pTimer2CcerReg  =   (uint32_t*)0x40000020; // address of timer 2 capture/compare enable register
//     uint32_t *pTimer2SmcrReg  =   (uint32_t*)0x40000008; // address of timer 2 slave mode control register
//     uint32_t *pTimer2CR1Reg   =   (uint32_t*)0x40000000; // address of timer 2 control register 1
//
//    *pAPB1ClkCtrlReg |= 0x1;           //timer 2 clock enable
//    *pTimer2ArrReg   =  ARR_Value - 1; // maximum count value for counter
//    *pTimer2CCMR1Reg |= 0x1;   // mapping 'Input capture 1' mode in timer 2 Channel 1 and configuring it as input
//    *pTimer2CCMR1Reg |= 0x100; // mapping 'Input capture 2' mode in timer 2 Channel 2 and configuring it as input
//    *pTimer2CcerReg  |= 0x2;   // channel 1 triggered at rising edge
//    *pTimer2CcerReg  |= 0x20;  // channel 2 triggered at rising edge
//    *pTimer2SmcrReg  |= 0x3;   // counter counts up/down on both channel 1 and channel 2 edges depending on the level of the other input.
//    *pTimer2CR1Reg   |= 0x1;   // counter enable
//}
//
//
//void Encoder_Init2(uint16_t ARR_Value)
//{
//	 uint32_t *pAPB1ClkCtrlReg =   (uint32_t*)0x40023840; // address of APB2 clock control register
//	 uint32_t *pTimer3ArrReg   =   (uint32_t*)0x4000042C; // address of timer 3 auto reload register
//	 uint32_t *pTimer3CCMR1Reg =   (uint32_t*)0x40000418; // address of timer 3 capture/compare register
//	 uint32_t *pTimer3CcerReg  =   (uint32_t*)0x40000420; // address of timer 3 capture/compare enable register
//     uint32_t *pTimer3SmcrReg  =   (uint32_t*)0x40000408; // address of timer 3 slave mode control register
//     uint32_t *pTimer3CR1Reg   =   (uint32_t*)0x40000400; // address of timer 3 control register 1
//
//    *pAPB1ClkCtrlReg |= 0x2;           //timer 3 clock enable
//    *pTimer3ArrReg   =  ARR_Value - 1; // maximum count value for counter
//    *pTimer3CCMR1Reg |= 0x1;   // mapping 'Input capture 1' mode in timer 3 Channel 1 and configuring it as input
//    *pTimer3CCMR1Reg |= 0x100; // mapping 'Input capture 2' mode in timer 3 Channel 2 and configuring it as input
//    *pTimer3CcerReg  |= 0x2;   // channel 1 triggered at rising edge
//    *pTimer3CcerReg  |= 0x20;  // channel 2 triggered at rising edge
//    *pTimer3SmcrReg  |= 0x3;   // counter counts up/down on both channel 1 and channel 2 edges depending on the level of the other input.
//    *pTimer3CR1Reg   |= 0x1;   // counter enable
//}
//uint16_t Get_Encoder_Counts()
//{
//	 uint32_t *pTimer1CNTReg   =   (uint32_t*)0x40010024; // address of timer 2 counter register
//	 return *pTimer1CNTReg; // reading counts from counter register
//
//}
//uint16_t Get_Encoder_Counts1()
//{
//	 uint32_t *pTimer2CNTReg   =   (uint32_t*)0x40000024; // address of timer 2 counter register
//	 return *pTimer2CNTReg; // reading counts from counter register
//
//}
//uint16_t Get_Encoder_Counts2()
//{
//	 uint32_t *pTimer3CNTReg   =   (uint32_t*)0x40000424; // address of timer 3 counter register
//	 return *pTimer3CNTReg; // reading counts from counter register
//
//}



void Channel_A_Config(void)
{
  uint32_t *pAHB1ClkCtrlReg =   (uint32_t*)0x40023830; // address of AHB1 clock control register
  uint32_t *pAFRLPortAReg   =   (uint32_t*)0x40020820; // address of alternate function register of port A
  uint32_t *pPortCModeReg   =   (uint32_t*)0x40020800; // address of port C mode register
  uint32_t *pPortCPullUpReg =   (uint32_t*)0x4002080C; // address of port C pull up register

  *pAHB1ClkCtrlReg |= 0x4;        // port C clock enable
  *pAFRLPortAReg   |= 0x03000000; // alternate function of Timer 8 Channel 1 enabled
  *pPortCModeReg   &= 0xFFFFCFFF;
  *pPortCModeReg   |= 0x00002000; // port C Pin 6 configured for 'Alternate function'
  *pPortCPullUpReg |= 0x1000;    // port C Pin 6 pulled up
}

void Channel_B_Config(void)
{
  uint32_t *pAHB1ClkCtrlReg =   (uint32_t*)0x40023830; // address of AHB1 clock control register
  uint32_t *pAFRLPortAReg   =   (uint32_t*)0x40020820; // address of alternate function register of port A
  uint32_t *pPortCModeReg   =   (uint32_t*)0x40020800; // address of port C mode register
  uint32_t *pPortCPullUpReg =   (uint32_t*)0x4002080C; // address of port C pull up register

  *pAHB1ClkCtrlReg |= 0x4;        // port C clock enable
  *pAFRLPortAReg   |= 0x30000000; // alternate function of Timer 8 Channel 2 enabled
  *pPortCModeReg   &= 0xFFFF3FFF;
  *pPortCModeReg   |= 0x00008000; // port C Pin 7 configured for 'Alternate function'
  *pPortCPullUpReg |= 0x4000;    // port C Pin 7 pulled up
}

void Encoder_Init(uint16_t ARR_Value)
{
	 uint32_t *pAPB2ClkCtrlReg =   (uint32_t*)0x40023844; // address of APB2 clock control register
	 uint32_t *pTimer8ArrReg   =   (uint32_t*)0x4001082C; // address of timer 8 auto reload register
	 uint32_t *pTimer8CCMR1Reg =   (uint32_t*)0x40010818; // address of timer 8 capture/compare register
	 uint32_t *pTimer8CcerReg  =   (uint32_t*)0x40010820; // address of timer 8 capture/compare enable register
     uint32_t *pTimer8SmcrReg  =   (uint32_t*)0x40010808; // address of timer 8 slave mode control register
     uint32_t *pTimer8CR1Reg   =   (uint32_t*)0x40010800; // address of timer 8 control register 1

    *pAPB2ClkCtrlReg |= 0x2;           //timer 1 clock enable
    *pTimer8ArrReg   =  ARR_Value - 1; // maximum count value for counter
    *pTimer8CCMR1Reg |= 0x1;   // mapping 'Input capture 1' mode in timer 8 Channel 1 and configuring it as input
    *pTimer8CCMR1Reg |= 0x100; // mapping 'Input capture 2' mode in timer 8 Channel 2 and configuring it as input
    *pTimer8CcerReg  |= 0x2;   // channel 1 triggered at rising edge
    *pTimer8CcerReg  |= 0x20;  // channel 2 triggered at rising edge
    *pTimer8SmcrReg  |= 0x3;   // counter counts up/down on both channel 1 and channel 2 edges depending on the level of the other input.
    *pTimer8CR1Reg   |= 0x1;   // counter enable
}

uint16_t Get_Encoder_Counts()
{
	 uint32_t *pTimer8CNTReg   =   (uint32_t*)0x40010824; // address of timer 8 counter register
	 return *pTimer8CNTReg; // reading counts from counter register

}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Channel_A_Config();
  Channel_B_Config();
  Encoder_Init(65535);

//  Channel_A_Config1();
//  Channel_B_Config1();
//  Encoder_Init1(65535);
//
//  Channel_A_Config2();
//  Channel_B_Config2();
//  Encoder_Init2(65535);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  printf("Encoder: %u %u %u\r\n", Get_Encoder_Counts(), Get_Encoder_Counts1(), Get_Encoder_Counts2());
	  printf("Encoder: %u\r\n", Get_Encoder_Counts());
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);
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
