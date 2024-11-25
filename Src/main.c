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
#include "liquidcrystal_i2c.h"
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
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
uint8_t keyPressed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
uint8_t calculateChecksum(const uint8_t* data, uint8_t length);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void SendCommand(uint8_t *command, uint8_t length);
HAL_StatusTypeDef UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t size, uint32_t timeout);
uint8_t uint8ToBCD(uint8_t value);
HAL_StatusTypeDef WriteUnitPrice(uint8_t address, int price);
HAL_StatusTypeDef ReadUnitPrice(uint8_t address);
HAL_StatusTypeDef gainControl(uint8_t address);
HAL_StatusTypeDef setPreset(uint8_t address,uint8_t fuelAmount);
HAL_StatusTypeDef liftPreset(uint8_t address);
HAL_StatusTypeDef startFueling(uint8_t address);
HAL_StatusTypeDef resumeFueling(uint8_t address);
HAL_StatusTypeDef stopFueling(uint8_t address);
HAL_StatusTypeDef pauseFueling(uint8_t address);
HAL_StatusTypeDef returnControl(uint8_t address);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	HAL_StatusTypeDef Write_UnitPrice;
	HAL_StatusTypeDef Read_UnitPrice;
	HAL_StatusTypeDef gain_Control;
	HAL_StatusTypeDef return_Control;
	HAL_StatusTypeDef set_Preset;
	HAL_StatusTypeDef lift_Preset;
	HAL_StatusTypeDef start_Fueling;
	HAL_StatusTypeDef resume_Fueling;
	HAL_StatusTypeDef stop_Fueling;
	HAL_StatusTypeDef pause_Fueling;
	uint8_t address;
	uint8_t fuelAmount;
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
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
  /* USER CODE END 2 */
  HD44780_Init(2);
  HD44780_Clear();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  HD44780_SetCursor(0,0);
	  HD44780_PrintStr("Select the Dispenser ID: ");
	  HAL_Delay(500);
	  if(keyPressed >=1 && keyPressed <=99){
		  address = keyPressed;
		  HD44780_Clear();

		  if(keyPressed == 'A'){// A is for Control

			  if(keyPressed == '1')
				  gain_Control = gainControl(address);
			  else if(keyPressed == '2')
				  return_Control = returnControl(address);
			  else{
				  HD44780_SetCursor(0,0);
				  HD44780_PrintStr("Select the Dispenser ID: ");
				  HAL_Delay(500);
			  }
		  }
		  else if(keyPressed == 'B'){ // B is for fueling
			  lift_Preset = liftPreset(address);
			  HAL_Delay(500);
			  HD44780_Clear();
			  HD44780_SetCursor(0,0);
			  HD44780_PrintStr("Press the Amount in Liter");
			  HAL_Delay(500);
			  HD44780_Clear();
			  if(keyPressed >=1 && keyPressed <=5){
				  fuelAmount = keyPressed;
				  set_Preset = setPreset(address, fuelAmount);
				  HD44780_Clear();
				  HD44780_SetCursor(0,0);
				  HD44780_PrintStr("Press 1 to start fueling");
				  HAL_Delay(500);
				  HD44780_Clear();
				  HD44780_PrintStr("Press 2 to Pause fueling");
				  HAL_Delay(500);
				  HD44780_Clear();
				  HD44780_PrintStr("Press 3 to Resume fueling");
				  HAL_Delay(500);
				  HD44780_Clear();
				  HD44780_PrintStr("Press 4 to Stop fueling");
				  HAL_Delay(500);
				  if(keyPressed == '1'){
					  start_Fueling = startFueling(address);
				  }else if(keyPressed == '2'){
					  pause_Fueling = pauseFueling(address);
				  }else if(keyPressed == '3'){
					  resume_Fueling = resumeFueling(address);
				  }else if(keyPressed == '4'){
					  stop_Fueling = stopFueling(address);
				  }
			  }
		  }
		  else if(keyPressed == 'C'){ // C is for price change

		  }
		  else if(keyPressed == 'D'){

		  }
	  }
	  else
	  {
		  HD44780_Clear();
		  HD44780_SetCursor(0,0);
		  HD44780_PrintStr("Select the right Dispenser ID");
	  }


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  currentMillis = HAL_GetTick();
  if (currentMillis - previousMillis > 10) {
    /*Configure GPIO pins : PB6 PB7 PB8 PB9 to GPIO_INPUT*/
    GPIO_InitStructPrivate.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
    GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructPrivate);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
    if(GPIO_Pin == GPIO_PIN_6 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))
    {
      keyPressed = 68; //ASCII value of D
    }
    else if(GPIO_Pin == GPIO_PIN_7 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5))
    {
      keyPressed = 67; //ASCII value of C
    }
    else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6))
    {
      keyPressed = 66; //ASCII value of B
    }
    else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))
    {
      keyPressed = 65; //ASCII value of A
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
    if(GPIO_Pin == GPIO_PIN_6 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))
    {
      keyPressed = 35; //ASCII value of #
    }
    else if(GPIO_Pin == GPIO_PIN_7 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5))
    {
      keyPressed = 57; //ASCII value of 9
    }
    else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6))
    {
      keyPressed = 54; //ASCII value of 6
    }
    else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))
    {
      keyPressed = 51; //ASCII value of 3
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
    if(GPIO_Pin == GPIO_PIN_6 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))
    {
      keyPressed = 48; //ASCII value of 0
    }
    else if(GPIO_Pin == GPIO_PIN_7 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5))
    {
      keyPressed = 56; //ASCII value of 8
    }
    else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6))
    {
      keyPressed = 53; //ASCII value of 5
    }
    else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))
    {
      keyPressed = 50; //ASCII value of 2
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
    if(GPIO_Pin == GPIO_PIN_6 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))
    {
      keyPressed = 42; //ASCII value of *
    }
    else if(GPIO_Pin == GPIO_PIN_7 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5))
    {
      keyPressed = 55; //ASCII value of 7
    }
    else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6))
    {
      keyPressed = 52; //ASCII value of 4
    }
    else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))
    {
      keyPressed = 49; //ASCII value of 1
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
    /*Configure GPIO pins : PB6 PB7 PB8 PB9 back to EXTI*/
    GPIO_InitStructPrivate.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructPrivate.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructPrivate);
    previousMillis = currentMillis;
  }
}
uint8_t calculateChecksum(const uint8_t* data, uint8_t length) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum ^= data[i]; // XOR operation
    }
    // Clear the most significant bit (MSB)
    checksum &= 0x7F; // 0x7F = 01111111 in binary
    return checksum;
}
HAL_StatusTypeDef WriteUnitPrice(uint8_t address, int price) {
    uint8_t command[6] = {0xF5, address, 0xA4, 0x00, 0xB2, 0x00};
    size_t length = sizeof(command) / sizeof(command[0]);
    uint8_t price2 = uint8ToBCD(price);
    command[length-3] = price2;
    uint8_t sum = calculateChecksum(command, length);
    uint8_t rxBuffer[10];
    command[length-1] = sum;
    SendCommand(command, length);
    HAL_StatusTypeDef status = UART_Receive(&huart1, rxBuffer, 10, 1000);
    return status;
}
uint8_t uint8ToBCD(uint8_t value)
{
    uint8_t bcd = 0;
    uint8_t shift = 0;

    // Convert each decimal digit into BCD
    while (value > 0)
    {
        bcd |= (value % 10) << (shift * 4); // Extract a decimal digit and shift it
        value /= 10;
        shift++;
    }

    return bcd;
}

void SendCommand(uint8_t *command, uint8_t length) {
    uint8_t checksum = calculateChecksum(command, length);
    command[length - 1] = checksum;
    if (HAL_UART_Transmit(&huart1, command, length, HAL_MAX_DELAY) != HAL_OK) {
        Error_Handler();
    }
}
HAL_StatusTypeDef UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t size, uint32_t timeout)
{
    // Ensure the UART handle is valid
    if (huart == NULL || pData == NULL)
    {
        return HAL_ERROR;  // Invalid parameters
    }

    // Call the HAL function for UART reception
    return HAL_UART_Receive(huart, pData, size, timeout);
}
HAL_StatusTypeDef ReadUnitPrice(uint8_t address) {
    uint8_t command[5] = {0xF5, address, 0xA2, 0xB6, 0x00};
    size_t length = sizeof(command) / sizeof(command[0]);
    uint8_t sum = calculateChecksum(command, length);
    uint8_t rxBuffer[10];
    command[length-1] = sum;
    SendCommand(command, length);
    HAL_StatusTypeDef status = UART_Receive(&huart1, rxBuffer, 10, 1000);
    return status;
}
HAL_StatusTypeDef gainControl(uint8_t address) {
    uint8_t command[5] = {0xF5, address, 0xA2, 0xE5, 0x00};
    size_t length = sizeof(command) / sizeof(command[0]);
    uint8_t sum = calculateChecksum(command, length);
    uint8_t rxBuffer[10];
    command[length-1] = sum;
    SendCommand(command, length);
    HAL_StatusTypeDef status = UART_Receive(&huart1, rxBuffer, 10, 1000);
    return status;
}
HAL_StatusTypeDef returnControl(uint8_t address) {
    uint8_t command[5] = {0xF5, address, 0xA2, 0xE5, 0x00};
    size_t length = sizeof(command) / sizeof(command[0]);
    uint8_t sum = calculateChecksum(command, length);
    uint8_t rxBuffer[10];
    command[length-1] = sum;
    SendCommand(command, length);
    HAL_StatusTypeDef status = UART_Receive(&huart1, rxBuffer, 10, 1000);
    return status;
}
HAL_StatusTypeDef setPreset(uint8_t address,uint8_t fuelAmount) {
    uint8_t command[6] = {0xF5, address, 0xA5, 0x00, 0xB5, 0x00};
    size_t length = sizeof(command) / sizeof(command[0]);
    uint8_t fuelAmount2 = uint8ToBCD(fuelAmount);
    command[length-3] = fuelAmount2;
    uint8_t sum = calculateChecksum(command, length);
    uint8_t rxBuffer[10];
    command[length-1] = sum;
    SendCommand(command, length);
    HAL_StatusTypeDef status = UART_Receive(&huart1, rxBuffer, 10, 1000);
    return status;
}
HAL_StatusTypeDef liftPreset(uint8_t address) {
    uint8_t command[6] = {0xF5, address, 0xA6, 0x00, 0xB9, 0x00};
    size_t length = sizeof(command) / sizeof(command[0]);
    uint8_t sum = calculateChecksum(command, length);
    uint8_t rxBuffer[10];
    command[length-1] = sum;
    SendCommand(command, length);
    HAL_StatusTypeDef status = UART_Receive(&huart1, rxBuffer, 10, 1000);
    return status;
}
HAL_StatusTypeDef startFueling(uint8_t address) {
    uint8_t command[5] = {0xF5, address, 0xA2, 0xC3, 0x00};
    size_t length = sizeof(command) / sizeof(command[0]);
    uint8_t sum = calculateChecksum(command, length);
    uint8_t rxBuffer[10];
    command[length-1] = sum;
    SendCommand(command, length);
    HAL_StatusTypeDef status = UART_Receive(&huart1, rxBuffer, 10, 1000);
    return status;
}
HAL_StatusTypeDef stopFueling(uint8_t address) {
    uint8_t command[5] = {0xF5, address, 0xA2, 0xCA, 0x00};
    size_t length = sizeof(command) / sizeof(command[0]);
    uint8_t sum = calculateChecksum(command, length);
    uint8_t rxBuffer[10];
    command[length-1] = sum;
    SendCommand(command, length);
    HAL_StatusTypeDef status = UART_Receive(&huart1, rxBuffer, 10, 1000);
    return status;
}
HAL_StatusTypeDef resumeFueling(uint8_t address) {
    uint8_t command[5] = {0xF5, address, 0xA2, 0xB3, 0x00};
    size_t length = sizeof(command) / sizeof(command[0]);
    uint8_t sum = calculateChecksum(command, length);
    uint8_t rxBuffer[10];
    command[length-1] = sum;
    SendCommand(command, length);
    HAL_StatusTypeDef status = UART_Receive(&huart1, rxBuffer, 10, 1000);
    return status;
}
HAL_StatusTypeDef pauseFueling(uint8_t address) {
    uint8_t command[5] = {0xF5, address, 0xA2, 0xBA, 0x00};
    size_t length = sizeof(command) / sizeof(command[0]);
    uint8_t sum = calculateChecksum(command, length);
    uint8_t rxBuffer[10];
    command[length-1] = sum;
    SendCommand(command, length);
    HAL_StatusTypeDef status = UART_Receive(&huart1, rxBuffer, 10, 1000);
    return status;
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
