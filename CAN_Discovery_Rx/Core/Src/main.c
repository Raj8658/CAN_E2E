#include <main.h>
#include <string.h>
#include <stdio.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_i2c.h>
#include <stm32f4xx_hal_def.h>
#include <stm32f4xx_it.h>

CAN_HandleTypeDef hcan2;
I2C_HandleTypeDef hi2c1;

extern I2C_HandleTypeDef hi2c1;
#define LCD_ADDR (0x27 << 1)
#define GREEN_LED_PIN GPIO_PIN_12
#define RED_LED_PIN GPIO_PIN_14
#define LED_GPIO_PORT GPIOD

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN2_Init(void);
static void MX_I2C1_Init(void);
void LCD_Init(void);
void LCD_SendCommand(uint8_t);
void LCD_SendData(uint8_t);
void LCD_SendString(char*);
void LCD_SetCursor(uint8_t, uint8_t);
void LCD_SendNibble(uint8_t, uint8_t);
uint8_t calculate_crc(uint8_t* data, uint8_t len);

void LCD_Delay(uint32_t);

void Error_Handler(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_CAN2_Init();
  MX_I2C1_Init();
  LCD_Init();

  HAL_CAN_Start(&hcan2);

  CAN_FilterTypeDef filter;
  filter.FilterActivation = CAN_FILTER_ENABLE;
  filter.FilterBank = 0;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  HAL_CAN_ConfigFilter(&hcan2, &filter);

  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];
  char display_buffer[17];

  while (1)
  {
    if (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) > 0)
    {
      if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
      {
        uint8_t counter = rxData[0];
        uint8_t data_len = rxHeader.DLC - 2;
        uint8_t received_crc = rxData[data_len + 1];

        if (received_crc == calculate_crc(rxData, data_len + 1))
        {
          memcpy(display_buffer, &rxData[1], data_len);
          display_buffer[data_len] = '\0';

          LCD_SetCursor(0, 0);
          LCD_SendString("Received:");
          LCD_SetCursor(1, 0);
          LCD_SendString(display_buffer);

          HAL_GPIO_WritePin(LED_GPIO_PORT, GREEN_LED_PIN, GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_GPIO_PORT, RED_LED_PIN, GPIO_PIN_RESET);
        }
        else
        {
          LCD_SetCursor(0, 0);
          LCD_SendString("CRC Error     ");
          HAL_GPIO_WritePin(LED_GPIO_PORT, GREEN_LED_PIN, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_GPIO_PORT, RED_LED_PIN, GPIO_PIN_SET);
        }
      }
    }
  }
}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

uint8_t calculate_crc(uint8_t* data, uint8_t len)
{
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++)
      crc = (crc & 0x80) ? (crc << 1) ^ 0x1D : (crc << 1);
  }
  return crc;
}

// LCD Driver for PCF8574-based 16x2 I2C LCD
void LCD_Init(void)
{
  HAL_Delay(50);
  LCD_SendNibble(0x30, 0);
  HAL_Delay(5);
  LCD_SendNibble(0x30, 0);
  HAL_Delay(1);
  LCD_SendNibble(0x30, 0);
  LCD_SendNibble(0x20, 0);

  LCD_SendCommand(0x28);
  LCD_SendCommand(0x0C);
  LCD_SendCommand(0x06);
  LCD_SendCommand(0x01);
  HAL_Delay(2);
}

void LCD_SendNibble(uint8_t nibble, uint8_t rs)
{
  uint8_t data = (nibble & 0xF0) | (rs ? 0x01 : 0x00) | 0x08;
  uint8_t en = data | 0x04;
  uint8_t seq[4] = {data, en, data, 0};
  HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, seq, 4, 100);
}

void LCD_SendCommand(uint8_t cmd)
{
  LCD_SendNibble(cmd & 0xF0, 0);
  LCD_SendNibble((cmd << 4) & 0xF0, 0);
}

void LCD_SendData(uint8_t data)
{
  LCD_SendNibble(data & 0xF0, 1);
  LCD_SendNibble((data << 4) & 0xF0, 1);
}

void LCD_SendString(char *str)
{
  while (*str)
    LCD_SendData(*str++);
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
  LCD_SendCommand(0x80 | (row ? 0x40 : 0x00) | col);
}

// Initialize GPIO for LEDs
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GREEN_LED_PIN | RED_LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

static void MX_CAN2_Init(void)
{
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 5;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(&hcan2) != HAL_OK)
    Error_Handler();
}

static void MX_I2C1_Init(void)
{
  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
    Error_Handler();
}
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


//#include <main.h>
//#include <string.h>
//#include <stdio.h>
//#include <stm32f4xx_hal.h>
//#include <stm32f4xx_hal_i2c.h>
//#include <stm32f4xx_hal_def.h>
//#include <stm32f4xx_it.h>
//
//CAN_HandleTypeDef hcan1;
//I2C_HandleTypeDef hi2c1;
//
//extern I2C_HandleTypeDef hi2c1;
//#define LCD_ADDR (0x27 << 1)
//#define GREEN_LED_PIN GPIO_PIN_12
//#define RED_LED_PIN GPIO_PIN_14
//#define LED_GPIO_PORT GPIOD
//
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_CAN1_Init(void);
//static void MX_I2C1_Init(void);
//void LCD_Init(void);
//void LCD_SendCommand(uint8_t);
//void LCD_SendData(uint8_t);
//void LCD_SendString(char*);
//void LCD_SetCursor(uint8_t, uint8_t);
//void LCD_SendNibble(uint8_t, uint8_t);
//uint8_t calculate_crc(uint8_t* data, uint8_t len);
//
//void LCD_Delay(uint32_t);
//
//void Error_Handler(void);
//
//int main(void)
//{
//  HAL_Init();
//  SystemClock_Config();
//  MX_GPIO_Init();
//  MX_CAN1_Init();
//  MX_I2C1_Init();
//  LCD_Init();
//
//  HAL_CAN_Start(&hcan1);
//
//  CAN_FilterTypeDef filter;
//  filter.FilterActivation = CAN_FILTER_ENABLE;
//  filter.FilterBank = 0;
//  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//  filter.FilterIdHigh = 0x0000;
//  filter.FilterIdLow = 0x0000;
//  filter.FilterMaskIdHigh = 0x0000;
//  filter.FilterMaskIdLow = 0x0000;
//  filter.FilterMode = CAN_FILTERMODE_IDMASK;
//  filter.FilterScale = CAN_FILTERSCALE_32BIT;
//  HAL_CAN_ConfigFilter(&hcan1, &filter);
//
//  CAN_RxHeaderTypeDef rxHeader;
//  uint8_t rxData[8];
//  char display_buffer[17];
//
//  while (1)
//  {
//    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
//    {
//      if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
//      {
//        uint8_t counter = rxData[0];
//        uint8_t data_len = rxHeader.DLC - 2;
//        uint8_t received_crc = rxData[data_len + 1];
//
//        if (received_crc == calculate_crc(rxData, data_len + 1))
//        {
//          memcpy(display_buffer, &rxData[1], data_len);
//          display_buffer[data_len] = '\0';
//
//          LCD_SetCursor(0, 0);
//          LCD_SendString("Received:");
//          LCD_SetCursor(1, 0);
//          LCD_SendString(display_buffer);
//
//          HAL_GPIO_WritePin(LED_GPIO_PORT, GREEN_LED_PIN, GPIO_PIN_SET);
//          HAL_GPIO_WritePin(LED_GPIO_PORT, RED_LED_PIN, GPIO_PIN_RESET);
//        }
//        else
//        {
//          LCD_SetCursor(0, 0);
//          LCD_SendString("CRC Error     ");
//          HAL_GPIO_WritePin(LED_GPIO_PORT, GREEN_LED_PIN, GPIO_PIN_RESET);
//          HAL_GPIO_WritePin(LED_GPIO_PORT, RED_LED_PIN, GPIO_PIN_SET);
//        }
//      }
//    }
//  }
//}
//
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage
//  */
//  __HAL_RCC_PWR_CLK_ENABLE();
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 4;
//  RCC_OscInitStruct.PLL.PLLN = 50;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//  RCC_OscInitStruct.PLL.PLLQ = 7;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
//uint8_t calculate_crc(uint8_t* data, uint8_t len)
//{
//  uint8_t crc = 0xFF;
//  for (uint8_t i = 0; i < len; i++)
//  {
//    crc ^= data[i];
//    for (uint8_t j = 0; j < 8; j++)
//      crc = (crc & 0x80) ? (crc << 1) ^ 0x1D : (crc << 1);
//  }
//  return crc;
//}
//
//// LCD Driver for PCF8574-based 16x2 I2C LCD
//void LCD_Init(void)
//{
//  HAL_Delay(50);
//  LCD_SendNibble(0x30, 0);
//  HAL_Delay(5);
//  LCD_SendNibble(0x30, 0);
//  HAL_Delay(1);
//  LCD_SendNibble(0x30, 0);
//  LCD_SendNibble(0x20, 0);
//
//  LCD_SendCommand(0x28);
//  LCD_SendCommand(0x0C);
//  LCD_SendCommand(0x06);
//  LCD_SendCommand(0x01);
//  HAL_Delay(2);
//}
//
//void LCD_SendNibble(uint8_t nibble, uint8_t rs)
//{
//  uint8_t data = (nibble & 0xF0) | (rs ? 0x01 : 0x00) | 0x08;
//  uint8_t en = data | 0x04;
//  uint8_t seq[4] = {data, en, data, 0};
//  HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, seq, 4, 100);
//}
//
//void LCD_SendCommand(uint8_t cmd)
//{
//  LCD_SendNibble(cmd & 0xF0, 0);
//  LCD_SendNibble((cmd << 4) & 0xF0, 0);
//}
//
//void LCD_SendData(uint8_t data)
//{
//  LCD_SendNibble(data & 0xF0, 1);
//  LCD_SendNibble((data << 4) & 0xF0, 1);
//}
//
//void LCD_SendString(char *str)
//{
//  while (*str)
//    LCD_SendData(*str++);
//}
//
//void LCD_SetCursor(uint8_t row, uint8_t col)
//{
//  LCD_SendCommand(0x80 | (row ? 0x40 : 0x00) | col);
//}
//
//// Initialize GPIO for LEDs
//static void MX_GPIO_Init(void)
//{
//  __HAL_RCC_GPIOD_CLK_ENABLE();
//
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  GPIO_InitStruct.Pin = GREEN_LED_PIN | RED_LED_PIN;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
//}
//
//static void MX_CAN1_Init(void)
//{
//  hcan1.Instance = CAN1;
//  hcan1.Init.Prescaler = 5;
//  hcan1.Init.Mode = CAN_MODE_NORMAL;
//  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
//  hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
//  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
//  hcan1.Init.TimeTriggeredMode = DISABLE;
//  hcan1.Init.AutoBusOff = DISABLE;
//  hcan1.Init.AutoWakeUp = DISABLE;
//  hcan1.Init.AutoRetransmission = DISABLE;
//  hcan1.Init.ReceiveFifoLocked = DISABLE;
//  hcan1.Init.TransmitFifoPriority = DISABLE;
//
//  if (HAL_CAN_Init(&hcan1) != HAL_OK)
//    Error_Handler();
//}
//
//static void MX_I2C1_Init(void)
//{
//  __HAL_RCC_I2C1_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
//  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//  hi2c1.Instance = I2C1;
//  hi2c1.Init.ClockSpeed = 100000;
//  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
//  hi2c1.Init.OwnAddress1 = 0;
//  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//  hi2c1.Init.OwnAddress2 = 0;
//  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
//    Error_Handler();
//}
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */
//}


//#include "main.h"
//#include<string.h>
//#include<stdio.h>
//CAN_HandleTypeDef hcan2;
//UART_HandleTypeDef huart5;
//
//uint8_t led_c = 0;
//
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_CAN2_Init(void);
//static void MX_UART5_Init(void);
//void can_filter_config(void);
//
//int main(void)
//{
//
//  HAL_Init();
//
//  SystemClock_Config();
//
//  MX_GPIO_Init();
//  MX_UART5_Init();
//  MX_CAN2_Init();
//  can_filter_config();
//  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
//  HAL_CAN_Start(&hcan2);
//
//  while (1)
//  {
//	  /*if(led_c==1){
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
//	  }else if(led_c==2){
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
//	  }else if(led_c==3){
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
//	  }else if(led_c == 4){
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15,0);
//	  }else{
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
//	  }*/
//
//  }
//
//}
//
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage
//  */
//  __HAL_RCC_PWR_CLK_ENABLE();
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 4;
//  RCC_OscInitStruct.PLL.PLLN = 50;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//  RCC_OscInitStruct.PLL.PLLQ = 7;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
//
//static void MX_CAN2_Init(void)
//{
//
//  hcan2.Instance = CAN2;
//  hcan2.Init.Prescaler = 5;
//  hcan2.Init.Mode = CAN_MODE_NORMAL;
//  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
//  hcan2.Init.TimeSeg1 = CAN_BS1_8TQ;
//  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
//  hcan2.Init.TimeTriggeredMode = DISABLE;
//  hcan2.Init.AutoBusOff = DISABLE;
//  hcan2.Init.AutoWakeUp = DISABLE;
//  hcan2.Init.AutoRetransmission = DISABLE;
//  hcan2.Init.ReceiveFifoLocked = DISABLE;
//  hcan2.Init.TransmitFifoPriority = DISABLE;
//  if (HAL_CAN_Init(&hcan2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN CAN2_Init 2 */
//
//  /* USER CODE END CAN2_Init 2 */
//
//}
//
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
//
//	   CAN_RxHeaderTypeDef rx;
//		uint8_t Rx_data[8];
//
//		char msg[50];
//
//		HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx, Rx_data);
//
//		sprintf(msg,"Received msg: %s \r\n",Rx_data);
//		HAL_UART_Transmit(&huart5, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//		led_c++;
//		if(led_c==1){
//				  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
//			  }else if(led_c==2){
//				  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
//			  }else if(led_c==3){
//				  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
//			  }else if(led_c == 4){
//				  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15,0);
//			  }else{
//				  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
//				  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
//			  }
//}
//
//void can_filter_config(void){
//
//	CAN_FilterTypeDef can_filter;
//	can_filter.FilterActivation = ENABLE;
//	can_filter.FilterBank = 0;
//	can_filter.FilterFIFOAssignment = CAN_FilterFIFO0;
//	can_filter.FilterIdHigh = 0x0000;
//	can_filter.FilterIdLow = 0x0000;
//	can_filter.FilterMaskIdHigh = 0x0000;
//	can_filter.FilterMaskIdLow = 0x0000;
//	can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
//	can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
//
//	HAL_CAN_ConfigFilter(&hcan2, &can_filter);
//
//}
//
//static void MX_UART5_Init(void)
//{
//
//  /* USER CODE BEGIN UART5_Init 0 */
//
//  /* USER CODE END UART5_Init 0 */
//
//  /* USER CODE BEGIN UART5_Init 1 */
//
//  /* USER CODE END UART5_Init 1 */
//  huart5.Instance = UART5;
//  huart5.Init.BaudRate = 115200;
//  huart5.Init.WordLength = UART_WORDLENGTH_8B;
//  huart5.Init.StopBits = UART_STOPBITS_1;
//  huart5.Init.Parity = UART_PARITY_NONE;
//  huart5.Init.Mode = UART_MODE_TX_RX;
//  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
//  if (HAL_UART_Init(&huart5) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN UART5_Init 2 */
//
//  /* USER CODE END UART5_Init 2 */
//
//}
//
//
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOH_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//  __HAL_RCC_GPIOD_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin : PD14 */
//  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
//
//}
//
//
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
