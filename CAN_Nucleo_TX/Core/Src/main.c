#include "main.h"
#include <string.h>

CAN_HandleTypeDef hcan2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN2_Init(void);
void filter_config(void);
void CAN_TX(uint8_t* data, uint8_t len);
char Keypad_Scan(void);
void Process_Keypad(void);
uint8_t calculate_crc(uint8_t* data, uint8_t len);

char input_buffer[8];
uint8_t input_index = 0;
uint8_t e2e_counter = 0;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_CAN2_Init();
  filter_config();
  HAL_CAN_Start(&hcan2);

  while (1)
  {
    Process_Keypad();
    HAL_Delay(100);
  }
}

void CAN_TX(uint8_t* data, uint8_t len)
{
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox;

  TxHeader.DLC = len;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x666;

  if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &TxMailbox) != HAL_OK)
  {
    Error_Handler();
  }
}

void Process_Keypad(void)
{
  char key = Keypad_Scan();
  if (key != 0)
  {
    if (key == '#')
    {
      if (input_index > 0 && input_index <= 5)
      {
        uint8_t payload[8] = {0};
        payload[0] = e2e_counter++;
        memcpy(&payload[1], input_buffer, input_index);
        payload[input_index + 1] = calculate_crc(payload, input_index + 1);
        CAN_TX(payload, input_index + 2);
        input_index = 0;
        memset(input_buffer, 0, sizeof(input_buffer));
      }
    }
    else if (input_index < 6)
    {
      input_buffer[input_index++] = key;
    }
  }
}

char Keypad_Scan(void)
{
  const uint8_t row_pins[4] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3};
  const uint8_t col_pins[4] = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7};
  const char keys[4][4] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
  };

  for (int row = 0; row < 4; row++)
  {
    for (int r = 0; r < 4; r++)
      HAL_GPIO_WritePin(GPIOB, row_pins[r], GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, row_pins[row], GPIO_PIN_RESET);

    for (int col = 0; col < 4; col++)
    {
      if (HAL_GPIO_ReadPin(GPIOB, col_pins[col]) == GPIO_PIN_RESET)
      {
        HAL_Delay(20);
        while (HAL_GPIO_ReadPin(GPIOB, col_pins[col]) == GPIO_PIN_RESET);
        return keys[row][col];
      }
    }
  }
  return 0;
}

uint8_t calculate_crc(uint8_t* data, uint8_t len)
{
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++)
    {
      if (crc & 0x80) crc = (crc << 1) ^ 0x1D;
      else crc <<= 1;
    }
  }
  return crc;
}

void filter_config(void)
{
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
}

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  // PA5 LED
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Keypad ROWS: PB0-PB3 Output
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Keypad COLS: PB4-PB7 Input Pull-Up
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}





//#include "main.h"
//
//CAN_HandleTypeDef hcan2;
//
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_CAN2_Init(void);
//void CAN_TX(void);
//void filter_config(void);
//
//int main(void)
//{
//  HAL_Init();
//  SystemClock_Config();
//  MX_GPIO_Init();
//  MX_CAN2_Init();
//  filter_config();
//  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);
//  HAL_CAN_Start(&hcan2);
//
//  while (1)
//  {
//
//  }
//}
//
//
//
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage
//  */
//  __HAL_RCC_PWR_CLK_ENABLE();
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//  RCC_OscInitStruct.PLL.PLLM = 8;
//  RCC_OscInitStruct.PLL.PLLN = 50;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//  RCC_OscInitStruct.PLL.PLLQ = 2;
//  RCC_OscInitStruct.PLL.PLLR = 2;
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
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
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
//
//}
//
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin : PC13 */
//  GPIO_InitStruct.Pin = GPIO_PIN_13;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : PA5 */
//  GPIO_InitStruct.Pin = GPIO_PIN_5;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//  /* EXTI interrupt init*/
//  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
//
//}
//
//void CAN_TX(void){
//
//	   uint32_t Tx_mailBox;
//       CAN_TxHeaderTypeDef Tx;
//
//       Tx.DLC = 5;
//       Tx.IDE = CAN_ID_STD;
//       Tx.RTR = CAN_RTR_DATA;
//       Tx.StdId = 0x666;
//
//       uint8_t data[5] = {'H','L','L','E','O'};
//
//       if (HAL_CAN_AddTxMessage(&hcan2, &Tx, data, &Tx_mailBox)!=HAL_OK){
//
//    	   Error_Handler();
//       }
//
//}
//
//void filter_config(void){
//
//	CAN_FilterTypeDef filter;
//	filter.FilterActivation = CAN_FILTER_ENABLE;
//	filter.FilterBank  = 0;
//	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//	filter.FilterIdHigh = 0x0000;
//	filter.FilterIdLow = 0x0000;
//	filter.FilterMaskIdHigh = 0x0000;
//	filter.FilterMaskIdLow = 0x0000;
//	filter.FilterMode = CAN_FILTERMODE_IDMASK;
//	filter.FilterScale = CAN_FILTERSCALE_32BIT;
//
//	HAL_CAN_ConfigFilter(&hcan2, &filter);
//}
//
//void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
//}
//
//void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
//}
//
//void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
//}
//
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//	CAN_TX();
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
