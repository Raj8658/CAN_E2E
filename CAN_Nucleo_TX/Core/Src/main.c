#include "main.h"
#include <string.h>

CAN_HandleTypeDef hcan2;

// E2E Configuration
#define MAX_COUNTER 0xFF
uint8_t e2e_counter = 0;
uint8_t transmission_count = 0;

// Keypad Buffer
char input_buffer[5];  // 4 chars + null terminator
uint8_t input_index = 0;

// Checksum Configuration
#define BLOCK_SIZE 4  // 4-bit blocks for checksum

uint8_t calculate_checksum(uint8_t* data, uint8_t len);

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN2_Init(void);
void CAN_Tx(uint8_t* data, uint8_t len);
char Keypad_Scan(void);

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_CAN2_Init();
  HAL_CAN_Start(&hcan2);

  while (1) {
    char key = Keypad_Scan();
    if (key != 0) {
      if (key == '#') {
        if (input_index > 0) {
          // Prepare payload: [counter | data | checksum]
          uint8_t payload[8] = {0};
          uint8_t data_len = input_index;

          // Counter logic
          if(transmission_count < 2) {
            e2e_counter = (e2e_counter + 1) % MAX_COUNTER;
            transmission_count++;
          }
          // On 3rd transmission, keep previous counter

          payload[0] = e2e_counter;
          memcpy(&payload[1], input_buffer, data_len);

          // Calculate checksum for (counter + data)
          uint8_t checksum = calculate_checksum(payload, data_len + 1);
          payload[data_len + 1] = checksum;

          CAN_Tx(payload, data_len + 2);

          input_index = 0;
          memset(input_buffer, 0, sizeof(input_buffer));
        }
      }
      else if (input_index < 4) {
        input_buffer[input_index++] = key;
      }
    }
    HAL_Delay(100);
  }
}

uint8_t calculate_checksum(uint8_t* data, uint8_t len) {
  uint32_t sum = 0;
  uint8_t num_blocks = (len * 8) / BLOCK_SIZE;

  // Process 4-bit blocks
  for(uint8_t i=0; i<num_blocks; i++) {
    uint8_t block = 0;
    uint8_t byte_pos = i / 2;
    uint8_t bit_shift = (i % 2) * 4;

    block = (data[byte_pos] >> bit_shift) & 0x0F;
    sum += block;
  }

  // Add carry
  while(sum >> 4) {
    sum = (sum & 0x0F) + (sum >> 4);
  }

  // One's complement
  return (~sum) & 0x0F;
}

void CAN_Tx(uint8_t* data, uint8_t len) {
  CAN_TxHeaderTypeDef tx_header = {
    .StdId = 0x666,
    .RTR = CAN_RTR_DATA,
    .IDE = CAN_ID_STD,
    .DLC = len,
    .TransmitGlobalTime = DISABLE
  };

  uint32_t mailbox;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, data, &mailbox);
}
