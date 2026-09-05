/* telemetry.c - VOFA+ JustFloat six-channel streaming over USART1 */
#include "telemetry.h"

#include <string.h>
#include "usart.h"

void Telemetry_SendFrame(const float channels[6])
{
  uint8_t frame[6u * sizeof(float) + 4u];

  memcpy(frame, channels, 6u * sizeof(float));
  frame[24u] = 0x00u;
  frame[25u] = 0x00u;
  frame[26u] = 0x80u;
  frame[27u] = 0x7Fu;

  HAL_UART_Transmit(&huart1, frame, sizeof(frame), 10u);
}
