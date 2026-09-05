/* angle_encoder.c - MT6701 dual-encoder DMA interface
 *
 * The motor-side encoder is read over SPI3 and the reducer-side over SPI1.
 * SPI is serviced by DMA; this module owns the SPI completion callbacks and
 * unwraps each single-turn angle into a continuous position.  The motor-side
 * single-turn angle also drives the electrical angle for FOC.
 */
#include "angle_encoder.h"

#include <math.h>
#include <arm_math.h>

#include "main.h"
#include "spi.h"
#include "foc_config.h"

#define MT6701_RESOLUTION (1 << 14)
#define MT6701_FRAME_LEN 3u

uint8_t MotorEncFrame[3] = {0u, 0u, 0u};
uint8_t OutputEncFrame[3] = {0u, 0u, 0u};

volatile float MotorAngleDeg = 0.0f;
volatile float OutputAngleDeg = 0.0f;
volatile float ElecAngleRad = 0.0f;
volatile float RawElecAngleRad = 0.0f;
volatile float RawOutputAngleRad = 0.0f;

volatile float EncoderZero = 0.0f;

/* Convert a signed radian difference into [-pi, pi]. */
static float WrapPi(float diff)
{
  if (diff < -PI)
  {
    diff += 2.0f * PI;
  }
  else if (diff > PI)
  {
    diff -= 2.0f * PI;
  }
  return diff;
}

void AngleEncoder_InitCapture(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | MT6701_CSN_Pin, GPIO_PIN_RESET);

  HAL_SPI_TransmitReceive_DMA(&hspi3,
                              MotorEncFrame,
                              MotorEncFrame,
                              MT6701_FRAME_LEN);

  HAL_SPI_TransmitReceive_DMA(&hspi1,
                              OutputEncFrame,
                              OutputEncFrame,
                              MT6701_FRAME_LEN);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI3)
  {
    static float last_motor_rad = 0.0f;
    static uint8_t first = 1u;

    HAL_GPIO_WritePin(GPIOA, MT6701_CSN_Pin, GPIO_PIN_SET);

    uint16_t raw =
        (uint16_t)((MotorEncFrame[0] << 6) | (MotorEncFrame[1] >> 2));
    float single_turn_rad = 2.0f * PI * (float)raw / (float)MT6701_RESOLUTION;

    /* Keep the electrical angle continuous over 0/2pi wrap using the raw
     * single-turn angle minus the alignment offset. */
    RawElecAngleRad = single_turn_rad;
    float wrapped = WrapPi(single_turn_rad - EncoderZero);
    float elec = fmodf(wrapped * (float)FOC_POLE_PAIRS, 2.0f * PI);
    if (elec < 0.0f)
    {
      elec += 2.0f * PI;
    }
    ElecAngleRad = elec;

    if (first)
    {
      first = 0u;
      last_motor_rad = single_turn_rad;
    }
    float delta = WrapPi(single_turn_rad - last_motor_rad);
    last_motor_rad = single_turn_rad;

    static float motor_accum_rad = 0.0f;
    motor_accum_rad += delta;
    MotorAngleDeg = motor_accum_rad * 180.0f / PI;

    /* Re-arm motor-side capture. */
    HAL_GPIO_WritePin(GPIOA, MT6701_CSN_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(&hspi3,
                                MotorEncFrame,
                                MotorEncFrame,
                                MT6701_FRAME_LEN);
  }
  else if (hspi->Instance == SPI1)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    uint16_t raw =
        (uint16_t)((OutputEncFrame[0] << 6) | (OutputEncFrame[1] >> 2));
    float single_turn_rad = 2.0f * PI * (float)raw / (float)MT6701_RESOLUTION;
    RawOutputAngleRad = single_turn_rad;
    OutputAngleDeg = single_turn_rad * 180.0f / PI;

    /* Re-arm output-side capture. */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(&hspi1,
                                OutputEncFrame,
                                OutputEncFrame,
                                MT6701_FRAME_LEN);
  }
}
