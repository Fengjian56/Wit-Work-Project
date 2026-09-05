/* svpwm_driver.c - SVPWM synthesis and PWM compare write
 *
 * The duty generation is a seven-segment SVPWM in the normalized range
 * [0,1]. Phase mapping follows the physical half-bridge wiring, so writing
 * "phase A" updates the TIM1 channel that is actually connected to phase A.
 */
#include "svpwm_driver.h"

#include <arm_math.h>
#include <math.h>
#include <stdbool.h>
#include "main.h"
#include "tim.h"

#ifndef M_SQRT3
#define M_SQRT3 1.73205080757f
#endif

#define DUTY_MIN 0.0f
#define DUTY_MAX 0.95f
#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

void Svpwm_Apply(float ud, float uq, float theta_rad,
                 float *du, float *dv, float *dw)
{
  /* Keep modulation inside the linear region. */
  ud = CLAMP(ud, -1.0f, 1.0f);
  uq = CLAMP(uq, -1.0f, 1.0f);

  /* Inverse Park: dq -> alpha/beta. */
  float sin_t = arm_sin_f32(theta_rad);
  float cos_t = arm_cos_f32(theta_rad);
  float alpha = cos_t * ud - sin_t * uq;
  float beta  = sin_t * ud + cos_t * uq;

  /* Sector selection from alpha/beta signs. */
  bool sA = beta > 0.0f;
  bool sB = fabsf(beta) > M_SQRT3 * fabsf(alpha);
  bool sC = alpha > 0.0f;
  uint8_t idx = (uint8_t)(4u * sA + 2u * sB + 1u * sC);

  /* Map ternary index to one of six 60-degree sectors (1..6). */
  static const uint8_t map[8] = {4u, 6u, 5u, 5u, 3u, 1u, 2u, 2u};
  uint8_t sector = map[idx];

  /* Active-vector dwell times in units of the PWM period. */
  const float rad60 = 60.0f * PI / 180.0f;
  float tm = arm_sin_f32((float)sector * rad60) * alpha
           - arm_cos_f32((float)sector * rad60) * beta;
  float tn = beta * arm_cos_f32(((float)sector - 1.0f) * rad60)
           - alpha * arm_sin_f32(((float)sector - 1.0f) * rad60);
  float t0 = 1.0f - tm - tn;

  /* Basic vectors per sector: 100,110,010,011,001,101. */
  static const uint8_t v[6][3] = {
      {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
      {0, 1, 1}, {0, 0, 1}, {1, 0, 1}};

  uint8_t prev = (uint8_t)((sector == 1u) ? 5u : sector - 1u);
  uint8_t next = (uint8_t)(sector % 6u);

  *du = (float)v[prev][0] * tm + (float)v[next][0] * tn + t0 * 0.5f;
  *dv = (float)v[prev][1] * tm + (float)v[next][1] * tn + t0 * 0.5f;
  *dw = (float)v[prev][2] * tm + (float)v[next][2] * tn + t0 * 0.5f;
}

void Pwm_WritePhases(float du, float dv, float dw)
{
  du = CLAMP(du, DUTY_MIN, DUTY_MAX);
  dv = CLAMP(dv, DUTY_MIN, DUTY_MAX);
  dw = CLAMP(dw, DUTY_MIN, DUTY_MAX);

  /* Physical wiring: TIM1_CH1 -> C, TIM1_CH2 -> B, TIM1_CH3 -> A.
   * Keep the three compare writes atomic with respect to ISRs. */
  __disable_irq();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(dw * (float)TIM1->ARR));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(dv * (float)TIM1->ARR));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(du * (float)TIM1->ARR));
  __enable_irq();
}
