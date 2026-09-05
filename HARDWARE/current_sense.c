/* current_sense.c - phase-current acquisition, dq feedback and current loop
 *
 * The ADC injection conversion is hardware-triggered by TIM1.  Center-aligned
 * PWM produces two update events per period, so only one counting direction is
 * accepted to keep the effective current-loop rate at the PWM rate (20 kHz).
 *
 * Two low-side phases (A/B) are sampled and C is reconstructed.  The sampled
 * polarity is inverted because the sense chain is opposite to the FOC sign
 * convention used by the transforms.
 */
#include "current_sense.h"

#include <arm_math.h>
#include <math.h>

#include "adc.h"
#include "angle_encoder.h"
#include "foc_config.h"
#include "lowpass_filter.h"
#include "motor_controller.h"
#include "pid_regulator.h"
#include "svpwm_driver.h"

volatile float PhaseA_A = 0.0f;
volatile float PhaseB_A = 0.0f;
volatile float PhaseC_A = 0.0f;

float IAlpha = 0.0f;
float IBeta = 0.0f;
float Id = 0.0f;
float Iq = 0.0f;
float FilterId = 0.0f;
float FilterIq = 0.0f;

volatile uint8_t CurrentLoopEnable = 0u;
volatile uint8_t VoltageTestEnable = 0u;
volatile float IdRef = 0.0f;
volatile float IqRef = 0.0f;
volatile float UdTest = 0.1f;
volatile float UqTest = 0.0f;

volatile float IdFilterAlpha = 0.1f;
volatile float IqFilterAlpha = 0.1f;

volatile uint32_t AdcInjectedCount = 0u;

/* Offset calibration state. */
static volatile uint8_t calibrating = 0u;
static volatile uint8_t calibrated = 0u;
static volatile uint32_t offset_samples = 0u;
static uint32_t offset_sum_a = 0u;
static uint32_t offset_sum_b = 0u;
static volatile float offset_a_adc = 0.0f;
static volatile float offset_b_adc = 0.0f;
static volatile float offset_a_v = 1.65f;
static volatile float offset_b_v = 1.65f;

static float RawToAmps(uint16_t adc_val, float offset_v)
{
  float volts = (float)adc_val * FOC_ADC_VREF / FOC_ADC_FS - offset_v;
  return volts / (FOC_SHUNT_OHM * FOC_CSA_GAIN);
}

void CurrentSense_StartOffsetCalibration(void)
{
  __disable_irq();
  offset_sum_a = 0u;
  offset_sum_b = 0u;
  offset_samples = 0u;
  calibrated = 0u;
  calibrating = 1u;
  __enable_irq();
}

uint8_t CurrentSense_IsOffsetCalibrated(void)
{
  return (uint8_t)calibrated;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance != ADC1)
  {
    return;
  }

  AdcInjectedCount++;

  /* Keep only one counting direction from the two update events per period. */
  if ((TIM1->CR1 & TIM_CR1_DIR) == 0u)
  {
    return;
  }

  uint16_t ia = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
  uint16_t ib = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);

  if (calibrating != 0u)
  {
    offset_sum_a += ia;
    offset_sum_b += ib;
    offset_samples++;

    if (offset_samples >= FOC_OFFSET_SAMPLES)
    {
      offset_a_adc = (float)offset_sum_a / (float)FOC_OFFSET_SAMPLES;
      offset_b_adc = (float)offset_sum_b / (float)FOC_OFFSET_SAMPLES;
      offset_a_v = offset_a_adc * FOC_ADC_VREF / FOC_ADC_FS;
      offset_b_v = offset_b_adc * FOC_ADC_VREF / FOC_ADC_FS;

      PhaseA_A = 0.0f;
      PhaseB_A = 0.0f;
      PhaseC_A = 0.0f;
      IAlpha = 0.0f;
      IBeta = 0.0f;
      Id = 0.0f;
      Iq = 0.0f;
      FilterId = 0.0f;
      FilterIq = 0.0f;

      calibrating = 0u;
      calibrated = 1u;
    }
    return;
  }

  /* Sense polarity is opposite to the FOC phase-current convention. */
  PhaseA_A = -RawToAmps(ia, offset_a_v);
  PhaseB_A = -RawToAmps(ib, offset_b_v);
  PhaseC_A = -(PhaseA_A + PhaseB_A);

  arm_clarke_f32(PhaseA_A, PhaseB_A, &IAlpha, &IBeta);

  float sin_t = arm_sin_f32(ElecAngleRad);
  float cos_t = arm_cos_f32(ElecAngleRad);
  arm_park_f32(IAlpha, IBeta, &Id, &Iq, sin_t, cos_t);

  FilterId = Lowpass_Next(Id, FilterId, IdFilterAlpha);
  FilterIq = Lowpass_Next(Iq, FilterIq, IqFilterAlpha);

  if (CurrentLoopEnable != 0u)
  {
    float vd = PidReg_Update(&TorquePidD, IdRef - FilterId);
    float vq = PidReg_Update(&TorquePidQ, IqRef - FilterIq);

    float du, dv, dw;
    Svpwm_Apply(vd, vq, ElecAngleRad, &du, &dv, &dw);
    Pwm_WritePhases(du, dv, dw);
  }
  else if (VoltageTestEnable != 0u)
  {
    float du, dv, dw;
    Svpwm_Apply(UdTest, UqTest, 0.0f, &du, &dv, &dw);
    Pwm_WritePhases(du, dv, dw);
  }
}
