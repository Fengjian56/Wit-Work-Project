/* foc_app.c - application orchestration on top of the HAL skeleton
 *
 * Responsibilities:
 *  - start the injected ADC and encoder DMA chains
 *  - run an open-loop electrical alignment and record the encoder zero
 *  - calibrate current offsets with the power stage in a known zero state
 *  - initialise PID/MIT regulators
 *  - run the selected test trajectory and stream VOFA+ telemetry
 */
#include "foc_app.h"

#include <math.h>
#include <arm_math.h>

#include "adc.h"
#include "angle_encoder.h"
#include "current_sense.h"
#include "dma.h"
#include "foc_config.h"
#include "gpio.h"
#include "main.h"
#include "motor_controller.h"
#include "pid_regulator.h"
#include "spi.h"
#include "svpwm_driver.h"
#include "telemetry.h"
#include "tim.h"
#include "usart.h"

/* ------------------------------------------------------------------------- */
/* Test-mode selection (edit FOC_APP_MODE to switch).                        */
/* ------------------------------------------------------------------------- */
#define FOC_MODE_CURRENT 1u
#define FOC_MODE_SPEED_CURRENT 2u
#define FOC_MODE_POSITION_SPEED_CURRENT 3u
#define FOC_MODE_MIT 4u

#define FOC_APP_MODE FOC_MODE_MIT

/* Regular-group DMA buffer: bus voltage and temperature probes. */
uint16_t AdcRegularRaw[3] = {0u, 0u, 0u};

static void OpenLoop_Align(void)
{
  float theta;
  float du, dv, dw;
  const float v_mag = 0.3f;

  /* Sweep a rotating voltage vector to pull the rotor into electrical lock. */
  for (theta = 0.0f; theta < 3.0f * PI; theta += 0.1f)
  {
    Svpwm_Apply(0.0f, v_mag, theta, &du, &dv, &dw);
    Pwm_WritePhases(du, dv, dw);
    HAL_Delay(10);
  }

  /* Return slowly and reduce magnitude to settle without overshoot. */
  for (theta = 3.0f * PI; theta > 0.0f; theta -= 0.1f)
  {
    Svpwm_Apply(0.0f, v_mag * 0.5f, theta, &du, &dv, &dw);
    Pwm_WritePhases(du, dv, dw);
    HAL_Delay(10);
  }

  /* Hold d-axis voltage to lock the rotor at the electrical zero. */
  Svpwm_Apply(v_mag, 0.0f, 0.0f, &du, &dv, &dw);
  Pwm_WritePhases(du, dv, dw);
  HAL_Delay(800);

  EncoderZero = RawElecAngleRad;

  Pwm_WritePhases(0.0f, 0.0f, 0.0f);
}

static void WaitForOffsetCalibration(void)
{
  uint32_t start = HAL_GetTick();
  while (CurrentSense_IsOffsetCalibrated() == 0u)
  {
    if ((HAL_GetTick() - start) > 100u)
    {
      Pwm_WritePhases(0.0f, 0.0f, 0.0f);
      Error_Handler();
    }
  }
}

static void StartInjectedAdc(void)
{
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_InjectedStart_IT(&hadc1);
}

static void StartPwmChannels(void)
{
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

static void InitRegulators(void)
{
  PidReg_Init(&TorquePidD,
              2.0f, 0.005f, 0.0f,
              0.60f, -0.60f,
              1500.0f, -500.0f);
  PidReg_Init(&TorquePidQ,
              2.0f, 0.005f, 0.0f,
              0.60f, -0.60f,
              1500.0f, -1500.0f);
  PidReg_Init(&SpeedPid,
              0.5f, 0.0001f, 0.0f,
              0.5f, -0.5f,
              500.0f, -500.0f);
  PidReg_Init(&PositionPid,
              0.1f, 0.00001f, 0.0f,
              0.8f, -0.8f,
              3000.0f, -3000.0f);
  MitReg_Init(&MitCtrl,
              0.05f, 0.02f,
              0.50f, -0.50f,
              0.0f);
}

static void ConfigureLoopEnables(void)
{
  CurrentLoopEnable = 1u;
  VoltageTestEnable = 0u;

#if FOC_APP_MODE == FOC_MODE_CURRENT
  SpeedLoopEnable = 0u;
  PositionLoopEnable = 0u;
  MitLoopEnable = 0u;
#elif FOC_APP_MODE == FOC_MODE_SPEED_CURRENT
  SpeedLoopEnable = 1u;
  PositionLoopEnable = 0u;
  MitLoopEnable = 0u;
#elif FOC_APP_MODE == FOC_MODE_POSITION_SPEED_CURRENT
  SpeedLoopEnable = 1u;
  PositionLoopEnable = 1u;
  MitLoopEnable = 0u;
#else
  SpeedLoopEnable = 0u;
  PositionLoopEnable = 0u;
  MitLoopEnable = 1u;
#endif

  IdRef = 0.0f;
  IqRef = 0.0f;
  SpeedRef = 0.0f;
  PositionTargetDeg = MotorAngleDeg;
  PositionReferenceDeg = MotorAngleDeg;
  MitPositionTargetDeg = MotorAngleDeg;
  MitSpeedRef = 0.0f;
}

void FocApp_Init(void)
{
  StartInjectedAdc();
  AngleEncoder_InitCapture();
  HAL_Delay(10);

  Pwm_WritePhases(0.0f, 0.0f, 0.0f);
  StartPwmChannels();

  /* Let the sense amplifiers settle, then calibrate offsets. */
  HAL_Delay(20);
  CurrentSense_StartOffsetCalibration();
  WaitForOffsetCalibration();

  OpenLoop_Align();

  /* Recalibrate with the real powered-on operating point. */
  Pwm_WritePhases(0.0f, 0.0f, 0.0f);
  HAL_Delay(50);
  CurrentSense_StartOffsetCalibration();
  WaitForOffsetCalibration();
  HAL_Delay(10);

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)AdcRegularRaw, 3u);

  InitRegulators();
  ConfigureLoopEnables();
}

#if FOC_APP_MODE == FOC_MODE_CURRENT
static void UpdateCurrentTestTarget(uint32_t tick)
{
  static uint32_t start = 0u;
  if (start == 0u)
  {
    start = tick;
  }
  uint32_t t = (tick - start) % 8000u;
  IqRef = (t < 4000u) ? 0.3f : -0.3f;
}
#endif

#if FOC_APP_MODE == FOC_MODE_SPEED_CURRENT
static void UpdateSpeedTestTarget(uint32_t tick)
{
  static uint32_t start = 0u;
  if (start == 0u)
  {
    start = tick;
  }
  uint32_t t = (tick - start) % 8000u;
  float phase = 2.0f * PI * (float)t / 8000.0f;
  SpeedRef = 2.0f * arm_sin_f32(phase);
}
#endif

#if FOC_APP_MODE == FOC_MODE_MIT
static void UpdateMitTestTarget(uint32_t tick)
{
  static uint32_t start = 0u;
  static float origin_deg = 0.0f;
  if (start == 0u)
  {
    start = tick;
    origin_deg = MotorAngleDeg;
  }

  uint32_t t = (tick - start) % 16000u;
  float phase = 2.0f * PI * (float)t / 16000.0f;
  MitPositionTargetDeg = origin_deg + 5.0f * arm_sin_f32(phase);
  MitSpeedRef = (5.0f * PI / 180.0f) * (2.0f * PI / 16.0f)
              * arm_cos_f32(phase);
}
#endif

#if FOC_APP_MODE == FOC_MODE_POSITION_SPEED_CURRENT
static void UpdatePositionTestTarget(uint32_t tick)
{
  static uint32_t start = 0u;
  static float origin_deg = 0.0f;
  if (start == 0u)
  {
    start = tick;
    origin_deg = MotorAngleDeg;
  }

  uint32_t t = (tick - start) % 8000u;
  float phase = 2.0f * PI * (float)t / 8000.0f;
  PositionTargetDeg = origin_deg + 45.0f * arm_sin_f32(phase);
}
#endif

void FocApp_Tick(void)
{
  uint32_t tick = HAL_GetTick();

#if FOC_APP_MODE == FOC_MODE_CURRENT
  UpdateCurrentTestTarget(tick);
#elif FOC_APP_MODE == FOC_MODE_SPEED_CURRENT
  UpdateSpeedTestTarget(tick);
#elif FOC_APP_MODE == FOC_MODE_POSITION_SPEED_CURRENT
  UpdatePositionTestTarget(tick);
#else
  UpdateMitTestTarget(tick);
#endif

  static uint32_t vofa_tick = 0u;
  if ((HAL_GetTick() - vofa_tick) >= 10u)
  {
    vofa_tick = HAL_GetTick();

#if FOC_APP_MODE == FOC_MODE_CURRENT
    float ch[6] = {
        IqRef, FilterIq, TorquePidQ.Output,
        IdRef, FilterId, TorquePidD.Output};
#elif FOC_APP_MODE == FOC_MODE_SPEED_CURRENT
    float ch[6] = {
        SpeedRef, FilteredSpeed, IqRef,
        FilterIq, FilterId, TorquePidQ.Output};
#elif FOC_APP_MODE == FOC_MODE_POSITION_SPEED_CURRENT
    float ch[6] = {
        PositionTargetDeg, MotorAngleDeg,
        PositionTargetDeg - MotorAngleDeg,
        SpeedRef, SpeedPid.Integrator, IqRef};
#else
    float ch[6] = {
        MitPositionTargetDeg, MotorAngleDeg,
        MitPositionTargetDeg - MotorAngleDeg,
        IqRef, FilterIq, FilteredSpeed};
#endif
    Telemetry_SendFrame(ch);
  }
  HAL_Delay(1);
}
