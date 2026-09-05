/* motor_controller.c - TIM2 1 kHz scheduling of speed/MIT/position loops
 *
 * Computes speed from the unwrapped motor angle, applies a one-pole filter,
 * and then executes either the MIT law or the cascaded position->speed
 * structure.  Output is an Iq reference consumed by the ADC current loop.
 */
#include "motor_controller.h"

#include <arm_math.h>
#include "adc.h"
#include "angle_encoder.h"
#include "current_sense.h"
#include "foc_config.h"
#include "lowpass_filter.h"
#include "pid_regulator.h"

volatile float FilteredSpeed = 0.0f;
volatile float SpeedFilterAlpha = 0.05f;

PidReg_t TorquePidD;
PidReg_t TorquePidQ;
PidReg_t SpeedPid;
PidReg_t PositionPid;
MitReg_t MitCtrl;

volatile float SpeedRef = 0.0f;
volatile float PositionTargetDeg = 0.0f;
volatile float PositionReferenceDeg = 0.0f;
static uint8_t position_ref_initialized = 0u;
static uint8_t position_divider = 0u;

volatile float MitPositionTargetDeg = 0.0f;
volatile float MitSpeedRef = 0.0f;

volatile uint8_t SpeedLoopEnable = 0u;
volatile uint8_t PositionLoopEnable = 0u;
volatile uint8_t MitLoopEnable = 0u;

void MotorControl_1kHzTick(void)
{
  static float last_enc_rad = 0.0f;
  static uint8_t first = 1u;

  if (first)
  {
    first = 0u;
    last_enc_rad = MotorAngleDeg * PI / 180.0f;
  }

  float now_rad = MotorAngleDeg * PI / 180.0f;
  float diff = now_rad - last_enc_rad;
  last_enc_rad = now_rad;

  /* Correct wrap around +/- 180 deg. */
  if (diff > PI)
  {
    diff -= 2.0f * PI;
  }
  else if (diff < -PI)
  {
    diff += 2.0f * PI;
  }

  float speed_rad_s = diff / FOC_TIM2_PERIOD_S;
  FilteredSpeed = Lowpass_Next(speed_rad_s, FilteredSpeed, SpeedFilterAlpha);

  if (MitLoopEnable != 0u)
  {
    float pos_err = MitPositionTargetDeg - MotorAngleDeg;
    float spd_err = MitSpeedRef - FilteredSpeed;
    IqRef = MitReg_Update(&MitCtrl, pos_err, spd_err);
    return;
  }

  if (PositionLoopEnable != 0u)
  {
    position_divider++;
    if (position_divider >= FOC_POSITION_DIVIDER)
    {
      position_divider = 0u;

      if (position_ref_initialized == 0u)
      {
        PositionReferenceDeg = MotorAngleDeg;
        position_ref_initialized = 1u;
      }

      float err = PositionTargetDeg - PositionReferenceDeg;
      float max_step = FOC_RAMP_SPEED_DEG_S *
                       (FOC_TIM2_PERIOD_S * (float)FOC_POSITION_DIVIDER);
      if (err > max_step)
      {
        err = max_step;
      }
      else if (err < -max_step)
      {
        err = -max_step;
      }
      PositionReferenceDeg += err;

      /* Position loop outputs a speed reference in rad/s. */
      SpeedRef = PidReg_Update(&PositionPid, PositionReferenceDeg - MotorAngleDeg);
    }
  }

  if (SpeedLoopEnable != 0u)
  {
    IqRef = PidReg_Update(&SpeedPid, SpeedRef - FilteredSpeed);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    MotorControl_1kHzTick();
  }
}
