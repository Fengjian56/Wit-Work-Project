/* pid_regulator.c - generic PID and MIT/PD controller
 *
 * PID uses bounded integration and output clamping.  The MIT block is a
 * PD position/speed law: it maps position error (deg) and speed error
 * (rad/s) into an Iq reference.
 */
#include "pid_regulator.h"

void PidReg_Init(PidReg_t *r,
                 float kp, float ki, float kd,
                 float out_max, float out_min,
                 float integ_max, float integ_min)
{
  r->Kp = kp;
  r->Ki = ki;
  r->Kd = kd;
  r->OutMax = out_max;
  r->OutMin = out_min;
  r->IntegratorMax = integ_max;
  r->IntegratorMin = integ_min;

  r->Error = 0.0f;
  r->LastError = 0.0f;
  r->ErrorSum = 0.0f;
  r->Integrator = 0.0f;
  r->Derivative = 0.0f;
  r->Output = 0.0f;
}

float PidReg_Update(PidReg_t *r, float err)
{
  r->Error = err;

  /* Bounded error accumulation (position-form PID anti-windup). */
  r->ErrorSum += r->Error;
  if (r->ErrorSum > r->IntegratorMax)
  {
    r->ErrorSum = r->IntegratorMax;
  }
  else if (r->ErrorSum < r->IntegratorMin)
  {
    r->ErrorSum = r->IntegratorMin;
  }
  r->Integrator = r->Ki * r->ErrorSum;

  r->Derivative = r->Kd * (r->Error - r->LastError);
  r->LastError = r->Error;

  r->Output = r->Kp * r->Error + r->Integrator + r->Derivative;

  if (r->Output > r->OutMax)
  {
    r->Output = r->OutMax;
  }
  else if (r->Output < r->OutMin)
  {
    r->Output = r->OutMin;
  }

  return r->Output;
}

void MitReg_Init(MitReg_t *r,
                 float kp, float kd,
                 float out_max, float out_min,
                 float iq_feed)
{
  r->Kp = kp;
  r->Kd = kd;
  r->OutMax = out_max;
  r->OutMin = out_min;
  r->IqFeed = iq_feed;
  r->Output = 0.0f;
}

float MitReg_Update(MitReg_t *r, float pos_err, float speed_err)
{
  float out = r->Kp * pos_err + r->Kd * speed_err + r->IqFeed;

  if (out > r->OutMax)
  {
    out = r->OutMax;
  }
  else if (out < r->OutMin)
  {
    out = r->OutMin;
  }

  r->Output = out;
  return out;
}
