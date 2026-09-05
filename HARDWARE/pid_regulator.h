/* pid_regulator.h - generic PID and MIT/PD position controller
 *
 * Position-form PID with bounded error accumulation and output clamping.
 * The MIT controller is a PD position/speed law whose output is an Iq
 * reference fed into the q-axis current loop.
 */
#ifndef PID_REGULATOR_H
#define PID_REGULATOR_H

typedef struct
{
  float Kp;
  float Ki;
  float Kd;
  float OutMax;
  float OutMin;
  float IntegratorMax;
  float IntegratorMin;

  float Error;
  float LastError;
  float ErrorSum;
  float Integrator;
  float Derivative;
  float Output;
} PidReg_t;

typedef struct
{
  float Kp;      /* Iq per degree */
  float Kd;      /* Iq per rad/s  */
  float IqFeed;  /* constant current feedforward */
  float OutMax;
  float OutMin;
  float Output;
} MitReg_t;

void PidReg_Init(PidReg_t *r,
                 float kp, float ki, float kd,
                 float out_max, float out_min,
                 float integ_max, float integ_min);

float PidReg_Update(PidReg_t *r, float err);

void MitReg_Init(MitReg_t *r,
                 float kp, float kd,
                 float out_max, float out_min,
                 float iq_feed);

float MitReg_Update(MitReg_t *r, float pos_err, float speed_err);

#endif
