/* svpwm_driver.h - space-vector PWM synthesis and PWM compare write
 *
 * Converts Ud/Uq and electrical angle into three phase duty cycles using
 * inverse Park plus a seven-segment SVPWM, then maps the phases onto the
 * TIM1 compare channels for the physical half-bridge wiring.
 */
#ifndef SVPWM_DRIVER_H
#define SVPWM_DRIVER_H

void Svpwm_Apply(float ud, float uq, float theta_rad,
                 float *du, float *dv, float *dw);

void Pwm_WritePhases(float du, float dv, float dw);

#endif
