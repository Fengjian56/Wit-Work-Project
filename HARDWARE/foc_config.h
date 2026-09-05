/* foc_config.h - project-wide motor and control constants
 *
 * Shared parameters used by the FOC application modules. Values mirror the
 * hardware under test: 14-pole-pair joint motor, INA240A1PWR current sense
 * (gain 20 V/V) on 5 mOhm low-side shunts, dual MT6701 encoders over SPI.
 */
#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

#include <stdint.h>

/* Pole pairs of the motor under test. */
#define FOC_POLE_PAIRS 14

/* Current sense chain: shunt resistance and amplifier gain. */
#define FOC_SHUNT_OHM 0.005f
#define FOC_CSA_GAIN 20.0f

/* ADC electrical scale. */
#define FOC_ADC_VREF 3.3f
#define FOC_ADC_FS 4095.0f

/* Number of zero-current samples averaged for offset calibration. */
#define FOC_OFFSET_SAMPLES 100U

/* One-millisecond TIM2 base interval used for speed/MIT scheduling. */
#define FOC_TIM2_PERIOD_S 0.001f

/* Position loop runs every N TIM2 ticks -> 1 kHz / 10 = 100 Hz. */
#define FOC_POSITION_DIVIDER 10U

/* Internal position-reference slew limit. */
#define FOC_RAMP_SPEED_DEG_S 180.0f

#endif
