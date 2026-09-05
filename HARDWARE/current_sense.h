/* current_sense.h - phase-current acquisition and dq feedback
 *
 * Implements two-phase low-side current sampling with zero-current offset
 * calibration, C-phase reconstruction, Clarke/Park transforms, and filtered
 * Id/Iq feedback used by the current regulators.
 */
#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <stdint.h>

/* Raw phase currents in amperes. */
extern volatile float PhaseA_A;
extern volatile float PhaseB_A;
extern volatile float PhaseC_A;

/* Clarke and Park outputs. */
extern float IAlpha;
extern float IBeta;
extern float Id;
extern float Iq;

/* Filtered dq feedback (configured alpha). */
extern float FilterId;
extern float FilterIq;

/* Current regulator loop enables and target refs. */
extern volatile uint8_t CurrentLoopEnable;
extern volatile uint8_t VoltageTestEnable;
extern volatile float IdRef;
extern volatile float IqRef;
extern volatile float UdTest;
extern volatile float UqTest;

/* dq feedback first-order filter coefficients. */
extern volatile float IdFilterAlpha;
extern volatile float IqFilterAlpha;

/* Diagnostics. */
extern volatile uint32_t AdcInjectedCount;

void CurrentSense_StartOffsetCalibration(void);
uint8_t CurrentSense_IsOffsetCalibrated(void);

#endif
