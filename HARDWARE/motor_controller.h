/* motor_controller.h - cascaded speed/position/MIT scheduling
 *
 * Runs inside the TIM2 1 kHz callback: computes filtered motor speed from
 * encoder displacement, then executes either the MIT law or the cascaded
 * position->speed->current structure and writes IqRef for the ADC ISR.
 */
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <stdint.h>
#include "pid_regulator.h"

extern volatile float FilteredSpeed;    /* rad/s */
extern volatile float SpeedFilterAlpha;

extern PidReg_t SpeedPid;
extern PidReg_t PositionPid;
extern MitReg_t MitCtrl;
extern PidReg_t TorquePidD;
extern PidReg_t TorquePidQ;

extern volatile float SpeedRef;              /* rad/s */
extern volatile float PositionTargetDeg;     /* target in degrees */
extern volatile float PositionReferenceDeg;  /* ramped reference */

extern volatile float MitPositionTargetDeg;
extern volatile float MitSpeedRef;

extern volatile uint8_t SpeedLoopEnable;
extern volatile uint8_t PositionLoopEnable;
extern volatile uint8_t MitLoopEnable;

void MotorControl_1kHzTick(void);

#endif
