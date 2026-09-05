/* angle_encoder.h - MT6701 dual-encoder interface
 *
 * Reads the motor-side (SPI3) and reducer-side (SPI1) MT6701 absolute
 * encoders over DMA, unwraps each angle into a continuous position, and
 * derives the electrical angle used by the FOC transforms.
 */
#ifndef ANGLE_ENCODER_H
#define ANGLE_ENCODER_H

#include <stdint.h>

/* Continuous mechanical angle of the motor shaft, in degrees. */
extern volatile float MotorAngleDeg;

/* Continuous output angle of the reducer-side encoder, in degrees. */
extern volatile float OutputAngleDeg;

/* Electrical angle of the motor in radians, in [0, 2*pi). */
extern volatile float ElecAngleRad;

/* Latest single-turn raw electrical angle (radians), used for alignment. */
extern volatile float RawElecAngleRad;

/* Latest single-turn output encoder angle (radians). */
extern volatile float RawOutputAngleRad;

/* SPI data buffers for motor-side and output-side encoders. */
extern uint8_t MotorEncFrame[3];
extern uint8_t OutputEncFrame[3];

/* Alignment offset stored at open-loop zero reference, in radians. */
extern volatile float EncoderZero;

void AngleEncoder_InitCapture(void);

#endif
