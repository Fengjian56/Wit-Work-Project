# G431FOC - PMSM Field-Oriented Control on STM32G431CBT6

Independent PMSM FOC motor-driver implementation on STM32G431CBT6. The firmware covers a joint-actuator motor: MT6701 magnetic encoders over SPI+DMA, two-phase low-side current sensing with zero-offset calibration, Clarke/Park transforms, SVPWM, complementary PWM, and cascaded current/speed/position loops plus a simplified MIT impedance controller.

> The control software is written on top of ST HAL and ARM CMSIS-DSP libraries, which are standard vendor components. This repository is a clean-room style re-implementation used for portfolio and learning purposes.

## Hardware

| Item | Part / Value |
| --- | --- |
| MCU | STM32G431CBT6, 170 MHz |
| Motor encoders | MT6701 (motor side over SPI3, reducer side over SPI1) |
| Current sense | INA240A1PWR (gain 20 V/V), two low-side shunts |
| Shunt | 5 mOhm |
| Gate driver | FD6288Q |
| Power MOSFETs | WSD4070DN |

## Control loops

| Loop | Rate |
| --- | --- |
| Current | 20 kHz |
| Speed / MIT | 1 kHz |
| Position | 100 Hz |

The ADC injection group is triggered by the center-aligned PWM. Because two update events occur per PWM period, only one TIM counting direction is accepted so the effective current-loop rate is 20 kHz while the raw ADC callback rate is about 40 kHz.

## Layout

```text
Core/       Application init, ISRs, HAL configuration
HARDWARE/   FOC application modules (new implementation)
Drivers/    ST HAL and CMSIS
Middlewares/ ARM CMSIS-DSP
MDK-ARM/    Keil project
media/      bench waveforms, photos and demo video
G431FOC.ioc  STM32CubeMX configuration
```

## FOC modules

| File | Responsibility |
| --- | --- |
| `HARDWARE/angle_encoder.c` | MT6701 SPI+DMA, angle unwrap, electrical angle |
| `HARDWARE/current_sense.c` | Injected ADC, offset calibration, Clarke/Park, current loop |
| `HARDWARE/svpwm_driver.c` | SVPWM synthesis and TIM1 compare mapping |
| `HARDWARE/pid_regulator.c` | Position-form PID and MIT/PD controller |
| `HARDWARE/motor_controller.c` | 1 kHz speed/MIT/position scheduling |
| `HARDWARE/foc_app.c` | Init, alignment, test trajectories, telemetry |
| `HARDWARE/telemetry.c` | VOFA+ JustFloat output |

## Build

Open in Keil:

```text
MDK-ARM/G431FOC.uvprojx
```

or open `G431FOC.ioc` in STM32CubeMX to inspect the peripheral setup.

## Results / debug media

The repository includes bench waveforms, PCB 3D views and a motor-run video
under `media/`. These are debug material captured on the joint-actuator
motor platform that this firmware targets:

```text
media/current_loop_iq_id.png               current-loop Iq/Id tracking
media/current_loop_positive_step_windup.png  positive-step saturation case
media/current_loop_negative_step_anti_windup.png negative-step recovery case
media/three_loop_position.png              position/speed/current tracking
media/before_adc_spi_fix.png               before ADC/SPI timing fix
media/after_adc_spi_fix.png                after ADC/SPI timing fix
media/speed_ripple_1_2_rad.png             speed-ripple comparison
media/magnet_eccentric_severe.png          magnet eccentricity case
media/magnet_eccentric_comparison.png      magnet eccentricity comparison
media/magnet_eccentric_optimized.png       after eccentricity optimization
media/driver_board_photo.png               driver board / bench photo
media/motor_run.mp4                        motor run demo
```

> These media are shared as platform debug material. Firmware behaviour on a
> specific motor still needs to be revalidated after hardware bring-up.

## Test modes

Select the active test trajectory by editing `FOC_APP_MODE` in `HARDWARE/foc_app.c`:

```c
FOC_MODE_CURRENT                    // Iq step tracking
FOC_MODE_SPEED_CURRENT              // speed-current cascade, sinusoidal target
FOC_MODE_POSITION_SPEED_CURRENT     // position-speed-current cascade
FOC_MODE_MIT                        // simplified MIT impedance controller
```

> Motor-driver firmware. Confirm power connections, phase order, current limits, encoder direction and mechanical fixation before applying power.
