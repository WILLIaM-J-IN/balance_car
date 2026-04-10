#ifndef __BALANCE_H
#define __BALANCE_H

#include "stm32f4xx_hal.h"
#include "pid.h"
#include "motor.h"
#include <stdint.h>

/*============================================================
 *  Balance PID (inner loop)
 *  Input : angle error (deg)
 *  Output: motor PWM
 *============================================================*/
#define BALANCE_KP    88.0f
#define BALANCE_KI     0.0f
#define BALANCE_KD     1.8f

/*============================================================
 *  Speed PID (outer loop)
 *  Input : speed error (m/s)
 *  Output: angle compensation (deg)
 *============================================================*/
#define SPEED_KP     -0.0f
#define SPEED_KI      -0.00f
#define SPEED_KD       0.0f

/* Steering coefficient */
#define TURN_KP      100.0f

/* Upright angle setpoint (deg) */
#define MECH_ZERO_ANGLE     0.72f

/* Fall protection threshold (deg) */
#define FALL_ANGLE_THRESHOLD    30.0f

void Balance_Init(void);
void Balance_Update(void);
void Balance_SetAngleOffset(float deg);

#endif /* __BALANCE_H */
