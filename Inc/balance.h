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
#define BALANCE_KP    55.0f
#define BALANCE_KI     0.0f
#define BALANCE_KD     1.0f

/*============================================================
 *  Speed PID (outer loop)
 *  Input : speed error (m/s)
 *  Output: angle compensation (deg)
 *============================================================*/
#define SPEED_KP      -8.0f
#define SPEED_KI      -0.02f
#define SPEED_KD       0.0f

/* Steering coefficient */
#define TURN_KP      100.0f

/* Upright angle setpoint (deg) */
#define BALANCE_ANGLE_SETPOINT  -5.9f

/* Fall protection threshold (deg) */
#define FALL_ANGLE_THRESHOLD    30.0f

void Balance_Init(void);
void Balance_Update(void);
void Balance_SetAngleOffset(float deg);

#endif /* __BALANCE_H */
