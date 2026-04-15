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
#define BALANCE_KP    760.0f
#define BALANCE_KI    0.0f
#define BALANCE_KD     16.0f

/*============================================================
 *  Speed PID (outer loop, when position loop disabled)
 *  Input : speed error (m/s)
 *  Output: angle compensation (deg)
 *============================================================*/
#define SPEED_KP     -0.001f
#define SPEED_KI      -0.000001f
#define SPEED_KD       0.0f

/*============================================================
 *  Position PID (outermost loop, optional)
 *  Input : position error (m)
 *  Output: angle compensation (deg)
 *============================================================*/


/* Steering coefficient */
#define TURN_KP      100.0f

/* Upright angle setpoint (deg) */
#define MECH_ZERO_ANGLE    -1.45f


/* Fall protection threshold (deg) */
#define FALL_ANGLE_THRESHOLD    35.0f



#define POS_TRIGGER_MM      40      /* 偏移 ≥ 40mm 触发位置环 */
#define POS_RELEASE_MM      15      /* 偏移 ≤ 15mm 释放位置环 */

#define POS_KP              2.0f
#define POS_KD              0.0f
#define POS_OUT_LIMIT       3.0f
#define POS_REACHED_THRESH  0.02f    /* 到达目标判定阈值 (米) */

/*------------------------------------------------------------
 *  Core API
 *------------------------------------------------------------*/
void Balance_Init(void);
void Balance_Update(void);
void Balance_SetAngleOffset(float deg);

/*------------------------------------------------------------
 *  Position loop API
 *------------------------------------------------------------*/
void    Balance_HoldPosition(void);              /* 锁定当前位置 */
void    Balance_MoveDistance(float meters);      /* 相对当前位置移动(米) */
void    Balance_SetTargetPosition(float meters); /* 设置绝对目标位置 */
void    Balance_DisablePosition(void);           /* 禁用位置环 */
uint8_t Balance_IsPositionHoldActive(void);
float   Balance_GetPositionError(void);          /* 目标 - 当前 (米) */



void Balance_EnableAutoHold(void);
void Balance_DisableAutoHold(void);
uint8_t Balance_IsAutoHoldActive(void);

#endif /* __BALANCE_H */
