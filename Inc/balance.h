#ifndef __BALANCE_H
#define __BALANCE_H

#include "main.h"

// 直立点角度（安装后需校正）
#define BALANCE_ANGLE_SETPOINT   0.0f

// 直立环PID初始参数（需调试）
#define BALANCE_KP    200.0f
#define BALANCE_KI      0.0f
#define BALANCE_KD     10.0f

// 速度环PID初始参数
#define SPEED_KP       50.0f
#define SPEED_KI        0.5f
#define SPEED_KD        0.0f

// 转向比例系数
#define TURN_KP       100.0f

// 倾角保护阈值（超过此角度立即停车）
#define FALL_ANGLE_THRESHOLD  30.0f

void  Balance_Init(void);
void  Balance_Update(void);              // 每5ms在定时器中断中调用
void  Balance_SetAngleOffset(float deg); // 校正直立角度偏移

#endif
