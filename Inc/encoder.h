#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

// 根据你的电机参数修改
#define REDUCTION_RATIO      30        // 减速比
#define ENCODER_PPR          13        // 霍尔编码器线数
#define READ_PERIOD_MS       5         // 读取周期(ms)
#define WHEEL_CIRCUMFERENCE  0.204f    // 轮子周长(m)，根据实际修改

// 编码器引脚对应关系（仅供参考，实际由CubeMX配置）
// 电机A编码器: E1A=PA6(TIM3_CH1)  E2A=PA7(TIM3_CH2)
// 电机B编码器: E2B=PB6(TIM4_CH1)  E1B=PB7(TIM4_CH2)

void    Encoder_Init(void);
void    Encoder_Update(void);            // 每READ_PERIOD_MS ms调用一次

int16_t Encoder_GetCountA(void);         // 电机A编码器计数（带方向）
int16_t Encoder_GetCountB(void);         // 电机B编码器计数（带方向）
float   Encoder_GetSpeedA_rps(void);     // 电机A转速 r/s
float   Encoder_GetSpeedB_rps(void);     // 电机B转速 r/s
float   Encoder_GetSpeedAvg_ms(void);    // 两轮平均线速度 m/s

#endif
