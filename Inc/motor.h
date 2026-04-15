#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"
#include <stdint.h>

/* ============================================================
 *  方向引脚配置 ⚠️ 根据实际接线修改!
 *
 *  打开 main.h 搜索 "AIN1" 或 "BIN1",看 CubeMX 给的标签名,
 *  或者直接写成 GPIOx + GPIO_PIN_n 的形式
 * ============================================================ */
#define MOTORA_AIN1_PORT    GPIOB
#define MOTORA_AIN1_PIN     GPIO_PIN_0
#define MOTORA_AIN2_PORT    GPIOB
#define MOTORA_AIN2_PIN     GPIO_PIN_1

#define MOTORB_BIN1_PORT    GPIOC       /* ← 改 */
#define MOTORB_BIN1_PIN     GPIO_PIN_11  /* ← 改 */
#define MOTORB_BIN2_PORT    GPIOC       /* ← 改 */
#define MOTORB_BIN2_PIN     GPIO_PIN_10  /* ← 改 */

/* ============================================================
 *  PWM 配置
 *  硬件: TIM1_CH1 = MotorA PWM (PA8)
 *        TIM1_CH2 = MotorB PWM (PA9)
 *  ARR = 7200, PWM 频率 ≈ 14kHz (100MHz / 7200)
 * ============================================================ */
#define MOTOR_MAX_PWM       7200
#define MOTOR_MIN_PWM      (-7200)

/* ============================================================
 *  死区补偿 (正反不对称)
 *  按 ARR=7200 的比例初值,实测后微调
 * ============================================================ */
#define MOTOR_DEADZONE_A_FWD   430   /* A 电机正转死区 */
#define MOTOR_DEADZONE_A_BKW   430   /* A 电机反转死区 (+20%) */
#define MOTOR_DEADZONE_B_FWD   430   /* B 电机正转死区 */
#define MOTOR_DEADZONE_B_BKW   430   /* B 电机反转死区 */

/* 小于此阈值视为停止指令,不补死区(避免静止抖动) */
#define MOTOR_ZERO_BAND        20

/* ============================================================
 *  API
 * ============================================================ */
void Motor_Init(void);
void Motor_SetSpeed(int16_t motorA, int16_t motorB);      /* 带死区补偿 */
void Motor_SetSpeed_Raw(int16_t motorA, int16_t motorB);  /* 无补偿,测试用 */
void Motor_Stop(void);

#endif /* __MOTOR_H__ */
