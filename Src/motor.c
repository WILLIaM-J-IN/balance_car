#include "motor.h"
#include "tim.h"
#include "rtt_log.h"
#include <stdlib.h>

/* ============================================================
 *  死区补偿 (Dead-zone compensation)
 *  MG513P30-12V 实测启动 PWM ≈ 90
 *  四个方向(A正/A反/B正/B反)必须对称,否则平衡环无法收敛
 *  如果两个电机机械差异较大,可分别定义 A/B 死区
 * ============================================================ */
#define MOTOR_DEADZONE_A   60   /* A 电机静摩擦补偿 */
#define MOTOR_DEADZONE_B   60   /* B 电机静摩擦补偿 */

/* 小于此阈值视为停止指令,不补死区(避免静止时抖动) */
#define MOTOR_ZERO_BAND    3

void Motor_Init(void)
{
    HAL_StatusTypeDef ret;

    ret = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    if (ret != HAL_OK) {
        RTT_LOG_ERROR("[Motor] TIM1_CH1 PWM start FAILED HAL_ERR=%d", (int)ret);
    } else {
        RTT_LOG_INFO("[Motor] TIM1_CH1 PWM start OK (PA8 MotorA)");
    }

    ret = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    if (ret != HAL_OK) {
        RTT_LOG_ERROR("[Motor] TIM1_CH2 PWM start FAILED HAL_ERR=%d", (int)ret);
    } else {
        RTT_LOG_INFO("[Motor] TIM1_CH2 PWM start OK (PA9 MotorB)");
    }

    Motor_Stop();
    RTT_LOG_INFO("[Motor] Init done, deadzone A=%d B=%d",
                 MOTOR_DEADZONE_A, MOTOR_DEADZONE_B);
}

/* ----------- 方向引脚 ----------- */
static void MotorA_SetDir(int16_t speed)
{
    if (speed > 0) {
        HAL_GPIO_WritePin(MOTORA_AIN1_PORT, MOTORA_AIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTORA_AIN2_PORT, MOTORA_AIN2_PIN, GPIO_PIN_RESET);
    } else if (speed < 0) {
        HAL_GPIO_WritePin(MOTORA_AIN1_PORT, MOTORA_AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTORA_AIN2_PORT, MOTORA_AIN2_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(MOTORA_AIN1_PORT, MOTORA_AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTORA_AIN2_PORT, MOTORA_AIN2_PIN, GPIO_PIN_RESET);
    }
}

static void MotorB_SetDir(int16_t speed)
{
    if (speed > 0) {
        HAL_GPIO_WritePin(MOTORB_BIN1_PORT, MOTORB_BIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTORB_BIN2_PORT, MOTORB_BIN2_PIN, GPIO_PIN_RESET);
    } else if (speed < 0) {
        HAL_GPIO_WritePin(MOTORB_BIN1_PORT, MOTORB_BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTORB_BIN2_PORT, MOTORB_BIN2_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(MOTORB_BIN1_PORT, MOTORB_BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTORB_BIN2_PORT, MOTORB_BIN2_PIN, GPIO_PIN_RESET);
    }
}

/* ----------- 对称死区补偿 -----------
 * 关键思路:输出 = sign(u) * (deadzone + |u| * (MAX-deadzone)/MAX)
 * 这样小指令直接跳到 deadzone,大指令线性映射到 MAX,
 * 整体在零点附近连续单调,不会出现正反向不对称。
 */
static int16_t apply_deadzone(int16_t u, int16_t dz)
{
    if (u >  MOTOR_ZERO_BAND) {
        int32_t out = dz + ((int32_t)(MOTOR_MAX_PWM - dz) * u) / MOTOR_MAX_PWM;
        if (out > MOTOR_MAX_PWM) out = MOTOR_MAX_PWM;
        return (int16_t)out;
    }
    if (u < -MOTOR_ZERO_BAND) {
        int32_t out = -dz + ((int32_t)(MOTOR_MAX_PWM - dz) * u) / MOTOR_MAX_PWM;
        if (out < -MOTOR_MAX_PWM) out = -MOTOR_MAX_PWM;
        return (int16_t)out;
    }
    return 0;
}

void Motor_SetSpeed(int16_t motorA, int16_t motorB)
{
    motorA = apply_deadzone(motorA, MOTOR_DEADZONE_A);
    motorB = apply_deadzone(motorB, MOTOR_DEADZONE_B);

    /* 终极限幅(防御性) */
    if (motorA >  MOTOR_MAX_PWM) motorA =  MOTOR_MAX_PWM;
    if (motorA <  MOTOR_MIN_PWM) motorA =  MOTOR_MIN_PWM;
    if (motorB >  MOTOR_MAX_PWM) motorB =  MOTOR_MAX_PWM;
    if (motorB <  MOTOR_MIN_PWM) motorB =  MOTOR_MIN_PWM;

    MotorA_SetDir(motorA);
    MotorB_SetDir(motorB);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)abs(motorA));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)abs(motorB));
}

void Motor_Stop(void)
{
    HAL_GPIO_WritePin(MOTORA_AIN1_PORT, MOTORA_AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTORA_AIN2_PORT, MOTORA_AIN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTORB_BIN1_PORT, MOTORB_BIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTORB_BIN2_PORT, MOTORB_BIN2_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}
