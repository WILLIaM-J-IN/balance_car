#include "motor.h"
#include "tim.h"
#include "rtt_log.h"
#include <stdlib.h>

/*============================================================
 * 初始化
 *============================================================*/
void Motor_Init(void)
{
    HAL_StatusTypeDef ret;

    ret = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    if (ret != HAL_OK) {
        RTT_LOG_ERROR("[Motor] TIM1_CH1 PWM start FAILED HAL_ERR=%d", (int)ret);
    } else {
        RTT_LOG_INFO("[Motor] TIM1_CH1 PWM start OK (MotorA)");
    }

    ret = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    if (ret != HAL_OK) {
        RTT_LOG_ERROR("[Motor] TIM1_CH2 PWM start FAILED HAL_ERR=%d", (int)ret);
    } else {
        RTT_LOG_INFO("[Motor] TIM1_CH2 PWM start OK (MotorB)");
    }

    Motor_Stop();
    RTT_LOG_INFO("[Motor] Init done, MAX_PWM=%d, DZ A=%d/%d B=%d/%d",
                 MOTOR_MAX_PWM,
                 MOTOR_DEADZONE_A_FWD, MOTOR_DEADZONE_A_BKW,
                 MOTOR_DEADZONE_B_FWD, MOTOR_DEADZONE_B_BKW);
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

/* ----------- 对称死区补偿 (支持正反不对称) -----------
 * 输出 = sign(u) * (dz + |u| * (MAX-dz)/MAX)
 */
static int16_t apply_deadzone(int16_t u, int16_t dz_fwd, int16_t dz_bkw)
{
    if (u >  MOTOR_ZERO_BAND) {
        int32_t out = dz_fwd + ((int32_t)(MOTOR_MAX_PWM - dz_fwd) * u) / MOTOR_MAX_PWM;
        if (out > MOTOR_MAX_PWM) out = MOTOR_MAX_PWM;
        return (int16_t)out;
    }
    if (u < -MOTOR_ZERO_BAND) {
        int32_t out = -dz_bkw + ((int32_t)(MOTOR_MAX_PWM - dz_bkw) * u) / MOTOR_MAX_PWM;
        if (out < -MOTOR_MAX_PWM) out = -MOTOR_MAX_PWM;
        return (int16_t)out;
    }
    return 0;
}

void Motor_SetSpeed(int16_t motorA, int16_t motorB)
{
    motorA = apply_deadzone(motorA, MOTOR_DEADZONE_A_FWD, MOTOR_DEADZONE_A_BKW);
    motorB = apply_deadzone(motorB, MOTOR_DEADZONE_B_FWD, MOTOR_DEADZONE_B_BKW);

    /* 终极限幅 */
    if (motorA >  MOTOR_MAX_PWM) motorA =  MOTOR_MAX_PWM;
    if (motorA <  MOTOR_MIN_PWM) motorA =  MOTOR_MIN_PWM;
    if (motorB >  MOTOR_MAX_PWM) motorB =  MOTOR_MAX_PWM;
    if (motorB <  MOTOR_MIN_PWM) motorB =  MOTOR_MIN_PWM;

    MotorA_SetDir(motorA);
    MotorB_SetDir(motorB);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)abs(motorA));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)abs(motorB));
}

/* 无死区补偿,用于死区测试 */
void Motor_SetSpeed_Raw(int16_t motorA, int16_t motorB)
{
    if (motorA >  MOTOR_MAX_PWM) motorA =  MOTOR_MAX_PWM;
    if (motorA < -MOTOR_MAX_PWM) motorA = -MOTOR_MAX_PWM;
    if (motorB >  MOTOR_MAX_PWM) motorB =  MOTOR_MAX_PWM;
    if (motorB < -MOTOR_MAX_PWM) motorB = -MOTOR_MAX_PWM;

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
