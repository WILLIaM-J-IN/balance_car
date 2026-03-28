#include "motor.h"
#include "tim.h"
#include "rtt_log.h"
#include <stdlib.h>

void Motor_Init(void) {
    HAL_StatusTypeDef ret;

    ret = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    if (ret != HAL_OK) {
        RTT_LOG_ERROR("[Motor] TIM1_CH1 PWM启动失败 HAL_ERR=%d", (int)ret);
    } else {
        RTT_LOG_INFO("[Motor] TIM1_CH1 PWM启动成功 (PA8 电机A)");
    }

    ret = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    if (ret != HAL_OK) {
        RTT_LOG_ERROR("[Motor] TIM1_CH2 PWM启动失败 HAL_ERR=%d", (int)ret);
    } else {
        RTT_LOG_INFO("[Motor] TIM1_CH2 PWM启动成功 (PA9 电机B)");
    }

    Motor_Stop();
    RTT_LOG_INFO("[Motor] 初始化完成，电机已停止");
}

static void MotorA_SetDir(int16_t speed) {
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

static void MotorB_SetDir(int16_t speed) {
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

void Motor_SetSpeed(int16_t motorA, int16_t motorB) {
    // 限幅检查并警告
    if (motorA > MOTOR_MAX_PWM || motorA < MOTOR_MIN_PWM) {
        RTT_LOG_WARN("[Motor] 电机A PWM超限: %d, 已截断至 [%d, %d]",
                     motorA, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
        motorA = (motorA > MOTOR_MAX_PWM) ? MOTOR_MAX_PWM : MOTOR_MIN_PWM;
    }
    if (motorB > MOTOR_MAX_PWM || motorB < MOTOR_MIN_PWM) {
        RTT_LOG_WARN("[Motor] 电机B PWM超限: %d, 已截断至 [%d, %d]",
                     motorB, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
        motorB = (motorB > MOTOR_MAX_PWM) ? MOTOR_MAX_PWM : MOTOR_MIN_PWM;
    }

    MotorA_SetDir(motorA);
    MotorB_SetDir(motorB);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)abs(motorA));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)abs(motorB));
}

void Motor_Stop(void) {
    Motor_SetSpeed(0, 0);
    RTT_LOG_DEBUG("[Motor] 电机停止");
}
