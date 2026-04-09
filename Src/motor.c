#include "motor.h"
#include "tim.h"
#include "rtt_log.h"
#include <stdlib.h>

/* Dead zone compensation
 * Measured: motor starts moving at PWM=90
 * Any PWM below 90 will be boosted to 90 to overcome static friction */
#define MOTOR_DEADZONE  70

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
    RTT_LOG_INFO("[Motor] Init done, deadzone=%d", MOTOR_DEADZONE);
}

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

void Motor_SetSpeed(int16_t motorA, int16_t motorB)
{
    /* Dead zone compensation:
     * If command is nonzero but below deadzone threshold,
     * boost to deadzone value so motor actually moves.
     * If command is 0, keep 0 (motor stop). */
    if (motorA > 0 && motorA < MOTOR_DEADZONE)
        motorA = MOTOR_DEADZONE+150;
    else if (motorA < 0 && motorA > -MOTOR_DEADZONE)
        motorA = -MOTOR_DEADZONE;

    if (motorB > 0 && motorB < MOTOR_DEADZONE)
        motorB = MOTOR_DEADZONE;
    else if (motorB < 0 && motorB > -MOTOR_DEADZONE)
        motorB = -MOTOR_DEADZONE;

    /* Clamp to valid range */
    if (motorA > MOTOR_MAX_PWM)  motorA =  MOTOR_MAX_PWM;
    if (motorA < MOTOR_MIN_PWM)  motorA =  MOTOR_MIN_PWM;
    if (motorB > MOTOR_MAX_PWM)  motorB =  MOTOR_MAX_PWM;
    if (motorB < MOTOR_MIN_PWM)  motorB =  MOTOR_MIN_PWM;

    MotorA_SetDir(motorA);
    MotorB_SetDir(motorB);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)abs(motorA));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)abs(motorB));
}

void Motor_Stop(void)
{
    /* Stop: set direction pins low then zero PWM */
    HAL_GPIO_WritePin(MOTORA_AIN1_PORT, MOTORA_AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTORA_AIN2_PORT, MOTORA_AIN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTORB_BIN1_PORT, MOTORB_BIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTORB_BIN2_PORT, MOTORB_BIN2_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}
