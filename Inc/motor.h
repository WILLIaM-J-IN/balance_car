#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

// PWM最大值，对应CubeMX中Counter Period = 999
#define MOTOR_MAX_PWM    999
#define MOTOR_MIN_PWM   -999

// 电机A: PWM=PA8(TIM1_CH1), AIN1=PB0, AIN2=PB1
// 电机B: PWM=PA9(TIM1_CH2), BIN1=PC11, BIN2=PC10
#define MOTORA_AIN1_PORT    GPIOB
#define MOTORA_AIN1_PIN     GPIO_PIN_0
#define MOTORA_AIN2_PORT    GPIOB
#define MOTORA_AIN2_PIN     GPIO_PIN_1

#define MOTORB_BIN1_PORT    GPIOC
#define MOTORB_BIN1_PIN     GPIO_PIN_11
#define MOTORB_BIN2_PORT    GPIOC
#define MOTORB_BIN2_PIN     GPIO_PIN_10

void Motor_Init(void);
void Motor_SetSpeed(int16_t motorA, int16_t motorB);
void Motor_Stop(void);

#endif
