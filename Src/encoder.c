#include "encoder.h"
#include "tim.h"
#include "rtt_log.h"

static int16_t encA_count = 0;
static int16_t encB_count = 0;
static float   speedA_rps = 0.0f;
static float   speedB_rps = 0.0f;

void Encoder_Init(void) {
    HAL_StatusTypeDef ret;

    // 启动 TIM3 电机A编码器（PA6=E1A CH1, PA7=E2A CH2）
    ret = HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    if (ret != HAL_OK) {
        RTT_LOG_ERROR("[Encoder] TIM3 编码器启动失败 (电机A) HAL_ERR=%d", (int)ret);
    } else {
        RTT_LOG_INFO("[Encoder] TIM3 编码器启动成功 (电机A: PA6=E1A, PA7=E2A)");
    }

    // 启动 TIM4 电机B编码器（PB6=E2B CH1, PB7=E1B CH2）
    ret = HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    if (ret != HAL_OK) {
        RTT_LOG_ERROR("[Encoder] TIM4 编码器启动失败 (电机B) HAL_ERR=%d", (int)ret);
    } else {
        RTT_LOG_INFO("[Encoder] TIM4 编码器启动成功 (电机B: PB6=E2B, PB7=E1B)");
    }

    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    RTT_LOG_INFO("[Encoder] 初始化完成，计数器已清零");
}

void Encoder_Update(void) {
    encA_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    encB_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    float f = 1000.0f / (float)READ_PERIOD_MS;
    speedA_rps = (encA_count * f / 4.0f) / (REDUCTION_RATIO * ENCODER_PPR);
    speedB_rps = (encB_count * f / 4.0f) / (REDUCTION_RATIO * ENCODER_PPR);

    // 异常速度检测（编码器可能断线或计数异常）
    if (speedA_rps > 50.0f || speedA_rps < -50.0f) {
        RTT_LOG_ERROR("[Encoder] 电机A速度异常: %.2f r/s，请检查编码器接线 (PA6/PA7)",
                      speedA_rps);
    }
    if (speedB_rps > 50.0f || speedB_rps < -50.0f) {
        RTT_LOG_ERROR("[Encoder] 电机B速度异常: %.2f r/s，请检查编码器接线 (PB6/PB7)",
                      speedB_rps);
    }
}

int16_t Encoder_GetCountA(void)       { return encA_count; }
int16_t Encoder_GetCountB(void)       { return encB_count; }
float   Encoder_GetSpeedA_rps(void)   { return speedA_rps; }
float   Encoder_GetSpeedB_rps(void)   { return speedB_rps; }

float Encoder_GetSpeedAvg_ms(void) {
    return ((speedA_rps + speedB_rps) / 2.0f) * WHEEL_CIRCUMFERENCE;
}
