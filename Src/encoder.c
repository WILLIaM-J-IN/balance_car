#include "encoder.h"
#include "tim.h"
#include "rtt_log.h"

/* ---- 原始寄存器值 (16 位,会回绕) ---- */
static int16_t  last_rawA = 0;
static int16_t  last_rawB = 0;

/* ---- 本周期的增量 (用于算速度) ---- */
static int16_t  encA_count = 0;
static int16_t  encB_count = 0;

/* ---- 累积总位置 (用于位置环) ---- */
static int32_t  totalA = 0;
static int32_t  totalB = 0;

/* ---- 速度 ---- */
static float    speedA_rps = 0.0f;
static float    speedB_rps = 0.0f;

void Encoder_Init(void) {
    HAL_StatusTypeDef ret;

    ret = HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    if (ret != HAL_OK) RTT_LOG_ERROR("[Encoder] TIM3 启动失败");
    else               RTT_LOG_INFO ("[Encoder] TIM3 OK (电机A)");

    ret = HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    if (ret != HAL_OK) RTT_LOG_ERROR("[Encoder] TIM4 启动失败");
    else               RTT_LOG_INFO ("[Encoder] TIM4 OK (电机B)");

    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    last_rawA = 0;
    last_rawB = 0;
    totalA    = 0;
    totalB    = 0;

    RTT_LOG_INFO("[Encoder] 初始化完成");
}

void Encoder_Update(void) {
    int16_t rawA = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t rawB = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

    int16_t deltaA = rawA - last_rawA;
    int16_t deltaB = -(rawB - last_rawB);

    last_rawA = rawA;
    last_rawB = rawB;

    /* ⭐ 异常脉冲过滤
     * 5ms 内增量超过 200 个脉冲基本不可能(对应转速 > 30 r/s),
     * 这种情况判为干扰,丢弃本次增量 */
    if (deltaA > 200 || deltaA < -200) {
        RTT_LOG_WARN("[Encoder] A spike rejected: %d", (int)deltaA);
        deltaA = 0;
    }
    if (deltaB > 200 || deltaB < -200) {
        RTT_LOG_WARN("[Encoder] B spike rejected: %d", (int)deltaB);
        deltaB = 0;
    }

    encA_count = deltaA;
    encB_count = deltaB;

    totalA += encA_count;
    totalB += encB_count;

    /* 后续速度计算不变 */
    float f = 1000.0f / (float)READ_PERIOD_MS;
    speedA_rps = (encA_count * f / 2.0f) / (REDUCTION_RATIO * ENCODER_PPR);
    speedB_rps = (encB_count * f / 2.0f) / (REDUCTION_RATIO * ENCODER_PPR);

    if (speedA_rps > 50.0f || speedA_rps < -50.0f) {
        int s = (int)(speedA_rps * 100);
        RTT_LOG_ERROR("[Encoder] A speed abnormal: %d (0.01 r/s)", s);
    }
    if (speedB_rps > 50.0f || speedB_rps < -50.0f) {
        int s = (int)(speedB_rps * 100);
        RTT_LOG_ERROR("[Encoder] B speed abnormal: %d (0.01 r/s)", s);
    }
}
/* 本周期增量 (原有接口) */
int16_t Encoder_GetCountA(void)     { return encA_count; }
int16_t Encoder_GetCountB(void)     { return encB_count; }
float   Encoder_GetSpeedA_rps(void) { return speedA_rps; }
float   Encoder_GetSpeedB_rps(void) { return speedB_rps; }

float Encoder_GetSpeedAvg_ms(void) {
    return ((speedA_rps + speedB_rps) / 2.0f) * WHEEL_CIRCUMFERENCE;
}

/* ---- 位置环用的新接口 ---- */

/* 累积总脉冲数 */
int32_t Encoder_GetTotalA(void) { return totalA; }
int32_t Encoder_GetTotalB(void) { return totalB; }
int32_t Encoder_GetTotalAvg(void) { return (totalA + totalB) / 2; }

/* 累积距离 (米) */
float Encoder_GetDistA_m(void) {
    return ((float)totalA / 4.0f / (REDUCTION_RATIO * ENCODER_PPR)) * WHEEL_CIRCUMFERENCE;
}
float Encoder_GetDistB_m(void) {
    return ((float)totalB / 4.0f / (REDUCTION_RATIO * ENCODER_PPR)) * WHEEL_CIRCUMFERENCE;
}
float Encoder_GetDistAvg_m(void) {
    return (Encoder_GetDistA_m() + Encoder_GetDistB_m()) / 2.0f;
}

/* 位置清零 (用于设置原点) */
void Encoder_ResetDistance(void) {
    totalA = 0;
    totalB = 0;
    RTT_LOG_INFO("[Encoder] Distance reset to 0");
}
