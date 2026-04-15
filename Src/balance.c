#include "balance.h"
#include "mpu6886.h"
#include "encoder.h"
#include "motor.h"
#include "bluetooth.h"
#include "pid.h"
#include "rtt_log.h"
#include <math.h>

/* ============================================================
 *  三级串级 PID 平衡控制
 *  ┌──────────┐                  ┌──────────┐
 *  │ 位置环 PD │ (可选,启用时)     │          │
 *  └──────────┘ ───┐              │          │
 *                  ├── angle_comp → 直立环 PD │──▶ PWM
 *  ┌──────────┐   ┘              │          │
 *  │ 速度环 PI │ (未启用位置环时) │          │
 *  └──────────┘                  └──────────┘
 *
 *  调用频率: Balance_Update() 必须固定 5ms 周期
 *  外环每 SPEED_LOOP_DIV 次调用执行一次
 * ============================================================ */

/* ---------- 可调参数 ---------- */

#define SPEED_OUT_LIMIT     5.0f     /* 速度环输出限幅(度) */
#define STARTUP_SETTLE_CNT  200      /* 启动稳定期,5ms*200=1s */
#define SPEED_LOOP_DIV      2        /* 外环分频,每 N 次主循环跑一次 */

/* ---------- 模块状态 ---------- */
static PID_Controller pid_balance;
static PID_Controller pid_speed;
static float    angle_offset    = 0.0f;
static uint8_t  fall_flag       = 0;
static uint32_t startup_counter = 0;
static float    angle_comp      = 0.0f;   /* 外环输出,主循环间保持 */

/* ---------- 位置环状态 ---------- */
static uint8_t  pos_hold_enabled = 0;
static float    pos_setpoint_m   = 0.0f;
static float    pos_error_last   = 0.0f;   /* 调试/查询用 */
static uint8_t  pos_auto_mode    = 0;     /* 1=自动模式启用 */
static float    pos_auto_origin  = 0.0f;  /* 自动模式的"原点" */

/* ============================================================
 *  初始化
 * ============================================================ */
void Balance_Init(void)
{
    /* 直立环: 纯 PD */
    PID_Init(&pid_balance, BALANCE_KP, 0.0f, BALANCE_KD,
             MOTOR_MAX_PWM, 0.0f);

    /* 速度环: PI */
    PID_Init(&pid_speed, SPEED_KP, SPEED_KI, 0.0f,
             SPEED_OUT_LIMIT, SPEED_OUT_LIMIT * 0.5f);

    fall_flag        = 0;
    angle_offset     = 0.0f;
    angle_comp       = 0.0f;
    startup_counter  = 0;
    pos_hold_enabled = 0;
    pos_setpoint_m   = 0.0f;
    pos_error_last   = 0.0f;

    RTT_LOG_INFO("[Balance] Cascade PID init OK (pos loop ready)");
}

/* ============================================================
 *  主控制函数 - 必须 5ms 周期调用
 * ============================================================ */
void Balance_Update(void)
{
    /* ---------- Step 1: 数据采集 ---------- */
    MPU6886_Update();
    Encoder_Update();

    float pitch      = MPU6886_GetPitch();
    float pitch_rate = MPU6886_GetPitchRate();
    float speed_ms   = Encoder_GetSpeedAvg_ms();

    /* 角度轻量低通 */
    static float pitch_lpf = 0.0f;
    pitch_lpf = 0.7f * pitch_lpf + 0.3f * pitch;
    pitch = pitch_lpf;

    /* ---------- Step 2: 启动稳定期 ---------- */
    if (startup_counter < STARTUP_SETTLE_CNT) {
        startup_counter++;
        Motor_Stop();
        PID_Reset(&pid_balance);
        PID_Reset(&pid_speed);
        angle_comp = 0.0f;
        return;
    }

    BT_ControlData *bt = BT_GetControlData();

    /* ---------- Step 3: 跌倒保护 ---------- */
    if (pitch > FALL_ANGLE_THRESHOLD || pitch < -FALL_ANGLE_THRESHOLD) {
        Motor_Stop();
        PID_Reset(&pid_balance);
        PID_Reset(&pid_speed);
        angle_comp       = 0.0f;
        pos_hold_enabled = 0;          /* 摔倒自动禁用位置环 */
        fall_flag        = 1;
        return;
    }
    /* 扶正后等车接近垂直才恢复 */
    if (pitch > FALL_ANGLE_THRESHOLD || pitch < -FALL_ANGLE_THRESHOLD) {
            Motor_Stop();
            PID_Reset(&pid_balance);
            PID_Reset(&pid_speed);
            angle_comp       = 0.0f;
            pos_hold_enabled = 0;
            pos_auto_mode    = 0;        /* ← 新增,跌倒退出自动模式 */
            fall_flag        = 1;
            return;
        }

    /* ---------- Step 4: 外环控制 ---------- */
       static uint8_t ctrl_div = 0;
       if (++ctrl_div >= SPEED_LOOP_DIV) {
           ctrl_div = 0;

           /* 自动模式: 检测位置偏移,决定是否启动位置环 */
           if (pos_auto_mode) {
               float current_pos = Encoder_GetDistAvg_m();
               float drift_m     = current_pos - pos_auto_origin;
               float drift_mm    = drift_m * 1000.0f;
               float abs_drift   = (drift_mm < 0) ? -drift_mm : drift_mm;

               if (!pos_hold_enabled && abs_drift >= POS_TRIGGER_MM) {
                   /* 偏移过大,启动位置环 */
                   pos_setpoint_m   = pos_auto_origin;
                   pos_hold_enabled = 1;
                   RTT_LOG_INFO("[Balance] Auto POS ON (drift=%d mm)", (int)drift_mm);
               }
               else if (pos_hold_enabled && abs_drift <= POS_RELEASE_MM) {
                   /* 已回到附近,退回速度环模式 */
                   pos_hold_enabled = 0;
                   angle_comp       = 0.0f;
                   PID_Reset(&pid_speed);     /* 清速度环积分,避免冲击 */
                   RTT_LOG_INFO("[Balance] Auto POS OFF (drift=%d mm)", (int)drift_mm);
               }
           }

           /* 真正的环路计算 */
           if (pos_hold_enabled) {
               /* ---- 位置环 PD ---- */
               float current_pos = Encoder_GetDistAvg_m();
               float pos_err     = pos_setpoint_m - current_pos;
               pos_error_last    = pos_err;

               angle_comp = POS_KP * pos_err - POS_KD * speed_ms;

               if (angle_comp >  POS_OUT_LIMIT) angle_comp =  POS_OUT_LIMIT;
               if (angle_comp < -POS_OUT_LIMIT) angle_comp = -POS_OUT_LIMIT;

           } else {
               /* ---- 速度环 PI ---- */
               angle_comp = PID_Compute(&pid_speed, bt->speed_target, speed_ms);

               if (angle_comp >  SPEED_OUT_LIMIT) angle_comp =  SPEED_OUT_LIMIT;
               if (angle_comp < -SPEED_OUT_LIMIT) angle_comp = -SPEED_OUT_LIMIT;
           }
       }

    /* ---------- Step 5: 直立环 PD (每周期) ---------- */
    float final_setpoint = MECH_ZERO_ANGLE + angle_offset + angle_comp;
    float err            = pitch - final_setpoint;

    float balance_out = pid_balance.Kp * err + pid_balance.Kd * pitch_rate;

    /* ---------- Step 6: 输出限幅 ---------- */
    if (balance_out >  (float)MOTOR_MAX_PWM) balance_out =  (float)MOTOR_MAX_PWM;
    if (balance_out < -(float)MOTOR_MAX_PWM) balance_out = -(float)MOTOR_MAX_PWM;

    /* ---------- Step 7: 转向叠加并下发 ---------- */
    int16_t turn_pwm = (int16_t)(bt->turn_target * TURN_KP);
    int16_t pwmA     = (int16_t)balance_out + turn_pwm;
    int16_t pwmB     = (int16_t)balance_out - turn_pwm;

    Motor_SetSpeed(pwmA, pwmB);

    /* ---------- Step 8: 调试日志 (1s 一次) ---------- */
    static uint32_t log_cnt = 0;
    if (++log_cnt >= 200) {
        log_cnt = 0;
        if (pos_hold_enabled) {
            int err_mm = (int)(pos_error_last * 1000);
            int tgt_mm = (int)(pos_setpoint_m * 1000);
            RTT_LOG_DEBUG("POS P:%d Cmp:%d(0.1deg) PWM:%d Tgt:%d mm Err:%d mm",
                          (int)pitch, (int)(angle_comp * 10),
                          (int)balance_out, tgt_mm, err_mm);
        } else {
            RTT_LOG_DEBUG("SPD P:%d Set:%d Err:%d Cmp:%d PWM:%d Spd:%d",
                          (int)pitch,
                          (int)final_setpoint,
                          (int)err,
                          (int)angle_comp,
                          (int)balance_out,
                          (int)(speed_ms * 100));
        }
    }
}

/*============================================================
 *  角度偏置接口 (保留)
 *============================================================*/
void Balance_SetAngleOffset(float deg) { angle_offset = deg; }

/*============================================================
 *  位置环 API 实现
 *============================================================*/
void Balance_HoldPosition(void)
{
    pos_setpoint_m   = Encoder_GetDistAvg_m();
    pos_hold_enabled = 1;
    angle_comp       = 0.0f;                 /* 清一下外环输出,避免切换冲击 */
    int pos_mm = (int)(pos_setpoint_m * 1000);
    RTT_LOG_INFO("[Balance] Position HOLD at %d mm", pos_mm);
}

void Balance_MoveDistance(float meters)
{
    pos_setpoint_m   = Encoder_GetDistAvg_m() + meters;
    pos_hold_enabled = 1;
    angle_comp       = 0.0f;
    int d_mm   = (int)(meters * 1000);
    int tgt_mm = (int)(pos_setpoint_m * 1000);
    RTT_LOG_INFO("[Balance] MOVE %d mm -> target %d mm", d_mm, tgt_mm);
}

void Balance_SetTargetPosition(float meters)
{
    pos_setpoint_m   = meters;
    pos_hold_enabled = 1;
    angle_comp       = 0.0f;
    int tgt_mm = (int)(pos_setpoint_m * 1000);
    RTT_LOG_INFO("[Balance] Set target %d mm", tgt_mm);
}

void Balance_DisablePosition(void)
{
    pos_hold_enabled = 0;
    angle_comp       = 0.0f;
    RTT_LOG_INFO("[Balance] Position loop DISABLED");
}

uint8_t Balance_IsPositionHoldActive(void) { return pos_hold_enabled; }
float   Balance_GetPositionError(void)     { return pos_error_last; }


void Balance_EnableAutoHold(void)
{
    pos_auto_origin  = Encoder_GetDistAvg_m();
    pos_auto_mode    = 1;
    pos_hold_enabled = 0;     /* 初始不启用,等检测到偏移再触发 */
    int origin_mm = (int)(pos_auto_origin * 1000);
    RTT_LOG_INFO("[Balance] Auto-Hold mode ON, origin=%d mm", origin_mm);
}

void Balance_DisableAutoHold(void)
{
    pos_auto_mode    = 0;
    pos_hold_enabled = 0;
    angle_comp       = 0.0f;
    RTT_LOG_INFO("[Balance] Auto-Hold mode OFF");
}

uint8_t Balance_IsAutoHoldActive(void) { return pos_auto_mode; }
