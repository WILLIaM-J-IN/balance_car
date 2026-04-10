#include "balance.h"
#include "mpu6886.h"
#include "encoder.h"
#include "motor.h"
#include "bluetooth.h"
#include "pid.h"
#include "rtt_log.h"
#include <math.h>

/* ============================================================
 *  串级 PID 平衡控制
 *  ┌──────────┐  speed_err  ┌──────────┐ angle_setpoint
 *  │ 速度环 PI │────────────▶│ 直立环 PD │──▶ PWM
 *  └──────────┘             └──────────┘
 *
 *  调用频率: Balance_Update() 必须固定 5ms 周期
 *  速度环每 5 次调用执行一次 (=25ms)
 * ============================================================ */

/* ---------- 可调参数 (建议从这里开始整定) ---------- */


/* 速度环输出限幅(单位:度) —— 防止速度环把目标角度推太狠导致系统震荡 */
#define SPEED_OUT_LIMIT     6.0f

/* 启动稳定期: 上电后丢弃前 N 次 (5ms*200=1s) 滤波器/IMU 收敛 */
#define STARTUP_SETTLE_CNT  200

/* 速度环分频: 每 N 次主循环跑一次速度环 */
#define SPEED_LOOP_DIV      5

/* ---------- 模块状态 ---------- */
static PID_Controller pid_balance;
static PID_Controller pid_speed;
static float    angle_offset    = 0.0f;
static uint8_t  fall_flag       = 0;
static uint32_t startup_counter = 0;
static float    angle_comp      = 0.0f;   /* 速度环输出,在主循环间保持 */

/* ============================================================
 *  初始化
 * ============================================================ */
void Balance_Init(void)
{
    /* 直立环: 纯 PD,积分项不要,Imax=0 */
    PID_Init(&pid_balance, BALANCE_KP, 0.0f, BALANCE_KD,
             MOTOR_MAX_PWM, 0.0f);

    /* 速度环: PI,积分限幅约为输出限幅的 50% */
    PID_Init(&pid_speed, SPEED_KP, SPEED_KI, 0.0f,
             SPEED_OUT_LIMIT, SPEED_OUT_LIMIT * 0.5f);

    fall_flag       = 0;
    angle_offset    = 0.0f;
    angle_comp      = 0.0f;
    startup_counter = 0;

    RTT_LOG_INFO("[Balance] Cascade PID init OK");
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

    /* 角度做轻量低通(系数 0.3),保留高频以免破坏 Kd 相位 */
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
        angle_comp = 0.0f;
        fall_flag  = 1;
        return;
    }
    /* 跌倒后被扶正,等回到接近垂直再恢复,避免反复救车 */
    if (fall_flag) {
        if (pitch > -2.0f && pitch < 2.0f) {
            fall_flag = 0;
        } else {
            Motor_Stop();
            return;
        }
    }

    /* ---------- Step 4: 速度环 (降频运行) ---------- */
    static uint8_t speed_div = 0;
    if (++speed_div >= SPEED_LOOP_DIV) {
        speed_div = 0;
        /* 注意符号: 当小车前倾(pitch>0)时车体加速向前,
         * 速度环要把目标角度往后拉一点来抵消累积速度。
         * 这里假设 PID_Compute(setpoint, measure) = Kp*(setpoint-measure) */
        angle_comp = PID_Compute(&pid_speed, bt->speed_target, speed_ms);

        if (angle_comp >  SPEED_OUT_LIMIT) angle_comp =  SPEED_OUT_LIMIT;
        if (angle_comp < -SPEED_OUT_LIMIT) angle_comp = -SPEED_OUT_LIMIT;
    }

    /* ---------- Step 5: 直立环 (PD,每周期都跑) ---------- */
    float final_setpoint = MECH_ZERO_ANGLE + angle_offset + angle_comp;
    float err            = pitch - final_setpoint;

    /* 直立环手写 PD,避免依赖 PID_Compute 内部状态 */
    float balance_out = pid_balance.Kp * err + pid_balance.Kd * pitch_rate;
    /* 注意:pitch_rate 与 err 同号时表示角度正在恶化,
     * 所以 Kd 项 与 Kp 项 同号相加(都把车往反方向推)。
     * 如果你的陀螺仪轴定义相反,把 + 改成 - 即可。 */

    /* ---------- Step 6: 输出限幅 ---------- */
    if (balance_out >  (float)MOTOR_MAX_PWM) balance_out =  (float)MOTOR_MAX_PWM;
    if (balance_out < -(float)MOTOR_MAX_PWM) balance_out = -(float)MOTOR_MAX_PWM;

    /* ---------- Step 7: 转向叠加并下发 ----------
     * 死区补偿统一交给 motor.c 的 apply_deadzone() 处理,
     * 这里不再做任何死区相关运算!
     */
    int16_t turn_pwm = (int16_t)(bt->turn_target * TURN_KP);
    int16_t pwmA     = (int16_t)balance_out + turn_pwm;
    int16_t pwmB     = (int16_t)balance_out - turn_pwm;

    Motor_SetSpeed(pwmA, pwmB);

    /* ---------- Step 8: 调试日志 (1s 一次) ---------- */
    static uint32_t log_cnt = 0;
    if (++log_cnt >= 200) {
        log_cnt = 0;
        RTT_LOG_DEBUG("P:%d Set:%d Err:%d Cmp:%d PWM:%d Spd:%d",
                      (int)pitch,
                      (int)final_setpoint,
                      (int)err,
                      (int)angle_comp,
                      (int)balance_out,
                      (int)(speed_ms * 100));
    }
}

void Balance_SetAngleOffset(float deg) { angle_offset = deg; }
