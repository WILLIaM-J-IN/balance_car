#include "balance.h"
#include "mpu6886.h"
#include "encoder.h"
#include "motor.h"
#include "bluetooth.h"
#include "pid.h"
#include "rtt_log.h"
#include <math.h>

/*============================================================
 * 核心控制参数
 *============================================================*/
static PID_Controller pid_balance;
static PID_Controller pid_speed;
static float   angle_offset  = 0.0f;
static uint8_t fall_flag     = 0;

/**
 * 标准线性死区参数
 * 逻辑：|err| = 0 时补 0，|err| >= SLOPE_ANGLE 时补全额 base_dz
 */
#define DZ_FWD           40.0f  // 向前救车全额死区 (针对前倾问题)
#define DZ_BKW           40.0f   // 向后救车全额死区
#define SLOPE_ANGLE      1.5f    // 线性过渡区间 (单位：度)

/*============================================================
 * 初始化
 *============================================================*/
void Balance_Init(void)
{
    // 直立环 PD
    PID_Init(&pid_balance, BALANCE_KP, 0.0f, BALANCE_KD, MOTOR_MAX_PWM, 200.0f);

    // 速度环 PI (注意：若编码器B报错，请将SPEED_KP设为0)
    PID_Init(&pid_speed, SPEED_KP, SPEED_KI, 0.0f, 5.0f, 100.0f);

    fall_flag    = 0;
    angle_offset = 0.0f;

    RTT_LOG_INFO("[Balance] Reverted to Standard Linear Deadzone");
}

/*============================================================
 * 核心更新函数 (每 5ms 调用一次)
 *============================================================*/
void Balance_Update(void)
{
	/*--- Step 1: 数据采集 ---*/
	MPU6886_Update();
	Encoder_Update();

	float pitch      = MPU6886_GetPitch();   /* 保持原来这行不变 */
	float pitch_rate = MPU6886_GetPitchRate();
	float speed_ms   = Encoder_GetSpeedAvg_ms();

	/* Low-pass filter on pitch */
	static float pitch_filtered = 0.0f;
	pitch_filtered = 0.85f * pitch_filtered + 0.15f * pitch;
	pitch = pitch_filtered;   /* 直接覆盖，不重新声明 */

	BT_ControlData *bt = BT_GetControlData();

    /*--- Step 2: 跌落保护 ---*/
    if (pitch > FALL_ANGLE_THRESHOLD || pitch < -FALL_ANGLE_THRESHOLD)
    {
        Motor_Stop();
        PID_Reset(&pid_balance);
        PID_Reset(&pid_speed);
        fall_flag = 1;
        return;
    }
    fall_flag = 0;

    /*--- Step 3: 目标角度与外环补偿 ---*/
    float base_target = 2.5f; // 锁定物理平衡点
    float angle_comp = PID_Compute(&pid_speed, bt->speed_target, speed_ms);
    float final_setpoint = base_target + angle_offset + angle_comp;

    /*--- Step 4: 直立环核心 PD 计算 ---*/
    float err = pitch - final_setpoint;
    float balance_out = (pid_balance.Kp * err) - (pid_balance.Kd * pitch_rate);

    /*--- Step 5: 标准线性死区补偿 (误差越大，补偿越大) ---*/
    float abs_err = (float)fabs(err);
    float current_dz = 0;

    // 确定基础死区方向
    float base_dz = (balance_out > 0) ? DZ_FWD : DZ_BKW;

    // 线性缩放逻辑：在 SLOPE_ANGLE 度以内平滑增加
    if (abs_err < SLOPE_ANGLE) {
        current_dz = base_dz * (abs_err / SLOPE_ANGLE);
    } else {
        current_dz = base_dz; // 超过设定角度，全额补偿
    }

    // 应用补偿
    if (balance_out > 0.1f)       balance_out += current_dz;
    else if (balance_out < -0.1f) balance_out -= current_dz;
    else                          balance_out = 0;

    /*--- Step 6: 限幅与电机输出 ---*/
    if (balance_out >  (float)MOTOR_MAX_PWM) balance_out =  (float)MOTOR_MAX_PWM;
    if (balance_out < -(float)MOTOR_MAX_PWM) balance_out = -(float)MOTOR_MAX_PWM;

    int16_t turn_pwm = (int16_t)(bt->turn_target * TURN_KP);
    int16_t pwmA     = (int16_t)balance_out + turn_pwm;
    int16_t pwmB     = (int16_t)balance_out - turn_pwm;

    Motor_SetSpeed(pwmA, pwmB);

    /*--- Step 7: 调试日志 (整数打印防止乱码) ---*/
    static uint32_t log_cnt = 0;
    if (++log_cnt >= 200) {
        log_cnt = 0;
        int i_pitch = (int)pitch;
        int i_err   = (int)err;
        int i_pwm   = (int)balance_out;
        RTT_LOG_DEBUG("Pitch:%d | Err:%d | PWM:%d", i_pitch, i_err, i_pwm);
    }
}

void Balance_SetAngleOffset(float deg) { angle_offset = deg; }
