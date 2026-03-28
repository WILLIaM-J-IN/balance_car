#include "balance.h"
#include "mpu6886.h"
#include "encoder.h"
#include "motor.h"
#include "bluetooth.h"
#include "pid.h"
#include "rtt_log.h"

static PID_Controller pid_balance;
static PID_Controller pid_speed;
static float angle_offset  = 0.0f;
static uint8_t fall_flag   = 0;    // 跌倒标志

void Balance_Init(void) {
    PID_Init(&pid_balance,
             BALANCE_KP, BALANCE_KI, BALANCE_KD,
             MOTOR_MAX_PWM, 200.0f);

    PID_Init(&pid_speed,
             SPEED_KP, SPEED_KI, SPEED_KD,
             15.0f, 100.0f);

    fall_flag = 0;

    RTT_LOG_INFO("[Balance] 平衡控制初始化完成");
    RTT_LOG_INFO("[Balance] 直立环 Kp=%.1f Ki=%.1f Kd=%.1f",
                 BALANCE_KP, BALANCE_KI, BALANCE_KD);
    RTT_LOG_INFO("[Balance] 速度环 Kp=%.1f Ki=%.1f Kd=%.1f",
                 SPEED_KP, SPEED_KI, SPEED_KD);
    RTT_LOG_INFO("[Balance] 直立角度目标: %.1f 度 (offset=%.1f)",
                 BALANCE_ANGLE_SETPOINT, angle_offset);
}

void Balance_Update(void) {
    MPU6886_Update();
    Encoder_Update();

    float pitch    = MPU6886_GetPitch();
    float gyroY    = MPU6886_GetGyroY();
    float speed_ms = Encoder_GetSpeedAvg_ms();

    BT_ControlData *bt = BT_GetControlData();

    // 倾角保护
    if (pitch > FALL_ANGLE_THRESHOLD || pitch < -FALL_ANGLE_THRESHOLD) {
        if (!fall_flag) {
            RTT_LOG_ERROR("[Balance] 倾角过大！pitch=%.1f 超过阈值±%.1f°，触发保护停车",
                          pitch, FALL_ANGLE_THRESHOLD);
            fall_flag = 1;
        }
        Motor_Stop();
        PID_Reset(&pid_balance);
        PID_Reset(&pid_speed);
        return;
    }

    // 从跌倒状态恢复
    if (fall_flag) {
        RTT_LOG_INFO("[Balance] 姿态恢复正常 pitch=%.1f°，重新启动平衡控制", pitch);
        fall_flag = 0;
    }

    // 速度环
    float angle_comp = PID_Compute(&pid_speed,
                                    bt->speed_target,
                                    speed_ms);

    // 直立环目标角
    float angle_setpoint = BALANCE_ANGLE_SETPOINT
                         + angle_offset
                         + angle_comp;

    // 直立环输出（D项用陀螺仪）
    float balance_out = pid_balance.Kp * (angle_setpoint - pitch)
                      - pid_balance.Kd * gyroY;

    // 输出限幅
    if (balance_out >  MOTOR_MAX_PWM) balance_out =  MOTOR_MAX_PWM;
    if (balance_out < -MOTOR_MAX_PWM) balance_out = -MOTOR_MAX_PWM;

    // 转向叠加
    int16_t turn_pwm = (int16_t)(bt->turn_target * TURN_KP);
    int16_t pwmA = (int16_t)balance_out + turn_pwm;
    int16_t pwmB = (int16_t)balance_out - turn_pwm;

    Motor_SetSpeed(pwmA, pwmB);

    // 每200次（约1s）打印一次调试信息
    static uint32_t log_cnt = 0;
    if (++log_cnt >= 200) {
        log_cnt = 0;
        RTT_LOG_DEBUG("[Balance] pitch=%.2f° gyroY=%.1f speed=%.2fm/s "
                      "pwmA=%d pwmB=%d",
                      pitch, gyroY, speed_ms, pwmA, pwmB);
    }
}

void Balance_SetAngleOffset(float deg) {
    angle_offset = deg;
    RTT_LOG_INFO("[Balance] 直立角度偏移更新: %.2f°", deg);
}
