#include "pid.h"

void PID_Init(PID_Controller *pid,
              float Kp, float Ki, float Kd,
              float out_max, float integral_max) {
    pid->Kp           = Kp;
    pid->Ki           = Ki;
    pid->Kd           = Kd;
    pid->error        = 0.0f;
    pid->last_error   = 0.0f;
    pid->integral     = 0.0f;
    pid->output       = 0.0f;
    pid->output_max   = out_max;
    pid->integral_max = integral_max;
}

float PID_Compute(PID_Controller *pid,
                  float setpoint, float measured) {
    pid->error = setpoint - measured;

    // 积分累加（限幅防积分饱和）
    pid->integral += pid->error;
    if (pid->integral >  pid->integral_max)
        pid->integral =  pid->integral_max;
    if (pid->integral < -pid->integral_max)
        pid->integral = -pid->integral_max;

    // PID输出
    pid->output = pid->Kp * pid->error
                + pid->Ki * pid->integral
                + pid->Kd * (pid->error - pid->last_error);

    pid->last_error = pid->error;

    // 输出限幅
    if (pid->output >  pid->output_max)
        pid->output =  pid->output_max;
    if (pid->output < -pid->output_max)
        pid->output = -pid->output_max;

    return pid->output;
}

void PID_Reset(PID_Controller *pid) {
    pid->error      = 0.0f;
    pid->last_error = 0.0f;
    pid->integral   = 0.0f;
    pid->output     = 0.0f;
}

void PID_SetParams(PID_Controller *pid,
                   float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}
