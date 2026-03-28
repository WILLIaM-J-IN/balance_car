#ifndef __PID_H
#define __PID_H

#include "main.h"

typedef struct {
    float Kp, Ki, Kd;
    float error;
    float last_error;
    float integral;
    float output;
    float output_max;
    float integral_max;
} PID_Controller;

void  PID_Init(PID_Controller *pid,
               float Kp, float Ki, float Kd,
               float out_max, float integral_max);
float PID_Compute(PID_Controller *pid,
                  float setpoint, float measured);
void  PID_Reset(PID_Controller *pid);
void  PID_SetParams(PID_Controller *pid,
                    float Kp, float Ki, float Kd);

#endif
