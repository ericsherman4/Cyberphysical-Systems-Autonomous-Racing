#include "pid.h"

static PID_t pid = {0};

void PID_Init(int32_t sample_time, int32_t kp, int32_t ki, int32_t kd)
{
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
    pid.T = sample_time;
}

int32_t PID_Run(int32_t curr_val, int32_t setpoint)
{
    return 0;
}
