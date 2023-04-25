#ifndef PID_H_
#define PID_H_

#include <stdint.h>



typedef struct
{
    int32_t ki;
    int32_t kp;
    int32_t kd;

    int32_t T;

    int32_t integrator;
    int32_t prevError;

} PID_t;

void PID_Init(int32_t sample_time, int32_t kp, int32_t ki, int32_t kd);

int32_t PID_Run(int32_t curr_val, int32_t setpoint);

#endif
