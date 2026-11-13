#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float error_prev;
    float integral;
    float output_max;
    float output_min;
} PID_Controller;

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_max, float out_min);
float PID_Compute(PID_Controller *pid, float setpoint, float measured_value, float dt);

#endif /* INC_PID_H_ */
