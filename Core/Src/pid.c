#include "pid.h"

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_max, float out_min) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->error_prev = 0.0f;
    pid->integral = 0.0f;
    pid->output_max = out_max;
    pid->output_min = out_min;
}

float PID_Compute(PID_Controller *pid, float setpoint, float measured_value, float dt) {
    // Calculate error
    float error = setpoint - measured_value;

    // Accumulate integral (anti-windup not included for simplicity, but keep bounded)
    pid->integral += error * dt;

    // Calculate derivative
    float derivative = (error - pid->error_prev) / dt;

    // Compute output
    float output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);

    // Clamp output
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    // Store error for next time
    pid->error_prev = error;

    return output;
}
