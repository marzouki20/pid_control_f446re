#include "motor.h"

extern TIM_HandleTypeDef htim1;
#define MOTOR_MAX_PWM 8999

void Motor_Init(void) {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    __HAL_TIM_MOE_ENABLE(&htim1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}

void Motor_SetSpeed(float speed_percent) {
    // Limit speed to -100% to +100%
    if (speed_percent > 100.0f) speed_percent = 100.0f;
    if (speed_percent < -100.0f) speed_percent = -100.0f;

    uint32_t pwm_value = (uint32_t)((fabsf(speed_percent) * MOTOR_MAX_PWM) / 100.0f);

    if (speed_percent > 0.5f) {
        // FORWARD
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value); // RPWM
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);         // LPWM
    }
    else if (speed_percent < -0.5f) {
        // BACKWARD
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);         // RPWM
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_value); // LPWM
    }
    else {
        // STOP
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
}

void Motor_Stop(void) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}
