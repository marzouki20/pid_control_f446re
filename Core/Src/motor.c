#include "motor.h"

extern TIM_HandleTypeDef htim1;

// CHANGED: Max PWM is now 255
#define MOTOR_MAX_PWM 999

void Motor_Init(void) {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    __HAL_TIM_MOE_ENABLE(&htim1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}

void Motor_SetPWM(int32_t pwm_value) {
    // Limit to max hardware PWM (255)
    if (pwm_value > MOTOR_MAX_PWM) pwm_value = MOTOR_MAX_PWM;
    if (pwm_value < -MOTOR_MAX_PWM) pwm_value = -MOTOR_MAX_PWM;

    if (pwm_value > 5) {
        // FORWARD
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
    else if (pwm_value < -5) {
        // BACKWARD
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -pwm_value);
    }
    else {
        // STOP (Deadband)
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
}

void Motor_Stop(void) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}
