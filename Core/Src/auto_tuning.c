#include "auto_tuning.h"
#include "motor.h"
#include "encoder.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;

AutoTuneResult AutoTune_Run(float target_position, float relay_amplitude, float noise_band, uint32_t timeout_ms) {
    AutoTuneResult result = {2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0};

    __HAL_TIM_SET_COUNTER(&htim3, 0);
    uint32_t start_time = HAL_GetTick();
    uint32_t last_cross_time = 0;
    float total_period = 0;
    int cross_count = 0;
    int last_direction = 0;

    HAL_Delay(300);

    // Boucle d'oscillation
    while (HAL_GetTick() - start_time < timeout_ms) {
        float pos = (float)Encoder_GetCount();
        int current_dir = 0;

        if (pos < (target_position - noise_band)) {
            current_dir = 1;
        } else if (pos > (target_position + noise_band)) {
            current_dir = -1;
        }

        if (current_dir != 0 && current_dir != last_direction) {
            if (last_cross_time != 0) {
                total_period += (HAL_GetTick() - last_cross_time);
                cross_count++;
            }
            last_cross_time = HAL_GetTick();
            last_direction = current_dir;
        }

        if (current_dir == 1) {
            Motor_SetPWM((int32_t)relay_amplitude);
        } else if (current_dir == -1) {
            Motor_SetPWM((int32_t)-relay_amplitude);
        }

        HAL_Delay(5);
    }

    Motor_Stop();
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    // Calcul des gains
    if (cross_count >= 2) {
        result.Tu = (total_period / cross_count) / 1000.0f;
        result.Ku = (4.0f * relay_amplitude) / (3.14159f * 800.0f);

        result.Kp = 0.6f * result.Ku;
        result.Ki = 1.2f * result.Ku / result.Tu;
        result.Kd = 0.075f * result.Ku * result.Tu;

        result.is_done = 1;
    } else {
        result.Kp = 2.0f;
        result.Ki = 0.0f;
        result.Kd = 0.0f;
        result.is_done = 0;
    }

    return result;
}
