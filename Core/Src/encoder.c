#include "encoder.h"

// Tell the compiler that htim3 exists in main.c
extern TIM_HandleTypeDef htim3;

static uint16_t past_raw = 0;
static int32_t total_count = 0;

void Encoder_Init(void) {
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    past_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    total_count = 0;
}

int32_t Encoder_GetCount(void) {
    uint16_t current_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

    if (current_raw != past_raw) {
        int16_t diff = (int16_t)(current_raw - past_raw);
        total_count += diff;
        past_raw = current_raw;
    }

    return total_count;
}
