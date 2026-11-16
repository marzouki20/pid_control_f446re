#include "encoder.h"

// Déclaration des timers (créés dans main.c)
extern TIM_HandleTypeDef htim2; // Encodeur Gauche
extern TIM_HandleTypeDef htim4; // Encodeur Droit

// Variables internes pour l'Encodeur Gauche (TIM2)
static uint16_t past_raw_G = 0;
static int32_t total_count_G = 0;

// Variables internes pour l'Encodeur Droit (TIM4)
static uint16_t past_raw_D = 0;
static int32_t total_count_D = 0;

// ==========================================
// INITIALISATION
// ==========================================
void Encoder_Init(void) {
    // --- Initialisation Encodeur Gauche (TIM2) ---
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    past_raw_G = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    total_count_G = 0;

    // --- Initialisation Encodeur Droit (TIM4) ---
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    past_raw_D = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    total_count_D = 0;
}

// ==========================================
// LECTURE ENCODEUR GAUCHE (TIM2)
// ==========================================
int32_t Encoder_GetCount_Gauche(void) {
    uint16_t current_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);

    if (current_raw != past_raw_G) {
        int16_t diff = (int16_t)(current_raw - past_raw_G);
        total_count_G += diff;
        past_raw_G = current_raw;
    }

    return total_count_G;
}

// ==========================================
// LECTURE ENCODEUR DROIT (TIM4)
// ==========================================
int32_t Encoder_GetCount_Droite(void) {
    uint16_t current_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);

    if (current_raw != past_raw_D) {
        int16_t diff = (int16_t)(current_raw - past_raw_D);
        total_count_D += diff;
        past_raw_D = current_raw;
    }

    return total_count_D;
}
