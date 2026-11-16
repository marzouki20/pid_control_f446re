#include "motor.h"

// Déclaration des timers
extern TIM_HandleTypeDef htim1; // Moteur 1 (Gauche)
extern TIM_HandleTypeDef htim3; // Moteur 2 (Droite)

#define MOTOR_MAX_PWM 999

// ==========================================
// INITIALISATION
// ==========================================
void Motor_Init(void) {
    // 1. Activer les drivers BTS7960 (R_EN et L_EN à HIGH)
    // Moteur 1 : PC8 (R_EN) et PC9 (L_EN)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);

    // Moteur 2 : PC6 (R_EN) et PC7 (L_EN)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);

    // 2. Démarrer les PWM
    // Moteur 1 : TIM1 (PA8 et PA9)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    // Moteur 2 : TIM3 (PA6 et PA7)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    __HAL_TIM_MOE_ENABLE(&htim1);

    // 3. S'assurer que tout est à l'arrêt
    Motor_Stop();
}

// ==========================================
// CONTRÔLE DU MOTEUR GAUCHE (TIM1 - PA8/PA9)
// ==========================================
void Motor_SetSpeed_Gauche(int32_t speed) {
    if (speed > MOTOR_MAX_PWM) speed = MOTOR_MAX_PWM;
    if (speed < -MOTOR_MAX_PWM) speed = -MOTOR_MAX_PWM;

    uint32_t pwm_value = (speed >= 0) ? speed : -speed;

    if (speed > 5) {
        // Avant
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value); // PA8
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);         // PA9
    }
    else if (speed < -5) {
        // Arrière
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);         // PA8
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_value); // PA9
    }
    else {
        // Arrêt
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
}

// ==========================================
// CONTRÔLE DU MOTEUR DROIT (TIM3 - PA6/PA7)
// ==========================================
void Motor_SetSpeed_Droite(int32_t speed) {
    if (speed > MOTOR_MAX_PWM) speed = MOTOR_MAX_PWM;
    if (speed < -MOTOR_MAX_PWM) speed = -MOTOR_MAX_PWM;

    uint32_t pwm_value = (speed >= 0) ? speed : -speed;

    if (speed > 5) {
        // Avant
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_value); // PA6
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);         // PA7
    }
    else if (speed < -5) {
        // Arrière
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);         // PA6
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_value); // PA7
    }
    else {
        // Arrêt
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }
}

// ==========================================
// ARRÊT D'URGENCE
// ==========================================
void Motor_Stop(void) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
}
