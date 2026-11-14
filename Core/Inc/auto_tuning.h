#ifndef INC_AUTO_TUNING_H_
#define INC_AUTO_TUNING_H_

#include "main.h"

// Structure pour stocker les résultats
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Tu; // Période d'oscillation
    float Ku; // Gain critique
    uint8_t is_done;
} AutoTuneResult;

// Fonction principale à appeler au démarrage
// Renvoie les gains calculés
AutoTuneResult AutoTune_Run(float target_position, float relay_amplitude, float noise_band, uint32_t timeout_ms);

#endif /* INC_AUTO_TUNING_H_ */
