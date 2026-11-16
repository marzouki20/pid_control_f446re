#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"

// Initialisation des deux encodeurs
void Encoder_Init(void);

// Fonctions pour lire chaque encodeur indépendamment
int32_t Encoder_GetCount_Gauche(void);
int32_t Encoder_GetCount_Droite(void);

#endif /* INC_ENCODER_H_ */
