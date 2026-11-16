#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

void Motor_Init(void);
void Motor_Stop(void);

// Contrôle des moteurs (-999 à +999)
void Motor_SetSpeed_Gauche(int32_t speed);
void Motor_SetSpeed_Droite(int32_t speed);

#endif /* INC_MOTOR_H_ */
