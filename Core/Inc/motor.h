#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

void Motor_Init(void);
void Motor_SetSpeed(float speed_percent); // Positive = Forward, Negative = Backward
void Motor_Stop(void);

#endif /* INC_MOTOR_H_ */
