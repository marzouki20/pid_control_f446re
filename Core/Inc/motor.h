#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

void Motor_Init(void);
void Motor_SetPWM(int32_t pwm_value); // -255 to +255
void Motor_Stop(void);

#endif /* INC_MOTOR_H_ */
