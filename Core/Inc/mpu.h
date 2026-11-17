#ifndef INC_MPU_H_
#define INC_MPU_H_

#include "main.h"

// Adresse I2C du MPU6050 (0x68 si AD0 est à GND, 0x69 si à VCC)
#define MPU6050_ADDR (0x68 << 1)

void MPU_Init(I2C_HandleTypeDef *hi2c);
float MPU_Get_Yaw(void);
void MPU_Calibrate(void);

#endif /* INC_MPU_H_ */
