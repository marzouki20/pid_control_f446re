#include "mpu.h"
#include <math.h>

static I2C_HandleTypeDef *mpu_i2c;
static float gyro_z_offset = 0.0f;
static float yaw = 0.0f;
static uint32_t last_time = 0;

// Fonction pour écrire un registre
static void MPU_WriteReg(uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(mpu_i2c, MPU6050_ADDR, reg, 1, &data, 1, 100);
}

// Fonction pour lire un registre
static void MPU_ReadRegs(uint8_t reg, uint8_t *data, uint8_t len) {
    HAL_I2C_Mem_Read(mpu_i2c, MPU6050_ADDR, reg, 1, data, len, 100);
}

void MPU_Init(I2C_HandleTypeDef *hi2c) {
    mpu_i2c = hi2c;

    // Réveiller le MPU6050
    MPU_WriteReg(0x6B, 0x00);
    HAL_Delay(100);

    // Configurer le gyroscope à +/- 500 °/s
    MPU_WriteReg(0x1B, 0x08);

    // Configurer le filtre passe-bas (DLPF) pour lisser le bruit
    MPU_WriteReg(0x1A, 0x03);

    yaw = 0.0f;
    last_time = HAL_GetTick();
}

void MPU_Calibrate(void) {
    int32_t sum_z = 0;
    uint8_t data[2];
    int16_t gyro_z;

    // On fait une moyenne sur 200 lectures pour trouver l'offset
    for (int i = 0; i < 200; i++) {
        MPU_ReadRegs(0x47, data, 2); // 0x47 = GYRO_ZOUT_H
        gyro_z = (int16_t)((data[0] << 8) | data[1]);
        sum_z += gyro_z;
        HAL_Delay(5);
    }
    gyro_z_offset = (float)sum_z / 200.0f;
    yaw = 0.0f;
}

float MPU_Get_Yaw(void) {
    uint8_t data[2];
    int16_t gyro_z_raw;

    // Lire la vitesse angulaire Z
    MPU_ReadRegs(0x47, data, 2);
    gyro_z_raw = (int16_t)((data[0] << 8) | data[1]);

    // Soustraire l'offset de calibration
    float gyro_z = (float)gyro_z_raw - gyro_z_offset;

    // Conversion en °/s (sensibilité 65.5 LSB/(°/s) pour +/- 500 °/s)
    float gyro_z_dps = gyro_z / 65.5f;

    // Intégration pour avoir l'angle (Yaw)
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_time) / 1000.0f;
    last_time = current_time;

    yaw += gyro_z_dps * dt;

    return yaw;
}
