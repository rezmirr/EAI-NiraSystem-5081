#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"

#define MPU6050_AXES			3U

typedef struct
{
  float ax, ay, az;   /* [g]    full scale +-16 g     */
  float gx, gy, gz;   /* [dps]  full scale +-2000 dps */
  float temp;         /* [degC]                       */
} mpu6050_t;

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c, uint8_t *who_am_i);
HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef *hi2c, mpu6050_t *d);
HAL_StatusTypeDef MPU6050_CalibrateGyro(I2C_HandleTypeDef *hi2c, uint16_t n, float bias[MPU6050_AXES]);

#endif /* MPU6050_H */
