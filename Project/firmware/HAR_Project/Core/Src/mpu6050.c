#include "mpu6050.h"

#define MPU_ADDR           (0x68U << 1)   /* AD0 = GND. If AD0 = VCC use 0x69 */

#define REG_SMPLRT_DIV     0x19U
#define REG_CONFIG         0x1AU
#define REG_GYRO_CONFIG    0x1BU
#define REG_ACCEL_CONFIG   0x1CU
#define REG_ACCEL_XOUT_H   0x3BU
#define REG_PWR_MGMT_1     0x6BU
#define REG_WHO_AM_I       0x75U

#define I2C_TIMEOUT_MS     10U

#define ACCEL_LSB_PER_G    2048.0f        /* ACCEL_CONFIG = 0x18 -> +-16 g    */
#define GYRO_LSB_PER_DPS   16.4f          /* GYRO_CONFIG  = 0x18 -> +-2000 dps */

static HAL_StatusTypeDef reg_write(I2C_HandleTypeDef *h, uint8_t reg, uint8_t val)
{
  return HAL_I2C_Mem_Write(h, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *h, uint8_t *who_am_i)
{
  HAL_StatusTypeDef st;
  uint8_t id = 0;

  st = HAL_I2C_IsDeviceReady(h, MPU_ADDR, 3, 50);
  if (st != HAL_OK) return st;

  st = HAL_I2C_Mem_Read(h, MPU_ADDR, REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &id, 1, I2C_TIMEOUT_MS);
  if (st != HAL_OK) return st;
  if (who_am_i) *who_am_i = id;

  if ((st = reg_write(h, REG_PWR_MGMT_1, 0x80U)) != HAL_OK) return st; /* device reset   */
  HAL_Delay(100);
  if ((st = reg_write(h, REG_PWR_MGMT_1, 0x01U)) != HAL_OK) return st; /* wake, PLL clk  */
  HAL_Delay(10);
  if ((st = reg_write(h, REG_CONFIG,       0x04U)) != HAL_OK) return st; /* DLPF ~21 Hz  */
  if ((st = reg_write(h, REG_SMPLRT_DIV,   0x00U)) != HAL_OK) return st; /* 1 kHz inside */
  if ((st = reg_write(h, REG_GYRO_CONFIG,  0x18U)) != HAL_OK) return st; /* +-2000 dps   */
  if ((st = reg_write(h, REG_ACCEL_CONFIG, 0x18U)) != HAL_OK) return st; /* +-16 g       */
  HAL_Delay(50);

  return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef *h, mpu6050_t *d)
{
  uint8_t b[14];
  int16_t r[7];
  HAL_StatusTypeDef st;
  int i;

  /* one burst: ACCEL(6) + TEMP(2) + GYRO(6) -> all values from the same instant */
  st = HAL_I2C_Mem_Read(h, MPU_ADDR, REG_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, b, 14, I2C_TIMEOUT_MS);
  if (st != HAL_OK) return st;

  for (i = 0; i < 7; i++)
  {
    r[i] = (int16_t)(((uint16_t)b[2 * i] << 8) | b[2 * i + 1]);
  }

  d->ax   = (float)r[0] / ACCEL_LSB_PER_G;
  d->ay   = (float)r[1] / ACCEL_LSB_PER_G;
  d->az   = (float)r[2] / ACCEL_LSB_PER_G;
  d->temp = (float)r[3] / 340.0f + 36.53f;
  d->gx   = (float)r[4] / GYRO_LSB_PER_DPS;
  d->gy   = (float)r[5] / GYRO_LSB_PER_DPS;
  d->gz   = (float)r[6] / GYRO_LSB_PER_DPS;

  return HAL_OK;
}

HAL_StatusTypeDef MPU6050_CalibrateGyro(I2C_HandleTypeDef *hi2c, uint16_t n, float bias[MPU6050_AXES]){
	mpu6050_t d;
	
	const float *g[MPU6050_AXES] = {&d.gx, &d.gy, &d.gz};
	float s[MPU6050_AXES] = {0.0f};
	uint16_t ok = 0;
	
	for(uint32_t i = 0; i < n; i++){
		if(MPU6050_ReadAll(hi2c, &d) == HAL_OK){
			for(uint32_t k = 0; k < MPU6050_AXES; k++) s[k] += *g[k];
			ok++;
		}
		HAL_Delay(5);
	}
	
	if(ok == 0U)	return HAL_ERROR;
	for(uint32_t k = 0; k < MPU6050_AXES; k++) bias[k] = s[k] / (float)ok;
	
	return HAL_OK;
}

