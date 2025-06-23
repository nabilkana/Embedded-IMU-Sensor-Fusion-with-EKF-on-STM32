#ifndef MPU9250_H
#define MPU9250_H

#include "stm32f1xx_hal.h"  // Replace with your MCU's HAL header
#include <string.h>
#include <stdio.h>

#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_GYRO_XOUT_H  0x43
#define MPU9250_ADDR (0x68 << 1)
#define WHO_AM_I_REG  (0x75<< 1 )
#define PWR_MGMT_1 0x6B


typedef struct {
    float ax, ay, az;
} AccelData;

typedef struct {
    float gx, gy, gz;
} GyroData;
uint8_t MPU9250_Init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);
uint8_t MPU9250_ReadAccel(I2C_HandleTypeDef *hi2c, AccelData *data);
uint8_t MPU9250_ReadGyro(I2C_HandleTypeDef *hi2c, GyroData *data);
void uart_print(UART_HandleTypeDef *huart, const char *msg);


#endif
