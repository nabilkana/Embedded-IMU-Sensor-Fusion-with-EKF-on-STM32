#include "accel.h"
#include "shared_buffers.h"

static int16_t combine_bytes(uint8_t high, uint8_t low) {
    return ((int16_t)high << 8) | low;
}

void uart_print(UART_HandleTypeDef *huart, const char *msg) {
    HAL_UART_Transmit(huart, (uint8_t *)msg, strlen(msg), 100);

}

uint8_t MPU9250_Init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart) {
    uint8_t wake_cmd = 0x00;
    uint8_t who_am_i = 0;
    uart_print(huart, "Trying to wake MPU9250\r\n");

    // Wake up device
    if (HAL_I2C_Mem_Write(hi2c, MPU9250_ADDR, 0x6B, 1, &wake_cmd, 1, 1000) != HAL_OK) {
        uart_print(huart, "MPU9250: Failed to wake up\r\n");
        return 0;
    }

    HAL_Delay(100); // Let sensor stabilize

    // Read WHO_AM_I register
    if (HAL_I2C_Mem_Read(hi2c, MPU9250_ADDR, 0x75, 1, &who_am_i, 1, HAL_MAX_DELAY) != HAL_OK) {
        uart_print(huart, "MPU9250: Failed to read WHO_AM_I\r\n");
        return 0;
    }



    uart_print(huart, "READY\n");
    return 1;
}

uint8_t MPU9250_ReadAccel(I2C_HandleTypeDef *hi2c, AccelData *data) {
    uint8_t raw[6];
    if (HAL_I2C_Mem_Read(hi2c, MPU9250_ADDR, MPU9250_ACCEL_XOUT_H, 1, raw, 6, 100) != HAL_OK)
        return 0;

    data->ax = combine_bytes(raw[0], raw[1]) / 16384.0f;
    data->ay = combine_bytes(raw[2], raw[3]) / 16384.0f;
    data->az = combine_bytes(raw[4], raw[5]) / 16384.0f;
    return 1;
}

uint8_t MPU9250_ReadGyro(I2C_HandleTypeDef *hi2c, GyroData *data) {
    uint8_t raw[6];
    if (HAL_I2C_Mem_Read(hi2c, MPU9250_ADDR, MPU9250_GYRO_XOUT_H, 1, raw, 6, 100) != HAL_OK)
        return 0;

    data->gx = combine_bytes(raw[0], raw[1]) / 131.0f;
    data->gy = combine_bytes(raw[2], raw[3]) / 131.0f;
    data->gz = combine_bytes(raw[4], raw[5]) / 131.0f;
    return 1;
}










