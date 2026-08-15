#ifndef DRIVER_IMU_H
#define DRIVER_IMU_H

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"

// MPU6050 地址
#define MPU6050_ADDRESS          0x68

// 寄存器地址
#define MPU_WHO_AM_I_REG         0x75
#define MPU_PWR_MGMT1_REG        0x6B
#define MPU_PWR_MGMT2_REG        0x6C
#define MPU_SAMPLE_RATE_REG      0x19
#define MPU_CFG_REG              0x1A
#define MPU_GYRO_CFG_REG         0x1B
#define MPU_ACCEL_CFG_REG        0x1C
#define MPU_GYRO_ZOUT_H_REG      0x47
#define MPU_GYRO_ZOUT_L_REG      0x48

// I2C 超时时间
#define I2C_TIMEOUT              100000

// 上电延迟
#define MPU6050_PWR_UP_DELAY_MS  50

bool MPU6050_Init(void);
bool MPU6050_Read_GyroZ(int16_t *gyro_z);
void MPU6050_Update_Yaw(float *yaw);
void MPU6050_Calibrate_GyroZ(int samples);
void delay_ms(uint32_t ms);

#endif