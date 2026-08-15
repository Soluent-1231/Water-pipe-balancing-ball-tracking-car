#include "driver_imu.h"
#include <stdio.h>

// 陀螺仪零偏校准值（静态变量）
static int16_t gyro_z_offset = 0;

/**
 * @brief 毫秒延时
 * @param ms 毫秒数
 */
void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t j = 0; j < 16000; j++);  // 80MHz 时钟约 1ms
    }
}

/**
 * @brief 写 MPU6050 寄存器
 * @param reg_addr 寄存器地址
 * @param data 要写入的数据
 * @return true:成功 false:失败
 */
bool MPU6050_Write_Register(uint8_t reg_addr, uint8_t data)
{
    uint8_t buffer[2];
    buffer[0] = reg_addr;
    buffer[1] = data;

    DL_I2C_fillControllerTXFIFO(I2C_1_INST, buffer, 2);
    DL_I2C_startControllerTransfer(I2C_1_INST, MPU6050_ADDRESS, 
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    uint32_t timeout = I2C_TIMEOUT;
    while (!(DL_I2C_getControllerStatus(I2C_1_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return false;
    }

    if (DL_I2C_getControllerStatus(I2C_1_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_disableController(I2C_1_INST);
        DL_I2C_enableController(I2C_1_INST);
        return false;
    }

    return true;
}

/**
 * @brief 读 MPU6050 单寄存器
 * @param reg_addr 寄存器地址
 * @param data 读取的数据指针
 * @return true:成功 false:失败
 */
bool MPU6050_Read_Register(uint8_t reg_addr, uint8_t *data)
{
    DL_I2C_fillControllerTXFIFO(I2C_1_INST, &reg_addr, 1);
    DL_I2C_startControllerTransfer(I2C_1_INST, MPU6050_ADDRESS, 
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    uint32_t timeout = I2C_TIMEOUT;
    while (!(DL_I2C_getControllerStatus(I2C_1_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return false;
    }

    if (DL_I2C_getControllerStatus(I2C_1_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_disableController(I2C_1_INST);
        DL_I2C_enableController(I2C_1_INST);
        return false;
    }

    DL_I2C_startControllerTransfer(I2C_1_INST, MPU6050_ADDRESS, 
                                   DL_I2C_CONTROLLER_DIRECTION_RX, 1);

    timeout = I2C_TIMEOUT;
    while (!(DL_I2C_getControllerStatus(I2C_1_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return false;
    }

    if (DL_I2C_getControllerStatus(I2C_1_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_disableController(I2C_1_INST);
        DL_I2C_enableController(I2C_1_INST);
        return false;
    }

    if (!DL_I2C_isControllerRXFIFOEmpty(I2C_1_INST)) {
        *data = DL_I2C_receiveControllerData(I2C_1_INST);
        return true;
    }

    return false;
}

/**
 * @brief 读 MPU6050 多个连续寄存器
 * @param start_reg 起始寄存器地址
 * @param data 数据缓冲区
 * @param len 读取字节数
 * @return true:成功 false:失败
 */
bool MPU6050_Read_Registers(uint8_t start_reg, uint8_t *data, uint8_t len)
{
    DL_I2C_fillControllerTXFIFO(I2C_1_INST, &start_reg, 1);
    DL_I2C_startControllerTransfer(I2C_1_INST, MPU6050_ADDRESS, 
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    uint32_t timeout = I2C_TIMEOUT;
    while (!(DL_I2C_getControllerStatus(I2C_1_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return false;
    }

    if (DL_I2C_getControllerStatus(I2C_1_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_disableController(I2C_1_INST);
        DL_I2C_enableController(I2C_1_INST);
        return false;
    }

    DL_I2C_startControllerTransfer(I2C_1_INST, MPU6050_ADDRESS, 
                                   DL_I2C_CONTROLLER_DIRECTION_RX, len);

    timeout = I2C_TIMEOUT;
    while (!(DL_I2C_getControllerStatus(I2C_1_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return false;
    }

    if (DL_I2C_getControllerStatus(I2C_1_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_disableController(I2C_1_INST);
        DL_I2C_enableController(I2C_1_INST);
        return false;
    }

    for (uint8_t i = 0; i < len; i++) {
        if (DL_I2C_isControllerRXFIFOEmpty(I2C_1_INST)) {
            return false;
        }
        data[i] = DL_I2C_receiveControllerData(I2C_1_INST);
    }

    return true;
}

/**
 * @brief 读取陀螺仪 Z 轴原始数据
 * @param gyro_z 陀螺仪 Z 轴数据指针（单位：LSB）
 * @return true:成功 false:失败
 */
bool MPU6050_Read_GyroZ(int16_t *gyro_z)
{
    uint8_t buffer[2];

    if (!MPU6050_Read_Registers(MPU_GYRO_ZOUT_H_REG, buffer, 2)) {
        return false;
    }

    *gyro_z = (int16_t)((buffer[0] << 8) | buffer[1]);
    return true;
}

/**
 * @brief 校准陀螺仪 Z 轴零偏
 * @param samples 采样次数
 */
void MPU6050_Calibrate_GyroZ(int samples)
{
    int32_t sum = 0;
    int16_t raw;
    int valid_samples = 0;

    for (int i = 0; i < samples; i++) {
        if (MPU6050_Read_GyroZ(&raw)) {
            sum += raw;
            valid_samples++;
        }
        delay_ms(5);
    }

    if (valid_samples > 0) {
        gyro_z_offset = sum / valid_samples;
    } else {
        gyro_z_offset = 0;
    }
}

/**
 * @brief 初始化 MPU6050
 * @return true:成功 false:失败
 */
bool MPU6050_Init(void)
{
    delay_ms(MPU6050_PWR_UP_DELAY_MS);

    uint8_t who_am_i;
    if (!MPU6050_Read_Register(MPU_WHO_AM_I_REG, &who_am_i)) {
        return false;
    }
    if (who_am_i != 0x68) {
        return false;
    }
		
    if (!MPU6050_Write_Register(MPU_PWR_MGMT1_REG, 0x03)) return false;
    if (!MPU6050_Write_Register(MPU_PWR_MGMT2_REG, 0x00)) return false;
    if (!MPU6050_Write_Register(MPU_SAMPLE_RATE_REG, 0x04)) return false;
    if (!MPU6050_Write_Register(MPU_CFG_REG, 0x03)) return false;
    if (!MPU6050_Write_Register(MPU_GYRO_CFG_REG, 0x08)) return false;
    if (!MPU6050_Write_Register(MPU_ACCEL_CFG_REG, 0x08)) return false;

    return true;
}

/**
 * @brief 更新 Yaw 角（使用固定时间间隔）
 * @param yaw 当前 yaw 角指针（单位：度）
 * @param dt 时间间隔（单位：秒），固定为 0.02s (50Hz)
 */
void MPU6050_Update_Yaw(float *yaw)
{
    int16_t gyro_z_raw;

    if (!MPU6050_Read_GyroZ(&gyro_z_raw))
    {
        return;
    }


    float gyro_z = ((float)gyro_z_raw - gyro_z_offset) / 65.5f;


		static float gyro_z_filter = 0;

		gyro_z_filter = gyro_z_filter * 0.8f + gyro_z * 0.2f;

		gyro_z = gyro_z_filter;


		if(fabs(gyro_z)<0.3f)
		{
				gyro_z=0;
		}


    float dt = 0.01f;


    *yaw += gyro_z * dt;


    if(*yaw > 180)
        *yaw -= 360;

    if(*yaw < -180)
        *yaw += 360;
}


//MPU6050_Init();
//MPU6050_Calibrate_GyroZ(100);
//float yaw = 0.0f;
//MPU6050_Update_Yaw(&yaw);
//counter++;
//if (counter % 10 == 0) {
//		printf("(int: %d)\r\n",  (int)yaw);
//}
//delay_ms(20);
