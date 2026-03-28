#ifndef __MPU6886_H
#define __MPU6886_H

#include "main.h"

// I2C1: SCL=PB8(GYRO_SCL)  SDA=PB9(GYRO_SDA)
// AD0接GND时地址为0x68
#define MPU6886_ADDR           (0x68 << 1)

// 寄存器定义
#define MPU6886_PWR_MGMT_1     0x6B
#define MPU6886_SMPLRT_DIV     0x19
#define MPU6886_CONFIG         0x1A
#define MPU6886_GYRO_CONFIG    0x1B
#define MPU6886_ACCEL_CONFIG   0x1C
#define MPU6886_ACCEL_CONFIG2  0x1D
#define MPU6886_ACCEL_XOUT_H   0x3B
#define MPU6886_GYRO_XOUT_H    0x43
#define MPU6886_WHO_AM_I       0x75
#define MPU6886_ACCEL_INTEL    0x69

typedef struct {
    float accX,  accY,  accZ;   // 加速度 (g)
    float gyroX, gyroY, gyroZ;  // 角速度 (dps)
    float temp;                  // 温度 (°C)
    float pitch;                 // 俯仰角 (度)
    float roll;                  // 横滚角 (度)
} MPU6886_Data;

uint8_t       MPU6886_Init(void);
void          MPU6886_Update(void);
MPU6886_Data* MPU6886_GetData(void);
float         MPU6886_GetPitch(void);
float         MPU6886_GetGyroY(void);

#endif
