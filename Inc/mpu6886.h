#ifndef __MPU6886_H
#define __MPU6886_H

#include "stm32f4xx_hal.h"
#include "SEGGER_RTT.h"
#include <stdint.h>

/* I2C1: SCL=PB8  SDA=PB9  AD0=GND → addr=0x68 */
#define MPU6886_ADDR          (0x68 << 1)

/* Registers */
#define MPU6886_PWR_MGMT_1    0x6B
#define MPU6886_SMPLRT_DIV    0x19
#define MPU6886_CONFIG        0x1A
#define MPU6886_GYRO_CONFIG   0x1B
#define MPU6886_ACCEL_CONFIG  0x1C
#define MPU6886_ACCEL_CONFIG2 0x1D
#define MPU6886_ACCEL_XOUT_H  0x3B
#define MPU6886_GYRO_XOUT_H   0x43
#define MPU6886_WHO_AM_I      0x75
#define MPU6886_ACCEL_INTEL   0x69

/* IMU data */
typedef struct {
    float accX,  accY,  accZ;   /* Accel (g) */
    float gyroX, gyroY, gyroZ;  /* Gyro  (dps) */
    float temp;                  /* Temperature (degC) */
    float pitch;                 /* Balance angle (deg), 0=upright */
    float roll;                  /* Roll angle (deg) */
} MPU6886_Data;

uint8_t        MPU6886_Init(void);
void           MPU6886_Update(void);
MPU6886_Data*  MPU6886_GetData(void);
float          MPU6886_GetPitch(void);       /* balance angle */
float          MPU6886_GetPitchRate(void);   /* gyroZ: pitch rate for D-term */

#endif /* __MPU6886_H */
