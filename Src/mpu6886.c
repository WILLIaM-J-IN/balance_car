#include "mpu6886.h"
#include "i2c.h"
#include "rtt_log.h"
#include <math.h>

/*============================================================
 * MPU6886 - Horizontal Mount Correction (水平安装)
 *
 * Physical layout (your car):
 * accZ = -1.0g when flat → Z axis points DOWN (gravity = -Z)
 * accX = 1.0g  when tip forward → X axis is forward/backward
 * accY = 1.0g  when tip right   → Y axis is left/right
 *============================================================*/

static MPU6886_Data imu;

/* Gyro bias (removed during calibration) */
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;

#define CALIB_SAMPLES  200

/*============================================================
 * Internal I2C helpers
 *============================================================*/
static HAL_StatusTypeDef WriteReg(uint8_t reg, uint8_t data)
{
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(
        &hi2c1, MPU6886_ADDR, reg,
        I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    if (ret != HAL_OK)
        RTT_LOG_ERROR("[MPU6886] Write FAILED REG=0x%02X HAL_ERR=%d", reg, (int)ret);
    return ret;
}

static HAL_StatusTypeDef ReadReg(uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(
        &hi2c1, MPU6886_ADDR, reg,
        I2C_MEMADD_SIZE_8BIT, buf, len, 100);
    if (ret != HAL_OK)
        RTT_LOG_ERROR("[MPU6886] Read FAILED REG=0x%02X LEN=%d HAL_ERR=%d",
                      reg, len, (int)ret);
    return ret;
}

/*============================================================
 * Gyro bias calibration (~1 second, keep car still)
 *============================================================*/
static void MPU6886_CalibrateGyro(void)
{
    float sum_x = 0, sum_y = 0, sum_z = 0;
    uint8_t buf[6];

    RTT_LOG_INFO("[MPU6886] Calibrating gyro bias, keep still for 1s...");

    for (int i = 0; i < CALIB_SAMPLES; i++)
    {
        if (ReadReg(MPU6886_GYRO_XOUT_H, buf, 6) == HAL_OK)
        {
            sum_x += (int16_t)((buf[0] << 8) | buf[1]) / 16.4f;
            sum_y += (int16_t)((buf[2] << 8) | buf[3]) / 16.4f;
            sum_z += (int16_t)((buf[4] << 8) | buf[5]) / 16.4f;
        }
        HAL_Delay(5);
    }

    gyro_bias_x = sum_x / CALIB_SAMPLES;
    gyro_bias_y = sum_y / CALIB_SAMPLES;
    gyro_bias_z = sum_z / CALIB_SAMPLES;

    /* Print bias using int conversion */
    int bx = (int)(gyro_bias_x * 100);
    int by = (int)(gyro_bias_y * 100);
    int bz = (int)(gyro_bias_z * 100);
    SEGGER_RTT_printf(0,
        "[MPU6886] Gyro bias: X=%d.%02d Y=%d.%02d Z=%d.%02d dps\r\n",
        bx/100, bx%100 < 0 ? -bx%100 : bx%100,
        by/100, by%100 < 0 ? -by%100 : by%100,
        bz/100, bz%100 < 0 ? -bz%100 : bz%100);
}

/*============================================================
 * Init
 *============================================================*/
uint8_t MPU6886_Init(void)
{
    uint8_t who = 0;

    RTT_LOG_INFO("[MPU6886] Init (I2C1 PB8=SCL PB9=SDA addr=0x68)");

    if (ReadReg(MPU6886_WHO_AM_I, &who, 1) != HAL_OK) {
        RTT_LOG_ERROR("[MPU6886] WHO_AM_I read failed");
        return 0;
    }
    if (who != 0x19) {
        RTT_LOG_ERROR("[MPU6886] WHO_AM_I=0x%02X expected 0x19", who);
        return 0;
    }

    WriteReg(MPU6886_PWR_MGMT_1, 0x80);
    HAL_Delay(100);
    WriteReg(MPU6886_PWR_MGMT_1, 0x01);
    HAL_Delay(10);

    WriteReg(MPU6886_GYRO_CONFIG, 0x18);
    WriteReg(MPU6886_ACCEL_CONFIG, 0x10);
    WriteReg(MPU6886_CONFIG, 0x02);
    WriteReg(MPU6886_ACCEL_CONFIG2, 0x02);
    WriteReg(MPU6886_SMPLRT_DIV, 0x04);
    WriteReg(MPU6886_ACCEL_INTEL, 0x02);

    imu.pitch = 0.0f;
    imu.roll  = 0.0f;

    MPU6886_CalibrateGyro();

    RTT_LOG_INFO("[MPU6886] Init SUCCESS");
    return 1;
}

/*============================================================
 * Update (call every 5ms from balance loop)
 *============================================================*/
void MPU6886_Update(void)
{
    uint8_t buf[14];

    if (ReadReg(MPU6886_ACCEL_XOUT_H, buf, 14) != HAL_OK) return;

    int16_t ax = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4]  << 8) | buf[5]);
    int16_t t  = (int16_t)((buf[6]  << 8) | buf[7]);
    int16_t gx = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

    /* Physical units */
    imu.accX  = ax / 4096.0f;
    imu.accY  = ay / 4096.0f;
    imu.accZ  = az / 4096.0f;

    /* Subtract calibrated bias */
    imu.gyroX = gx / 16.4f - gyro_bias_x;
    imu.gyroY = gy / 16.4f - gyro_bias_y;
    imu.gyroZ = gz / 16.4f - gyro_bias_z;
    imu.temp  = t / 326.8f + 25.0f;

    /*--- Accel-based angles (重力在 -Z 轴，前进在 X 轴) ---*/
    float acc_pitch = atan2f(imu.accX, -imu.accZ) * 180.0f / (float)M_PI;
    float acc_roll  = atan2f(imu.accY, -imu.accZ) * 180.0f / (float)M_PI;

    /*--- Complementary filter ---*/
    // X轴前后倾倒，对应的是绕Y轴旋转(gyroY)
    imu.pitch = 0.98f * (imu.pitch + imu.gyroY * 0.005f) + 0.02f * acc_pitch;
    imu.roll  = 0.98f * (imu.roll  + imu.gyroX * 0.005f) + 0.02f * acc_roll;
}

/*============================================================
 * Data access
 *============================================================*/
MPU6886_Data* MPU6886_GetData(void)      { return &imu; }
float         MPU6886_GetPitch(void)     { return imu.pitch; }

/* Pitch RATE for PID D-term (返回 gyroY，因为这是真正的俯仰角速度) */
float         MPU6886_GetPitchRate(void) { return imu.gyroY; }
