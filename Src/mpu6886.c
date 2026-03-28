#include "mpu6886.h"
#include "i2c.h"
#include "rtt_log.h"
#include <math.h>

/* I2C1: SCL=PB8(GYRO_SCL)  SDA=PB9(GYRO_SDA) */

static MPU6886_Data imu;   /* 全局IMU数据 */

/*============================================================
 *  内部 I2C 读写函数
 *============================================================*/
static HAL_StatusTypeDef WriteReg(uint8_t reg, uint8_t data)
{
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(
        &hi2c1,
        MPU6886_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &data, 1, 100);

    if (ret != HAL_OK) {
        RTT_LOG_ERROR("[MPU6886] I2C写失败 REG=0x%02X DATA=0x%02X HAL_ERR=%d",
                      reg, data, (int)ret);
    }
    return ret;
}

static HAL_StatusTypeDef ReadReg(uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(
        &hi2c1,
        MPU6886_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        buf, len, 100);

    if (ret != HAL_OK) {
        RTT_LOG_ERROR("[MPU6886] I2C读失败 REG=0x%02X LEN=%d HAL_ERR=%d",
                      reg, len, (int)ret);
    }
    return ret;
}

/*============================================================
 *  初始化
 *============================================================*/
uint8_t MPU6886_Init(void)
{
    uint8_t who = 0;

    RTT_LOG_INFO("[MPU6886] 开始初始化 (I2C1: PB8=SCL, PB9=SDA)");
    RTT_LOG_INFO("[MPU6886] I2C地址: 0x%02X (AD0=GND)", MPU6886_ADDR >> 1);

    /*--- 1. 验证 WHO_AM_I ---*/
    /* 数据手册 Register 117 (0x75): 默认值 0x19 */
    if (ReadReg(MPU6886_WHO_AM_I, &who, 1) != HAL_OK) {
        RTT_LOG_ERROR("[MPU6886] 无法读取WHO_AM_I，请检查:");
        RTT_LOG_ERROR("[MPU6886]   1. I2C接线 PB8(SCL) PB9(SDA)");
        RTT_LOG_ERROR("[MPU6886]   2. MPU6886 供电 3.3V");
        RTT_LOG_ERROR("[MPU6886]   3. AD0 引脚电平 (GND=0x68, VCC=0x69)");
        return 0;
    }
    if (who != 0x19) {
        RTT_LOG_ERROR("[MPU6886] WHO_AM_I 验证失败: 读到0x%02X，期望0x19", who);
        RTT_LOG_ERROR("[MPU6886] 请确认芯片型号是否为 MPU-6886");
        return 0;
    }
    RTT_LOG_INFO("[MPU6886] WHO_AM_I=0x%02X 验证通过", who);

    /*--- 2. 复位设备 ---*/
    /* 数据手册 Register 107 (0x6B) bit[7]: DEVICE_RESET=1 */
    if (WriteReg(MPU6886_PWR_MGMT_1, 0x80) != HAL_OK) {
        RTT_LOG_ERROR("[MPU6886] 设备复位失败");
        return 0;
    }
    HAL_Delay(100);   /* 等待复位完成 */

    /*--- 3. 唤醒 + 自动选择最优时钟 ---*/
    /* 数据手册 Register 107 (0x6B):
     * SLEEP=0, CLKSEL=001(auto-select best clock)
     * Use Notes 9.6: "CLKSEL[2:0] must be set to 001" */
    if (WriteReg(MPU6886_PWR_MGMT_1, 0x01) != HAL_OK) {
        RTT_LOG_ERROR("[MPU6886] 唤醒失败");
        return 0;
    }
    HAL_Delay(10);

    /*--- 4. 陀螺仪量程 ±2000 dps ---*/
    /* 数据手册 Register 27 (0x1B):
     * FS_SEL[4:3]=11 → ±2000 dps
     * 灵敏度: 16.4 LSB/dps (Table 1) */
    if (WriteReg(MPU6886_GYRO_CONFIG, 0x18) != HAL_OK) {
        RTT_LOG_ERROR("[MPU6886] 陀螺仪量程配置失败");
        return 0;
    }
    RTT_LOG_INFO("[MPU6886] 陀螺仪量程: ±2000 dps (16.4 LSB/dps)");

    /*--- 5. 加速度计量程 ±8g ---*/
    /* 数据手册 Register 28 (0x1C):
     * ACCEL_FS_SEL[4:3]=10 → ±8g
     * 灵敏度: 4096 LSB/g (Table 2) */
    if (WriteReg(MPU6886_ACCEL_CONFIG, 0x10) != HAL_OK) {
        RTT_LOG_ERROR("[MPU6886] 加速度计量程配置失败");
        return 0;
    }
    RTT_LOG_INFO("[MPU6886] 加速度计量程: ±8g (4096 LSB/g)");

    /*--- 6. 陀螺仪低通滤波 92Hz ---*/
    /* 数据手册 Register 26 (0x1A):
     * bit[7]=0 (手册要求用户主动清零，默认是1)
     * DLPF_CFG=2 → 带宽92Hz，采样率1kHz (Table 16) */
    if (WriteReg(MPU6886_CONFIG, 0x02) != HAL_OK) {
        RTT_LOG_ERROR("[MPU6886] 陀螺仪低通滤波配置失败");
        return 0;
    }
    RTT_LOG_INFO("[MPU6886] 陀螺仪低通滤波: 92Hz");

    /*--- 7. 加速度计低通滤波 99Hz ---*/
    /* 数据手册 Register 29 (0x1D):
     * ACCEL_FCHOICE_B=0, A_DLPF_CFG=2 → 带宽99Hz (Table 17)
     * 与陀螺仪92Hz接近，保持一致性 */
    if (WriteReg(MPU6886_ACCEL_CONFIG2, 0x02) != HAL_OK) {
        RTT_LOG_ERROR("[MPU6886] 加速度计低通滤波配置失败");
        return 0;
    }
    RTT_LOG_INFO("[MPU6886] 加速度计低通滤波: 99Hz");

    /*--- 8. 采样率 200Hz ---*/
    /* 数据手册 Register 25 (0x19):
     * SAMPLE_RATE = 1kHz / (1 + SMPLRT_DIV)
     *             = 1000 / (1 + 4) = 200Hz */
    if (WriteReg(MPU6886_SMPLRT_DIV, 0x04) != HAL_OK) {
        RTT_LOG_ERROR("[MPU6886] 采样率配置失败");
        return 0;
    }
    RTT_LOG_INFO("[MPU6886] 采样率: 200Hz");

    /*--- 9. 防止加速度输出被限制 ---*/
    /* 数据手册 Register 105 (0x69) bit[1]: OUTPUT_LIMIT=1
     * "To avoid limiting sensor output to less than 0x7F7F,
     *  set this bit to 1. This should be done every time
     *  the MPU-6886 is powered up." */
    if (WriteReg(MPU6886_ACCEL_INTEL, 0x02) != HAL_OK) {
        RTT_LOG_WARN("[MPU6886] OUTPUT_LIMIT 设置失败（非致命）");
    }

    /* 初始化角度为0（等待互补滤波收敛） */
    imu.pitch = 0.0f;
    imu.roll  = 0.0f;

    RTT_LOG_INFO("[MPU6886] 初始化成功");
    return 1;
}

/*============================================================
 *  数据更新（每 5ms 调用一次）
 *============================================================*/
void MPU6886_Update(void)
{
    uint8_t buf[14];

    /*--- 连续读取 14 字节传感器数据 ---*/
    /* 数据手册 Register 59~72:
     * buf[0~5]  : AccX_H AccX_L AccY_H AccY_L AccZ_H AccZ_L
     * buf[6~7]  : Temp_H Temp_L
     * buf[8~13] : GyroX_H GyroX_L GyroY_H GyroY_L GyroZ_H GyroZ_L */
    if (ReadReg(MPU6886_ACCEL_XOUT_H, buf, 14) != HAL_OK) {
        RTT_LOG_ERROR("[MPU6886] 传感器数据读取失败，跳过本次更新");
        return;
    }

    /* 拼接 16 位有符号整数（二进制补码格式） */
    int16_t ax = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4]  << 8) | buf[5]);
    int16_t t  = (int16_t)((buf[6]  << 8) | buf[7]);
    int16_t gx = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

    /*--- 物理量转换 ---*/
    /* 加速度计: 数据手册 Table 2, AFS_SEL=2 → 4096 LSB/g */
    imu.accX  = ax / 4096.0f;
    imu.accY  = ay / 4096.0f;
    imu.accZ  = az / 4096.0f;

    /* 陀螺仪: 数据手册 Table 1, FS_SEL=3 → 16.4 LSB/dps */
    imu.gyroX = gx / 16.4f;
    imu.gyroY = gy / 16.4f;
    imu.gyroZ = gz / 16.4f;

    /* 温度: 数据手册 Register 66
     * TEMP_degC = TEMP_OUT / 326.8 + 25
     * Temp_Sensitivity = 326.8 LSB/°C
     * RoomTemp_Offset  = 25°C */
    imu.temp = t / 326.8f + 25.0f;

    /*--- 加速度计计算角度（用于互补滤波修正） ---*/
    /* 俯仰角 pitch：绕Y轴旋转（平衡车前后倾斜）*/
    float acc_pitch = atan2f(imu.accX,
                              sqrtf(imu.accY * imu.accY +
                                     imu.accZ * imu.accZ))
                      * 180.0f / (float)M_PI;

    /* 横滚角 roll：绕X轴旋转（平衡车左右倾斜）*/
    float acc_roll  = atan2f(imu.accY, imu.accZ)
                      * 180.0f / (float)M_PI;

    /*--- 互补滤波（陀螺仪积分 + 加速度计修正）---*/
    /* 原理：
     *   陀螺仪：短期准确，长期有漂移
     *   加速度计：长期准确，短期有噪声（运动时）
     *   互补滤波 = 98%陀螺仪积分 + 2%加速度计
     *
     * 数学公式：
     *   pitch = α × (pitch + gyroY × dt) + (1-α) × acc_pitch
     *   α = 0.98（陀螺仪权重）
     *   dt = 0.005s（5ms采样周期）
     *
     * 陀螺仪单位 dps（度/秒），乘以 dt 得到角度增量 */
    imu.pitch = 0.98f * (imu.pitch + imu.gyroY * 0.005f)
              + 0.02f * acc_pitch;

    imu.roll  = 0.98f * (imu.roll  + imu.gyroX * 0.005f)
              + 0.02f * acc_roll;

    /*--- 数据合理性检查 ---*/
    /* 温度检查（数据手册工作范围 -40°C ~ +85°C）*/
    if (imu.temp < -40.0f || imu.temp > 85.0f) {
        RTT_LOG_WARN("[MPU6886] 温度异常: %.1f°C (范围-40~85)", imu.temp);
    }

    /* 加速度计全零检查（传感器未响应）*/
    if (ax == 0 && ay == 0 && az == 0) {
        RTT_LOG_ERROR("[MPU6886] 加速度计全零！传感器可能未正常工作");
    }

    /* 陀螺仪全零检查 */
    if (gx == 0 && gy == 0 && gz == 0) {
        RTT_LOG_WARN("[MPU6886] 陀螺仪全零，可能处于静止或异常");
    }
}

/*============================================================
 *  数据获取接口
 *============================================================*/

MPU6886_Data* MPU6886_GetData(void)
{
    return &imu;
}

float MPU6886_GetPitch(void)
{
    return imu.pitch;
}

float MPU6886_GetGyroY(void)
{
    return imu.gyroY;
}
