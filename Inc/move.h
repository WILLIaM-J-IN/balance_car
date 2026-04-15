#ifndef __MOVE_H__
#define __MOVE_H__

#include "main.h"
#include <stdint.h>

/* ============================================================
 *  运动控制模块
 *  职责: 在平衡环之上生成速度/转向目标
 *
 *  注意: 当前速度环关闭,speed_target 的含义是"前倾角度偏置"(度)
 *       正值 = 前倾 = 前进, 负值 = 后仰 = 后退
 *       数值越大跑越快,但无速度闭环,会溜坡、受阻力影响
 * ============================================================ */

/* ---- 目标限幅 ---- */
#define MOVE_SPEED_MAX        3.0f      /* 最大前倾偏置 (度),先保守 */
#define MOVE_TURN_MAX         1500.0f   /* 最大转向量 (PWM 单位,配合 TURN_KP) */

/* ---- 加速度限幅 (防止急起急停翻车) ---- */
#define MOVE_SPEED_ACCEL      6.0f      /* 度/秒,每秒最多增加 6 度偏置 */
#define MOVE_TURN_ACCEL       3000.0f   /* 每秒转向变化上限 */

/* ---- 更新周期 ---- */
#define MOVE_UPDATE_MS        10

/* ---- 运动模式 ---- */
typedef enum {
    MOVE_MODE_STOP = 0,      /* 停车,不产生任何指令 */
    MOVE_MODE_MANUAL,        /* 手动:蓝牙摇杆驱动 */
    MOVE_MODE_AUTO           /* 自动:API 驱动 */
} Move_Mode_t;

/* ---- 初始化 ---- */
void Move_Init(void);

/* ---- 周期性调用 (建议 10ms) ---- */
void Move_Update(void);

/* ---- 模式切换 ---- */
void Move_SetMode(Move_Mode_t mode);
Move_Mode_t Move_GetMode(void);

/* ---- 自动模式 API (调用后自动切到 AUTO 模式) ---- */
void Move_Stop(void);                             /* 停止 + 切 STOP 模式 */
void Move_Forward(float tilt_deg);                /* 前倾多少度,负值=后退 */
void Move_Turn(float turn_pwm);                   /* 转向 正=右 负=左 */
void Move_SetTarget(float tilt_deg, float turn);  /* 同时设置 */
void Move_RotateInPlace(float turn_pwm);          /* 原地旋转 */
void Move_StraightForward(float tilt_deg);        /* 直行(强制不转向) */

/* ---- 状态查询 ---- */
float Move_GetSpeedTarget(void);
float Move_GetTurnTarget(void);

#endif /* __MOVE_H__ */
