#include "move.h"
#include "bluetooth.h"
#include "rtt_log.h"
#include <math.h>

/*============================================================
 * 内部状态
 *============================================================*/
static Move_Mode_t  s_mode           = MOVE_MODE_MANUAL;

/* AUTO 模式下的用户期望值 */
static float        s_target_speed   = 0.0f;
static float        s_target_turn    = 0.0f;

/* 实际下发值 (经过斜率限制后) */
static float        s_current_speed  = 0.0f;
static float        s_current_turn   = 0.0f;

/*============================================================
 * 工具函数
 *============================================================*/
static float clampf(float v, float min, float max)
{
    if (v > max) return max;
    if (v < min) return min;
    return v;
}

/* 斜率限制 */
static float slew(float current, float target, float max_step)
{
    float diff = target - current;
    if (diff >  max_step) diff =  max_step;
    if (diff < -max_step) diff = -max_step;
    return current + diff;
}

/*============================================================
 * 初始化
 *============================================================*/
void Move_Init(void)
{
    s_mode          = MOVE_MODE_MANUAL;   /* 上电默认手动 */
    s_target_speed  = 0.0f;
    s_target_turn   = 0.0f;
    s_current_speed = 0.0f;
    s_current_turn  = 0.0f;

    RTT_LOG_INFO("[Move] Init done, mode=MANUAL");
}

/*============================================================
 * 周期性更新 (建议 10ms)
 *============================================================*/
void Move_Update(void)
{
    float want_speed = 0.0f;
    float want_turn  = 0.0f;

    BT_ControlData *bt = BT_GetControlData();

    /*--- 1) 根据模式选择目标来源 ---*/
    switch (s_mode)
    {
        case MOVE_MODE_STOP:
            want_speed = 0.0f;
            want_turn  = 0.0f;
            break;

        case MOVE_MODE_MANUAL:
            /* 直接用蓝牙的值,不覆盖 bt->speed_target */
            want_speed = bt->speed_target;
            want_turn  = bt->turn_target;
            break;

        case MOVE_MODE_AUTO:
            /* 自动模式用 API 设定的目标值 */
            want_speed = s_target_speed;
            want_turn  = s_target_turn;
            break;

        default:
            break;
    }

    /*--- 2) 限幅 ---*/
    want_speed = clampf(want_speed, -MOVE_SPEED_MAX, MOVE_SPEED_MAX);
    want_turn  = clampf(want_turn,  -MOVE_TURN_MAX,  MOVE_TURN_MAX);

    /*--- 3) 斜率限制(加速度) ---*/
    float dt         = MOVE_UPDATE_MS / 1000.0f;
    float speed_step = MOVE_SPEED_ACCEL * dt;
    float turn_step  = MOVE_TURN_ACCEL  * dt;

    s_current_speed = slew(s_current_speed, want_speed, speed_step);
    s_current_turn  = slew(s_current_turn,  want_turn,  turn_step);

    /*--- 4) 写回 BT_ControlData,供 balance.c 读取 ---*/
    /*      注意:手动模式下这里覆盖了蓝牙原值,但值是一样的,无副作用
     *             自动模式下会替换蓝牙值,实现"接管" */
    bt->speed_target = s_current_speed;
    bt->turn_target  = s_current_turn;

    /*--- 5) 调试日志 (每 500ms 一次) ---*/
    static uint32_t log_cnt = 0;
    if (++log_cnt >= (500 / MOVE_UPDATE_MS)) {
        log_cnt = 0;
        int i_spd = (int)(s_current_speed * 100);   /* 度 × 100 方便看小数 */
        int i_trn = (int)s_current_turn;
        RTT_LOG_DEBUG("[Move] mode=%d spd=%d(0.01deg) turn=%d",
                      (int)s_mode, i_spd, i_trn);
    }
}

/*============================================================
 * 模式切换
 *============================================================*/
void Move_SetMode(Move_Mode_t mode)
{
    if (mode != s_mode) {
        s_mode = mode;
        /* 切模式时清空自动目标,避免残留 */
        if (mode != MOVE_MODE_AUTO) {
            s_target_speed = 0.0f;
            s_target_turn  = 0.0f;
        }
        RTT_LOG_INFO("[Move] Mode -> %d", (int)mode);
    }
}

Move_Mode_t Move_GetMode(void) { return s_mode; }

/*============================================================
 * 自动模式 API
 * 调用任何一个都会自动切到 AUTO,接管蓝牙输入
 *============================================================*/
void Move_Stop(void)
{
    /* 如果之前在动,先给反向刹车脉冲,抵消惯性 */
    if (fabsf(s_current_speed) > 0.5f) {
        s_target_speed = -s_current_speed * 0.6f;   /* 反向 60% 刹车 */
        s_target_turn  = 0.0f;
        s_mode         = MOVE_MODE_AUTO;
        RTT_LOG_INFO("[Move] Brake pulse %d(0.01deg)", (int)(s_target_speed * 100));
        /* 等一小段时间让刹车生效 */
        HAL_Delay(200);
    }
    /* 然后才真正停 */
    s_target_speed = 0.0f;
    s_target_turn  = 0.0f;
    s_mode         = MOVE_MODE_STOP;
    RTT_LOG_INFO("[Move] STOP");
}

void Move_Forward(float tilt_deg)
{
    s_target_speed = clampf(tilt_deg, -MOVE_SPEED_MAX, MOVE_SPEED_MAX);
    s_mode         = MOVE_MODE_AUTO;
    RTT_LOG_INFO("[Move] Forward tilt=%d(0.01deg)", (int)(s_target_speed * 100));
}

void Move_Turn(float turn_pwm)
{
    s_target_turn = clampf(turn_pwm, -MOVE_TURN_MAX, MOVE_TURN_MAX);
    s_mode        = MOVE_MODE_AUTO;
    RTT_LOG_INFO("[Move] Turn=%d", (int)s_target_turn);
}

void Move_SetTarget(float tilt_deg, float turn)
{
    s_target_speed = clampf(tilt_deg, -MOVE_SPEED_MAX, MOVE_SPEED_MAX);
    s_target_turn  = clampf(turn,     -MOVE_TURN_MAX,  MOVE_TURN_MAX);
    s_mode         = MOVE_MODE_AUTO;
}

void Move_RotateInPlace(float turn_pwm)
{
    s_target_speed = 0.0f;
    s_target_turn  = clampf(turn_pwm, -MOVE_TURN_MAX, MOVE_TURN_MAX);
    s_mode         = MOVE_MODE_AUTO;
    RTT_LOG_INFO("[Move] RotateInPlace turn=%d", (int)s_target_turn);
}

void Move_StraightForward(float tilt_deg)
{
    s_target_speed = clampf(tilt_deg, -MOVE_SPEED_MAX, MOVE_SPEED_MAX);
    s_target_turn  = 0.0f;
    s_mode         = MOVE_MODE_AUTO;
    RTT_LOG_INFO("[Move] Straight tilt=%d(0.01deg)", (int)(s_target_speed * 100));
}

/*============================================================
 * 状态查询
 *============================================================*/
float Move_GetSpeedTarget(void) { return s_current_speed; }
float Move_GetTurnTarget(void)  { return s_current_turn;  }
