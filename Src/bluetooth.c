#include "bluetooth.h"
#include "usart.h"
#include "rtt_log.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/*============================================================
 *  常量与状态
 *============================================================*/
#define FRAME_START   '$'
#define FRAME_END     '#'
#define BT_BUF_SIZE   32

typedef enum {
    BT_WAIT_START = 0,    /* 等待 '$' */
    BT_RECV_BODY  = 1,    /* 接收帧内容,直到 '#' */
} BT_ParseState;

/* 解析器内部状态 */
static BT_ControlData ctrl;
static uint8_t        rx_byte    = 0;
static BT_ParseState  bt_state   = BT_WAIT_START;
static char           bt_buf[BT_BUF_SIZE];
static uint8_t        bt_buf_pos = 0;

/* ACK 异步发送缓冲 (中断里只填,主循环 BT_FlushAck 真正发) */
volatile uint8_t bt_ack_pending = 0;
char             bt_ack_buf[BT_BUF_SIZE] = {0};

/*============================================================
 *  初始化
 *============================================================*/
void BT_Init(void)
{
    ctrl.command      = BT_CMD_NONE;
    ctrl.speed_target = 0.0f;
    ctrl.turn_target  = 0.0f;
    ctrl.updated      = 0;

    bt_state   = BT_WAIT_START;
    bt_buf_pos = 0;
    memset(bt_buf, 0, BT_BUF_SIZE);

    bt_ack_pending = 0;
    memset(bt_ack_buf, 0, BT_BUF_SIZE);

    HAL_StatusTypeDef ret = HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    if (ret != HAL_OK) {
        RTT_LOG_ERROR("[BT] UART2 receive IT start FAILED HAL_ERR=%d", (int)ret);
        RTT_LOG_ERROR("[BT] Check: 1.UART2 config  2.PA3 wiring  3.HC-05 power");
    } else {
        RTT_LOG_INFO("[BT] UART2 init OK (RX=PA3, Baud=9600)");
        RTT_LOG_INFO("[BT] Protocol v2: $X# or $X,arg1,arg2#");
        RTT_LOG_INFO("[BT] Waiting for connection...");
    }
}

/*============================================================
 *  帧解析 (UART RX 中断里调用)
 *  - 收到 '$' 进入接收模式
 *  - 之后每个字节存入缓冲,直到 '#'
 *  - 收到 '#' 后调用 BT_ProcessFrame 处理
 *============================================================*/
void BT_ParseByte(uint8_t byte)
{
    switch (bt_state)
    {
        case BT_WAIT_START:
            if (byte == FRAME_START) {
                bt_state   = BT_RECV_BODY;
                bt_buf_pos = 0;
                memset(bt_buf, 0, BT_BUF_SIZE);
            }
            /* 其他字节静默丢弃 (HC-05 状态码、噪声等) */
            break;

        case BT_RECV_BODY:
            if (byte == FRAME_END) {
                /* 帧结束 → 处理 */
                bt_buf[bt_buf_pos] = '\0';
                BT_ProcessFrame(bt_buf);
                bt_state   = BT_WAIT_START;
                bt_buf_pos = 0;
            }
            else if (byte == FRAME_START) {
                /* 中途又遇到 $,认为前面错乱了,重新开始 */
                bt_buf_pos = 0;
                memset(bt_buf, 0, BT_BUF_SIZE);
                RTT_LOG_WARN("[BT] Restart frame mid-stream");
            }
            else if (bt_buf_pos < BT_BUF_SIZE - 1) {
                bt_buf[bt_buf_pos++] = (char)byte;
            }
            else {
                /* 超长,丢弃 */
                RTT_LOG_WARN("[BT] Frame too long, reset");
                bt_state   = BT_WAIT_START;
                bt_buf_pos = 0;
            }
            break;

        default:
            bt_state = BT_WAIT_START;
            break;
    }
}

/*============================================================
 *  帧处理 (输入: 不含 $ 和 # 的帧内容)
 *  例如 frame = "F"        → 单字符指令
 *      frame = "V,30,-50"  → 命令 V 带两个参数
 *============================================================*/
void BT_ProcessFrame(const char *frame)
{
    if (frame == NULL || frame[0] == '\0') {
        return;
    }

    char cmd = frame[0];

    /* 解析参数 (如果有) */
    int arg1 = 0, arg2 = 0;
    int n_args = 0;
    if (frame[1] == ',') {
        n_args = sscanf(frame + 1, ",%d,%d", &arg1, &arg2);
        /* n_args = 2: 两个参数, 1: 一个, 0/-1: 解析失败 */
    }

    /* 排队 ACK 给上位机 (主循环 BT_FlushAck 会发出去) */
    snprintf(bt_ack_buf, BT_BUF_SIZE, "ACK:%c,%d,%d\r\n", cmd, arg1, arg2);
    bt_ack_pending = 1;

    /* 默认标记有更新,处理失败的命令再清回 0 */
    ctrl.updated = 1;

    switch (cmd)
    {
        case 'F':       /* Forward */
            ctrl.command      = BT_CMD_FORWARD;
            ctrl.speed_target = (n_args >= 1) ? (arg1 / 100.0f) : 0.3f;
            ctrl.turn_target  = 0.0f;
            break;

        case 'B':       /* Backward */
            ctrl.command      = BT_CMD_BACKWARD;
            ctrl.speed_target = (n_args >= 1) ? -(arg1 / 100.0f) : -0.3f;
            ctrl.turn_target  = 0.0f;
            break;

        case 'L':       /* Left */
            ctrl.command      = BT_CMD_LEFT;
            ctrl.speed_target = 0.0f;
            ctrl.turn_target  = (n_args >= 1) ? -(arg1 / 100.0f) : -0.5f;
            break;

        case 'R':       /* Right */
            ctrl.command      = BT_CMD_RIGHT;
            ctrl.speed_target = 0.0f;
            ctrl.turn_target  = (n_args >= 1) ?  (arg1 / 100.0f) :  0.5f;
            break;

        case 'S':       /* Stop */
            ctrl.command      = BT_CMD_STOP;
            ctrl.speed_target = 0.0f;
            ctrl.turn_target  = 0.0f;
            break;

        case 'V':       /* Velocity: $V,sp,tn# 同时设速度+转向 */
            ctrl.command      = BT_CMD_VELOCITY;
            ctrl.speed_target = (n_args >= 1) ? (arg1 / 100.0f) : 0.0f;
            ctrl.turn_target  = (n_args >= 2) ? (arg2 / 100.0f) : 0.0f;
            break;

        case 'P':       /* Ping: 心跳,只 ACK 不更新控制 */
            ctrl.updated = 0;
            break;

        default:
            ctrl.updated = 0;
            RTT_LOG_WARN("[BT] Unknown cmd '%c' (frame=%s)", cmd, frame);
            return;
    }

    /* 简短日志,详细解析交给主循环的 BT_DebugMonitor */
    RTT_LOG_DEBUG("[BT] Frame: %s", frame);
}

/*============================================================
 *  ACK 异步发送 (主循环里调用)
 *  避免在 UART 中断里使用阻塞的 HAL_UART_Transmit
 *============================================================*/
void BT_FlushAck(void)
{
    if (bt_ack_pending) {
        bt_ack_pending = 0;
        HAL_UART_Transmit(&huart2,
                          (uint8_t*)bt_ack_buf,
                          strlen(bt_ack_buf),
                          50);
    }
}

/*============================================================
 *  发送 (主循环里用,不要在中断里调)
 *============================================================*/
void BT_SendString(const char *str)
{
    HAL_StatusTypeDef ret = HAL_UART_Transmit(
        &huart2, (uint8_t*)str, strlen(str), 100);
    if (ret != HAL_OK) {
        RTT_LOG_ERROR("[BT] Send FAILED HAL_ERR=%d", (int)ret);
    }
}

void BT_SendFloat(const char *label, float value)
{
    char buf[64];
    int v_i = (int)value;
    int v_f = (int)((value - v_i) * 100);
    if (v_f < 0) v_f = -v_f;
    snprintf(buf, sizeof(buf), "%s:%d.%02d\r\n", label, v_i, v_f);
    BT_SendString(buf);
}

/*============================================================
 *  解析器复位 (UART 错误回调里调用)
 *============================================================*/
void BT_ResetParser(void)
{
    bt_state   = BT_WAIT_START;
    bt_buf_pos = 0;
    memset(bt_buf, 0, BT_BUF_SIZE);
    /* 不打印,避免错误回调里大量日志 */
}

/*============================================================
 *  数据访问
 *============================================================*/
BT_ControlData* BT_GetControlData(void) { return &ctrl; }
uint8_t*        BT_GetRxBytePtr(void)   { return &rx_byte; }
