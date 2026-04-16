#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "main.h"
#include <stdint.h>

/*============================================================
 *  蓝牙协议 v2
 *
 *  帧格式: $内容#
 *
 *  内容格式:
 *    单字符指令          : F            -> $F#
 *    带参数指令(整数)    : V,30,-50    -> $V,30,-50#
 *
 *  参数约定:
 *    speed:  cm/s   (例如 30 表示 0.30 m/s)
 *    turn :  ×100  (例如 -50 表示 -0.50)
 *
 *  支持指令:
 *    $F#         前进默认速度
 *    $F,50#      前进 0.50 m/s
 *    $B#         后退默认速度
 *    $B,30#      后退 0.30 m/s
 *    $L#  $L,40# 左转
 *    $R#  $R,40# 右转
 *    $S#         停止
 *    $V,sp,tn#   同时设置速度和转向
 *    $P#         心跳 (只 ACK 不动作)
 *============================================================*/

/*------------------------------------------------------------
 *  指令枚举
 *------------------------------------------------------------*/
typedef enum {
    BT_CMD_NONE      = 0,
    BT_CMD_FORWARD   = 'F',
    BT_CMD_BACKWARD  = 'B',
    BT_CMD_LEFT      = 'L',
    BT_CMD_RIGHT     = 'R',
    BT_CMD_STOP      = 'S',
    BT_CMD_VELOCITY  = 'V',
    BT_CMD_PING      = 'P'
} BT_Command;

/*------------------------------------------------------------
 *  控制数据结构
 *------------------------------------------------------------*/
typedef struct {
    BT_Command  command;        /* 最近收到的指令 */
    float       speed_target;   /* 目标速度 (m/s 或角度偏置) */
    float       turn_target;    /* 转向量 (无量纲,平衡环再缩放) */
    uint8_t     updated;        /* 1=有新指令,主循环消费后清 0 */
} BT_ControlData;

/*------------------------------------------------------------
 *  ACK 缓冲 (供主循环 BT_FlushAck 使用)
 *------------------------------------------------------------*/
extern volatile uint8_t bt_ack_pending;
extern char             bt_ack_buf[];

/*------------------------------------------------------------
 *  API
 *------------------------------------------------------------*/
void BT_Init(void);

/* UART RX 中断里调用 (单字节) */
void BT_ParseByte(uint8_t byte);

/* 处理完整帧内容 (不含 $ 和 #) - 内部用,通常不直接调 */
void BT_ProcessFrame(const char *frame);

/* 主循环里调用,把待发送的 ACK 发出去 (避免在中断里阻塞) */
void BT_FlushAck(void);

/* 发送字符串 / 浮点数 */
void BT_SendString(const char *str);
void BT_SendFloat(const char *label, float value);

/* UART 错误时复位解析器 */
void BT_ResetParser(void);

/* 数据访问 */
BT_ControlData* BT_GetControlData(void);
uint8_t*        BT_GetRxBytePtr(void);

#endif /* __BLUETOOTH_H */
