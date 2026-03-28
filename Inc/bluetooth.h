#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "stm32f4xx_hal.h"   /* HAL基础类型，必须在最前 */
#include <stdint.h>

/* UART2: TX=PA2  RX=PA3(blue_rx)  波特率9600 */

/* 蓝牙指令枚举 */
typedef enum {
    BT_CMD_NONE     = 0,
    BT_CMD_FORWARD  = 'F',
    BT_CMD_BACKWARD = 'B',
    BT_CMD_LEFT     = 'L',
    BT_CMD_RIGHT    = 'R',
    BT_CMD_STOP     = 'S',
    BT_CMD_SPEED_UP = '+',
    BT_CMD_SPEED_DN = '-',
} BT_Command;

/* 蓝牙控制数据结构 */
typedef struct {
    BT_Command command;
    float      speed_target;   /* 目标速度 m/s（前后） */
    float      turn_target;    /* 转向量（-1.0~1.0） */
    uint8_t    updated;        /* 是否有新指令 */
} BT_ControlData;

/* 函数声明 */
void            BT_Init(void);
void            BT_SendString(const char *str);
void            BT_SendFloat(const char *label, float value);
BT_ControlData* BT_GetControlData(void);
void            BT_ProcessRxByte(uint8_t byte);
uint8_t*        BT_GetRxBytePtr(void);

#endif /* __BLUETOOTH_H */
