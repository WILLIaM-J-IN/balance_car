#include "bluetooth.h"
#include "usart.h"
#include "rtt_log.h"
#include <string.h>
#include <stdio.h>

static BT_ControlData ctrl;
static uint8_t rx_byte = 0;

void BT_Init(void) {
    ctrl.command      = BT_CMD_NONE;
    ctrl.speed_target = 0.0f;
    ctrl.turn_target  = 0.0f;
    ctrl.updated      = 0;

    HAL_StatusTypeDef ret = HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    if (ret != HAL_OK) {
        RTT_LOG_ERROR("[BT] UART2 中断接收启动失败 HAL_ERR=%d", (int)ret);
        RTT_LOG_ERROR("[BT] 请检查: 1.UART2配置 2.PA3接线 3.HC-06供电");
    } else {
        RTT_LOG_INFO("[BT] UART2 初始化成功 (RX=PA3, 波特率9600)");
        RTT_LOG_INFO("[BT] 等待手机蓝牙连接...");
    }
}

void BT_SendString(const char *str) {
    HAL_StatusTypeDef ret = HAL_UART_Transmit(
        &huart2, (uint8_t*)str, strlen(str), 100);
    if (ret != HAL_OK) {
        RTT_LOG_ERROR("[BT] 发送失败 HAL_ERR=%d", (int)ret);
    }
}

void BT_SendFloat(const char *label, float value) {
    char buf[64];
    snprintf(buf, sizeof(buf), "%s:%.2f\r\n", label, value);
    BT_SendString(buf);
}

void BT_ProcessRxByte(uint8_t byte) {
    ctrl.updated = 1;
    switch (byte) {
        case 'F':
            ctrl.command      = BT_CMD_FORWARD;
            ctrl.speed_target =  0.3f;
            ctrl.turn_target  =  0.0f;
            RTT_LOG_DEBUG("[BT] 指令: 前进 speed=%.1f", ctrl.speed_target);
            break;
        case 'B':
            ctrl.command      = BT_CMD_BACKWARD;
            ctrl.speed_target = -0.3f;
            ctrl.turn_target  =  0.0f;
            RTT_LOG_DEBUG("[BT] 指令: 后退 speed=%.1f", ctrl.speed_target);
            break;
        case 'L':
            ctrl.command      = BT_CMD_LEFT;
            ctrl.speed_target =  0.0f;
            ctrl.turn_target  = -0.5f;
            RTT_LOG_DEBUG("[BT] 指令: 左转");
            break;
        case 'R':
            ctrl.command      = BT_CMD_RIGHT;
            ctrl.speed_target =  0.0f;
            ctrl.turn_target  =  0.5f;
            RTT_LOG_DEBUG("[BT] 指令: 右转");
            break;
        case 'S':
            ctrl.command      = BT_CMD_STOP;
            ctrl.speed_target =  0.0f;
            ctrl.turn_target  =  0.0f;
            RTT_LOG_DEBUG("[BT] 指令: 停止");
            break;
        case '+':
            ctrl.speed_target += 0.1f;
            if (ctrl.speed_target > 1.0f) {
                ctrl.speed_target = 1.0f;
                RTT_LOG_WARN("[BT] 速度已到上限: 1.0 m/s");
            }
            RTT_LOG_DEBUG("[BT] 加速, speed=%.1f", ctrl.speed_target);
            break;
        case '-':
            ctrl.speed_target -= 0.1f;
            if (ctrl.speed_target < -1.0f) {
                ctrl.speed_target = -1.0f;
                RTT_LOG_WARN("[BT] 速度已到下限: -1.0 m/s");
            }
            RTT_LOG_DEBUG("[BT] 减速, speed=%.1f", ctrl.speed_target);
            break;
        default:
            ctrl.updated = 0;
            RTT_LOG_WARN("[BT] 收到未知指令: 0x%02X ('%c')", byte, byte);
            break;
    }
}

BT_ControlData* BT_GetControlData(void) { return &ctrl; }
uint8_t*        BT_GetRxBytePtr(void)   { return &rx_byte; }
