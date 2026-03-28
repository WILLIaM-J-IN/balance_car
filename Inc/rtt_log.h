#ifndef __RTT_LOG_H
#define __RTT_LOG_H

#include "SEGGER_RTT.h"

// 日志级别宏（带颜色）
#define RTT_LOG_INFO(fmt, ...)  \
    SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN  "[INFO ] " fmt RTT_CTRL_RESET "\r\n", ##__VA_ARGS__)

#define RTT_LOG_WARN(fmt, ...)  \
    SEGGER_RTT_printf(0, RTT_CTRL_TEXT_YELLOW "[WARN ] " fmt RTT_CTRL_RESET "\r\n", ##__VA_ARGS__)

#define RTT_LOG_ERROR(fmt, ...) \
    SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED    "[ERROR] " fmt RTT_CTRL_RESET "\r\n", ##__VA_ARGS__)

#define RTT_LOG_DEBUG(fmt, ...) \
    SEGGER_RTT_printf(0, RTT_CTRL_TEXT_CYAN   "[DEBUG] " fmt RTT_CTRL_RESET "\r\n", ##__VA_ARGS__)

// HAL状态检查宏（失败时自动输出错误信息）
#define RTT_CHECK_HAL(expr, msg)                        \
    do {                                                 \
        HAL_StatusTypeDef _ret = (expr);                 \
        if (_ret != HAL_OK) {                            \
            RTT_LOG_ERROR(msg " HAL_ERR=%d", (int)_ret); \
        }                                                \
    } while(0)

#endif
