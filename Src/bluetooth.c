#include "bluetooth.h"
#include "usart.h"
#include "rtt_log.h"
#include <string.h>
#include <stdio.h>

/*============================================================
 *  Protocol definition
 *  Frame format: $CMD#
 *  Example:      $F# = Forward
 *                $S# = Stop
 *============================================================*/

#define FRAME_START  '$'
#define FRAME_END    '#'

/* Frame parser state machine */
typedef enum {
    BT_WAIT_START = 0,   /* waiting for '$' */
    BT_RECV_CMD   = 1,   /* receiving command byte */
    BT_WAIT_END   = 2,   /* waiting for '#' */
} BT_ParseState;

/* Internal state */
static BT_ControlData  ctrl;
static uint8_t         rx_byte   = 0;
static BT_ParseState   bt_state  = BT_WAIT_START;
static char            bt_cmd    = 0;

/*============================================================
 *  Init
 *============================================================*/
void BT_Init(void)
{
    ctrl.command      = BT_CMD_NONE;
    ctrl.speed_target = 0.0f;
    ctrl.turn_target  = 0.0f;
    ctrl.updated      = 0;

    bt_state = BT_WAIT_START;
    bt_cmd   = 0;

    HAL_StatusTypeDef ret = HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    if (ret != HAL_OK) {
        RTT_LOG_ERROR("[BT] UART2 receive IT start FAILED HAL_ERR=%d", (int)ret);
        RTT_LOG_ERROR("[BT] Check: 1.UART2 config  2.PA3 wiring  3.HC-05 power");
    } else {
        RTT_LOG_INFO("[BT] UART2 init OK (RX=PA3, Baud=9600)");
        RTT_LOG_INFO("[BT] Frame protocol: $CMD# (e.g. $F# = Forward)");
        RTT_LOG_INFO("[BT] Waiting for connection...");
    }
}

/*============================================================
 *  Frame parser (call from UART RX interrupt)
 *============================================================*/
void BT_ParseByte(uint8_t byte)
{
    switch (bt_state)
    {
        /*--- Wait for start byte '$' ---*/
        case BT_WAIT_START:
            if (byte == FRAME_START) {
                bt_state = BT_RECV_CMD;
            }
            /* Silently ignore all other bytes (noise/HC-05 status) */
            break;

        /*--- Receive command byte ---*/
        case BT_RECV_CMD:
            if (byte >= 0x20 && byte <= 0x7E) {
                bt_cmd   = (char)byte;
                bt_state = BT_WAIT_END;
                RTT_LOG_DEBUG("[BT] CMD byte: '%c' (0x%02X)", bt_cmd, byte);
            } else {
                RTT_LOG_WARN("[BT] Invalid CMD byte 0x%02X, reset", byte);
                bt_state = BT_WAIT_START;
                bt_cmd   = 0;
            }
            break;

        /*--- Wait for end byte '#' ---*/
        case BT_WAIT_END:
            if (byte == FRAME_END) {
                /* Complete frame, process command */
                SEGGER_RTT_printf(0,
                    "\x1B[1;31m[BT] Frame OK: cmd='%c'\x1B[0m\r\n", bt_cmd);

                /* Send ACK back to PC */
                char ack[16];
                snprintf(ack, sizeof(ack), "ACK:%c\r\n", bt_cmd);
                HAL_UART_Transmit(&huart2, (uint8_t*)ack, strlen(ack), 100);

                /* Process command */
                BT_ProcessRxByte((uint8_t)bt_cmd);

            } else {
                RTT_LOG_WARN("[BT] Frame ERROR: expected '#' got 0x%02X, reset",
                             byte);
            }
            /* Reset state machine */
            bt_state = BT_WAIT_START;
            bt_cmd   = 0;
            break;

        default:
            bt_state = BT_WAIT_START;
            bt_cmd   = 0;
            break;
    }
}

/*============================================================
 *  Command processor
 *============================================================*/
void BT_ProcessRxByte(uint8_t byte)
{
    ctrl.updated = 1;

    switch (byte)
    {
        case 'F':
            ctrl.command      = BT_CMD_FORWARD;
            ctrl.speed_target =  0.3f;
            ctrl.turn_target  =  0.0f;
            RTT_LOG_DEBUG("[BT] CMD: FORWARD speed=%.1f", ctrl.speed_target);
            break;

        case 'B':
            ctrl.command      = BT_CMD_BACKWARD;
            ctrl.speed_target = -0.3f;
            ctrl.turn_target  =  0.0f;
            RTT_LOG_DEBUG("[BT] CMD: BACKWARD speed=%.1f", ctrl.speed_target);
            break;

        case 'L':
            ctrl.command      = BT_CMD_LEFT;
            ctrl.speed_target =  0.0f;
            ctrl.turn_target  = -0.5f;
            RTT_LOG_DEBUG("[BT] CMD: LEFT");
            break;

        case 'R':
            ctrl.command      = BT_CMD_RIGHT;
            ctrl.speed_target =  0.0f;
            ctrl.turn_target  =  0.5f;
            RTT_LOG_DEBUG("[BT] CMD: RIGHT");
            break;

        case 'S':
            ctrl.command      = BT_CMD_STOP;
            ctrl.speed_target =  0.0f;
            ctrl.turn_target  =  0.0f;
            RTT_LOG_DEBUG("[BT] CMD: STOP");
            break;

        case '+':
            ctrl.speed_target += 0.1f;
            if (ctrl.speed_target > 1.0f) {
                ctrl.speed_target = 1.0f;
                RTT_LOG_WARN("[BT] Speed upper limit: 1.0 m/s");
            }
            RTT_LOG_DEBUG("[BT] CMD: SPEED UP speed=%.1f", ctrl.speed_target);
            break;

        case '-':
            ctrl.speed_target -= 0.1f;
            if (ctrl.speed_target < -1.0f) {
                ctrl.speed_target = -1.0f;
                RTT_LOG_WARN("[BT] Speed lower limit: -1.0 m/s");
            }
            RTT_LOG_DEBUG("[BT] CMD: SPEED DOWN speed=%.1f", ctrl.speed_target);
            break;

        default:
            ctrl.updated = 0;
            RTT_LOG_WARN("[BT] Unknown cmd: '%c' (0x%02X)", (char)byte, byte);
            break;
    }
}

/*============================================================
 *  Send functions
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
    snprintf(buf, sizeof(buf), "%s:%.2f\r\n", label, value);
    BT_SendString(buf);
}

/*============================================================
 *  Parser reset (call from UART error callback)
 *============================================================*/
void BT_ResetParser(void)
{
    bt_state = BT_WAIT_START;
    bt_cmd   = 0;
    RTT_LOG_WARN("[BT] Parser reset");
}

/*============================================================
 *  Data access
 *============================================================*/
BT_ControlData* BT_GetControlData(void) { return &ctrl; }
uint8_t*        BT_GetRxBytePtr(void)   { return &rx_byte; }
