#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "stm32f4xx_hal.h"
#include "SEGGER_RTT.h"
#include <stdint.h>

/* UART2: TX=PA2  RX=PA3(blue_rx)  Baud=9600  HC-05 module */

/*============================================================
 *  Protocol: $CMD#
 *  Example:  $F# = Forward
 *            $S# = Stop
 *============================================================*/

/* Command enum */
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

/* Control data structure */
typedef struct {
    BT_Command command;
    float      speed_target;   /* target speed m/s (forward/backward) */
    float      turn_target;    /* turn value (-1.0 ~ 1.0) */
    uint8_t    updated;        /* new command flag */
} BT_ControlData;

/* Function declarations */
void            BT_Init(void);
void            BT_ParseByte(uint8_t byte);      /* call from UART RX interrupt */
void            BT_ProcessRxByte(uint8_t byte);  /* process command byte */
void            BT_ResetParser(void);            /* call from UART error callback */
void            BT_SendString(const char *str);
void            BT_SendFloat(const char *label, float value);
BT_ControlData* BT_GetControlData(void);
uint8_t*        BT_GetRxBytePtr(void);

#endif /* __BLUETOOTH_H */
