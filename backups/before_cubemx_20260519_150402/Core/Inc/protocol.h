#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "main.h"

#define PROTOCOL_INPUT_MIN   (-1000)
#define PROTOCOL_INPUT_MAX   (1000)

typedef struct
{
    int16_t throttle;
    int16_t turn;
    uint32_t tick_ms;
} Protocol_RcCommand_t;

void Protocol_Init(void);
uint8_t Protocol_GetRcCommand(Protocol_RcCommand_t *cmd);
uint32_t Protocol_GetLastRxTick(void);

#endif
