#ifndef __REMOTE_CONTROL_H
#define __REMOTE_CONTROL_H

#include "main.h"

#define REMOTE_CONTROL_TIMEOUT_MS 500U
#define REMOTE_CONTROL_DEADZONE   50
#define REMOTE_CONTROL_INPUT_MAX  1000
#define REMOTE_CONTROL_MAX_SPEED  25.0f

typedef struct
{
    int16_t throttle;
    int16_t turn;
    float left_target;
    float right_target;
    uint8_t connected;
    uint32_t last_update_ms;
} RemoteControl_State_t;

void RemoteControl_Init(void);
uint8_t RemoteControl_UpdateTargets(float *left_target, float *right_target);
uint8_t RemoteControl_IsConnected(void);
const RemoteControl_State_t *RemoteControl_GetState(void);

#endif
