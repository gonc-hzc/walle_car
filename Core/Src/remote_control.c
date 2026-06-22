#include "remote_control.h"
#include "protocol.h"
#include <stdlib.h>

static RemoteControl_State_t remote_state;

static int16_t RemoteControl_ClampInput(int32_t value)
{
    if (value > REMOTE_CONTROL_INPUT_MAX) value = REMOTE_CONTROL_INPUT_MAX;
    if (value < -REMOTE_CONTROL_INPUT_MAX) value = -REMOTE_CONTROL_INPUT_MAX;
    return (int16_t)value;
}

static int16_t RemoteControl_ApplyDeadzone(int16_t value)
{
    int32_t magnitude = abs(value);
    int32_t scaled;

    if (magnitude <= REMOTE_CONTROL_DEADZONE)
    {
        return 0;
    }

    scaled = (magnitude - REMOTE_CONTROL_DEADZONE) * REMOTE_CONTROL_INPUT_MAX;
    scaled /= (REMOTE_CONTROL_INPUT_MAX - REMOTE_CONTROL_DEADZONE);

    if (value < 0)
    {
        scaled = -scaled;
    }

    return RemoteControl_ClampInput(scaled);
}

static float RemoteControl_FilterTarget(float old_value, float new_value)
{
    if ((old_value > 0.0f && new_value < 0.0f) ||
        (old_value < 0.0f && new_value > 0.0f))
    {
        return new_value;
    }

    return (old_value + new_value * 3.0f) / 4.0f;
}

static void RemoteControl_SetTargetsFromCommand(const Protocol_RcCommand_t *cmd)
{
    int16_t throttle = RemoteControl_ApplyDeadzone(cmd->throttle);
    int16_t turn = RemoteControl_ApplyDeadzone(cmd->turn);
    int16_t left_mix = RemoteControl_ClampInput((int32_t)throttle + turn);
    int16_t right_mix = RemoteControl_ClampInput((int32_t)throttle - turn);
    float new_left = (float)left_mix * REMOTE_CONTROL_MAX_SPEED / REMOTE_CONTROL_INPUT_MAX;
    float new_right = (float)right_mix * REMOTE_CONTROL_MAX_SPEED / REMOTE_CONTROL_INPUT_MAX;

    remote_state.throttle = throttle;
    remote_state.turn = turn;
    if (throttle == 0 && turn == 0)
    {
        remote_state.left_target = 0.0f;
        remote_state.right_target = 0.0f;
    }
    else
    {
        remote_state.left_target = RemoteControl_FilterTarget(remote_state.left_target, new_left);
        remote_state.right_target = RemoteControl_FilterTarget(remote_state.right_target, new_right);
    }
    remote_state.last_update_ms = cmd->tick_ms;
    remote_state.connected = 1;
}

void RemoteControl_Init(void)
{
    remote_state.throttle = 0;
    remote_state.turn = 0;
    remote_state.left_target = 0.0f;
    remote_state.right_target = 0.0f;
    remote_state.connected = 0;
    remote_state.last_update_ms = 0;
}

uint8_t RemoteControl_UpdateTargets(float *left_target, float *right_target)
{
    Protocol_RcCommand_t cmd;
    uint32_t now = HAL_GetTick();

    if (Protocol_GetRcCommand(&cmd))
    {
        RemoteControl_SetTargetsFromCommand(&cmd);
    }

    if (remote_state.last_update_ms == 0 ||
        now - remote_state.last_update_ms > REMOTE_CONTROL_TIMEOUT_MS)
    {
        remote_state.connected = 0;
        remote_state.throttle = 0;
        remote_state.turn = 0;
        remote_state.left_target = 0.0f;
        remote_state.right_target = 0.0f;
    }

    *left_target = remote_state.left_target;
    *right_target = remote_state.right_target;

    return remote_state.connected;
}

uint8_t RemoteControl_IsConnected(void)
{
    return remote_state.connected;
}

const RemoteControl_State_t *RemoteControl_GetState(void)
{
    return &remote_state;
}

