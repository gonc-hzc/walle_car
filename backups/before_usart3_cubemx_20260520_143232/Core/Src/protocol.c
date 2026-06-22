#include "protocol.h"
#include "usart.h"
#include <stdlib.h>
#include <string.h>

#define PROTOCOL_RX_BUFFER_SIZE 128

static uint8_t protocol_rx_byte;
static char protocol_rx_buffer[PROTOCOL_RX_BUFFER_SIZE];
static uint16_t protocol_rx_len = 0;

static volatile Protocol_RcCommand_t protocol_latest_cmd;
static volatile uint8_t protocol_cmd_pending = 0;
static volatile uint32_t protocol_last_rx_tick = 0;

static int16_t Protocol_ClampInput(int value)
{
    if (value > PROTOCOL_INPUT_MAX) value = PROTOCOL_INPUT_MAX;
    if (value < PROTOCOL_INPUT_MIN) value = PROTOCOL_INPUT_MIN;
    return (int16_t)value;
}

static uint8_t Protocol_ParseIntKey(const char *json, const char *key, int *value)
{
    const char *pos = strstr(json, key);
    char *end = NULL;
    long parsed;

    if (pos == NULL)
    {
        return 0;
    }

    pos = strchr(pos, ':');
    if (pos == NULL)
    {
        return 0;
    }

    parsed = strtol(pos + 1, &end, 10);
    if (end == pos + 1)
    {
        return 0;
    }

    *value = (int)parsed;
    return 1;
}

static uint8_t Protocol_ParseRcJson(const char *json, Protocol_RcCommand_t *cmd)
{
    int throttle = 0;
    int turn = 0;
    uint8_t has_throttle;
    uint8_t has_turn;

    has_throttle = Protocol_ParseIntKey(json, "\"throttle\"", &throttle) ||
                   Protocol_ParseIntKey(json, "\"ly\"", &throttle) ||
                   Protocol_ParseIntKey(json, "\"v\"", &throttle);

    has_turn = Protocol_ParseIntKey(json, "\"turn\"", &turn) ||
               Protocol_ParseIntKey(json, "\"rx\"", &turn) ||
               Protocol_ParseIntKey(json, "\"w\"", &turn);

    if (!has_throttle || !has_turn)
    {
        return 0;
    }

    cmd->throttle = Protocol_ClampInput(throttle);
    cmd->turn = Protocol_ClampInput(turn);
    cmd->tick_ms = HAL_GetTick();
    return 1;
}

static void Protocol_StoreLine(void)
{
    Protocol_RcCommand_t cmd;

    protocol_rx_buffer[protocol_rx_len] = '\0';
    if (Protocol_ParseRcJson(protocol_rx_buffer, &cmd))
    {
        protocol_latest_cmd = cmd;
        protocol_last_rx_tick = cmd.tick_ms;
        protocol_cmd_pending = 1;
    }

    protocol_rx_len = 0;
}

static void Protocol_ProcessByte(uint8_t byte)
{
    if (byte == '\r')
    {
        return;
    }

    if (byte == '\n')
    {
        if (protocol_rx_len > 0)
        {
            Protocol_StoreLine();
        }
        return;
    }

    if (protocol_rx_len >= PROTOCOL_RX_BUFFER_SIZE - 1)
    {
        protocol_rx_len = 0;
    }

    protocol_rx_buffer[protocol_rx_len++] = (char)byte;

    if (byte == '}')
    {
        Protocol_StoreLine();
    }
}

void Protocol_Init(void)
{
    protocol_rx_len = 0;
    protocol_cmd_pending = 0;
    protocol_last_rx_tick = 0;
    HAL_UART_Receive_IT(&huart1, &protocol_rx_byte, 1);
}

uint8_t Protocol_GetRcCommand(Protocol_RcCommand_t *cmd)
{
    uint32_t primask;

    if (!protocol_cmd_pending)
    {
        return 0;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    cmd->throttle = protocol_latest_cmd.throttle;
    cmd->turn = protocol_latest_cmd.turn;
    cmd->tick_ms = protocol_latest_cmd.tick_ms;
    protocol_cmd_pending = 0;
    if (!primask)
    {
        __enable_irq();
    }

    return 1;
}

uint32_t Protocol_GetLastRxTick(void)
{
    return protocol_last_rx_tick;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        Protocol_ProcessByte(protocol_rx_byte);
        HAL_UART_Receive_IT(&huart1, &protocol_rx_byte, 1);
    }
}
