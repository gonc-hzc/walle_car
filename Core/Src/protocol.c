#include "protocol.h"
#include "bluetooth.h"
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PROTOCOL_RX_BUFFER_SIZE 128
#define PROTOCOL_RX_QUEUE_SIZE 512
#define PROTOCOL_TX_BUFFER_SIZE 96

static uint8_t protocol_rx_byte;
static uint8_t protocol_rx_queue[PROTOCOL_RX_QUEUE_SIZE];
static volatile uint16_t protocol_rx_head = 0;
static volatile uint16_t protocol_rx_tail = 0;
static volatile uint8_t protocol_rx_overflow = 0;
static volatile uint8_t protocol_rx_rearm_pending = 0;

static char protocol_rx_buffer[PROTOCOL_RX_BUFFER_SIZE];
static uint16_t protocol_rx_len = 0;

static char protocol_tx_buffer[PROTOCOL_TX_BUFFER_SIZE];
static uint16_t protocol_tx_len = 0;
static volatile uint8_t protocol_tx_pending = 0;

static volatile Protocol_RcCommand_t protocol_latest_cmd;
static volatile uint8_t protocol_cmd_pending = 0;
static volatile uint32_t protocol_last_rx_tick = 0;

static uint16_t Protocol_FormatAckReply(char *buffer,
                                        uint16_t buffer_size,
                                        const Protocol_RcCommand_t *cmd);
static void Protocol_StoreCommand(const Protocol_RcCommand_t *cmd);

static uint16_t Protocol_AdvanceQueueIndex(uint16_t index)
{
    index++;
    if (index >= PROTOCOL_RX_QUEUE_SIZE)
    {
        index = 0;
    }
    return index;
}

static void Protocol_StartReceive(void)
{
    if (HAL_UART_Receive_IT(&huart1, &protocol_rx_byte, 1) == HAL_OK)
    {
        protocol_rx_rearm_pending = 0;
    }
    else
    {
        protocol_rx_rearm_pending = 1;
    }
}

static void Protocol_QueueByte(uint8_t byte)
{
    uint16_t next_head = Protocol_AdvanceQueueIndex(protocol_rx_head);

    if (next_head == protocol_rx_tail)
    {
        protocol_rx_overflow = 1;
        return;
    }

    protocol_rx_queue[protocol_rx_head] = byte;
    protocol_rx_head = next_head;
}

static uint8_t Protocol_DequeueByte(uint8_t *byte)
{
    uint32_t primask;
    uint8_t has_byte = 0;

    primask = __get_PRIMASK();
    __disable_irq();
    if (protocol_rx_tail != protocol_rx_head)
    {
        *byte = protocol_rx_queue[protocol_rx_tail];
        protocol_rx_tail = Protocol_AdvanceQueueIndex(protocol_rx_tail);
        has_byte = 1;
    }
    if (!primask)
    {
        __enable_irq();
    }

    return has_byte;
}

static uint8_t Protocol_TakeOverflowFlag(void)
{
    uint32_t primask;
    uint8_t overflow;

    primask = __get_PRIMASK();
    __disable_irq();
    overflow = protocol_rx_overflow;
    protocol_rx_overflow = 0;
    if (!primask)
    {
        __enable_irq();
    }

    return overflow;
}

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

static void Protocol_StoreCommand(const Protocol_RcCommand_t *cmd)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    protocol_latest_cmd = *cmd;
    protocol_last_rx_tick = cmd->tick_ms;
    protocol_cmd_pending = 1;
    if (!primask)
    {
        __enable_irq();
    }
}

static void Protocol_SetReply(const char *text)
{
    uint16_t len = (uint16_t)strlen(text);

    if (len >= PROTOCOL_TX_BUFFER_SIZE)
    {
        len = PROTOCOL_TX_BUFFER_SIZE - 1;
    }

    memcpy(protocol_tx_buffer, text, len);
    protocol_tx_buffer[len] = '\0';
    protocol_tx_len = len;
    protocol_tx_pending = 1;
}

static uint16_t Protocol_FormatAckReply(char *buffer,
                                        uint16_t buffer_size,
                                        const Protocol_RcCommand_t *cmd)
{
    int len;

    if (buffer == NULL || buffer_size == 0)
    {
        return 0;
    }

    len = snprintf(buffer, buffer_size,
                   "{\"ok\":1,\"throttle\":%d,\"turn\":%d}\n",
                   cmd->throttle, cmd->turn);

    if (len < 0)
    {
        buffer[0] = '\0';
        return 0;
    }

    if (len >= buffer_size)
    {
        len = buffer_size - 1;
        buffer[len] = '\0';
    }

    return (uint16_t)len;
}

uint8_t Protocol_SubmitRcLine(const char *line, char *ack_buffer, uint16_t ack_buffer_size)
{
    Protocol_RcCommand_t cmd;

    if (line == NULL || line[0] == '\0')
    {
        return 0;
    }

    if (!Protocol_ParseRcJson(line, &cmd))
    {
        return 0;
    }

    Protocol_StoreCommand(&cmd);
    Protocol_FormatAckReply(ack_buffer, ack_buffer_size, &cmd);
    return 1;
}

uint8_t Protocol_SubmitRcValues(int throttle, int turn, Protocol_RcCommand_t *submitted_cmd)
{
    Protocol_RcCommand_t cmd;

    cmd.throttle = Protocol_ClampInput(throttle);
    cmd.turn = Protocol_ClampInput(turn);
    cmd.tick_ms = HAL_GetTick();

    Protocol_StoreCommand(&cmd);

    if (submitted_cmd != NULL)
    {
        *submitted_cmd = cmd;
    }

    return 1;
}

static void Protocol_StoreLine(void)
{
    protocol_rx_buffer[protocol_rx_len] = '\0';
    if (Protocol_SubmitRcLine(protocol_rx_buffer,
                              protocol_tx_buffer,
                              PROTOCOL_TX_BUFFER_SIZE))
    {
        protocol_tx_len = (uint16_t)strlen(protocol_tx_buffer);
        protocol_tx_pending = 1;
    }
    else
    {
        Protocol_SetReply("{\"ok\":0,\"err\":\"parse\"}\n");
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
    protocol_rx_head = 0;
    protocol_rx_tail = 0;
    protocol_rx_overflow = 0;
    protocol_rx_rearm_pending = 0;
    protocol_cmd_pending = 0;
    protocol_tx_pending = 0;
    protocol_tx_len = 0;
    protocol_last_rx_tick = 0;
    Protocol_StartReceive();
}

void Protocol_Task(void)
{
    uint8_t rx_byte;
    char tx_copy[PROTOCOL_TX_BUFFER_SIZE];
    uint16_t tx_len;
    uint32_t primask;

    if (protocol_rx_rearm_pending)
    {
        Protocol_StartReceive();
    }

    if (Protocol_TakeOverflowFlag())
    {
        protocol_rx_len = 0;
        Protocol_SetReply("{\"ok\":0,\"err\":\"overflow\"}\n");
    }

    while (Protocol_DequeueByte(&rx_byte))
    {
        Protocol_ProcessByte(rx_byte);
    }

    if (!protocol_tx_pending)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    tx_len = protocol_tx_len;
    if (tx_len > 0 && tx_len < PROTOCOL_TX_BUFFER_SIZE)
    {
        memcpy(tx_copy, protocol_tx_buffer, tx_len);
        protocol_tx_pending = 0;
    }
    else
    {
        protocol_tx_pending = 0;
        tx_len = 0;
    }
    if (!primask)
    {
        __enable_irq();
    }

    if (tx_len > 0)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)tx_copy, tx_len, 20);
    }
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
        Protocol_QueueByte(protocol_rx_byte);
        Protocol_StartReceive();
    }
    else if (huart->Instance == USART2)
    {
        Bluetooth_RxCpltCallback(huart);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_OREFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    if (huart->Instance == USART1)
    {
        Protocol_StartReceive();
    }
    else if (huart->Instance == USART2)
    {
        Bluetooth_ErrorCallback(huart);
    }
}
