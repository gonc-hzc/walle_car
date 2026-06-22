#include "bluetooth.h"
#include "protocol.h"
#include "protocol_short_json.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#define BLUETOOTH_RX_BUFFER_SIZE 128
#define BLUETOOTH_RX_QUEUE_SIZE 512
#define BLUETOOTH_TX_BUFFER_SIZE 160

static uint8_t bluetooth_rx_byte;
static uint8_t bluetooth_rx_queue[BLUETOOTH_RX_QUEUE_SIZE];
static volatile uint16_t bluetooth_rx_head = 0;
static volatile uint16_t bluetooth_rx_tail = 0;
static volatile uint8_t bluetooth_rx_overflow = 0;
static volatile uint8_t bluetooth_rx_rearm_pending = 0;

static char bluetooth_rx_buffer[BLUETOOTH_RX_BUFFER_SIZE];
static uint16_t bluetooth_rx_len = 0;

static char bluetooth_tx_buffer[BLUETOOTH_TX_BUFFER_SIZE];
static uint16_t bluetooth_tx_len = 0;
static volatile uint8_t bluetooth_tx_pending = 0;

static uint16_t Bluetooth_AdvanceQueueIndex(uint16_t index)
{
    index++;
    if (index >= BLUETOOTH_RX_QUEUE_SIZE)
    {
        index = 0;
    }
    return index;
}

static void Bluetooth_StartReceive(void)
{
    if (HAL_UART_Receive_IT(&huart2, &bluetooth_rx_byte, 1) == HAL_OK)
    {
        bluetooth_rx_rearm_pending = 0;
    }
    else
    {
        bluetooth_rx_rearm_pending = 1;
    }
}

static void Bluetooth_QueueByte(uint8_t byte)
{
    uint16_t next_head = Bluetooth_AdvanceQueueIndex(bluetooth_rx_head);

    if (next_head == bluetooth_rx_tail)
    {
        bluetooth_rx_overflow = 1;
        return;
    }

    bluetooth_rx_queue[bluetooth_rx_head] = byte;
    bluetooth_rx_head = next_head;
}

static uint8_t Bluetooth_DequeueByte(uint8_t *byte)
{
    uint32_t primask;
    uint8_t has_byte = 0;

    primask = __get_PRIMASK();
    __disable_irq();
    if (bluetooth_rx_tail != bluetooth_rx_head)
    {
        *byte = bluetooth_rx_queue[bluetooth_rx_tail];
        bluetooth_rx_tail = Bluetooth_AdvanceQueueIndex(bluetooth_rx_tail);
        has_byte = 1;
    }
    if (!primask)
    {
        __enable_irq();
    }

    return has_byte;
}

static uint8_t Bluetooth_TakeOverflowFlag(void)
{
    uint32_t primask;
    uint8_t overflow;

    primask = __get_PRIMASK();
    __disable_irq();
    overflow = bluetooth_rx_overflow;
    bluetooth_rx_overflow = 0;
    if (!primask)
    {
        __enable_irq();
    }

    return overflow;
}

static void Bluetooth_SetReply(const char *text)
{
    uint16_t len = (uint16_t)strlen(text);

    if (len >= BLUETOOTH_TX_BUFFER_SIZE)
    {
        len = BLUETOOTH_TX_BUFFER_SIZE - 1;
    }

    memcpy(bluetooth_tx_buffer, text, len);
    bluetooth_tx_buffer[len] = '\0';
    bluetooth_tx_len = len;
    bluetooth_tx_pending = 1;
}

static void Bluetooth_StoreLine(void)
{
    int len;

    bluetooth_rx_buffer[bluetooth_rx_len] = '\0';

    if (ProtocolShortJson_SubmitLine(bluetooth_rx_buffer,
                                     bluetooth_tx_buffer,
                                     BLUETOOTH_TX_BUFFER_SIZE) ||
        Protocol_SubmitRcLine(bluetooth_rx_buffer,
                              bluetooth_tx_buffer,
                              BLUETOOTH_TX_BUFFER_SIZE))
    {
        bluetooth_tx_len = (uint16_t)strlen(bluetooth_tx_buffer);
        bluetooth_tx_pending = 1;
        bluetooth_rx_len = 0;
        return;
    }

    len = snprintf(bluetooth_tx_buffer, BLUETOOTH_TX_BUFFER_SIZE,
                   "BT RX: %s\r\n", bluetooth_rx_buffer);
    if (len < 0)
    {
        bluetooth_rx_len = 0;
        return;
    }

    if (len >= BLUETOOTH_TX_BUFFER_SIZE)
    {
        len = BLUETOOTH_TX_BUFFER_SIZE - 1;
        bluetooth_tx_buffer[len] = '\0';
    }

    bluetooth_tx_len = (uint16_t)len;
    bluetooth_tx_pending = 1;
    bluetooth_rx_len = 0;
}

static void Bluetooth_ProcessByte(uint8_t byte)
{
    if (byte == '\r')
    {
        return;
    }

    if (byte == '\n')
    {
        if (bluetooth_rx_len > 0)
        {
            Bluetooth_StoreLine();
        }
        return;
    }

    if (bluetooth_rx_len >= BLUETOOTH_RX_BUFFER_SIZE - 1)
    {
        bluetooth_rx_len = 0;
        Bluetooth_SetReply("BT ERR: overflow\r\n");
        return;
    }

    bluetooth_rx_buffer[bluetooth_rx_len++] = (char)byte;

    if (byte == '}')
    {
        Bluetooth_StoreLine();
    }
}

void Bluetooth_Init(void)
{
    bluetooth_rx_len = 0;
    bluetooth_rx_head = 0;
    bluetooth_rx_tail = 0;
    bluetooth_rx_overflow = 0;
    bluetooth_rx_rearm_pending = 0;
    bluetooth_tx_len = 0;
    bluetooth_tx_pending = 0;
    Bluetooth_StartReceive();
}

void Bluetooth_Task(void)
{
    uint8_t rx_byte;
    char tx_copy[BLUETOOTH_TX_BUFFER_SIZE];
    uint16_t tx_len;
    uint32_t primask;

    if (bluetooth_rx_rearm_pending)
    {
        Bluetooth_StartReceive();
    }

    if (Bluetooth_TakeOverflowFlag())
    {
        bluetooth_rx_len = 0;
        Bluetooth_SetReply("BT ERR: queue overflow\r\n");
    }

    while (Bluetooth_DequeueByte(&rx_byte))
    {
        Bluetooth_ProcessByte(rx_byte);
    }

    if (!bluetooth_tx_pending)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    tx_len = bluetooth_tx_len;
    if (tx_len > 0 && tx_len < BLUETOOTH_TX_BUFFER_SIZE)
    {
        memcpy(tx_copy, bluetooth_tx_buffer, tx_len);
        bluetooth_tx_pending = 0;
    }
    else
    {
        bluetooth_tx_pending = 0;
        tx_len = 0;
    }
    if (!primask)
    {
        __enable_irq();
    }

    if (tx_len > 0)
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)tx_copy, tx_len, 20);
    }
}

void Bluetooth_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        Bluetooth_QueueByte(bluetooth_rx_byte);
        Bluetooth_StartReceive();
    }
}

void Bluetooth_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        Bluetooth_StartReceive();
    }
}
