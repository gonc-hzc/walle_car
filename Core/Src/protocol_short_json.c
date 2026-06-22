#include "protocol_short_json.h"
#include "protocol.h"
#include <stdio.h>
#include <stdlib.h>

static const char *ProtocolShortJson_SkipSpace(const char *pos)
{
    while (*pos == ' ' || *pos == '\t' || *pos == '\r' || *pos == '\n')
    {
        pos++;
    }

    return pos;
}

static uint8_t ProtocolShortJson_ParseKeyValue(const char **cursor,
                                               char *key,
                                               int *value)
{
    const char *pos = ProtocolShortJson_SkipSpace(*cursor);
    char *end = NULL;
    long parsed;

    if (pos[0] != '"' || pos[2] != '"' || (pos[1] != 'v' && pos[1] != 'w'))
    {
        return 0;
    }

    *key = pos[1];
    pos += 3;
    pos = ProtocolShortJson_SkipSpace(pos);
    if (*pos != ':')
    {
        return 0;
    }

    pos++;
    pos = ProtocolShortJson_SkipSpace(pos);
    parsed = strtol(pos, &end, 10);
    if (end == pos)
    {
        return 0;
    }

    *value = (int)parsed;
    *cursor = end;
    return 1;
}

static uint8_t ProtocolShortJson_ParseLine(const char *line, int *v, int *w)
{
    const char *pos;
    char key;
    int value;
    uint8_t has_v = 0;
    uint8_t has_w = 0;

    if (line == NULL)
    {
        return 0;
    }

    pos = ProtocolShortJson_SkipSpace(line);
    if (*pos != '{')
    {
        return 0;
    }

    pos++;
    while (1)
    {
        pos = ProtocolShortJson_SkipSpace(pos);
        if (*pos == '}')
        {
            return has_v && has_w;
        }

        if (!ProtocolShortJson_ParseKeyValue(&pos, &key, &value))
        {
            return 0;
        }

        if (key == 'v')
        {
            *v = value;
            has_v = 1;
        }
        else
        {
            *w = value;
            has_w = 1;
        }

        pos = ProtocolShortJson_SkipSpace(pos);
        if (*pos == ',')
        {
            pos++;
            continue;
        }
        if (*pos == '}')
        {
            return has_v && has_w;
        }

        return 0;
    }
}

static void ProtocolShortJson_SetAck(char *buffer,
                                     uint16_t buffer_size,
                                     const Protocol_RcCommand_t *cmd)
{
    int len;

    if (buffer == NULL || buffer_size == 0)
    {
        return;
    }

    len = snprintf(buffer, buffer_size,
                   "{\"o\":1,\"v\":%d,\"w\":%d}\n",
                   cmd->throttle,
                   cmd->turn);
    if (len < 0)
    {
        buffer[0] = '\0';
        return;
    }

    if (len >= buffer_size)
    {
        buffer[buffer_size - 1] = '\0';
    }
}

uint8_t ProtocolShortJson_SubmitLine(const char *line,
                                     char *ack_buffer,
                                     uint16_t ack_buffer_size)
{
    Protocol_RcCommand_t cmd;
    int v = 0;
    int w = 0;

    if (!ProtocolShortJson_ParseLine(line, &v, &w))
    {
        return 0;
    }

    Protocol_SubmitRcValues(v, w, &cmd);
    ProtocolShortJson_SetAck(ack_buffer, ack_buffer_size, &cmd);
    return 1;
}
