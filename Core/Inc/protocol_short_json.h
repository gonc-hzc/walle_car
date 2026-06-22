#ifndef __PROTOCOL_SHORT_JSON_H
#define __PROTOCOL_SHORT_JSON_H

#include "main.h"

uint8_t ProtocolShortJson_SubmitLine(const char *line,
                                     char *ack_buffer,
                                     uint16_t ack_buffer_size);

#endif
