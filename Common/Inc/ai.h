#ifndef __AI_HEADER_H__
#define __AI_HEADER_H__

#include "common.h"

#ifdef CORE_CM4

void ai_comms_init(void);
void ai_comms_start(void);
void ai_send(const uint8_t *data, uint16_t data_length);

#endif  // #ifdef CORE_CM4

#endif  // #ifndef __AI_HEADER_H__
