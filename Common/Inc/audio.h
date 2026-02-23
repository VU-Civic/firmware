#ifndef __AUDIO_HEADER_H__
#define __AUDIO_HEADER_H__

#include "common.h"

void audio_init(void);
void audio_start(void);

#ifdef CORE_CM4

uint8_t audio_new_data_available(void);
void audio_process_new_data(cell_audio_transmit_command_t transmit_evidence);

#endif  // #ifdef CORE_CM4

#endif  // #ifndef __AUDIO_HEADER_H__
