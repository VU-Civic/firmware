#ifndef __AUDIO_HEADER_H__
#define __AUDIO_HEADER_H__

#include "common.h"

void audio_init(void);

#ifdef CORE_CM7

void audio_start(void);

#else

uint8_t audio_process_new_data(cell_audio_transmit_command_t transmit_evidence);

#endif  // #ifdef CORE_CM7

#endif  // #ifndef __AUDIO_HEADER_H__
