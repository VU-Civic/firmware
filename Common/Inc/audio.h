#ifndef __AUDIO_HEADER_H__
#define __AUDIO_HEADER_H__

#include "common.h"

void audio_init(void);

#ifdef CORE_CM7

void audio_start(void);

#endif  // #ifdef CORE_CM7

#endif  // #ifndef __AUDIO_HEADER_H__
