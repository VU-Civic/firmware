#ifndef __ONSET_DETECTION_H
#define __ONSET_DETECTION_H

#include "common.h"

typedef struct { uint32_t num_onsets, *indices; } onset_details_t;

void onset_detection_init(void);
onset_details_t onset_detection_invoke(const int16_t (*audio_samples)[AUDIO_BUFFER_SAMPLES / AUDIO_NUM_CHANNELS]);

#endif /* __ONSET_DETECTION_H */
