#ifndef __ONSET_DETECTION_H
#define __ONSET_DETECTION_H

#include "common.h"

#ifdef CORE_CM7

void onset_detection_init(void);
onset_details_t onset_detection_invoke(const int16_t (*audio_samples)[AUDIO_BUFFER_SAMPLES_PER_CHANNEL], channel_alarms_t channel_alarms);

#endif  // #ifdef CORE_CM7

#endif /* __ONSET_DETECTION_H */
