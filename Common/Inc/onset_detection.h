#ifndef __ONSET_DETECTION_H
#define __ONSET_DETECTION_H

#include "common.h"

#ifdef CORE_CM7

void onset_detection_init(void);
double onset_detection_invoke(double packet_timestamp, const int16_t (*audio_samples)[AUDIO_BUFFER_SAMPLES_PER_CHANNEL], volatile data_packet_t* const packet);

#endif  // #ifdef CORE_CM7

#endif /* __ONSET_DETECTION_H */
