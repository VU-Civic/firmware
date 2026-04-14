#ifndef __SHOT_DETECTOR_HEADER_H__
#define __SHOT_DETECTOR_HEADER_H__

#include "common.h"

#ifdef CORE_CM4

void shot_detector_init(void);
void shot_detector_new_clip(void);
void shot_detector_add_onset(uint8_t onset_detected, double onset_timestamp);
void shot_detector_add_classification(uint8_t gunshot_probability);
uint8_t shot_detector_pending_processing(void);
void shot_detector_process_detections(void);

#endif  // #ifdef CORE_CM4

#endif  // #ifndef __SHOT_DETECTOR_HEADER_H__
