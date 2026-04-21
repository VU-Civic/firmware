#ifndef __SHOT_DETECTOR_HEADER_H__
#define __SHOT_DETECTOR_HEADER_H__

#include "common.h"

#ifdef CORE_CM4

void shot_detector_init(void);
void shot_detector_new_clip(void);
void shot_detector_add_onset(volatile data_packet_t* volatile packet);
void shot_detector_add_classification(uint8_t gunshot_probability);
uint8_t shot_detector_pending_processing(void);
cell_audio_transmit_command_t shot_detector_process_detections(uint8_t audio_clip_id);

#endif  // #ifdef CORE_CM4

#endif  // #ifndef __SHOT_DETECTOR_HEADER_H__
