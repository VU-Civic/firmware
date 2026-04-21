// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "cellular.h"
#include "shot_detector.h"


// Shot Detection Type Definitions and Static Variables ----------------------------------------------------------------

typedef struct
{
   double onset_timestamp;
   float onset_magnitude, onset_aoa[3];
   uint8_t onset_detected, gunshot_probability;
   uint8_t onset_info_received, gunshot_info_received;
   uint8_t detection_handled;
} detection_info_t;

static volatile detection_info_t detection_info[AUDIO_NUM_DMAS_PER_CLIP];


// Public API Functions ------------------------------------------------------------------------------------------------

void shot_detector_init(void)
{
   // Initialize the detection info structure
   memset((void*)detection_info, 0, sizeof(detection_info));
}

void shot_detector_new_clip(void)
{
   // Make room for a new set of detection data
   for (uint32_t i = 1; i < AUDIO_NUM_DMAS_PER_CLIP; ++i)
      detection_info[i-1] = detection_info[i];
   memset((void*)&detection_info[AUDIO_NUM_DMAS_PER_CLIP-1], 0, sizeof(detection_info[0]));
}

void shot_detector_add_onset(volatile data_packet_t* volatile packet)
{
   // Add onset information to the end of the detection data
   detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].onset_detected = packet->onset_detected;
   detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].onset_timestamp = packet->onset_timestamp;
   detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].onset_magnitude = packet->onset_magnitude;
   detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].onset_aoa[0] = packet->angle_of_arrival[0];
   detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].onset_aoa[1] = packet->angle_of_arrival[1];
   detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].onset_aoa[2] = packet->angle_of_arrival[2];
   detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].onset_info_received = 1;
}

void shot_detector_add_classification(uint8_t gunshot_probability)
{
   // Add classification information to the end of the detection data
   detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].gunshot_probability = gunshot_probability;
   detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].gunshot_info_received = 1;
}

uint8_t shot_detector_pending_processing(void)
{
   // Return whether the most recent shot detection is pending processing
   return !detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].detection_handled &&
           detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].onset_info_received &&
           detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].gunshot_info_received;
}

void shot_detector_process_detections(uint8_t audio_clip_id)
{
   // Only proceed if the most recent shot detection has not yet been processed
   if (shot_detector_pending_processing())
   {
      // Iterate through all detection data to determine whether a shot is present
      uint8_t gunshot_detected = 0;
      for (uint32_t i = 0; i < AUDIO_NUM_DMAS_PER_CLIP; ++i)
         gunshot_detected |= detection_info[i].onset_detected;

      // TODO: Figure out when to send gunshot alert (use cellular)
      //device_info.device_config.shot_detection_min_threshold;
      //device_info.device_config.shot_detection_good_threshold;
      //if (gunshot_probability > )

      // Set the detection packet processed flag
      detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].detection_handled = 1;
   }
   // TODO: Check audio_clip_id logic - right now, it will be 1 less than expected the second we detect a new shot, because no evidence clip has yet started to be transmitted
}
