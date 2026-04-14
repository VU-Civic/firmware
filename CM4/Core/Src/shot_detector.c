// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "cellular.h"
#include "shot_detector.h"


// Shot Detection Type Definitions and Static Variables ----------------------------------------------------------------

typedef struct
{
   double onset_timestamp;
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

void shot_detector_add_onset(uint8_t onset_detected, double onset_timestamp)
{
   // Add onset information to the end of the detection data
   detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].onset_detected = onset_detected;
   detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].onset_timestamp = onset_timestamp;
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

void shot_detector_process_detections(void)
{
   // Only proceed if the most recent shot detection has not yet processed
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
}
