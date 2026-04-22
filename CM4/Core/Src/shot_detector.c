// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "cellular.h"
#include "shot_detector.h"


// Shot Detection Type Definitions and Static Variables ----------------------------------------------------------------

typedef struct
{
   double onset_timestamp;
   float onset_magnitude, onset_aoa[3], gunshot_classification;
   uint8_t onset_detected, onset_info_received, gunshot_info_received;
   uint8_t detection_handled;
} detection_info_t;

static volatile detection_info_t detection_info[AUDIO_NUM_DMAS_PER_CLIP];
static uint8_t incident_occurring, incident_packets_received;
static cell_audio_transmit_command_t audio_transmit_command;
static alert_message_t alert_message;


// Private Helper Functions --------------------------------------------------------------------------------------------

#ifndef NON_PRODUCTION

static void fill_detection_event(event_info_t *event_info, volatile detection_info_t *detection_info)
{
   // Fill in the event info structure
   event_info->timestamp = detection_info->onset_timestamp;
   event_info->magnitude = detection_info->onset_magnitude;
   event_info->angle_of_arrival[0] = detection_info->onset_aoa[0];
   event_info->angle_of_arrival[1] = detection_info->onset_aoa[1];
   event_info->angle_of_arrival[2] = detection_info->onset_aoa[2];
}

#endif


// Public API Functions ------------------------------------------------------------------------------------------------

void shot_detector_init(void)
{
   // Initialize the detection info structure
   memset((void*)detection_info, 0, sizeof(detection_info));
   incident_occurring = incident_packets_received = 0;
   alert_message.device_id = device_info.device_id;
   audio_transmit_command = CELL_AUDIO_NO_TRANSMIT;
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

void shot_detector_add_classification(float gunshot_classification)
{
   // Add classification information to the end of the detection data
   detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].gunshot_classification= gunshot_classification;
   detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].gunshot_info_received = 1;
}

uint8_t shot_detector_pending_processing(void)
{
   // Return whether the most recent shot detection is pending processing
   return !detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].detection_handled &&
           detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].onset_info_received &&
           detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].gunshot_info_received;
}

cell_audio_transmit_command_t shot_detector_process_detections(uint8_t audio_clip_id)
{
   // Only proceed if the most recent shot detection has not yet been processed
   if (shot_detector_pending_processing())
   {
#ifndef NON_PRODUCTION

      // Determine whether a shot was detected in the most recent audio clip
      uint8_t gunshot_detected = 0;
      static float max_classification = 0.0f;
      for (uint32_t i = 0; i < AUDIO_NUM_DMAS_PER_CLIP; ++i)
         gunshot_detected |= detection_info[i].onset_detected;
      gunshot_detected &= (detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].gunshot_classification >= device_info.device_config.shot_detection_min_threshold);  // TODO: device_info.device_config.shot_detection_good_threshold for evidence?;

      // Accumulate evidence during an active incident, sending alerts once per full clip
      if (incident_occurring)
      {
         audio_transmit_command = CELL_AUDIO_TRANSMIT_CONTINUE;
         max_classification = (max_classification > detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].gunshot_classification) ? max_classification : detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].gunshot_classification;
         if (detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].onset_detected)
            fill_detection_event(&alert_message.events[alert_message.num_events++], &detection_info[AUDIO_NUM_DMAS_PER_CLIP-1]);
         if (++incident_packets_received >= AUDIO_NUM_DMAS_PER_CLIP)
         {
            for (uint8_t i = 0; i < alert_message.num_events; ++i)
               alert_message.events[i].confidence = max_classification;
            alert_message.audio_clip_id = audio_clip_id;
            incident_occurring = (max_classification >= device_info.device_config.shot_detection_min_threshold) ? alert_message.num_events : 0;
            if (incident_occurring)
               cell_transmit_alert(&alert_message);
            else
               audio_transmit_command = CELL_AUDIO_TRANSMIT_END;
            alert_message.num_events = incident_packets_received = 0;
            max_classification = 0.0f;
         }
      }
      else if (gunshot_detected)
      {
         // Add detected onsets from any point in the most recent audio clip
         alert_message.num_events = 0;
         max_classification = detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].gunshot_classification;
         for (uint32_t i = 0; i < AUDIO_NUM_DMAS_PER_CLIP; ++i)
            if (detection_info[i].onset_detected)
               fill_detection_event(&alert_message.events[alert_message.num_events++], &detection_info[i]);
         audio_transmit_command = CELL_AUDIO_TRANSMIT_BEGIN;
         incident_occurring = incident_packets_received = 1;
      }
      else
         audio_transmit_command = CELL_AUDIO_NO_TRANSMIT;

#endif

      // Set the detection packet processed flag
      detection_info[AUDIO_NUM_DMAS_PER_CLIP-1].detection_handled = 1;
   }
   return audio_transmit_command;
}
