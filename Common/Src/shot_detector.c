#ifdef CORE_CM4

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "cellular.h"
#include "shot_detector.h"


// Shot Detection Type Definitions and Static Variables ----------------------------------------------------------------


#define SHOT_DETECTOR_RING_EXTRA_SLOTS       4
#define SHOT_DETECTOR_RING_SIZE              (AUDIO_NUM_DMAS_PER_CLIP + SHOT_DETECTOR_RING_EXTRA_SLOTS)

typedef struct
{
   double onset_timestamp;
   float onset_magnitude, onset_aoa[3], gunshot_classification;
   uint8_t onset_detected, onset_info_received, gunshot_info_received;
   uint8_t detection_handled;
} detection_info_t;

static volatile historical_onset_t old_onsets[HISTORICAL_ONSETS_MAX_SIZE];
static volatile detection_info_t detection_info[SHOT_DETECTOR_RING_SIZE];
static volatile uint32_t detection_ring_head, detection_ring_count;
static volatile uint32_t detection_pending_count, next_onset_index;
static alert_message_t alert_message;


// Private Helper Functions --------------------------------------------------------------------------------------------

static void fill_detection_event(event_info_t *event_info, volatile detection_info_t* const detection_info)
{
   // Fill in the event info structure
   event_info->timestamp = detection_info->onset_timestamp;
   event_info->magnitude = detection_info->onset_magnitude;
   event_info->angle_of_arrival[0] = detection_info->onset_aoa[0];
   event_info->angle_of_arrival[1] = detection_info->onset_aoa[1];
   event_info->angle_of_arrival[2] = detection_info->onset_aoa[2];
}

static uint32_t detection_ring_index(uint32_t logical_index)
{
   return (detection_ring_head + logical_index) % SHOT_DETECTOR_RING_SIZE;
}

static uint32_t detection_ring_index_from_head(uint32_t ring_head, uint32_t logical_index)
{
   return (ring_head + logical_index) % SHOT_DETECTOR_RING_SIZE;
}

static uint32_t detection_latest_index(void)
{
   return detection_ring_index(detection_ring_count - 1U);
}

static uint32_t detection_oldest_pending_offset(void)
{
   return detection_ring_count - detection_pending_count;
}


// Public API Functions ------------------------------------------------------------------------------------------------

void shot_detector_init(void)
{
   // Initialize the detection info structure
   memset((void*)old_onsets, 0, sizeof(old_onsets));
   memset((void*)detection_info, 0, sizeof(detection_info));
   detection_ring_head = detection_ring_count = detection_pending_count = next_onset_index = 0;
   alert_message.device_id = device_info.device_id;
}

void shot_detector_new_clip(void)
{
   // If full, drop oldest entry to make room for this new packet
   if (detection_ring_count == SHOT_DETECTOR_RING_SIZE)
   {
      if (detection_pending_count)
         --detection_pending_count;
      detection_ring_head = (detection_ring_head + 1) % SHOT_DETECTOR_RING_SIZE;
      --detection_ring_count;
   }

   // Append a new newest slot and mark it pending
   const uint32_t new_slot = detection_ring_index(detection_ring_count);
   memset((void*)&detection_info[new_slot], 0, sizeof(detection_info[0]));
   ++detection_ring_count;
   ++detection_pending_count;
}

void shot_detector_add_onset(volatile data_packet_t* volatile packet)
{
   // Add onset information to the end of the detection data
   if (detection_ring_count)
   {
      volatile detection_info_t* const latest_detection = &detection_info[detection_latest_index()];
      latest_detection->onset_detected = packet->onset_detected;
      latest_detection->onset_timestamp = packet->onset_timestamp;
      latest_detection->onset_magnitude = packet->onset_magnitude;
      latest_detection->onset_aoa[0] = packet->angle_of_arrival[0];
      latest_detection->onset_aoa[1] = packet->angle_of_arrival[1];
      latest_detection->onset_aoa[2] = packet->angle_of_arrival[2];
      latest_detection->onset_info_received = 1;
   }

   // If an onset was detected, add it to the historical onset list
   if (packet->onset_detected)
   {
      old_onsets[next_onset_index].onset_timestamp = packet->onset_timestamp;
      old_onsets[next_onset_index].onset_magnitude = packet->onset_magnitude;
      old_onsets[next_onset_index].onset_aoa[0] = packet->angle_of_arrival[0];
      old_onsets[next_onset_index].onset_aoa[1] = packet->angle_of_arrival[1];
      old_onsets[next_onset_index].onset_aoa[2] = packet->angle_of_arrival[2];
      next_onset_index = (next_onset_index + 1) % HISTORICAL_ONSETS_MAX_SIZE;
   }
}

void shot_detector_add_classification(float gunshot_classification)
{
   // Add classification information to the end of the detection data
   if (detection_ring_count)
   {
      volatile detection_info_t* const latest_detection = &detection_info[detection_latest_index()];
      latest_detection->gunshot_classification = gunshot_classification;
      latest_detection->gunshot_info_received = 1;
   }
}

uint8_t shot_detector_pending_processing(void)
{
   // Return whether the most recent shot detection is pending processing
   if (detection_pending_count)
   {
      const volatile detection_info_t* const detection = &detection_info[detection_ring_index(detection_oldest_pending_offset())];
      return !detection->detection_handled && detection->onset_info_received && detection->gunshot_info_received;
   }
   else
      return 0;
}

uint8_t shot_detector_process_detections(uint8_t audio_clip_id)
{
   // Create statically allocated detection processing state variables
   static uint8_t incident_occurring = 0, incident_packets_received = 0, transmit_audio = 0;
   static float max_classification = 0.0f;

   // Only proceed if the most recent shot detection has not yet been processed
   if (shot_detector_pending_processing())
   {
      // Freeze queue processing locations and process the oldest pending packet first
      const uint32_t ring_head_snapshot = detection_ring_head, target_offset = detection_ring_count - detection_pending_count;
      volatile detection_info_t* const latest_detection = &detection_info[detection_ring_index_from_head(ring_head_snapshot, target_offset)];

      // Window for this target packet: logically oldest -> newest, ending at target
      const uint32_t window_start = ((target_offset + 1) > AUDIO_NUM_DMAS_PER_CLIP) ? (target_offset + 1 - AUDIO_NUM_DMAS_PER_CLIP) : 0U;

      // Determine whether a shot was detected in the most recent audio clip
      uint8_t gunshot_detected = 0;
      for (uint32_t i = window_start; i <= target_offset; ++i)
         gunshot_detected |= detection_info[detection_ring_index_from_head(ring_head_snapshot, i)].onset_detected;
      if (!device_info.device_config.test_mode_start_time)
         gunshot_detected &= (latest_detection->gunshot_classification >= device_info.device_config.shot_detection_min_threshold);

      // Accumulate evidence during an active incident, sending alerts once per full clip
      if (incident_occurring)
      {
         max_classification = (max_classification > latest_detection->gunshot_classification) ? max_classification : latest_detection->gunshot_classification;
         transmit_audio |= (max_classification >= device_info.device_config.shot_detection_good_threshold);
         if (latest_detection->onset_detected)
            fill_detection_event(&alert_message.events[alert_message.num_events++], latest_detection);
         if (++incident_packets_received >= AUDIO_NUM_DMAS_PER_CLIP)
         {
            for (uint8_t i = 0; i < alert_message.num_events; ++i)
               alert_message.events[i].confidence = max_classification;
            alert_message.audio_clip_id = audio_clip_id;
            incident_occurring = (max_classification >= device_info.device_config.shot_detection_min_threshold) ? alert_message.num_events : 0;
            if (incident_occurring || device_info.device_config.test_mode_start_time)
               cell_transmit_alert(&alert_message);
            alert_message.num_events = incident_packets_received = 0;
            max_classification = 0.0f;
         }
      }
      else if (gunshot_detected)
      {
         // Add detected onsets from any point in the most recent audio clip
         alert_message.num_events = 0;
         max_classification = latest_detection->gunshot_classification;
         transmit_audio = (max_classification >= device_info.device_config.shot_detection_good_threshold);
         for (uint32_t i = window_start; i <= target_offset; ++i)
         {
            volatile detection_info_t* const detection = &detection_info[detection_ring_index_from_head(ring_head_snapshot, i)];
            if (detection->onset_detected)
               fill_detection_event(&alert_message.events[alert_message.num_events++], detection);
         }
         incident_occurring = incident_packets_received = 1;
      }

      // Set the detection packet processed flag
      latest_detection->detection_handled = 1;
      --detection_pending_count;

      // Drop handled entries that are older than any remaining pending packet
      while (detection_ring_count > detection_pending_count)
      {
         volatile detection_info_t* const oldest = &detection_info[detection_ring_head];
         if (!oldest->detection_handled)
            break;
         detection_ring_head = (detection_ring_head + 1U) % SHOT_DETECTOR_RING_SIZE;
         --detection_ring_count;
      }
   }
   return incident_occurring && transmit_audio;
}

void shot_detector_find_historical_onsets(double starting_timestamp, historical_onset_message_t *onsets_packet)
{
   onsets_packet->num_onsets = 0;
   const int32_t search_start = (next_onset_index == 0) ? (HISTORICAL_ONSETS_MAX_SIZE - 1) : (int32_t)(next_onset_index - 1);
   for (int32_t onset_idx = search_start; (onset_idx >= 0) && (old_onsets[onset_idx].onset_timestamp >= starting_timestamp) && (onsets_packet->num_onsets < HISTORICAL_ONSETS_MAX_RESULTS); --onset_idx)
      if (old_onsets[onset_idx].onset_timestamp <= (starting_timestamp + HISTORICAL_ONSETS_SEARCH_SECONDS))
         onsets_packet->onsets[onsets_packet->num_onsets++] = old_onsets[onset_idx];
   if (next_onset_index > 0)
      for (int32_t onset_idx = HISTORICAL_ONSETS_MAX_SIZE - 1; (onset_idx >= next_onset_index) && (old_onsets[onset_idx].onset_timestamp >= starting_timestamp) && (onsets_packet->num_onsets < HISTORICAL_ONSETS_MAX_RESULTS); --onset_idx)
         if (old_onsets[onset_idx].onset_timestamp <= (starting_timestamp + HISTORICAL_ONSETS_SEARCH_SECONDS))
            onsets_packet->onsets[onsets_packet->num_onsets++] = old_onsets[onset_idx];
}

#endif  // #ifdef CORE_CM4
