#ifndef __COMMON_HEADER_H__
#define __COMMON_HEADER_H__

// Common Header Inclusions --------------------------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "config_private.h"


// Custom Application Definitions --------------------------------------------------------------------------------------

#define STRINGIZE_HELPER(x)                  #x
#define STRINGIZE(x)                         STRINGIZE_HELPER(x)

#define FIRMWARE_BUILD_TIMESTAMP             _DATETIME
#define FIRMWARE_REVISION                    STRINGIZE(_FW_REVISION)
#define FIRMWARE_VERSION_LENGTH              8

#define AUDIO_NUM_CHANNELS                   4
#define AUDIO_SAMPLE_RATE_HZ                 48000
#define AUDIO_BUFFER_SAMPLES                 32000
#define AUDIO_DIGITAL_GAIN                   0xD1  // 0xC9 = 0dB, 0xFF = 27dB, 0.5dB increase per value
#define AUDIO_SILENCE_TIMEOUT_SECONDS        30
#define AUDIO_BUFFER_SAMPLES_PER_CHANNEL     (AUDIO_BUFFER_SAMPLES / AUDIO_NUM_CHANNELS)
#define AUDIO_NUM_DMAS_PER_CLIP              (AUDIO_SAMPLE_RATE_HZ * AUDIO_NUM_CHANNELS / AUDIO_BUFFER_SAMPLES)

#define CELL_CONNECTION_TIMEOUT_SECONDS      3600
#define CELL_BAD_CONN_TIMEOUT_MINUTES        10
#define CELL_BAD_PDP_TIMEOUT_MINUTES         2
#define CELL_BAD_MQTT_TIMEOUT_MINUTES        5
#define CELL_SERVER_RESPONSE_TIMEOUT_SECONDS 5
#define CELL_IMEI_LENGTH                     15
#define CELL_IMSI_LENGTH                     15
#ifdef CELL_MQTT_USE_BINARY_PUBLISH
#define CELL_MQTT_MAX_PAYLOAD_SIZE_BYTES     1016
#else
#define CELL_MQTT_MAX_PAYLOAD_SIZE_BYTES     762
#endif
#define CELL_MQTT_MESSAGE_INDEX_MASK         0x7F
#define CELL_MQTT_MESSAGE_FINAL_MASK         0x80
#define CELL_EVIDENCE_MAX_PAYLOAD_SIZE       (CELL_MQTT_MAX_PAYLOAD_SIZE_BYTES - 9)

#define USB_VID                              4617
#define USB_PID                              2829
#define USB_PRODUCT_STRING                   "CivicAlert"
#define USB_MANUFACTURER_STRING              "CivicAlert"

#define HISTORICAL_ONSETS_SEARCH_SECONDS     4
#define HISTORICAL_ONSETS_MAX_SIZE           100
#define HISTORICAL_ONSETS_MAX_RESULTS        23  // Constrained by the size of an MQTT packet, otherwise (HISTORICAL_ONSETS_SEARCH_SECONDS * AUDIO_NUM_DMAS_PER_CLIP)

#define OPUS_FRAME_DELIMITER                 0xAA
#define OPUS_ENCODED_BIT_RATE                15000
#define OPUS_MS_PER_FRAME                    20
#define OPUS_HISTORY_MS                      960

#define MIC_CH1_CH2_OFFSET                   {   0.0f, 0.065f }
#define MIC_CH1_CH3_OFFSET                   { 0.065f,   0.0f }
#define MIC_CH1_CH4_OFFSET                   { 0.065f, 0.065f }

#define AOA_MIN_ELEVATION_DEG                -15.0f

#define AI_FIRMWARE_VERSION_LENGTH           8
#define AI_NUM_CLASSES                       1
#define AI_COMMS_TIMEOUT_SECONDS             10

#define MAX_NUM_EVENTS_PER_ALERT             (2 * AUDIO_NUM_DMAS_PER_CLIP)

#define TEST_MODE_AUTO_DISABLE_SECONDS       300

#ifdef PACKET_FULL_AUDIO
#define PACKET_AUDIO_SAMPLES                 AUDIO_BUFFER_SAMPLES
#else
#define PACKET_AUDIO_SAMPLES                 AUDIO_BUFFER_SAMPLES_PER_CHANNEL
#endif

#define PACKET_START_DELIMITER               { 0xAE, 0xA0, 0xA2, 0xF5 }
#define PACKET_END_DELIMITER                 { 0xFE, 0xF0, 0xF2, 0x25 }
#define PACKET_RESPONSE_DELIMITER            { 0xFE, 0xF9 }
#define PACKET_RESPONSE_ACK                  { 0x01, 0x02 }

#ifndef MIN
  #define MIN(a, b)                          (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
  #define MAX(a, b)                          (((a) > (b)) ? (a) : (b))
#endif

#define DEFAULT_CONFIG_INITIALIZATION_TAG                         74
#define DEFAULT_DEVICE_STATUS_UPDATE_INTERVAL_MINUTES             15
#define DEFAULT_SD_STORAGE_AUDIO_CLIP_MIN_SECONDS                 3
#define DEFAULT_SD_STORAGE_PROBABILITY_THRESHOLD                  0.1f
#define DEFAULT_MIN_SHOT_ALERT_PROBABILITY                        0.01f
#define DEFAULT_GOOD_SHOT_ALERT_PROBABILITY                       0.5f
#define DEFAULT_MQTT_DEVICE_INFO_QOS                              0
#define DEFAULT_MQTT_ALERT_QOS                                    0
#define DEFAULT_MQTT_AUDIO_CLIP_QOS                               0

#define DMA_STREAM0_4_INDEX                  0U
#define DMA_STREAM1_5_INDEX                  6U
#define DMA_STREAM2_6_INDEX                  16U
#define DMA_STREAM3_7_INDEX                  22U

#define CORE_TO_CORE_HSEM_NUMBER             1U


// Global Type Definitions ---------------------------------------------------------------------------------------------

typedef struct
{
   volatile uint32_t ISR;
   volatile uint32_t Reserved0;
   volatile uint32_t IFCR;
} dma_int_registers_t;

typedef struct
{
   volatile uint32_t ISR;
   volatile uint32_t IFCR;
} bdma_int_registers_t;

typedef struct __attribute__ ((__packed__))
{
   uint8_t initialized_tag;
   uint8_t mqtt_device_info_qos, mqtt_alert_qos, mqtt_audio_qos;
   float shot_detection_min_threshold, shot_detection_good_threshold, storage_classification_threshold;
   uint8_t audio_clip_length_seconds, device_status_transmission_interval_minutes;
   uint8_t bad_audio_restart_attempted, bad_ai_restart_attempted;
   uint32_t test_mode_start_time;
   uint8_t reserved[8];
} config_data_t;

typedef struct __attribute__ ((__packed__))
{
   float storage_classification_threshold;
   uint8_t audio_clip_length_seconds;
} ai_config_t;

typedef struct __attribute__ ((__packed__))
{
   uint8_t ai_firmware_version[AI_FIRMWARE_VERSION_LENGTH];
   float class_outputs[AI_NUM_CLASSES];
} ai_result_t;

typedef struct __attribute__ ((__packed__))
{
   double onset_timestamp;
   float onset_magnitude, onset_aoa[3];
} historical_onset_t;

typedef union
{
   uint8_t alarms;
   struct {
      uint8_t ch1 : 1;
      uint8_t ch2 : 1;
      uint8_t ch3 : 1;
      uint8_t ch4 : 1;
      uint8_t     : 4;
   } alarm;
} channel_alarms_t;

typedef struct __attribute__ ((__packed__, aligned (16)))
{
   uint8_t start_delimiter[4];
   int16_t audio[PACKET_AUDIO_SAMPLES];
   double timestamp;
   float lat, lon, ht;
   int32_t q1, q2, q3;
   char imei[CELL_IMEI_LENGTH+1], imsi[CELL_IMSI_LENGTH+1];
   ai_config_t ai_config;
   channel_alarms_t channel_alarms;
   uint8_t onset_detected;
   double onset_timestamp;
   float onset_magnitude, angle_of_arrival[3];
   uint8_t reserved[9];
   uint8_t end_delimiter[4];
} data_packet_t;

typedef struct __attribute__ ((__packed__, aligned (16)))
{
   data_packet_t packets[2];
   int32_t audio_read_index, audio_clip_complete;
} data_packet_container_t;

typedef struct __attribute__ ((__packed__))
{
   uint64_t device_id;
   char imsi[CELL_IMSI_LENGTH+1];
   uint8_t firmware_version[FIRMWARE_VERSION_LENGTH];
   uint8_t ai_firmware_version[AI_FIRMWARE_VERSION_LENGTH];
   float lat, lon, ht;
   int32_t q1, q2, q3;
   uint8_t signal_power, signal_quality, reserved;
   channel_alarms_t channel_alarms;
   config_data_t device_config;
} device_info_t;

typedef struct __attribute__ ((__packed__))
{
   double timestamp;
   float confidence, magnitude, angle_of_arrival[3];
} event_info_t;

typedef struct __attribute__ ((__packed__))
{
   uint64_t device_id;
   int32_t sensor_q1, sensor_q2, sensor_q3;
   float sensor_lat, sensor_lon, sensor_ht;
   uint8_t audio_clip_id, num_events, cell_signal_power, cell_signal_quality;
   event_info_t events[MAX_NUM_EVENTS_PER_ALERT];
} alert_message_t;

typedef struct __attribute__ ((__packed__))
{
   uint8_t device_id[7], clip_id, message_idx_and_final;
   uint8_t data[CELL_EVIDENCE_MAX_PAYLOAD_SIZE];
} evidence_message_t;

typedef struct __attribute__ ((__packed__))
{
   uint64_t device_id;
   uint32_t num_onsets;
   historical_onset_t onsets[HISTORICAL_ONSETS_MAX_RESULTS];
} historical_onset_message_t;

typedef enum
{
   MQTT_DEVICE_CONFIG_UPDATE = '1',
   MQTT_DEVICE_TEST_MODE = '2',
   MQTT_REQUEST_ONSETS = '4',
   MQTT_DEVICE_RESET = '8'
} mqtt_device_message_t;

typedef enum
{
   CELL_AUDIO_NO_TRANSMIT = 0,
   CELL_AUDIO_TRANSMIT_BEGIN,
   CELL_AUDIO_TRANSMIT_CONTINUE,
   CELL_AUDIO_TRANSMIT_END
} cell_audio_transmit_command_t;


// Shared Application Variables for Both Cores -------------------------------------------------------------------------

extern volatile data_packet_container_t data;


// Shared Application Variables for Core CM4 or CM7 --------------------------------------------------------------------

#ifdef CORE_CM4
extern volatile device_info_t device_info;
#endif

#endif  // #ifndef __COMMON_HEADER_H__
