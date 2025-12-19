#ifndef __COMMON_HEADER_H__
#define __COMMON_HEADER_H__

// Common Header Inclusions --------------------------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"


// Custom Application Definitions --------------------------------------------------------------------------------------

#define FIRMWARE_BUILD_TIMESTAMP             _DATETIME

#define AUDIO_NUM_CHANNELS                   4
#define AUDIO_SAMPLE_RATE_HZ                 48000
#define AUDIO_BUFFER_SAMPLES                 32000
#define AUDIO_DIGITAL_GAIN                   0xD1  // 0xC9 = 0dB, 0xFF = 27dB, 0.5dB increase per value
#define AUDIO_BUFFER_SAMPLES_PER_CHANNEL     (AUDIO_BUFFER_SAMPLES / AUDIO_NUM_CHANNELS)
#define AUDIO_NUM_DMAS_PER_CLIP              (AUDIO_SAMPLE_RATE_HZ * AUDIO_NUM_CHANNELS / AUDIO_BUFFER_SAMPLES)

#define CELL_CONNECTION_TIMEOUT_SECONDS      65520
#define CELL_SERVER_RESPONSE_TIMEOUT_SECONDS 5
#define CELL_IMEI_LENGTH                     15
#define CELL_MQTT_BROKER_APN                 tsudp
#define CELL_MQTT_BROKER_IP                  10.7.0.55
#define CELL_MQTT_BROKER_PORT                2442
#define CELL_MQTT_ALERT_TOPIC                111
#define CELL_MQTT_EVIDENCE_TOPIC             234
#define CELL_MQTT_DEVICES_TOPIC              35543
#define CELL_MQTT_CONTROL_TOPIC              24553
#define CELL_MQTT_MAX_PAYLOAD_SIZE_BYTES     1016
#define CELL_EVIDENCE_MAX_PAYLOAD_SIZE       (CELL_MQTT_MAX_PAYLOAD_SIZE_BYTES - 2)

#define USB_VID                              4617
#define USB_PID                              2829
#define USB_PRODUCT_STRING                   "CivicAlert"
#define USB_MANUFACTURER_STRING              "HedgeTech"

#define OPUS_FRAME_DELIMITER                 0xAAAA
#define OPUS_INPUT_AUDIO_NUM_CHANNELS        1
#define OPUS_APPLICATION_TYPE                OPUS_APPLICATION_VOIP // TODO: TEST CHANGING THIS TO _AUDIO
#define OPUS_SIGNAL_TYPE                     OPUS_AUTO // TODO: SEE IF OPUS_SIGNAL_VOICE OR OPUS_SIGNAL_MUSIC USES LESS CPU WITHOUT AFFECTING SOUND
#define OPUS_ENCODED_BIT_RATE                16000
#define OPUS_COMPLEXITY                      0
#define OPUS_MS_PER_FRAME                    60
#define OPUS_HISTORY_MS                      960

#define MIN_MS_BETWEEN_ONSETS                20
#define MAX_NUM_ONSETS                       (2 + (1000 / MIN_MS_BETWEEN_ONSETS))

#define AI_NUM_CLASSES                       2

#define MAX_NUM_EVENTS_PER_ALERT             AUDIO_NUM_DMAS_PER_CLIP

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

#define DMA_STREAM0_4_INDEX                  0U
#define DMA_STREAM1_5_INDEX                  6U
#define DMA_STREAM2_6_INDEX                  16U
#define DMA_STREAM3_7_INDEX                  22U


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

typedef struct __attribute__ ((__packed__, aligned (32)))
{
   uint8_t mqtt_device_info_qos, mqtt_alert_qos, mqtt_audio_qos;
} non_volatile_data_t;

typedef struct __attribute__ ((__packed__, aligned (4)))
{
   uint8_t start_delimiter[4];
   int16_t audio[PACKET_AUDIO_SAMPLES];
   double timestamp;
   float lat, lon, ht;
   int32_t q1, q2, q3;
   uint8_t end_delimiter[4];
} data_packet_t;

typedef struct __attribute__ ((__packed__, aligned (4)))
{
   data_packet_t packets[2];
   int32_t audio_read_index, audio_clip_complete;
} data_packet_container_t;

typedef struct __attribute__ ((__packed__, aligned (4)))
{
   double timestamp;
   float lat, lon, ht;
   int32_t q1, q2, q3;
   uint32_t firmware_date;
   uint8_t chip_temperature_alert;
   uint8_t signal_power, signal_quality;
} device_info_t;

typedef struct __attribute__ ((__packed__))
{
   uint32_t ai_firmware_version;
   uint8_t class_probabilities[AI_NUM_CLASSES];
} ai_result_t;

typedef struct __attribute__ ((__packed__))
{
   uint32_t id, class;
   double timestamp;
   float confidence, angle_of_arrival[3];
} event_info_t;

typedef struct __attribute__ ((__packed__, aligned (4)))
{
   uint8_t num_events, cell_signal_power;
   uint8_t cell_signal_quality, sensor_temperature_alert;
   int32_t sensor_q1, sensor_q2, sensor_q3;
   float sensor_lat, sensor_lon, sensor_ht;
   char device_id[CELL_IMEI_LENGTH+1];
   event_info_t events[MAX_NUM_EVENTS_PER_ALERT];
} alert_message_t;

typedef struct __attribute__ ((__packed__, aligned (4)))
{
   uint8_t message_idx, is_final_message;
   uint8_t data[CELL_EVIDENCE_MAX_PAYLOAD_SIZE];
} evidence_message_t;

typedef enum
{
   CELL_AUDIO_NO_TRANSMIT = 0,
   CELL_AUDIO_TRANSMIT_BEGIN,
   CELL_AUDIO_TRANSMIT_CONTINUE,
   CELL_AUDIO_TRANSMIT_END
} cell_audio_transmit_command_t;


// Shared Application Variables for Both Cores -------------------------------------------------------------------------

extern non_volatile_data_t non_volatile_data;
extern volatile data_packet_container_t data;


// Shared Application Variables for Core CM4 ---------------------------------------------------------------------------

#ifdef CORE_CM4
extern volatile device_info_t device_info;
#endif

#endif  // #ifndef __COMMON_HEADER_H__
