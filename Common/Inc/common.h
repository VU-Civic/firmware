#ifndef __COMMON_HEADER_H__
#define __COMMON_HEADER_H__

// Common Header Inclusions --------------------------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "revisions.h"


// Custom Application Definitions --------------------------------------------------------------------------------------

#define AUDIO_NUM_CHANNELS                   4
#define AUDIO_SAMPLE_RATE_HZ                 96000
#define AUDIO_BUFFER_SAMPLES                 32000
#define AUDIO_DIGITAL_GAIN                   0xFF  // 0xC9 = 0dB, 0xFF = 27dB, 0.5dB increase per value  // TODO: Fine-tune this

#define CELL_CONNECTION_TIMEOUT_SECONDS      65520
#define CELL_SERVER_RESPONSE_TIMEOUT_SECONDS 5
#define CELL_MQTT_BROKER_APN                 tsudp
#define CELL_MQTT_BROKER_IP                  10.7.0.55
#define CELL_MQTT_BROKER_PORT                2442
#define CELL_MQTT_ALERT_TOPIC                111
#define CELL_MQTT_EVIDENCE_TOPIC             234
#define CELL_MQTT_DEVICES_TOPIC              35543
#define CELL_MQTT_CONTROL_TOPIC              24553

#define USB_VID                              4617
#define USB_PID                              2829
#define USB_PRODUCT_STRING                   "CivicAlert"
#define USB_MANUFACTURER_STRING              "HedgeTech"

#define MIN_MS_BETWEEN_ONSETS                20
#define MAX_NUM_ONSETS                       (2 + (1000 / MIN_MS_BETWEEN_ONSETS))

#define MAX_NUM_EVENTS_PER_ALERT             10

#ifdef PACKET_FULL_AUDIO
#define PACKET_AUDIO_SAMPLES                 (AUDIO_BUFFER_SAMPLES)
#else
#define PACKET_AUDIO_SAMPLES                 (AUDIO_BUFFER_SAMPLES / AUDIO_NUM_CHANNELS)
#endif

#define PACKET_END_DELIMITER                 { 0xFE, 0xF0, 0xF2, 0x25 }
#define PACKET_RESPONSE_DELIMITER            { 0xFE, 0xF9 }
#define PACKET_RESPONSE_ACK                  { 0x01, 0x02 }


// Global Type Definitions ---------------------------------------------------------------------------------------------

typedef struct __attribute__ ((__packed__, aligned(4)))
{
   int16_t audio[2][PACKET_AUDIO_SAMPLES];
   double timestamp;
   float lat, lon, ht;
   int32_t q1, q2, q3;
   uint8_t delimiter[4];
   int32_t audio_read_index, audio_clip_complete;
} data_packet_t;

typedef struct __attribute__ ((__packed__, aligned(4)))
{
   double timestamp;
   float lat, lon, ht;
   int32_t q1, q2, q3;
   uint8_t chip_temperature_alert;
   uint8_t signal_power, signal_quality;
} device_info_t;

typedef struct __attribute__ ((__packed__, aligned(4)))
{
   double timestamps[MAX_NUM_EVENTS_PER_ALERT];
   float confidences[MAX_NUM_EVENTS_PER_ALERT];
   float lat, lon, ht;
   int32_t q1, q2, q3, event_id;
   uint8_t event_class, num_events;
   uint8_t chip_temperature_alert;
   uint8_t signal_power, signal_quality;
} event_message_t;

typedef struct __attribute__ ((__packed__, aligned(32)))
{
   uint8_t mqtt_device_info_qos, mqtt_alert_qos, mqtt_audio_qos;
} non_volatile_data_t;


// Shared Application Variables for Both Cores -------------------------------------------------------------------------

extern non_volatile_data_t non_volatile_data;
extern volatile data_packet_t data;


// Shared Application Variables for Core CM4 ---------------------------------------------------------------------------

#ifdef CORE_CM4
extern volatile device_info_t device_info;
#endif

#endif  // #ifndef __COMMON_HEADER_H__
