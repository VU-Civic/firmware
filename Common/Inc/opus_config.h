#ifndef __OPUSENC_HEADER_H__
#define __OPUSENC_HEADER_H__

#include "common.h"

struct __attribute__ ((__packed__, aligned (4))) opus_frame_t;

typedef struct __attribute__ ((__packed__, aligned (4))) opus_frame_t
{
   uint8_t frame_delimiter, num_encoded_bytes;
   uint8_t encoded_data[2 * ((OPUS_ENCODED_BIT_RATE/8) / (1000/OPUS_MS_PER_FRAME))];
   struct opus_frame_t *next;
} opus_frame_t;

typedef struct
{
   uint32_t packet_encode_calls, packet_samples_last;
   uint32_t packet_frames_last, packet_frames_max;
   uint32_t encode_cycles_last, encode_cycles_max;
   uint64_t encode_cycles_total,encoded_bytes_total;
} opusenc_telemetry_t;

void opusenc_init(void);
void opusenc_encode(const int16_t* restrict audio_in, const opus_frame_t** restrict result_begin, const opus_frame_t** restrict result_end);
const opus_frame_t* opusenc_get_history(void);
void opusenc_get_telemetry(opusenc_telemetry_t *telemetry);
void opusenc_reset_telemetry(void);

#endif  // #ifndef __OPUSENC_HEADER_H__
