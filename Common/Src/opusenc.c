#ifdef CORE_CM4

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "opus_config.h"
#include "opus.h"


// Opus Encoder Type Definitions ---------------------------------------------------------------------------------------

#define OPUS_FRAME_SIZE                      ((AUDIO_SAMPLE_RATE_HZ / 1000) * OPUS_MS_PER_FRAME)
#define OPUS_NUM_HISTORICAL_FRAMES           (OPUS_HISTORY_MS / OPUS_MS_PER_FRAME)


// Static Opus Encoding Variables --------------------------------------------------------------------------------------

__attribute__ ((aligned (4)))
static float encode_buffer[OPUS_FRAME_SIZE];

static opus_encoder_t opus_encoder;
static opus_frame_t opus_frames[OPUS_NUM_HISTORICAL_FRAMES], *opus_frame;

#ifdef TELEMETRY_ENABLE_METRICS
static volatile opusenc_telemetry_t telemetry;
#endif


// Public API Functions ------------------------------------------------------------------------------------------------

void opusenc_init(void)
{
   // Initialize a linked list of historical Opus frames
   for (uint32_t i = 0; i < OPUS_NUM_HISTORICAL_FRAMES; ++i)
   {
      opus_frames[i].frame_delimiter = OPUS_FRAME_DELIMITER;
      opus_frames[i].next = ((i + 1) == OPUS_NUM_HISTORICAL_FRAMES) ? &opus_frames[0] : &opus_frames[i+1];
   }
   opus_frame = &opus_frames[0];

   // Initialize an Opus encoder for the specified runtime configuration
   opus_encoder_create(&opus_encoder, OPUS_ENCODED_BIT_RATE, OPUS_FRAME_SIZE);

   // Reset the telemetry metrics data structure
#ifdef TELEMETRY_ENABLE_METRICS
   memset((void*)&telemetry, 0, sizeof(telemetry));
#endif
}

void opusenc_encode(const int16_t* restrict audio_in, const opus_frame_t** restrict result_begin, const opus_frame_t** restrict result_end)
{
   // Initialize the result start and end pointers
   static size_t encode_buffer_index = 0;
   *result_begin = *result_end = opus_frame;

   // Initialize the telemetry metrics
#ifdef TELEMETRY_ENABLE_METRICS
   const uint32_t cycle_start = READ_REG(DWT->CYCCNT);
   uint32_t frames_encoded = 0, encoded_bytes = 0;
#endif

   // Loop until all input samples have been consumed
   for (size_t i = 0; i < AUDIO_BUFFER_SAMPLES_PER_CHANNEL; )
   {
      // Copy enough data to fill the Opus encoding buffer
      const size_t samples_to_copy = MIN(OPUS_FRAME_SIZE - encode_buffer_index, AUDIO_BUFFER_SAMPLES_PER_CHANNEL - i);
      float *dst = &encode_buffer[encode_buffer_index];
      const int16_t *src = audio_in + i;
      size_t remaining = samples_to_copy;
      while (remaining >= 4)
      {
         dst[0] = src[0]; dst[1] = src[1]; dst[2] = src[2]; dst[3] = src[3];
         src += 4; dst += 4;
         remaining -= 4;
      }
      while (remaining--)
         *(dst++) = *(src++);
      encode_buffer_index = (encode_buffer_index + samples_to_copy) % OPUS_FRAME_SIZE;
      i += samples_to_copy;

      // If encoding buffer is full, encode the audio data frame
      if (!encode_buffer_index)
      {
         const uint8_t frame_bytes = (uint8_t)opus_encode(&opus_encoder, encode_buffer, opus_frame->encoded_data, sizeof(opus_frame->encoded_data));
         opus_frame->num_encoded_bytes = frame_bytes;
#ifdef TELEMETRY_ENABLE_METRICS
         encoded_bytes += frame_bytes;
         ++frames_encoded;
#endif
         *result_end = opus_frame = opus_frame->next;
      }
   }

   // Update the telemetry metrics
#ifdef TELEMETRY_ENABLE_METRICS
   const uint32_t elapsed_cycles = READ_REG(DWT->CYCCNT) - cycle_start;
   ++telemetry.packet_encode_calls;
   telemetry.packet_samples_last = AUDIO_BUFFER_SAMPLES_PER_CHANNEL;
   telemetry.packet_frames_last = frames_encoded;
   if (frames_encoded > telemetry.packet_frames_max)
      telemetry.packet_frames_max = frames_encoded;
   telemetry.encoded_bytes_total += encoded_bytes;
   telemetry.encode_cycles_last = elapsed_cycles;
   if (elapsed_cycles > telemetry.encode_cycles_max)
      telemetry.encode_cycles_max = elapsed_cycles;
   telemetry.encode_cycles_total += elapsed_cycles;
#endif
}

const opus_frame_t* opusenc_get_history(void)
{
   // Return the history frame pointer which currently points to the oldest frame
   //   Caller should iterate through history via opus_frame->next until reaching the original pointer
   return opus_frame;
}

void opusenc_get_telemetry(opusenc_telemetry_t *telemetry_out)
{
   // Retrieve current Opus encoding telemetry metrics
   if (telemetry_out)
   {
#ifdef TELEMETRY_ENABLE_METRICS
      memcpy(telemetry_out, (const void*)&telemetry, sizeof(telemetry));
#else
      memset(telemetry_out, 0, sizeof(*telemetry_out));
#endif
   }
}

void opusenc_reset_telemetry(void)
{
   // Reset all Opus encoding telemetry metrics
#ifdef TELEMETRY_ENABLE_METRICS
   memset((void*)&telemetry, 0, sizeof(telemetry));
#endif
}

#endif  // #ifdef CORE_CM4
