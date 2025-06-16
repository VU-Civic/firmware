#ifdef CORE_CM7

#include <arm_math.h>
#include "common.h"
#include "onset_detection.h"

#define WINDOW_SIZE                256
#define STEP_SIZE                  (WINDOW_SIZE / 2)
#define NUM_WINDOWS                (1 + ((AUDIO_SAMPLE_RATE_HZ - WINDOW_SIZE) / STEP_SIZE))

//static arm_rfft_instance_q15 fft;
//static q15_t hanning_window[WINDOW_SIZE];
//static uint32_t onset_indices[MAX_NUM_ONSETS];

void onset_detection_init(void)
{
   // Initialize the FFT structure
   /*arm_rfft_init_q15(&fft, WINDOW_SIZE, 0, 1);
   memset(onset_indices, 0, sizeof(onset_indices));

   // Generate a static Hanning window
   for (size_t i = 0; i < WINDOW_SIZE; ++i)
   {
      const float val = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / WINDOW_SIZE));
      arm_float_to_q15(&val, &hanning_window[i], 1);
   }*/
}

onset_details_t onset_detection_invoke(const int16_t (*audio_samples)[AUDIO_BUFFER_SAMPLES / AUDIO_NUM_CHANNELS])
{
   static int16_t values[4][100];
   for (int i = 0; i < 4; ++i)
      memcpy(values[i], audio_samples[i], sizeof(values[i]));
   // TODO: audio_samples should be 1 channel of audio (probably need mem-to-mem DMA transfer for this
/*
   // Set up all necessary FFT structures
   static q15_t windowed_signal[WINDOW_SIZE];
   static q15_t fft_output[WINDOW_SIZE * 2], fft_magnitudes[WINDOW_SIZE / 2];
   onset_details_t details = { .num_onsets = 0, .indices = onset_indices };

   // Iterate through all spectrogram windows
   for (uint32_t i = 0, window = 0; window < NUM_WINDOWS; i += STEP_SIZE, ++window)
   {
      // Apply a Hanning window to the audio signal
      arm_mult_q15(audio_samples + i, hanning_window, windowed_signal, WINDOW_SIZE);

      // Compute the FFT of the signal followed by its complex magnitude
      arm_rfft_q15(&fft, windowed_signal, fft_output);
      arm_cmplx_mag_q15(fft_output, fft_magnitudes, WINDOW_SIZE / 2);
      // TODO: SEE IF NORMALIZING fft_magnitudes HELPS (i.e.: fft_magnitudes / sum(fft_magnitudes)) <- HOW WOULD THIS WORK IN Q15

      // TODO: Compute spectral flux: sum((mags - mags_prev) .^ 2)

      // TODO: Search for acoustic event onsets
   }

   // Return details about detected onsets
   return details;*/

   /*
   Will be of format:
   for (uint32_t ch = 0; ch < AUDIO_NUM_CHANNELS; ++ch)
      for (uint32_t sample = 0; sample < (AUDIO_BUFFER_SAMPLES / AUDIO_NUM_CHANNELS); ++sample)
         process(audio_samples[ch][sample]);
    */
   return (onset_details_t){ .num_onsets = 0, .indices = NULL };
}

#endif  // #ifdef CORE_CM7
