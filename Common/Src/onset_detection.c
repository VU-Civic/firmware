#ifdef CORE_CM7

#include <arm_math.h>
#include "common.h"
#include "onset_detection.h"

#define WINDOW_SIZE                256
#define STEP_SIZE                  (WINDOW_SIZE / 2)
#define NUM_WINDOWS                (1 + ((AUDIO_BUFFER_SAMPLES_PER_CHANNEL - WINDOW_SIZE) / STEP_SIZE))

#define arm_rfft_init              ARM_EXPAND(WINDOW_SIZE)
#define ARM_EXPAND(x)              ARM_STRINGIFY(x)
#define ARM_STRINGIFY(x)           arm_rfft_fast_init_ ## x ## _f32

static arm_rfft_fast_instance_f32 fft;
static float32_t hanning_window[WINDOW_SIZE];
static uint32_t onset_indices[MAX_NUM_ONSETS];

void onset_detection_init(void)
{
   // Initialize the FFT structure
   arm_rfft_init(&fft);
   memset(onset_indices, 0, sizeof(onset_indices));

   // Generate a static Hanning window
   for (size_t i = 0; i < WINDOW_SIZE; ++i)
      hanning_window[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / WINDOW_SIZE));
}

onset_details_t onset_detection_invoke(const int16_t (*audio_samples)[AUDIO_BUFFER_SAMPLES_PER_CHANNEL])
{
   // Set up all necessary FFT structures
   static float32_t input_signal[WINDOW_SIZE], windowed_signal[WINDOW_SIZE], fft_output[WINDOW_SIZE];
   static float32_t fft_magnitudes[WINDOW_SIZE / 2], prev_magnitudes[WINDOW_SIZE / 2] = { 0.0f };
   onset_details_t details = { .num_onsets = 0, .indices = onset_indices };

   // Iterate through all spectrogram windows
   for (uint32_t i = 0, window = 0; window < NUM_WINDOWS; i += STEP_SIZE, ++window)
   {
      // Convert the audio signal to a normalized floating point representation
      arm_q15_to_float(&audio_samples[0][i], input_signal, WINDOW_SIZE);

      // Apply a Hanning window to the input signal
      arm_mult_f32(input_signal, hanning_window, windowed_signal, WINDOW_SIZE);

      // Compute the FFT of the signal followed by its complex magnitude
      arm_rfft_fast_f32(&fft, windowed_signal, fft_output, 0);
      arm_cmplx_mag_f32(fft_output, fft_magnitudes, WINDOW_SIZE / 2);
      // TODO: SEE IF NORMALIZING fft_magnitudes HELPS (i.e.: fft_magnitudes / sum(fft_magnitudes))

      // Compute the positive-only spectral flux
      float32_t spectral_flux = 0.0f;
      for (uint32_t bin = 0; bin < (WINDOW_SIZE / 2); ++bin)
      {
         const float32_t difference = fft_magnitudes[bin] - prev_magnitudes[bin];
         if (difference > 0.0f)
            spectral_flux += difference;
      }

      // Update the previous magnitudes for the next iteration
      arm_copy_f32(fft_magnitudes, prev_magnitudes, WINDOW_SIZE / 2);

      // TODO: Search for acoustic event onsets (details.indices[details.num_onsets++] = i)
   }

   // TODO: Search for AoA for each detected onset
   /*for (uint32_t i = 0; i < details.num_onsets; ++i)
   {
      uint32_t onset = details.indices[i];
      for (uint32_t ch = 0; ch < AUDIO_NUM_CHANNELS; ++ch)
         audio_samples[ch][onset];
   }*/

   // Return details about detected onsets
   return details;
}

#endif  // #ifdef CORE_CM7
