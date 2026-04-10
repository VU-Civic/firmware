#ifdef CORE_CM7

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include <arm_math.h>
#include <math.h>
#include "common.h"
#include "onset_detection.h"


// Onset Detector Definitions ------------------------------------------------------------------------------------------

#define AUDIO_PACKET_NUM_SAMPLES               AUDIO_BUFFER_SAMPLES_PER_CHANNEL

#define SPECTROGRAM_MIN_FREQUENCY_HZ           800
#define SPECTROGRAM_MAX_FREQUENCY_HZ           4300

#define FFT_FILTER_SIZE                        4096
#define FFT_STEP_SIZE                          1000
#define FFT_WINDOW_SIZE                        4000

#define FFT_MIN_BIN                            (SPECTROGRAM_MIN_FREQUENCY_HZ * FFT_FILTER_SIZE / AUDIO_SAMPLE_RATE_HZ)
#define FFT_MAX_BIN                            (2 + (SPECTROGRAM_MAX_FREQUENCY_HZ * FFT_FILTER_SIZE / AUDIO_SAMPLE_RATE_HZ))
#define NUM_FFT_BINS                           (FFT_MAX_BIN - FFT_MIN_BIN)

#define FFT_FINE_FILTER_SIZE                   256
#define FFT_FINE_STEP_SIZE                     64
#define FFT_FINE_WINDOW_SIZE                   256

#define FFT_FINE_MIN_BIN                       (SPECTROGRAM_MIN_FREQUENCY_HZ * FFT_FINE_FILTER_SIZE / AUDIO_SAMPLE_RATE_HZ)
#define FFT_FINE_MAX_BIN                       (2 + (SPECTROGRAM_MAX_FREQUENCY_HZ * FFT_FINE_FILTER_SIZE / AUDIO_SAMPLE_RATE_HZ))
#define NUM_FFT_FINE_BINS                      (FFT_FINE_MAX_BIN - FFT_FINE_MIN_BIN)

#define FLUX_MIN_THRESHOLD                     0.6f
#define FLUX_EMA_ALPHA                         0.01f
#define FLUX_THRESHOLD_NUM_SIGMAS              2.5f
#define FLUX_WARMUP_FRAMES                     50

#define BROADBAND_ACTIVATION_BIN_SIZE_HZ       500
#define BROADBAND_ACTIVATION_MIN_THRESHOLD     0.75f
#define BROADBAND_NOISE_EMA_ALPHA              0.01f
#define BROADBAND_NOISE_RISE_LIN               2.511886f    // 10^(8.0 dB / 20.0)

#define BROADBAND_BAND_BINS                    (BROADBAND_ACTIVATION_BIN_SIZE_HZ * FFT_FILTER_SIZE / AUDIO_SAMPLE_RATE_HZ)
#define BROADBAND_NOISE_BANDS                  (NUM_FFT_BINS / BROADBAND_BAND_BINS)

#define ENERGY_BAND_RATIO_LOW_HZ               400
#define ENERGY_BAND_RATIO_MID_HZ               1500
#define ENERGY_BAND_RATIO_HIGH_HZ              8000
#define ENERGY_BAND_RATIO_THRESHOLD            120.0

#define ENERGY_BAND_RATIO_LOW_BIN             (ENERGY_BAND_RATIO_LOW_HZ * FFT_FILTER_SIZE / AUDIO_SAMPLE_RATE_HZ)
#define ENERGY_BAND_RATIO_MID_BIN             (ENERGY_BAND_RATIO_MID_HZ * FFT_FILTER_SIZE / AUDIO_SAMPLE_RATE_HZ)
#define ENERGY_BAND_RATIO_HIGH_BIN            (1 + (ENERGY_BAND_RATIO_HIGH_HZ * FFT_FILTER_SIZE / AUDIO_SAMPLE_RATE_HZ))

#define MIN_TIME_BETWEEN_ONSETS_MS             200
#define MIN_SAMPLES_BETWEEN_ONSETS             (MIN_TIME_BETWEEN_ONSETS_MS * AUDIO_SAMPLE_RATE_HZ / 1000)

#define NUM_TIME_STEPS_PER_PACKET              (AUDIO_PACKET_NUM_SAMPLES / FFT_STEP_SIZE)
#define AVAILABLE_WINDOWS_PER_PACKET           (1 + (AUDIO_PACKET_NUM_SAMPLES - FFT_WINDOW_SIZE) / FFT_STEP_SIZE)
#define MISSING_WINDOWS_PER_PACKET             (NUM_TIME_STEPS_PER_PACKET - AVAILABLE_WINDOWS_PER_PACKET)

#define arm_rfft_init                          ARM_EXPAND(FFT_FILTER_SIZE)
#define arm_rfft_fine_init                     ARM_EXPAND(FFT_FINE_FILTER_SIZE)
#define ARM_EXPAND(x)                          ARM_STRINGIFY(x)
#define ARM_STRINGIFY(x)                       arm_rfft_fast_init_ ## x ## _f32


// Static Onset Detection Variables ------------------------------------------------------------------------------------

static arm_rfft_fast_instance_f32 fft_coarse, fft_fine;
static const float mic_offsets[AUDIO_NUM_CHANNELS-1][2] = { MIC_CH1_CH2_OFFSET, MIC_CH1_CH3_OFFSET, MIC_CH1_CH4_OFFSET };
static float fft_buf[FFT_FILTER_SIZE], hanning_window[FFT_WINDOW_SIZE], hanning_window_fine[FFT_FINE_WINDOW_SIZE];
static float broadband_noise_floor[BROADBAND_NOISE_BANDS], magnitudes[FFT_FILTER_SIZE/2], prev_magnitudes[FFT_FILTER_SIZE/2];
static float magnitudes_fine[NUM_FFT_FINE_BINS], prev_magnitudes_fine[NUM_FFT_FINE_BINS];
static float flux_ema_mean, flux_ema_var, prev_flux;
static int16_t pending_buf[FFT_WINDOW_SIZE];
static int32_t samples_since_prev_onset;
static uint32_t flux_warmup_count;


// Private Helper Functions --------------------------------------------------------------------------------------------

static void least_squares_3x2(float *output, const float *observations)
{
   // Statically compute A'A
   static const float m00 = mic_offsets[0][0]*mic_offsets[0][0] + mic_offsets[1][0]*mic_offsets[1][0] + mic_offsets[2][0]*mic_offsets[2][0];
   static const float m01 = mic_offsets[0][0]*mic_offsets[0][1] + mic_offsets[1][0]*mic_offsets[1][1] + mic_offsets[2][0]*mic_offsets[2][1];
   static const float m11 = mic_offsets[0][1]*mic_offsets[0][1] + mic_offsets[1][1]*mic_offsets[1][1] + mic_offsets[2][1]*mic_offsets[2][1];

   // Compute A'b
   const float v0 = mic_offsets[0][0]*observations[0] + mic_offsets[1][0]*observations[1] + mic_offsets[2][0]*observations[2];
   const float v1 = mic_offsets[0][1]*observations[0] + mic_offsets[1][1]*observations[1] + mic_offsets[2][1]*observations[2];

   // Use Cramer's rule to solve M*x = v
   const float det = m00*m11 - m01*m01;
   const float eps = 1.0e-10f * (m00 + m11);
   if ((det > -eps) && (det < eps))
      output[0] = output[1] = 0.0f;
   else
   {
      const float inv_det = 1.0f / det;
      output[0] = (v0*m11 - v1*m01) * inv_det;
      output[1] = (m00*v1 - m01*v0) * inv_det;
   }
}

static float compute_flux_coarse(const int16_t *audio)
{
   // Convert Q15 audio to float in a zero-padded FFT buffer
   arm_q15_to_float(audio, fft_buf, FFT_WINDOW_SIZE);
   arm_fill_f32(0.0f, fft_buf + FFT_WINDOW_SIZE, FFT_FILTER_SIZE - FFT_WINDOW_SIZE);

   // Apply Hanning window to the unpadded signal region and compute in-place forward real FFT
   arm_mult_f32(fft_buf, hanning_window, fft_buf, FFT_WINDOW_SIZE);
   arm_rfft_fast_f32(&fft_coarse, fft_buf, fft_buf, 0);

   // Compute the power spectrum (magnitude squared) for all frequency bins
   arm_cmplx_mag_squared_f32(fft_buf, magnitudes, FFT_FILTER_SIZE / 2);

   // Positive spectral flux: sum of per-bin energy increases only
   float flux = 0.0f;
   for (int i = FFT_MIN_BIN; i <= FFT_MAX_BIN; ++i)
   {
      const float delta = magnitudes[i] - prev_magnitudes[i];
      if (delta > 0.0f)
         flux += delta;
   }

   // Store the magnitude spectrum for the next iteration
   arm_copy_f32(magnitudes, prev_magnitudes, FFT_FILTER_SIZE / 2);
   return flux;
}

static float compute_flux_fine(const int16_t *audio)
{
   // Convert Q15 audio to float, apply Hanning window, and compute in-place forward real FFT
   arm_q15_to_float(audio, fft_buf, FFT_FINE_WINDOW_SIZE);
   arm_mult_f32(fft_buf, hanning_window_fine, fft_buf, FFT_FINE_WINDOW_SIZE);
   arm_rfft_fast_f32(&fft_fine, fft_buf, fft_buf, 0);

   // Power spectrum (magnitude squared) for bins of interest
   arm_cmplx_mag_squared_f32(fft_buf + (2 * FFT_FINE_MIN_BIN), magnitudes_fine, NUM_FFT_FINE_BINS);

   // Positive spectral flux: sum of per-bin energy increases only
   float flux = 0.0f;
   for (int i = 0; i < NUM_FFT_FINE_BINS; i++)
   {
      const float delta = magnitudes_fine[i] - prev_magnitudes_fine[i];
      if (delta > 0.0f)
         flux += delta;
   }

   // Store the fine magnitude spectrum for the next iteration
   arm_copy_f32(magnitudes_fine, prev_magnitudes_fine, NUM_FFT_FINE_BINS);
   return flux;
}

static float update_flux_statistics(float flux)
{
   // Welford online mean/variance during warmup: accumulate simple running averages
   if (flux_warmup_count < FLUX_WARMUP_FRAMES)
   {
      const float n = ++flux_warmup_count;
      const float mean_new = flux_ema_mean + ((flux - flux_ema_mean) / n);
      if (flux_warmup_count > 1)
         flux_ema_var += (((flux - mean_new) * (flux - mean_new)) - flux_ema_var) / n;
      flux_ema_mean = mean_new;
      return FLUX_MIN_THRESHOLD;
   }

   // Exponential moving average update post-warmup
   const float delta = flux - flux_ema_mean;
   flux_ema_mean += FLUX_EMA_ALPHA * delta;
   flux_ema_var = (1.0f - FLUX_EMA_ALPHA) * (flux_ema_var + (FLUX_EMA_ALPHA * delta * delta));

   // Adaptive threshold: mean + (k * stddev), floored at the absolute minimum
   const float stddev = sqrtf(flux_ema_var > 0.0f ? flux_ema_var : 0.0f);
   const float threshold = flux_ema_mean + (FLUX_THRESHOLD_NUM_SIGMAS * stddev);
   return (threshold > FLUX_MIN_THRESHOLD) ? threshold : FLUX_MIN_THRESHOLD;
}

static float broadband_activation(void)
{
   float active_bands = 0.0f;
   for (int b = 0; b < BROADBAND_NOISE_BANDS; ++b)
   {
      // Mean energy across this frequency band
      float band_energy;
      const int lo = FFT_MIN_BIN + (b * BROADBAND_BAND_BINS);
      arm_mean_f32(magnitudes + lo, BROADBAND_BAND_BINS, &band_energy);

      // Slow EMA noise floor: initialise from first non-zero frame
      if (broadband_noise_floor[b] < 1e-12f)
         broadband_noise_floor[b] = band_energy;
      else
         broadband_noise_floor[b] += BROADBAND_NOISE_EMA_ALPHA * (band_energy - broadband_noise_floor[b]);

      // Band is active if energy exceeds noise floor by the required margin
      if (band_energy > (broadband_noise_floor[b] * BROADBAND_NOISE_RISE_LIN))
         active_bands += 1.0f;
   }
   return active_bands / BROADBAND_NOISE_BANDS;
}

static float compute_energy_band_ratio(void)
{
   // Compute the ratio of total energy in the high band to the low band
   float total_energy_high, total_energy_low;
   arm_accumulate_f32(magnitudes + ENERGY_BAND_RATIO_LOW_BIN, ENERGY_BAND_RATIO_MID_BIN - ENERGY_BAND_RATIO_LOW_BIN + 1, &total_energy_low);
   arm_accumulate_f32(magnitudes + ENERGY_BAND_RATIO_MID_BIN, ENERGY_BAND_RATIO_HIGH_BIN - ENERGY_BAND_RATIO_MID_BIN, &total_energy_high);
   return total_energy_high / (total_energy_low + 1e-12);
}

static int check_onset_conditions(float flux, float adaptive_threshold)
{
   // Gate 1: Enough frequency bands must be activated above their noise floor
   if (broadband_activation() < BROADBAND_ACTIVATION_MIN_THRESHOLD)
      return 0;

   // Gate 2: Flux must exceed the adaptive threshold and be at a local peak
   if ((flux <= adaptive_threshold) || (flux < prev_flux))
      return 0;

   // Gate 3: Energy distribution must have enough low-frequency content
   if (compute_energy_band_ratio() > ENERGY_BAND_RATIO_THRESHOLD)
      return 0;

   // Gate 4: Enforce minimum refractory period between onsets
   if (samples_since_prev_onset < MIN_SAMPLES_BETWEEN_ONSETS)
      return 0;
   return 1;
}

static double onset_detector_refine(const int16_t *window)
{
   // Scan fine-grained FFT windows within the coarse detection frame to pinpoint onset
   int32_t onset_offset = 0;
   float threshold = 2.0f * compute_flux_fine(window);
   for (int32_t off = FFT_FINE_STEP_SIZE; off <= (FFT_WINDOW_SIZE - FFT_FINE_WINDOW_SIZE); off += FFT_FINE_STEP_SIZE)
   {
      const float flux = compute_flux_fine(window + off);
      if (flux > threshold)
      {
         threshold = 2.0f * flux;
         onset_offset = off;
      }
   }
   samples_since_prev_onset = -onset_offset;
   return (double)onset_offset / AUDIO_SAMPLE_RATE_HZ;
}

static double process_time_step(const int16_t *window, double window_timestamp)
{
   // Calculate spectral flux on the current window and update the adaptive threshold
   const float flux = compute_flux_coarse(window);
   const float adaptive_threshold = update_flux_statistics(flux);
   samples_since_prev_onset += FFT_STEP_SIZE;

   // Check if this time step meets all conditions for an onset
   double onset_timestamp = 0.0;
   if (check_onset_conditions(flux, adaptive_threshold))
      onset_timestamp = window_timestamp + onset_detector_refine(window);
   prev_flux = flux;
   return onset_timestamp;
}


// Public API Functions ------------------------------------------------------------------------------------------------

void onset_detection_init(void)
{
   // Initialize both FFT instances and pre-compute Hanning windows
   arm_rfft_init(&fft_coarse);
   arm_rfft_fine_init(&fft_fine);
   arm_hanning_f32(hanning_window, FFT_WINDOW_SIZE);
   arm_hanning_f32(hanning_window_fine, FFT_FINE_WINDOW_SIZE);

   // Zero all detection state
   arm_fill_f32(0.0f, magnitudes, FFT_FILTER_SIZE / 2);
   arm_fill_f32(0.0f, prev_magnitudes, FFT_FILTER_SIZE / 2);
   arm_fill_f32(0.0f, magnitudes_fine, NUM_FFT_FINE_BINS);
   arm_fill_f32(0.0f, prev_magnitudes_fine, NUM_FFT_FINE_BINS);
   arm_fill_q15(0, pending_buf, FFT_WINDOW_SIZE);
   arm_fill_f32(0.0f, broadband_noise_floor, BROADBAND_NOISE_BANDS);
   flux_ema_mean = flux_ema_var = prev_flux = 0.0f;
   samples_since_prev_onset = 10000000;
   flux_warmup_count = 0;
}

double onset_detection_invoke(double packet_timestamp, const int16_t (*audio_samples)[AUDIO_BUFFER_SAMPLES_PER_CHANNEL], channel_alarms_t channel_alarms)
{
   // Use the first audio channel that is not in an alarm state
   double onset_timestamp = 0.0;
   const int16_t *audio_packet = !channel_alarms.alarm.ch1 ? audio_samples[0] : (!channel_alarms.alarm.ch2 ? audio_samples[1] : (!channel_alarms.alarm.ch3 ? audio_samples[2] : audio_samples[3]));

   // Process the audio windows that straddle the previous and current packets
   for (int m = 0; m < MISSING_WINDOWS_PER_PACKET; ++m)
   {
      // Append the next step of the current packet into the pending buffer's tail slot
      arm_copy_q15(audio_packet + (m * FFT_STEP_SIZE), pending_buf + FFT_WINDOW_SIZE - FFT_STEP_SIZE, FFT_STEP_SIZE);

      // Process the pending buffer as a complete window and save any detected onsets as timestamps
      const double onset_time = process_time_step(pending_buf, packet_timestamp - ((AUDIO_PACKET_NUM_SAMPLES - ((AVAILABLE_WINDOWS_PER_PACKET + m) * FFT_STEP_SIZE)) / (double)AUDIO_SAMPLE_RATE_HZ));
      if (onset_time > 0.0)
         onset_timestamp = onset_time;

      // Slide the pending buffer forward by one step for the next missing window
      memmove(pending_buf, pending_buf + FFT_STEP_SIZE, (FFT_WINDOW_SIZE - FFT_STEP_SIZE) * sizeof(pending_buf[0]));
   }

   // Process the audio windows fully contained within the current packet
   for (int t = 0; t < AVAILABLE_WINDOWS_PER_PACKET; ++t)
   {
      const double onset_time = process_time_step(audio_packet + (t * FFT_STEP_SIZE), packet_timestamp + ((t * FFT_STEP_SIZE) / (double)AUDIO_SAMPLE_RATE_HZ));
      if (onset_time > 0.0)
         onset_timestamp = onset_time;
   }

   // Save the tail of the current packet for next invocation's missing windows
   arm_copy_q15(audio_packet + (AVAILABLE_WINDOWS_PER_PACKET * FFT_STEP_SIZE), pending_buf, AUDIO_PACKET_NUM_SAMPLES - (AVAILABLE_WINDOWS_PER_PACKET * FFT_STEP_SIZE));

   // TODO: Calculate the angle of arrival for each detected onset (unless a channel is in an alarm state)
   /*if (!channel_alarms.alarms)
      for (uint32_t i = 0; i < onsets.num_onsets; ++i)
      {
         const int32_t onset = onsets.indices[i];
         // TODO: Cross-correlation to get all intra-channel offsets
         float observations[AUDIO_NUM_CHANNELS-1] = { -343.0f * (onsets[1] - onsets[0]), -343.0f * (onsets[2] - onsets[0]), -343.0f * (onsets[3] - onsets[0]) };
         float angle_of_arrival[3] = { 0 };
         least_squares_3x2(angle_of_arrival, observations);
         angle_of_arrival[2] = angle_of_arrival[1];
         angle_of_arrival[1] = sqrtf(1.0 - (angle_of_arrival[0]*angle_of_arrival[0] + angle_of_arrival[2]*angle_of_arrival[2]));

      }*/
   return onset_timestamp;
}

#endif  // #ifdef CORE_CM7
