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

#define ENERGY_BAND_RATIO_LOW_BIN              (ENERGY_BAND_RATIO_LOW_HZ * FFT_FILTER_SIZE / AUDIO_SAMPLE_RATE_HZ)
#define ENERGY_BAND_RATIO_MID_BIN              (ENERGY_BAND_RATIO_MID_HZ * FFT_FILTER_SIZE / AUDIO_SAMPLE_RATE_HZ)
#define ENERGY_BAND_RATIO_HIGH_BIN             (1 + (ENERGY_BAND_RATIO_HIGH_HZ * FFT_FILTER_SIZE / AUDIO_SAMPLE_RATE_HZ))

#define MIN_TIME_BETWEEN_ONSETS_MS             200
#define MIN_SAMPLES_BETWEEN_ONSETS             (MIN_TIME_BETWEEN_ONSETS_MS * AUDIO_SAMPLE_RATE_HZ / 1000)

#define XCORR_WINDOW_SIZE                      256
#define XCORR_FFT_SIZE                         512
#define XCORR_MAX_DELAY_SAMP                   9
#define XCORR_PHAT_MIN_BIN                     (SPECTROGRAM_MIN_FREQUENCY_HZ * XCORR_FFT_SIZE / AUDIO_SAMPLE_RATE_HZ)
#define XCORR_PHAT_MAX_BIN                     (SPECTROGRAM_MAX_FREQUENCY_HZ * XCORR_FFT_SIZE / AUDIO_SAMPLE_RATE_HZ)

#define REFINE_SEARCH_BEFORE_MS                100
#define REFINE_SEARCH_AFTER_MS                 50
#define REFINE_NOISE_WIN_MS                    60
#define REFINE_NOISE_GAP_MS                    10
#define REFINE_STE_WIN_MS                      3
#define REFINE_STE_HOP_MS                      1
#define REFINE_PEAK_FRACTION                   0.25f
#define REFINE_NOISE_MULT                      9.0f

#define REFINE_NOISE_WIN_S                     (REFINE_NOISE_WIN_MS * AUDIO_SAMPLE_RATE_HZ / 1000)
#define REFINE_NOISE_GAP_S                     (REFINE_NOISE_GAP_MS * AUDIO_SAMPLE_RATE_HZ / 1000)
#define REFINE_SB_S                            (REFINE_SEARCH_BEFORE_MS * AUDIO_SAMPLE_RATE_HZ / 1000)
#define REFINE_SA_S                            (REFINE_SEARCH_AFTER_MS * AUDIO_SAMPLE_RATE_HZ / 1000)
#define REFINE_STE_WIN_S                       (REFINE_STE_WIN_MS * AUDIO_SAMPLE_RATE_HZ / 1000)
#define REFINE_STE_HOP_S                       (REFINE_STE_HOP_MS * AUDIO_SAMPLE_RATE_HZ / 1000)
#define REFINE_ONSET_IN_CTX                    (REFINE_NOISE_WIN_S + REFINE_NOISE_GAP_S + REFINE_SB_S)
#define REFINE_CTX_LEN                         (REFINE_ONSET_IN_CTX + REFINE_SA_S)

#define NUM_TIME_STEPS_PER_PACKET              (AUDIO_PACKET_NUM_SAMPLES / FFT_STEP_SIZE)
#define AVAILABLE_WINDOWS_PER_PACKET           (1 + (AUDIO_PACKET_NUM_SAMPLES - FFT_WINDOW_SIZE) / FFT_STEP_SIZE)
#define MISSING_WINDOWS_PER_PACKET             (NUM_TIME_STEPS_PER_PACKET - AVAILABLE_WINDOWS_PER_PACKET)

#define arm_rfft_init                          ARM_EXPAND(FFT_FILTER_SIZE)
#define arm_rfft_fine_init                     ARM_EXPAND(FFT_FINE_FILTER_SIZE)
#define arm_rfft_xcorr_init                    ARM_EXPAND(XCORR_FFT_SIZE)
#define ARM_EXPAND(x)                          ARM_STRINGIFY(x)
#define ARM_STRINGIFY(x)                       arm_rfft_fast_init_ ## x ## _f32


// Static Onset Detection Variables ------------------------------------------------------------------------------------

static double previous_timestamp;
static int64_t previous_raw_onset_sample, previous_start_sample;
static arm_rfft_fast_instance_f32 fft_coarse, fft_fine, fft_xcorr;
static int16_t previous_audio[AUDIO_NUM_CHANNELS][AUDIO_PACKET_NUM_SAMPLES], refinement_buf[REFINE_CTX_LEN];
static const float mic_offsets[AUDIO_NUM_CHANNELS-1][2] = { MIC_CH1_CH2_OFFSET, MIC_CH1_CH3_OFFSET, MIC_CH1_CH4_OFFSET };
static float fft_buf[FFT_FILTER_SIZE], hanning_window[FFT_WINDOW_SIZE], hanning_window_fine[FFT_FINE_WINDOW_SIZE];
static float broadband_noise_floor[BROADBAND_NOISE_BANDS], magnitudes[FFT_FILTER_SIZE/2], prev_magnitudes[FFT_FILTER_SIZE/2];
static float magnitudes_fine[NUM_FFT_FINE_BINS], prev_magnitudes_fine[NUM_FFT_FINE_BINS], gcc_ch0_fft[XCORR_FFT_SIZE];
static float flux_ema_mean, flux_ema_var, prev_flux;
static int32_t samples_since_prev_onset;
static uint32_t flux_warmup_count;

#ifdef TELEMETRY_ENABLE_METRICS
static volatile uint32_t onset_invoke_count, onset_detect_count;
static volatile float onset_ms_last, onset_ms_max;
#endif


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

static void build_refinement_buffer(int16_t *buff, int64_t onset_timestamp, int64_t current_start, const int16_t (*audio_samples)[AUDIO_BUFFER_SAMPLES_PER_CHANNEL], int selected_ch)
{
   // Compute the window boundaries of the previous and current packets relative to the detected onset
   const int32_t ctx_start_rel = (int32_t)(onset_timestamp - current_start) - REFINE_ONSET_IN_CTX, previous_offset = (int32_t)(previous_start_sample - current_start);
   const int32_t current_lo = (ctx_start_rel < 0) ? -ctx_start_rel : 0, current_hi_u = AUDIO_PACKET_NUM_SAMPLES - ctx_start_rel;
   const int32_t current_hi = (current_hi_u < REFINE_CTX_LEN) ? current_hi_u : REFINE_CTX_LEN;
   const int32_t previous_lo_u = previous_offset - ctx_start_rel, previous_hi_u = previous_offset + AUDIO_PACKET_NUM_SAMPLES - ctx_start_rel;
   const int32_t previous_lo = (previous_lo_u > 0) ? previous_lo_u : 0, previous_hi = (previous_hi_u < REFINE_CTX_LEN) ? previous_hi_u : REFINE_CTX_LEN;

   // Generate a zero-padded contiguous refinement buffer by copying from previous_audio and audio_samples as needed (buff[REFINE_ONSET_IN_CTX] will be the onset sample)
   arm_fill_q15(0, buff, REFINE_CTX_LEN);
   if (previous_lo < previous_hi)
      arm_copy_q15(previous_audio[selected_ch] + (previous_lo + ctx_start_rel - previous_offset), buff + previous_lo, (uint32_t)(previous_hi - previous_lo));
   if (current_lo < current_hi)
      arm_copy_q15(audio_samples[selected_ch] + (current_lo + ctx_start_rel), buff + current_lo, (uint32_t)(current_hi - current_lo));
}

static int64_t refine_onset_using_ste(const int16_t *buff, int32_t nearby_offset, uint8_t nearby_offset_valid)
{
   // Advance noise_start past any earlier onset so that the estimate reflects true background energy
   int32_t noise_start = 0;
   const int32_t noise_end = REFINE_NOISE_WIN_S;
   if (nearby_offset_valid && (nearby_offset < 0))
   {
      const int32_t prev_in_ctx = nearby_offset + REFINE_ONSET_IN_CTX;
      if ((prev_in_ctx >= noise_start) && (prev_in_ctx < noise_end))
      {
         const int32_t new_start = prev_in_ctx + (MIN_SAMPLES_BETWEEN_ONSETS / 2);
         if ((new_start > noise_start) && (new_start <= noise_end))
            noise_start = new_start;
      }
   }

   // Compute a noise energy estimate from the noise reference window
   float noise_energy = 1e-12f;
   if (noise_end > noise_start)
   {
      q63_t power_raw;
      arm_power_q15(buff + noise_start, (uint32_t)(noise_end - noise_start), &power_raw);
      const float cand = (float)((double)power_raw / ((noise_end - noise_start) * 1073741824.0));
      if (cand > noise_energy) noise_energy = cand;
   }

   // Pass 1: Forward scan over the full search+forward window to find the true signal peak.
   // Using the peak at REFINE_ONSET_IN_CTX alone underestimates peak STE for distant shots
   // because the acoustic envelope typically peaks well after the spectral-flux trigger.
   q63_t power_raw;
   float peak_ste = 0.0f;
   int32_t peak_pos = REFINE_ONSET_IN_CTX;
   const int32_t scan_start = REFINE_NOISE_WIN_S + REFINE_NOISE_GAP_S, fwd_end = REFINE_CTX_LEN - REFINE_STE_WIN_S;
   for (int32_t pos = scan_start; pos <= fwd_end; pos += REFINE_STE_HOP_S)
   {
      arm_power_q15(buff + pos, REFINE_STE_WIN_S, &power_raw);
      const float ste = (float)((double)power_raw / (REFINE_STE_WIN_S * 1073741824.0));
      if (ste > peak_ste) { peak_ste = ste; peak_pos = pos; }
   }
   if (peak_pos > REFINE_ONSET_IN_CTX)
      peak_pos = REFINE_ONSET_IN_CTX;

   // Threshold based on the true peak; for high-SNR shots the 25%-of-peak term dominates,
   // keeping the onset sharp; for low-SNR shots the noise-floor guard dominates.
   const float threshold = ((peak_ste * REFINE_PEAK_FRACTION) > (noise_energy * REFINE_NOISE_MULT)) ? peak_ste * REFINE_PEAK_FRACTION : noise_energy * REFINE_NOISE_MULT;

   // Pass 2: Backward scan from the coarse onset location or the earlier true peak toward the noise region
   int32_t onset_pos = -1;
   for (int32_t pos = peak_pos; pos >= scan_start; pos -= REFINE_STE_HOP_S)
   {
      arm_power_q15(buff + pos, REFINE_STE_WIN_S, &power_raw);
      const float ste = (float)((double)power_raw / (REFINE_STE_WIN_S * 1073741824.0));
      if (ste < threshold) break;
      onset_pos = pos;
   }
   if (onset_pos < 0) return 0;

   // Sample-level refinement: Scan forward within the triggering STE window for the first
   // individual sample whose squared amplitude exceeds the threshold.
   int64_t result = (int64_t)(onset_pos - REFINE_ONSET_IN_CTX);
   const int32_t P_end = onset_pos + REFINE_STE_WIN_S;
   for (int32_t s = onset_pos; s < P_end; ++s)
   {
      const float sv = (float)buff[s] / 32768.0f;
      if ((sv * sv) >= threshold) { result = (int64_t)(s - REFINE_ONSET_IN_CTX); break; }
   }
   return result;
}

static void fill_xcorr_window_q15(int16_t *out, int ch, int64_t onset_samp_abs, int64_t current_start, const int16_t (*audio_samples)[AUDIO_BUFFER_SAMPLES_PER_CHANNEL])
{
   const int32_t win_start_rel = (int32_t)(onset_samp_abs - (XCORR_WINDOW_SIZE / 2) - current_start), previous_offset = (int32_t)(previous_start_sample - current_start);
   const int32_t cur_lo = (win_start_rel < 0) ? -win_start_rel : 0, cur_hi = (AUDIO_PACKET_NUM_SAMPLES - win_start_rel < XCORR_WINDOW_SIZE) ? AUDIO_PACKET_NUM_SAMPLES - win_start_rel : XCORR_WINDOW_SIZE;
   const int32_t previous_lo_u = previous_offset - win_start_rel, previous_hi_u = previous_offset + AUDIO_PACKET_NUM_SAMPLES - win_start_rel, previous_lo = (previous_lo_u > 0) ? previous_lo_u : 0;
   const int32_t previous_hi = (previous_hi_u < XCORR_WINDOW_SIZE) ? previous_hi_u : XCORR_WINDOW_SIZE;
   arm_fill_q15(0, out, XCORR_WINDOW_SIZE);
   if (previous_lo < previous_hi)
      arm_copy_q15(previous_audio[ch] + (previous_lo + win_start_rel - previous_offset), out + previous_lo, (uint32_t)(previous_hi - previous_lo));
   if (cur_lo < cur_hi)
      arm_copy_q15(audio_samples[ch] + (cur_lo + win_start_rel), out + cur_lo, (uint32_t)(cur_hi - cur_lo));
}

static void fill_xcorr_channel(float *out, int ch, int64_t onset_samp_abs, int64_t current_start, const int16_t (*audio_samples)[AUDIO_BUFFER_SAMPLES_PER_CHANNEL])
{
   int16_t xcorr_win[XCORR_WINDOW_SIZE];
   fill_xcorr_window_q15(xcorr_win, ch, onset_samp_abs, current_start, audio_samples);
   arm_q15_to_float(xcorr_win, out, XCORR_WINDOW_SIZE);
}

static void gcc_phat_correlate(float *out, int ch, int64_t onset_samp_abs, int64_t current_start, const int16_t (*audio_samples)[AUDIO_BUFFER_SAMPLES_PER_CHANNEL])
{
   // Create a window and take the FFT of the indicated channel
   static float gcc_buf[XCORR_FFT_SIZE];
   fill_xcorr_channel(out, ch, onset_samp_abs, current_start, audio_samples);
   arm_fill_f32(0.0f, out + XCORR_WINDOW_SIZE, XCORR_FFT_SIZE - XCORR_WINDOW_SIZE);
   arm_rfft_fast_f32(&fft_xcorr, out, out, 0);

   // Compute the cross-spectrum gcc_ch0_fft * conj(out) for in-band bins into gcc_buf
   arm_copy_f32(gcc_ch0_fft, gcc_buf, XCORR_FFT_SIZE);
   gcc_buf[0] = gcc_buf[1] = 0.0f;
   for (int k = 1; k < XCORR_FFT_SIZE / 2; ++k)
   {
      if ((k < XCORR_PHAT_MIN_BIN) || (k > XCORR_PHAT_MAX_BIN))
      {
         gcc_buf[2*k] = gcc_buf[2*k + 1] = 0.0f;
         continue;
      }
      const float ar = gcc_buf[2*k], ai = gcc_buf[2*k + 1];
      const float br = out[2*k], bi = out[2*k + 1];
      gcc_buf[2*k] = (ar * br) + (ai * bi);
      gcc_buf[2*k + 1] = (ai * br) - (ar * bi);
   }

   // Batch-compute the FFT magnitudes, then normalize gcc_buf
   arm_cmplx_mag_f32(gcc_buf + 2, out, XCORR_FFT_SIZE / 2 - 1);
   for (int k = 1; k < XCORR_FFT_SIZE / 2; ++k)
   {
      const float mag = out[k - 1];
      if (mag > 1e-12f)
      {
         const float inv_mag = 1.0f / mag;
         gcc_buf[2*k] *= inv_mag;
         gcc_buf[2*k + 1] *= inv_mag;
      }
      else
         gcc_buf[2*k] = gcc_buf[2*k + 1] = 0.0f;
   }

   // Take the IFFT to generate the correlation function in the delay domain
   arm_rfft_fast_f32(&fft_xcorr, gcc_buf, out, 1);
}

static double calculate_interchannel_delays(int64_t onset_samp_abs, int64_t current_start, const int16_t (*audio_samples)[AUDIO_BUFFER_SAMPLES_PER_CHANNEL], double inter_channel_delays[AUDIO_NUM_CHANNELS-1], int selected_ch)
{
   // Compute and cache the ch0 FFT once
   fill_xcorr_channel(gcc_ch0_fft, 0, onset_samp_abs, current_start, audio_samples);
   arm_fill_f32(0.0f, gcc_ch0_fft + XCORR_WINDOW_SIZE, XCORR_FFT_SIZE - XCORR_WINDOW_SIZE);
   arm_rfft_fast_f32(&fft_xcorr, gcc_ch0_fft, gcc_ch0_fft, 0);

   // Compute the GCC-PHAT correlation function for each channel pair
   static float corr[AUDIO_NUM_CHANNELS-1][XCORR_FFT_SIZE];
   for (int c = 1; c < AUDIO_NUM_CHANNELS; ++c)
      gcc_phat_correlate(corr[c - 1], c, onset_samp_abs, current_start, audio_samples);

   // Joint search: find (d1, d2, d3) that maximizes the sum of correlation values subject to:
   //   d1 >= d1_min  (elevation floor: mic pair 2 measures the vertical axis)
   //   d1^2 + d2^2 <= D_sq  (unit-sphere constraint)
   //   d3 in {d1+d2-1, d1+d2, d1+d2+1}  (geometric consistency with +-1 quantization slack)
   float best_score = -1e30f;
   int best_d1 = 0, best_d2 = 0, best_d3 = 0;
   const int D_sq = XCORR_MAX_DELAY_SAMP * XCORR_MAX_DELAY_SAMP + 1;
   const int d1_min = (int)ceilf(sinf(AOA_MIN_ELEVATION_DEG * ((float)M_PI / 180.0f)) * (float)XCORR_MAX_DELAY_SAMP);
   for (int d1 = d1_min; d1 <= XCORR_MAX_DELAY_SAMP; ++d1)
   {
      const int i1 = (d1 >= 0) ? d1 : (XCORR_FFT_SIZE + d1);
      const float s1 = corr[0][i1];
      for (int d2 = -XCORR_MAX_DELAY_SAMP; d2 <= XCORR_MAX_DELAY_SAMP; ++d2)
      {
         if (((d1*d1) + (d2*d2)) > D_sq)
            continue;

         const int i2 = (d2 >= 0) ? d2 : (XCORR_FFT_SIZE + d2);
         const float s12 = s1 + corr[1][i2];
         for (int dd = -1; dd <= 1; ++dd)
         {
            const int d3 = d1 + d2 + dd;
            const int i3 = (d3 >= 0) ? d3 : (XCORR_FFT_SIZE + d3);
            const float score = s12 + corr[2][i3];
            if (score > best_score)
            {
               best_score = score;
               best_d1 = d1; best_d2 = d2; best_d3 = d3;
            }
         }
      }
   }
   inter_channel_delays[0] = (double)best_d1 / AUDIO_SAMPLE_RATE_HZ;
   inter_channel_delays[1] = (double)best_d2 / AUDIO_SAMPLE_RATE_HZ;
   inter_channel_delays[2] = (double)best_d3 / AUDIO_SAMPLE_RATE_HZ;

   // Compute the normalized magnitude as the RMS of the currently selected channel's onset window
   q63_t power_raw;
   int16_t xcorr_win[XCORR_WINDOW_SIZE];
   fill_xcorr_window_q15(xcorr_win, selected_ch, onset_samp_abs, current_start, audio_samples);
   arm_power_q15(xcorr_win, XCORR_WINDOW_SIZE, &power_raw);
   return sqrt((double)power_raw / (XCORR_WINDOW_SIZE * 1073741824.0));
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
   // Check for spectral activation across a broad number of frequency bands
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

static int32_t onset_detector_refine(const int16_t *window)
{
   // Scan fine-grained FFT windows within the coarse detection frame to pinpoint the onset
   int32_t onset_starting_sample = 0;
   float threshold = 2.0f * compute_flux_fine(window);
   for (int32_t off = FFT_FINE_STEP_SIZE; off <= (FFT_WINDOW_SIZE - FFT_FINE_WINDOW_SIZE); off += FFT_FINE_STEP_SIZE)
   {
      const float flux = compute_flux_fine(window + off);
      if (flux > threshold)
      {
         threshold = 2.0f * flux;
         onset_starting_sample = off;
      }
   }
   samples_since_prev_onset = -onset_starting_sample;
   return onset_starting_sample + FFT_FINE_WINDOW_SIZE - FFT_FINE_STEP_SIZE;
}

static int32_t process_time_step(const int16_t *window)
{
   // Calculate spectral flux on the current window and update the adaptive threshold
   const float flux = compute_flux_coarse(window);
   const float adaptive_threshold = update_flux_statistics(flux);
   samples_since_prev_onset += FFT_STEP_SIZE;

   // Check if this time step meets all conditions for an onset
   int32_t onset_starting_sample = -1;
   if (check_onset_conditions(flux, adaptive_threshold))
      onset_starting_sample = onset_detector_refine(window);
   prev_flux = flux;
   return onset_starting_sample;
}


// Public API Functions ------------------------------------------------------------------------------------------------

void onset_detection_init(void)
{
   // Initialize all FFT instances and pre-compute Hanning windows
   arm_rfft_init(&fft_coarse);
   arm_rfft_fine_init(&fft_fine);
   arm_rfft_xcorr_init(&fft_xcorr);
   arm_hanning_f32(hanning_window, FFT_WINDOW_SIZE);
   arm_hanning_f32(hanning_window_fine, FFT_FINE_WINDOW_SIZE);

   // Zero the detection state
   arm_fill_f32(0.0f, magnitudes, FFT_FILTER_SIZE / 2);
   arm_fill_f32(0.0f, prev_magnitudes, FFT_FILTER_SIZE / 2);
   arm_fill_f32(0.0f, magnitudes_fine, NUM_FFT_FINE_BINS);
   arm_fill_f32(0.0f, prev_magnitudes_fine, NUM_FFT_FINE_BINS);
   arm_fill_f32(0.0f, broadband_noise_floor, BROADBAND_NOISE_BANDS);
   arm_fill_q15(0, refinement_buf, REFINE_CTX_LEN);
   memset(previous_audio, 0, sizeof(previous_audio));
   flux_ema_mean = flux_ema_var = prev_flux = 0.0f;
   flux_warmup_count = 0;
   samples_since_prev_onset = 10000000;
   previous_timestamp = 0.0;
   previous_start_sample = 0;
   previous_raw_onset_sample = -1;

   // Clear all telemetry metrics
#ifdef TELEMETRY_ENABLE_METRICS
   onset_invoke_count = onset_detect_count = 0;
   onset_ms_last = onset_ms_max = 0.0f;
#endif
}

void onset_detection_invoke(double packet_timestamp, const int16_t (*audio_samples)[AUDIO_BUFFER_SAMPLES_PER_CHANNEL], volatile data_packet_t* const packet)
{
   // Initialize telemetry metric cycle count
#ifdef TELEMETRY_ENABLE_METRICS
   const uint32_t cycle_start = READ_REG(DWT->CYCCNT);
#endif

   // Use the first audio channel that is not in an alarm state
   int selected_ch = 0;
   packet->onset_detected = 0;
   const int16_t *audio_packet = NULL;
   if (!packet->channel_alarms.alarm.ch1) { audio_packet = audio_samples[0]; selected_ch = 0; }
   else if (!packet->channel_alarms.alarm.ch2) { audio_packet = audio_samples[1]; selected_ch = 1; }
   else if (!packet->channel_alarms.alarm.ch3) { audio_packet = audio_samples[2]; selected_ch = 2; }
   else if (!packet->channel_alarms.alarm.ch4) { audio_packet = audio_samples[3]; selected_ch = 3; }
   else return;

   // Process the audio windows that straddle the previous and current packets
   double onset_timestamp = 0.0;
   static int16_t pending_window[FFT_WINDOW_SIZE];
   for (int m = 0; m < MISSING_WINDOWS_PER_PACKET; ++m)
   {
      const int left_len = (FFT_WINDOW_SIZE - FFT_STEP_SIZE) - (m * FFT_STEP_SIZE);
      arm_copy_q15(&previous_audio[selected_ch][AUDIO_PACKET_NUM_SAMPLES - left_len], pending_window, left_len);
      arm_copy_q15(audio_packet, pending_window + left_len, FFT_WINDOW_SIZE - left_len);
      const int32_t onset_sample = process_time_step(pending_window);
      if (onset_sample >= 0)
         onset_timestamp = packet_timestamp + ((onset_sample - AUDIO_PACKET_NUM_SAMPLES + ((AVAILABLE_WINDOWS_PER_PACKET + m) * FFT_STEP_SIZE)) / (double)AUDIO_SAMPLE_RATE_HZ);
   }

   // Process the audio windows fully contained within the current packet
   for (int t = 0; t < AVAILABLE_WINDOWS_PER_PACKET; ++t)
   {
      const int32_t onset_sample = process_time_step(audio_packet + (t * FFT_STEP_SIZE));
      if (onset_sample >= 0)
         onset_timestamp = packet_timestamp + (((t * FFT_STEP_SIZE) + onset_sample) / (double)AUDIO_SAMPLE_RATE_HZ);
   }

   // Continue processing the event if an onset was detected
   const int64_t current_start = (int64_t)round(packet_timestamp * AUDIO_SAMPLE_RATE_HZ);
   if (onset_timestamp > 0.0)
   {
      // Refine the onset timestamp using short-time energy analysis on the raw waveform
      const int64_t onset_sample_raw = (int64_t)round(onset_timestamp * AUDIO_SAMPLE_RATE_HZ);
      build_refinement_buffer(refinement_buf, onset_sample_raw, current_start, audio_samples, selected_ch);
      const int64_t onset_delta = refine_onset_using_ste(refinement_buf, (int32_t)(previous_raw_onset_sample - onset_sample_raw), previous_raw_onset_sample >= 0);
      onset_timestamp += (double)onset_delta / AUDIO_SAMPLE_RATE_HZ;
      previous_raw_onset_sample = onset_sample_raw + onset_delta;

      // Retrieve all inter-channel delays along with a normalized event magnitude
      double inter_channel_delays[AUDIO_NUM_CHANNELS-1];
      const double onset_magnitude = calculate_interchannel_delays(previous_raw_onset_sample, current_start, audio_samples, inter_channel_delays, selected_ch);

      // Calculate the angle of arrival of the detected onset
      float angle_of_arrival[3] = { 0 };
      float observations[AUDIO_NUM_CHANNELS-1] = { -343.0f * inter_channel_delays[0], -343.0f * inter_channel_delays[1], -343.0f * inter_channel_delays[2] };
      least_squares_3x2(angle_of_arrival, observations);
      angle_of_arrival[2] = angle_of_arrival[1];
      const float horiz_sq = angle_of_arrival[0]*angle_of_arrival[0] + angle_of_arrival[2]*angle_of_arrival[2];
      angle_of_arrival[1] = sqrtf((horiz_sq < 1.0f) ? 1.0f - horiz_sq : 0.0f);

      // Write detection results into the specified data packet
      packet->onset_detected = 1;
      packet->onset_timestamp = onset_timestamp;
      packet->onset_magnitude = onset_magnitude;
      packet->angle_of_arrival[0] = angle_of_arrival[0];
      packet->angle_of_arrival[1] = angle_of_arrival[1];
      packet->angle_of_arrival[2] = angle_of_arrival[2];
   }

   // Update the previous audio buffer for the next onset detection invocation
   previous_timestamp = packet_timestamp;
   previous_start_sample = current_start;
   for (int c = 0; c < AUDIO_NUM_CHANNELS; ++c)
      arm_copy_q15(audio_samples[c], previous_audio[c], AUDIO_PACKET_NUM_SAMPLES);

   // Update the most recent telemetry metrics
#ifdef TELEMETRY_ENABLE_METRICS
   onset_ms_last = 1000.0f * ((float)(READ_REG(DWT->CYCCNT) - cycle_start) / SystemCoreClock);
   ++onset_invoke_count;
   if (onset_timestamp > 0.0)
      ++onset_detect_count;
   if (onset_ms_last > onset_ms_max)
      onset_ms_max = onset_ms_last;
#endif
}

#endif  // #ifdef CORE_CM7
