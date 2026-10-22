#ifndef DSP_PIPELINE_H
#define DSP_PIPELINE_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "esp_err.h"
#include "esp_timer.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DSP_MAX_SAMPLES              2048
#define DSP_SAMPLES_PER_DEPTH        512
#define DSP_DEPTH_RESOLUTION         256
#define DSP_MAX_ASCAN_POINTS         1024
#define DSP_MAX_BSCAN_COLUMNS        2048
#define DSP_FILTER_ORDER_MAX         128
#define DSP_FFT_SIZE_MAX             1024
#define DSP_Window_SIZE_MAX          512
#define DSP_CORRELATION_MAX_LAG      256
#define DSP_MAX_PEAKS                32
#define DSP_SPECTRAL_BINS            512
#define DSP_ENVELOPE_SMOOTH_WINDOW   7
#define DSP_HILBERT_TAPS             257
#define DSP_STFT_HOP_SIZE            128
#define DSP_STFT_FRAME_SIZE          512
#define DSP_STFT_OVERLAP             384

typedef enum {
    DSP_STAGE_IDLE = 0,
    DSP_STAGE_DC_REMOVAL = 1,
    DSP_STAGE_TGC_COMPENSATION = 2,
    DSP_STAGE_BANDPASS_FILTER = 3,
    DSP_STAGE_MTI_FILTER = 4,
    DSP_STAGE_WHITENING = 5,
    DSP_STAGE_FFT_ANALYSIS = 6,
    DSP_STAGE_HILBERT_ENVELOPE = 7,
    DSP_STAGE_PEAK_DETECTION = 8,
    DSP_STAGE_FEATURE_EXTRACTION = 9,
    DSP_STAGE_NORMALIZATION = 10,
    DSP_STAGE_COMPLETE = 11
} dsp_processing_stage_t;

typedef enum {
    FILTER_TYPE_LOWPASS = 0,
    FILTER_TYPE_HIGHPASS = 1,
    FILTER_TYPE_BANDPASS = 2,
    FILTER_TYPE_BANDSTOP = 3,
    FILTER_TYPE_MatchedFilter = 4,
    FILTER_TYPE_WIENER = 5,
    FILTER_TYPE_KALMAN = 6
} filter_type_t;

typedef enum {
    WINDOW_TYPE_RECTANGULAR = 0,
    WINDOW_TYPE_HANNING = 1,
    WINDOW_TYPE_HAMMING = 2,
    WINDOW_TYPE_BLACKMAN = 3,
    WINDOW_TYPE_KAISER = 4,
    WINDOW_TYPE_GAUSSIAN = 5,
    WINDOW_TYPE_CHEBYSHEV = 6,
    WINDOW_TYPE_BARTLETT = 7,
    WINDOW_TYPE_WELCH = 8
} window_type_t;

typedef enum {
    FFT_FORWARD = 0,
    FFT_INVERSE = 1
} fft_direction_t;

typedef enum {
    NORMALIZATION_NONE = 0,
    NORMALIZATION_MIN_MAX = 1,
    NORMALIZATION_Z_SCORE = 2,
    NORMALIZATION_ROBUST = 3,
    NORMALIZATION_PERCENTILE = 4
} normalization_method_t;

typedef enum {
    PEAK_MODE_LOCAL = 0,
    PEAK_MODE_GLOBAL = 1,
    PEAK_MODE_ADAPTIVE = 2
} peak_detection_mode_t;

typedef enum {
    TGC_MODE_LINEAR = 0,
    TGC_MODE_EXPONENTIAL = 1,
    TGC_MODE_SEGMENTED = 2,
    TGC_MODE_ADAPTIVE = 3,
    TGC_MODE_COMPRESSED = 4
} tgc_mode_t;

typedef enum {
    MTI_MODE_NONE = 0,
    MTI_MODE_HIGH_PASS = 1,
    MTI_MODE_SUBTRACTION = 2,
    MTI_MODE_EWMA = 3,
    MTI_MODE_ALPHA_TRIMMED = 4,
    MTI_MODE_MEDIAN = 5
} mti_mode_t;

typedef enum {
    SPECTRAL_ESTIMATE_FFT = 0,
    SPECTRAL_ESTIMATE_WELCH = 1,
    SPECTRAL_ESTIMATE_MUSIC = 2,
    SPECTRAL_ESTIMATE_AR = 3
} spectral_estimate_method_t;

typedef enum {
    TRACE_STATE_FRESH = 0,
    TRACE_STATE_PROCESSING = 1,
    TRACE_STATE_DONE = 2,
    TRACE_STATE_ERROR = 3
} trace_state_t;

typedef struct {
    float fc_low;
    float fc_high;
    float ripple_db;
    float stopband_atten_db;
    uint16_t order;
    filter_type_t filter_type;
    window_type_t window_type;
    float cutoff_freq_normalized;
    bool use_embedded_ coefficients;
} bandpass_filter_config_t;

typedef struct {
    uint32_t sample_count;
    float sample_rate_hz;
    float center_frequency_hz;
    float bandwidth_hz;
    float pulse_width_us;
    float prf_hz;
    float antenna_delay_ns;
    float velocity_m_per_ns;
    float ground_permittivity;
    float attenuation_db_per_m;
    float noise_floor_dbm;
    float dynamic_range_db;
} radar_parameters_t;

typedef struct {
    float* input_buffer;
    float* output_buffer;
    float* dc_offset_buffer;
    float* temp_buffer_a;
    float* temp_buffer_b;
    float* fft_real_input;
    float* fft_imag_input;
    float* fft_magnitude;
    float* hilbert_coeffs;
    float* envelope_output;
    float* background_model;
    float* mti_output;
    float* peak_indices;
    float* peak_magnitudes;
    float* feature_vector;
    float* spectral_density;
    float* correlation_result;
    uint16_t input_size;
    uint16_t output_size;
    trace_state_t state;
    uint32_t processing_flags;
    uint64_t timestamp_us;
} dsp_buffer_t;

typedef struct {
    float mean;
    float variance;
    float std_dev;
    float skewness;
    float kurtosis;
    float rms;
    float peak_to_peak;
    float crest_factor;
    float zero_crossing_rate;
    float spectral_centroid;
    float spectral_entropy;
    float spectral_flatness;
    float spectral_rolloff;
    float energy_95_percent;
    float dominance_frequency;
    float snr_estimate;
} signal_statistics_t;

typedef struct {
    float peak_value;
    uint16_t peak_index;
    float peak_width;
    float peak_area;
    float peak_snr;
    float peak_prominence;
    uint8_t peak_type;
    bool is_valid;
} peak_info_t;

typedef struct {
    float bandwidth_ratio;
    float symmetry_index;
    float rise_time_ns;
    float fall_time_ns;
    float pulse_duration_ns;
    float ringing_ratio;
    float main_lobe_width;
    float side_lobe_level_db;
} pulse_shape_features_t;

typedef struct {
    float depth_meters;
    float amplitude;
    float phase;
    float bandwidth;
    float pulse_width;
    float arrival_time_ns;
    float reflection_coefficient;
    float snr_db;
    bool is_anomaly;
    float anomaly_score;
    float confidence;
} depth_slice_features_t;

typedef struct {
    dsp_processing_stage_t current_stage;
    dsp_processing_stage_t next_stage;
    uint32_t stage_mask;
    uint16_t samples_processed;
    uint16_t samples_total;
    float stage_elapsed_ms;
    float total_elapsed_ms;
    uint32_t pipeline_errors;
    bool bypass_enabled;
    bool diagnostic_mode;
} processing_status_t;

typedef struct {
    uint16_t fft_size;
    uint16_t overlap_samples;
    uint16_t hop_size;
    window_type_t window;
    float noise_floor_normalized;
    float spectrum_scaling_factor;
    float frequency_resolution_hz;
    float* window_coefficients;
    float* fft_scratch;
    bool is_initialized;
    uint32_t frame_count;
} stft_config_t;

typedef struct {
    float bandwidth_ratio;
    float center_frequency;
    float quality_factor;
    float ring_duration_ns;
    float damping_factor;
    float resonance_peak_db;
    bool has_ringing;
} wavelet_features_t;

typedef struct {
    bool enable_dc_removal;
    bool enable_tgc;
    bool enable_bandpass;
    bool enable_mti;
    bool enable_whitening;
    bool enable_hilbert;
    bool enable_fft_analysis;
    bool enable_normalization;
    bool enable_peak_detection;
    bool enable_feature_extraction;
    bool enable_spectral_analysis;
    bool enable_wavelet_transform;
    uint32_t processing_flags;
} processing_flags_t;

typedef struct {
    float target_gain_db;
    float attenuation_coefficient_alpha;
    float depth_compensation_factor;
    float saturation_threshold;
    float noise_floor_threshold;
    uint16_t tgc_table_size;
    float* tgc_lookup_table;
    tgc_mode_t mode;
    bool adaptive_mode_enabled;
    float adaptation_rate;
    uint16_t depth_slices;
} tgc_config_t;

typedef struct {
    float mti_coefficient_beta;
    float mti_threshold;
    uint16_t background_update_interval;
    uint16_t initialization_samples;
    float convergence_rate;
    float adaptation_factor;
    bool adaptive_background;
    mti_mode_t mode;
    uint8_t filter_order;
    uint16_t reset_counter;
    float background_variance;
    float background_mean;
} mti_config_t;

typedef struct {
    float lower_threshold;
    float upper_threshold;
    float peak_spacing_min;
    float prominence_threshold;
    float width_threshold_percent;
    uint16_t max_peaks;
    peak_detection_mode_t mode;
    bool enable_validation;
    float snr_min_db;
    float valley_threshold_ratio;
    uint8_t local_window_radius;
} peak_detection_config_t;

typedef struct {
    float threshold_multiplier;
    float percentile_cutoff;
    float robust_scale_factor;
    normalization_method_t method;
    float clip_min;
    float clip_max;
    bool auto_scale;
} normalization_config_t;

typedef struct {
    float bandwidth_hz;
    float center_freq_hz;
    uint16_t filter_length;
    float transition_bw_hz;
    float ripple_db;
    float stopband_attenuation_db;
    float* filter_coefficients;
    float* filter_state;
    uint16_t state_length;
    bool is_adaptive;
    float adaptation_mu;
    float adaptation_gamma;
} adaptive_filter_state_t;

typedef struct {
    float* spectral_estimates;
    float* power_spectrum;
    float* cumulative_power;
    uint16_t num_frames;
    uint16_t frame_length;
    uint16_t overlap_length;
    float* window_function;
    uint16_t fft_size;
    bool is_welch_active;
    uint32_t averaging_count;
} spectral_analysis_state_t;

typedef struct {
    bool is_configured;
    bool is_running;
    bool pipeline_busy;
    bool overflow_detected;
    uint32_t total_traces_processed;
    uint32_t traces_dropped;
    uint32_t processing_errors;
    float average_processing_time_us;
    float max_processing_time_us;
    float min_processing_time_us;
    uint32_t peak_buffer_overflows;
    uint32_t filter_stages_executed;
} pipeline_status_monitor_t;

typedef struct {
    float max_phase_deviation;
    float amplitude_stability_index;
    float baseline_drift_rate;
    float noise_floor_track[16];
    uint8_t noise_floor_index;
    float temperature_compensation;
    float humidity_compensation;
    float calibration_age_hours;
    bool calibration_valid;
    float gain_drift_percent;
    float phase_drift_degrees;
} calibration_tracking_t;

typedef struct {
    float spectral_centroid_hz;
    float spectral_bandwidth_hz;
    float spectral_contrast;
    float spectral_rolloff_percent;
    float spectral_flatness;
    float spectral_entropy;
    float spectral_flux;
    float spectral_energy;
    float spectral_spread;
    float spectral_skewness;
    float spectral_kurtosis;
} spectral_features_t;

typedef struct {
    dsp_buffer_t buffers;
    radar_parameters_t radar_params;
    processing_flags_t flags;
    tgc_config_t tgc_cfg;
    mti_config_t mti_cfg;
    peak_detection_config_t peak_cfg;
    normalization_config_t norm_cfg;
    stft_config_t stft_cfg;
    processing_status_t status;
    pipeline_status_monitor_t monitor;
    calibration_tracking_t cal_tracking;
    signal_statistics_t stats;
    peak_info_t peaks[DSP_MAX_PEAKS];
    uint16_t peak_count;
    spectral_features_t spectral_feat;
    esp_timer_handle_t processing_timer;
    SemaphoreHandle_t pipeline_mutex;
    bool is_initialized;
    uint8_t instance_id;
} dsp_pipeline_context_t;

typedef struct {
    float threshold_percentile;
    float peak_prominence_min;
    float peak_separation_min;
    float min_peak_width;
    float max_peak_width;
    bool use_relative_threshold;
    bool refine_peak_location;
    uint8_t interpolation_order;
    float curvature_threshold;
} peak_refinement_config_t;

typedef struct {
    float gain_db;
    float phase_degrees;
    float group_delay_samples;
    float noise_figure_db;
    float dynamic_range_db;
    bool calibration_valid;
    uint64_t calibration_timestamp;
    float temperature_at_calibration;
} frontend_calibration_t;

esp_err_t dsp_pipeline_init(dsp_pipeline_context_t* ctx, uint8_t instance_id);
esp_err_t dsp_pipeline_deinit(dsp_pipeline_context_t* ctx);
esp_err_t dsp_pipeline_configure(dsp_pipeline_context_t* ctx, const radar_parameters_t* params);
esp_err_t dsp_pipeline_reset(dsp_pipeline_context_t* ctx);
esp_err_t dsp_pipeline_process(dsp_pipeline_context_t* ctx, const float* input, float* output, uint16_t length);
esp_err_t dsp_pipeline_process_async(dsp_pipeline_context_t* ctx, const float* input, float* output, uint16_t length);
esp_err_t dsp_pipeline_set_stage_bypass(dsp_pipeline_context_t* ctx, dsp_processing_stage_t stage, bool bypass);
esp_err_t dsp_pipeline_get_processing_time(dsp_pipeline_context_t* ctx, float* elapsed_ms);
esp_err_t dsp_pipeline_get_statistics(dsp_pipeline_context_t* ctx, signal_statistics_t* stats);
esp_err_t dsp_pipeline_get_peaks(dsp_pipeline_context_t* ctx, peak_info_t* peaks, uint16_t* count);
esp_err_t dsp_pipeline_set_tgc_mode(dsp_pipeline_context_t* ctx, tgc_mode_t mode);
esp_err_t dsp_pipeline_set_mti_mode(dsp_pipeline_context_t* ctx, mti_mode_t mode, float beta);
esp_err_t dsp_pipeline_set_peak_config(dsp_pipeline_context_t* ctx, const peak_detection_config_t* config);
esp_err_t dsp_pipeline_set_normalization(dsp_pipeline_context_t* ctx, normalization_method_t method, float clip_min, float clip_max);
esp_err_t dsp_pipeline_update_background_model(dsp_pipeline_context_t* ctx, const float* background, uint16_t length);
esp_err_t dsp_pipeline_get_spectral_features(dsp_pipeline_context_t* ctx, spectral_features_t* features);
esp_err_t dsp_pipeline_execute_fft(dsp_pipeline_context_t* ctx, const float* input, float* magnitude, float* phase, uint16_t size);
esp_err_t dsp_pipeline_execute_ifft(dsp_pipeline_context_t* ctx, const float* real_in, const float* imag_in, float* output, uint16_t size);
esp_err_t dsp_pipeline_compute_hilbert_envelope(dsp_pipeline_context_t* ctx, const float* input, float* envelope, uint16_t length);
esp_err_t dsp_pipeline_apply_bandpass_filter(dsp_pipeline_context_t* ctx, const float* input, float* output, uint16_t length);
esp_err_t dsp_pipeline_apply_mti_filter(dsp_pipeline_context_t* ctx, const float* input, float* output, uint16_t length);
esp_err_t dsp_pipeline_remove_dc_offset(dsp_pipeline_context_t* ctx, const float* input, float* output, uint16_t length);
esp_err_t dsp_pipeline_apply_tgc(dsp_pipeline_context_t* ctx, const float* input, float* output, uint16_t length);
esp_err_t dsp_pipeline_detect_peaks(dsp_pipeline_context_t* ctx, const float* signal, uint16_t length);
esp_err_t dsp_pipeline_normalize(dsp_pipeline_context_t* ctx, const float* input, float* output, uint16_t length);
esp_err_t dsp_pipeline_compute_stft(dsp_pipeline_context_t* ctx, const float* input, float* stft_output, uint16_t num_frames);
esp_err_t dsp_pipeline_compute_wavelet_transform(dsp_pipeline_context_t* ctx, const float* input, float* wavelet_output, uint16_t scales);
esp_err_t dsp_pipeline_reset_statistics(dsp_pipeline_context_t* ctx);
esp_err_t dsp_pipeline_reset_background(dsp_pipeline_context_t* ctx);
esp_err_t dsp_pipeline_get_stage_status(dsp_pipeline_context_t* ctx, dsp_processing_stage_t stage, bool* enabled, float* elapsed_us);
esp_err_t dsp_pipeline_set_diagnostic_mode(dsp_pipeline_context_t* ctx, bool enable);
esp_err_t dsp_pipeline_log_stage_times(dsp_pipeline_context_t* ctx, uint64_t timestamp);
esp_err_t dsp_pipeline_allocate_buffers(dsp_pipeline_context_t* ctx, uint16_t max_samples);
esp_err_t dsp_pipeline_free_buffers(dsp_pipeline_context_t* ctx);
esp_err_t dsp_pipeline_validate_configuration(dsp_pipeline_context_t* ctx);
esp_err_t dsp_pipeline_export_state(dsp_pipeline_context_t* ctx, uint8_t* buffer, uint32_t* size);
esp_err_t dsp_pipeline_import_state(dsp_pipeline_context_t* ctx, const uint8_t* buffer, uint32_t size);

float dsp_compute_signal_statistics(const float* data, uint16_t length, signal_statistics_t* stats);
float dsp_compute_rms(const float* data, uint16_t length);
float dsp_compute_snr(const float* signal, const float* noise, uint16_t length);
float dsp_find_threshold_percentile(const float* data, uint16_t length, float percentile);
float dsp_interpolate_peak_location(const float* y, uint16_t idx, uint8_t order);
void dsp_apply_window_function(float* data, uint16_t length, window_type_t window_type);
void dsp_normalize_vector(float* data, uint16_t length, float target_rms);
void dsp_scale_vector(float* data, uint16_t length, float scale_factor);
void dsp_clip_vector(float* data, uint16_t length, float min_val, float max_val);
void dsp_vector_subtract(float* result, const float* a, const float* b, uint16_t length);
void dsp_vector_add(float* result, const float* a, const float* b, uint16_t length);
void dsp_vector_multiply(float* result, const float* a, const float* b, uint16_t length);
void dsp_scalar_multiply(float* data, uint16_t length, float scalar);
float dsp_dot_product(const float* a, const float* b, uint16_t length);
float dsp_vector_mean(const float* data, uint16_t length);
float dsp_vector_variance(const float* data, uint16_t length, float mean);
float dsp_vector_std_dev(const float* data, uint16_t length);
float dsp_correlation_coefficient(const float* a, const float* b, uint16_t length);
void dsp_cross_correlation(const float* a, const float* b, float* result, uint16_t len_a, uint16_t len_b, uint16_t max_lag);
void dsp_autocorrelation(const float* data, float* result, uint16_t length, uint16_t max_lag);
float dsp_compute_spectral_entropy(const float* spectrum, uint16_t length);
float dsp_compute_spectral_flatness(const float* spectrum, uint16_t length);
float dsp_compute_spectral_centroid(const float* spectrum, uint16_t length, float sample_rate);
float dsp_compute_spectral_rolloff(const float* spectrum, uint16_t length, float threshold_percent);
float dsp_compute_zero_crossing_rate(const float* data, uint16_t length);
float dsp_compute_energy(const float* data, uint16_t length);
float dsp_compute_max(const float* data, uint16_t length);
float dsp_compute_min(const float* data, uint16_t length);
void dsp_find_local_maxima(const float* data, uint16_t length, uint16_t* indices, uint16_t* count, uint16_t min_spacing);
void dsp_quantize_signal(const float* input, int16_t* output, uint16_t length, float lsb_value);
void dsp_dequantize_signal(const int16_t* input, float* output, uint16_t length, float lsb_value);
void dsp_compute_moving_average(const float* input, float* output, uint16_t length, uint16_t window_size);
void dsp_compute_exponential_moving_average(const float* input, float* output, uint16_t length, float alpha);
void dsp_smooth_signal(const float* input, float* output, uint16_t length, uint8_t kernel_size);
void dsp_median_filter(const float* input, float* output, uint16_t length, uint8_t window_size);
void dsp_derivative(const float* input, float* output, uint16_t length);
void dsp_integrate(const float* input, float* output, uint16_t length);
void dsp_compute_gradient(const float* input, float* grad_x, float* grad_y, uint16_t width, uint16_t height);
void dsp_apply_morphological_operation(const float* input, float* output, uint16_t length, bool dilate);

esp_err_t dsp_fft_init(uint16_t fft_size);
esp_err_t dsp_fft_execute(const float* input_real, const float* input_imag, float* output_real, float* output_imag, uint16_t size, fft_direction_t direction);
esp_err_t dsp_fft_compute_magnitude(const float* real, const float* imag, float* magnitude, uint16_t size);
esp_err_t dsp_fft_compute_phase(const float* real, const float* imag, float* phase, uint16_t size);
esp_err_t dsp_fft_shift(float* data, uint16_t size);
esp_err_t dsp_fft_apply_window(float* data, uint16_t size, window_type_t window);
esp_err_t dsp_fft_deinit(void);

esp_err_t dsp_fir_filter_init(float* coefficients, uint16_t num_taps);
esp_err_t dsp_fir_filter_execute(const float* input, float* output, uint16_t length, const float* coefficients, uint16_t num_taps);
esp_err_t dsp_fir_filter_execute_block(const float* input, float* output, uint16_t block_size, const float* coefficients, uint16_t num_taps);
esp_err_t dsp_iir_filter_init(const float* b_coeffs, const float* a_coeffs, uint8_t sections);
esp_err_t dsp_iir_filter_execute(const float* input, float* output, uint16_t length, const float* b_coeffs, const float* a_coeffs, uint8_t sections);
esp_err_t dsp_design_bandpass_filter(float* coefficients, uint16_t* num_taps, float low_cutoff, float high_cutoff, float sample_rate, uint16_t order, window_type_t window);
esp_err_t dsp_design_lowpass_filter(float* coefficients, uint16_t* num_taps, float cutoff_freq, float sample_rate, uint16_t order, window_type_t window);
esp_err_t dsp_design_highpass_filter(float* coefficients, uint16_t* num_taps, float cutoff_freq, float sample_rate, uint16_t order, window_type_t window);
esp_err_t dsp_apply_filter(const float* input, float* output, uint16_t length, const float* coefficients, uint16_t num_taps);

esp_err_t dsp_generate_tgc_lookup_table(tgc_config_t* config, float sample_rate, float max_depth_meters, float velocity_m_per_ns);
esp_err_t dsp_apply_tgc_lookup(const float* input, float* output, uint16_t length, const float* tgc_lut);
esp_err_t dsp_adapt_tgc_gain(float* tgc_lut, uint16_t size, float current_snr, float target_snr);

esp_err_t dsp_init_mti_filter(mti_config_t* config, uint16_t max_samples);
esp_err_t dsp_update_mti_background(mti_config_t* config, const float* new_sample);
esp_err_t dsp_apply_mti_filter(mti_config_t* config, const float* input, float* output, uint16_t length);
esp_err_t dsp_reset_mti_filter(mti_config_t* config);
esp_err_t dsp_set_mti_coefficient(mti_config_t* config, float beta);

esp_err_t dsp_init_hilbert_transformer(uint16_t signal_length);
esp_err_t dsp_compute_hilbert_transform(const float* input, float* analytic_output, uint16_t length);
esp_err_t dsp_extract_envelope(const float* analytic_signal, float* envelope, uint16_t length);
esp_err_t dsp_extract_instantaneous_phase(const float* analytic_signal, float* phase, uint16_t length);
esp_err_t dsp_extract_instantaneous_frequency(const float* phase, float* frequency, uint16_t length);
esp_err_t dsp_deinit_hilbert_transformer(void);

esp_err_t dsp_detect_peaks_local(const float* signal, uint16_t length, const peak_detection_config_t* config, peak_info_t* peaks, uint16_t* peak_count);
esp_err_t dsp_detect_peaks_global(const float* signal, uint16_t length, const peak_detection_config_t* config, peak_info_t* peaks, uint16_t* peak_count);
esp_err_t dsp_detect_peaks_adaptive(const float* signal, uint16_t length, const peak_detection_config_t* config, peak_info_t* peaks, uint16_t* peak_count);
esp_err_t dsp_refine_peak_locations(const float* signal, peak_info_t* peaks, uint16_t peak_count, const peak_refinement_config_t* config);
esp_err_t dsp_validate_peaks(const float* signal, peak_info_t* peaks, uint16_t peak_count, float noise_floor);

esp_err_t dsp_init_spectral_analyzer(spectral_analysis_state_t* state, uint16_t fft_size, uint16_t frame_length, uint16_t overlap);
esp_err_t dsp_compute_power_spectrum(const float* frame, float* spectrum, uint16_t length);
esp_err_t dsp_compute_welch_periodogram(const float* input, uint16_t length, float* psd, uint16_t psd_size, uint16_t fft_size, uint16_t overlap, const float* window);
esp_err_t dsp_compute_psd_welch(const float* input, uint16_t length, float* psd, uint16_t psd_bins);
esp_err_t dsp_estimate_ar_spectrum(const float* input, uint16_t length, float* spectrum, uint16_t order, uint16_t psd_size);
esp_err_t dsp_free_spectral_analyzer(spectral_analysis_state_t* state);

esp_err_t dsp_normalize_min_max(const float* input, float* output, uint16_t length, float target_min, float target_max);
esp_err_t dsp_normalize_z_score(const float* input, float* output, uint16_t length);
esp_err_t dsp_normalize_robust(const float* input, float* output, uint16_t length, float percentile);
esp_err_t dsp_normalize_percentile(const float* input, float* output, uint16_t length, float lower_percentile, float upper_percentile);

esp_err_t dsp_init_stft(stft_config_t* config, uint16_t frame_size, uint16_t hop_size, window_type_t window_type);
esp_err_t dsp_compute_stft_frames(const float* input, uint16_t input_length, float* stft_output, stft_config_t* config, uint16_t* num_frames);
esp_err_t dsp_compute_inverse_stft(const float* stft_input, uint16_t num_frames, float* output, uint16_t output_length, stft_config_t* config);
esp_err_t dsp_free_stft_config(stft_config_t* config);

esp_err_t dsp_compute_cross_spectrum(const float* a_real, const float* a_imag, const float* b_real, const float* b_imag, float* cross_real, float* cross_imag, uint16_t size);
esp_err_t dsp_compute_coherence(const float* signal_a, const float* signal_b, uint16_t length, float* coherence, uint16_t freq_bins);
esp_err_t dsp_compute_phase_difference(const float* signal_a, const float* signal_b, uint16_t length, float* phase_diff);

esp_err_t dsp_pipeline_set_temperature_compensation(dsp_pipeline_context_t* ctx, float temperature_celsius);
esp_err_t dsp_pipeline_set_humidity_compensation(dsp_pipeline_context_t* ctx, float humidity_percent);
esp_err_t dsp_pipeline_update_calibration(dsp_pipeline_context_t* ctx, const frontend_calibration_t* calibration);
esp_err_t dsp_pipeline_get_calibration_status(dsp_pipeline_context_t* ctx, calibration_tracking_t* status);

void dsp_print_pipeline_info(const dsp_pipeline_context_t* ctx);
void dsp_print_statistics(const signal_statistics_t* stats);
void dsp_print_peak_info(const peak_info_t* peaks, uint16_t count);
void dsp_print_processing_status(const processing_status_t* status);
void dsp_print_tgc_config(const tgc_config_t* config);
void dsp_print_mti_config(const mti_config_t* config);
void dsp_print_filter_coefficients(const float* coeffs, uint16_t num_taps);

#ifdef __cplusplus
}
#endif

#endif