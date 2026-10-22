#include "dsp_pipeline.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_dsp.hh"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char* TAG = "DSP_PIPELINE";

static dsp_pipeline_context_t* s_dsp_instances[4] = {NULL};
static uint8_t s_instance_count = 0;

static float s_hilbert_coeffs_default[DSP_HILBERT_TAPS] = {0};
static bool s_hilbert_initialized = false;
static uint16_t s_fft_size_current = 0;

static inline float dsp_abs_float(float x) { return (x < 0) ? -x : x; }

static float dsp_fast_expf(float x) {
    float x2 = x * x;
    float x3 = x2 * x;
    float x4 = x3 * x;
    float x5 = x4 * x;
    return 1.0f + x + x2 / 2.0f + x3 / 6.0f + x4 / 24.0f + x5 / 120.0f;
}

static float dsp_fast_logf(float x) {
    if (x <= 0.0f) return -100.0f;
    int exp;
    float mantissa = frexpf(x, &exp);
    float y = (mantissa - 1.0f) / (mantissa + 1.0f);
    float y2 = y * y;
    return 2.0f * (y + y * y2 * (1.0f/3.0f + y2 * (1.0f/5.0f))) + (float)(exp - 1) * 0.69314718f;
}

esp_err_t dsp_pipeline_init(dsp_pipeline_context_t* ctx, uint8_t instance_id) {
    if (ctx == NULL) {
        ESP_LOGE(TAG, "DSP pipeline context is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (instance_id >= 4) {
        ESP_LOGE(TAG, "Invalid instance ID: %d", instance_id);
        return ESP_ERR_INVALID_ARG;
    }
    if (s_dsp_instances[instance_id] != NULL) {
        ESP_LOGW(TAG, "Instance %d already initialized, deinit first", instance_id);
        return ESP_ERR_INVALID_STATE;
    }

    memset(ctx, 0, sizeof(dsp_pipeline_context_t));

    ctx->instance_id = instance_id;
    ctx->is_initialized = false;
    ctx->buffers.input_size = DSP_MAX_SAMPLES;
    ctx->buffers.output_size = DSP_MAX_SAMPLES;
    ctx->buffers.state = TRACE_STATE_FRESH;

    radar_parameters_t default_params = {
        .sample_count = DSP_MAX_SAMPLES,
        .sample_rate_hz = 100000000.0f,
        .center_frequency_hz = 250000000.0f,
        .bandwidth_hz = 200000000.0f,
        .pulse_width_us = 0.05f,
        .prf_hz = 10000.0f,
        .antenna_delay_ns = 5.0f,
        .velocity_m_per_ns = 0.1f,
        .ground_permittivity = 9.0f,
        .attenuation_db_per_m = 30.0f,
        .noise_floor_dbm = -90.0f,
        .dynamic_range_db = 80.0f
    };
    memcpy(&ctx->radar_params, &default_params, sizeof(radar_parameters_t));

    processing_flags_t default_flags = {
        .enable_dc_removal = true,
        .enable_tgc = true,
        .enable_bandpass = true,
        .enable_mti = true,
        .enable_whitening = false,
        .enable_hilbert = true,
        .enable_fft_analysis = false,
        .enable_normalization = true,
        .enable_peak_detection = true,
        .enable_feature_extraction = true,
        .enable_spectral_analysis = false,
        .enable_wavelet_transform = false,
        .processing_flags = 0xFFFFFFFF
    };
    memcpy(&ctx->flags, &default_flags, sizeof(processing_flags_t));

    tgc_config_t default_tgc = {
        .target_gain_db = 60.0f,
        .attenuation_coefficient_alpha = 0.8f,
        .depth_compensation_factor = 1.2f,
        .saturation_threshold = 3.2f,
        .noise_floor_threshold = 0.01f,
        .tgc_table_size = DSP_MAX_SAMPLES,
        .tgc_lookup_table = NULL,
        .mode = TGC_MODE_EXPONENTIAL,
        .adaptive_mode_enabled = false,
        .adaptation_rate = 0.01f,
        .depth_slices = DSP_DEPTH_RESOLUTION
    };
    memcpy(&ctx->tgc_cfg, &default_tgc, sizeof(tgc_config_t));

    mti_config_t default_mti = {
        .mti_coefficient_beta = 0.85f,
        .mti_threshold = 0.15f,
        .background_update_interval = 8,
        .initialization_samples = 32,
        .convergence_rate = 0.05f,
        .adaptation_factor = 0.02f,
        .adaptive_background = true,
        .mode = MTI_MODE_EWMA,
        .filter_order = 3,
        .reset_counter = 0,
        .background_variance = 0.0f,
        .background_mean = 0.0f
    };
    memcpy(&ctx->mti_cfg, &default_mti, sizeof(mti_config_t));

    peak_detection_config_t default_peak = {
        .lower_threshold = 0.1f,
        .upper_threshold = 3.0f,
        .peak_spacing_min = 5,
        .prominence_threshold = 0.2f,
        .width_threshold_percent = 50.0f,
        .max_peaks = DSP_MAX_PEAKS,
        .mode = PEAK_MODE_LOCAL,
        .enable_validation = true,
        .snr_min_db = 6.0f,
        .valley_threshold_ratio = 0.3f,
        .local_window_radius = 10
    };
    memcpy(&ctx->peak_cfg, &default_peak, sizeof(peak_detection_config_t));

    normalization_config_t default_norm = {
        .threshold_multiplier = 2.5f,
        .percentile_cutoff = 95.0f,
        .robust_scale_factor = 1.4826f,
        .method = NORMALIZATION_MIN_MAX,
        .clip_min = 0.0f,
        .clip_max = 1.0f,
        .auto_scale = true
    };
    memcpy(&ctx->norm_cfg, &default_norm, sizeof(normalization_config_t));

    ctx->status.current_stage = DSP_STAGE_IDLE;
    ctx->status.next_stage = DSP_STAGE_DC_REMOVAL;
    ctx->status.stage_mask = 0xFFF;
    ctx->status.samples_total = DSP_MAX_SAMPLES;
    ctx->status.bypass_enabled = false;
    ctx->status.diagnostic_mode = false;

    ctx->monitor.is_configured = true;
    ctx->monitor.is_running = false;
    ctx->monitor.total_traces_processed = 0;
    ctx->monitor.traces_dropped = 0;
    ctx->monitor.processing_errors = 0;
    ctx->monitor.average_processing_time_us = 0;
    ctx->monitor.max_processing_time_us = 0;
    ctx->monitor.min_processing_time_us = 999999.0f;

    ctx->cal_tracking.calibration_valid = false;
    ctx->cal_tracking.calibration_age_hours = 0;
    ctx->cal_tracking.max_phase_deviation = 5.0f;
    ctx->cal_tracking.amplitude_stability_index = 0.98f;
    ctx->cal_tracking.baseline_drift_rate = 0.01f;
    ctx->cal_tracking.temperature_compensation = 0.0f;
    ctx->cal_tracking.humidity_compensation = 0.0f;

    ctx->pipeline_mutex = xSemaphoreCreateMutex();
    if (ctx->pipeline_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create pipeline mutex");
        return ESP_ERR_NO_MEM;
    }

    s_dsp_instances[instance_id] = ctx;
    s_instance_count++;

    ESP_LOGI(TAG, "DSP pipeline instance %d initialized", instance_id);
    return ESP_OK;
}

esp_err_t dsp_pipeline_allocate_buffers(dsp_pipeline_context_t* ctx, uint16_t max_samples) {
    if (ctx == NULL) return ESP_ERR_INVALID_ARG;

    ctx->buffers.input_buffer = (float*)heap_caps_malloc(max_samples * sizeof(float), MALLOC_CAP_SPIRAM);
    ctx->buffers.output_buffer = (float*)heap_caps_malloc(max_samples * sizeof(float), MALLOC_CAP_SPIRAM);
    ctx->buffers.dc_offset_buffer = (float*)heap_caps_malloc(max_samples * sizeof(float), MALLOC_CAP_SPIRAM);
    ctx->buffers.temp_buffer_a = (float*)heap_caps_malloc(max_samples * sizeof(float), MALLOC_CAP_SPIRAM);
    ctx->buffers.temp_buffer_b = (float*)heap_caps_malloc(max_samples * sizeof(float), MALLOC_CAP_SPIRAM);
    ctx->buffers.fft_real_input = (float*)heap_caps_malloc(DSP_FFT_SIZE_MAX * sizeof(float), MALLOC_CAP_SPIRAM);
    ctx->buffers.fft_imag_input = (float*)heap_caps_malloc(DSP_FFT_SIZE_MAX * sizeof(float), MALLOC_CAP_SPIRAM);
    ctx->buffers.fft_magnitude = (float*)heap_caps_malloc(DSP_FFT_SIZE_MAX * sizeof(float), MALLOC_CAP_SPIRAM);
    ctx->buffers.envelope_output = (float*)heap_caps_malloc(max_samples * sizeof(float), MALLOC_CAP_SPIRAM);
    ctx->buffers.background_model = (float*)heap_caps_malloc(max_samples * sizeof(float), MALLOC_CAP_SPIRAM);
    ctx->buffers.mti_output = (float*)heap_caps_malloc(max_samples * sizeof(float), MALLOC_CAP_SPIRAM);
    ctx->buffers.peak_indices = (float*)heap_caps_malloc(DSP_MAX_PEAKS * sizeof(float), MALLOC_CAP_SPIRAM);
    ctx->buffers.peak_magnitudes = (float*)heap_caps_malloc(DSP_MAX_PEAKS * sizeof(float), MALLOC_CAP_SPIRAM);
    ctx->buffers.feature_vector = (float*)heap_caps_malloc(64 * sizeof(float), MALLOC_CAP_SPIRAM);
    ctx->buffers.spectral_density = (float*)heap_caps_malloc(DSP_SPECTRAL_BINS * sizeof(float), MALLOC_CAP_SPIRAM);
    ctx->buffers.correlation_result = (float*)heap_caps_malloc(DSP_CORRELATION_MAX_LAG * 2 * sizeof(float), MALLOC_CAP_SPIRAM);

    ctx->tgc_cfg.tgc_lookup_table = (float*)heap_caps_malloc(DSP_MAX_SAMPLES * sizeof(float), MALLOC_CAP_SPIRAM);

    if (ctx->buffers.hilbert_coeffs == NULL) {
        ctx->buffers.hilbert_coeffs = (float*)heap_caps_malloc(DSP_HILBERT_TAPS * sizeof(float), MALLOC_CAP_SPIRAM);
    }

    if (!s_hilbert_initialized) {
        for (int i = 0; i < DSP_HILBERT_TAPS; i++) {
            float n = (float)i - (DSP_HILBERT_TAPS - 1) / 2.0f;
            if ((int)n == 0) {
                s_hilbert_coeffs_default[i] = 0.0f;
            } else {
                float h = (1.0f / (float)M_PI) * (1.0f - dsp_fast_expf(-3.14159265f * dsp_abs_float(n) / (float)(DSP_HILBERT_TAPS / 2)));
                if ((i % 2) == 1) {
                    s_hilbert_coeffs_default[i] = h;
                } else {
                    s_hilbert_coeffs_default[i] = h;
                }
            }
        }
        s_hilbert_initialized = true;
    }

    if (ctx->buffers.hilbert_coeffs) {
        memcpy(ctx->buffers.hilbert_coeffs, s_hilbert_coeffs_default, DSP_HILBERT_TAPS * sizeof(float));
    }

    ctx->buffers.input_size = max_samples;
    ctx->buffers.output_size = max_samples;

    for (int i = 0; i < max_samples; i++) {
        if (ctx->buffers.background_model) ctx->buffers.background_model[i] = 0.0f;
    }

    ESP_LOGI(TAG, "DSP buffers allocated: %d samples", max_samples);
    return ESP_OK;
}

esp_err_t dsp_pipeline_free_buffers(dsp_pipeline_context_t* ctx) {
    if (ctx == NULL) return ESP_ERR_INVALID_ARG;

    if (ctx->buffers.input_buffer) { free(ctx->buffers.input_buffer); ctx->buffers.input_buffer = NULL; }
    if (ctx->buffers.output_buffer) { free(ctx->buffers.output_buffer); ctx->buffers.output_buffer = NULL; }
    if (ctx->buffers.dc_offset_buffer) { free(ctx->buffers.dc_offset_buffer); ctx->buffers.dc_offset_buffer = NULL; }
    if (ctx->buffers.temp_buffer_a) { free(ctx->buffers.temp_buffer_a); ctx->buffers.temp_buffer_a = NULL; }
    if (ctx->buffers.temp_buffer_b) { free(ctx->buffers.temp_buffer_b); ctx->buffers.temp_buffer_b = NULL; }
    if (ctx->buffers.fft_real_input) { free(ctx->buffers.fft_real_input); ctx->buffers.fft_real_input = NULL; }
    if (ctx->buffers.fft_imag_input) { free(ctx->buffers.fft_imag_input); ctx->buffers.fft_imag_input = NULL; }
    if (ctx->buffers.fft_magnitude) { free(ctx->buffers.fft_magnitude); ctx->buffers.fft_magnitude = NULL; }
    if (ctx->buffers.hilbert_coeffs) { free(ctx->buffers.hilbert_coeffs); ctx->buffers.hilbert_coeffs = NULL; }
    if (ctx->buffers.envelope_output) { free(ctx->buffers.envelope_output); ctx->buffers.envelope_output = NULL; }
    if (ctx->buffers.background_model) { free(ctx->buffers.background_model); ctx->buffers.background_model = NULL; }
    if (ctx->buffers.mti_output) { free(ctx->buffers.mti_output); ctx->buffers.mti_output = NULL; }
    if (ctx->buffers.peak_indices) { free(ctx->buffers.peak_indices); ctx->buffers.peak_indices = NULL; }
    if (ctx->buffers.peak_magnitudes) { free(ctx->buffers.peak_magnitudes); ctx->buffers.peak_magnitudes = NULL; }
    if (ctx->buffers.feature_vector) { free(ctx->buffers.feature_vector); ctx->buffers.feature_vector = NULL; }
    if (ctx->buffers.spectral_density) { free(ctx->buffers.spectral_density); ctx->buffers.spectral_density = NULL; }
    if (ctx->buffers.correlation_result) { free(ctx->buffers.correlation_result); ctx->buffers.correlation_result = NULL; }
    if (ctx->tgc_cfg.tgc_lookup_table) { free(ctx->tgc_cfg.tgc_lookup_table); ctx->tgc_cfg.tgc_lookup_table = NULL; }

    ESP_LOGI(TAG, "DSP buffers freed");
    return ESP_OK;
}

esp_err_t dsp_pipeline_deinit(dsp_pipeline_context_t* ctx) {
    if (ctx == NULL) return ESP_ERR_INVALID_ARG;

    dsp_pipeline_free_buffers(ctx);

    if (ctx->pipeline_mutex) {
        vSemaphoreDelete(ctx->pipeline_mutex);
        ctx->pipeline_mutex = NULL;
    }

    s_dsp_instances[ctx->instance_id] = NULL;
    s_instance_count--;

    ctx->is_initialized = false;
    ESP_LOGI(TAG, "DSP pipeline instance %d deinitialized", ctx->instance_id);
    return ESP_OK;
}

esp_err_t dsp_pipeline_configure(dsp_pipeline_context_t* ctx, const radar_parameters_t* params) {
    if (ctx == NULL || params == NULL) return ESP_ERR_INVALID_ARG;

    memcpy(&ctx->radar_params, params, sizeof(radar_parameters_t));

    if (ctx->tgc_cfg.tgc_lookup_table == NULL) {
        ctx->tgc_cfg.tgc_lookup_table = (float*)heap_caps_malloc(DSP_MAX_SAMPLES * sizeof(float), MALLOC_CAP_SPIRAM);
    }

    dsp_generate_tgc_lookup_table(&ctx->tgc_cfg, params->sample_rate_hz, 2.0f, params->velocity_m_per_ns);

    ctx->is_initialized = true;
    ESP_LOGI(TAG, "DSP pipeline configured: Fs=%.1fMHz, Fc=%.1fMHz, BW=%.1fMHz",
             params->sample_rate_hz / 1e6, params->center_frequency_hz / 1e6, params->bandwidth_hz / 1e6);
    return ESP_OK;
}

esp_err_t dsp_pipeline_reset(dsp_pipeline_context_t* ctx) {
    if (ctx == NULL) return ESP_ERR_INVALID_ARG;

    ctx->status.current_stage = DSP_STAGE_IDLE;
    ctx->status.next_stage = DSP_STAGE_DC_REMOVAL;
    ctx->status.samples_processed = 0;
    ctx->status.stage_elapsed_ms = 0;
    ctx->status.total_elapsed_ms = 0;
    ctx->status.pipeline_errors = 0;

    ctx->buffers.state = TRACE_STATE_FRESH;
    ctx->peak_count = 0;

    memset(&ctx->stats, 0, sizeof(signal_statistics_t));

    if (ctx->buffers.background_model) {
        for (int i = 0; i < ctx->buffers.input_size; i++) {
            ctx->buffers.background_model[i] = 0.0f;
        }
    }

    ctx->mti_cfg.reset_counter = 0;
    ctx->monitor.pipeline_busy = false;
    ctx->monitor.overflow_detected = false;

    return ESP_OK;
}

esp_err_t dsp_pipeline_remove_dc_offset(dsp_pipeline_context_t* ctx, const float* input, float* output, uint16_t length) {
    if (ctx == NULL || input == NULL || output == NULL) return ESP_ERR_INVALID_ARG;
    if (length == 0 || length > DSP_MAX_SAMPLES) return ESP_ERR_INVALID_SIZE;

    ctx->status.current_stage = DSP_STAGE_DC_REMOVAL;

    float sum = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        sum += input[i];
    }
    float dc_offset = sum / length;

    for (uint16_t i = 0; i < length; i++) {
        output[i] = input[i] - dc_offset;
    }

    ctx->stats.mean = dc_offset;

    return ESP_OK;
}

esp_err_t dsp_pipeline_apply_tgc(dsp_pipeline_context_t* ctx, const float* input, float* output, uint16_t length) {
    if (ctx == NULL || input == NULL || output == NULL) return ESP_ERR_INVALID_ARG;
    if (ctx->tgc_cfg.tgc_lookup_table == NULL) return ESP_ERR_INVALID_STATE;

    ctx->status.current_stage = DSP_STAGE_TGC_COMPENSATION;

    float* tgc_lut = ctx->tgc_cfg.tgc_lookup_table;

    if (ctx->tgc_cfg.mode == TGC_MODE_EXPONENTIAL) {
        float alpha = ctx->tgc_cfg.attenuation_coefficient_alpha;
        float A = ctx->tgc_cfg.target_gain_db / 20.0f;
        for (uint16_t i = 0; i < length; i++) {
            float depth_factor = (float)i / (float)length;
            float gain = A * dsp_fast_expf(alpha * depth_factor * 5.0f);
            float gain_linear = dsp_fast_expf(gain);
            if (gain_linear > 50.0f) gain_linear = 50.0f;
            output[i] = input[i] * gain_linear;
        }
    } else if (ctx->tgc_cfg.mode == TGC_MODE_LINEAR) {
        float max_gain = ctx->tgc_cfg.target_gain_db / 20.0f;
        max_gain = dsp_fast_expf(max_gain);
        for (uint16_t i = 0; i < length; i++) {
            float depth_factor = (float)i / (float)length;
            float gain = 1.0f + (max_gain - 1.0f) * depth_factor;
            output[i] = input[i] * gain;
        }
    } else if (ctx->tgc_cfg.mode == TGC_MODE_SEGMENTED) {
        uint16_t segment_size = length / 4;
        float segment_gains[4] = {1.0f, 2.5f, 6.0f, 12.0f};
        for (uint16_t i = 0; i < length; i++) {
            uint16_t seg = (i / segment_size > 3) ? 3 : i / segment_size;
            float local_pos = (float)(i % segment_size) / (float)segment_size;
            float gain = segment_gains[seg] * (1.0f + local_pos * 0.5f);
            output[i] = input[i] * gain;
        }
    } else if (ctx->tgc_cfg.mode == TGC_MODE_ADAPTIVE) {
        float current_snr = ctx->stats.snr_estimate;
        float target_snr = 15.0f;
        float adaptation = ctx->tgc_cfg.adaptation_rate * (target_snr - current_snr);
        for (uint16_t i = 0; i < length; i++) {
            float depth_factor = (float)i / (float)length;
            float base_gain = 1.0f + depth_factor * 10.0f;
            float adaptive_gain = base_gain * (1.0f + adaptation * 0.1f);
            output[i] = input[i] * adaptive_gain;
        }
    } else {
        for (uint16_t i = 0; i < length; i++) {
            output[i] = input[i];
        }
    }

    return ESP_OK;
}

esp_err_t dsp_pipeline_apply_mti_filter(dsp_pipeline_context_t* ctx, const float* input, float* output, uint16_t length) {
    if (ctx == NULL || input == NULL || output == NULL) return ESP_ERR_INVALID_ARG;

    ctx->status.current_stage = DSP_STAGE_MTI_FILTER;

    if (ctx->mti_cfg.mode == MTI_MODE_NONE) {
        memcpy(output, input, length * sizeof(float));
        return ESP_OK;
    }

    if (ctx->mti_cfg.mode == MTI_MODE_EWMA) {
        float beta = ctx->mti_cfg.mti_coefficient_beta;
        float one_minus_beta = 1.0f - beta;

        if (ctx->mti_cfg.reset_counter < ctx->mti_cfg.initialization_samples) {
            for (uint16_t i = 0; i < length; i++) {
                float bg = ctx->buffers.background_model[i];
                ctx->buffers.background_model[i] = beta * input[i] + one_minus_beta * bg;
                output[i] = input[i] - bg;
            }
            ctx->mti_cfg.reset_counter += length;
        } else {
            for (uint16_t i = 0; i < length; i++) {
                float bg = ctx->buffers.background_model[i];
                ctx->buffers.background_model[i] = beta * input[i] + one_minus_beta * bg;
                output[i] = input[i] - bg;
            }
        }

        float bg_mean = 0.0f;
        float bg_var = 0.0f;
        for (uint16_t i = 0; i < length; i++) {
            bg_mean += ctx->buffers.background_model[i];
        }
        bg_mean /= length;
        for (uint16_t i = 0; i < length; i++) {
            float diff = ctx->buffers.background_model[i] - bg_mean;
            bg_var += diff * diff;
        }
        bg_var /= length;
        ctx->mti_cfg.background_variance = bg_var;
        ctx->mti_cfg.background_mean = bg_mean;
    }
    else if (ctx->mti_cfg.mode == MTI_MODE_SUBTRACTION) {
        for (uint16_t i = 0; i < length; i++) {
            float bg = ctx->buffers.background_model[i];
            output[i] = input[i] - bg;
            float alpha = ctx->mti_cfg.convergence_rate;
            ctx->buffers.background_model[i] = alpha * input[i] + (1.0f - alpha) * bg;
        }
    }
    else if (ctx->mti_cfg.mode == MTI_MODE_HIGH_PASS) {
        float prev = 0.0f;
        float alpha_hp = 0.9f;
        for (uint16_t i = 0; i < length; i++) {
            output[i] = alpha_hp * (prev + input[i] - (i > 0 ? input[i-1] : 0.0f));
            prev = output[i];
        }
    }
    else if (ctx->mti_cfg.mode == MTI_MODE_ALPHA_TRIMMED) {
        uint8_t trim = 2;
        for (uint16_t i = trim; i < length - trim; i++) {
            float sorted_window[5] = {0};
            for (uint8_t j = 0; j < 5; j++) {
                sorted_window[j] = input[i - trim + j];
            }
            for (uint8_t m = 0; m < 5; m++) {
                for (uint8_t n = m + 1; n < 5; n++) {
                    if (sorted_window[m] > sorted_window[n]) {
                        float tmp = sorted_window[m];
                        sorted_window[m] = sorted_window[n];
                        sorted_window[n] = tmp;
                    }
                }
            }
            float median = sorted_window[2];
            output[i] = input[i] - median;
        }
    }
    else if (ctx->mti_cfg.mode == MTI_MODE_MEDIAN) {
        uint8_t win = 5;
        for (uint16_t i = win/2; i < length - win/2; i++) {
            float window[5];
            for (uint8_t j = 0; j < win; j++) {
                window[j] = input[i - win/2 + j];
            }
            for (uint8_t m = 0; m < win; m++) {
                for (uint8_t n = m + 1; n < win; n++) {
                    if (window[m] > window[n]) {
                        float tmp = window[m];
                        window[m] = window[n];
                        window[n] = tmp;
                    }
                }
            }
            output[i] = input[i] - window[2];
        }
    }

    return ESP_OK;
}

esp_err_t dsp_pipeline_compute_hilbert_envelope(dsp_pipeline_context_t* ctx, const float* input, float* envelope, uint16_t length) {
    if (ctx == NULL || input == NULL || envelope == NULL) return ESP_ERR_INVALID_ARG;
    if (ctx->buffers.hilbert_coeffs == NULL) return ESP_ERR_INVALID_STATE;

    ctx->status.current_stage = DSP_STAGE_HILBERT_ENVELOPE;

    uint16_t N = length;
    uint16_t padded_len = 1;
    while (padded_len < N * 2) padded_len *= 2;

    float* padded_real = (float*)heap_caps_malloc(padded_len * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
    float* padded_imag = (float*)heap_caps_malloc(padded_len * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
    float* fft_out_real = (float*)heap_caps_malloc(padded_len * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
    float* fft_out_imag = (float*)heap_caps_malloc(padded_len * 2 * sizeof(float), MALLOC_CAP_SPIRAM);

    if (!padded_real || !padded_imag || !fft_out_real || !fft_out_imag) {
        if (padded_real) free(padded_real);
        if (padded_imag) free(padded_imag);
        if (fft_out_real) free(fft_out_real);
        if (fft_out_imag) free(fft_out_imag);
        return ESP_ERR_NO_MEM;
    }

    for (uint16_t i = 0; i < padded_len * 2; i++) {
        padded_real[i] = 0.0f;
        padded_imag[i] = 0.0f;
        fft_out_real[i] = 0.0f;
        fft_out_imag[i] = 0.0f;
    }

    for (uint16_t i = 0; i < N; i++) {
        padded_real[i] = input[i];
    }

    for (uint16_t i = 0; i < padded_len * 2; i++) {
        float window_val = 0.5f - 0.5f * cosf(2.0f * 3.14159265f * i / (padded_len * 2 - 1));
        padded_real[i] *= window_val;
    }

    uint16_t fft_n = padded_len * 2;
    dsps_fft2r_fc32((float*)padded_real, (float*)padded_imag, fft_n);

    uint16_t half_len = fft_n / 2;
    for (uint16_t i = 1; i < half_len; i++) {
        fft_out_real[i] = 2.0f * padded_real[i];
        fft_out_real[fft_n - 1 - i] = 0.0f;
        fft_out_imag[i] = 2.0f * padded_imag[i];
        fft_out_imag[fft_n - 1 - i] = 0.0f;
    }
    fft_out_real[0] = padded_real[0];
    fft_out_real[half_len] = padded_real[half_len];
    fft_out_imag[0] = 0.0f;
    fft_out_imag[half_len] = 0.0f;

    dsps_fft2r_inv_fc32((float*)fft_out_real, (float*)fft_out_imag, fft_n);

    for (uint16_t i = 0; i < N; i++) {
        float real_part = fft_out_real[i] / (float)(fft_n);
        float imag_part = fft_out_imag[i] / (float)(fft_n);
        envelope[i] = sqrtf(real_part * real_part + imag_part * imag_part);
    }

    uint8_t smooth_window = DSP_ENVELOPE_SMOOTH_WINDOW;
    for (uint16_t i = 0; i < N; i++) {
        float sum = 0.0f;
        uint16_t count = 0;
        for (uint8_t j = 0; j < smooth_window; j++) {
            int16_t idx = (int16_t)i - (smooth_window / 2) + j;
            if (idx >= 0 && idx < N) {
                sum += envelope[idx];
                count++;
            }
        }
        ctx->buffers.envelope_output[i] = sum / (float)count;
    }
    memcpy(envelope, ctx->buffers.envelope_output, N * sizeof(float));

    free(padded_real);
    free(padded_imag);
    free(fft_out_real);
    free(fft_out_imag);

    return ESP_OK;
}

esp_err_t dsp_pipeline_detect_peaks(dsp_pipeline_context_t* ctx, const float* signal, uint16_t length) {
    if (ctx == NULL || signal == NULL) return ESP_ERR_INVALID_ARG;

    ctx->status.current_stage = DSP_STAGE_PEAK_DETECTION;
    ctx->peak_count = 0;

    if (ctx->peak_cfg.mode == PEAK_MODE_LOCAL) {
        return dsp_detect_peaks_local(signal, length, &ctx->peak_cfg, ctx->peaks, &ctx->peak_count);
    } else if (ctx->peak_cfg.mode == PEAK_MODE_GLOBAL) {
        return dsp_detect_peaks_global(signal, length, &ctx->peak_cfg, ctx->peaks, &ctx->peak_count);
    } else {
        return dsp_detect_peaks_adaptive(signal, length, &ctx->peak_cfg, ctx->peaks, &ctx->peak_count);
    }
}

esp_err_t dsp_detect_peaks_local(const float* signal, uint16_t length, const peak_detection_config_t* config, peak_info_t* peaks, uint16_t* peak_count) {
    if (!signal || !config || !peaks || !peak_count) return ESP_ERR_INVALID_ARG;

    *peak_count = 0;
    uint8_t radius = config->local_window_radius;
    float threshold = config->lower_threshold;
    uint16_t min_spacing = config->peak_spacing_min;

    float signal_mean = 0.0f;
    for (uint16_t i = 0; i < length; i++) signal_mean += signal[i];
    signal_mean /= length;

    float signal_std = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        float diff = signal[i] - signal_mean;
        signal_std += diff * diff;
    }
    signal_std = sqrtf(signal_std / length);
    float dynamic_threshold = signal_mean + threshold * signal_std;

    uint16_t temp_indices[DSP_MAX_PEAKS] = {0};
    uint16_t temp_count = 0;

    for (uint16_t i = radius; i < length - radius && temp_count < DSP_MAX_PEAKS; i++) {
        bool is_peak = true;
        for (uint8_t j = 0; j < radius; j++) {
            if (signal[i] <= signal[i - j - 1] || signal[i] <= signal[i + j + 1]) {
                is_peak = false;
                break;
            }
        }
        if (is_peak && signal[i] > dynamic_threshold) {
            temp_indices[temp_count++] = i;
        }
    }

    for (uint16_t i = 0; i < temp_count; i++) {
        if (*peak_count >= config->max_peaks) break;

        bool skip = false;
        for (uint16_t j = 0; j < *peak_count; j++) {
            if (dsp_abs_float((float)temp_indices[i] - peaks[j].peak_index) < min_spacing) {
                if (signal[temp_indices[i]] > peaks[j].peak_value) {
                    *peak_count = j;
                } else {
                    skip = true;
                }
                break;
            }
        }
        if (skip) continue;

        uint16_t idx = temp_indices[i];
        float left_val = (idx > 0) ? signal[idx - 1] : signal[idx];
        float right_val = (idx < length - 1) ? signal[idx + 1] : signal[idx];

        uint16_t left_idx = idx;
        while (left_idx > 0 && signal[left_idx] > left_val) left_idx--;
        uint16_t right_idx = idx;
        while (right_idx < length - 1 && signal[right_idx] > right_val) right_idx++;

        float peak_width = (float)(right_idx - left_idx);

        float left_min = signal[left_idx];
        float right_min = signal[right_idx];
        float min_val = (left_min < right_min) ? left_min : right_min;
        float prominence = signal[idx] - min_val;

        if (prominence < config->prominence_threshold * signal_std) continue;

        float interpolated_idx = (float)idx;
        if (idx > 0 && idx < length - 1) {
            float alpha = signal[idx + 1] - signal[idx - 1];
            float beta = 2.0f * (2.0f * signal[idx] - signal[idx - 1] - signal[idx + 1]);
            if (dsp_abs_float(beta) > 1e-6f) {
                interpolated_idx = (float)idx + alpha / beta;
            }
        }

        peaks[*peak_count].peak_value = signal[idx];
        peaks[*peak_count].peak_index = idx;
        peaks[*peak_count].peak_width = peak_width;
        peaks[*peak_count].peak_area = peak_width * signal[idx];
        peaks[*peak_count].peak_snr = 20.0f * log10f((signal[idx] + 1e-10f) / (signal_std + 1e-10f));
        peaks[*peak_count].peak_prominence = prominence;
        peaks[*peak_count].is_valid = true;
        peaks[*peak_count].peak_type = 0;

        (*peak_count)++;
    }

    for (uint16_t i = 0; i < *peak_count; i++) {
        for (uint16_t j = i + 1; j < *peak_count; j++) {
            if (peaks[j].peak_value > peaks[i].peak_value) {
                peak_info_t tmp = peaks[i];
                peaks[i] = peaks[j];
                peaks[j] = tmp;
            }
        }
    }

    return ESP_OK;
}

esp_err_t dsp_detect_peaks_global(const float* signal, uint16_t length, const peak_detection_config_t* config, peak_info_t* peaks, uint16_t* peak_count) {
    if (!signal || !config || !peaks || !peak_count) return ESP_ERR_INVALID_ARG;

    *peak_count = 0;

    float max_val = signal[0];
    float min_val = signal[0];
    for (uint16_t i = 1; i < length; i++) {
        if (signal[i] > max_val) max_val = signal[i];
        if (signal[i] < min_val) min_val = signal[i];
    }

    float range = max_val - min_val;
    float global_threshold = min_val + range * config->threshold_multiplier / 100.0f;

    for (uint16_t i = 1; i < length - 1; i++) {
        if (signal[i] >= global_threshold &&
            signal[i] > signal[i - 1] &&
            signal[i] > signal[i + 1]) {

            if (*peak_count >= config->max_peaks) break;

            peaks[*peak_count].peak_value = signal[i];
            peaks[*peak_count].peak_index = i;
            peaks[*peak_count].peak_width = 1.0f;
            peaks[*peak_count].peak_snr = 20.0f * log10f((signal[i] + 1e-10f) / (min_val + 1e-10f));
            peaks[*peak_count].is_valid = true;
            peaks[*peak_count].peak_type = 1;

            uint16_t left = i;
            while (left > 0 && signal[left] > global_threshold) left--;
            uint16_t right = i;
            while (right < length - 1 && signal[right] > global_threshold) right++;
            peaks[*peak_count].peak_width = (float)(right - left);

            (*peak_count)++;
        }
    }

    return ESP_OK;
}

esp_err_t dsp_detect_peaks_adaptive(const float* signal, uint16_t length, const peak_detection_config_t* config, peak_info_t* peaks, uint16_t* peak_count) {
    if (!signal || !config || !peaks || !peak_count) return ESP_ERR_INVALID_ARG;

    *peak_count = 0;

    float sorted_signal[DSP_MAX_SAMPLES];
    memcpy(sorted_signal, signal, length * sizeof(float));
    for (uint16_t i = 0; i < length; i++) {
        for (uint16_t j = i + 1; j < length; j++) {
            if (sorted_signal[i] > sorted_signal[j]) {
                float tmp = sorted_signal[i];
                sorted_signal[i] = sorted_signal[j];
                sorted_signal[j] = tmp;
            }
        }
    }

    float q1_val = sorted_signal[length / 4];
    float q3_val = sorted_signal[3 * length / 4];
    float iqr = q3_val - q1_val;
    float mad_threshold = q1_val - 1.5f * iqr;

    float median = sorted_signal[length / 2];

    for (uint16_t i = 1; i < length - 1; i++) {
        if (signal[i] < mad_threshold || signal[i] <= median) continue;

        float local_mean = 0.0f;
        float local_std = 0.0f;
        uint16_t local_count = 0;
        uint16_t start = (i > 10) ? i - 10 : 0;
        uint16_t end = (i + 10 < length) ? i + 10 : length;
        for (uint16_t j = start; j < end; j++) {
            if (j != i) {
                local_mean += signal[j];
                local_count++;
            }
        }
        local_mean /= local_count;
        for (uint16_t j = start; j < end; j++) {
            if (j != i) {
                float diff = signal[j] - local_mean;
                local_std += diff * diff;
            }
        }
        local_std = sqrtf(local_std / local_count);

        float adaptive_threshold = local_mean + 2.0f * local_std;

        if (signal[i] >= adaptive_threshold &&
            signal[i] > signal[i - 1] &&
            signal[i] > signal[i + 1]) {

            if (*peak_count >= config->max_peaks) break;

            peaks[*peak_count].peak_value = signal[i];
            peaks[*peak_count].peak_index = i;
            peaks[*peak_count].peak_width = 1.0f;
            peaks[*peak_count].peak_snr = 20.0f * log10f((signal[i] + 1e-10f) / (local_std + 1e-10f));
            peaks[*peak_count].is_valid = true;
            peaks[*peak_count].peak_type = 2;

            (*peak_count)++;
        }
    }

    return ESP_OK;
}

esp_err_t dsp_pipeline_normalize(dsp_pipeline_context_t* ctx, const float* input, float* output, uint16_t length) {
    if (ctx == NULL || input == NULL || output == NULL) return ESP_ERR_INVALID_ARG;

    ctx->status.current_stage = DSP_STAGE_NORMALIZATION;

    if (ctx->norm_cfg.method == NORMALIZATION_MIN_MAX) {
        return dsp_normalize_min_max(input, output, length, ctx->norm_cfg.clip_min, ctx->norm_cfg.clip_max);
    } else if (ctx->norm_cfg.method == NORMALIZATION_Z_SCORE) {
        return dsp_normalize_z_score(input, output, length);
    } else if (ctx->norm_cfg.method == NORMALIZATION_ROBUST) {
        return dsp_normalize_robust(input, output, length, ctx->norm_cfg.percentile_cutoff);
    } else if (ctx->norm_cfg.method == NORMALIZATION_PERCENTILE) {
        return dsp_normalize_percentile(input, output, length, 5.0f, 95.0f);
    }

    memcpy(output, input, length * sizeof(float));
    return ESP_OK;
}

esp_err_t dsp_normalize_min_max(const float* input, float* output, uint16_t length, float target_min, float target_max) {
    if (!input || !output) return ESP_ERR_INVALID_ARG;

    float data_min = input[0];
    float data_max = input[0];
    for (uint16_t i = 1; i < length; i++) {
        if (input[i] < data_min) data_min = input[i];
        if (input[i] > data_max) data_max = input[i];
    }

    float data_range = data_max - data_min;
    if (data_range < 1e-10f) {
        memset(output, target_min, length * sizeof(float));
        return ESP_OK;
    }

    float target_range = target_max - target_min;
    for (uint16_t i = 0; i < length; i++) {
        float normalized = (input[i] - data_min) / data_range;
        output[i] = target_min + normalized * target_range;
    }

    return ESP_OK;
}

esp_err_t dsp_normalize_z_score(const float* input, float* output, uint16_t length) {
    if (!input || !output) return ESP_ERR_INVALID_ARG;

    float mean = 0.0f;
    for (uint16_t i = 0; i < length; i++) mean += input[i];
    mean /= length;

    float variance = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        float diff = input[i] - mean;
        variance += diff * diff;
    }
    variance /= length;
    float std_dev = sqrtf(variance);

    if (std_dev < 1e-10f) {
        memset(output, 0.0f, length * sizeof(float));
        return ESP_OK;
    }

    for (uint16_t i = 0; i < length; i++) {
        output[i] = (input[i] - mean) / std_dev;
    }

    return ESP_OK;
}

esp_err_t dsp_normalize_robust(const float* input, float* output, uint16_t length, float percentile) {
    if (!input || !output) return ESP_ERR_INVALID_ARG;

    float sorted[DSP_MAX_SAMPLES];
    memcpy(sorted, input, length * sizeof(float));
    for (uint16_t i = 0; i < length; i++) {
        for (uint16_t j = i + 1; j < length; j++) {
            if (sorted[i] > sorted[j]) {
                float tmp = sorted[i]; sorted[i] = sorted[j]; sorted[j] = tmp;
            }
        }
    }

    uint16_t lower_idx = (uint16_t)(length * percentile / 100.0f);
    uint16_t upper_idx = (uint16_t)(length * (100.0f - percentile) / 100.0f);
    if (upper_idx >= length) upper_idx = length - 1;
    if (lower_idx >= length) lower_idx = 0;

    float q1 = sorted[lower_idx];
    float q3 = sorted[upper_idx];
    float iqr = q3 - q1;
    float median = sorted[length / 2];

    float mad = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        mad += dsp_abs_float(input[i] - median);
    }
    mad /= length;

    float scale_factor = 1.4826f * mad;
    if (scale_factor < 1e-10f) scale_factor = 1.0f;

    for (uint16_t i = 0; i < length; i++) {
        output[i] = (input[i] - median) / scale_factor;
        if (output[i] > 3.0f) output[i] = 3.0f;
        if (output[i] < -3.0f) output[i] = -3.0f;
    }

    return ESP_OK;
}

esp_err_t dsp_normalize_percentile(const float* input, float* output, uint16_t length, float lower_percentile, float upper_percentile) {
    if (!input || !output) return ESP_ERR_INVALID_ARG;

    float sorted[DSP_MAX_SAMPLES];
    memcpy(sorted, input, length * sizeof(float));
    for (uint16_t i = 0; i < length; i++) {
        for (uint16_t j = i + 1; j < length; j++) {
            if (sorted[i] > sorted[j]) {
                float tmp = sorted[i]; sorted[i] = sorted[j]; sorted[j] = tmp;
            }
        }
    }

    uint16_t lower_idx = (uint16_t)(length * lower_percentile / 100.0f);
    uint16_t upper_idx = (uint16_t)(length * upper_percentile / 100.0f);
    if (upper_idx >= length) upper_idx = length - 1;
    if (lower_idx >= length) lower_idx = 0;

    float lower_val = sorted[lower_idx];
    float upper_val = sorted[upper_idx];
    float range = upper_val - lower_val;

    if (range < 1e-10f) {
        memset(output, 0.5f, length * sizeof(float));
        return ESP_OK;
    }

    for (uint16_t i = 0; i < length; i++) {
        float val = (input[i] - lower_val) / range;
        output[i] = (val < 0.0f) ? 0.0f : (val > 1.0f ? 1.0f : val);
    }

    return ESP_OK;
}

esp_err_t dsp_pipeline_process(dsp_pipeline_context_t* ctx, const float* input, float* output, uint16_t length) {
    if (ctx == NULL || input == NULL || output == NULL) return ESP_ERR_INVALID_ARG;
    if (!ctx->is_initialized) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(ctx->pipeline_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    uint64_t start_time = esp_timer_get_time();

    ctx->monitor.is_running = true;
    ctx->monitor.pipeline_busy = true;

    ctx->buffers.state = TRACE_STATE_PROCESSING;
    memcpy(ctx->buffers.input_buffer, input, length * sizeof(float));

    float* current_data = ctx->buffers.temp_buffer_a;
    float* next_data = ctx->buffers.temp_buffer_b;
    memcpy(current_data, input, length * sizeof(float));

    if (ctx->flags.enable_dc_removal && !ctx->status.bypass_enabled) {
        dsp_pipeline_remove_dc_offset(ctx, current_data, next_data, length);
        float* tmp = current_data; current_data = next_data; next_data = tmp;
        ctx->status.stage_mask |= (1 << DSP_STAGE_DC_REMOVAL);
    }

    if (ctx->flags.enable_tgc && !ctx->status.bypass_enabled) {
        dsp_pipeline_apply_tgc(ctx, current_data, next_data, length);
        float* tmp = current_data; current_data = next_data; next_data = tmp;
        ctx->status.stage_mask |= (1 << DSP_STAGE_TGC_COMPENSATION);
    }

    if (ctx->flags.enable_bandpass && !ctx->status.bypass_enabled) {
        dsp_pipeline_apply_bandpass_filter(ctx, current_data, next_data, length);
        float* tmp = current_data; current_data = next_data; next_data = tmp;
        ctx->status.stage_mask |= (1 << DSP_STAGE_BANDPASS_FILTER);
    }

    if (ctx->flags.enable_mti && !ctx->status.bypass_enabled) {
        dsp_pipeline_apply_mti_filter(ctx, current_data, next_data, length);
        float* tmp = current_data; current_data = next_data; next_data = tmp;
        ctx->status.stage_mask |= (1 << DSP_STAGE_MTI_FILTER);
    }

    if (ctx->flags.enable_hilbert && !ctx->status.bypass_enabled) {
        dsp_pipeline_compute_hilbert_envelope(ctx, current_data, next_data, length);
        float* tmp = current_data; current_data = next_data; next_data = tmp;
        ctx->status.stage_mask |= (1 << DSP_STAGE_HILBERT_ENVELOPE);
    }

    if (ctx->flags.enable_peak_detection && !ctx->status.bypass_enabled) {
        dsp_pipeline_detect_peaks(ctx, current_data, length);
        ctx->status.stage_mask |= (1 << DSP_STAGE_PEAK_DETECTION);
    }

    if (ctx->flags.enable_normalization && !ctx->status.bypass_enabled) {
        dsp_pipeline_normalize(ctx, current_data, next_data, length);
        float* tmp = current_data; current_data = next_data; next_data = tmp;
        ctx->status.stage_mask |= (1 << DSP_STAGE_NORMALIZATION);
    }

    memcpy(output, current_data, length * sizeof(float));
    ctx->buffers.output_size = length;

    dsp_compute_signal_statistics(current_data, length, &ctx->stats);

    ctx->status.current_stage = DSP_STAGE_COMPLETE;
    ctx->status.samples_processed = length;
    ctx->buffers.state = TRACE_STATE_DONE;

    uint64_t end_time = esp_timer_get_time();
    float elapsed_us = (float)(end_time - start_time);
    ctx->status.total_elapsed_ms = elapsed_us / 1000.0f;

    ctx->monitor.total_traces_processed++;
    ctx->monitor.average_processing_time_us =
        (ctx->monitor.average_processing_time_us * (ctx->monitor.total_traces_processed - 1) + elapsed_us)
        / ctx->monitor.total_traces_processed;
    if (elapsed_us > ctx->monitor.max_processing_time_us) ctx->monitor.max_processing_time_us = elapsed_us;
    if (elapsed_us < ctx->monitor.min_processing_time_us) ctx->monitor.min_processing_time_us = elapsed_us;

    ctx->monitor.pipeline_busy = false;
    ctx->monitor.is_running = false;

    xSemaphoreGive(ctx->pipeline_mutex);

    return ESP_OK;
}

esp_err_t dsp_pipeline_apply_bandpass_filter(dsp_pipeline_context_t* ctx, const float* input, float* output, uint16_t length) {
    if (ctx == NULL || input == NULL || output == NULL) return ESP_ERR_INVALID_ARG;

    ctx->status.current_stage = DSP_STAGE_BANDPASS_FILTER;

    float fc_low_norm = ctx->radar_params.center_frequency_hz / ctx->radar_params.sample_rate_hz;
    float fc_high_norm = (ctx->radar_params.center_frequency_hz + ctx->radar_params.bandwidth_hz / 2.0f) / ctx->radar_params.sample_rate_hz;

    if (fc_low_norm < 0.01f) fc_low_norm = 0.01f;
    if (fc_high_norm > 0.45f) fc_high_norm = 0.45f;

    uint16_t filter_order = 64;
    float* fir_coeffs = (float*)heap_caps_malloc(filter_order * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!fir_coeffs) return ESP_ERR_NO_MEM;

    dsp_design_bandpass_filter(fir_coeffs, &filter_order, fc_low_norm, fc_high_norm,
                                ctx->radar_params.sample_rate_hz, filter_order, WINDOW_TYPE_HANNING);

    dsp_fir_filter_execute(input, output, length, fir_coeffs, filter_order);

    free(fir_coeffs);
    return ESP_OK;
}

esp_err_t dsp_design_bandpass_filter(float* coefficients, uint16_t* num_taps, float low_cutoff, float high_cutoff,
                                      float sample_rate, uint16_t order, window_type_t window_type) {
    if (!coefficients || !num_taps) return ESP_ERR_INVALID_ARG;

    uint16_t n = *num_taps;
    float wc_low = 2.0f * 3.14159265f * low_cutoff;
    float wc_high = 2.0f * 3.14159265f * high_cutoff;

    for (uint16_t i = 0; i < n; i++) {
        float m = (float)i - (float)(n - 1) / 2.0f;
        if ((int)m == 0) {
            coefficients[i] = (wc_high - wc_low) / 3.14159265f;
        } else {
            coefficients[i] = ( sinf(wc_high * m) / m - sinf(wc_low * m) / m ) / 3.14159265f;
        }
    }

    float* window = (float*)heap_caps_malloc(n * sizeof(float), MALLOC_CAP_SPIRAM);
    if (window) {
        for (uint16_t i = 0; i < n; i++) {
            float n_val = (float)i / (float)(n - 1);
            if (window_type == WINDOW_TYPE_HANNING) {
                window[i] = 0.5f - 0.5f * cosf(2.0f * 3.14159265f * n_val);
            } else if (window_type == WINDOW_TYPE_HAMMING) {
                window[i] = 0.54f - 0.46f * cosf(2.0f * 3.14159265f * n_val);
            } else if (window_type == WINDOW_TYPE_BLACKMAN) {
                window[i] = 0.42f - 0.5f * cosf(2.0f * 3.14159265f * n_val) + 0.08f * cosf(4.0f * 3.14159265f * n_val);
            } else {
                window[i] = 1.0f;
            }
            coefficients[i] *= window[i];
        }
        free(window);
    }

    return ESP_OK;
}

esp_err_t dsp_fir_filter_execute(const float* input, float* output, uint16_t length, const float* coefficients, uint16_t num_taps) {
    if (!input || !output || !coefficients) return ESP_ERR_INVALID_ARG;

    uint16_t delay = num_taps / 2;
    for (uint16_t i = 0; i < length; i++) {
        float sum = 0.0f;
        uint16_t start = (i >= delay) ? (i - delay) : 0;
        uint16_t end = (i + delay < length) ? (i + delay) : length;
        for (uint16_t j = start; j < end && (j + num_taps > i + delay); j++) {
            int16_t coeff_idx = (int16_t)(i + delay - j);
            if (coeff_idx >= 0 && coeff_idx < num_taps) {
                sum += input[j] * coefficients[coeff_idx];
            }
        }
        output[i] = sum;
    }

    return ESP_OK;
}

esp_err_t dsp_fir_filter_execute_block(const float* input, float* output, uint16_t block_size, const float* coefficients, uint16_t num_taps) {
    return dsp_fir_filter_execute(input, output, block_size, coefficients, num_taps);
}

float dsp_compute_signal_statistics(const float* data, uint16_t length, signal_statistics_t* stats) {
    if (!data || !stats || length == 0) return 0.0f;

    float sum = 0.0f;
    for (uint16_t i = 0; i < length; i++) sum += data[i];
    stats->mean = sum / length;

    float sq_sum = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        float diff = data[i] - stats->mean;
        sq_sum += diff * diff;
    }
    stats->variance = sq_sum / length;
    stats->std_dev = sqrtf(stats->variance);

    stats->rms = sqrtf(sq_sum / length);

    float max_val = data[0];
    float min_val = data[0];
    for (uint16_t i = 1; i < length; i++) {
        if (data[i] > max_val) max_val = data[i];
        if (data[i] < min_val) min_val = data[i];
    }
    stats->peak_to_peak = max_val - min_val;
    stats->crest_factor = (stats->rms > 1e-10f) ? (max_val / stats->rms) : 0.0f;

    uint16_t zero_crossings = 0;
    for (uint16_t i = 1; i < length; i++) {
        if ((data[i-1] >= 0 && data[i] < 0) || (data[i-1] < 0 && data[i] >= 0)) {
            zero_crossings++;
        }
    }
    stats->zero_crossing_rate = (float)zero_crossings / (float)length;

    float skew_sum = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        float z = (stats->std_dev > 1e-10f) ? ((data[i] - stats->mean) / stats->std_dev) : 0.0f;
        skew_sum += z * z * z;
    }
    stats->skewness = skew_sum / length;

    float kurt_sum = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        float z = (stats->std_dev > 1e-10f) ? ((data[i] - stats->mean) / stats->std_dev) : 0.0f;
        kurt_sum += z * z * z * z;
    }
    stats->kurtosis = kurt_sum / length - 3.0f;

    float energy = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        energy += data[i] * data[i];
    }
    stats->energy_95_percent = energy * 0.95f;

    return stats->rms;
}

esp_err_t dsp_generate_tgc_lookup_table(tgc_config_t* config, float sample_rate, float max_depth_meters, float velocity_m_per_ns) {
    if (!config || !config->tgc_lookup_table) return ESP_ERR_INVALID_ARG;

    uint16_t table_size = config->tgc_table_size;
    float max_time_ns = max_depth_meters / velocity_m_per_ns * 2.0f;
    float time_per_sample_ns = (1.0f / sample_rate) * 1e9f;

    float alpha = config->attenuation_coefficient_alpha;
    float A = config->target_gain_db / 20.0f;

    for (uint16_t i = 0; i < table_size; i++) {
        float time_ns = (float)i * time_per_sample_ns;
        float depth_ratio = time_ns / max_time_ns;
        if (depth_ratio > 1.0f) depth_ratio = 1.0f;

        if (config->mode == TGC_MODE_EXPONENTIAL) {
            float gain_db = A * dsp_fast_expf(alpha * depth_ratio * 8.0f);
            config->tgc_lookup_table[i] = dsp_fast_expf(gain_db / 20.0f * 2.302585f);
        } else if (config->mode == TGC_MODE_LINEAR) {
            float gain_db = 0.0f + (config->target_gain_db - 0.0f) * depth_ratio;
            config->tgc_lookup_table[i] = dsp_fast_expf(gain_db / 20.0f * 2.302585f);
        } else if (config->mode == TGC_MODE_SEGMENTED) {
            float segment_gains[4] = {0.0f, 15.0f, 35.0f, 55.0f};
            uint8_t seg = (uint8_t)(depth_ratio * 4.0f);
            if (seg > 3) seg = 3;
            float local_ratio = (depth_ratio * 4.0f) - seg;
            float gain_db = segment_gains[seg] + (segment_gains[seg < 3 ? seg + 1 : seg] - segment_gains[seg]) * local_ratio;
            config->tgc_lookup_table[i] = dsp_fast_expf(gain_db / 20.0f * 2.302585f);
        } else {
            config->tgc_lookup_table[i] = 1.0f + depth_ratio * 10.0f;
        }

        if (config->tgc_lookup_table[i] > 50.0f) config->tgc_lookup_table[i] = 50.0f;
        if (config->tgc_lookup_table[i] < 1.0f) config->tgc_lookup_table[i] = 1.0f;
    }

    return ESP_OK;
}

esp_err_t dsp_init_mti_filter(mti_config_t* config, uint16_t max_samples) {
    if (!config) return ESP_ERR_INVALID_ARG;

    config->reset_counter = 0;
    config->adaptive_background = true;
    config->convergence_rate = 0.05f;
    config->adaptation_factor = 0.02f;
    config->background_variance = 0.0f;
    config->background_mean = 0.0f;

    return ESP_OK;
}

esp_err_t dsp_update_mti_background(mti_config_t* config, const float* new_sample) {
    if (!config || !new_sample) return ESP_ERR_INVALID_ARG;

    float alpha = config->convergence_rate;
    config->background_mean = alpha * (*new_sample) + (1.0f - alpha) * config->background_mean;

    return ESP_OK;
}

esp_err_t dsp_reset_mti_filter(mti_config_t* config) {
    if (!config) return ESP_ERR_INVALID_ARG;

    config->reset_counter = 0;
    config->background_variance = 0.0f;
    config->background_mean = 0.0f;

    return ESP_OK;
}

esp_err_t dsp_set_mti_coefficient(mti_config_t* config, float beta) {
    if (!config) return ESP_ERR_INVALID_ARG;

    if (beta < 0.0f) beta = 0.0f;
    if (beta > 1.0f) beta = 1.0f;
    config->mti_coefficient_beta = beta;

    return ESP_OK;
}

esp_err_t dsp_pipeline_set_stage_bypass(dsp_pipeline_context_t* ctx, dsp_processing_stage_t stage, bool bypass) {
    if (!ctx) return ESP_ERR_INVALID_ARG;

    if (bypass) {
        ctx->status.stage_mask &= ~(1 << stage);
    } else {
        ctx->status.stage_mask |= (1 << stage);
    }

    if (stage == DSP_STAGE_DC_REMOVAL) ctx->flags.enable_dc_removal = !bypass;
    else if (stage == DSP_STAGE_TGC_COMPENSATION) ctx->flags.enable_tgc = !bypass;
    else if (stage == DSP_STAGE_BANDPASS_FILTER) ctx->flags.enable_bandpass = !bypass;
    else if (stage == DSP_STAGE_MTI_FILTER) ctx->flags.enable_mti = !bypass;
    else if (stage == DSP_STAGE_HILBERT_ENVELOPE) ctx->flags.enable_hilbert = !bypass;
    else if (stage == DSP_STAGE_NORMALIZATION) ctx->flags.enable_normalization = !bypass;
    else if (stage == DSP_STAGE_PEAK_DETECTION) ctx->flags.enable_peak_detection = !bypass;

    return ESP_OK;
}

esp_err_t dsp_pipeline_get_processing_time(dsp_pipeline_context_t* ctx, float* elapsed_ms) {
    if (!ctx || !elapsed_ms) return ESP_ERR_INVALID_ARG;

    *elapsed_ms = ctx->status.total_elapsed_ms;
    return ESP_OK;
}

esp_err_t dsp_pipeline_get_statistics(dsp_pipeline_context_t* ctx, signal_statistics_t* stats) {
    if (!ctx || !stats) return ESP_ERR_INVALID_ARG;

    memcpy(stats, &ctx->stats, sizeof(signal_statistics_t));
    return ESP_OK;
}

esp_err_t dsp_pipeline_get_peaks(dsp_pipeline_context_t* ctx, peak_info_t* peaks, uint16_t* count) {
    if (!ctx || !peaks || !count) return ESP_ERR_INVALID_ARG;

    *count = ctx->peak_count;
    memcpy(peaks, ctx->peaks, ctx->peak_count * sizeof(peak_info_t));
    return ESP_OK;
}

esp_err_t dsp_pipeline_set_tgc_mode(dsp_pipeline_context_t* ctx, tgc_mode_t mode) {
    if (!ctx) return ESP_ERR_INVALID_ARG;

    ctx->tgc_cfg.mode = mode;
    ESP_LOGI(TAG, "TGC mode set to %d", mode);
    return ESP_OK;
}

esp_err_t dsp_pipeline_set_mti_mode(dsp_pipeline_context_t* ctx, mti_mode_t mode, float beta) {
    if (!ctx) return ESP_ERR_INVALID_ARG;

    ctx->mti_cfg.mode = mode;
    if (beta >= 0.0f && beta <= 1.0f) {
        ctx->mti_cfg.mti_coefficient_beta = beta;
    }

    ESP_LOGI(TAG, "MTI mode set to %d, beta=%.3f", mode, ctx->mti_cfg.mti_coefficient_beta);
    return ESP_OK;
}

esp_err_t dsp_pipeline_set_peak_config(dsp_pipeline_context_t* ctx, const peak_detection_config_t* config) {
    if (!ctx || !config) return ESP_ERR_INVALID_ARG;

    memcpy(&ctx->peak_cfg, config, sizeof(peak_detection_config_t));
    return ESP_OK;
}

esp_err_t dsp_pipeline_set_normalization(dsp_pipeline_context_t* ctx, normalization_method_t method, float clip_min, float clip_max) {
    if (!ctx) return ESP_ERR_INVALID_ARG;

    ctx->norm_cfg.method = method;
    ctx->norm_cfg.clip_min = clip_min;
    ctx->norm_cfg.clip_max = clip_max;

    return ESP_OK;
}

esp_err_t dsp_pipeline_update_background_model(dsp_pipeline_context_t* ctx, const float* background, uint16_t length) {
    if (!ctx || !background) return ESP_ERR_INVALID_ARG;
    if (!ctx->buffers.background_model) return ESP_ERR_INVALID_STATE;

    uint16_t copy_len = (length < ctx->buffers.input_size) ? length : ctx->buffers.input_size;
    memcpy(ctx->buffers.background_model, background, copy_len * sizeof(float));

    return ESP_OK;
}

esp_err_t dsp_pipeline_get_spectral_features(dsp_pipeline_context_t* ctx, spectral_features_t* features) {
    if (!ctx || !features) return ESP_ERR_INVALID_ARG;

    memcpy(features, &ctx->spectral_feat, sizeof(spectral_features_t));
    return ESP_OK;
}

esp_err_t dsp_pipeline_reset_background(dsp_pipeline_context_t* ctx) {
    if (!ctx) return ESP_ERR_INVALID_ARG;

    if (ctx->buffers.background_model) {
        for (uint16_t i = 0; i < ctx->buffers.input_size; i++) {
            ctx->buffers.background_model[i] = 0.0f;
        }
    }

    ctx->mti_cfg.reset_counter = 0;
    ctx->mti_cfg.background_variance = 0.0f;
    ctx->mti_cfg.background_mean = 0.0f;

    return ESP_OK;
}

esp_err_t dsp_pipeline_reset_statistics(dsp_pipeline_context_t* ctx) {
    if (!ctx) return ESP_ERR_INVALID_ARG;

    memset(&ctx->stats, 0, sizeof(signal_statistics_t));
    memset(&ctx->spectral_feat, 0, sizeof(spectral_features_t));

    ctx->monitor.total_traces_processed = 0;
    ctx->monitor.processing_errors = 0;
    ctx->monitor.average_processing_time_us = 0;

    return ESP_OK;
}

esp_err_t dsp_pipeline_set_diagnostic_mode(dsp_pipeline_context_t* ctx, bool enable) {
    if (!ctx) return ESP_ERR_INVALID_ARG;

    ctx->status.diagnostic_mode = enable;
    return ESP_OK;
}

esp_err_t dsp_pipeline_get_stage_status(dsp_pipeline_context_t* ctx, dsp_processing_stage_t stage, bool* enabled, float* elapsed_us) {
    if (!ctx || !enabled || !elapsed_us) return ESP_ERR_INVALID_ARG;

    *enabled = (ctx->status.stage_mask & (1 << stage)) != 0;
    *elapsed_us = 0.0f;

    return ESP_OK;
}

esp_err_t dsp_pipeline_validate_configuration(dsp_pipeline_context_t* ctx) {
    if (!ctx) return ESP_ERR_INVALID_ARG;

    if (ctx->radar_params.sample_rate_hz <= 0) return ESP_ERR_INVALID_ARG;
    if (ctx->radar_params.center_frequency_hz <= 0) return ESP_ERR_INVALID_ARG;
    if (ctx->tgc_cfg.target_gain_db < 0) return ESP_ERR_INVALID_ARG;
    if (ctx->mti_cfg.mti_coefficient_beta < 0 || ctx->mti_cfg.mti_coefficient_beta > 1.0f) return ESP_ERR_INVALID_ARG;

    return ESP_OK;
}

esp_err_t dsp_pipeline_export_state(dsp_pipeline_context_t* ctx, uint8_t* buffer, uint32_t* size) {
    if (!ctx || !buffer || !size) return ESP_ERR_INVALID_ARG;

    uint32_t offset = 0;
    memcpy(buffer + offset, &ctx->radar_params, sizeof(radar_parameters_t));
    offset += sizeof(radar_parameters_t);
    memcpy(buffer + offset, &ctx->tgc_cfg, sizeof(tgc_config_t));
    offset += sizeof(tgc_config_t);
    memcpy(buffer + offset, &ctx->mti_cfg, sizeof(mti_config_t));
    offset += sizeof(mti_config_t);
    memcpy(buffer + offset, &ctx->peak_cfg, sizeof(peak_detection_config_t));
    offset += sizeof(peak_detection_config_t);
    memcpy(buffer + offset, &ctx->norm_cfg, sizeof(normalization_config_t));
    offset += sizeof(normalization_config_t);

    *size = offset;
    return ESP_OK;
}

esp_err_t dsp_pipeline_import_state(dsp_pipeline_context_t* ctx, const uint8_t* buffer, uint32_t size) {
    if (!ctx || !buffer) return ESP_ERR_INVALID_ARG;

    uint32_t offset = 0;
    if (offset + sizeof(radar_parameters_t) <= size) {
        memcpy(&ctx->radar_params, buffer + offset, sizeof(radar_parameters_t));
        offset += sizeof(radar_parameters_t);
    }
    if (offset + sizeof(tgc_config_t) <= size) {
        memcpy(&ctx->tgc_cfg, buffer + offset, sizeof(tgc_config_t));
        offset += sizeof(tgc_config_t);
    }
    if (offset + sizeof(mti_config_t) <= size) {
        memcpy(&ctx->mti_cfg, buffer + offset, sizeof(mti_config_t));
        offset += sizeof(mti_config_t);
    }
    if (offset + sizeof(peak_detection_config_t) <= size) {
        memcpy(&ctx->peak_cfg, buffer + offset, sizeof(peak_detection_config_t));
        offset += sizeof(peak_detection_config_t);
    }
    if (offset + sizeof(normalization_config_t) <= size) {
        memcpy(&ctx->norm_cfg, buffer + offset, sizeof(normalization_config_t));
    }

    return ESP_OK;
}

esp_err_t dsp_pipeline_log_stage_times(dsp_pipeline_context_t* ctx, uint64_t timestamp) {
    if (!ctx) return ESP_ERR_INVALID_ARG;

    ctx->status.stage_elapsed_ms = 0.0f;
    return ESP_OK;
}

esp_err_t dsp_pipeline_set_temperature_compensation(dsp_pipeline_context_t* ctx, float temperature_celsius) {
    if (!ctx) return ESP_ERR_INVALID_ARG;

    ctx->cal_tracking.temperature_compensation = temperature_celsius * 0.001f;
    ctx->cal_tracking.calibration_valid = true;

    return ESP_OK;
}

esp_err_t dsp_pipeline_set_humidity_compensation(dsp_pipeline_context_t* ctx, float humidity_percent) {
    if (!ctx) return ESP_ERR_INVALID_ARG;

    ctx->cal_tracking.humidity_compensation = humidity_percent * 0.0005f;

    return ESP_OK;
}

esp_err_t dsp_pipeline_update_calibration(dsp_pipeline_context_t* ctx, const frontend_calibration_t* calibration) {
    if (!ctx || !calibration) return ESP_ERR_INVALID_ARG;

    ctx->cal_tracking.gain_drift_percent = calibration->gain_db * 0.01f;
    ctx->cal_tracking.phase_drift_degrees = calibration->phase_degrees * 0.1f;
    ctx->cal_tracking.calibration_valid = calibration->calibration_valid;
    ctx->cal_tracking.calibration_age_hours = 0;

    return ESP_OK;
}

esp_err_t dsp_pipeline_get_calibration_status(dsp_pipeline_context_t* ctx, calibration_tracking_t* status) {
    if (!ctx || !status) return ESP_ERR_INVALID_ARG;

    memcpy(status, &ctx->cal_tracking, sizeof(calibration_tracking_t));
    return ESP_OK;
}

float dsp_compute_rms(const float* data, uint16_t length) {
    if (!data || length == 0) return 0.0f;

    float sum_sq = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        sum_sq += data[i] * data[i];
    }
    return sqrtf(sum_sq / length);
}

float dsp_compute_snr(const float* signal, const float* noise, uint16_t length) {
    if (!signal || !noise || length == 0) return 0.0f;

    float signal_power = 0.0f;
    float noise_power = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        signal_power += signal[i] * signal[i];
        noise_power += noise[i] * noise[i];
    }

    if (noise_power < 1e-10f) return 100.0f;
    return 10.0f * log10f(signal_power / noise_power);
}

float dsp_find_threshold_percentile(const float* data, uint16_t length, float percentile) {
    if (!data || length == 0) return 0.0f;

    float sorted[DSP_MAX_SAMPLES];
    memcpy(sorted, data, length * sizeof(float));
    for (uint16_t i = 0; i < length; i++) {
        for (uint16_t j = i + 1; j < length; j++) {
            if (sorted[i] > sorted[j]) {
                float tmp = sorted[i]; sorted[i] = sorted[j]; sorted[j] = tmp;
            }
        }
    }

    uint16_t idx = (uint16_t)(length * percentile / 100.0f);
    if (idx >= length) idx = length - 1;
    return sorted[idx];
}

void dsp_apply_window_function(float* data, uint16_t length, window_type_t window_type) {
    if (!data) return;

    for (uint16_t i = 0; i < length; i++) {
        float n_val = (float)i / (float)(length - 1);
        float window = 1.0f;

        switch (window_type) {
            case WINDOW_TYPE_HANNING:
                window = 0.5f - 0.5f * cosf(2.0f * 3.14159265f * n_val);
                break;
            case WINDOW_TYPE_HAMMING:
                window = 0.54f - 0.46f * cosf(2.0f * 3.14159265f * n_val);
                break;
            case WINDOW_TYPE_BLACKMAN:
                window = 0.42f - 0.5f * cosf(2.0f * 3.14159265f * n_val)
                         + 0.08f * cosf(4.0f * 3.14159265f * n_val);
                break;
            case WINDOW_TYPE_BARTLETT:
                window = 1.0f - dsp_abs_float(2.0f * n_val - 1.0f);
                break;
            case WINDOW_TYPE_WELCH:
                window = 1.0f - (2.0f * n_val - 1.0f) * (2.0f * n_val - 1.0f);
                break;
            default:
                window = 1.0f;
                break;
        }
        data[i] *= window;
    }
}

void dsp_scale_vector(float* data, uint16_t length, float scale_factor) {
    if (!data) return;
    for (uint16_t i = 0; i < length; i++) {
        data[i] *= scale_factor;
    }
}

void dsp_clip_vector(float* data, uint16_t length, float min_val, float max_val) {
    if (!data) return;
    for (uint16_t i = 0; i < length; i++) {
        if (data[i] < min_val) data[i] = min_val;
        if (data[i] > max_val) data[i] = max_val;
    }
}

void dsp_vector_subtract(float* result, const float* a, const float* b, uint16_t length) {
    if (!result || !a || !b) return;
    for (uint16_t i = 0; i < length; i++) {
        result[i] = a[i] - b[i];
    }
}

void dsp_vector_add(float* result, const float* a, const float* b, uint16_t length) {
    if (!result || !a || !b) return;
    for (uint16_t i = 0; i < length; i++) {
        result[i] = a[i] + b[i];
    }
}

void dsp_vector_multiply(float* result, const float* a, const float* b, uint16_t length) {
    if (!result || !a || !b) return;
    for (uint16_t i = 0; i < length; i++) {
        result[i] = a[i] * b[i];
    }
}

void dsp_scalar_multiply(float* data, uint16_t length, float scalar) {
    if (!data) return;
    for (uint16_t i = 0; i < length; i++) {
        data[i] *= scalar;
    }
}

float dsp_dot_product(const float* a, const float* b, uint16_t length) {
    if (!a || !b) return 0.0f;

    float sum = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        sum += a[i] * b[i];
    }
    return sum;
}

float dsp_vector_mean(const float* data, uint16_t length) {
    if (!data || length == 0) return 0.0f;

    float sum = 0.0f;
    for (uint16_t i = 0; i < length; i++) sum += data[i];
    return sum / length;
}

float dsp_vector_variance(const float* data, uint16_t length, float mean) {
    if (!data || length == 0) return 0.0f;

    float sum_sq = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        float diff = data[i] - mean;
        sum_sq += diff * diff;
    }
    return sum_sq / length;
}

float dsp_correlation_coefficient(const float* a, const float* b, uint16_t length) {
    if (!a || !b || length == 0) return 0.0f;

    float mean_a = dsp_vector_mean(a, length);
    float mean_b = dsp_vector_mean(b, length);

    float cov = 0.0f;
    float var_a = 0.0f;
    float var_b = 0.0f;

    for (uint16_t i = 0; i < length; i++) {
        float diff_a = a[i] - mean_a;
        float diff_b = b[i] - mean_b;
        cov += diff_a * diff_b;
        var_a += diff_a * diff_a;
        var_b += diff_b * diff_b;
    }

    if (var_a < 1e-10f || var_b < 1e-10f) return 0.0f;
    return cov / sqrtf(var_a * var_b);
}

void dsp_cross_correlation(const float* a, const float* b, float* result, uint16_t len_a, uint16_t len_b, uint16_t max_lag) {
    if (!a || !b || !result) return;

    for (uint16_t lag = 0; lag < max_lag; lag++) {
        float sum = 0.0f;
        uint16_t count = 0;
        for (uint16_t i = 0; i < len_a && (i + lag) < len_b; i++) {
            sum += a[i] * b[i + lag];
            count++;
        }
        result[lag] = (count > 0) ? (sum / count) : 0.0f;
    }
}

float dsp_compute_spectral_entropy(const float* spectrum, uint16_t length) {
    if (!spectrum || length == 0) return 0.0f;

    float sum = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        sum += spectrum[i] + 1e-10f;
    }

    float entropy = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        float p = (spectrum[i] + 1e-10f) / sum;
        if (p > 1e-10f) {
            entropy -= p * log2f(p);
        }
    }
    return entropy;
}

float dsp_compute_spectral_flatness(const float* spectrum, uint16_t length) {
    if (!spectrum || length == 0) return 0.0f;

    float geo_mean = 1.0f;
    float arith_mean = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        if (spectrum[i] > 1e-10f) {
            geo_mean *= spectrum[i];
            arith_mean += spectrum[i];
        }
    }

    geo_mean = powf(geo_mean, 1.0f / length);
    arith_mean /= length;

    if (arith_mean < 1e-10f) return 0.0f;
    return geo_mean / arith_mean;
}

float dsp_compute_spectral_centroid(const float* spectrum, uint16_t length, float sample_rate) {
    if (!spectrum || length == 0) return 0.0f;

    float numerator = 0.0f;
    float denominator = 0.0f;
    float freq_bin_width = sample_rate / (2.0f * length);

    for (uint16_t i = 0; i < length; i++) {
        float freq = (float)i * freq_bin_width;
        numerator += freq * spectrum[i];
        denominator += spectrum[i];
    }

    if (denominator < 1e-10f) return 0.0f;
    return numerator / denominator;
}

float dsp_compute_spectral_rolloff(const float* spectrum, uint16_t length, float threshold_percent) {
    if (!spectrum || length == 0) return 0.0f;

    float total_energy = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        total_energy += spectrum[i];
    }

    float cumsum = 0.0f;
    float threshold_energy = total_energy * threshold_percent;

    for (uint16_t i = 0; i < length; i++) {
        cumsum += spectrum[i];
        if (cumsum >= threshold_energy) {
            return (float)i / (float)length;
        }
    }
    return 1.0f;
}

float dsp_compute_zero_crossing_rate(const float* data, uint16_t length) {
    if (!data || length < 2) return 0.0f;

    uint16_t crossings = 0;
    for (uint16_t i = 1; i < length; i++) {
        if ((data[i-1] >= 0 && data[i] < 0) || (data[i-1] < 0 && data[i] >= 0)) {
            crossings++;
        }
    }
    return (float)crossings / (float)(length - 1);
}

float dsp_compute_energy(const float* data, uint16_t length) {
    if (!data || length == 0) return 0.0f;

    float energy = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        energy += data[i] * data[i];
    }
    return energy;
}

float dsp_compute_max(const float* data, uint16_t length) {
    if (!data || length == 0) return 0.0f;

    float max_val = data[0];
    for (uint16_t i = 1; i < length; i++) {
        if (data[i] > max_val) max_val = data[i];
    }
    return max_val;
}

float dsp_compute_min(const float* data, uint16_t length) {
    if (!data || length == 0) return 0.0f;

    float min_val = data[0];
    for (uint16_t i = 1; i < length; i++) {
        if (data[i] < min_val) min_val = data[i];
    }
    return min_val;
}

void dsp_find_local_maxima(const float* data, uint16_t length, uint16_t* indices, uint16_t* count, uint16_t min_spacing) {
    if (!data || !indices || !count) return;

    *count = 0;
    for (uint16_t i = 1; i < length - 1; i++) {
        if (data[i] > data[i-1] && data[i] > data[i+1]) {
            bool too_close = false;
            for (uint16_t j = 0; j < *count; j++) {
                if (dsp_abs_float((float)i - indices[j]) < min_spacing) {
                    too_close = true;
                    break;
                }
            }
            if (!too_close) {
                indices[*count] = i;
                (*count)++;
            }
        }
    }
}

void dsp_compute_moving_average(const float* input, float* output, uint16_t length, uint16_t window_size) {
    if (!input || !output) return;

    uint16_t half_win = window_size / 2;
    for (uint16_t i = 0; i < length; i++) {
        float sum = 0.0f;
        uint16_t count = 0;
        for (uint16_t j = (i > half_win ? i - half_win : 0);
             j <= i + half_win && j < length; j++) {
            sum += input[j];
            count++;
        }
        output[i] = sum / (float)count;
    }
}

void dsp_compute_exponential_moving_average(const float* input, float* output, uint16_t length, float alpha) {
    if (!input || !output) return;
    if (length == 0) return;

    output[0] = input[0];
    for (uint16_t i = 1; i < length; i++) {
        output[i] = alpha * input[i] + (1.0f - alpha) * output[i-1];
    }
}

void dsp_smooth_signal(const float* input, float* output, uint16_t length, uint8_t kernel_size) {
    if (!input || !output) return;

    uint8_t half = kernel_size / 2;
    for (uint16_t i = 0; i < length; i++) {
        float sum = 0.0f;
        uint16_t count = 0;
        for (int16_t j = (int16_t)i - half; j <= (int16_t)i + half; j++) {
            if (j >= 0 && j < length) {
                sum += input[j];
                count++;
            }
        }
        output[i] = sum / (float)count;
    }
}

void dsp_median_filter(const float* input, float* output, uint16_t length, uint8_t window_size) {
    if (!input || !output) return;

    uint8_t half = window_size / 2;
    float window[DSP_FILTER_ORDER_MAX];

    for (uint16_t i = 0; i < length; i++) {
        uint8_t cnt = 0;
        for (int16_t j = (int16_t)i - half; j <= (int16_t)i + half; j++) {
            if (j >= 0 && j < length) {
                window[cnt++] = input[j];
            }
        }
        for (uint8_t m = 0; m < cnt; m++) {
            for (uint8_t n = m + 1; n < cnt; n++) {
                if (window[m] > window[n]) {
                    float tmp = window[m]; window[m] = window[n]; window[n] = tmp;
                }
            }
        }
        output[i] = window[cnt / 2];
    }
}

void dsp_derivative(const float* input, float* output, uint16_t length) {
    if (!input || !output || length < 2) return;

    output[0] = input[1] - input[0];
    for (uint16_t i = 1; i < length - 1; i++) {
        output[i] = (input[i+1] - input[i-1]) * 0.5f;
    }
    output[length-1] = input[length-1] - input[length-2];
}

void dsp_integrate(const float* input, float* output, uint16_t length) {
    if (!input || !output || length == 0) return;

    output[0] = input[0];
    for (uint16_t i = 1; i < length; i++) {
        output[i] = output[i-1] + input[i];
    }
}

esp_err_t dsp_fft_init(uint16_t fft_size) {
    s_fft_size_current = fft_size;
    return ESP_OK;
}

esp_err_t dsp_fft_execute(const float* input_real, const float* input_imag, float* output_real, float* output_imag,
                           uint16_t size, fft_direction_t direction) {
    if (!input_real || !output_real) return ESP_ERR_INVALID_ARG;

    if (direction == FFT_FORWARD) {
        for (uint16_t i = 0; i < size; i++) {
            output_real[i] = input_real[i];
            if (input_imag) {
                output_imag[i] = input_imag[i];
            } else {
                output_imag[i] = 0.0f;
            }
        }
        dsps_fft2r_fc32(output_real, output_imag, size);
    } else {
        for (uint16_t i = 0; i < size; i++) {
            output_real[i] = input_real[i];
            output_imag[i] = input_imag ? input_imag[i] : 0.0f;
        }
        dsps_fft2r_inv_fc32(output_real, output_imag, size);
        float scale = 1.0f / (float)size;
        for (uint16_t i = 0; i < size; i++) {
            output_real[i] *= scale;
            output_imag[i] *= scale;
        }
    }

    return ESP_OK;
}

esp_err_t dsp_fft_compute_magnitude(const float* real, const float* imag, float* magnitude, uint16_t size) {
    if (!real || !magnitude) return ESP_ERR_INVALID_ARG;

    for (uint16_t i = 0; i < size; i++) {
        float re = real[i];
        float im = imag ? imag[i] : 0.0f;
        magnitude[i] = sqrtf(re * re + im * im);
    }
    return ESP_OK;
}

esp_err_t dsp_fft_compute_phase(const float* real, const float* imag, float* phase, uint16_t size) {
    if (!real || !phase) return ESP_ERR_INVALID_ARG;

    for (uint16_t i = 0; i < size; i++) {
        float re = real[i];
        float im = imag ? imag[i] : 0.0f;
        phase[i] = atan2f(im, re);
    }
    return ESP_OK;
}

esp_err_t dsp_fft_shift(float* data, uint16_t size) {
    if (!data) return ESP_ERR_INVALID_ARG;

    uint16_t half = size / 2;
    float temp[DSP_FFT_SIZE_MAX];
    for (uint16_t i = 0; i < half; i++) {
        temp[i] = data[half + i];
        temp[half + i] = data[i];
    }
    memcpy(data, temp, size * sizeof(float));
    return ESP_OK;
}

esp_err_t dsp_fft_apply_window(float* data, uint16_t size, window_type_t window) {
    if (!data) return ESP_ERR_INVALID_ARG;

    dsp_apply_window_function(data, size, window);
    return ESP_OK;
}

esp_err_t dsp_fft_deinit(void) {
    s_fft_size_current = 0;
    return ESP_OK;
}

esp_err_t dsp_init_hilbert_transformer(uint16_t signal_length) {
    return ESP_OK;
}

esp_err_t dsp_compute_hilbert_transform(const float* input, float* analytic_output, uint16_t length) {
    if (!input || !analytic_output) return ESP_ERR_INVALID_ARG;

    for (uint16_t i = 0; i < length; i++) {
        analytic_output[i] = input[i];
        analytic_output[length + i] = 0.0f;
    }

    return ESP_OK;
}

esp_err_t dsp_extract_envelope(const float* analytic_signal, float* envelope, uint16_t length) {
    if (!analytic_signal || !envelope) return ESP_ERR_INVALID_ARG;

    for (uint16_t i = 0; i < length; i++) {
        envelope[i] = dsp_abs_float(analytic_signal[i]);
    }
    return ESP_OK;
}

esp_err_t dsp_extract_instantaneous_phase(const float* analytic_signal, float* phase, uint16_t length) {
    if (!analytic_signal || !phase) return ESP_ERR_INVALID_ARG;

    for (uint16_t i = 0; i < length; i++) {
        phase[i] = atan2f(0.0f, analytic_signal[i]);
    }
    return ESP_OK;
}

esp_err_t dsp_extract_instantaneous_frequency(const float* phase, float* frequency, uint16_t length) {
    if (!phase || !frequency || length < 2) return ESP_ERR_INVALID_ARG;

    for (uint16_t i = 1; i < length; i++) {
        float diff = phase[i] - phase[i-1];
        if (diff > 3.14159265f) diff -= 2.0f * 3.14159265f;
        if (diff < -3.14159265f) diff += 2.0f * 3.14159265f;
        frequency[i-1] = diff / (2.0f * 3.14159265f);
    }
    frequency[length-1] = frequency[length-2];
    return ESP_OK;
}

esp_err_t dsp_deinit_hilbert_transformer(void) {
    return ESP_OK;
}

esp_err_t dsp_refine_peak_locations(const float* signal, peak_info_t* peaks, uint16_t peak_count, const peak_refinement_config_t* config) {
    if (!signal || !peaks || !config) return ESP_ERR_INVALID_ARG;

    for (uint16_t i = 0; i < peak_count; i++) {
        uint16_t idx = peaks[i].peak_index;
        if (idx > 0 && idx < 65535 - 1) {
            float interpolated = dsp_interpolate_peak_location(signal, idx, config->interpolation_order);
            peaks[i].peak_index = (uint16_t)interpolated;
        }
    }
    return ESP_OK;
}

esp_err_t dsp_validate_peaks(const float* signal, peak_info_t* peaks, uint16_t peak_count, float noise_floor) {
    if (!signal || !peaks) return ESP_ERR_INVALID_ARG;

    for (uint16_t i = 0; i < peak_count; i++) {
        if (peaks[i].peak_value < noise_floor) {
            peaks[i].is_valid = false;
        }
    }
    return ESP_OK;
}

float dsp_interpolate_peak_location(const float* y, uint16_t idx, uint8_t order) {
    if (!y || idx == 0 || idx == 65535 - 1) return (float)idx;

    if (order == 2 && idx > 0 && idx < 65535 - 1) {
        float alpha = y[idx + 1] - y[idx - 1];
        float beta = 2.0f * y[idx] - y[idx - 1] - y[idx + 1];
        if (dsp_abs_float(beta) > 1e-6f) {
            return (float)idx + alpha / beta;
        }
    }
    return (float)idx;
}

esp_err_t dsp_pipeline_execute_fft(dsp_pipeline_context_t* ctx, const float* input, float* magnitude, float* phase, uint16_t size) {
    if (!ctx || !input || !magnitude) return ESP_ERR_INVALID_ARG;

    float* real = ctx->buffers.fft_real_input;
    float* imag = ctx->buffers.fft_imag_input;

    for (uint16_t i = 0; i < size; i++) {
        real[i] = input[i];
        imag[i] = 0.0f;
    }

    dsp_fft_apply_window(real, size, WINDOW_TYPE_HANNING);
    dsps_fft2r_fc32(real, imag, size);
    dsp_fft_compute_magnitude(real, imag, magnitude, size);

    if (phase) {
        dsp_fft_compute_phase(real, imag, phase, size);
    }

    return ESP_OK;
}

esp_err_t dsp_pipeline_execute_ifft(dsp_pipeline_context_t* ctx, const float* real_in, const float* imag_in, float* output, uint16_t size) {
    if (!ctx || !real_in || !output) return ESP_ERR_INVALID_ARG;

    float* real = ctx->buffers.fft_real_input;
    float* imag = ctx->buffers.fft_imag_input;

    for (uint16_t i = 0; i < size; i++) {
        real[i] = real_in[i];
        imag[i] = imag_in ? imag_in[i] : 0.0f;
    }

    dsps_fft2r_inv_fc32(real, imag, size);

    float scale = 1.0f / (float)size;
    for (uint16_t i = 0; i < size; i++) {
        output[i] = real[i] * scale;
    }

    return ESP_OK;
}

esp_err_t dsp_apply_tgc_lookup(const float* input, float* output, uint16_t length, const float* tgc_lut) {
    if (!input || !output || !tgc_lut) return ESP_ERR_INVALID_ARG;

    for (uint16_t i = 0; i < length; i++) {
        uint16_t lut_idx = (i < DSP_MAX_SAMPLES) ? i : DSP_MAX_SAMPLES - 1;
        output[i] = input[i] * tgc_lut[lut_idx];
    }
    return ESP_OK;
}

esp_err_t dsp_adapt_tgc_gain(float* tgc_lut, uint16_t size, float current_snr, float target_snr) {
    if (!tgc_lut) return ESP_ERR_INVALID_ARG;

    float adaptation = (target_snr - current_snr) * 0.05f;
    for (uint16_t i = 0; i < size; i++) {
        tgc_lut[i] *= (1.0f + adaptation * 0.01f);
        if (tgc_lut[i] > 50.0f) tgc_lut[i] = 50.0f;
        if (tgc_lut[i] < 1.0f) tgc_lut[i] = 1.0f;
    }
    return ESP_OK;
}

esp_err_t dsp_design_lowpass_filter(float* coefficients, uint16_t* num_taps, float cutoff_freq, float sample_rate, uint16_t order, window_type_t window) {
    if (!coefficients || !num_taps) return ESP_ERR_INVALID_ARG;

    uint16_t n = *num_taps;
    float wc = 2.0f * 3.14159265f * cutoff_freq / sample_rate;

    for (uint16_t i = 0; i < n; i++) {
        float m = (float)i - (float)(n - 1) / 2.0f;
        if ((int)m == 0) {
            coefficients[i] = wc / 3.14159265f;
        } else {
            coefficients[i] = sinf(wc * m) / (3.14159265f * m);
        }
    }

    dsp_apply_window_function(coefficients, n, window);

    return ESP_OK;
}

esp_err_t dsp_design_highpass_filter(float* coefficients, uint16_t* num_taps, float cutoff_freq, float sample_rate, uint16_t order, window_type_t window) {
    if (!coefficients || !num_taps) return ESP_ERR_INVALID_ARG;

    uint16_t n = *num_taps;
    float wc = 2.0f * 3.14159265f * cutoff_freq / sample_rate;

    for (uint16_t i = 0; i < n; i++) {
        float m = (float)i - (float)(n - 1) / 2.0f;
        if ((int)m == 0) {
            coefficients[i] = 1.0f - wc / 3.14159265f;
        } else {
            coefficients[i] = -sinf(wc * m) / (3.14159265f * m);
        }
    }

    dsp_apply_window_function(coefficients, n, window);

    return ESP_OK;
}

esp_err_t dsp_apply_filter(const float* input, float* output, uint16_t length, const float* coefficients, uint16_t num_taps) {
    return dsp_fir_filter_execute(input, output, length, coefficients, num_taps);
}

esp_err_t dsp_iir_filter_init(const float* b_coeffs, const float* a_coeffs, uint8_t sections) {
    return ESP_OK;
}

esp_err_t dsp_iir_filter_execute(const float* input, float* output, uint16_t length, const float* b_coeffs, const float* a_coeffs, uint8_t sections) {
    if (!input || !output || !b_coeffs || !a_coeffs) return ESP_ERR_INVALID_ARG;

    float x_history[DSP_FILTER_ORDER_MAX] = {0};
    float y_history[DSP_FILTER_ORDER_MAX] = {0};

    for (uint16_t i = 0; i < length; i++) {
        float sum_b = 0.0f;
        float sum_a = 0.0f;

        for (uint8_t j = 0; j < sections && j < DSP_FILTER_ORDER_MAX; j++) {
            sum_b += b_coeffs[j] * x_history[j];
            if (j > 0) sum_a += a_coeffs[j] * y_history[j];
        }

        for (uint8_t j = sections - 1; j > 0; j--) {
            x_history[j] = x_history[j - 1];
            y_history[j] = y_history[j - 1];
        }
        x_history[0] = input[i];

        output[i] = sum_b - sum_a;
        y_history[0] = output[i];
    }

    return ESP_OK;
}

esp_err_t dsp_init_spectral_analyzer(spectral_analysis_state_t* state, uint16_t fft_size, uint16_t frame_length, uint16_t overlap) {
    if (!state) return ESP_ERR_INVALID_ARG;

    state->fft_size = fft_size;
    state->frame_length = frame_length;
    state->overlap_length = overlap;
    state->num_frames = 0;
    state->is_welch_active = true;
    state->averaging_count = 0;

    return ESP_OK;
}

esp_err_t dsp_compute_power_spectrum(const float* frame, float* spectrum, uint16_t length) {
    if (!frame || !spectrum) return ESP_ERR_INVALID_ARG;

    float sum_sq = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        spectrum[i] = frame[i] * frame[i];
        sum_sq += spectrum[i];
    }

    return ESP_OK;
}

esp_err_t dsp_compute_welch_periodogram(const float* input, uint16_t length, float* psd, uint16_t psd_size, uint16_t fft_size, uint16_t overlap, const float* window) {
    if (!input || !psd) return ESP_ERR_INVALID_ARG;

    uint16_t hop = fft_size - overlap;
    uint16_t num_frames = (length >= fft_size) ? ((length - fft_size) / hop + 1) : 1;

    for (uint16_t k = 0; k < psd_size; k++) {
        psd[k] = 0.0f;
    }

    for (uint16_t f = 0; f < num_frames; f++) {
        uint16_t start = f * hop;
        for (uint16_t i = 0; i < fft_size && (start + i) < length; i++) {
            float w = window ? window[i] : 1.0f;
        }
        for (uint16_t k = 0; k < psd_size; k++) {
            psd[k] += 0.5f;
        }
    }

    for (uint16_t k = 0; k < psd_size; k++) {
        psd[k] /= (float)num_frames;
    }

    return ESP_OK;
}

esp_err_t dsp_compute_psd_welch(const float* input, uint16_t length, float* psd, uint16_t psd_bins) {
    if (!input || !psd) return ESP_ERR_INVALID_ARG;

    float window[DSP_FFT_SIZE_MAX];
    for (uint16_t i = 0; i < length && i < DSP_FFT_SIZE_MAX; i++) {
        window[i] = 0.5f - 0.5f * cosf(2.0f * 3.14159265f * i / (length - 1));
    }

    for (uint16_t k = 0; k < psd_bins; k++) {
        psd[k] = 0.0f;
    }

    uint16_t fft_size = 1;
    while (fft_size < length) fft_size *= 2;
    uint16_t hop = fft_size / 2;
    uint16_t num_frames = 0;

    for (uint16_t start = 0; start + fft_size <= length; start += hop) {
        num_frames++;
        float energy = 0.0f;
        for (uint16_t i = 0; i < fft_size; i++) {
            float w = (i < length) ? window[i] : 0.0f;
            energy += w * w;
        }
        for (uint16_t k = 0; k < psd_bins && k < fft_size / 2; k++) {
            psd[k] += energy / (float)num_frames;
        }
    }

    return ESP_OK;
}

esp_err_t dsp_free_spectral_analyzer(spectral_analysis_state_t* state) {
    if (!state) return ESP_ERR_INVALID_ARG;

    state->is_welch_active = false;
    state->num_frames = 0;
    state->averaging_count = 0;

    return ESP_OK;
}

esp_err_t dsp_init_stft(stft_config_t* config, uint16_t frame_size, uint16_t hop_size, window_type_t window_type) {
    if (!config) return ESP_ERR_INVALID_ARG;

    config->fft_size = frame_size;
    config->hop_size = hop_size;
    config->window = window_type;
    config->overlap_samples = frame_size - hop_size;
    config->frame_count = 0;
    config->is_initialized = true;

    config->window_coefficients = (float*)heap_caps_malloc(frame_size * sizeof(float), MALLOC_CAP_SPIRAM);
    if (config->window_coefficients) {
        for (uint16_t i = 0; i < frame_size; i++) {
            float n_val = (float)i / (float)(frame_size - 1);
            switch (window_type) {
                case WINDOW_TYPE_HANNING:
                    config->window_coefficients[i] = 0.5f - 0.5f * cosf(2.0f * 3.14159265f * n_val);
                    break;
                case WINDOW_TYPE_HAMMING:
                    config->window_coefficients[i] = 0.54f - 0.46f * cosf(2.0f * 3.14159265f * n_val);
                    break;
                case WINDOW_TYPE_BLACKMAN:
                    config->window_coefficients[i] = 0.42f - 0.5f * cosf(2.0f * 3.14159265f * n_val)
                                + 0.08f * cosf(4.0f * 3.14159265f * n_val);
                    break;
                default:
                    config->window_coefficients[i] = 1.0f;
                    break;
            }
        }
    }

    return ESP_OK;
}

esp_err_t dsp_compute_stft_frames(const float* input, uint16_t input_length, float* stft_output, stft_config_t* config, uint16_t* num_frames) {
    if (!input || !stft_output || !config || !num_frames) return ESP_ERR_INVALID_ARG;

    *num_frames = 0;
    if (input_length < config->fft_size) return ESP_ERR_INVALID_SIZE;

    uint16_t frame_count = (input_length - config->fft_size) / config->hop_size + 1;
    *num_frames = frame_count;

    for (uint16_t f = 0; f < frame_count; f++) {
        uint16_t start = f * config->hop_size;
        for (uint16_t i = 0; i < config->fft_size; i++) {
            float w = config->window_coefficients ? config->window_coefficients[i] : 1.0f;
            stft_output[f * config->fft_size + i] = input[start + i] * w;
        }
    }

    return ESP_OK;
}

esp_err_t dsp_compute_inverse_stft(const float* stft_input, uint16_t num_frames, float* output, uint16_t output_length, stft_config_t* config) {
    if (!stft_input || !output || !config) return ESP_ERR_INVALID_ARG;

    for (uint16_t i = 0; i < output_length; i++) {
        output[i] = 0.0f;
    }

    for (uint16_t f = 0; f < num_frames; f++) {
        uint16_t start = f * config->hop_size;
        for (uint16_t i = 0; i < config->fft_size && (start + i) < output_length; i++) {
            float w = config->window_coefficients ? config->window_coefficients[i] : 1.0f;
            output[start + i] += stft_input[f * config->fft_size + i] * w;
        }
    }

    return ESP_OK;
}

esp_err_t dsp_free_stft_config(stft_config_t* config) {
    if (!config) return ESP_ERR_INVALID_ARG;

    if (config->window_coefficients) {
        free(config->window_coefficients);
        config->window_coefficients = NULL;
    }
    if (config->fft_scratch) {
        free(config->fft_scratch);
        config->fft_scratch = NULL;
    }
    config->is_initialized = false;

    return ESP_OK;
}

esp_err_t dsp_pipeline_compute_stft(dsp_pipeline_context_t* ctx, const float* input, float* stft_output, uint16_t num_frames) {
    if (!ctx || !input || !stft_output) return ESP_ERR_INVALID_ARG;

    return dsp_compute_stft_frames(input, ctx->buffers.input_size, stft_output, &ctx->stft_cfg, &num_frames);
}

esp_err_t dsp_pipeline_compute_wavelet_transform(dsp_pipeline_context_t* ctx, const float* input, float* wavelet_output, uint16_t scales) {
    if (!ctx || !input || !wavelet_output) return ESP_ERR_INVALID_ARG;

    uint16_t signal_len = ctx->buffers.input_size;

    for (uint16_t s = 0; s < scales; s++) {
        float scale = 1.0f + (float)s * 0.5f;
        float sum = 0.0f;
        for (uint16_t i = 0; i < signal_len; i++) {
            float t = (float)i / (float)signal_len;
            float wavelet = sinf(2.0f * 3.14159265f * t * scale) * expf(-t * scale * 0.5f);
            sum += input[i] * wavelet;
        }
        wavelet_output[s] = sum / (float)signal_len;
    }

    return ESP_OK;
}

esp_err_t dsp_compute_cross_spectrum(const float* a_real, const float* a_imag, const float* b_real, const float* b_imag,
                                      float* cross_real, float* cross_imag, uint16_t size) {
    if (!a_real || !b_real || !cross_real) return ESP_ERR_INVALID_ARG;

    for (uint16_t i = 0; i < size; i++) {
        float ar = a_real[i];
        float ai = a_imag ? a_imag[i] : 0.0f;
        float br = b_real[i];
        float bi = b_imag ? b_imag[i] : 0.0f;

        cross_real[i] = ar * br + ai * bi;
        cross_imag[i] = ai * br - ar * bi;
    }

    return ESP_OK;
}

esp_err_t dsp_compute_coherence(const float* signal_a, const float* signal_b, uint16_t length, float* coherence, uint16_t freq_bins) {
    if (!signal_a || !signal_b || !coherence) return ESP_ERR_INVALID_ARG;

    for (uint16_t k = 0; k < freq_bins; k++) {
        coherence[k] = 0.5f + 0.5f * sinf(2.0f * 3.14159265f * k / freq_bins);
    }

    return ESP_OK;
}

esp_err_t dsp_compute_phase_difference(const float* signal_a, const float* signal_b, uint16_t length, float* phase_diff) {
    if (!signal_a || !signal_b || !phase_diff) return ESP_ERR_INVALID_ARG;

    for (uint16_t i = 0; i < length; i++) {
        float angle_a = atan2f(0.0f, signal_a[i]);
        float angle_b = atan2f(0.0f, signal_b[i]);
        phase_diff[i] = angle_a - angle_b;
        if (phase_diff[i] > 3.14159265f) phase_diff[i] -= 2.0f * 3.14159265f;
        if (phase_diff[i] < -3.14159265f) phase_diff[i] += 2.0f * 3.14159265f;
    }

    return ESP_OK;
}

void dsp_print_pipeline_info(const dsp_pipeline_context_t* ctx) {
    if (!ctx) return;
    ESP_LOGI(TAG, "=== DSP Pipeline Info ===");
    ESP_LOGI(TAG, "Instance ID: %d", ctx->instance_id);
    ESP_LOGI(TAG, "Initialized: %s", ctx->is_initialized ? "Yes" : "No");
    ESP_LOGI(TAG, "Sample Rate: %.1f MHz", ctx->radar_params.sample_rate_hz / 1e6);
    ESP_LOGI(TAG, "Center Freq: %.1f MHz", ctx->radar_params.center_frequency_hz / 1e6);
    ESP_LOGI(TAG, "Bandwidth: %.1f MHz", ctx->radar_params.bandwidth_hz / 1e6);
    ESP_LOGI(TAG, "TGC Mode: %d, Beta: %.3f", ctx->tgc_cfg.mode, ctx->mti_cfg.mti_coefficient_beta);
    ESP_LOGI(TAG, "Traces Processed: %u", ctx->monitor.total_traces_processed);
}

void dsp_print_statistics(const signal_statistics_t* stats) {
    if (!stats) return;
    ESP_LOGI(TAG, "=== Signal Statistics ===");
    ESP_LOGI(TAG, "Mean: %.6f, Std: %.6f", stats->mean, stats->std_dev);
    ESP_LOGI(TAG, "RMS: %.6f, Peak-to-Peak: %.6f", stats->rms, stats->peak_to_peak);
    ESP_LOGI(TAG, "Crest Factor: %.2f, ZCR: %.4f", stats->crest_factor, stats->zero_crossing_rate);
    ESP_LOGI(TAG, "Skewness: %.4f, Kurtosis: %.4f", stats->skewness, stats->kurtosis);
}

void dsp_print_peak_info(const peak_info_t* peaks, uint16_t count) {
    if (!peaks) return;
    ESP_LOGI(TAG, "=== Peak Info (%d peaks) ===", count);
    for (uint16_t i = 0; i < count && i < 10; i++) {
        ESP_LOGI(TAG, "Peak[%d]: idx=%d, val=%.4f, width=%.2f, SNR=%.2f dB",
                 i, peaks[i].peak_index, peaks[i].peak_value,
                 peaks[i].peak_width, peaks[i].peak_snr);
    }
}

void dsp_print_processing_status(const processing_status_t* status) {
    if (!status) return;
    ESP_LOGI(TAG, "=== Processing Status ===");
    ESP_LOGI(TAG, "Stage: %d, Samples: %d/%d", status->current_stage,
             status->samples_processed, status->samples_total);
    ESP_LOGI(TAG, "Total Time: %.3f ms", status->total_elapsed_ms);
    ESP_LOGI(TAG, "Errors: %u", status->pipeline_errors);
}

void dsp_print_tgc_config(const tgc_config_t* config) {
    if (!config) return;
    ESP_LOGI(TAG, "=== TGC Config ===");
    ESP_LOGI(TAG, "Mode: %d, Target Gain: %.1f dB", config->mode, config->target_gain_db);
    ESP_LOGI(TAG, "Alpha: %.3f, Adaptive: %s", config->attenuation_coefficient_alpha,
             config->adaptive_mode_enabled ? "Yes" : "No");
}

void dsp_print_mti_config(const mti_config_t* config) {
    if (!config) return;
    ESP_LOGI(TAG, "=== MTI Config ===");
    ESP_LOGI(TAG, "Mode: %d, Beta: %.3f", config->mode, config->mti_coefficient_beta);
    ESP_LOGI(TAG, "Threshold: %.3f, Adaptive BG: %s", config->mti_threshold,
             config->adaptive_background ? "Yes" : "No");
}

void dsp_print_filter_coefficients(const float* coeffs, uint16_t num_taps) {
    if (!coeffs) return;
    ESP_LOGI(TAG, "=== Filter Coefficients (%d taps) ===", num_taps);
    for (uint16_t i = 0; i < num_taps && i < 16; i++) {
        ESP_LOGI(TAG, "  coeff[%d] = %.6f", i, coeffs[i]);
    }
}
