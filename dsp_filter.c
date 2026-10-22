#include "dsp_pipeline.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_dsp.hh"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

static const char* TAG = "DSP_FILTER";

#define MAX_FILTER_STAGES 16
#define MAX_FILTER_ORDER 256
#define FILTER_COEFF_PRECISION 0.00001f

typedef struct {
    float* b_coeffs;
    float* a_coeffs;
    float* x_delay;
    float* y_delay;
    uint16_t num_sections;
    uint16_t max_order;
    bool is_biquad;
    bool is_adaptive;
    float adaptation_mu;
    float adaptation_gamma;
    float norm_factor;
} iir_filter_bank_t;

typedef struct {
    float* coefficients;
    float* state_real;
    float* state_imag;
    uint16_t num_taps;
    uint16_t fft_size;
    bool is_complex;
    uint32_t magic;
} overlap_save_filter_t;

typedef struct {
    float* filter_coefficients;
    float* overlap_buffer;
    float* temp_buffer;
    uint16_t block_size;
    uint16_t filter_length;
    uint16_t fft_size;
    uint16_t overlap_length;
    bool is_initialized;
} overlap_add_filter_t;

typedef struct {
    float* matched_template;
    uint16_t template_length;
    float correlation_threshold;
    float snr_threshold;
    float pulse_width_min;
    float pulse_width_max;
    bool template_valid;
} matched_filter_bank_t;

typedef struct {
    float* filter_coeff_lp;
    float* filter_coeff_hp;
    float* filter_coeff_bp;
    float* filter_coeff_bs;
    uint16_t filter_order;
    uint16_t current_mode;
    bool is_parallel_mode;
    float parallel_gain_lp;
    float parallel_gain_hp;
    float parallel_gain_bp;
    float parallel_gain_bs;
} multi_mode_filter_t;

typedef struct {
    float current_value;
    float previous_value;
    float alpha;
    float beta;
    float gamma;
    float delta;
    uint16_t initialization_samples;
    uint16_t sample_count;
    bool is_stabilized;
} adaptive_filter_state_single_t;

typedef struct {
    float cutoff_frequency;
    float sampling_frequency;
    float q_factor;
    float gain_db;
    float transition_bw;
    float ripple_db;
    float stopband_atten;
    bool is_high_order;
    uint8_t filter_sections;
    float* section_coeffs;
} filter_design_params_t;

typedef struct {
    float input;
    float output;
    float delayed_input[8];
    float delayed_output[8];
    float coeff_a[8];
    float coeff_b[8];
    uint8_t delay_index;
    uint8_t num_sections;
} biquad_cascade_t;

typedef struct {
    float lower_band;
    float upper_band;
    float transition_width;
    float passband_ripple;
    float stopband_attenuation;
    uint8_t filter_type;
    uint8_t window_function;
    uint16_t filter_order;
    float* amplitude_response;
    float* phase_response;
    float* group_delay;
    uint16_t num_points;
} filter_characteristics_t;

typedef struct {
    float* real_coeffs;
    float* imag_coeffs;
    uint16_t num_taps;
    bool is_symmetric;
    bool is_anti_symmetric;
    float norm_factor;
} complex_filter_t;

typedef struct {
    float* fir_coeffs_low;
    float* fir_coeffs_high;
    uint16_t fir_order_low;
    uint16_t fir_order_high;
    uint16_t decomposition_level;
    bool is_perfect_reconstruction;
    float reconstruction_error;
} wavelet_decomp_filter_t;

typedef struct {
    float bandwidth_low;
    float bandwidth_mid;
    float bandwidth_high;
    float center_freq_low;
    float center_freq_mid;
    float center_freq_high;
    float quality_low;
    float quality_mid;
    float quality_high;
    float gain_low;
    float gain_mid;
    float gain_high;
    uint8_t num_bands;
} multi_band_equalizer_t;

typedef struct {
    float* kalman_gain;
    float* estimate;
    float* prediction_error;
    float process_noise;
    float measurement_noise;
    float error_covariance;
    uint16_t state_dimension;
    uint16_t measurement_dimension;
    bool is_initialized;
} kalman_filter_t;

typedef struct {
    float threshold_low;
    float threshold_mid;
    float threshold_high;
    float alpha_adapt;
    float beta_adapt;
    float adaptation_rate;
    uint8_t compression_ratio;
    uint8_t expansion_ratio;
    float makeup_gain;
    float knee_softness;
    bool enable_limiter;
    float limiter_threshold;
} dynamic_filter_t;

typedef struct {
    float spectral_centroid;
    float spectral_bandwidth;
    float spectral_contrast_low;
    float spectral_contrast_mid;
    float spectral_contrast_high;
    float spectral_flatness_measure;
    float spectral_rolloff_point;
    float spectral_skewness;
    float spectral_kurtosis;
    float temporal_centroid;
    float temporal_bandwidth;
    float temporal_flux;
} audio_features_t;

typedef struct {
    float current_value;
    float peak_value;
    float rms_value;
    float average_value;
    float variance_value;
    float peak_hold;
    float peak_decay_rate;
    uint64_t sample_count;
    float max_peak;
    float min_valley;
} envelope_follower_t;

typedef struct {
    float* fir_coefficients;
    float* iir_coefficients;
    float* state_fir;
    float* state_iir;
    uint16_t fir_order;
    uint16_t iir_order;
    uint8_t num_stages;
    float overall_gain;
    bool is_combined;
} hybrid_filter_t;

typedef struct {
    float f0;
    float f1;
    float f2;
    float Q1;
    float Q2;
    float A1;
    float A2;
    float A3;
    float w0;
    float cos_w0;
    float sin_w0;
    float alpha;
    float beta;
} parametric_eq_band_t;

typedef struct {
    parametric_eq_band_t low_shelf;
    parametric_eq_band_t parametric1;
    parametric_eq_band_t parametric2;
    parametric_eq_band_t high_shelf;
    uint8_t num_bands;
    float master_gain;
    float output_level;
    float clip_threshold;
} parametric_eq_t;

static iir_filter_bank_t s_iir_banks[MAX_FILTER_STAGES];
static uint8_t s_iir_bank_count = 0;
static matched_filter_bank_t s_matched_bank;
static kalman_filter_t s_kalman_filter;
static adaptive_filter_state_single_t s_adaptive_states[MAX_FILTER_STAGES];
static uint8_t s_adaptive_state_count = 0;

static inline float clamp_value(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

static float compute_sigmoid(float x, float slope, float center) {
    return 1.0f / (1.0f + expf(-slope * (x - center)));
}

esp_err_t dsp_fir_filter_init(float* coefficients, uint16_t num_taps) {
    if (!coefficients || num_taps == 0 || num_taps > MAX_FILTER_ORDER) {
        ESP_LOGE(TAG, "Invalid FIR filter parameters");
        return ESP_ERR_INVALID_ARG;
    }

    float sum = 0.0f;
    for (uint16_t i = 0; i < num_taps; i++) {
        sum += coefficients[i];
    }

    if (fabsf(sum) > FILTER_COEFF_PRECISION) {
        float norm_factor = 1.0f / sum;
        for (uint16_t i = 0; i < num_taps; i++) {
            coefficients[i] *= norm_factor;
        }
    }

    ESP_LOGI(TAG, "FIR filter initialized with %d taps", num_taps);
    return ESP_OK;
}

esp_err_t dsp_fir_filter_execute(const float* input, float* output, uint16_t length,
                                  const float* coefficients, uint16_t num_taps) {
    if (!input || !output || !coefficients) {
        return ESP_ERR_INVALID_ARG;
    }
    if (num_taps == 0 || num_taps > MAX_FILTER_ORDER) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint16_t delay = num_taps / 2;

    for (uint16_t i = 0; i < length; i++) {
        float sum = 0.0f;
        uint16_t start = (i >= delay) ? (i - delay) : 0;

        for (uint16_t j = 0; j < num_taps && (start + j) < length; j++) {
            sum += input[start + j] * coefficients[num_taps - 1 - j];
        }

        output[i] = sum;
    }

    return ESP_OK;
}

esp_err_t dsp_fir_filter_execute_block(const float* input, float* output, uint16_t block_size,
                                        const float* coefficients, uint16_t num_taps) {
    return dsp_fir_filter_execute(input, output, block_size, coefficients, num_taps);
}

esp_err_t dsp_iir_filter_init(const float* b_coeffs, const float* a_coeffs, uint8_t sections) {
    if (!b_coeffs || !a_coeffs) {
        return ESP_ERR_INVALID_ARG;
    }
    if (sections == 0 || sections > 8) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_iir_bank_count >= MAX_FILTER_STAGES) {
        ESP_LOGE(TAG, "IIR filter bank limit reached");
        return ESP_ERR_NO_MEM;
    }

    iir_filter_bank_t* bank = &s_iir_banks[s_iir_bank_count];

    bank->b_coeffs = (float*)heap_caps_malloc(sections * 3 * sizeof(float), MALLOC_CAP_SPIRAM);
    bank->a_coeffs = (float*)heap_caps_malloc(sections * 3 * sizeof(float), MALLOC_CAP_SPIRAM);
    bank->x_delay = (float*)heap_caps_malloc(sections * 3 * sizeof(float), MALLOC_CAP_SPIRAM);
    bank->y_delay = (float*)heap_caps_malloc(sections * 3 * sizeof(float), MALLOC_CAP_SPIRAM);

    if (!bank->b_coeffs || !bank->a_coeffs || !bank->x_delay || !bank->y_delay) {
        ESP_LOGE(TAG, "IIR filter bank memory allocation failed");
        return ESP_ERR_NO_MEM;
    }

    bank->num_sections = sections;
    bank->max_order = sections * 2;
    bank->is_biquad = true;
    bank->is_adaptive = false;
    bank->adaptation_mu = 0.001f;
    bank->adaptation_gamma = 0.999f;
    bank->norm_factor = 1.0f;

    for (uint8_t s = 0; s < sections; s++) {
        uint8_t idx = s * 3;
        bank->b_coeffs[idx] = b_coeffs[s * 3];
        bank->b_coeffs[idx + 1] = b_coeffs[s * 3 + 1];
        bank->b_coeffs[idx + 2] = b_coeffs[s * 3 + 2];
        bank->a_coeffs[idx] = a_coeffs[s * 3];
        bank->a_coeffs[idx + 1] = a_coeffs[s * 3 + 1];
        bank->a_coeffs[idx + 2] = a_coeffs[s * 3 + 2];

        bank->x_delay[idx] = 0.0f;
        bank->x_delay[idx + 1] = 0.0f;
        bank->x_delay[idx + 2] = 0.0f;
        bank->y_delay[idx] = 0.0f;
        bank->y_delay[idx + 1] = 0.0f;
        bank->y_delay[idx + 2] = 0.0f;
    }

    s_iir_bank_count++;

    ESP_LOGI(TAG, "IIR filter bank %d initialized with %d sections", s_iir_bank_count - 1, sections);
    return ESP_OK;
}

esp_err_t dsp_iir_filter_execute(const float* input, float* output, uint16_t length,
                                  const float* b_coeffs, const float* a_coeffs, uint8_t sections) {
    if (!input || !output || !b_coeffs || !a_coeffs) {
        return ESP_ERR_INVALID_ARG;
    }

    float x_delay[8 * 3] = {0};
    float y_delay[8 * 3] = {0};

    for (uint16_t i = 0; i < length; i++) {
        float x = input[i];
        float y = 0.0f;

        uint8_t effective_sections = (sections > 8) ? 8 : sections;

        for (uint8_t s = 0; s < effective_sections; s++) {
            uint8_t idx = s * 3;

            x_delay[idx + 2] = x_delay[idx + 1];
            x_delay[idx + 1] = x_delay[idx];
            x_delay[idx] = (s == 0) ? x : y;

            y_delay[idx + 2] = y_delay[idx + 1];
            y_delay[idx + 1] = y_delay[idx];
            y_delay[idx] = 0.0f;

            float b0 = b_coeffs[idx];
            float b1 = b_coeffs[idx + 1];
            float b2 = b_coeffs[idx + 2];
            float a1 = a_coeffs[idx + 1];
            float a2 = a_coeffs[idx + 2];

            if (fabsf(a_coeffs[idx]) > FILTER_COEFF_PRECISION) {
                float norm = 1.0f / a_coeffs[idx];
                b0 *= norm;
                b1 *= norm;
                b2 *= norm;
                a1 *= norm;
                a2 *= norm;
            }

            y_delay[idx] = b0 * x_delay[idx] + b1 * x_delay[idx + 1] + b2 * x_delay[idx + 2]
                         - a1 * y_delay[idx + 1] - a2 * y_delay[idx + 2];

            y = y_delay[idx];
        }

        output[i] = y;
    }

    return ESP_OK;
}

esp_err_t dsp_design_bandpass_filter(float* coefficients, uint16_t* num_taps,
                                      float low_cutoff, float high_cutoff,
                                      float sample_rate, uint16_t order,
                                      window_type_t window_type) {
    if (!coefficients || !num_taps) {
        return ESP_ERR_INVALID_ARG;
    }

    if (low_cutoff <= 0.0f || high_cutoff <= 0.0f || low_cutoff >= high_cutoff) {
        ESP_LOGE(TAG, "Invalid cutoff frequencies: low=%.2f, high=%.2f", low_cutoff, high_cutoff);
        return ESP_ERR_INVALID_ARG;
    }

    if (low_cutoff > sample_rate * 0.5f || high_cutoff > sample_rate * 0.5f) {
        ESP_LOGE(TAG, "Cutoff frequencies exceed Nyquist rate");
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t n = (order > 0) ? order : 64;
    if (n % 2 == 0) n++;

    *num_taps = n;

    float wc_low = 2.0f * (float)M_PI * low_cutoff / sample_rate;
    float wc_high = 2.0f * (float)M_PI * high_cutoff / sample_rate;
    float wc_center = (wc_low + wc_high) / 2.0f;
    float wc_width = wc_high - wc_low;

    for (uint16_t i = 0; i < n; i++) {
        float m = (float)i - (float)(n - 1) / 2.0f;

        if ((int)m == 0) {
            coefficients[i] = (wc_high - wc_low) / (float)M_PI;
        } else {
            float term1 = sinf(wc_high * m) / (float)M_PI / m;
            float term2 = sinf(wc_low * m) / (float)M_PI / m;
            coefficients[i] = term1 - term2;
        }

        float window = 1.0f;
        float n_norm = (float)i / (float)(n - 1);

        switch (window_type) {
            case WINDOW_TYPE_HANNING:
                window = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * n_norm);
                break;
            case WINDOW_TYPE_HAMMING:
                window = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * n_norm);
                break;
            case WINDOW_TYPE_BLACKMAN:
                window = 0.42f - 0.5f * cosf(2.0f * (float)M_PI * n_norm)
                         + 0.08f * cosf(4.0f * (float)M_PI * n_norm);
                break;
            case WINDOW_TYPE_KAISER: {
                float beta = 8.0f;
                float alpha = beta * sqrtf(1.0f - powf(2.0f * n_norm - 1.0f, 2.0f));
                window = sinf((float)M_PI * alpha) / ((float)M_PI * alpha + 1e-10f);
                break;
            }
            case WINDOW_TYPE_BARTLETT:
                window = 1.0f - fabsf(2.0f * n_norm - 1.0f);
                break;
            case WINDOW_TYPE_WELCH:
                window = 1.0f - powf(2.0f * n_norm - 1.0f, 2.0f);
                break;
            default:
                window = 1.0f;
                break;
        }

        coefficients[i] *= window;
    }

    float sum = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        sum += coefficients[i];
    }
    if (fabsf(sum) > FILTER_COEFF_PRECISION) {
        float norm = 1.0f / sum;
        for (uint16_t i = 0; i < n; i++) {
            coefficients[i] *= norm;
        }
    }

    ESP_LOGI(TAG, "Bandpass filter designed: order=%d, low=%.2fHz, high=%.2fHz",
             n, low_cutoff, high_cutoff);

    return ESP_OK;
}

esp_err_t dsp_design_lowpass_filter(float* coefficients, uint16_t* num_taps,
                                     float cutoff_freq, float sample_rate,
                                     uint16_t order, window_type_t window_type) {
    if (!coefficients || !num_taps) {
        return ESP_ERR_INVALID_ARG;
    }

    if (cutoff_freq <= 0.0f || cutoff_freq > sample_rate * 0.5f) {
        ESP_LOGE(TAG, "Invalid cutoff frequency: %.2f Hz", cutoff_freq);
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t n = (order > 0) ? order : 64;
    if (n % 2 == 0) n++;

    *num_taps = n;

    float wc = 2.0f * (float)M_PI * cutoff_freq / sample_rate;

    for (uint16_t i = 0; i < n; i++) {
        float m = (float)i - (float)(n - 1) / 2.0f;

        if ((int)m == 0) {
            coefficients[i] = wc / (float)M_PI;
        } else {
            coefficients[i] = sinf(wc * m) / ((float)M_PI * m);
        }

        float window = 1.0f;
        float n_norm = (float)i / (float)(n - 1);

        switch (window_type) {
            case WINDOW_TYPE_HANNING:
                window = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * n_norm);
                break;
            case WINDOW_TYPE_HAMMING:
                window = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * n_norm);
                break;
            case WINDOW_TYPE_BLACKMAN:
                window = 0.42f - 0.5f * cosf(2.0f * (float)M_PI * n_norm)
                         + 0.08f * cosf(4.0f * (float)M_PI * n_norm);
                break;
            default:
                window = 1.0f;
                break;
        }

        coefficients[i] *= window;
    }

    float sum = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        sum += coefficients[i];
    }
    if (fabsf(sum) > FILTER_COEFF_PRECISION) {
        float norm = 1.0f / sum;
        for (uint16_t i = 0; i < n; i++) {
            coefficients[i] *= norm;
        }
    }

    return ESP_OK;
}

esp_err_t dsp_design_highpass_filter(float* coefficients, uint16_t* num_taps,
                                      float cutoff_freq, float sample_rate,
                                      uint16_t order, window_type_t window_type) {
    if (!coefficients || !num_taps) {
        return ESP_ERR_INVALID_ARG;
    }

    if (cutoff_freq <= 0.0f || cutoff_freq > sample_rate * 0.5f) {
        ESP_LOGE(TAG, "Invalid cutoff frequency: %.2f Hz", cutoff_freq);
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t n = (order > 0) ? order : 64;
    if (n % 2 == 0) n++;

    *num_taps = n;

    float wc = 2.0f * (float)M_PI * cutoff_freq / sample_rate;

    for (uint16_t i = 0; i < n; i++) {
        float m = (float)i - (float)(n - 1) / 2.0f;

        if ((int)m == 0) {
            coefficients[i] = 1.0f - wc / (float)M_PI;
        } else {
            coefficients[i] = -sinf(wc * m) / ((float)M_PI * m);
        }

        float window = 1.0f;
        float n_norm = (float)i / (float)(n - 1);

        switch (window_type) {
            case WINDOW_TYPE_HANNING:
                window = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * n_norm);
                break;
            case WINDOW_TYPE_HAMMING:
                window = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * n_norm);
                break;
            case WINDOW_TYPE_BLACKMAN:
                window = 0.42f - 0.5f * cosf(2.0f * (float)M_PI * n_norm)
                         + 0.08f * cosf(4.0f * (float)M_PI * n_norm);
                break;
            default:
                window = 1.0f;
                break;
        }

        coefficients[i] *= window;

        if (i % 2 == 1) {
            coefficients[i] = -coefficients[i];
        }
    }

    float sum = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        sum += coefficients[i];
    }
    if (fabsf(sum) > FILTER_COEFF_PRECISION) {
        float norm = 1.0f / sum;
        for (uint16_t i = 0; i < n; i++) {
            coefficients[i] *= norm;
        }
    }

    return ESP_OK;
}

esp_err_t dsp_apply_filter(const float* input, float* output, uint16_t length,
                           const float* coefficients, uint16_t num_taps) {
    return dsp_fir_filter_execute(input, output, length, coefficients, num_taps);
}

esp_err_t dsp_create_notch_filter(float* b_coeffs, float* a_coeffs,
                                    float center_freq, float sample_rate,
                                    float q_factor) {
    if (!b_coeffs || !a_coeffs) {
        return ESP_ERR_INVALID_ARG;
    }

    float w0 = 2.0f * (float)M_PI * center_freq / sample_rate;
    float alpha = sinf(w0) / (2.0f * q_factor);
    float cos_w0 = cosf(w0);

    b_coeffs[0] = 1.0f;
    b_coeffs[1] = -2.0f * cos_w0;
    b_coeffs[2] = 1.0f;

    a_coeffs[0] = 1.0f + alpha;
    a_coeffs[1] = -2.0f * cos_w0;
    a_coeffs[2] = 1.0f - alpha;

    float norm = 1.0f / a_coeffs[0];
    for (uint8_t i = 0; i < 3; i++) {
        b_coeffs[i] *= norm;
        a_coeffs[i] *= norm;
    }

    ESP_LOGI(TAG, "Notch filter created: f0=%.1fHz, Q=%.2f", center_freq, q_factor);

    return ESP_OK;
}

esp_err_t dsp_create_peaking_eq_filter(float* b_coeffs, float* a_coeffs,
                                        float center_freq, float sample_rate,
                                        float gain_db, float q_factor) {
    if (!b_coeffs || !a_coeffs) {
        return ESP_ERR_INVALID_ARG;
    }

    float A = powf(10.0f, gain_db / 40.0f);
    float w0 = 2.0f * (float)M_PI * center_freq / sample_rate;
    float alpha = sinf(w0) / (2.0f * q_factor);
    float cos_w0 = cosf(w0);

    b_coeffs[0] = 1.0f + alpha * A;
    b_coeffs[1] = -2.0f * cos_w0;
    b_coeffs[2] = 1.0f - alpha * A;

    a_coeffs[0] = 1.0f + alpha / A;
    a_coeffs[1] = -2.0f * cos_w0;
    a_coeffs[2] = 1.0f - alpha / A;

    float norm = 1.0f / a_coeffs[0];
    for (uint8_t i = 0; i < 3; i++) {
        b_coeffs[i] *= norm;
        a_coeffs[i] *= norm;
    }

    ESP_LOGI(TAG, "Peaking EQ created: f0=%.1fHz, gain=%.1fdB, Q=%.2f",
             center_freq, gain_db, q_factor);

    return ESP_OK;
}

esp_err_t dsp_create_lowshelf_filter(float* b_coeffs, float* a_coeffs,
                                      float cutoff_freq, float sample_rate,
                                      float gain_db) {
    if (!b_coeffs || !a_coeffs) {
        return ESP_ERR_INVALID_ARG;
    }

    float A = powf(10.0f, gain_db / 40.0f);
    float w0 = 2.0f * (float)M_PI * cutoff_freq / sample_rate;
    float cos_w0 = cosf(w0);
    float sin_w0 = sinf(w0);
    float alpha = sin_w0 / 2.0f * sqrtf((A + 1.0f / A) * (1.0f / 0.9f - 1.0f) + 2.0f);

    float sqrt_A_2 = 2.0f * sqrtf(A) * alpha;

    b_coeffs[0] = A * ((A + 1.0f) - (A - 1.0f) * cos_w0 + sqrt_A_2);
    b_coeffs[1] = 2.0f * A * ((A - 1.0f) - (A + 1.0f) * cos_w0);
    b_coeffs[2] = A * ((A + 1.0f) - (A - 1.0f) * cos_w0 - sqrt_A_2);

    a_coeffs[0] = (A + 1.0f) + (A - 1.0f) * cos_w0 + sqrt_A_2;
    a_coeffs[1] = -2.0f * ((A - 1.0f) + (A + 1.0f) * cos_w0);
    a_coeffs[2] = (A + 1.0f) + (A - 1.0f) * cos_w0 - sqrt_A_2;

    float norm = 1.0f / a_coeffs[0];
    for (uint8_t i = 0; i < 3; i++) {
        b_coeffs[i] *= norm;
        a_coeffs[i] *= norm;
    }

    return ESP_OK;
}

esp_err_t dsp_create_highshelf_filter(float* b_coeffs, float* a_coeffs,
                                       float cutoff_freq, float sample_rate,
                                       float gain_db) {
    if (!b_coeffs || !a_coeffs) {
        return ESP_ERR_INVALID_ARG;
    }

    float A = powf(10.0f, gain_db / 40.0f);
    float w0 = 2.0f * (float)M_PI * cutoff_freq / sample_rate;
    float cos_w0 = cosf(w0);
    float sin_w0 = sinf(w0);
    float alpha = sin_w0 / 2.0f * sqrtf((A + 1.0f / A) * (1.0f / 0.9f - 1.0f) + 2.0f);

    float sqrt_A_2 = 2.0f * sqrtf(A) * alpha;

    b_coeffs[0] = A * ((A + 1.0f) + (A - 1.0f) * cos_w0 + sqrt_A_2);
    b_coeffs[1] = -2.0f * A * ((A - 1.0f) + (A + 1.0f) * cos_w0);
    b_coeffs[2] = A * ((A + 1.0f) + (A - 1.0f) * cos_w0 - sqrt_A_2);

    a_coeffs[0] = (A + 1.0f) - (A - 1.0f) * cos_w0 + sqrt_A_2;
    a_coeffs[1] = 2.0f * ((A - 1.0f) - (A + 1.0f) * cos_w0);
    a_coeffs[2] = (A + 1.0f) - (A - 1.0f) * cos_w0 - sqrt_A_2;

    float norm = 1.0f / a_coeffs[0];
    for (uint8_t i = 0; i < 3; i++) {
        b_coeffs[i] *= norm;
        a_coeffs[i] *= norm;
    }

    return ESP_OK;
}

esp_err_t dsp_create_allpass_filter(float* b_coeffs, float* a_coeffs,
                                     float center_freq, float sample_rate,
                                     float q_factor) {
    if (!b_coeffs || !a_coeffs) {
        return ESP_ERR_INVALID_ARG;
    }

    float w0 = 2.0f * (float)M_PI * center_freq / sample_rate;
    float alpha = sinf(w0) / (2.0f * q_factor);
    float cos_w0 = cosf(w0);

    b_coeffs[0] = 1.0f - alpha;
    b_coeffs[1] = -2.0f * cos_w0;
    b_coeffs[2] = 1.0f + alpha;

    a_coeffs[0] = 1.0f + alpha;
    a_coeffs[1] = -2.0f * cos_w0;
    a_coeffs[2] = 1.0f - alpha;

    float norm = 1.0f / a_coeffs[0];
    for (uint8_t i = 0; i < 3; i++) {
        b_coeffs[i] *= norm;
        a_coeffs[i] *= norm;
    }

    ESP_LOGI(TAG, "Allpass filter created: f0=%.1fHz, Q=%.2f", center_freq, q_factor);

    return ESP_OK;
}

esp_err_t dsp_apply_cascaded_biquads(const float* input, float* output, uint16_t length,
                                      float (*coeffs)[6], uint8_t num_biquads) {
    if (!input || !output || !coeffs) {
        return ESP_ERR_INVALID_ARG;
    }

    float x_delay[8] = {0};
    float y_delay[8] = {0};

    for (uint16_t i = 0; i < length; i++) {
        float x = input[i];
        float y = x;

        for (uint8_t s = 0; s < num_biquads && s < 8; s++) {
            for (uint8_t d = 2; d > 0; d--) {
                x_delay[s * 3 + d] = x_delay[s * 3 + d - 1];
                y_delay[s * 3 + d] = y_delay[s * 3 + d - 1];
            }

            x_delay[s * 3] = y;

            float b0 = coeffs[s][0];
            float b1 = coeffs[s][1];
            float b2 = coeffs[s][2];
            float a1 = coeffs[s][3];
            float a2 = coeffs[s][4];

            y_delay[s * 3] = b0 * x_delay[s * 3] + b1 * x_delay[s * 3 + 1] + b2 * x_delay[s * 3 + 2]
                           - a1 * y_delay[s * 3 + 1] - a2 * y_delay[s * 3 + 2];

            y = y_delay[s * 3];
        }

        output[i] = y;
    }

    return ESP_OK;
}

esp_err_t dsp_adapt_filter_lms(float* coefficients, uint16_t num_taps,
                                const float* input, const float* desired,
                                const float* error, float* output, uint16_t length,
                                float step_size) {
    if (!coefficients || !input || !desired || !error || !output) {
        return ESP_ERR_INVALID_ARG;
    }

    float* delay_line = (float*)heap_caps_malloc(num_taps * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!delay_line) return ESP_ERR_NO_MEM;

    for (uint16_t i = 0; i < num_taps; i++) {
        delay_line[i] = 0.0f;
    }

    float sum_error_sq = 0.0f;

    for (uint16_t n = 0; n < length; n++) {
        for (uint16_t k = 0; k < num_taps - 1; k++) {
            delay_line[k] = delay_line[k + 1];
        }
        delay_line[num_taps - 1] = input[n];

        float y = 0.0f;
        for (uint16_t k = 0; k < num_taps; k++) {
            y += coefficients[k] * delay_line[num_taps - 1 - k];
        }
        output[n] = y;

        float e = error[n] - y;
        sum_error_sq += e * e;

        float mu_eff = step_size / (1.0f + 0.01f * sum_error_sq / (float)(n + 1));

        for (uint16_t k = 0; k < num_taps; k++) {
            coefficients[k] += 2.0f * mu_eff * e * delay_line[num_taps - 1 - k];
        }
    }

    free(delay_line);

    return ESP_OK;
}

esp_err_t dsp_init_kalman_filter(uint16_t state_dim, uint16_t meas_dim,
                                   float process_noise, float measurement_noise) {
    s_kalman_filter.state_dimension = state_dim;
    s_kalman_filter.measurement_dimension = meas_dim;

    s_kalman_filter.estimate = (float*)calloc(state_dim, sizeof(float));
    s_kalman_filter.kalman_gain = (float*)calloc(state_dim * meas_dim, sizeof(float));
    s_kalman_filter.prediction_error = (float*)calloc(state_dim, sizeof(float));

    if (!s_kalman_filter.estimate || !s_kalman_filter.kalman_gain || !s_kalman_filter.prediction_error) {
        return ESP_ERR_NO_MEM;
    }

    s_kalman_filter.process_noise = process_noise;
    s_kalman_filter.measurement_noise = measurement_noise;
    s_kalman_filter.error_covariance = 1.0f;
    s_kalman_filter.is_initialized = true;

    ESP_LOGI(TAG, "Kalman filter initialized: state_dim=%d, meas_dim=%d",
             state_dim, meas_dim);

    return ESP_OK;
}

esp_err_t dsp_update_kalman_filter(float* measurement, float* state_estimate,
                                    float* covariance_estimate) {
    if (!s_kalman_filter.is_initialized || !measurement || !state_estimate) {
        return ESP_ERR_INVALID_STATE;
    }

    float S = s_kalman_filter.error_covariance + s_kalman_filter.measurement_noise;

    if (fabsf(S) < 1e-10f) S = 1e-10f;

    float K = s_kalman_filter.error_covariance / S;

    float innovation = *measurement - *state_estimate;
    *state_estimate += K * innovation;

    s_kalman_filter.error_covariance *= (1.0f - K);
    s_kalman_filter.error_covariance += s_kalman_filter.process_noise;

    if (covariance_estimate) {
        *covariance_estimate = s_kalman_filter.error_covariance;
    }

    return ESP_OK;
}

esp_err_t dsp_deinit_kalman_filter(void) {
    if (s_kalman_filter.estimate) {
        free(s_kalman_filter.estimate);
        s_kalman_filter.estimate = NULL;
    }
    if (s_kalman_filter.kalman_gain) {
        free(s_kalman_filter.kalman_gain);
        s_kalman_filter.kalman_gain = NULL;
    }
    if (s_kalman_filter.prediction_error) {
        free(s_kalman_filter.prediction_error);
        s_kalman_filter.prediction_error = NULL;
    }

    s_kalman_filter.is_initialized = false;

    return ESP_OK;
}

esp_err_t dsp_create_wavelet_filter(wavelet_decomp_filter_t* wavelet,
                                     const char* wavelet_name) {
    if (!wavelet) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(wavelet, 0, sizeof(wavelet_decomp_filter_t));

    if (strcmp(wavelet_name, "haar") == 0) {
        wavelet->fir_order_low = 2;
        wavelet->fir_order_high = 2;
        wavelet->filter_coeff_lp = (float*)malloc(2 * sizeof(float));
        wavelet->filter_coeff_hp = (float*)malloc(2 * sizeof(float));
        if (wavelet->filter_coeff_lp) {
            wavelet->filter_coeff_lp[0] = 0.70710678f;
            wavelet->filter_coeff_lp[1] = 0.70710678f;
        }
        if (wavelet->filter_coeff_hp) {
            wavelet->filter_coeff_hp[0] = -0.70710678f;
            wavelet->filter_coeff_hp[1] = 0.70710678f;
        }
    } else if (strcmp(wavelet_name, "db4") == 0) {
        wavelet->fir_order_low = 8;
        wavelet->fir_order_high = 8;
        wavelet->filter_coeff_lp = (float*)malloc(8 * sizeof(float));
        wavelet->filter_coeff_hp = (float*)malloc(8 * sizeof(float));
        if (wavelet->filter_coeff_lp) {
            wavelet->filter_coeff_lp[0] = -0.129409522550921f;
            wavelet->filter_coeff_lp[1] = 0.224143868042013f;
            wavelet->filter_coeff_lp[2] = 0.836516303737807f;
            wavelet->filter_coeff_lp[3] = 0.482962913144534f;
            wavelet->filter_coeff_lp[4] = -0.129409522550921f;
            wavelet->filter_coeff_lp[5] = -0.224143868042013f;
            wavelet->filter_coeff_lp[6] = 0.836516303737807f;
            wavelet->filter_coeff_lp[7] = -0.482962913144534f;
        }
        if (wavelet->filter_coeff_hp) {
            wavelet->filter_coeff_hp[0] = 0.482962913144534f;
            wavelet->filter_coeff_hp[1] = -0.836516303737807f;
            wavelet->filter_coeff_hp[2] = -0.224143868042013f;
            wavelet->filter_coeff_hp[3] = 0.129409522550921f;
            wavelet->filter_coeff_hp[4] = 0.482962913144534f;
            wavelet->filter_coeff_hp[5] = 0.836516303737807f;
            wavelet->filter_coeff_hp[6] = 0.224143868042013f;
            wavelet->filter_coeff_hp[7] = -0.129409522550921f;
        }
    } else {
        wavelet->fir_order_low = 4;
        wavelet->fir_order_high = 4;
        wavelet->filter_coeff_lp = (float*)malloc(4 * sizeof(float));
        wavelet->filter_coeff_hp = (float*)malloc(4 * sizeof(float));
        if (wavelet->filter_coeff_lp) {
            wavelet->filter_coeff_lp[0] = 0.0f;
            wavelet->filter_coeff_lp[1] = 0.5f;
            wavelet->filter_coeff_lp[2] = 1.0f;
            wavelet->filter_coeff_lp[3] = 0.5f;
        }
        if (wavelet->filter_coeff_hp) {
            wavelet->filter_coeff_hp[0] = -0.5f;
            wavelet->filter_coeff_hp[1] = 1.0f;
            wavelet->filter_coeff_hp[2] = -0.5f;
            wavelet->filter_coeff_hp[3] = 0.0f;
        }
    }

    wavelet->decomposition_level = 3;
    wavelet->is_perfect_reconstruction = true;
    wavelet->reconstruction_error = 0.0f;

    ESP_LOGI(TAG, "Wavelet filter created: %s, level=%d", wavelet_name, wavelet->decomposition_level);

    return ESP_OK;
}

esp_err_t dsp_create_multi_band_eq(multi_band_equalizer_t* eq,
                                    float sample_rate,
                                    uint8_t num_bands) {
    if (!eq) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(eq, 0, sizeof(multi_band_equalizer_t));

    eq->num_bands = (num_bands > 0 && num_bands <= 10) ? num_bands : 3;

    float total_bw = sample_rate / 2.0f;
    float band_width = total_bw / eq->num_bands;

    for (uint8_t b = 0; b < eq->num_bands; b++) {
        float center = band_width * (b + 0.5f);

        if (b == 0) {
            eq->bandwidth_low = band_width;
            eq->center_freq_low = center;
            eq->quality_low = center / band_width;
            eq->gain_low = 0.0f;
        } else if (b == 1) {
            eq->bandwidth_mid = band_width;
            eq->center_freq_mid = center;
            eq->quality_mid = center / band_width;
            eq->gain_mid = 0.0f;
        } else if (b == 2) {
            eq->bandwidth_high = band_width;
            eq->center_freq_high = center;
            eq->quality_high = center / band_width;
            eq->gain_high = 0.0f;
        }
    }

    ESP_LOGI(TAG, "Multi-band EQ created: %d bands", eq->num_bands);

    return ESP_OK;
}

esp_err_t dsp_apply_compressor(const float* input, float* output, uint16_t length,
                                float threshold_db, float ratio, float attack_coef,
                                float release_coef, float makeup_gain) {
    if (!input || !output) {
        return ESP_ERR_INVALID_ARG;
    }

    float threshold_lin = powf(10.0f, threshold_db / 20.0f);
    float makeup_lin = powf(10.0f, makeup_gain / 20.0f);

    float envelope = 0.0f;

    for (uint16_t i = 0; i < length; i++) {
        float x = input[i];
        float x_abs = fabsf(x);

        if (x_abs > envelope) {
            envelope = attack_coef * envelope + (1.0f - attack_coef) * x_abs;
        } else {
            envelope = release_coef * envelope + (1.0f - release_coef) * x_abs;
        }

        float gain_reduction = 1.0f;

        if (envelope > threshold_lin) {
            float over_db = 20.0f * log10f((envelope + 1e-10f) / threshold_lin);
            float compressed_db = over_db / ratio;
            gain_reduction = powf(10.0f, (compressed_db - over_db) / 20.0f);
        }

        output[i] = x * gain_reduction * makeup_lin;

        if (output[i] > 0.99f) output[i] = 0.99f;
        if (output[i] < -0.99f) output[i] = -0.99f;
    }

    return ESP_OK;
}

esp_err_t dsp_apply_gate(const float* input, float* output, uint16_t length,
                          float threshold_db, float attack_ms, float hold_ms,
                          float release_ms, float sample_rate) {
    if (!input || !output) {
        return ESP_ERR_INVALID_ARG;
    }

    float threshold_lin = powf(10.0f, threshold_db / 20.0f);

    float attack_coef = expf(-1.0f / (attack_ms * sample_rate / 1000.0f));
    float release_coef = expf(-1.0f / (release_ms * sample_rate / 1000.0f));

    float envelope = 0.0f;
    uint16_t hold_counter = 0;
    uint16_t hold_samples = (uint16_t)(hold_ms * sample_rate / 1000.0f);
    bool gate_open = false;

    for (uint16_t i = 0; i < length; i++) {
        float x = input[i];
        float x_abs = fabsf(x);

        if (x_abs > envelope) {
            envelope = attack_coef * envelope + (1.0f - attack_coef) * x_abs;
        } else {
            envelope = release_coef * envelope + (1.0f - release_coef) * x_abs;
        }

        if (envelope > threshold_lin) {
            gate_open = true;
            hold_counter = hold_samples;
        } else if (hold_counter > 0) {
            hold_counter--;
        } else {
            gate_open = false;
        }

        output[i] = gate_open ? x : 0.0f;
    }

    return ESP_OK;
}

esp_err_t dsp_apply_limiter(const float* input, float* output, uint16_t length,
                              float threshold_db, float release_ms, float sample_rate) {
    if (!input || !output) {
        return ESP_ERR_INVALID_ARG;
    }

    float threshold_lin = powf(10.0f, threshold_db / 20.0f);
    float release_coef = expf(-1.0f / (release_ms * sample_rate / 1000.0f));

    float gain = 1.0f;

    for (uint16_t i = 0; i < length; i++) {
        float x = input[i];
        float abs_x = fabsf(x);

        float desired_gain = threshold_lin / (abs_x + 1e-10f);
        if (desired_gain < 1.0f) {
            if (desired_gain < gain) {
                gain = desired_gain;
            } else {
                gain = release_coef * gain + (1.0f - release_coef) * desired_gain;
            }
        } else {
            gain = release_coef * gain + (1.0f - release_coef) * 1.0f;
        }

        output[i] = x * gain;

        if (output[i] > 0.99f) output[i] = 0.99f;
        if (output[i] < -0.99f) output[i] = -0.99f;
    }

    return ESP_OK;
}

esp_err_t dsp_apply_spectral_subtraction(const float* input, float* output, uint16_t length,
                                           float noise_floor_db, float reduction_db,
                                           uint16_t fft_size, float sample_rate) {
    if (!input || !output) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t hop_size = fft_size / 4;
    uint16_t num_frames = (length >= fft_size) ? ((length - fft_size) / hop_size + 1) : 1;

    float* frame = (float*)malloc(fft_size * sizeof(float));
    float* window = (float*)malloc(fft_size * sizeof(float));
    float* spectrum_mag = (float*)malloc(fft_size / 2 * sizeof(float));
    float* noise_estimate = (float*)malloc(fft_size / 2 * sizeof(float));
    float* processed = (float*)malloc(length * sizeof(float));

    if (!frame || !window || !spectrum_mag || !noise_estimate || !processed) {
        if (frame) free(frame);
        if (window) free(window);
        if (spectrum_mag) free(spectrum_mag);
        if (noise_estimate) free(noise_estimate);
        if (processed) free(processed);
        return ESP_ERR_NO_MEM;
    }

    for (uint16_t i = 0; i < fft_size; i++) {
        float n_norm = (float)i / (float)(fft_size - 1);
        window[i] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * n_norm);
    }

    for (uint16_t f = 0; f < fft_size / 2; f++) {
        noise_estimate[f] = 0.01f;
    }

    for (uint16_t f = 0; f < num_frames; f++) {
        uint16_t start = f * hop_size;

        for (uint16_t i = 0; i < fft_size && (start + i) < length; i++) {
            frame[i] = input[start + i] * window[i];
        }
        for (uint16_t i = length - start; i < fft_size; i++) {
            frame[i] = 0.0f;
        }

        for (uint16_t i = 0; i < fft_size / 2; i++) {
            float mag = fabsf(frame[i]);
            float reduction = powf(10.0f, reduction_db / 20.0f);

            if (mag > noise_estimate[i] * 2.0f) {
                spectrum_mag[i] = mag - noise_estimate[i] * reduction;
                if (spectrum_mag[i] < 0.0f) spectrum_mag[i] = 0.0f;
            } else {
                spectrum_mag[i] = noise_estimate[i];
            }

            noise_estimate[i] = 0.99f * noise_estimate[i] + 0.01f * spectrum_mag[i];
        }

        for (uint16_t i = 0; i < fft_size && (start + i) < length; i++) {
            processed[start + i] += frame[i] * window[i] / num_frames;
        }
    }

    memcpy(output, processed, length * sizeof(float));

    free(frame);
    free(window);
    free(spectrum_mag);
    free(noise_estimate);
    free(processed);

    return ESP_OK;
}

esp_err_t dsp_apply_wiener_filter(const float* input, float* output, uint16_t length,
                                    float snr_estimate_db, uint16_t filter_length) {
    if (!input || !output) {
        return ESP_ERR_INVALID_ARG;
    }

    float snr_linear = powf(10.0f, snr_estimate_db / 10.0f);

    float* filter_coeff = (float*)malloc(filter_length * sizeof(float));
    if (!filter_coeff) return ESP_ERR_NO_MEM;

    float noise_variance = 0.01f;
    float signal_variance = noise_variance * snr_linear;

    for (uint16_t i = 0; i < filter_length; i++) {
        float n_norm = (float)i / (float)(filter_length - 1) - 0.5f;
        filter_coeff[i] = expf(-n_norm * n_norm * 2.0f);
    }

    float sum = 0.0f;
    for (uint16_t i = 0; i < filter_length; i++) {
        sum += filter_coeff[i];
    }
    for (uint16_t i = 0; i < filter_length; i++) {
        filter_coeff[i] /= sum;
    }

    dsp_fir_filter_execute(input, output, length, filter_coeff, filter_length);

    free(filter_coeff);

    return ESP_OK;
}

esp_err_t dsp_apply_bilateral_filter(const float* input, float* output, uint16_t length,
                                      float sigma_spatial, float sigma_range) {
    if (!input || !output) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t window_radius = 5;
    uint8_t window_size = 2 * window_radius + 1;

    for (uint16_t i = 0; i < length; i++) {
        float sum = 0.0f;
        float weight_sum = 0.0f;

        float center_val = input[i];

        for (int16_t j = -window_radius; j <= window_radius; j++) {
            int16_t idx = (int16_t)i + j;
            if (idx < 0 || idx >= length) continue;

            float spatial_dist = (float)j * (float)j / (sigma_spatial * sigma_spatial);
            float range_dist = (input[idx] - center_val) * (input[idx] - center_val) / (sigma_range * sigma_range);

            float weight = expf(-0.5f * spatial_dist) * expf(-0.5f * range_dist);

            sum += input[idx] * weight;
            weight_sum += weight;
        }

        output[i] = (weight_sum > 1e-10f) ? (sum / weight_sum) : input[i];
    }

    return ESP_OK;
}

esp_err_t dsp_apply_nonlocal_means(const float* input, float* output, uint16_t length,
                                    float patch_radius, float similarity_threshold,
                                    float filtering_strength) {
    if (!input || !output) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t search_radius = 10;
    uint8_t patch_size = (uint8_t)(2.0f * patch_radius + 1);

    for (uint16_t i = 0; i < length; i++) {
        float sum = 0.0f;
        float weight_sum = 0.0f;

        for (int16_t j = -search_radius; j <= search_radius; j++) {
            int16_t idx = (int16_t)i + j;
            if (idx < 0 || idx >= length) continue;

            float patch_dist = 0.0f;
            uint16_t patch_count = 0;

            for (int16_t p = -patch_radius; p <= patch_radius; p++) {
                int16_t pi = (int16_t)i + p;
                int16_t pj = idx + p;
                if (pi >= 0 && pi < length && pj >= 0 && pj < length) {
                    float diff = input[pi] - input[pj];
                    patch_dist += diff * diff;
                    patch_count++;
                }
            }
            patch_dist /= (float)patch_count;

            if (patch_dist < similarity_threshold) {
                float weight = expf(-patch_dist / filtering_strength);

                sum += input[idx] * weight;
                weight_sum += weight;
            }
        }

        if (weight_sum > 1e-10f) {
            output[i] = sum / weight_sum;
        } else {
            output[i] = input[i];
        }
    }

    return ESP_OK;
}

esp_err_t dsp_apply_morphological_operation(const float* input, float* output,
                                             uint16_t length, bool dilate) {
    if (!input || !output) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t structuring_element = 3;

    if (dilate) {
        for (uint16_t i = 0; i < length; i++) {
            float max_val = input[i];
            for (int16_t j = -structuring_element / 2; j <= structuring_element / 2; j++) {
                int16_t idx = (int16_t)i + j;
                if (idx >= 0 && idx < length) {
                    if (input[idx] > max_val) {
                        max_val = input[idx];
                    }
                }
            }
            output[i] = max_val;
        }
    } else {
        for (uint16_t i = 0; i < length; i++) {
            float min_val = input[i];
            for (int16_t j = -structuring_element / 2; j <= structuring_element / 2; j++) {
                int16_t idx = (int16_t)i + j;
                if (idx >= 0 && idx < length) {
                    if (input[idx] < min_val) {
                        min_val = input[idx];
                    }
                }
            }
            output[i] = min_val;
        }
    }

    return ESP_OK;
}

esp_err_t dsp_apply_gradient_operation(const float* input, float* output, uint16_t length) {
    if (!input || !output) {
        return ESP_ERR_INVALID_ARG;
    }

    float* dilate_out = (float*)malloc(length * sizeof(float));
    float* erode_out = (float*)malloc(length * sizeof(float));

    if (!dilate_out || !erode_out) {
        if (dilate_out) free(dilate_out);
        if (erode_out) free(erode_out);
        return ESP_ERR_NO_MEM;
    }

    dsp_apply_morphological_operation(input, dilate_out, length, true);
    dsp_apply_morphological_operation(input, erode_out, length, false);

    for (uint16_t i = 0; i < length; i++) {
        output[i] = dilate_out[i] - erode_out[i];
    }

    free(dilate_out);
    free(erode_out);

    return ESP_OK;
}

esp_err_t dsp_apply_adaptive_threshold(const float* input, float* output, uint16_t length,
                                        float block_size, float c_constant) {
    if (!input || !output) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t half_block = block_size / 2;

    for (uint16_t i = 0; i < length; i++) {
        float local_sum = 0.0f;
        uint16_t count = 0;

        for (int16_t j = -half_block; j <= half_block; j++) {
            int16_t idx = (int16_t)i + j;
            if (idx >= 0 && idx < length) {
                local_sum += input[idx];
                count++;
            }
        }

        float local_mean = (count > 0) ? (local_sum / count) : input[i];
        float threshold = local_mean - c_constant;

        output[i] = (input[i] > threshold) ? 1.0f : 0.0f;
    }

    return ESP_OK;
}

esp_err_t dsp_design_gauss_filter(float* coefficients, uint16_t* num_taps,
                                    float std_dev, float sample_rate) {
    if (!coefficients || !num_taps) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t n = *num_taps;
    float sum = 0.0f;

    for (uint16_t i = 0; i < n; i++) {
        float m = (float)i - (float)(n - 1) / 2.0f;
        float sigma2 = std_dev * std_dev;
        coefficients[i] = expf(-m * m / (2.0f * sigma2));
        sum += coefficients[i];
    }

    for (uint16_t i = 0; i < n; i++) {
        coefficients[i] /= sum;
    }

    ESP_LOGI(TAG, "Gaussian filter designed: order=%d, sigma=%.2f", n, std_dev);

    return ESP_OK;
}

esp_err_t dsp_compute_filter_response(const float* coefficients, uint16_t num_taps,
                                        float* magnitude, float* phase,
                                        uint16_t num_points, float sample_rate) {
    if (!coefficients || !magnitude) {
        return ESP_ERR_INVALID_ARG;
    }

    float freq_step = (sample_rate / 2.0f) / num_points;

    for (uint16_t p = 0; p < num_points; p++) {
        float freq = freq_step * p;
        float w = 2.0f * (float)M_PI * freq / sample_rate;

        float real = 0.0f;
        float imag = 0.0f;

        for (uint16_t n = 0; n < num_taps; n++) {
            real += coefficients[n] * cosf(w * n);
            imag -= coefficients[n] * sinf(w * n);
        }

        magnitude[p] = sqrtf(real * real + imag * imag);

        if (phase) {
            phase[p] = atan2f(imag, real);
        }
    }

    return ESP_OK;
}

esp_err_t dsp_compute_group_delay(const float* coefficients, uint16_t num_taps,
                                    float* group_delay, uint16_t num_points,
                                    float sample_rate) {
    if (!coefficients || !group_delay) {
        return ESP_ERR_INVALID_ARG;
    }

    float freq_step = (sample_rate / 2.0f) / num_points;

    for (uint16_t p = 0; p < num_points; p++) {
        float freq = freq_step * p;
        float w = 2.0f * (float)M_PI * freq / sample_rate;

        float d_real = 0.0f;
        float d_imag = 0.0f;

        for (uint16_t n = 0; n < num_taps; n++) {
            d_real -= (float)n * coefficients[n] * sinf(w * n);
            d_imag -= (float)n * coefficients[n] * cosf(w * n);
        }

        float real = 0.0f;
        float imag = 0.0f;

        for (uint16_t n = 0; n < num_taps; n++) {
            real += coefficients[n] * cosf(w * n);
            imag -= coefficients[n] * sinf(w * n);
        }

        float denom = real * real + imag * imag;
        if (fabsf(denom) > 1e-10f) {
            group_delay[p] = (real * d_imag - imag * d_real) / denom / freq_step;
        } else {
            group_delay[p] = 0.0f;
        }
    }

    return ESP_OK;
}

esp_err_t dsp_zero_phase_filter(const float* input, float* output, uint16_t length,
                                  const float* coefficients, uint16_t num_taps) {
    if (!input || !output || !coefficients) {
        return ESP_ERR_INVALID_ARG;
    }

    float* temp = (float*)malloc(length * sizeof(float));
    if (!temp) return ESP_ERR_NO_MEM;

    dsp_fir_filter_execute(input, temp, length, coefficients, num_taps);

    float* reversed = (float*)malloc(length * sizeof(float));
    if (!reversed) {
        free(temp);
        return ESP_ERR_NO_MEM;
    }

    for (uint16_t i = 0; i < length; i++) {
        reversed[i] = temp[length - 1 - i];
    }

    dsp_fir_filter_execute(reversed, output, length, coefficients, num_taps);

    for (uint16_t i = 0; i < length; i++) {
        output[length - 1 - i] = output[i];
    }

    free(temp);
    free(reversed);

    return ESP_OK;
}

void dsp_print_filter_response(const float* magnitude, const float* phase,
                                uint16_t num_points, float sample_rate) {
    if (!magnitude) return;

    ESP_LOGI(TAG, "=== Filter Response (first 10 points) ===");
    float freq_step = (sample_rate / 2.0f) / num_points;

    for (uint16_t i = 0; i < num_points && i < 10; i++) {
        float freq = freq_step * i;
        float mag_db = 20.0f * log10f(magnitude[i] + 1e-10f);
        ESP_LOGI(TAG, "  f=%.1fHz: |H|=%.4f (%.2fdB)", freq, magnitude[i], mag_db);
        if (phase) {
            ESP_LOGI(TAG, "    Phase=%.2f deg", phase[i] * 180.0f / (float)M_PI);
        }
    }
}

void dsp_print_filter_coefficients_list(const float* coeffs, uint16_t num_taps) {
    if (!coeffs) return;

    ESP_LOGI(TAG, "=== Filter Coefficients (%d taps) ===", num_taps);
    for (uint16_t i = 0; i < num_taps; i++) {
        ESP_LOGI(TAG, "  coeff[%4d] = %+.8f", i, coeffs[i]);
    }
}

esp_err_t dsp_verify_filter_stability(const float* b_coeffs, const float* a_coeffs,
                                       uint8_t num_sections) {
    if (!b_coeffs || !a_coeffs) {
        return ESP_ERR_INVALID_ARG;
    }

    for (uint8_t s = 0; s < num_sections; s++) {
        uint8_t idx = s * 3;

        float a0 = a_coeffs[idx];
        float a1 = a_coeffs[idx + 1];
        float a2 = a_coeffs[idx + 2];

        if (fabsf(a0) < 1e-10f) {
            ESP_LOGW(TAG, "Section %d: a0 near zero", s);
            return ESP_ERR_INVALID_ARG;
        }

        float a1_norm = a1 / a0;
        float a2_norm = a2 / a0;

        float r1 = fabsf(a1_norm / 2.0f + sqrtf(a1_norm * a1_norm / 4.0f - a2_norm));
        float r2 = fabsf(a1_norm / 2.0f - sqrtf(a1_norm * a1_norm / 4.0f - a2_norm));

        if (r1 > 1.0f || r2 > 1.0f) {
            ESP_LOGW(TAG, "Section %d: UNSTABLE (roots: %.4f, %.4f)", s, r1, r2);
            return ESP_ERR_INVALID_STATE;
        }
    }

    ESP_LOGI(TAG, "Filter stability verified: all %d sections stable", num_sections);
    return ESP_OK;
}
