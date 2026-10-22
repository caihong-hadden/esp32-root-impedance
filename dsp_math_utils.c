#include "dsp_pipeline.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

static const char* TAG = "DSP_MATH_UTILS";

#define MATH_UTILS_MAX_VECTOR_SIZE 4096
#define MATH_UTILS_PI 3.14159265358979323846f
#define MATH_UTILS_TWO_PI 6.28318530717958647692f
#define MATH_UTILS_EPSILON 1.1920928955078125e-7f
#define MATH_UTILS_SQRT_TWO_PI 2.50662827463100050240f
#define MATH_UTILS_LN_SQRT_TWO_PI 0.91893853320467274178f

static inline float dsp_math_fast_sin(float x) {
    x = fmodf(x, MATH_UTILS_TWO_PI);
    if (x < 0.0f) x += MATH_UTILS_TWO_PI;

    float sign = 1.0f;
    if (x > MATH_UTILS_PI) {
        x -= MATH_UTILS_PI;
        sign = -1.0f;
    }
    if (x > MATH_UTILS_PI / 2.0f) {
        x = MATH_UTILS_PI - x;
    }

    float x2 = x * x;
    float x3 = x2 * x;
    float x5 = x3 * x2;
    float x7 = x5 * x2;
    float x9 = x7 * x2;

    float sin_val = x - x3 / 6.0f + x5 / 120.0f - x7 / 5040.0f + x9 / 362880.0f;
    return sign * sin_val;
}

static inline float dsp_math_fast_cos(float x) {
    x = fmodf(x, MATH_UTILS_TWO_PI);
    if (x < 0.0f) x += MATH_UTILS_TWO_PI;

    float sign = 1.0f;
    if (x > MATH_UTILS_PI) {
        x -= MATH_UTILS_PI;
        sign = -1.0f;
    }

    float x2 = x * x;
    float x4 = x2 * x2;
    float x6 = x4 * x2;
    float x8 = x6 * x2;
    float x10 = x8 * x2;

    float cos_val = 1.0f - x2 / 2.0f + x4 / 24.0f - x6 / 720.0f + x8 / 40320.0f - x10 / 3628800.0f;
    return sign * cos_val;
}

static inline float dsp_math_fast_tan(float x) {
    float cos_val = dsp_math_fast_cos(x);
    if (fabsf(cos_val) < MATH_UTILS_EPSILON) {
        return (cos_val >= 0.0f) ? 1e10f : -1e10f;
    }
    return dsp_math_fast_sin(x) / cos_val;
}

static inline float dsp_math_fast_atan(float x) {
    float abs_x = fabsf(x);
    float sign = (x >= 0.0f) ? 1.0f : -1.0f;

    float y;
    if (abs_x > 1.0f) {
        y = MATH_UTILS_PI / 2.0f - 1.0f / abs_x;
    } else {
        y = abs_x;
    }

    float y2 = y * y;
    float y3 = y2 * y;
    float y5 = y3 * y2;
    float y7 = y5 * y2;

    float atan_val = y - y3 / 3.0f + y5 / 5.0f - y7 / 7.0f;
    return sign * ((abs_x > 1.0f) ? (MATH_UTILS_PI / 2.0f - atan_val) : atan_val);
}

static inline float dsp_math_fast_atan2(float y, float x) {
    if (x == 0.0f) {
        if (y > 0.0f) return MATH_UTILS_PI / 2.0f;
        if (y < 0.0f) return -MATH_UTILS_PI / 2.0f;
        return 0.0f;
    }

    float atan_val = dsp_math_fast_atan(y / x);

    if (x > 0.0f) {
        return atan_val;
    } else {
        if (y >= 0.0f) {
            return atan_val + MATH_UTILS_PI;
        } else {
            return atan_val - MATH_UTILS_PI;
        }
    }
}

static inline float dsp_math_fast_exp(float x) {
    if (x < -10.0f) return 0.0f;
    if (x > 10.0f) return 22026.46579f;

    x = x / MATH_UTILS_TWO_PI;
    float k = floorf(x);
    x = (x - k) * MATH_UTILS_TWO_PI;

    float x2 = x * x;
    float x3 = x2 * x;
    float x4 = x3 * x;
    float x5 = x4 * x;

    float exp_val = 1.0f + x + x2 / 2.0f + x3 / 6.0f + x4 / 24.0f + x5 / 120.0f;

    float two_pi_k = MATH_UTILS_TWO_PI * k;
    float ek = 1.0f + two_pi_k + two_pi_k * two_pi_k / 2.0f;

    return exp_val * ek;
}

static inline float dsp_math_fast_log(float x) {
    if (x <= 0.0f) return -1e10f;

    int exp;
    float mantissa = frexpf(x, &exp);

    float y = (mantissa - 1.0f) / (mantissa + 1.0f);
    float y2 = y * y;
    float y3 = y2 * y;
    float y5 = y3 * y2;
    float y7 = y5 * y2;

    float ln2 = 0.69314718f;
    float log_val = 2.0f * (y + y3 / 3.0f + y5 / 5.0f + y7 / 7.0f) + (float)(exp - 1) * ln2;

    return log_val;
}

static inline float dsp_math_fast_pow(float x, float y) {
    if (x == 0.0f) return (y > 0.0f) ? 0.0f : 1e10f;
    if (y == 0.0f) return 1.0f;
    if (y == 1.0f) return x;
    if (y == 2.0f) return x * x;
    if (y == 0.5f) return sqrtf(x);
    if (y == -0.5f) return 1.0f / sqrtf(x);

    float log_x = dsp_math_fast_log(x);
    float y_log_x = y * log_x;
    return dsp_math_fast_exp(y_log_x);
}

static inline float dsp_math_fast_sqrt(float x) {
    if (x <= 0.0f) return 0.0f;
    if (x == 1.0f) return 1.0f;

    float guess = x / 2.0f;
    guess = (guess + x / guess) / 2.0f;
    guess = (guess + x / guess) / 2.0f;
    guess = (guess + x / guess) / 2.0f;
    guess = (guess + x / guess) / 2.0f;

    return guess;
}

static inline float dsp_math_fast_sinh(float x) {
    if (x > 5.0f) {
        return dsp_math_fast_exp(x) / 2.0f;
    } else if (x < -5.0f) {
        return -dsp_math_fast_exp(-x) / 2.0f;
    }

    float x2 = x * x;
    float x3 = x2 * x;
    float x5 = x3 * x2;
    float x7 = x5 * x2;

    return x + x3 / 6.0f + x5 / 120.0f + x7 / 5040.0f;
}

static inline float dsp_math_fast_cosh(float x) {
    if (x > 5.0f) {
        return dsp_math_fast_exp(x) / 2.0f;
    } else if (x < -5.0f) {
        return dsp_math_fast_exp(-x) / 2.0f;
    }

    float x2 = x * x;
    float x4 = x2 * x2;
    float x6 = x4 * x2;

    return 1.0f + x2 / 2.0f + x4 / 24.0f + x6 / 720.0f;
}

static inline float dsp_math_fast_tanh(float x) {
    if (x > 5.0f) return 1.0f;
    if (x < -5.0f) return -1.0f;

    float exp_2x = dsp_math_fast_exp(2.0f * x);
    return (exp_2x - 1.0f) / (exp_2x + 1.0f);
}

static inline float dsp_math_fast_asin(float x) {
    if (x < -1.0f) x = -1.0f;
    if (x > 1.0f) x = 1.0f;

    float sign = (x >= 0.0f) ? 1.0f : -1.0f;
    float abs_x = fabsf(x);

    float asin_val;
    if (abs_x < 0.7f) {
        float x2 = abs_x * abs_x;
        float x4 = x2 * x2;
        float x6 = x4 * x2;
        asin_val = abs_x + abs_x * x2 / 6.0f + 3.0f * abs_x * x4 / 40.0f + 5.0f * abs_x * x6 / 112.0f;
    } else {
        float sqrt_val = dsp_math_fast_sqrt(1.0f - abs_x);
        asin_val = MATH_UTILS_PI / 2.0f - dsp_math_fast_sqrt(2.0f * (1.0f - abs_x))
                   * (1.0f + (1.0f / 12.0f) * (1.0f - abs_x)
                   + (3.0f / 160.0f) * (1.0f - abs_x) * (1.0f - abs_x));
    }

    return sign * asin_val;
}

static inline float dsp_math_fast_acos(float x) {
    if (x < -1.0f) x = -1.0f;
    if (x > 1.0f) x = 1.0f;

    return MATH_UTILS_PI / 2.0f - dsp_math_fast_asin(x);
}

float dsp_compute_rms(const float* data, uint16_t length) {
    if (!data || length == 0) return 0.0f;

    float sum_sq = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        sum_sq += data[i] * data[i];
    }
    return dsp_math_fast_sqrt(sum_sq / length);
}

float dsp_compute_snr(const float* signal, const float* noise, uint16_t length) {
    if (!signal || !noise || length == 0) return 0.0f;

    float signal_power = 0.0f;
    float noise_power = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        signal_power += signal[i] * signal[i];
        noise_power += noise[i] * noise[i];
    }

    if (noise_power < MATH_UTILS_EPSILON) return 100.0f;
    return 10.0f * dsp_math_fast_log(signal_power / noise_power) / dsp_math_fast_log(10.0f);
}

float dsp_find_threshold_percentile(const float* data, uint16_t length, float percentile) {
    if (!data || length == 0) return 0.0f;

    float* sorted = (float*)malloc(length * sizeof(float));
    if (!sorted) return 0.0f;

    memcpy(sorted, data, length * sizeof(float));

    for (uint16_t i = 0; i < length; i++) {
        for (uint16_t j = i + 1; j < length; j++) {
            if (sorted[i] > sorted[j]) {
                float tmp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = tmp;
            }
        }
    }

    uint16_t idx = (uint16_t)(length * percentile / 100.0f);
    if (idx >= length) idx = length - 1;

    float result = sorted[idx];
    free(sorted);
    return result;
}

float dsp_interpolate_peak_location(const float* y, uint16_t idx, uint8_t order) {
    if (!y || idx == 0 || idx == 65535 - 1) return (float)idx;

    if (order == 2 && idx > 0 && idx < 65535 - 1) {
        float alpha = y[idx + 1] - y[idx - 1];
        float beta = 2.0f * y[idx] - y[idx - 1] - y[idx + 1];
        if (fabsf(beta) > MATH_UTILS_EPSILON) {
            return (float)idx + alpha / beta;
        }
    }
    return (float)idx;
}

void dsp_apply_window_function(float* data, uint16_t length, window_type_t window_type) {
    if (!data || length == 0) return;

    for (uint16_t i = 0; i < length; i++) {
        float n_val = (float)i / (float)(length - 1);
        float window = 1.0f;

        switch (window_type) {
            case WINDOW_TYPE_HANNING:
                window = 0.5f - 0.5f * dsp_math_fast_cos(MATH_UTILS_TWO_PI * n_val);
                break;
            case WINDOW_TYPE_HAMMING:
                window = 0.54f - 0.46f * dsp_math_fast_cos(MATH_UTILS_TWO_PI * n_val);
                break;
            case WINDOW_TYPE_BLACKMAN:
                window = 0.42f - 0.5f * dsp_math_fast_cos(MATH_UTILS_TWO_PI * n_val)
                         + 0.08f * dsp_math_fast_cos(MATH_UTILS_TWO_PI * 2.0f * n_val);
                break;
            case WINDOW_TYPE_BARTLETT:
                window = 1.0f - fabsf(2.0f * n_val - 1.0f);
                break;
            case WINDOW_TYPE_WELCH:
                window = 1.0f - (2.0f * n_val - 1.0f) * (2.0f * n_val - 1.0f);
                break;
            case WINDOW_TYPE_KAISER: {
                float beta = 8.0f;
                float alpha = beta * dsp_math_fast_sqrt(1.0f - dsp_math_fast_pow(2.0f * n_val - 1.0f, 2.0f));
                if (fabsf(alpha) < MATH_UTILS_EPSILON) {
                    window = 1.0f;
                } else {
                    window = dsp_math_fast_sin(MATH_UTILS_PI * alpha) / (MATH_UTILS_PI * alpha);
                }
                break;
            }
            case WINDOW_TYPE_GAUSSIAN: {
                float alpha = 2.5f;
                float arg = alpha * (n_val - 0.5f);
                window = dsp_math_fast_exp(-2.0f * arg * arg);
                break;
            }
            case WINDOW_TYPE_CHEBYSHEV: {
                float ripple = 40.0f;
                window = 1.0f - 0.05f * dsp_math_fast_cos(MATH_UTILS_TWO_PI * n_val);
                break;
            }
            default:
                window = 1.0f;
                break;
        }
        data[i] *= window;
    }
}

void dsp_normalize_vector(float* data, uint16_t length, float target_rms) {
    if (!data || length == 0) return;

    float current_rms = dsp_compute_rms(data, length);
    if (current_rms < MATH_UTILS_EPSILON) return;

    float scale = target_rms / current_rms;
    for (uint16_t i = 0; i < length; i++) {
        data[i] *= scale;
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
    for (uint16_t i = 0; i < length; i++) {
        sum += data[i];
    }
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

float dsp_vector_std_dev(const float* data, uint16_t length) {
    if (!data || length == 0) return 0.0f;

    float mean = dsp_vector_mean(data, length);
    float variance = dsp_vector_variance(data, length, mean);
    return dsp_math_fast_sqrt(variance);
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

    float denom = dsp_math_fast_sqrt(var_a * var_b);
    if (denom < MATH_UTILS_EPSILON) return 0.0f;

    return cov / denom;
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

void dsp_autocorrelation(const float* data, float* result, uint16_t length, uint16_t max_lag) {
    if (!data || !result) return;

    for (uint16_t lag = 0; lag < max_lag; lag++) {
        float sum = 0.0f;
        uint16_t count = 0;
        for (uint16_t i = 0; i + lag < length; i++) {
            sum += data[i] * data[i + lag];
            count++;
        }
        result[lag] = (count > 0) ? (sum / count) : 0.0f;
    }
}

float dsp_compute_spectral_entropy(const float* spectrum, uint16_t length) {
    if (!spectrum || length == 0) return 0.0f;

    float sum = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        sum += spectrum[i] + MATH_UTILS_EPSILON;
    }

    float entropy = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        float p = (spectrum[i] + MATH_UTILS_EPSILON) / sum;
        if (p > MATH_UTILS_EPSILON) {
            entropy -= p * dsp_math_fast_log(p) / dsp_math_fast_log(2.0f);
        }
    }
    return entropy;
}

float dsp_compute_spectral_flatness(const float* spectrum, uint16_t length) {
    if (!spectrum || length == 0) return 0.0f;

    float geo_mean = 1.0f;
    float arith_mean = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        if (spectrum[i] > MATH_UTILS_EPSILON) {
            geo_mean *= spectrum[i];
            arith_mean += spectrum[i];
        }
    }

    geo_mean = dsp_math_fast_pow(geo_mean, 1.0f / length);
    arith_mean /= length;

    if (arith_mean < MATH_UTILS_EPSILON) return 0.0f;
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

    if (denominator < MATH_UTILS_EPSILON) return 0.0f;
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
                float diff = (float)i - indices[j];
                if (diff < 0) diff = -diff;
                if (diff < min_spacing) {
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

void dsp_quantize_signal(const float* input, int16_t* output, uint16_t length, float lsb_value) {
    if (!input || !output || lsb_value <= 0.0f) return;

    float inv_lsb = 1.0f / lsb_value;

    for (uint16_t i = 0; i < length; i++) {
        float quantized = floorf(input[i] * inv_lsb + 0.5f);

        if (quantized > 32767.0f) quantized = 32767.0f;
        if (quantized < -32768.0f) quantized = -32768.0f;

        output[i] = (int16_t)quantized;
    }
}

void dsp_dequantize_signal(const int16_t* input, float* output, uint16_t length, float lsb_value) {
    if (!input || !output) return;

    for (uint16_t i = 0; i < length; i++) {
        output[i] = (float)input[i] * lsb_value;
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
    if (!input || !output || length == 0) return;

    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;

    output[0] = input[0];
    float one_minus_alpha = 1.0f - alpha;

    for (uint16_t i = 1; i < length; i++) {
        output[i] = alpha * input[i] + one_minus_alpha * output[i-1];
    }
}

void dsp_smooth_signal(const float* input, float* output, uint16_t length, uint8_t kernel_size) {
    if (!input || !output) return;

    if (kernel_size < 3) kernel_size = 3;
    if (kernel_size % 2 == 0) kernel_size++;

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

    if (window_size < 3) window_size = 3;
    if (window_size % 2 == 0) window_size++;

    uint8_t half = window_size / 2;
    float* window = (float*)malloc(window_size * sizeof(float));
    if (!window) {
        memcpy(output, input, length * sizeof(float));
        return;
    }

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
                    float tmp = window[m];
                    window[m] = window[n];
                    window[n] = tmp;
                }
            }
        }

        output[i] = window[cnt / 2];
    }

    free(window);
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

void dsp_compute_gradient(const float* input, float* grad_x, float* grad_y, uint16_t width, uint16_t height) {
    if (!input || !grad_x || !grad_y) return;

    for (uint16_t y = 0; y < height; y++) {
        for (uint16_t x = 0; x < width; x++) {
            uint16_t idx = y * width + x;

            uint16_t x_next = (x + 1 < width) ? x + 1 : x;
            uint16_t y_next = (y + 1 < height) ? y + 1 : y;

            uint16_t idx_x = y * width + x_next;
            uint16_t idx_y = y_next * width + x;

            grad_x[idx] = input[idx_x] - input[idx];
            grad_y[idx] = input[idx_y] - input[idx];
        }
    }
}

esp_err_t dsp_matrix_multiply(const float* a, const float* b, float* c,
                              uint16_t rows_a, uint16_t cols_a, uint16_t cols_b) {
    if (!a || !b || !c) return ESP_ERR_INVALID_ARG;

    for (uint16_t i = 0; i < rows_a; i++) {
        for (uint16_t j = 0; j < cols_b; j++) {
            float sum = 0.0f;
            for (uint16_t k = 0; k < cols_a; k++) {
                sum += a[i * cols_a + k] * b[k * cols_b + j];
            }
            c[i * cols_b + j] = sum;
        }
    }

    return ESP_OK;
}

esp_err_t dsp_matrix_transpose(const float* input, float* output, uint16_t rows, uint16_t cols) {
    if (!input || !output) return ESP_ERR_INVALID_ARG;

    for (uint16_t i = 0; i < rows; i++) {
        for (uint16_t j = 0; j < cols; j++) {
            output[j * rows + i] = input[i * cols + j];
        }
    }

    return ESP_OK;
}

esp_err_t dsp_matrix_inverse(const float* input, float* output, uint16_t size) {
    if (!input || !output || size == 0) return ESP_ERR_INVALID_ARG;

    if (size == 2) {
        float det = input[0] * input[3] - input[1] * input[2];
        if (fabsf(det) < MATH_UTILS_EPSILON) return ESP_FAIL;

        float inv_det = 1.0f / det;
        output[0] = input[3] * inv_det;
        output[1] = -input[1] * inv_det;
        output[2] = -input[2] * inv_det;
        output[3] = input[0] * inv_det;
        return ESP_OK;
    }

    uint16_t n = size;
    float* aug = (float*)malloc(n * n * 2 * sizeof(float));
    if (!aug) return ESP_ERR_NO_MEM;

    for (uint16_t i = 0; i < n; i++) {
        for (uint16_t j = 0; j < n; j++) {
            aug[i * 2 * n + j] = input[i * n + j];
            aug[i * 2 * n + n + j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for (uint16_t i = 0; i < n; i++) {
        float pivot = aug[i * 2 * n + i];
        if (fabsf(pivot) < MATH_UTILS_EPSILON) {
            free(aug);
            return ESP_FAIL;
        }

        for (uint16_t j = 0; j < 2 * n; j++) {
            aug[i * 2 * n + j] /= pivot;
        }

        for (uint16_t k = 0; k < n; k++) {
            if (k != i) {
                float factor = aug[k * 2 * n + i];
                for (uint16_t j = 0; j < 2 * n; j++) {
                    aug[k * 2 * n + j] -= factor * aug[i * 2 * n + j];
                }
            }
        }
    }

    for (uint16_t i = 0; i < n; i++) {
        for (uint16_t j = 0; j < n; j++) {
            output[i * n + j] = aug[i * 2 * n + n + j];
        }
    }

    free(aug);
    return ESP_OK;
}

esp_err_t dsp_matrix_eigenvalues(const float* matrix, float* eigenvalues, uint16_t size) {
    if (!matrix || !eigenvalues || size == 0) return ESP_ERR_INVALID_ARG;

    if (size == 2) {
        float a = 1.0f;
        float b = -(matrix[0] + matrix[3]);
        float c = matrix[0] * matrix[3] - matrix[1] * matrix[2];

        float discriminant = b * b - 4.0f * a * c;
        if (discriminant < 0.0f) discriminant = 0.0f;

        float sqrt_disc = dsp_math_fast_sqrt(discriminant);
        eigenvalues[0] = (-b + sqrt_disc) / (2.0f * a);
        eigenvalues[1] = (-b - sqrt_disc) / (2.0f * a);
        return ESP_OK;
    }

    for (uint16_t i = 0; i < size; i++) {
        eigenvalues[i] = matrix[i * size + i];
    }

    return ESP_OK;
}

esp_err_t dsp_svd(const float* input, float* u, float* sigma, float* vt,
                   uint16_t rows, uint16_t cols) {
    if (!input) return ESP_ERR_INVALID_ARG;

    uint16_t min_dim = (rows < cols) ? rows : cols;

    if (sigma) {
        for (uint16_t i = 0; i < min_dim; i++) {
            sigma[i] = 1.0f;
        }
    }

    if (u) {
        for (uint16_t i = 0; i < rows * rows; i++) {
            u[i] = 0.0f;
        }
        for (uint16_t i = 0; i < rows && i < rows; i++) {
            u[i * rows + i] = 1.0f;
        }
    }

    if (vt) {
        for (uint16_t i = 0; i < cols * cols; i++) {
            vt[i] = 0.0f;
        }
        for (uint16_t i = 0; i < cols && i < cols; i++) {
            vt[i * cols + i] = 1.0f;
        }
    }

    return ESP_OK;
}

float dsp_compute_determinant(const float* matrix, uint16_t size) {
    if (!matrix || size == 0) return 0.0f;

    if (size == 1) return matrix[0];
    if (size == 2) return matrix[0] * matrix[3] - matrix[1] * matrix[2];
    if (size == 3) {
        return matrix[0] * (matrix[4] * matrix[8] - matrix[5] * matrix[7])
             - matrix[1] * (matrix[3] * matrix[8] - matrix[5] * matrix[6])
             + matrix[2] * (matrix[3] * matrix[7] - matrix[4] * matrix[6]);
    }

    float det = 0.0f;
    for (uint16_t i = 0; i < size; i++) {
        float sign = (i % 2 == 0) ? 1.0f : -1.0f;
        float minor = dsp_compute_determinant_submatrix(matrix, size, 0, i);
        det += sign * matrix[i] * minor;
    }

    return det;
}

static float dsp_compute_determinant_submatrix(const float* matrix, uint16_t size, uint16_t row, uint16_t col) {
    if (size == 1) return matrix[0];

    uint16_t new_size = size - 1;
    float* sub = (float*)malloc(new_size * new_size * sizeof(float));
    if (!sub) return 0.0f;

    uint16_t idx = 0;
    for (uint16_t i = 0; i < size; i++) {
        if (i == row) continue;
        for (uint16_t j = 0; j < size; j++) {
            if (j == col) continue;
            sub[idx++] = matrix[i * size + j];
        }
    }

    float det = dsp_compute_determinant(sub, new_size);
    free(sub);

    return ((row + col) % 2 == 0) ? det : -det;
}

esp_err_t dsp_solve_linear_system(const float* a, const float* b, float* x, uint16_t size) {
    if (!a || !b || !x || size == 0) return ESP_ERR_INVALID_ARG;

    float det = dsp_compute_determinant(a, size);
    if (fabsf(det) < MATH_UTILS_EPSILON) return ESP_FAIL;

    float inv_det = 1.0f / det;
    float* adj = (float*)malloc(size * size * sizeof(float));
    if (!adj) return ESP_ERR_NO_MEM;

    dsp_compute_adjugate(a, adj, size);

    for (uint16_t i = 0; i < size; i++) {
        x[i] = 0.0f;
        for (uint16_t j = 0; j < size; j++) {
            x[i] += adj[j * size + i] * b[j];
        }
        x[i] *= inv_det;
    }

    free(adj);
    return ESP_OK;
}

esp_err_t dsp_compute_adjugate(const float* input, float* output, uint16_t size) {
    if (!input || !output) return ESP_ERR_INVALID_ARG;

    for (uint16_t i = 0; i < size; i++) {
        for (uint16_t j = 0; j < size; j++) {
            uint16_t new_size = size - 1;
            float* sub = (float*)malloc(new_size * new_size * sizeof(float));
            if (!sub) return ESP_ERR_NO_MEM;

            uint16_t idx = 0;
            for (uint16_t ii = 0; ii < size; ii++) {
                for (uint16_t jj = 0; jj < size; jj++) {
                    if (ii == i || jj == j) continue;
                    sub[idx++] = input[ii * size + jj];
                }
            }

            float cofactor = dsp_compute_determinant(sub, new_size);
            if (((i + j) % 2) == 1) cofactor = -cofactor;

            output[j * size + i] = cofactor;

            free(sub);
        }
    }

    return ESP_OK;
}

esp_err_t dsp_cholesky_decomposition(const float* input, float* output, uint16_t size) {
    if (!input || !output) return ESP_ERR_INVALID_ARG;

    for (uint16_t i = 0; i < size; i++) {
        for (uint16_t j = 0; j <= i; j++) {
            float sum = 0.0f;

            if (j == i) {
                for (uint16_t k = 0; k < j; k++) {
                    sum += output[j * size + k] * output[j * size + k];
                }
                float diag = input[j * size + j] - sum;
                if (diag <= 0.0f) return ESP_FAIL;
                output[j * size + j] = dsp_math_fast_sqrt(diag);
            } else {
                for (uint16_t k = 0; k < j; k++) {
                    sum += output[i * size + k] * output[j * size + k];
                }
                if (fabsf(output[j * size + j]) < MATH_UTILS_EPSILON) return ESP_FAIL;
                output[i * size + j] = (input[i * size + j] - sum) / output[j * size + j];
            }
        }
    }

    return ESP_OK;
}

esp_err_t dsp_qr_decomposition(const float* input, float* q, float* r, uint16_t rows, uint16_t cols) {
    if (!input || !q || !r) return ESP_ERR_INVALID_ARG;

    uint16_t n = cols;

    for (uint16_t i = 0; i < rows * cols; i++) {
        r[i] = input[i];
    }
    for (uint16_t i = 0; i < rows * rows; i++) {
        q[i] = 0.0f;
    }

    for (uint16_t i = 0; i < rows && i < cols; i++) {
        q[i * rows + i] = 1.0f;
    }

    for (uint16_t k = 0; k < n - 1; k++) {
        float norm_x = 0.0f;
        for (uint16_t i = k; i < rows; i++) {
            norm_x += r[i * cols + k] * r[i * cols + k];
        }
        norm_x = dsp_math_fast_sqrt(norm_x);

        if (norm_x < MATH_UTILS_EPSILON) continue;

        float sign = (r[k * cols + k] >= 0.0f) ? 1.0f : -1.0f;
        r[k * cols + k] += sign * norm_x;

        for (uint16_t j = k; j < cols; j++) {
            float dot = 0.0f;
            for (uint16_t i = k; i < rows; i++) {
                dot += r[i * cols + k] * r[i * cols + j];
            }
            float norm_sq = 0.0f;
            for (uint16_t i = k; i < rows; i++) {
                norm_sq += r[i * cols + k] * r[i * cols + k];
            }
            if (fabsf(norm_sq) < MATH_UTILS_EPSILON) continue;

            float factor = dot / norm_sq;
            for (uint16_t i = k; i < rows; i++) {
                r[i * cols + j] -= factor * r[i * cols + k];
            }
        }

        r[k * cols + k] = -sign * norm_x;
        for (uint16_t i = k + 1; i < rows; i++) {
            r[i * cols + k] = 0.0f;
        }
    }

    return ESP_OK;
}

esp_err_t dsp_convolution(const float* signal, uint16_t signal_len,
                            const float* kernel, uint16_t kernel_len,
                            float* output) {
    if (!signal || !kernel || !output) return ESP_ERR_INVALID_ARG;

    uint16_t output_len = signal_len + kernel_len - 1;

    for (uint16_t i = 0; i < output_len; i++) {
        float sum = 0.0f;
        uint16_t start = (i >= kernel_len - 1) ? (i - (kernel_len - 1)) : 0;
        uint16_t end = (i < signal_len) ? (i + 1) : signal_len;

        for (uint16_t j = start; j < end; j++) {
            uint16_t k_idx = i - j;
            if (k_idx < kernel_len) {
                sum += signal[j] * kernel[k_idx];
            }
        }
        output[i] = sum;
    }

    return ESP_OK;
}

esp_err_t dsp_deconvolution(const float* signal, uint16_t signal_len,
                              const float* kernel, uint16_t kernel_len,
                              float* output) {
    if (!signal || !kernel || !output) return ESP_ERR_INVALID_ARG;

    for (uint16_t i = 0; i < signal_len; i++) {
        output[i] = signal[i];

        for (uint16_t j = 1; j < kernel_len && j <= i; j++) {
            output[i] -= kernel[j] * output[i - j];
        }

        if (fabsf(kernel[0]) < MATH_UTILS_EPSILON) {
            output[i] = 0.0f;
        } else {
            output[i] /= kernel[0];
        }
    }

    return ESP_OK;
}

esp_err_t dsp_circular_convolution(const float* a, const float* b, float* output,
                                     uint16_t length) {
    if (!a || !b || !output) return ESP_ERR_INVALID_ARG;

    float* temp = (float*)malloc(length * sizeof(float));
    if (!temp) return ESP_ERR_NO_MEM;

    for (uint16_t i = 0; i < length; i++) {
        float sum = 0.0f;
        for (uint16_t j = 0; j < length; j++) {
            uint16_t idx = (i + j) % length;
            sum += a[idx] * b[j];
        }
        temp[i] = sum;
    }

    memcpy(output, temp, length * sizeof(float));
    free(temp);

    return ESP_OK;
}

esp_err_t dsp_polyval(const float* coefficients, uint16_t order, float x, float* result) {
    if (!coefficients || !result) return ESP_ERR_INVALID_ARG;

    *result = coefficients[order];
    for (int16_t i = order - 1; i >= 0; i--) {
        *result = *result * x + coefficients[i];
    }

    return ESP_OK;
}

esp_err_t dsp_polyfit(const float* x, const float* y, uint16_t n, float* coefficients, uint16_t order) {
    if (!x || !y || !coefficients || n == 0 || order == 0) return ESP_ERR_INVALID_ARG;

    uint16_t m = order + 1;
    float* xtx = (float*)calloc(m * m, sizeof(float));
    float* xty = (float*)calloc(m, sizeof(float));

    if (!xtx || !xty) {
        if (xtx) free(xtx);
        if (xty) free(xty);
        return ESP_ERR_NO_MEM;
    }

    for (uint16_t i = 0; i < m; i++) {
        for (uint16_t j = 0; j < m; j++) {
            float sum = 0.0f;
            for (uint16_t k = 0; k < n; k++) {
                sum += dsp_math_fast_pow(x[k], i + j);
            }
            xtx[i * m + j] = sum;
        }
    }

    for (uint16_t i = 0; i < m; i++) {
        float sum = 0.0f;
        for (uint16_t k = 0; k < n; k++) {
            sum += y[k] * dsp_math_fast_pow(x[k], i);
        }
        xty[i] = sum;
    }

    dsp_solve_linear_system(xtx, xty, coefficients, m);

    free(xtx);
    free(xty);

    return ESP_OK;
}

esp_err_t dsp_interpolate_linear(const float* x_data, const float* y_data, uint16_t n,
                                  const float* x_query, float* y_result, uint16_t m) {
    if (!x_data || !y_data || !x_query || !y_result) return ESP_ERR_INVALID_ARG;

    for (uint16_t i = 0; i < m; i++) {
        float x = x_query[i];

        if (x <= x_data[0]) {
            y_result[i] = y_data[0];
            continue;
        }
        if (x >= x_data[n-1]) {
            y_result[i] = y_data[n-1];
            continue;
        }

        for (uint16_t j = 0; j < n - 1; j++) {
            if (x >= x_data[j] && x <= x_data[j+1]) {
                float t = (x - x_data[j]) / (x_data[j+1] - x_data[j]);
                y_result[i] = y_data[j] + t * (y_data[j+1] - y_data[j]);
                break;
            }
        }
    }

    return ESP_OK;
}

esp_err_t dsp_interpolate_cubic(const float* x_data, const float* y_data, uint16_t n,
                                 const float* x_query, float* y_result, uint16_t m) {
    if (!x_data || !y_data || !x_query || !y_result || n < 4) return ESP_ERR_INVALID_ARG;

    float* m_coeffs = (float*)malloc(n * sizeof(float));
    if (!m_coeffs) return ESP_ERR_NO_MEM;

    for (uint16_t i = 1; i < n - 1; i++) {
        m_coeffs[i] = 3.0f * ((y_data[i+1] - y_data[i]) / (x_data[i+1] - x_data[i])
                           - (y_data[i] - y_data[i-1]) / (x_data[i] - x_data[i-1]));
    }
    m_coeffs[0] = m_coeffs[1];
    m_coeffs[n-1] = m_coeffs[n-2];

    for (uint16_t i = 0; i < m; i++) {
        float x = x_query[i];

        if (x <= x_data[0]) {
            y_result[i] = y_data[0];
            continue;
        }
        if (x >= x_data[n-1]) {
            y_result[i] = y_data[n-1];
            continue;
        }

        uint16_t k = 0;
        for (uint16_t j = 0; j < n - 1; j++) {
            if (x >= x_data[j] && x <= x_data[j+1]) {
                k = j;
                break;
            }
        }

        float h = x_data[k+1] - x_data[k];
        float t = (x - x_data[k]) / h;

        float a = y_data[k];
        float b = (y_data[k+1] - y_data[k]) / h - h * (2.0f * m_coeffs[k] + m_coeffs[k+1]) / 6.0f;
        float c = m_coeffs[k] / 2.0f;
        float d = (m_coeffs[k+1] - m_coeffs[k]) / (6.0f * h);

        y_result[i] = a + b * t + c * t * t + d * t * t * t;
    }

    free(m_coeffs);
    return ESP_OK;
}

esp_err_t dsp_interpolate_spline(const float* x_data, const float* y_data, uint16_t n,
                                  const float* x_query, float* y_result, uint16_t m) {
    return dsp_interpolate_cubic(x_data, y_data, n, x_query, y_result, m);
}

esp_err_t dsp_lagrange_interpolation(const float* x_data, const float* y_data, uint16_t n,
                                      float x, float* result) {
    if (!x_data || !y_data || !result) return ESP_ERR_INVALID_ARG;

    *result = 0.0f;

    for (uint16_t i = 0; i < n; i++) {
        float basis = y_data[i];
        for (uint16_t j = 0; j < n; j++) {
            if (i != j) {
                if (fabsf(x_data[i] - x_data[j]) < MATH_UTILS_EPSILON) {
                    basis = 0.0f;
                    break;
                }
                basis *= (x - x_data[j]) / (x_data[i] - x_data[j]);
            }
        }
        *result += basis;
    }

    return ESP_OK;
}

esp_err_t dsp_binomial_coefficient(int32_t n, int32_t k, float* result) {
    if (!result) return ESP_ERR_INVALID_ARG;

    if (k < 0 || k > n) {
        *result = 0.0f;
        return ESP_OK;
    }

    if (k > n - k) k = n - k;

    float result_val = 1.0f;
    for (int32_t i = 0; i < k; i++) {
        result_val *= (float)(n - i) / (float)(i + 1);
    }

    *result = result_val;
    return ESP_OK;
}

esp_err_t dsp_legendre_polynomial(float x, uint16_t n, float* result) {
    if (!result) return ESP_ERR_INVALID_ARG;

    if (n == 0) {
        *result = 1.0f;
        return ESP_OK;
    }

    if (n == 1) {
        *result = x;
        return ESP_OK;
    }

    float p0 = 1.0f;
    float p1 = x;

    *result = p1;

    for (uint16_t i = 2; i <= n; i++) {
        float p2 = ((2.0f * i - 1.0f) * x * p1 - (i - 1.0f) * p0) / (float)i;
        p0 = p1;
        p1 = p2;
        *result = p2;
    }

    return ESP_OK;
}

esp_err_t dsp_hermite_polynomial(float x, uint16_t n, float* result) {
    if (!result) return ESP_ERR_INVALID_ARG;

    if (n == 0) {
        *result = 1.0f;
        return ESP_OK;
    }

    if (n == 1) {
        *result = 2.0f * x;
        return ESP_OK;
    }

    float h0 = 1.0f;
    float h1 = 2.0f * x;

    *result = h1;

    for (uint16_t i = 2; i <= n; i++) {
        float h2 = 2.0f * x * h1 - 2.0f * (i - 1) * h0;
        h0 = h1;
        h1 = h2;
        *result = h2;
    }

    return ESP_OK;
}

float dsp_gamma_function(float x) {
    if (x <= 0.0f) return 0.0f;

    if (x < 1.0f) {
        return dsp_gamma_function(x + 1.0f) / x;
    }

    if (x > 2.0f) {
        return (x - 1.0f) * dsp_gamma_function(x - 1.0f);
    }

    float x_minus_1 = x - 1.0f;
    float series = 1.0f;
    float term = 1.0f;

    for (uint16_t n = 1; n <= 10; n++) {
        term *= x_minus_1 / (float)n;
        series += term;
    }

    float g = 7.0f;
    float c[8] = {0.99999999999980993f, 676.5203681218851f, -1259.1392167224028f,
                  771.32342877765313f, -176.61502916214059f, 12.507343278686905f,
                  -0.13857109526572012f, 9.9843695780195716e-6f, 1.5056327351493116e-7f};

    float p = c[0];
    for (uint16_t i = 1; i < 7; i++) {
        p += c[i] / (x_minus_1 + i);
    }

    float t = x_minus_1 + g + 0.5f;
    float y = dsp_math_fast_sqrt(MATH_UTILS_TWO_PI) * dsp_math_fast_pow(t, x_minus_1 + 0.5f) * dsp_math_fast_exp(-t) * p;

    return y;
}

float dsp_beta_function(float a, float b) {
    if (a <= 0.0f || b <= 0.0f) return 0.0f;

    return dsp_gamma_function(a) * dsp_gamma_function(b) / dsp_gamma_function(a + b);
}

float dsp_error_function(float x) {
    if (fabsf(x) > 3.0f) {
        float sign = (x >= 0.0f) ? 1.0f : -1.0f;
        float t = 1.0f / (1.0f + 0.3275911f * fabsf(x));
        float y = 1.0f - t * dsp_math_fast_exp(-x * x) *
                  (t * (t * (t * 0.254829592f - 0.284496736f) + 1.421413741f) -
                   t * (t * 1.453152027f - 1.061405429f) + 0.6820018f);
        return sign * y;
    }

    float x2 = x * x;
    float sum = x;
    float term = x;
    float x_power = x;

    for (uint16_t n = 1; n <= 50; n++) {
        float n_float = (float)n;
        term *= -x2 / n_float;
        x_power *= x;
        float increment = term * x_power / (2.0f * n_float + 1.0f);
        sum += increment;
        if (fabsf(increment) < 1e-10f) break;
    }

    return (2.0f / dsp_math_fast_sqrt(MATH_UTILS_PI)) * sum;
}

float dsp_complementary_error_function(float x) {
    return 1.0f - dsp_error_function(x);
}

float dsp_normal_distribution_pdf(float x, float mean, float std_dev) {
    if (std_dev <= 0.0f) return 0.0f;

    float diff = x - mean;
    float exponent = -diff * diff / (2.0f * std_dev * std_dev);
    return dsp_math_fast_exp(exponent) / (std_dev * MATH_UTILS_SQRT_TWO_PI);
}

float dsp_normal_distribution_cdf(float x, float mean, float std_dev) {
    if (std_dev <= 0.0f) return 0.0f;

    float z = (x - mean) / (std_dev * dsp_math_fast_sqrt(2.0f));
    return 0.5f * (1.0f + dsp_error_function(z));
}

float dsp_uniform_distribution_pdf(float x, float a, float b) {
    if (x < a || x > b) return 0.0f;
    return 1.0f / (b - a);
}

float dsp_uniform_distribution_cdf(float x, float a, float b) {
    if (x < a) return 0.0f;
    if (x > b) return 1.0f;
    return (x - a) / (b - a);
}

float dsp_exponential_distribution_pdf(float x, float lambda) {
    if (x < 0.0f || lambda <= 0.0f) return 0.0f;
    return lambda * dsp_math_fast_exp(-lambda * x);
}

float dsp_exponential_distribution_cdf(float x, float lambda) {
    if (x < 0.0f || lambda <= 0.0f) return 0.0f;
    return 1.0f - dsp_math_fast_exp(-lambda * x);
}

float dsp_poisson_distribution_pmf(uint32_t k, float lambda) {
    if (lambda <= 0.0f) return 0.0f;

    float log_prob = -lambda + k * dsp_math_fast_log(lambda);

    for (uint32_t i = 2; i <= k; i++) {
        log_prob -= dsp_math_fast_log((float)i);
    }

    return dsp_math_fast_exp(log_prob);
}

float dsp_binomial_distribution_pmf(uint32_t k, uint32_t n, float p) {
    if (p < 0.0f || p > 1.0f) return 0.0f;

    if (k > n) return 0.0f;

    float log_comb = 0.0f;
    for (uint32_t i = 2; i <= n; i++) {
        log_comb += dsp_math_fast_log((float)i);
    }
    for (uint32_t i = 2; i <= k; i++) {
        log_comb -= dsp_math_fast_log((float)i);
    }
    for (uint32_t i = 2; i <= n - k; i++) {
        log_comb -= dsp_math_fast_log((float)i);
    }

    float log_prob = log_comb + k * dsp_math_fast_log(p) + (n - k) * dsp_math_fast_log(1.0f - p);

    return dsp_math_fast_exp(log_prob);
}

esp_err_t dsp_random_uniform(float* data, uint16_t length, float min_val, float max_val) {
    if (!data) return ESP_ERR_INVALID_ARG;

    if (max_val <= min_val) {
        for (uint16_t i = 0; i < length; i++) {
            data[i] = min_val;
        }
        return ESP_OK;
    }

    float range = max_val - min_val;

    for (uint16_t i = 0; i < length; i++) {
        uint32_t rand_val = esp_random();
        data[i] = min_val + (float)rand_val / (float)UINT32_MAX * range;
    }

    return ESP_OK;
}

esp_err_t dsp_random_normal(float* data, uint16_t length, float mean, float std_dev) {
    if (!data || std_dev <= 0.0f) return ESP_ERR_INVALID_ARG;

    for (uint16_t i = 0; i < length; i += 2) {
        uint32_t u1_rand = esp_random();
        uint32_t u2_rand = esp_random();

        float u1 = (float)u1_rand / (float)UINT32_MAX;
        float u2 = (float)u2_rand / (float)UINT32_MAX;

        float u1_transformed = dsp_math_fast_sqrt(-2.0f * dsp_math_fast_log(u1 + MATH_UTILS_EPSILON));
        float z0 = u1_transformed * dsp_math_fast_cos(MATH_UTILS_TWO_PI * u2);
        float z1 = u1_transformed * dsp_math_fast_sin(MATH_UTILS_TWO_PI * u2);

        data[i] = mean + std_dev * z0;
        if (i + 1 < length) {
            data[i + 1] = mean + std_dev * z1;
        }
    }

    return ESP_OK;
}

esp_err_t dsp_random_exponential(float* data, uint16_t length, float lambda) {
    if (!data || lambda <= 0.0f) return ESP_ERR_INVALID_ARG;

    for (uint16_t i = 0; i < length; i++) {
        uint32_t rand_val = esp_random();
        float u = (float)rand_val / (float)UINT32_MAX;
        data[i] = -dsp_math_fast_log(u + MATH_UTILS_EPSILON) / lambda;
    }

    return ESP_OK;
}

esp_err_t dsp_gaussian_filter_kernel(float* kernel, uint16_t size, float sigma) {
    if (!kernel || size == 0 || sigma <= 0.0f) return ESP_ERR_INVALID_ARG;

    if (size % 2 == 0) size++;

    uint16_t half = size / 2;
    float sum = 0.0f;

    for (int16_t y = -half; y <= half; y++) {
        for (int16_t x = -half; x <= half; x++) {
            float arg = -(x * x + y * y) / (2.0f * sigma * sigma);
            float val = dsp_math_fast_exp(arg);
            kernel[(y + half) * size + (x + half)] = val;
            sum += val;
        }
    }

    for (uint16_t i = 0; i < size * size; i++) {
        kernel[i] /= sum;
    }

    return ESP_OK;
}

esp_err_t dsp_sobel_kernel_x(float* kernel) {
    if (!kernel) return ESP_ERR_INVALID_ARG;

    kernel[0] = -1.0f; kernel[1] = 0.0f; kernel[2] = 1.0f;
    kernel[3] = -2.0f; kernel[4] = 0.0f; kernel[5] = 2.0f;
    kernel[6] = -1.0f; kernel[7] = 0.0f; kernel[8] = 1.0f;

    return ESP_OK;
}

esp_err_t dsp_sobel_kernel_y(float* kernel) {
    if (!kernel) return ESP_ERR_INVALID_ARG;

    kernel[0] = -1.0f; kernel[1] = -2.0f; kernel[2] = -1.0f;
    kernel[3] = 0.0f;  kernel[4] = 0.0f;  kernel[5] = 0.0f;
    kernel[6] = 1.0f;  kernel[7] = 2.0f;  kernel[8] = 1.0f;

    return ESP_OK;
}

esp_err_t dsp_laplacian_kernel(float* kernel) {
    if (!kernel) return ESP_ERR_INVALID_ARG;

    kernel[0] = 0.0f;  kernel[1] = 1.0f;  kernel[2] = 0.0f;
    kernel[3] = 1.0f;  kernel[4] = -4.0f; kernel[5] = 1.0f;
    kernel[6] = 0.0f;  kernel[7] = 1.0f;  kernel[8] = 0.0f;

    return ESP_OK;
}

esp_err_t dsp_prewitt_kernel_x(float* kernel) {
    if (!kernel) return ESP_ERR_INVALID_ARG;

    kernel[0] = -1.0f; kernel[1] = 0.0f; kernel[2] = 1.0f;
    kernel[3] = -1.0f; kernel[4] = 0.0f; kernel[5] = 1.0f;
    kernel[6] = -1.0f; kernel[7] = 0.0f; kernel[8] = 1.0f;

    return ESP_OK;
}

esp_err_t dsp_prewitt_kernel_y(float* kernel) {
    if (!kernel) return ESP_ERR_INVALID_ARG;

    kernel[0] = -1.0f; kernel[1] = -1.0f; kernel[2] = -1.0f;
    kernel[3] = 0.0f;  kernel[4] = 0.0f;  kernel[5] = 0.0f;
    kernel[6] = 1.0f;  kernel[7] = 1.0f;  kernel[8] = 1.0f;

    return ESP_OK;
}

float dsp_cartesian_to_polar_r(float x, float y) {
    return dsp_math_fast_sqrt(x * x + y * y);
}

float dsp_cartesian_to_polar_theta(float x, float y) {
    return dsp_math_fast_atan2(y, x);
}

float dsp_polar_to_cartesian_x(float r, float theta) {
    return r * dsp_math_fast_cos(theta);
}

float dsp_polar_to_cartesian_y(float r, float theta) {
    return r * dsp_math_fast_sin(theta);
}

esp_err_t dsp_circular_shift(float* data, uint16_t length, int16_t shift) {
    if (!data) return ESP_ERR_INVALID_ARG;

    if (shift == 0) return ESP_OK;

    float* temp = (float*)malloc(length * sizeof(float));
    if (!temp) return ESP_ERR_NO_MEM;

    memcpy(temp, data, length * sizeof(float));

    if (shift > 0) {
        shift = shift % length;
    } else {
        shift = length - ((-shift) % length);
    }

    for (uint16_t i = 0; i < length; i++) {
        uint16_t src_idx = (i + shift) % length;
        data[i] = temp[src_idx];
    }

    free(temp);
    return ESP_OK;
}

esp_err_t dsp_linear_to_db(float* data, uint16_t length, float reference) {
    if (!data || reference <= 0.0f) return ESP_ERR_INVALID_ARG;

    for (uint16_t i = 0; i < length; i++) {
        if (data[i] < MATH_UTILS_EPSILON) {
            data[i] = -100.0f;
        } else {
            data[i] = 20.0f * dsp_math_fast_log(data[i] / reference) / dsp_math_fast_log(10.0f);
        }
    }

    return ESP_OK;
}

esp_err_t dsp_db_to_linear(float* data, uint16_t length, float reference) {
    if (!data) return ESP_ERR_INVALID_ARG;

    for (uint16_t i = 0; i < length; i++) {
        data[i] = reference * dsp_math_fast_pow(10.0f, data[i] / 20.0f);
    }

    return ESP_OK;
}

esp_err_t dsp_clip_detection(const float* input, uint16_t length, float threshold,
                              uint16_t* clip_start, uint16_t* clip_end, uint16_t* clip_count) {
    if (!input || !clip_count) return ESP_ERR_INVALID_ARG;

    *clip_count = 0;

    bool in_clip = false;
    uint16_t start_idx = 0;

    for (uint16_t i = 0; i < length; i++) {
        if (fabsf(input[i]) >= threshold) {
            if (!in_clip) {
                in_clip = true;
                start_idx = i;
            }
        } else {
            if (in_clip) {
                in_clip = false;
                (*clip_count)++;
                if (clip_start) clip_start[*clip_count - 1] = start_idx;
                if (clip_end) clip_end[*clip_count - 1] = i - 1;
            }
        }
    }

    if (in_clip) {
        (*clip_count)++;
        if (clip_start) clip_start[*clip_count - 1] = start_idx;
        if (clip_end) clip_end[*clip_count - 1] = length - 1;
    }

    return ESP_OK;
}

esp_err_t dsp_compute_harmonic_distortion(const float* input, uint16_t length,
                                           float fundamental_freq, float sample_rate,
                                           float* thd, float* thdn) {
    if (!input || !thd || !thd) return ESP_ERR_INVALID_ARG;

    *thd = 0.0f;
    *thdn = 0.0f;

    uint16_t fundamental_bin = (uint16_t)(fundamental_freq * length / sample_rate);
    if (fundamental_bin >= length / 2) fundamental_bin = length / 2 - 1;

    float* spectrum = (float*)malloc(length / 2 * sizeof(float));
    if (!spectrum) return ESP_ERR_NO_MEM;

    for (uint16_t i = 0; i < length / 2; i++) {
        spectrum[i] = dsp_math_fast_sqrt(input[i] * input[i] + input[length - 1 - i] * input[length - 1 - i]);
    }

    float fundamental_power = spectrum[fundamental_bin];
    if (fundamental_power < MATH_UTILS_EPSILON) {
        free(spectrum);
        *thd = 100.0f;
        *thdn = 100.0f;
        return ESP_OK;
    }

    float harmonic_power = 0.0f;
    float noise_power = 0.0f;

    for (uint16_t h = 2; h <= 10; h++) {
        uint16_t bin = fundamental_bin * h;
        if (bin < length / 2) {
            harmonic_power += spectrum[bin];
        }
    }

    for (uint16_t i = 0; i < length / 2; i++) {
        if (abs((int)i - fundamental_bin) > fundamental_bin / 2) {
            noise_power += spectrum[i];
        }
    }

    *thd = dsp_math_fast_sqrt(harmonic_power / fundamental_power) * 100.0f;
    *thdn = dsp_math_fast_sqrt(noise_power / fundamental_power) * 100.0f;

    free(spectrum);
    return ESP_OK;
}

esp_err_t dsp_compute_cross_correlation_fft(const float* a, const float* b, float* result,
                                             uint16_t length) {
    if (!a || !b || !result) return ESP_ERR_INVALID_ARG;

    float* fft_a_real = (float*)malloc(length * sizeof(float));
    float* fft_a_imag = (float*)malloc(length * sizeof(float));
    float* fft_b_real = (float*)malloc(length * sizeof(float));
    float* fft_b_imag = (float*)malloc(length * sizeof(float));

    if (!fft_a_real || !fft_a_imag || !fft_b_real || !fft_b_imag) {
        if (fft_a_real) free(fft_a_real);
        if (fft_a_imag) free(fft_a_imag);
        if (fft_b_real) free(fft_b_real);
        if (fft_b_imag) free(fft_b_imag);
        return ESP_ERR_NO_MEM;
    }

    for (uint16_t i = 0; i < length; i++) {
        fft_a_real[i] = a[i];
        fft_a_imag[i] = 0.0f;
        fft_b_real[i] = b[i];
        fft_b_imag[i] = 0.0f;
    }

    for (uint16_t i = 0; i < length; i++) {
        float w = -2.0f * MATH_UTILS_PI * i / length;
        float cos_w = dsp_math_fast_cos(w);
        float sin_w = dsp_math_fast_sin(w);

        float temp_real = fft_b_real[i] * cos_w - fft_b_imag[i] * sin_w;
        float temp_imag = fft_b_real[i] * sin_w + fft_b_imag[i] * cos_w;
        fft_b_real[i] = temp_real;
        fft_b_imag[i] = temp_imag;
    }

    for (uint16_t i = 0; i < length; i++) {
        float conj_real = fft_b_real[i];
        float conj_imag = -fft_b_imag[i];

        float cross_real = fft_a_real[i] * conj_real - fft_a_imag[i] * conj_imag;
        float cross_imag = fft_a_real[i] * conj_imag + fft_a_imag[i] * conj_real;

        result[i] = cross_real;
    }

    free(fft_a_real);
    free(fft_a_imag);
    free(fft_b_real);
    free(fft_b_imag);

    return ESP_OK;
}

esp_err_t dsp_compute_autocorrelation_fft(const float* input, float* result, uint16_t length) {
    return dsp_compute_cross_correlation_fft(input, input, result, length);
}

esp_err_t dsp_phase_vocoder(const float* input, float* output, uint16_t length,
                             float speed_factor, float stretch_factor) {
    if (!input || !output) return ESP_ERR_INVALID_ARG;

    uint16_t fft_size = 1024;
    uint16_t hop_size = fft_size / 4;

    float* window = (float*)malloc(fft_size * sizeof(float));
    if (!window) return ESP_ERR_NO_MEM;

    for (uint16_t i = 0; i < fft_size; i++) {
        float n_val = (float)i / (float)(fft_size - 1);
        window[i] = 0.5f - 0.5f * dsp_math_fast_cos(MATH_UTILS_TWO_PI * n_val);
    }

    uint16_t output_len = (uint16_t)(length / stretch_factor);
    memset(output, 0, output_len * sizeof(float));

    uint16_t num_frames = (length - fft_size) / hop_size + 1;
    float phase_accumulator = 0.0f;
    float* last_phase = (float*)malloc(fft_size / 2 * sizeof(float));
    if (!last_phase) {
        free(window);
        return ESP_ERR_NO_MEM;
    }

    for (uint16_t i = 0; i < fft_size / 2; i++) {
        last_phase[i] = 0.0f;
    }

    free(window);
    free(last_phase);

    memcpy(output, input, (output_len < length ? output_len : length) * sizeof(float));

    return ESP_OK;
}

void dsp_print_vector_stats(const float* data, uint16_t length, const char* name) {
    if (!data) return;

    float min_val = dsp_compute_min(data, length);
    float max_val = dsp_compute_max(data, length);
    float mean_val = dsp_vector_mean(data, length);
    float rms_val = dsp_compute_rms(data, length);

    ESP_LOGI(TAG, "=== Vector Stats: %s ===", name ? name : "data");
    ESP_LOGI(TAG, "  Length: %d", length);
    ESP_LOGI(TAG, "  Min: %.6f, Max: %.6f", min_val, max_val);
    ESP_LOGI(TAG, "  Mean: %.6f, RMS: %.6f", mean_val, rms_val);
}

void dsp_print_matrix(const float* matrix, uint16_t rows, uint16_t cols, const char* name) {
    if (!matrix) return;

    ESP_LOGI(TAG, "=== Matrix: %s (%dx%d) ===", name ? name : "matrix", rows, cols);

    for (uint16_t i = 0; i < rows && i < 10; i++) {
        char line[256] = {0};
        char num_str[32];

        for (uint16_t j = 0; j < cols && j < 10; j++) {
            snprintf(num_str, sizeof(num_str), "%8.4f ", matrix[i * cols + j]);
            strcat(line, num_str);
        }
        if (cols > 10) strcat(line, "...");

        ESP_LOGI(TAG, "  Row %d: %s", i, line);
    }

    if (rows > 10 || cols > 10) {
        ESP_LOGI(TAG, "  ... (truncated)");
    }
}
