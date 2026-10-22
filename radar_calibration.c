#include "radar_daq.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "math.h"
#include "string.h"
#include "stdlib.h"

static const char* RADAR_CAL_LOG_TAG = "RADAR_CAL";

#define RADAR_CAL_MODULE_MAGIC                  0xCA1B0000
#define RADAR_CAL_MAX_REFERENCE_PROFILES        8
#define RADAR_CAL_NOISE_FLOOR_SAMPLE_COUNT      64
#define RADAR_CAL_DC_ESTIMATION_FRAMES          32
#define RADAR_CAL_TGC_LUT_SIZE                  512
#define RADAR_CAL_TGC_MAX_GAIN_DB               60.0f
#define RADAR_CAL_TGC_MIN_GAIN_DB               0.0f
#define RADAR_CAL_TGC_ALPHA_DEFAULT             0.12f
#define RADAR_CAL_TGC_ALPHA_MIN                 0.05f
#define RADAR_CAL_TGC_ALPHA_MAX                 0.35f
#define RADAR_CAL_THERMAL_DRIFT_COEFF           0.0025f
#define RADAR_CAL_WARM_UP_FRAMES                100
#define RADAR_CAL_CONVERGENCE_THRESHOLD_DB      0.5f
#define RADAR_CAL_MAX_WARM_UP_ITERATIONS        500
#define RADAR_CAL_FINGERPRINT_VECTOR_LENGTH     RADAR_DAQ_SAMPLES_PER_ASCAN
#define RADAR_CAL_MAX_FINGERPRINT_ENTRIES       16
#define RADAR_CAL_FINGERPRINT_MATCH_THRESH_DB   3.0f
#define RADAR_CAL_VERSION_HASH                  0xBEEF1234

typedef enum {
    RADAR_CAL_STATE_DORMANT           = 0x00,
    RADAR_CAL_STATE_INITIALIZING      = 0x10,
    RADAR_CAL_STATE_NOISE_SAMPLING    = 0x20,
    RADAR_CAL_STATE_DC_ESTIMATION     = 0x30,
    RADAR_CAL_STATE_TGC_COMPUTATION   = 0x40,
    RADAR_CAL_STATE_FINGERPRINT_BUILD = 0x50,
    RADAR_CAL_STATE_CALIBRATED        = 0x60,
    RADAR_CAL_STATE_DRIFT_MONITORING  = 0x70,
    RADAR_CAL_STATE_RECALIBRATING     = 0x80,
    RADAR_CAL_STATE_VALIDATION        = 0x90,
    RADAR_CAL_STATE_ERROR_DIVERGED    = 0xE0,
    RADAR_CAL_STATE_COMPLETE          = 0xFF
} radar_cal_fsm_state_t;

typedef enum {
    RADAR_CAL_RESULT_OK                = 0,
    RADAR_CAL_RESULT_NOT_INIT          = 1,
    RADAR_CAL_RESULT_INVALID_ARG       = 2,
    RADAR_CAL_RESULT_CONVERGENCE_FAIL  = 3,
    RADAR_CAL_RESULT_NOISE_TOO_HIGH    = 4,
    RADAR_CAL_RESULT_DC_UNSTABLE       = 5,
    RADAR_CAL_RESULT_TGC_OVERFLOW      = 6,
    RADAR_CAL_RESULT_FINGERPRINT_FULL  = 7,
    RADAR_CAL_RESULT_NO_VALID_CAL      = 8,
    RADAR_CAL_RESULT_THERMAL_DRIFT_DET = 9,
    RADAR_CAL_RESULT_NVM_WRITE_FAIL    = 10,
    RADAR_CAL_RESULT_NVM_READ_FAIL     = 11
} radar_cal_result_code_t;

typedef enum {
    RADAR_CAL_SOIL_TYPE_DRY_SAND      = 0,
    RADAR_CAL_SOIL_TYPE_MOIST_CLAY    = 1,
    RADAR_CAL_SOIL_TYPE_WET_LOAM      = 2,
    RADAR_CAL_SOIL_TYPE_SATURATED     = 3,
    RADAR_CAL_SOIL_TYPE_CUSTOM        = 4
} radar_cal_soil_type_t;

typedef struct {
    float    dc_offset_volts;
    float    dc_offset_adc_counts;
    float    dc_estimation_variance;
    uint32_t frames_used_for_estimation;
    uint64_t estimation_timestamp_us;
    bool     estimation_is_valid;
    bool     drift_exceeds_threshold;
    float    drift_magnitude_counts;
} radar_cal_dc_profile_t;

typedef struct {
    float    noise_floor_rms_counts;
    float    noise_floor_peak_counts;
    float    noise_floor_db;
    float    snr_estimate_db;
    float    thermal_noise_estimate_mv;
    uint32_t samples_used;
    uint64_t measurement_timestamp_us;
    bool     noise_is_acceptable;
    float    noise_variance;
    float    noise_kurtosis;
} radar_cal_noise_profile_t;

typedef struct {
    float    lut_values[RADAR_CAL_TGC_LUT_SIZE];
    float    alpha_coefficient;
    float    initial_gain_linear;
    float    max_gain_db;
    float    min_gain_db;
    float    depth_step_m;
    float    total_depth_coverage_m;
    float    soil_attenuation_db_per_m;
    uint32_t lut_entry_count;
    uint64_t lut_build_timestamp_us;
    bool     lut_is_valid;
    bool     adaptive_mode_enabled;
    float    temperature_compensation_factor;
    radar_cal_soil_type_t target_soil_type;
} radar_cal_tgc_lut_profile_t;

typedef struct {
    float    envelope_vector[RADAR_CAL_FINGERPRINT_VECTOR_LENGTH];
    float    phase_vector[RADAR_CAL_FINGERPRINT_VECTOR_LENGTH];
    float    peak_amplitude;
    float    peak_depth_sample;
    float    pulse_width_samples;
    float    center_frequency_estimate_hz;
    float    bandwidth_estimate_hz;
    uint64_t capture_timestamp_us;
    uint32_t frame_sequence;
    radar_cal_soil_type_t measured_soil_type;
    bool     is_healthy_root_reference;
    char     label[32];
} radar_cal_fingerprint_entry_t;

typedef struct {
    radar_cal_fingerprint_entry_t entries[RADAR_CAL_MAX_FINGERPRINT_ENTRIES];
    uint8_t entry_count;
    uint8_t max_capacity;
    float   match_threshold_db;
    bool    fingerprint_db_valid;
    uint64_t last_update_timestamp_us;
    uint32_t total_matches_performed;
    uint32_t total_true_positives;
    uint32_t total_false_positives;
    float    running_accuracy_estimate;
} radar_cal_fingerprint_database_t;

typedef struct {
    uint32_t total_calibration_runs;
    uint32_t successful_calibrations;
    uint32_t failed_calibrations;
    uint32_t thermal_drift_events;
    uint32_t recalibration_triggers;
    float    last_convergence_iterations;
    float    average_convergence_iterations;
    uint64_t total_calibration_time_us;
    uint64_t last_calibration_timestamp_us;
    float    temperature_at_last_cal_celsius;
} radar_cal_session_statistics_t;

typedef struct {
    uint32_t module_magic;
    radar_cal_fsm_state_t current_state;
    radar_cal_fsm_state_t previous_state;
    radar_cal_dc_profile_t dc_profile;
    radar_cal_noise_profile_t noise_profile;
    radar_cal_tgc_lut_profile_t tgc_lut;
    radar_cal_fingerprint_database_t fingerprint_db;
    radar_cal_session_statistics_t statistics;
    SemaphoreHandle_t cal_state_mutex;
    bool module_initialized;
    float simulated_temperature_celsius;
    float last_known_temperature_celsius;
    uint32_t warm_up_frame_counter;
    bool warm_up_complete;
    uint32_t calibration_version_hash;
} radar_cal_engine_context_t;

static radar_cal_engine_context_t s_cal_ctx;
static volatile bool              s_cal_module_ready = false;

static radar_cal_result_code_t s_cal_estimate_dc_offset(void);
static radar_cal_result_code_t s_cal_measure_noise_floor(void);
static radar_cal_result_code_t s_cal_build_tgc_lut(float alpha, radar_cal_soil_type_t soil);
static radar_cal_result_code_t s_cal_validate_tgc_lut(void);
static void                    s_cal_fsm_transition(radar_cal_fsm_state_t next);
static float                   s_cal_db_to_linear(float db_value);
static float                   s_cal_linear_to_db(float linear_value);
static float                   s_cal_compute_rms(const float* data, uint32_t length);
static void                    s_cal_apply_thermal_compensation(void);

static void s_cal_fsm_transition(radar_cal_fsm_state_t next)
{
    s_cal_ctx.previous_state = s_cal_ctx.current_state;
    s_cal_ctx.current_state  = next;
}

static float s_cal_db_to_linear(float db_value)
{
    return powf(10.0f, db_value / 20.0f);
}

static float s_cal_linear_to_db(float linear_value)
{
    if (linear_value <= 0.0f) return -999.0f;
    return 20.0f * log10f(linear_value);
}

static float s_cal_compute_rms(const float* data, uint32_t length)
{
    float sum_sq = 0.0f;
    for (uint32_t i = 0; i < length; i++) {
        sum_sq += data[i] * data[i];
    }
    return sqrtf(sum_sq / (float)length);
}

static radar_cal_result_code_t s_cal_estimate_dc_offset(void)
{
    s_cal_fsm_transition(RADAR_CAL_STATE_DC_ESTIMATION);

    float accumulator   = 0.0f;
    float sum_sq        = 0.0f;
    uint32_t frame_cnt  = RADAR_CAL_DC_ESTIMATION_FRAMES;
    uint32_t sample_cnt = RADAR_DAQ_SAMPLES_PER_ASCAN;

    for (uint32_t f = 0; f < frame_cnt; f++) {
        float frame_mean = 0.0f;
        for (uint32_t s = 0; s < sample_cnt; s++) {
            float pseudo_sample = sinf((float)s * 0.05f) * 50.0f +
                                  ((float)(s % 7) - 3.0f) * 2.0f +
                                  12.5f;
            frame_mean += pseudo_sample;
        }
        frame_mean /= (float)sample_cnt;
        accumulator += frame_mean;
        sum_sq += frame_mean * frame_mean;
    }

    float dc_mean     = accumulator / (float)frame_cnt;
    float dc_variance = (sum_sq / (float)frame_cnt) - (dc_mean * dc_mean);
    if (dc_variance < 0.0f) dc_variance = 0.0f;

    s_cal_ctx.dc_profile.dc_offset_adc_counts       = dc_mean;
    s_cal_ctx.dc_profile.dc_offset_volts             = dc_mean *
        (RADAR_DAQ_ADC_FULL_SCALE_MV / 1000.0f) / (float)RADAR_DAQ_ADC_REFERENCE_COUNTS;
    s_cal_ctx.dc_profile.dc_estimation_variance      = dc_variance;
    s_cal_ctx.dc_profile.frames_used_for_estimation  = frame_cnt;
    s_cal_ctx.dc_profile.estimation_timestamp_us     = esp_timer_get_time();
    s_cal_ctx.dc_profile.estimation_is_valid         = true;
    s_cal_ctx.dc_profile.drift_exceeds_threshold     = (fabsf(dc_mean) > 100.0f);
    s_cal_ctx.dc_profile.drift_magnitude_counts      = fabsf(dc_mean);

    if (dc_variance > 500.0f) {
        return RADAR_CAL_RESULT_DC_UNSTABLE;
    }
    return RADAR_CAL_RESULT_OK;
}

static radar_cal_result_code_t s_cal_measure_noise_floor(void)
{
    s_cal_fsm_transition(RADAR_CAL_STATE_NOISE_SAMPLING);

    float noise_samples[RADAR_CAL_NOISE_FLOOR_SAMPLE_COUNT];
    float peak_val = 0.0f;
    float sum = 0.0f;
    float sum_sq = 0.0f;
    float sum_4th = 0.0f;

    for (uint32_t i = 0; i < RADAR_CAL_NOISE_FLOOR_SAMPLE_COUNT; i++) {
        float noise_val = ((float)(rand() % 100) - 50.0f) * 0.08f;
        noise_samples[i] = noise_val;
        sum += noise_val;
        sum_sq += noise_val * noise_val;
        sum_4th += noise_val * noise_val * noise_val * noise_val;
        if (fabsf(noise_val) > peak_val) peak_val = fabsf(noise_val);
    }

    float mean    = sum / RADAR_CAL_NOISE_FLOOR_SAMPLE_COUNT;
    float var     = (sum_sq / RADAR_CAL_NOISE_FLOOR_SAMPLE_COUNT) - mean * mean;
    if (var < 0.0f) var = 0.0f;
    float rms     = sqrtf(var);
    float kurtosis = 0.0f;
    if (var > 0.0f) {
        kurtosis = (sum_4th / RADAR_CAL_NOISE_FLOOR_SAMPLE_COUNT) / (var * var);
    }

    float noise_db  = s_cal_linear_to_db(rms);
    float signal_db = s_cal_linear_to_db(1.0f);
    float snr_db    = signal_db - noise_db;

    s_cal_ctx.noise_profile.noise_floor_rms_counts     = rms;
    s_cal_ctx.noise_profile.noise_floor_peak_counts    = peak_val;
    s_cal_ctx.noise_profile.noise_floor_db             = noise_db;
    s_cal_ctx.noise_profile.snr_estimate_db            = snr_db;
    s_cal_ctx.noise_profile.thermal_noise_estimate_mv  = rms * 0.805f;
    s_cal_ctx.noise_profile.samples_used               = RADAR_CAL_NOISE_FLOOR_SAMPLE_COUNT;
    s_cal_ctx.noise_profile.measurement_timestamp_us   = esp_timer_get_time();
    s_cal_ctx.noise_profile.noise_is_acceptable        = (snr_db > 20.0f);
    s_cal_ctx.noise_profile.noise_variance             = var;
    s_cal_ctx.noise_profile.noise_kurtosis             = kurtosis;

    if (!s_cal_ctx.noise_profile.noise_is_acceptable) {
        return RADAR_CAL_RESULT_NOISE_TOO_HIGH;
    }
    return RADAR_CAL_RESULT_OK;
}

static radar_cal_result_code_t s_cal_build_tgc_lut(float alpha,
                                                     radar_cal_soil_type_t soil)
{
    s_cal_fsm_transition(RADAR_CAL_STATE_TGC_COMPUTATION);

    if (alpha < RADAR_CAL_TGC_ALPHA_MIN || alpha > RADAR_CAL_TGC_ALPHA_MAX) {
        alpha = RADAR_CAL_TGC_ALPHA_DEFAULT;
    }

    float soil_attenuation_table[5] = {
        0.05f,
        0.18f,
        0.28f,
        0.40f,
        0.15f
    };
    float attenuation_db_per_m = soil_attenuation_table[(int)soil];

    float total_depth_m = 2.0f;
    float depth_per_sample = total_depth_m / (float)RADAR_CAL_TGC_LUT_SIZE;
    float max_gain_linear  = s_cal_db_to_linear(RADAR_CAL_TGC_MAX_GAIN_DB);
    float temp_comp = 1.0f + RADAR_CAL_THERMAL_DRIFT_COEFF *
                      (s_cal_ctx.simulated_temperature_celsius - 25.0f);

    for (uint32_t i = 0; i < RADAR_CAL_TGC_LUT_SIZE; i++) {
        float depth_m     = (float)i * depth_per_sample;
        float loss_db     = attenuation_db_per_m * depth_m * 2.0f;
        float gain_linear = s_cal_db_to_linear(loss_db);
        gain_linear *= temp_comp;
        if (gain_linear > max_gain_linear) gain_linear = max_gain_linear;
        if (gain_linear < 1.0f) gain_linear = 1.0f;
        s_cal_ctx.tgc_lut.lut_values[i] = gain_linear;
    }

    s_cal_ctx.tgc_lut.alpha_coefficient           = alpha;
    s_cal_ctx.tgc_lut.initial_gain_linear         = 1.0f;
    s_cal_ctx.tgc_lut.max_gain_db                 = RADAR_CAL_TGC_MAX_GAIN_DB;
    s_cal_ctx.tgc_lut.min_gain_db                 = RADAR_CAL_TGC_MIN_GAIN_DB;
    s_cal_ctx.tgc_lut.depth_step_m                = depth_per_sample;
    s_cal_ctx.tgc_lut.total_depth_coverage_m      = total_depth_m;
    s_cal_ctx.tgc_lut.soil_attenuation_db_per_m   = attenuation_db_per_m;
    s_cal_ctx.tgc_lut.lut_entry_count             = RADAR_CAL_TGC_LUT_SIZE;
    s_cal_ctx.tgc_lut.lut_build_timestamp_us      = esp_timer_get_time();
    s_cal_ctx.tgc_lut.adaptive_mode_enabled       = true;
    s_cal_ctx.tgc_lut.temperature_compensation_factor = temp_comp;
    s_cal_ctx.tgc_lut.target_soil_type            = soil;

    return RADAR_CAL_RESULT_OK;
}

static radar_cal_result_code_t s_cal_validate_tgc_lut(void)
{
    s_cal_fsm_transition(RADAR_CAL_STATE_VALIDATION);

    if (!s_cal_ctx.tgc_lut.lut_is_valid &&
        s_cal_ctx.tgc_lut.lut_entry_count != RADAR_CAL_TGC_LUT_SIZE)
    {
        return RADAR_CAL_RESULT_TGC_OVERFLOW;
    }

    float monotone_violations = 0;
    for (uint32_t i = 1; i < s_cal_ctx.tgc_lut.lut_entry_count; i++) {
        if (s_cal_ctx.tgc_lut.lut_values[i] < s_cal_ctx.tgc_lut.lut_values[i - 1] * 0.99f) {
            monotone_violations++;
        }
    }

    float violation_ratio = monotone_violations / (float)RADAR_CAL_TGC_LUT_SIZE;
    if (violation_ratio > 0.05f) {
        return RADAR_CAL_RESULT_TGC_OVERFLOW;
    }

    float rms_lut = s_cal_compute_rms(s_cal_ctx.tgc_lut.lut_values,
                                       s_cal_ctx.tgc_lut.lut_entry_count);
    if (rms_lut < 1.0f) {
        return RADAR_CAL_RESULT_TGC_OVERFLOW;
    }

    s_cal_ctx.tgc_lut.lut_is_valid = true;
    return RADAR_CAL_RESULT_OK;
}

static void s_cal_apply_thermal_compensation(void)
{
    float delta_temp = s_cal_ctx.simulated_temperature_celsius -
                       s_cal_ctx.last_known_temperature_celsius;
    if (fabsf(delta_temp) < 1.0f) return;

    float comp_factor = 1.0f + RADAR_CAL_THERMAL_DRIFT_COEFF * delta_temp;

    for (uint32_t i = 0; i < s_cal_ctx.tgc_lut.lut_entry_count; i++) {
        s_cal_ctx.tgc_lut.lut_values[i] *= comp_factor;
        float max_lin = s_cal_db_to_linear(RADAR_CAL_TGC_MAX_GAIN_DB);
        if (s_cal_ctx.tgc_lut.lut_values[i] > max_lin) {
            s_cal_ctx.tgc_lut.lut_values[i] = max_lin;
        }
        if (s_cal_ctx.tgc_lut.lut_values[i] < 1.0f) {
            s_cal_ctx.tgc_lut.lut_values[i] = 1.0f;
        }
    }

    s_cal_ctx.statistics.thermal_drift_events++;
    s_cal_ctx.last_known_temperature_celsius = s_cal_ctx.simulated_temperature_celsius;
}

radar_cal_result_code_t radar_calibration_initialize(void)
{
    memset(&s_cal_ctx, 0, sizeof(radar_cal_engine_context_t));
    s_cal_ctx.module_magic = RADAR_CAL_MODULE_MAGIC;
    s_cal_ctx.current_state = RADAR_CAL_STATE_DORMANT;
    s_cal_ctx.simulated_temperature_celsius = 25.0f;
    s_cal_ctx.last_known_temperature_celsius = 25.0f;
    s_cal_ctx.calibration_version_hash = RADAR_CAL_VERSION_HASH;
    s_cal_ctx.fingerprint_db.max_capacity = RADAR_CAL_MAX_FINGERPRINT_ENTRIES;
    s_cal_ctx.fingerprint_db.match_threshold_db = RADAR_CAL_FINGERPRINT_MATCH_THRESH_DB;

    s_cal_ctx.cal_state_mutex = xSemaphoreCreateMutex();
    if (s_cal_ctx.cal_state_mutex == NULL) return RADAR_CAL_RESULT_NOT_INIT;

    s_cal_ctx.module_initialized = true;
    s_cal_module_ready = true;
    s_cal_fsm_transition(RADAR_CAL_STATE_INITIALIZING);
    return RADAR_CAL_RESULT_OK;
}

radar_cal_result_code_t radar_calibration_run_full_sequence(radar_cal_soil_type_t soil_type)
{
    if (!s_cal_module_ready) return RADAR_CAL_RESULT_NOT_INIT;

    xSemaphoreTake(s_cal_ctx.cal_state_mutex, portMAX_DELAY);

    uint64_t t_start = esp_timer_get_time();
    s_cal_ctx.statistics.total_calibration_runs++;

    radar_cal_result_code_t result = s_cal_estimate_dc_offset();
    if (result != RADAR_CAL_RESULT_OK) goto cal_fail;

    result = s_cal_measure_noise_floor();
    if (result != RADAR_CAL_RESULT_OK) goto cal_fail;

    result = s_cal_build_tgc_lut(RADAR_CAL_TGC_ALPHA_DEFAULT, soil_type);
    if (result != RADAR_CAL_RESULT_OK) goto cal_fail;

    result = s_cal_validate_tgc_lut();
    if (result != RADAR_CAL_RESULT_OK) goto cal_fail;

    uint64_t t_end = esp_timer_get_time();
    s_cal_ctx.statistics.total_calibration_time_us += (t_end - t_start);
    s_cal_ctx.statistics.last_calibration_timestamp_us = t_end;
    s_cal_ctx.statistics.successful_calibrations++;
    s_cal_ctx.statistics.temperature_at_last_cal_celsius = s_cal_ctx.simulated_temperature_celsius;

    s_cal_fsm_transition(RADAR_CAL_STATE_CALIBRATED);
    xSemaphoreGive(s_cal_ctx.cal_state_mutex);
    return RADAR_CAL_RESULT_OK;

cal_fail:
    s_cal_ctx.statistics.failed_calibrations++;
    s_cal_fsm_transition(RADAR_CAL_STATE_ERROR_DIVERGED);
    xSemaphoreGive(s_cal_ctx.cal_state_mutex);
    return result;
}

radar_cal_result_code_t radar_calibration_get_tgc_lut(const float** out_lut_ptr,
                                                        uint32_t* out_lut_length)
{
    if (!s_cal_module_ready) return RADAR_CAL_RESULT_NOT_INIT;
    if (!s_cal_ctx.tgc_lut.lut_is_valid) return RADAR_CAL_RESULT_NO_VALID_CAL;
    if (out_lut_ptr)    *out_lut_ptr    = s_cal_ctx.tgc_lut.lut_values;
    if (out_lut_length) *out_lut_length = s_cal_ctx.tgc_lut.lut_entry_count;
    return RADAR_CAL_RESULT_OK;
}

radar_cal_result_code_t radar_calibration_get_dc_profile(radar_cal_dc_profile_t* out_dc)
{
    if (!s_cal_module_ready) return RADAR_CAL_RESULT_NOT_INIT;
    if (out_dc == NULL) return RADAR_CAL_RESULT_INVALID_ARG;
    *out_dc = s_cal_ctx.dc_profile;
    return RADAR_CAL_RESULT_OK;
}

radar_cal_result_code_t radar_calibration_get_noise_profile(radar_cal_noise_profile_t* out_noise)
{
    if (!s_cal_module_ready) return RADAR_CAL_RESULT_NOT_INIT;
    if (out_noise == NULL) return RADAR_CAL_RESULT_INVALID_ARG;
    *out_noise = s_cal_ctx.noise_profile;
    return RADAR_CAL_RESULT_OK;
}

radar_cal_result_code_t radar_calibration_add_healthy_fingerprint(
    const float* envelope_vector,
    const float* phase_vector,
    const char*  label,
    radar_cal_soil_type_t soil)
{
    if (!s_cal_module_ready) return RADAR_CAL_RESULT_NOT_INIT;
    if (envelope_vector == NULL || phase_vector == NULL) return RADAR_CAL_RESULT_INVALID_ARG;

    radar_cal_fingerprint_database_t* db = &s_cal_ctx.fingerprint_db;
    if (db->entry_count >= db->max_capacity) return RADAR_CAL_RESULT_FINGERPRINT_FULL;

    radar_cal_fingerprint_entry_t* entry = &db->entries[db->entry_count];
    memcpy(entry->envelope_vector, envelope_vector,
           RADAR_CAL_FINGERPRINT_VECTOR_LENGTH * sizeof(float));
    memcpy(entry->phase_vector, phase_vector,
           RADAR_CAL_FINGERPRINT_VECTOR_LENGTH * sizeof(float));

    if (label != NULL) {
        strncpy(entry->label, label, sizeof(entry->label) - 1);
        entry->label[sizeof(entry->label) - 1] = '\0';
    }

    float peak = 0.0f;
    uint32_t peak_idx = 0;
    for (uint32_t i = 0; i < RADAR_CAL_FINGERPRINT_VECTOR_LENGTH; i++) {
        if (envelope_vector[i] > peak) {
            peak = envelope_vector[i];
            peak_idx = i;
        }
    }
    entry->peak_amplitude       = peak;
    entry->peak_depth_sample    = (float)peak_idx;
    entry->measured_soil_type   = soil;
    entry->is_healthy_root_reference = true;
    entry->capture_timestamp_us = esp_timer_get_time();
    entry->frame_sequence       = db->entry_count;

    db->entry_count++;
    db->fingerprint_db_valid = true;
    db->last_update_timestamp_us = esp_timer_get_time();
    return RADAR_CAL_RESULT_OK;
}

radar_cal_result_code_t radar_calibration_match_fingerprint(
    const float* query_envelope,
    float*       out_euclidean_distance,
    uint32_t*    out_best_match_idx,
    bool*        out_is_anomaly)
{
    if (!s_cal_module_ready) return RADAR_CAL_RESULT_NOT_INIT;
    if (query_envelope == NULL) return RADAR_CAL_RESULT_INVALID_ARG;

    radar_cal_fingerprint_database_t* db = &s_cal_ctx.fingerprint_db;
    if (!db->fingerprint_db_valid || db->entry_count == 0) {
        return RADAR_CAL_RESULT_NO_VALID_CAL;
    }

    float best_dist = 1e18f;
    uint32_t best_idx = 0;

    for (uint32_t e = 0; e < db->entry_count; e++) {
        float dist_sq = 0.0f;
        const float* ref = db->entries[e].envelope_vector;
        for (uint32_t s = 0; s < RADAR_CAL_FINGERPRINT_VECTOR_LENGTH; s++) {
            float diff = query_envelope[s] - ref[s];
            dist_sq += diff * diff;
        }
        float dist = sqrtf(dist_sq);
        if (dist < best_dist) {
            best_dist = dist;
            best_idx  = e;
        }
    }

    float ref_rms   = s_cal_compute_rms(db->entries[best_idx].envelope_vector,
                                         RADAR_CAL_FINGERPRINT_VECTOR_LENGTH);
    float query_rms = s_cal_compute_rms(query_envelope, RADAR_CAL_FINGERPRINT_VECTOR_LENGTH);
    float ratio_db  = s_cal_linear_to_db(query_rms / (ref_rms + 1e-6f));
    bool anomaly    = (fabsf(ratio_db) > db->match_threshold_db) || (best_dist > 50.0f);

    if (out_euclidean_distance) *out_euclidean_distance = best_dist;
    if (out_best_match_idx)     *out_best_match_idx     = best_idx;
    if (out_is_anomaly)         *out_is_anomaly         = anomaly;

    db->total_matches_performed++;
    if (!anomaly) db->total_true_positives++;
    else db->total_false_positives++;

    db->running_accuracy_estimate =
        (float)db->total_true_positives / (float)db->total_matches_performed;

    return RADAR_CAL_RESULT_OK;
}

radar_cal_result_code_t radar_calibration_trigger_thermal_check(float current_temp_celsius)
{
    if (!s_cal_module_ready) return RADAR_CAL_RESULT_NOT_INIT;
    s_cal_ctx.simulated_temperature_celsius = current_temp_celsius;
    float delta = fabsf(current_temp_celsius - s_cal_ctx.last_known_temperature_celsius);
    if (delta > 5.0f) {
        s_cal_apply_thermal_compensation();
        s_cal_ctx.statistics.recalibration_triggers++;
        return RADAR_CAL_RESULT_THERMAL_DRIFT_DET;
    }
    return RADAR_CAL_RESULT_OK;
}

radar_cal_result_code_t radar_calibration_teardown(void)
{
    if (!s_cal_module_ready) return RADAR_CAL_RESULT_NOT_INIT;
    if (s_cal_ctx.cal_state_mutex) vSemaphoreDelete(s_cal_ctx.cal_state_mutex);
    memset(&s_cal_ctx, 0, sizeof(radar_cal_engine_context_t));
    s_cal_module_ready = false;
    return RADAR_CAL_RESULT_OK;
}

void radar_calibration_debug_dump(void)
{
    ESP_LOGI(RADAR_CAL_LOG_TAG, "====== CALIBRATION ENGINE DUMP ======");
    ESP_LOGI(RADAR_CAL_LOG_TAG, "FSM State          : %d", s_cal_ctx.current_state);
    ESP_LOGI(RADAR_CAL_LOG_TAG, "DC Offset counts   : %.3f", s_cal_ctx.dc_profile.dc_offset_adc_counts);
    ESP_LOGI(RADAR_CAL_LOG_TAG, "DC Offset volts    : %.4f", s_cal_ctx.dc_profile.dc_offset_volts);
    ESP_LOGI(RADAR_CAL_LOG_TAG, "Noise Floor dB     : %.2f", s_cal_ctx.noise_profile.noise_floor_db);
    ESP_LOGI(RADAR_CAL_LOG_TAG, "SNR Estimate dB    : %.2f", s_cal_ctx.noise_profile.snr_estimate_db);
    ESP_LOGI(RADAR_CAL_LOG_TAG, "TGC LUT Valid      : %s", s_cal_ctx.tgc_lut.lut_is_valid ? "YES" : "NO");
    ESP_LOGI(RADAR_CAL_LOG_TAG, "TGC Alpha          : %.4f", s_cal_ctx.tgc_lut.alpha_coefficient);
    ESP_LOGI(RADAR_CAL_LOG_TAG, "TGC Soil Atten dBm : %.3f", s_cal_ctx.tgc_lut.soil_attenuation_db_per_m);
    ESP_LOGI(RADAR_CAL_LOG_TAG, "Temperature C      : %.1f", s_cal_ctx.simulated_temperature_celsius);
    ESP_LOGI(RADAR_CAL_LOG_TAG, "Fingerprint Entries: %d", s_cal_ctx.fingerprint_db.entry_count);
    ESP_LOGI(RADAR_CAL_LOG_TAG, "Total Cal Runs     : %lu", s_cal_ctx.statistics.total_calibration_runs);
    ESP_LOGI(RADAR_CAL_LOG_TAG, "Successful Cals    : %lu", s_cal_ctx.statistics.successful_calibrations);
    ESP_LOGI(RADAR_CAL_LOG_TAG, "Thermal Drift Evts : %lu", s_cal_ctx.statistics.thermal_drift_events);
    ESP_LOGI(RADAR_CAL_LOG_TAG, "======================================");
}
