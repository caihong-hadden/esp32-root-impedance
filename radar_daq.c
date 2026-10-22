#include "radar_daq.h"
#include "sys_memory.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "driver/gptimer.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "string.h"
#include "math.h"

static const char* RADAR_DAQ_LOG_TAG = "RADAR_DAQ";

static radar_daq_engine_context_t s_daq_engine_ctx;
static volatile bool s_engine_globally_initialized = false;
static volatile uint32_t s_isr_trigger_accumulator = 0;
static volatile uint64_t s_last_isr_timestamp_us   = 0;
static volatile uint32_t s_isr_prf_deviation_count = 0;

static const uint32_t RADAR_DAQ_PRF_DEVIATION_TOLERANCE_PPM = 5000;
static const uint32_t RADAR_DAQ_MAX_CONSECUTIVE_ERRORS      = 10;
static const uint32_t RADAR_DAQ_RECOVERY_DELAY_MS           = 50;
static const float    RADAR_DAQ_DC_OFFSET_ALPHA_FACTOR      = 0.01f;
static const uint32_t RADAR_DAQ_TELEMETRY_UPDATE_INTERVAL_FRAMES = 100;

static radar_daq_error_code_t s_internal_spi_bus_initialize(void);
static radar_daq_error_code_t s_internal_spi_device_attach(void);
static radar_daq_error_code_t s_internal_gptimer_configure(uint32_t prf_hz);
static radar_daq_error_code_t s_internal_trigger_gpio_configure(void);
static radar_daq_error_code_t s_internal_pingpong_buffer_allocate(void);
static radar_daq_error_code_t s_internal_frame_ringbuf_allocate(void);
static radar_daq_error_code_t s_internal_validate_configuration(const radar_daq_configuration_params_t* cfg);
static void                   s_internal_fsm_transition(radar_daq_fsm_state_t next_state);
static void                   s_internal_apply_dc_offset_correction(radar_daq_ascan_frame_t* frame);
static void                   s_internal_compute_frame_crc32(radar_daq_ascan_frame_t* frame);
static bool                   s_internal_verify_frame_crc32(const radar_daq_ascan_frame_t* frame);
static void                   s_internal_normalize_raw_to_float(radar_daq_ascan_frame_t* frame);
static void                   s_internal_fill_frame_metadata(radar_daq_ascan_frame_t* frame, uint8_t slot_idx);
static radar_daq_error_code_t s_internal_hardware_self_test_spi_loopback(bool* out_pass);
static void                   s_internal_telemetry_update_periodic(void);
static void                   s_internal_trigger_pulse_emit(void);
static void                   s_internal_handle_dma_eof_event(radar_daq_ascan_frame_t* completed_frame);
static void                   s_internal_error_recovery_procedure(radar_daq_error_code_t err);

static bool IRAM_ATTR s_gptimer_alarm_isr_handler(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx);
static void           s_daq_main_acquisition_task(void* pvParam);

static uint32_t s_crc32_lookup_table[256];
static bool     s_crc32_table_initialized = false;

static void s_crc32_table_build(void)
{
    uint32_t polynomial = 0xEDB88320U;
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t crc = i;
        for (uint32_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ polynomial;
            } else {
                crc >>= 1;
            }
        }
        s_crc32_lookup_table[i] = crc;
    }
    s_crc32_table_initialized = true;
}

static uint32_t s_crc32_compute_block(const uint8_t* data, size_t length)
{
    if (!s_crc32_table_initialized) {
        s_crc32_table_build();
    }
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0; i < length; i++) {
        uint8_t byte_val = data[i];
        uint32_t lookup_idx = (crc ^ byte_val) & 0xFF;
        crc = (crc >> 8) ^ s_crc32_lookup_table[lookup_idx];
    }
    return crc ^ 0xFFFFFFFFU;
}

static bool IRAM_ATTR s_gptimer_alarm_isr_handler(gptimer_handle_t timer,
                                                    const gptimer_alarm_event_data_t* edata,
                                                    void* user_ctx)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    uint64_t now_us = esp_timer_get_time();
    uint64_t interval_us = now_us - s_last_isr_timestamp_us;
    s_last_isr_timestamp_us = now_us;

    s_isr_trigger_accumulator++;

    uint32_t expected_interval_us = 1000000UL / s_daq_engine_ctx.active_config.prf_frequency_hz;
    int64_t deviation_us = (int64_t)interval_us - (int64_t)expected_interval_us;
    uint32_t deviation_ppm = (uint32_t)(((int64_t)deviation_us < 0 ? -(int64_t)deviation_us : (int64_t)deviation_us)
                              * 1000000LL / (int64_t)expected_interval_us);
    if (deviation_ppm > RADAR_DAQ_PRF_DEVIATION_TOLERANCE_PPM) {
        s_isr_prf_deviation_count++;
    }

    if (s_daq_engine_ctx.acquisition_run_flag &&
        s_daq_engine_ctx.current_fsm_state == RADAR_DAQ_STATE_ACQUIRING)
    {
        gpio_set_level(RADAR_TRIGGER_GPIO, RADAR_TRIGGER_ACTIVE_LEVEL);

        volatile uint32_t pulse_delay = RADAR_TRIGGER_PULSE_WIDTH_US * 40;
        while (pulse_delay--) {
            __asm__ volatile ("nop");
        }

        gpio_set_level(RADAR_TRIGGER_GPIO, !RADAR_TRIGGER_ACTIVE_LEVEL);

        s_daq_engine_ctx.pending_trigger_count++;

        xSemaphoreGiveFromISR(s_daq_engine_ctx.dma_complete_semaphore, &higher_priority_task_woken);
    }

    return higher_priority_task_woken == pdTRUE;
}

static radar_daq_error_code_t s_internal_spi_bus_initialize(void)
{
    spi_bus_config_t bus_cfg = {
        .miso_io_num     = RADAR_SPI_MISO_GPIO,
        .mosi_io_num     = RADAR_SPI_MOSI_GPIO,
        .sclk_io_num     = RADAR_SPI_CLK_GPIO,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = RADAR_SPI_MAX_TRANSFER_BYTES,
        .flags           = SPICOMMON_BUSFLAG_MASTER,
        .intr_flags      = ESP_INTR_FLAG_IRAM
    };

    esp_err_t ret = spi_bus_initialize(RADAR_SPI_HOST_BUS, &bus_cfg, RADAR_SPI_DMA_CHANNEL);
    if (ret != ESP_OK) {
        return RADAR_DAQ_ERR_SPI_BUS_FAULT;
    }

    return RADAR_DAQ_ERR_OK;
}

static radar_daq_error_code_t s_internal_spi_device_attach(void)
{
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = RADAR_SPI_BUS_CLOCK_HZ,
        .mode           = 0,
        .spics_io_num   = RADAR_SPI_CS_GPIO,
        .queue_size     = RADAR_SPI_QUEUE_DEPTH,
        .flags          = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY,
        .pre_cb         = NULL,
        .post_cb        = NULL,
        .input_delay_ns = 10
    };

    esp_err_t ret = spi_bus_add_device(RADAR_SPI_HOST_BUS,
                                        &dev_cfg,
                                        &s_daq_engine_ctx.spi_device_handle);
    if (ret != ESP_OK) {
        return RADAR_DAQ_ERR_SPI_BUS_FAULT;
    }

    return RADAR_DAQ_ERR_OK;
}

static radar_daq_error_code_t s_internal_gptimer_configure(uint32_t prf_hz)
{
    if (prf_hz < RADAR_GPTIMER_PRF_MIN_HZ || prf_hz > RADAR_GPTIMER_PRF_MAX_HZ) {
        return RADAR_DAQ_ERR_PRF_OUT_OF_RANGE;
    }

    gptimer_config_t timer_cfg = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = RADAR_GPTIMER_RESOLUTION_HZ,
        .flags         = { .intr_shared = false }
    };

    esp_err_t ret = gptimer_new_timer(&timer_cfg, &s_daq_engine_ctx.gptimer_handle);
    if (ret != ESP_OK) {
        return RADAR_DAQ_ERR_TIMER_CONFIG_FAIL;
    }

    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count                = RADAR_GPTIMER_RESOLUTION_HZ / prf_hz,
        .reload_count               = 0,
        .flags.auto_reload_on_alarm = RADAR_GPTIMER_ALARM_AUTO_RELOAD
    };

    ret = gptimer_set_alarm_action(s_daq_engine_ctx.gptimer_handle, &alarm_cfg);
    if (ret != ESP_OK) {
        gptimer_del_timer(s_daq_engine_ctx.gptimer_handle);
        return RADAR_DAQ_ERR_TIMER_CONFIG_FAIL;
    }

    gptimer_event_callbacks_t cbs = {
        .on_alarm = s_gptimer_alarm_isr_handler
    };

    ret = gptimer_register_event_callbacks(s_daq_engine_ctx.gptimer_handle,
                                            &cbs,
                                            &s_daq_engine_ctx);
    if (ret != ESP_OK) {
        gptimer_del_timer(s_daq_engine_ctx.gptimer_handle);
        return RADAR_DAQ_ERR_TIMER_CONFIG_FAIL;
    }

    ret = gptimer_enable(s_daq_engine_ctx.gptimer_handle);
    if (ret != ESP_OK) {
        gptimer_del_timer(s_daq_engine_ctx.gptimer_handle);
        return RADAR_DAQ_ERR_TIMER_CONFIG_FAIL;
    }

    return RADAR_DAQ_ERR_OK;
}

static radar_daq_error_code_t s_internal_trigger_gpio_configure(void)
{
    gpio_config_t trigger_io_cfg = {
        .pin_bit_mask = (1ULL << RADAR_TRIGGER_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };

    esp_err_t ret = gpio_config(&trigger_io_cfg);
    if (ret != ESP_OK) {
        return RADAR_DAQ_ERR_TRIGGER_GPIO_FAULT;
    }

    gpio_set_level(RADAR_TRIGGER_GPIO, 0);

    return RADAR_DAQ_ERR_OK;
}

static radar_daq_error_code_t s_internal_pingpong_buffer_allocate(void)
{
    radar_daq_pingpong_buffer_manager_t* mgr = &s_daq_engine_ctx.pingpong_manager;

    memset(mgr, 0, sizeof(radar_daq_pingpong_buffer_manager_t));

    for (int slot = 0; slot < RADAR_DAQ_PINGPONG_BUFFER_COUNT; slot++) {
        memset(&mgr->slot[slot], 0, sizeof(radar_daq_ascan_frame_t));
        mgr->slot[slot].metadata.frame_magic_header = RADAR_DAQ_MAGIC_FRAME_HEADER;
        mgr->slot[slot].metadata.frame_magic_footer = RADAR_DAQ_MAGIC_FRAME_FOOTER;
        mgr->slot[slot].metadata.samples_per_ascan  = RADAR_DAQ_SAMPLES_PER_ASCAN;
        mgr->slot[slot].metadata.bytes_per_sample   = RADAR_DAQ_BYTES_PER_SAMPLE;
        mgr->slot[slot].metadata.total_payload_bytes = RADAR_DAQ_ASCAN_RAW_BYTES;
        mgr->slot[slot].is_actively_locked_by_dsp   = false;
        mgr->slot[slot].access_reference_count      = 0;
    }

    mgr->active_write_slot_index = 0;
    mgr->active_read_slot_index  = 1;
    mgr->flip_pending            = false;
    mgr->total_successful_swaps  = 0;
    mgr->total_swap_collisions   = 0;
    mgr->last_swap_timestamp_us  = 0;

    mgr->swap_synchronization_mutex = xSemaphoreCreateMutex();
    if (mgr->swap_synchronization_mutex == NULL) {
        return RADAR_DAQ_ERR_BUFFER_LOCK_FAIL;
    }

    return RADAR_DAQ_ERR_OK;
}

static radar_daq_error_code_t s_internal_frame_ringbuf_allocate(void)
{
    size_t ringbuf_item_size = sizeof(radar_daq_ascan_frame_t);
    size_t ringbuf_total_size = ringbuf_item_size * RADAR_DAQ_FRAME_QUEUE_DEPTH;

    s_daq_engine_ctx.frame_output_ringbuf = xRingbufferCreate(ringbuf_total_size,
                                                                RINGBUF_TYPE_NOSPLIT);
    if (s_daq_engine_ctx.frame_output_ringbuf == NULL) {
        return RADAR_DAQ_ERR_DMA_OVERFLOW;
    }

    return RADAR_DAQ_ERR_OK;
}

static radar_daq_error_code_t s_internal_validate_configuration(const radar_daq_configuration_params_t* cfg)
{
    if (cfg == NULL) {
        return RADAR_DAQ_ERR_NOT_INITIALIZED;
    }
    if (cfg->prf_frequency_hz < RADAR_GPTIMER_PRF_MIN_HZ ||
        cfg->prf_frequency_hz > RADAR_GPTIMER_PRF_MAX_HZ) {
        return RADAR_DAQ_ERR_PRF_OUT_OF_RANGE;
    }
    if (cfg->burst_mode_frame_count > RADAR_DAQ_BURST_MODE_MAX_FRAMES) {
        return RADAR_DAQ_ERR_PRF_OUT_OF_RANGE;
    }
    if (cfg->dc_offset_correction_mv < -500.0f || cfg->dc_offset_correction_mv > 500.0f) {
        return RADAR_DAQ_ERR_CALIBRATION_INVALID;
    }
    return RADAR_DAQ_ERR_OK;
}

static void s_internal_fsm_transition(radar_daq_fsm_state_t next_state)
{
    s_daq_engine_ctx.previous_fsm_state = s_daq_engine_ctx.current_fsm_state;
    s_daq_engine_ctx.current_fsm_state  = next_state;
}

static void s_internal_apply_dc_offset_correction(radar_daq_ascan_frame_t* frame)
{
    int64_t dc_accumulator = 0;
    for (int i = 0; i < RADAR_DAQ_SAMPLES_PER_ASCAN; i++) {
        dc_accumulator += (int64_t)frame->corrected_sample_array[i];
    }
    int16_t dc_mean = (int16_t)(dc_accumulator / RADAR_DAQ_SAMPLES_PER_ASCAN);

    for (int i = 0; i < RADAR_DAQ_SAMPLES_PER_ASCAN; i++) {
        int32_t corrected = (int32_t)frame->corrected_sample_array[i] - (int32_t)dc_mean;
        if (corrected > 32767)  corrected = 32767;
        if (corrected < -32768) corrected = -32768;
        frame->corrected_sample_array[i] = (int16_t)corrected;
    }
}

static void s_internal_compute_frame_crc32(radar_daq_ascan_frame_t* frame)
{
    frame->metadata.crc32_payload_checksum = s_crc32_compute_block(
        frame->raw_dma_buffer,
        RADAR_DAQ_ASCAN_RAW_BYTES
    );
}

static bool s_internal_verify_frame_crc32(const radar_daq_ascan_frame_t* frame)
{
    uint32_t computed = s_crc32_compute_block(
        frame->raw_dma_buffer,
        RADAR_DAQ_ASCAN_RAW_BYTES
    );
    return (computed == frame->metadata.crc32_payload_checksum);
}

static void s_internal_normalize_raw_to_float(radar_daq_ascan_frame_t* frame)
{
    float inv_scale = 1.0f / (float)(RADAR_DAQ_ADC_REFERENCE_COUNTS / 2);
    for (int i = 0; i < RADAR_DAQ_SAMPLES_PER_ASCAN; i++) {
        frame->normalized_float_array[i] = (float)frame->corrected_sample_array[i] * inv_scale;
        if (frame->normalized_float_array[i] >  1.0f) frame->normalized_float_array[i] =  1.0f;
        if (frame->normalized_float_array[i] < -1.0f) frame->normalized_float_array[i] = -1.0f;
    }
}

static void s_internal_fill_frame_metadata(radar_daq_ascan_frame_t* frame, uint8_t slot_idx)
{
    uint64_t now_us = esp_timer_get_time();
    frame->metadata.capture_timestamp_us  = now_us;
    frame->metadata.frame_sequence_number = s_daq_engine_ctx.telemetry.total_frames_captured;
    frame->metadata.buffer_slot_index     = slot_idx;
    frame->metadata.sync_word             = RADAR_DAQ_SYNC_WORD_PATTERN;
    frame->metadata.trigger_source_id     = (uint8_t)s_daq_engine_ctx.active_config.trigger_mode;
    frame->metadata.dma_completed_without_error = true;
    frame->metadata.overflow_detected_this_frame = false;
    frame->metadata.calibration_frame_flag = false;

    if (s_daq_engine_ctx.telemetry.last_frame_timestamp_us > 0) {
        uint64_t delta_us = now_us - s_daq_engine_ctx.telemetry.last_frame_timestamp_us;
        frame->metadata.trigger_interval_actual_us = (float)delta_us;
    } else {
        frame->metadata.trigger_interval_actual_us = 0.0f;
    }
    frame->metadata.spi_clock_actual_mhz = (float)RADAR_SPI_BUS_CLOCK_HZ / 1000000.0f;
}

static void s_internal_handle_dma_eof_event(radar_daq_ascan_frame_t* completed_frame)
{
    uint64_t t_start = esp_timer_get_time();

    s_internal_fill_frame_metadata(completed_frame,
        s_daq_engine_ctx.pingpong_manager.active_write_slot_index);

    const uint8_t* raw_u8 = completed_frame->raw_dma_buffer;
    for (int i = 0; i < RADAR_DAQ_SAMPLES_PER_ASCAN; i++) {
        uint16_t raw_val  = ((uint16_t)raw_u8[i * 2 + 1] << 8) | raw_u8[i * 2];
        completed_frame->corrected_sample_array[i] = (int16_t)(raw_val - (RADAR_DAQ_ADC_REFERENCE_COUNTS / 2));
    }

    s_internal_apply_dc_offset_correction(completed_frame);

    if (s_daq_engine_ctx.active_config.enable_frame_crc32_verification) {
        s_internal_compute_frame_crc32(completed_frame);
    }

    s_internal_normalize_raw_to_float(completed_frame);

    uint64_t t_end = esp_timer_get_time();
    float processing_us = (float)(t_end - t_start);

    s_daq_engine_ctx.telemetry.total_frames_captured++;
    s_daq_engine_ctx.telemetry.last_frame_timestamp_us = t_start;

    if (processing_us > s_daq_engine_ctx.telemetry.peak_dma_transfer_time_us) {
        s_daq_engine_ctx.telemetry.peak_dma_transfer_time_us = processing_us;
    }
    float alpha = RADAR_DAQ_DC_OFFSET_ALPHA_FACTOR;
    s_daq_engine_ctx.telemetry.average_dma_transfer_time_us =
        alpha * processing_us + (1.0f - alpha) *
        s_daq_engine_ctx.telemetry.average_dma_transfer_time_us;

    BaseType_t sent = xRingbufferSend(
        s_daq_engine_ctx.frame_output_ringbuf,
        completed_frame,
        sizeof(radar_daq_ascan_frame_t),
        0
    );
    if (sent != pdTRUE) {
        s_daq_engine_ctx.telemetry.total_frames_dropped++;
        s_daq_engine_ctx.telemetry.total_overflow_events++;
        completed_frame->metadata.overflow_detected_this_frame = true;
        if (s_daq_engine_ctx.callback_registry.on_buffer_overflow) {
            s_daq_engine_ctx.callback_registry.on_buffer_overflow(
                s_daq_engine_ctx.telemetry.total_frames_dropped,
                s_daq_engine_ctx.callback_registry.callback_user_context
            );
        }
    } else {
        if (s_daq_engine_ctx.callback_registry.on_frame_ready) {
            s_daq_engine_ctx.callback_registry.on_frame_ready(
                completed_frame,
                s_daq_engine_ctx.callback_registry.callback_user_context
            );
        }
    }

    radar_daq_pingpong_buffer_manager_t* mgr = &s_daq_engine_ctx.pingpong_manager;
    if (xSemaphoreTake(mgr->swap_synchronization_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        uint8_t tmp = mgr->active_write_slot_index;
        mgr->active_write_slot_index = mgr->active_read_slot_index;
        mgr->active_read_slot_index  = tmp;
        mgr->total_successful_swaps++;
        mgr->last_swap_timestamp_us = esp_timer_get_time();
        xSemaphoreGive(mgr->swap_synchronization_mutex);
    } else {
        mgr->total_swap_collisions++;
    }
}

static void s_internal_error_recovery_procedure(radar_daq_error_code_t err)
{
    s_internal_fsm_transition(RADAR_DAQ_STATE_ERROR_RECOVERY);
    s_daq_engine_ctx.telemetry.consecutive_error_streak++;

    vTaskDelay(pdMS_TO_TICKS(RADAR_DAQ_RECOVERY_DELAY_MS));

    if (s_daq_engine_ctx.telemetry.consecutive_error_streak >= RADAR_DAQ_MAX_CONSECUTIVE_ERRORS) {
        s_internal_fsm_transition(RADAR_DAQ_STATE_HARDWARE_FAULT);
        s_daq_engine_ctx.acquisition_run_flag = false;
        if (s_daq_engine_ctx.callback_registry.on_acquisition_error) {
            s_daq_engine_ctx.callback_registry.on_acquisition_error(
                err,
                RADAR_DAQ_STATE_HARDWARE_FAULT,
                s_daq_engine_ctx.callback_registry.callback_user_context
            );
        }
        return;
    }

    s_internal_fsm_transition(RADAR_DAQ_STATE_ACQUIRING);
    s_daq_engine_ctx.telemetry.consecutive_error_streak = 0;
}

static void s_daq_main_acquisition_task(void* pvParam)
{
    (void)pvParam;
    radar_daq_engine_context_t* ctx = &s_daq_engine_ctx;

    spi_transaction_t spi_trans;
    memset(&spi_trans, 0, sizeof(spi_trans));
    spi_trans.rxlength = RADAR_DAQ_ASCAN_RAW_BYTES * 8;
    spi_trans.flags    = SPI_TRANS_USE_RXDATA;

    s_internal_fsm_transition(RADAR_DAQ_STATE_ACQUIRING);

    while (ctx->acquisition_run_flag)
    {
        BaseType_t sem_result = xSemaphoreTake(ctx->dma_complete_semaphore,
                                                pdMS_TO_TICKS(RADAR_DAQ_WATCHDOG_TIMEOUT_MS));
        if (sem_result != pdTRUE) {
            ctx->telemetry.total_trigger_misses++;
            s_internal_error_recovery_procedure(RADAR_DAQ_ERR_DMA_TIMEOUT);
            continue;
        }

        uint8_t write_slot = ctx->pingpong_manager.active_write_slot_index;
        radar_daq_ascan_frame_t* active_frame = &ctx->pingpong_manager.slot[write_slot];

        spi_trans.rx_buffer = active_frame->raw_dma_buffer;
        spi_trans.rxlength  = RADAR_DAQ_ASCAN_RAW_BYTES * 8;

        esp_err_t spi_ret = spi_device_transmit(ctx->spi_device_handle, &spi_trans);
        if (spi_ret != ESP_OK) {
            ctx->telemetry.total_dma_errors++;
            s_internal_error_recovery_procedure(RADAR_DAQ_ERR_SPI_BUS_FAULT);
            continue;
        }

        ctx->pending_trigger_count = (ctx->pending_trigger_count > 0)
                                      ? ctx->pending_trigger_count - 1
                                      : 0;

        s_internal_handle_dma_eof_event(active_frame);

        if (ctx->telemetry.total_frames_captured % RADAR_DAQ_TELEMETRY_UPDATE_INTERVAL_FRAMES == 0) {
            s_internal_telemetry_update_periodic();
        }

        ctx->telemetry.consecutive_error_streak = 0;
        taskYIELD();
    }

    s_internal_fsm_transition(RADAR_DAQ_STATE_IDLE);
    vTaskDelete(NULL);
}

static void s_internal_telemetry_update_periodic(void)
{
    radar_daq_runtime_telemetry_t* tele = &s_daq_engine_ctx.telemetry;

    uint64_t elapsed_us = esp_timer_get_time() - tele->session_start_timestamp_us;
    if (elapsed_us > 0) {
        tele->actual_prf_hz_measured =
            (float)tele->total_frames_captured / ((float)elapsed_us / 1000000.0f);
    }

    float total_transfer_us = tele->average_dma_transfer_time_us * (float)tele->total_frames_captured;
    tele->spi_bus_utilization_percent =
        (total_transfer_us / (float)elapsed_us) * 100.0f;
    if (tele->spi_bus_utilization_percent > 100.0f) {
        tele->spi_bus_utilization_percent = 100.0f;
    }

    UBaseType_t queue_fill = 0;
    vRingbufferGetInfo(s_daq_engine_ctx.frame_output_ringbuf,
                       NULL, NULL, NULL, NULL, &queue_fill);
    if (queue_fill > tele->maximum_queue_fill_level) {
        tele->maximum_queue_fill_level = queue_fill;
    }
}

static radar_daq_error_code_t s_internal_hardware_self_test_spi_loopback(bool* out_pass)
{
    *out_pass = false;
    uint8_t tx_pattern[64];
    uint8_t rx_pattern[64];

    for (int i = 0; i < 64; i++) {
        tx_pattern[i] = (uint8_t)(i ^ 0xA5);
    }
    memset(rx_pattern, 0, 64);

    spi_transaction_t loopback_trans = {
        .length    = 64 * 8,
        .rxlength  = 64 * 8,
        .tx_buffer = tx_pattern,
        .rx_buffer = rx_pattern,
        .flags     = 0
    };

    esp_err_t ret = spi_device_transmit(s_daq_engine_ctx.spi_device_handle, &loopback_trans);
    if (ret != ESP_OK) {
        return RADAR_DAQ_ERR_SPI_BUS_FAULT;
    }

    uint32_t mismatch_count = 0;
    for (int i = 0; i < 64; i++) {
        if (tx_pattern[i] != rx_pattern[i]) {
            mismatch_count++;
        }
    }

    *out_pass = (mismatch_count == 0);
    return RADAR_DAQ_ERR_OK;
}

radar_daq_error_code_t radar_daq_engine_initialize(const radar_daq_configuration_params_t* config,
                                                     const radar_daq_callback_registry_t* callbacks)
{
    if (s_engine_globally_initialized) {
        return RADAR_DAQ_ERR_ALREADY_RUNNING;
    }

    radar_daq_error_code_t err = s_internal_validate_configuration(config);
    if (err != RADAR_DAQ_ERR_OK) {
        return err;
    }

    memset(&s_daq_engine_ctx, 0, sizeof(radar_daq_engine_context_t));
    s_crc32_table_build();

    s_daq_engine_ctx.init_magic_token   = RADAR_DAQ_MAGIC_INIT_TOKEN;
    s_daq_engine_ctx.current_fsm_state  = RADAR_DAQ_STATE_UNINITIALIZED;
    s_daq_engine_ctx.active_config      = *config;

    if (callbacks != NULL) {
        s_daq_engine_ctx.callback_registry = *callbacks;
    }

    s_daq_engine_ctx.dma_complete_semaphore = xSemaphoreCreateBinary();
    if (s_daq_engine_ctx.dma_complete_semaphore == NULL) {
        return RADAR_DAQ_ERR_BUFFER_LOCK_FAIL;
    }

    s_daq_engine_ctx.config_update_mutex = xSemaphoreCreateMutex();
    if (s_daq_engine_ctx.config_update_mutex == NULL) {
        vSemaphoreDelete(s_daq_engine_ctx.dma_complete_semaphore);
        return RADAR_DAQ_ERR_BUFFER_LOCK_FAIL;
    }

    err = s_internal_trigger_gpio_configure();
    if (err != RADAR_DAQ_ERR_OK) goto init_fail;

    err = s_internal_spi_bus_initialize();
    if (err != RADAR_DAQ_ERR_OK) goto init_fail;

    err = s_internal_spi_device_attach();
    if (err != RADAR_DAQ_ERR_OK) goto init_fail;

    err = s_internal_gptimer_configure(config->prf_frequency_hz);
    if (err != RADAR_DAQ_ERR_OK) goto init_fail;

    err = s_internal_pingpong_buffer_allocate();
    if (err != RADAR_DAQ_ERR_OK) goto init_fail;

    err = s_internal_frame_ringbuf_allocate();
    if (err != RADAR_DAQ_ERR_OK) goto init_fail;

    s_daq_engine_ctx.telemetry.session_start_timestamp_us = esp_timer_get_time();
    s_last_isr_timestamp_us = s_daq_engine_ctx.telemetry.session_start_timestamp_us;

    s_internal_fsm_transition(RADAR_DAQ_STATE_IDLE);
    s_engine_globally_initialized = true;

    return RADAR_DAQ_ERR_OK;

init_fail:
    if (s_daq_engine_ctx.dma_complete_semaphore)  vSemaphoreDelete(s_daq_engine_ctx.dma_complete_semaphore);
    if (s_daq_engine_ctx.config_update_mutex)     vSemaphoreDelete(s_daq_engine_ctx.config_update_mutex);
    if (s_daq_engine_ctx.frame_output_ringbuf)    vRingbufferDelete(s_daq_engine_ctx.frame_output_ringbuf);
    memset(&s_daq_engine_ctx, 0, sizeof(radar_daq_engine_context_t));
    return err;
}

radar_daq_error_code_t radar_daq_engine_start_acquisition(void)
{
    if (!s_engine_globally_initialized) return RADAR_DAQ_ERR_NOT_INITIALIZED;
    if (s_daq_engine_ctx.current_fsm_state == RADAR_DAQ_STATE_ACQUIRING) {
        return RADAR_DAQ_ERR_ALREADY_RUNNING;
    }
    if (s_daq_engine_ctx.current_fsm_state != RADAR_DAQ_STATE_IDLE) {
        return RADAR_DAQ_ERR_NOT_INITIALIZED;
    }

    s_internal_fsm_transition(RADAR_DAQ_STATE_ARMING);
    s_daq_engine_ctx.acquisition_run_flag = true;
    s_daq_engine_ctx.pending_trigger_count = 0;
    s_isr_trigger_accumulator = 0;
    s_isr_prf_deviation_count = 0;

    BaseType_t task_ret = xTaskCreatePinnedToCore(
        s_daq_main_acquisition_task,
        "radar_daq_acq",
        RADAR_DAQ_TASK_STACK_SIZE,
        NULL,
        RADAR_DAQ_TASK_PRIORITY,
        &s_daq_engine_ctx.daq_task_handle,
        RADAR_DAQ_TASK_CORE_AFFINITY
    );
    if (task_ret != pdPASS) {
        s_daq_engine_ctx.acquisition_run_flag = false;
        s_internal_fsm_transition(RADAR_DAQ_STATE_IDLE);
        return RADAR_DAQ_ERR_INTERNAL_FATAL;
    }

    esp_err_t timer_ret = gptimer_start(s_daq_engine_ctx.gptimer_handle);
    if (timer_ret != ESP_OK) {
        s_daq_engine_ctx.acquisition_run_flag = false;
        s_internal_fsm_transition(RADAR_DAQ_STATE_IDLE);
        return RADAR_DAQ_ERR_TIMER_CONFIG_FAIL;
    }

    s_internal_fsm_transition(RADAR_DAQ_STATE_ARMED_WAITING);
    return RADAR_DAQ_ERR_OK;
}

radar_daq_error_code_t radar_daq_engine_stop_acquisition(void)
{
    if (!s_engine_globally_initialized) return RADAR_DAQ_ERR_NOT_INITIALIZED;

    gptimer_stop(s_daq_engine_ctx.gptimer_handle);

    s_daq_engine_ctx.acquisition_run_flag = false;

    xSemaphoreGive(s_daq_engine_ctx.dma_complete_semaphore);

    vTaskDelay(pdMS_TO_TICKS(100));

    s_internal_fsm_transition(RADAR_DAQ_STATE_IDLE);
    return RADAR_DAQ_ERR_OK;
}

radar_daq_error_code_t radar_daq_engine_pause_acquisition(void)
{
    if (!s_engine_globally_initialized) return RADAR_DAQ_ERR_NOT_INITIALIZED;
    gptimer_stop(s_daq_engine_ctx.gptimer_handle);
    s_internal_fsm_transition(RADAR_DAQ_STATE_PAUSED);
    return RADAR_DAQ_ERR_OK;
}

radar_daq_error_code_t radar_daq_engine_resume_acquisition(void)
{
    if (!s_engine_globally_initialized) return RADAR_DAQ_ERR_NOT_INITIALIZED;
    if (s_daq_engine_ctx.current_fsm_state != RADAR_DAQ_STATE_PAUSED) {
        return RADAR_DAQ_ERR_NOT_INITIALIZED;
    }
    esp_err_t ret = gptimer_start(s_daq_engine_ctx.gptimer_handle);
    if (ret != ESP_OK) return RADAR_DAQ_ERR_TIMER_CONFIG_FAIL;
    s_internal_fsm_transition(RADAR_DAQ_STATE_ACQUIRING);
    return RADAR_DAQ_ERR_OK;
}

radar_daq_error_code_t radar_daq_engine_teardown(void)
{
    if (!s_engine_globally_initialized) return RADAR_DAQ_ERR_NOT_INITIALIZED;

    radar_daq_engine_stop_acquisition();
    vTaskDelay(pdMS_TO_TICKS(200));

    gptimer_disable(s_daq_engine_ctx.gptimer_handle);
    gptimer_del_timer(s_daq_engine_ctx.gptimer_handle);

    spi_bus_remove_device(s_daq_engine_ctx.spi_device_handle);
    spi_bus_free(RADAR_SPI_HOST_BUS);

    if (s_daq_engine_ctx.frame_output_ringbuf) {
        vRingbufferDelete(s_daq_engine_ctx.frame_output_ringbuf);
    }
    if (s_daq_engine_ctx.pingpong_manager.swap_synchronization_mutex) {
        vSemaphoreDelete(s_daq_engine_ctx.pingpong_manager.swap_synchronization_mutex);
    }
    if (s_daq_engine_ctx.dma_complete_semaphore) {
        vSemaphoreDelete(s_daq_engine_ctx.dma_complete_semaphore);
    }
    if (s_daq_engine_ctx.config_update_mutex) {
        vSemaphoreDelete(s_daq_engine_ctx.config_update_mutex);
    }

    memset(&s_daq_engine_ctx, 0, sizeof(radar_daq_engine_context_t));
    s_engine_globally_initialized = false;
    s_isr_trigger_accumulator = 0;

    return RADAR_DAQ_ERR_OK;
}

radar_daq_error_code_t radar_daq_engine_reconfigure_prf(uint32_t new_prf_hz)
{
    if (!s_engine_globally_initialized) return RADAR_DAQ_ERR_NOT_INITIALIZED;
    if (new_prf_hz < RADAR_GPTIMER_PRF_MIN_HZ || new_prf_hz > RADAR_GPTIMER_PRF_MAX_HZ) {
        return RADAR_DAQ_ERR_PRF_OUT_OF_RANGE;
    }

    if (xSemaphoreTake(s_daq_engine_ctx.config_update_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return RADAR_DAQ_ERR_BUFFER_LOCK_FAIL;
    }

    bool was_running = (s_daq_engine_ctx.current_fsm_state == RADAR_DAQ_STATE_ACQUIRING);
    if (was_running) gptimer_stop(s_daq_engine_ctx.gptimer_handle);

    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count                = RADAR_GPTIMER_RESOLUTION_HZ / new_prf_hz,
        .reload_count               = 0,
        .flags.auto_reload_on_alarm = true
    };
    gptimer_set_alarm_action(s_daq_engine_ctx.gptimer_handle, &alarm_cfg);
    s_daq_engine_ctx.active_config.prf_frequency_hz = new_prf_hz;

    if (was_running) gptimer_start(s_daq_engine_ctx.gptimer_handle);

    xSemaphoreGive(s_daq_engine_ctx.config_update_mutex);
    return RADAR_DAQ_ERR_OK;
}

radar_daq_error_code_t radar_daq_engine_force_single_capture(radar_daq_ascan_frame_t* out_frame)
{
    if (!s_engine_globally_initialized) return RADAR_DAQ_ERR_NOT_INITIALIZED;
    if (out_frame == NULL) return RADAR_DAQ_ERR_NOT_INITIALIZED;

    s_internal_trigger_pulse_emit();

    spi_transaction_t single_trans = {
        .rxlength  = RADAR_DAQ_ASCAN_RAW_BYTES * 8,
        .rx_buffer = out_frame->raw_dma_buffer,
        .flags     = 0
    };
    esp_err_t ret = spi_device_transmit(s_daq_engine_ctx.spi_device_handle, &single_trans);
    if (ret != ESP_OK) return RADAR_DAQ_ERR_SPI_BUS_FAULT;

    s_internal_fill_frame_metadata(out_frame, 0xFF);

    const uint8_t* raw_u8 = out_frame->raw_dma_buffer;
    for (int i = 0; i < RADAR_DAQ_SAMPLES_PER_ASCAN; i++) {
        uint16_t raw_val = ((uint16_t)raw_u8[i * 2 + 1] << 8) | raw_u8[i * 2];
        out_frame->corrected_sample_array[i] = (int16_t)(raw_val - (RADAR_DAQ_ADC_REFERENCE_COUNTS / 2));
    }
    s_internal_apply_dc_offset_correction(out_frame);
    s_internal_compute_frame_crc32(out_frame);
    s_internal_normalize_raw_to_float(out_frame);
    out_frame->metadata.calibration_frame_flag = true;

    return RADAR_DAQ_ERR_OK;
}

radar_daq_error_code_t radar_daq_engine_flush_frame_queue(void)
{
    if (!s_engine_globally_initialized) return RADAR_DAQ_ERR_NOT_INITIALIZED;

    size_t item_size;
    void* item;
    uint32_t flushed = 0;
    while ((item = xRingbufferReceive(s_daq_engine_ctx.frame_output_ringbuf,
                                       &item_size, 0)) != NULL)
    {
        vRingbufferReturnItem(s_daq_engine_ctx.frame_output_ringbuf, item);
        flushed++;
    }
    s_daq_engine_ctx.telemetry.total_frames_dropped += flushed;
    return RADAR_DAQ_ERR_OK;
}

radar_daq_error_code_t radar_daq_engine_get_telemetry(radar_daq_runtime_telemetry_t* out_telemetry)
{
    if (!s_engine_globally_initialized) return RADAR_DAQ_ERR_NOT_INITIALIZED;
    if (out_telemetry == NULL) return RADAR_DAQ_ERR_NOT_INITIALIZED;
    s_internal_telemetry_update_periodic();
    *out_telemetry = s_daq_engine_ctx.telemetry;
    return RADAR_DAQ_ERR_OK;
}

radar_daq_error_code_t radar_daq_engine_get_fsm_state(radar_daq_fsm_state_t* out_state)
{
    if (!s_engine_globally_initialized) return RADAR_DAQ_ERR_NOT_INITIALIZED;
    if (out_state == NULL) return RADAR_DAQ_ERR_NOT_INITIALIZED;
    *out_state = s_daq_engine_ctx.current_fsm_state;
    return RADAR_DAQ_ERR_OK;
}

radar_daq_error_code_t radar_daq_engine_reset_telemetry_counters(void)
{
    if (!s_engine_globally_initialized) return RADAR_DAQ_ERR_NOT_INITIALIZED;
    uint64_t start = s_daq_engine_ctx.telemetry.session_start_timestamp_us;
    memset(&s_daq_engine_ctx.telemetry, 0, sizeof(radar_daq_runtime_telemetry_t));
    s_daq_engine_ctx.telemetry.session_start_timestamp_us = esp_timer_get_time();
    (void)start;
    s_isr_trigger_accumulator = 0;
    s_isr_prf_deviation_count = 0;
    return RADAR_DAQ_ERR_OK;
}

radar_daq_error_code_t radar_daq_engine_validate_hardware_loopback(bool* out_pass)
{
    if (!s_engine_globally_initialized) return RADAR_DAQ_ERR_NOT_INITIALIZED;
    if (out_pass == NULL) return RADAR_DAQ_ERR_NOT_INITIALIZED;
    return s_internal_hardware_self_test_spi_loopback(out_pass);
}

radar_daq_error_code_t radar_daq_engine_request_hardware_reset(void)
{
    if (!s_engine_globally_initialized) return RADAR_DAQ_ERR_NOT_INITIALIZED;
    s_daq_engine_ctx.hardware_reset_counter++;
    s_internal_fsm_transition(RADAR_DAQ_STATE_SHUTDOWN);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(RADAR_TRIGGER_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    s_internal_fsm_transition(RADAR_DAQ_STATE_IDLE);
    return RADAR_DAQ_ERR_OK;
}

bool radar_daq_engine_is_running(void)
{
    return s_engine_globally_initialized &&
           s_daq_engine_ctx.acquisition_run_flag &&
           (s_daq_engine_ctx.current_fsm_state == RADAR_DAQ_STATE_ACQUIRING ||
            s_daq_engine_ctx.current_fsm_state == RADAR_DAQ_STATE_DMA_IN_FLIGHT ||
            s_daq_engine_ctx.current_fsm_state == RADAR_DAQ_STATE_ARMED_WAITING);
}

uint32_t radar_daq_engine_get_dropped_frame_count(void)
{
    return s_daq_engine_ctx.telemetry.total_frames_dropped;
}

void radar_daq_engine_register_external_error_hook(radar_daq_error_callback_t hook, void* ctx)
{
    s_daq_engine_ctx.callback_registry.on_acquisition_error = hook;
    s_daq_engine_ctx.callback_registry.callback_user_context = ctx;
}

static void s_internal_trigger_pulse_emit(void)
{
    gpio_set_level(RADAR_TRIGGER_GPIO, RADAR_TRIGGER_ACTIVE_LEVEL);
    esp_rom_delay_us(RADAR_TRIGGER_PULSE_WIDTH_US);
    gpio_set_level(RADAR_TRIGGER_GPIO, !RADAR_TRIGGER_ACTIVE_LEVEL);
}

void radar_daq_engine_debug_dump_context(void)
{
    radar_daq_runtime_telemetry_t* t = &s_daq_engine_ctx.telemetry;
    ESP_LOGI(RADAR_DAQ_LOG_TAG, "========= RADAR DAQ DEBUG DUMP =========");
    ESP_LOGI(RADAR_DAQ_LOG_TAG, "FSM State       : %d", s_daq_engine_ctx.current_fsm_state);
    ESP_LOGI(RADAR_DAQ_LOG_TAG, "Frames Captured : %lu", t->total_frames_captured);
    ESP_LOGI(RADAR_DAQ_LOG_TAG, "Frames Dropped  : %lu", t->total_frames_dropped);
    ESP_LOGI(RADAR_DAQ_LOG_TAG, "DMA Errors      : %lu", t->total_dma_errors);
    ESP_LOGI(RADAR_DAQ_LOG_TAG, "Trigger Misses  : %lu", t->total_trigger_misses);
    ESP_LOGI(RADAR_DAQ_LOG_TAG, "Actual PRF Hz   : %.2f", t->actual_prf_hz_measured);
    ESP_LOGI(RADAR_DAQ_LOG_TAG, "Avg DMA Time us : %.2f", t->average_dma_transfer_time_us);
    ESP_LOGI(RADAR_DAQ_LOG_TAG, "Peak DMA Time us: %.2f", t->peak_dma_transfer_time_us);
    ESP_LOGI(RADAR_DAQ_LOG_TAG, "SPI Utilization : %.1f%%", t->spi_bus_utilization_percent);
    ESP_LOGI(RADAR_DAQ_LOG_TAG, "Queue Max Fill  : %lu", t->maximum_queue_fill_level);
    ESP_LOGI(RADAR_DAQ_LOG_TAG, "ISR PRF Dev Cnt : %lu", s_isr_prf_deviation_count);
    ESP_LOGI(RADAR_DAQ_LOG_TAG, "ISR Total Trig  : %lu", s_isr_trigger_accumulator);
    ESP_LOGI(RADAR_DAQ_LOG_TAG, "HW Reset Count  : %lu", s_daq_engine_ctx.hardware_reset_counter);
    ESP_LOGI(RADAR_DAQ_LOG_TAG, "========================================");
}
