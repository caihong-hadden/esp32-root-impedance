#ifndef RADAR_DAQ_HIGH_SPEED_ACQUISITION_ENGINE_H
#define RADAR_DAQ_HIGH_SPEED_ACQUISITION_ENGINE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "driver/gptimer.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define RADAR_DAQ_MODULE_VERSION_MAJOR         2
#define RADAR_DAQ_MODULE_VERSION_MINOR         4
#define RADAR_DAQ_MODULE_VERSION_PATCH         11
#define RADAR_DAQ_MAGIC_INIT_TOKEN             0xDA410000
#define RADAR_DAQ_MAGIC_FRAME_HEADER           0xF4A3C2B1
#define RADAR_DAQ_MAGIC_FRAME_FOOTER           0xE5D4C3B2

#define RADAR_SPI_HOST_BUS                     SPI2_HOST
#define RADAR_SPI_MISO_GPIO                    13
#define RADAR_SPI_CLK_GPIO                     12
#define RADAR_SPI_CS_GPIO                      10
#define RADAR_SPI_MOSI_GPIO                    (-1)
#define RADAR_SPI_BUS_CLOCK_HZ                 (80 * 1000 * 1000)
#define RADAR_SPI_DMA_CHANNEL                  SPI_DMA_CH_AUTO
#define RADAR_SPI_QUEUE_DEPTH                  8
#define RADAR_SPI_MAX_TRANSFER_BYTES           4096

#define RADAR_TRIGGER_GPIO                     9
#define RADAR_TRIGGER_PULSE_WIDTH_US           2
#define RADAR_TRIGGER_ACTIVE_LEVEL             1

#define RADAR_GPTIMER_RESOLUTION_HZ            (1 * 1000 * 1000)
#define RADAR_GPTIMER_PRF_DEFAULT_HZ           1000
#define RADAR_GPTIMER_PRF_MIN_HZ               100
#define RADAR_GPTIMER_PRF_MAX_HZ               5000
#define RADAR_GPTIMER_ALARM_AUTO_RELOAD        true

#define RADAR_DAQ_SAMPLES_PER_ASCAN            512
#define RADAR_DAQ_BYTES_PER_SAMPLE             2
#define RADAR_DAQ_ASCAN_RAW_BYTES              (RADAR_DAQ_SAMPLES_PER_ASCAN * RADAR_DAQ_BYTES_PER_SAMPLE)
#define RADAR_DAQ_PINGPONG_BUFFER_COUNT        2
#define RADAR_DAQ_DMA_DESCRIPTOR_CHAIN_LEN     4
#define RADAR_DAQ_FRAME_QUEUE_DEPTH            32
#define RADAR_DAQ_OVERFLOW_WATERMARK_PERCENT   85

#define RADAR_DAQ_TASK_STACK_SIZE              (8 * 1024)
#define RADAR_DAQ_TASK_PRIORITY                24
#define RADAR_DAQ_TASK_CORE_AFFINITY           1
#define RADAR_DAQ_WATCHDOG_TIMEOUT_MS          500

#define RADAR_DAQ_ADC_FULL_SCALE_MV            3300
#define RADAR_DAQ_ADC_RESOLUTION_BITS          12
#define RADAR_DAQ_ADC_REFERENCE_COUNTS         4096
#define RADAR_DAQ_INTER_SAMPLE_DELAY_NS        25

#define RADAR_DAQ_SYNC_WORD_PATTERN            0xA5C3
#define RADAR_DAQ_TIMEOUT_DMA_COMPLETE_MS      50
#define RADAR_DAQ_TIMEOUT_TRIGGER_ACK_US       200
#define RADAR_DAQ_BURST_MODE_MAX_FRAMES        16

typedef enum {
    RADAR_DAQ_ERR_OK                  = 0x0000,
    RADAR_DAQ_ERR_NOT_INITIALIZED     = 0x0001,
    RADAR_DAQ_ERR_ALREADY_RUNNING     = 0x0002,
    RADAR_DAQ_ERR_SPI_BUS_FAULT       = 0x0003,
    RADAR_DAQ_ERR_DMA_OVERFLOW        = 0x0004,
    RADAR_DAQ_ERR_DMA_TIMEOUT         = 0x0005,
    RADAR_DAQ_ERR_TIMER_CONFIG_FAIL   = 0x0006,
    RADAR_DAQ_ERR_TRIGGER_GPIO_FAULT  = 0x0007,
    RADAR_DAQ_ERR_FRAME_QUEUE_FULL    = 0x0008,
    RADAR_DAQ_ERR_CHECKSUM_MISMATCH   = 0x0009,
    RADAR_DAQ_ERR_SYNC_LOST           = 0x000A,
    RADAR_DAQ_ERR_PRF_OUT_OF_RANGE    = 0x000B,
    RADAR_DAQ_ERR_BUFFER_LOCK_FAIL    = 0x000C,
    RADAR_DAQ_ERR_HARDWARE_RESET_REQ  = 0x000D,
    RADAR_DAQ_ERR_CALIBRATION_INVALID = 0x000E,
    RADAR_DAQ_ERR_INTERNAL_FATAL      = 0x00FF
} radar_daq_error_code_t;

typedef enum {
    RADAR_DAQ_STATE_UNINITIALIZED    = 0x00,
    RADAR_DAQ_STATE_IDLE             = 0x01,
    RADAR_DAQ_STATE_ARMING           = 0x02,
    RADAR_DAQ_STATE_ARMED_WAITING    = 0x03,
    RADAR_DAQ_STATE_ACQUIRING        = 0x04,
    RADAR_DAQ_STATE_DMA_IN_FLIGHT    = 0x05,
    RADAR_DAQ_STATE_FRAME_READY      = 0x06,
    RADAR_DAQ_STATE_FLUSHING         = 0x07,
    RADAR_DAQ_STATE_PAUSED           = 0x08,
    RADAR_DAQ_STATE_ERROR_RECOVERY   = 0x09,
    RADAR_DAQ_STATE_HARDWARE_FAULT   = 0x0A,
    RADAR_DAQ_STATE_SHUTDOWN         = 0x0B
} radar_daq_fsm_state_t;

typedef enum {
    RADAR_DAQ_TRIGGER_MODE_FREE_RUN     = 0,
    RADAR_DAQ_TRIGGER_MODE_GPTIMER_PRF = 1,
    RADAR_DAQ_TRIGGER_MODE_EXTERNAL_GPIO = 2,
    RADAR_DAQ_TRIGGER_MODE_SOFTWARE_CMD = 3,
    RADAR_DAQ_TRIGGER_MODE_BURST_SEQUENCE = 4
} radar_daq_trigger_mode_t;

typedef enum {
    RADAR_DAQ_SPI_MODE_STANDARD_FULL_DUPLEX = 0,
    RADAR_DAQ_SPI_MODE_HALF_DUPLEX_RX_ONLY  = 1,
    RADAR_DAQ_SPI_MODE_DMA_CHAIN_LINKED      = 2,
    RADAR_DAQ_SPI_MODE_PARALLEL_8BIT_LCD_CAM = 3
} radar_daq_spi_transfer_mode_t;

typedef enum {
    RADAR_DAQ_SAMPLE_FORMAT_INT16_RAW    = 0,
    RADAR_DAQ_SAMPLE_FORMAT_UINT16_RAW   = 1,
    RADAR_DAQ_SAMPLE_FORMAT_FLOAT32_NORM = 2,
    RADAR_DAQ_SAMPLE_FORMAT_FIXED_Q15    = 3
} radar_daq_sample_data_format_t;

typedef enum {
    RADAR_DAQ_DMA_STATUS_IDLE         = 0,
    RADAR_DAQ_DMA_STATUS_DESCRIPTOR_LOADING = 1,
    RADAR_DAQ_DMA_STATUS_TRANSFER_ACTIVE    = 2,
    RADAR_DAQ_DMA_STATUS_EOF_PENDING        = 3,
    RADAR_DAQ_DMA_STATUS_COMPLETE           = 4,
    RADAR_DAQ_DMA_STATUS_ERROR_OVERRUN      = 5,
    RADAR_DAQ_DMA_STATUS_ERROR_UNDERRUN     = 6,
    RADAR_DAQ_DMA_STATUS_STALLED            = 7
} radar_daq_dma_transfer_status_t;

typedef struct {
    uint32_t frame_magic_header;
    uint64_t capture_timestamp_us;
    uint32_t frame_sequence_number;
    uint16_t samples_per_ascan;
    uint16_t bytes_per_sample;
    uint32_t total_payload_bytes;
    uint32_t crc32_payload_checksum;
    uint16_t sync_word;
    uint8_t  trigger_source_id;
    uint8_t  buffer_slot_index;
    float    trigger_interval_actual_us;
    float    spi_clock_actual_mhz;
    bool     dma_completed_without_error;
    bool     overflow_detected_this_frame;
    bool     calibration_frame_flag;
    uint8_t  reserved_flags[5];
    uint32_t frame_magic_footer;
} radar_daq_frame_metadata_t;

typedef struct {
    uint8_t  raw_dma_buffer[RADAR_DAQ_ASCAN_RAW_BYTES];
    int16_t  corrected_sample_array[RADAR_DAQ_SAMPLES_PER_ASCAN];
    float    normalized_float_array[RADAR_DAQ_SAMPLES_PER_ASCAN];
    radar_daq_frame_metadata_t metadata;
    bool     is_actively_locked_by_dsp;
    volatile uint32_t access_reference_count;
} radar_daq_ascan_frame_t;

typedef struct {
    radar_daq_ascan_frame_t slot[RADAR_DAQ_PINGPONG_BUFFER_COUNT];
    volatile uint8_t active_write_slot_index;
    volatile uint8_t active_read_slot_index;
    volatile bool    flip_pending;
    SemaphoreHandle_t swap_synchronization_mutex;
    uint32_t total_successful_swaps;
    uint32_t total_swap_collisions;
    uint64_t last_swap_timestamp_us;
} radar_daq_pingpong_buffer_manager_t;

typedef struct {
    uint32_t total_frames_captured;
    uint32_t total_frames_dropped;
    uint32_t total_dma_errors;
    uint32_t total_trigger_misses;
    uint32_t total_sync_losses;
    uint32_t total_overflow_events;
    uint64_t session_start_timestamp_us;
    uint64_t last_frame_timestamp_us;
    float    actual_prf_hz_measured;
    float    average_dma_transfer_time_us;
    float    peak_dma_transfer_time_us;
    float    spi_bus_utilization_percent;
    uint32_t consecutive_error_streak;
    uint32_t maximum_queue_fill_level;
} radar_daq_runtime_telemetry_t;

typedef struct {
    uint32_t prf_frequency_hz;
    radar_daq_trigger_mode_t trigger_mode;
    radar_daq_spi_transfer_mode_t spi_transfer_mode;
    radar_daq_sample_data_format_t output_sample_format;
    uint16_t samples_per_ascan_override;
    bool enable_hardware_trigger_echo_gpio;
    bool enable_overflow_protection_circuit;
    bool enable_frame_crc32_verification;
    bool enable_auto_gain_normalization;
    bool enable_burst_acquisition_mode;
    uint8_t burst_mode_frame_count;
    uint32_t spi_pre_trigger_delay_ns;
    uint32_t spi_post_capture_hold_ns;
    uint32_t inter_frame_blanking_us;
    float    dc_offset_correction_mv;
} radar_daq_configuration_params_t;

typedef void (*radar_daq_frame_ready_callback_t)(const radar_daq_ascan_frame_t* frame, void* user_context_ptr);
typedef void (*radar_daq_error_callback_t)(radar_daq_error_code_t err_code, radar_daq_fsm_state_t state_at_fault, void* user_context_ptr);
typedef void (*radar_daq_overflow_callback_t)(uint32_t dropped_frame_count, void* user_context_ptr);

typedef struct {
    radar_daq_frame_ready_callback_t on_frame_ready;
    radar_daq_error_callback_t       on_acquisition_error;
    radar_daq_overflow_callback_t    on_buffer_overflow;
    void*                            callback_user_context;
} radar_daq_callback_registry_t;

typedef struct {
    uint32_t                          init_magic_token;
    radar_daq_fsm_state_t             current_fsm_state;
    radar_daq_fsm_state_t             previous_fsm_state;
    radar_daq_configuration_params_t  active_config;
    radar_daq_callback_registry_t     callback_registry;
    radar_daq_pingpong_buffer_manager_t pingpong_manager;
    radar_daq_runtime_telemetry_t     telemetry;
    gptimer_handle_t                  gptimer_handle;
    spi_device_handle_t               spi_device_handle;
    RingbufHandle_t                   frame_output_ringbuf;
    TaskHandle_t                      daq_task_handle;
    SemaphoreHandle_t                 dma_complete_semaphore;
    SemaphoreHandle_t                 config_update_mutex;
    volatile bool                     acquisition_run_flag;
    volatile uint32_t                 pending_trigger_count;
    uint32_t                          hardware_reset_counter;
} radar_daq_engine_context_t;

radar_daq_error_code_t radar_daq_engine_initialize(const radar_daq_configuration_params_t* config, const radar_daq_callback_registry_t* callbacks);
radar_daq_error_code_t radar_daq_engine_start_acquisition(void);
radar_daq_error_code_t radar_daq_engine_stop_acquisition(void);
radar_daq_error_code_t radar_daq_engine_pause_acquisition(void);
radar_daq_error_code_t radar_daq_engine_resume_acquisition(void);
radar_daq_error_code_t radar_daq_engine_teardown(void);
radar_daq_error_code_t radar_daq_engine_reconfigure_prf(uint32_t new_prf_hz);
radar_daq_error_code_t radar_daq_engine_force_single_capture(radar_daq_ascan_frame_t* out_frame);
radar_daq_error_code_t radar_daq_engine_flush_frame_queue(void);
radar_daq_error_code_t radar_daq_engine_get_telemetry(radar_daq_runtime_telemetry_t* out_telemetry);
radar_daq_error_code_t radar_daq_engine_get_fsm_state(radar_daq_fsm_state_t* out_state);
radar_daq_error_code_t radar_daq_engine_reset_telemetry_counters(void);
radar_daq_error_code_t radar_daq_engine_validate_hardware_loopback(bool* out_pass);
radar_daq_error_code_t radar_daq_engine_request_hardware_reset(void);
bool                   radar_daq_engine_is_running(void);
uint32_t               radar_daq_engine_get_dropped_frame_count(void);
void                   radar_daq_engine_register_external_error_hook(radar_daq_error_callback_t hook, void* ctx);
void                   radar_daq_engine_debug_dump_context(void);

#endif
