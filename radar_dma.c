#include "radar_daq.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "string.h"
#include "stdlib.h"

static const char* RADAR_DMA_LOG_TAG = "RADAR_DMA";

#define RADAR_DMA_DESCRIPTOR_NODE_MAGIC     0xD3A1B2C0
#define RADAR_DMA_MAX_DESCRIPTOR_COUNT      16
#define RADAR_DMA_TRANSFER_TIMEOUT_MS       100
#define RADAR_DMA_BURST_ALIGN_BYTES         4
#define RADAR_DMA_INTERNAL_SRAM_CAP         (MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL)
#define RADAR_DMA_PIPELINE_DEPTH            4
#define RADAR_DMA_WATCHDOG_KICK_INTERVAL_MS 200
#define RADAR_DMA_ERROR_INJECT_RATE_DENOM   0
#define RADAR_DMA_STATS_WINDOW_FRAMES       50
#define RADAR_DMA_MAX_RETRY_ON_ERROR        3
#define RADAR_DMA_IDLE_POLL_DELAY_MS        1

typedef enum {
    RADAR_DMA_CHAN_STATE_UNALLOCATED  = 0x00,
    RADAR_DMA_CHAN_STATE_IDLE         = 0x10,
    RADAR_DMA_CHAN_STATE_DESCRIPTORS_LOADED = 0x20,
    RADAR_DMA_CHAN_STATE_TRANSFER_ARMED     = 0x30,
    RADAR_DMA_CHAN_STATE_IN_PROGRESS        = 0x40,
    RADAR_DMA_CHAN_STATE_EOF_PENDING        = 0x50,
    RADAR_DMA_CHAN_STATE_COMPLETE_ACK_REQ   = 0x60,
    RADAR_DMA_CHAN_STATE_STALL_DETECTED     = 0x70,
    RADAR_DMA_CHAN_STATE_ERROR_BUS_FAULT    = 0x80,
    RADAR_DMA_CHAN_STATE_RECOVERING         = 0x90,
    RADAR_DMA_CHAN_STATE_DECOMMISSIONED     = 0xFF
} radar_dma_channel_state_t;

typedef enum {
    RADAR_DMA_BURST_SIZE_1_BYTE   = 1,
    RADAR_DMA_BURST_SIZE_4_BYTES  = 4,
    RADAR_DMA_BURST_SIZE_8_BYTES  = 8,
    RADAR_DMA_BURST_SIZE_16_BYTES = 16,
    RADAR_DMA_BURST_SIZE_32_BYTES = 32
} radar_dma_burst_size_t;

typedef enum {
    RADAR_DMA_PRIORITY_LOWEST  = 0,
    RADAR_DMA_PRIORITY_LOW     = 1,
    RADAR_DMA_PRIORITY_MEDIUM  = 2,
    RADAR_DMA_PRIORITY_HIGH    = 3,
    RADAR_DMA_PRIORITY_REALTIME = 4
} radar_dma_transfer_priority_t;

typedef enum {
    RADAR_DMA_ENDIAN_LITTLE   = 0,
    RADAR_DMA_ENDIAN_BIG      = 1,
    RADAR_DMA_ENDIAN_WORD_SWAP = 2
} radar_dma_byte_order_t;

typedef struct radar_dma_descriptor_node_s {
    uint32_t node_magic;
    uint32_t node_index;
    uint8_t* buffer_pointer;
    size_t   buffer_length_bytes;
    size_t   transfer_length_bytes;
    bool     is_eof_node;
    bool     is_head_node;
    bool     is_dma_locked;
    uint32_t transfer_attempt_count;
    uint32_t error_flag_register;
    struct radar_dma_descriptor_node_s* next_descriptor;
    struct radar_dma_descriptor_node_s* prev_descriptor;
    uint32_t node_crc32_integrity;
} radar_dma_descriptor_node_t;

typedef struct {
    uint32_t total_bytes_transferred;
    uint32_t total_descriptors_processed;
    uint32_t total_transfer_errors;
    uint32_t total_stall_events;
    uint32_t total_eof_interrupts;
    uint32_t total_retry_successes;
    uint32_t total_retry_failures;
    uint64_t last_eof_timestamp_us;
    float    average_throughput_mbps;
    float    peak_throughput_mbps;
    float    current_window_throughput_mbps;
    uint32_t bus_error_code_register;
    uint32_t consecutive_stall_count;
    uint32_t max_observed_transfer_time_us;
} radar_dma_channel_statistics_t;

typedef struct {
    radar_dma_burst_size_t        burst_size;
    radar_dma_transfer_priority_t priority;
    radar_dma_byte_order_t        byte_order;
    bool enable_circular_mode;
    bool enable_auto_flush_on_eof;
    bool enable_bus_error_interrupt;
    bool enable_descriptor_integrity_check;
    bool enable_throughput_monitoring;
    uint32_t stall_detection_timeout_ms;
    uint32_t max_transfer_burst_length_bytes;
} radar_dma_channel_config_t;

typedef struct {
    uint32_t channel_id;
    radar_dma_channel_state_t current_state;
    radar_dma_channel_state_t previous_state;
    radar_dma_channel_config_t config;
    radar_dma_channel_statistics_t stats;
    radar_dma_descriptor_node_t* descriptor_chain_head;
    radar_dma_descriptor_node_t* descriptor_chain_tail;
    radar_dma_descriptor_node_t* current_active_descriptor;
    uint32_t descriptor_count;
    SemaphoreHandle_t eof_notification_semaphore;
    SemaphoreHandle_t channel_access_mutex;
    uint8_t* dma_aligned_working_buffer;
    size_t   dma_working_buffer_size;
    uint64_t channel_creation_timestamp_us;
    bool     channel_is_active;
    uint32_t watchdog_kick_counter;
} radar_dma_channel_context_t;

typedef struct {
    uint32_t pipeline_id;
    radar_dma_channel_context_t stages[RADAR_DMA_PIPELINE_DEPTH];
    uint8_t active_stage_count;
    bool pipeline_running;
    SemaphoreHandle_t pipeline_sync_semaphore;
    uint64_t pipeline_start_time_us;
    uint32_t total_pipeline_completions;
    uint32_t total_pipeline_aborts;
    float    overall_pipeline_efficiency_percent;
} radar_dma_pipeline_context_t;

static radar_dma_channel_context_t  s_primary_rx_channel;
static radar_dma_pipeline_context_t s_acquisition_pipeline;
static volatile bool                s_dma_subsystem_initialized = false;
static volatile uint32_t            s_global_eof_interrupt_count = 0;
static volatile uint32_t            s_global_bus_error_count = 0;

static radar_daq_error_code_t s_dma_allocate_aligned_working_buffer(radar_dma_channel_context_t* ch,
                                                                      size_t required_bytes);
static radar_daq_error_code_t s_dma_build_descriptor_chain(radar_dma_channel_context_t* ch,
                                                             uint8_t* target_buffer,
                                                             size_t transfer_size);
static radar_daq_error_code_t s_dma_free_descriptor_chain(radar_dma_channel_context_t* ch);
static radar_daq_error_code_t s_dma_arm_transfer(radar_dma_channel_context_t* ch);
static radar_daq_error_code_t s_dma_wait_for_eof(radar_dma_channel_context_t* ch,
                                                   uint32_t timeout_ms);
static radar_daq_error_code_t s_dma_handle_stall_recovery(radar_dma_channel_context_t* ch);
static void                   s_dma_channel_state_transition(radar_dma_channel_context_t* ch,
                                                              radar_dma_channel_state_t next);
static void                   s_dma_update_throughput_stats(radar_dma_channel_context_t* ch,
                                                             size_t bytes_transferred,
                                                             uint64_t elapsed_us);
static bool                   s_dma_validate_descriptor_chain_integrity(const radar_dma_channel_context_t* ch);
static uint32_t               s_dma_compute_node_crc(const radar_dma_descriptor_node_t* node);
static void                   s_dma_kick_watchdog_counter(radar_dma_channel_context_t* ch);

static void s_dma_channel_state_transition(radar_dma_channel_context_t* ch,
                                            radar_dma_channel_state_t next)
{
    ch->previous_state = ch->current_state;
    ch->current_state  = next;
}

static void s_dma_kick_watchdog_counter(radar_dma_channel_context_t* ch)
{
    ch->watchdog_kick_counter++;
}

static uint32_t s_dma_compute_node_crc(const radar_dma_descriptor_node_t* node)
{
    uint32_t crc = node->node_magic ^ node->node_index;
    crc ^= (uint32_t)(uintptr_t)node->buffer_pointer;
    crc ^= (uint32_t)node->buffer_length_bytes;
    crc ^= (uint32_t)node->transfer_length_bytes;
    crc ^= node->is_eof_node  ? 0xAAAAAAAA : 0x55555555;
    crc ^= node->is_head_node ? 0xBBBBBBBB : 0x44444444;
    crc  = (crc >> 16) ^ (crc << 16);
    crc ^= 0xDEADBEEF;
    return crc;
}

static bool s_dma_validate_descriptor_chain_integrity(const radar_dma_channel_context_t* ch)
{
    if (ch->descriptor_chain_head == NULL) return false;

    radar_dma_descriptor_node_t* node = ch->descriptor_chain_head;
    uint32_t visited = 0;

    while (node != NULL && visited < RADAR_DMA_MAX_DESCRIPTOR_COUNT)
    {
        if (node->node_magic != RADAR_DMA_DESCRIPTOR_NODE_MAGIC) return false;

        uint32_t expected_crc = s_dma_compute_node_crc(node);
        if (node->node_crc32_integrity != expected_crc) return false;

        if (node->buffer_pointer == NULL) return false;
        if (node->buffer_length_bytes == 0) return false;
        if (node->transfer_length_bytes > node->buffer_length_bytes) return false;

        node = node->next_descriptor;
        visited++;
    }

    return (visited == ch->descriptor_count);
}

static radar_daq_error_code_t s_dma_allocate_aligned_working_buffer(radar_dma_channel_context_t* ch,
                                                                      size_t required_bytes)
{
    size_t aligned_size = (required_bytes + RADAR_DMA_BURST_ALIGN_BYTES - 1)
                          & ~(RADAR_DMA_BURST_ALIGN_BYTES - 1);

    ch->dma_aligned_working_buffer = heap_caps_aligned_alloc(
        RADAR_DMA_BURST_ALIGN_BYTES,
        aligned_size,
        RADAR_DMA_INTERNAL_SRAM_CAP
    );
    if (ch->dma_aligned_working_buffer == NULL) {
        return RADAR_DAQ_ERR_DMA_OVERFLOW;
    }

    memset(ch->dma_aligned_working_buffer, 0, aligned_size);
    ch->dma_working_buffer_size = aligned_size;
    return RADAR_DAQ_ERR_OK;
}

static radar_daq_error_code_t s_dma_build_descriptor_chain(radar_dma_channel_context_t* ch,
                                                             uint8_t* target_buffer,
                                                             size_t transfer_size)
{
    if (target_buffer == NULL || transfer_size == 0) {
        return RADAR_DAQ_ERR_DMA_OVERFLOW;
    }

    uint32_t bytes_remaining = (uint32_t)transfer_size;
    uint32_t max_node_payload = ch->config.max_transfer_burst_length_bytes;
    if (max_node_payload == 0) max_node_payload = 512;

    uint32_t nodes_needed = (bytes_remaining + max_node_payload - 1) / max_node_payload;
    if (nodes_needed > RADAR_DMA_MAX_DESCRIPTOR_COUNT) {
        return RADAR_DAQ_ERR_DMA_OVERFLOW;
    }

    radar_dma_descriptor_node_t* prev_node = NULL;
    ch->descriptor_chain_head = NULL;
    ch->descriptor_chain_tail = NULL;
    ch->descriptor_count      = 0;

    for (uint32_t idx = 0; idx < nodes_needed; idx++)
    {
        radar_dma_descriptor_node_t* node = (radar_dma_descriptor_node_t*)
            heap_caps_malloc(sizeof(radar_dma_descriptor_node_t), MALLOC_CAP_INTERNAL);
        if (node == NULL) {
            s_dma_free_descriptor_chain(ch);
            return RADAR_DAQ_ERR_DMA_OVERFLOW;
        }

        uint32_t this_node_bytes = (bytes_remaining > max_node_payload)
                                   ? max_node_payload : bytes_remaining;

        node->node_magic              = RADAR_DMA_DESCRIPTOR_NODE_MAGIC;
        node->node_index              = idx;
        node->buffer_pointer          = target_buffer + (idx * max_node_payload);
        node->buffer_length_bytes     = this_node_bytes;
        node->transfer_length_bytes   = this_node_bytes;
        node->is_eof_node             = (idx == nodes_needed - 1);
        node->is_head_node            = (idx == 0);
        node->is_dma_locked           = false;
        node->transfer_attempt_count  = 0;
        node->error_flag_register     = 0;
        node->next_descriptor         = NULL;
        node->prev_descriptor         = prev_node;
        node->node_crc32_integrity    = s_dma_compute_node_crc(node);

        if (prev_node != NULL) {
            prev_node->next_descriptor = node;
        } else {
            ch->descriptor_chain_head = node;
        }
        ch->descriptor_chain_tail = node;

        prev_node = node;
        bytes_remaining -= this_node_bytes;
        ch->descriptor_count++;
    }

    if (ch->config.enable_circular_mode && ch->descriptor_chain_tail != NULL) {
        ch->descriptor_chain_tail->next_descriptor = ch->descriptor_chain_head;
    }

    if (ch->config.enable_descriptor_integrity_check) {
        if (!s_dma_validate_descriptor_chain_integrity(ch)) {
            s_dma_free_descriptor_chain(ch);
            return RADAR_DAQ_ERR_DMA_OVERFLOW;
        }
    }

    ch->current_active_descriptor = ch->descriptor_chain_head;
    return RADAR_DAQ_ERR_OK;
}

static radar_daq_error_code_t s_dma_free_descriptor_chain(radar_dma_channel_context_t* ch)
{
    radar_dma_descriptor_node_t* node = ch->descriptor_chain_head;
    uint32_t freed_count = 0;

    while (node != NULL && freed_count < RADAR_DMA_MAX_DESCRIPTOR_COUNT)
    {
        radar_dma_descriptor_node_t* next = node->next_descriptor;
        if (next == ch->descriptor_chain_head && ch->config.enable_circular_mode) {
            free(node);
            freed_count++;
            break;
        }
        free(node);
        freed_count++;
        node = next;
    }

    ch->descriptor_chain_head = NULL;
    ch->descriptor_chain_tail = NULL;
    ch->current_active_descriptor = NULL;
    ch->descriptor_count = 0;

    return RADAR_DAQ_ERR_OK;
}

static radar_daq_error_code_t s_dma_arm_transfer(radar_dma_channel_context_t* ch)
{
    if (ch->current_state != RADAR_DMA_CHAN_STATE_DESCRIPTORS_LOADED) {
        return RADAR_DAQ_ERR_DMA_TIMEOUT;
    }

    if (ch->config.enable_descriptor_integrity_check) {
        if (!s_dma_validate_descriptor_chain_integrity(ch)) {
            ch->stats.bus_error_code_register |= 0x01;
            s_dma_channel_state_transition(ch, RADAR_DMA_CHAN_STATE_ERROR_BUS_FAULT);
            return RADAR_DAQ_ERR_DMA_OVERFLOW;
        }
    }

    radar_dma_descriptor_node_t* node = ch->descriptor_chain_head;
    while (node != NULL) {
        node->is_dma_locked = true;
        node->transfer_attempt_count++;
        if (!ch->config.enable_circular_mode && node->is_eof_node) break;
        node = node->next_descriptor;
    }

    s_dma_channel_state_transition(ch, RADAR_DMA_CHAN_STATE_TRANSFER_ARMED);
    vTaskDelay(pdMS_TO_TICKS(RADAR_DMA_IDLE_POLL_DELAY_MS));
    s_dma_channel_state_transition(ch, RADAR_DMA_CHAN_STATE_IN_PROGRESS);
    s_dma_kick_watchdog_counter(ch);

    return RADAR_DAQ_ERR_OK;
}

static void s_dma_update_throughput_stats(radar_dma_channel_context_t* ch,
                                           size_t bytes_transferred,
                                           uint64_t elapsed_us)
{
    if (elapsed_us == 0) return;

    float throughput_mbps = ((float)bytes_transferred * 8.0f) / (float)elapsed_us;

    if (throughput_mbps > ch->stats.peak_throughput_mbps) {
        ch->stats.peak_throughput_mbps = throughput_mbps;
    }

    float alpha = 1.0f / (float)RADAR_DMA_STATS_WINDOW_FRAMES;
    ch->stats.current_window_throughput_mbps =
        alpha * throughput_mbps + (1.0f - alpha) * ch->stats.current_window_throughput_mbps;

    ch->stats.average_throughput_mbps =
        (ch->stats.average_throughput_mbps * (float)(ch->stats.total_descriptors_processed)
         + throughput_mbps)
        / (float)(ch->stats.total_descriptors_processed + 1);

    ch->stats.total_bytes_transferred += (uint32_t)bytes_transferred;

    uint32_t elapsed_us_u32 = (elapsed_us > 0xFFFFFFFF) ? 0xFFFFFFFF : (uint32_t)elapsed_us;
    if (elapsed_us_u32 > ch->stats.max_observed_transfer_time_us) {
        ch->stats.max_observed_transfer_time_us = elapsed_us_u32;
    }
}

static radar_daq_error_code_t s_dma_wait_for_eof(radar_dma_channel_context_t* ch,
                                                   uint32_t timeout_ms)
{
    BaseType_t result = xSemaphoreTake(ch->eof_notification_semaphore,
                                        pdMS_TO_TICKS(timeout_ms));
    if (result != pdTRUE) {
        ch->stats.total_stall_events++;
        ch->stats.consecutive_stall_count++;
        return RADAR_DAQ_ERR_DMA_TIMEOUT;
    }

    ch->stats.consecutive_stall_count = 0;
    ch->stats.total_eof_interrupts++;
    ch->stats.last_eof_timestamp_us = esp_timer_get_time();
    s_global_eof_interrupt_count++;

    return RADAR_DAQ_ERR_OK;
}

static radar_daq_error_code_t s_dma_handle_stall_recovery(radar_dma_channel_context_t* ch)
{
    s_dma_channel_state_transition(ch, RADAR_DMA_CHAN_STATE_RECOVERING);
    ch->stats.total_stall_events++;

    for (int retry = 0; retry < RADAR_DMA_MAX_RETRY_ON_ERROR; retry++) {
        vTaskDelay(pdMS_TO_TICKS(5 * (retry + 1)));
        s_dma_kick_watchdog_counter(ch);

        radar_dma_descriptor_node_t* node = ch->descriptor_chain_head;
        while (node != NULL) {
            node->error_flag_register = 0;
            node->is_dma_locked = false;
            if (!ch->config.enable_circular_mode && node->is_eof_node) break;
            node = node->next_descriptor;
        }
        ch->current_active_descriptor = ch->descriptor_chain_head;

        radar_daq_error_code_t arm_result = s_dma_arm_transfer(ch);
        if (arm_result == RADAR_DAQ_ERR_OK) {
            radar_daq_error_code_t wait_result = s_dma_wait_for_eof(ch,
                RADAR_DMA_TRANSFER_TIMEOUT_MS);
            if (wait_result == RADAR_DAQ_ERR_OK) {
                ch->stats.total_retry_successes++;
                s_dma_channel_state_transition(ch, RADAR_DMA_CHAN_STATE_COMPLETE_ACK_REQ);
                return RADAR_DAQ_ERR_OK;
            }
        }
    }

    ch->stats.total_retry_failures++;
    s_dma_channel_state_transition(ch, RADAR_DMA_CHAN_STATE_ERROR_BUS_FAULT);
    s_global_bus_error_count++;
    return RADAR_DAQ_ERR_DMA_TIMEOUT;
}

radar_daq_error_code_t radar_dma_subsystem_initialize(void)
{
    if (s_dma_subsystem_initialized) return RADAR_DAQ_ERR_ALREADY_RUNNING;

    memset(&s_primary_rx_channel, 0, sizeof(radar_dma_channel_context_t));
    memset(&s_acquisition_pipeline, 0, sizeof(radar_dma_pipeline_context_t));

    s_primary_rx_channel.channel_id = 0;
    s_primary_rx_channel.config.burst_size = RADAR_DMA_BURST_SIZE_4_BYTES;
    s_primary_rx_channel.config.priority   = RADAR_DMA_PRIORITY_REALTIME;
    s_primary_rx_channel.config.byte_order = RADAR_DMA_ENDIAN_LITTLE;
    s_primary_rx_channel.config.enable_circular_mode             = false;
    s_primary_rx_channel.config.enable_auto_flush_on_eof         = true;
    s_primary_rx_channel.config.enable_bus_error_interrupt       = true;
    s_primary_rx_channel.config.enable_descriptor_integrity_check = true;
    s_primary_rx_channel.config.enable_throughput_monitoring     = true;
    s_primary_rx_channel.config.stall_detection_timeout_ms       = RADAR_DMA_TRANSFER_TIMEOUT_MS;
    s_primary_rx_channel.config.max_transfer_burst_length_bytes  = 512;

    s_primary_rx_channel.eof_notification_semaphore = xSemaphoreCreateBinary();
    if (s_primary_rx_channel.eof_notification_semaphore == NULL) {
        return RADAR_DAQ_ERR_BUFFER_LOCK_FAIL;
    }

    s_primary_rx_channel.channel_access_mutex = xSemaphoreCreateMutex();
    if (s_primary_rx_channel.channel_access_mutex == NULL) {
        vSemaphoreDelete(s_primary_rx_channel.eof_notification_semaphore);
        return RADAR_DAQ_ERR_BUFFER_LOCK_FAIL;
    }

    radar_daq_error_code_t err = s_dma_allocate_aligned_working_buffer(
        &s_primary_rx_channel,
        RADAR_DAQ_ASCAN_RAW_BYTES + RADAR_DMA_BURST_ALIGN_BYTES
    );
    if (err != RADAR_DAQ_ERR_OK) {
        vSemaphoreDelete(s_primary_rx_channel.eof_notification_semaphore);
        vSemaphoreDelete(s_primary_rx_channel.channel_access_mutex);
        return err;
    }

    s_primary_rx_channel.channel_creation_timestamp_us = esp_timer_get_time();
    s_primary_rx_channel.channel_is_active = true;
    s_dma_channel_state_transition(&s_primary_rx_channel, RADAR_DMA_CHAN_STATE_IDLE);

    s_acquisition_pipeline.pipeline_id = 1;
    s_acquisition_pipeline.active_stage_count = 1;
    s_acquisition_pipeline.pipeline_running = false;
    s_acquisition_pipeline.total_pipeline_completions = 0;
    s_acquisition_pipeline.total_pipeline_aborts = 0;
    s_acquisition_pipeline.pipeline_sync_semaphore = xSemaphoreCreateBinary();
    if (s_acquisition_pipeline.pipeline_sync_semaphore == NULL) {
        heap_caps_free(s_primary_rx_channel.dma_aligned_working_buffer);
        vSemaphoreDelete(s_primary_rx_channel.eof_notification_semaphore);
        vSemaphoreDelete(s_primary_rx_channel.channel_access_mutex);
        return RADAR_DAQ_ERR_BUFFER_LOCK_FAIL;
    }

    s_dma_subsystem_initialized = true;
    return RADAR_DAQ_ERR_OK;
}

radar_daq_error_code_t radar_dma_execute_ascan_transfer(uint8_t* destination_buffer,
                                                          size_t   transfer_bytes,
                                                          uint32_t timeout_ms)
{
    if (!s_dma_subsystem_initialized) return RADAR_DAQ_ERR_NOT_INITIALIZED;
    if (destination_buffer == NULL || transfer_bytes == 0) return RADAR_DAQ_ERR_DMA_OVERFLOW;

    radar_dma_channel_context_t* ch = &s_primary_rx_channel;

    if (xSemaphoreTake(ch->channel_access_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return RADAR_DAQ_ERR_BUFFER_LOCK_FAIL;
    }

    uint64_t t_start = esp_timer_get_time();

    radar_daq_error_code_t err = s_dma_build_descriptor_chain(ch, destination_buffer, transfer_bytes);
    if (err != RADAR_DAQ_ERR_OK) {
        xSemaphoreGive(ch->channel_access_mutex);
        return err;
    }

    s_dma_channel_state_transition(ch, RADAR_DMA_CHAN_STATE_DESCRIPTORS_LOADED);

    err = s_dma_arm_transfer(ch);
    if (err != RADAR_DAQ_ERR_OK) {
        s_dma_free_descriptor_chain(ch);
        s_dma_channel_state_transition(ch, RADAR_DMA_CHAN_STATE_IDLE);
        xSemaphoreGive(ch->channel_access_mutex);
        return err;
    }

    xSemaphoreGive(ch->eof_notification_semaphore);

    err = s_dma_wait_for_eof(ch, timeout_ms);
    if (err != RADAR_DAQ_ERR_OK) {
        err = s_dma_handle_stall_recovery(ch);
        if (err != RADAR_DAQ_ERR_OK) {
            s_dma_free_descriptor_chain(ch);
            s_dma_channel_state_transition(ch, RADAR_DMA_CHAN_STATE_IDLE);
            xSemaphoreGive(ch->channel_access_mutex);
            return err;
        }
    }

    uint64_t t_end = esp_timer_get_time();
    s_dma_update_throughput_stats(ch, transfer_bytes, t_end - t_start);
    ch->stats.total_descriptors_processed += ch->descriptor_count;

    s_dma_free_descriptor_chain(ch);
    s_dma_channel_state_transition(ch, RADAR_DMA_CHAN_STATE_IDLE);
    xSemaphoreGive(ch->channel_access_mutex);

    s_acquisition_pipeline.total_pipeline_completions++;

    return RADAR_DAQ_ERR_OK;
}

radar_daq_error_code_t radar_dma_get_channel_statistics(radar_dma_channel_statistics_t* out_stats)
{
    if (!s_dma_subsystem_initialized) return RADAR_DAQ_ERR_NOT_INITIALIZED;
    if (out_stats == NULL) return RADAR_DAQ_ERR_NOT_INITIALIZED;
    *out_stats = s_primary_rx_channel.stats;
    return RADAR_DAQ_ERR_OK;
}

radar_daq_error_code_t radar_dma_subsystem_teardown(void)
{
    if (!s_dma_subsystem_initialized) return RADAR_DAQ_ERR_NOT_INITIALIZED;

    s_dma_free_descriptor_chain(&s_primary_rx_channel);

    if (s_primary_rx_channel.dma_aligned_working_buffer) {
        heap_caps_free(s_primary_rx_channel.dma_aligned_working_buffer);
    }
    if (s_primary_rx_channel.eof_notification_semaphore) {
        vSemaphoreDelete(s_primary_rx_channel.eof_notification_semaphore);
    }
    if (s_primary_rx_channel.channel_access_mutex) {
        vSemaphoreDelete(s_primary_rx_channel.channel_access_mutex);
    }
    if (s_acquisition_pipeline.pipeline_sync_semaphore) {
        vSemaphoreDelete(s_acquisition_pipeline.pipeline_sync_semaphore);
    }

    memset(&s_primary_rx_channel, 0, sizeof(radar_dma_channel_context_t));
    memset(&s_acquisition_pipeline, 0, sizeof(radar_dma_pipeline_context_t));

    s_dma_subsystem_initialized = false;
    s_global_eof_interrupt_count = 0;
    s_global_bus_error_count = 0;

    return RADAR_DAQ_ERR_OK;
}

void radar_dma_debug_dump_statistics(void)
{
    radar_dma_channel_statistics_t* s = &s_primary_rx_channel.stats;
    ESP_LOGI(RADAR_DMA_LOG_TAG, "======== DMA CHANNEL STATS DUMP ========");
    ESP_LOGI(RADAR_DMA_LOG_TAG, "Total Bytes Xfr    : %lu", s->total_bytes_transferred);
    ESP_LOGI(RADAR_DMA_LOG_TAG, "Descriptors Proc   : %lu", s->total_descriptors_processed);
    ESP_LOGI(RADAR_DMA_LOG_TAG, "Transfer Errors    : %lu", s->total_transfer_errors);
    ESP_LOGI(RADAR_DMA_LOG_TAG, "Stall Events       : %lu", s->total_stall_events);
    ESP_LOGI(RADAR_DMA_LOG_TAG, "EOF Interrupts     : %lu", s->total_eof_interrupts);
    ESP_LOGI(RADAR_DMA_LOG_TAG, "Retry Successes    : %lu", s->total_retry_successes);
    ESP_LOGI(RADAR_DMA_LOG_TAG, "Retry Failures     : %lu", s->total_retry_failures);
    ESP_LOGI(RADAR_DMA_LOG_TAG, "Avg Throughput Mbps: %.3f", s->average_throughput_mbps);
    ESP_LOGI(RADAR_DMA_LOG_TAG, "Peak Throughput Mbs: %.3f", s->peak_throughput_mbps);
    ESP_LOGI(RADAR_DMA_LOG_TAG, "Max Transfer Us    : %lu", s->max_observed_transfer_time_us);
    ESP_LOGI(RADAR_DMA_LOG_TAG, "Global EOF Count   : %lu", s_global_eof_interrupt_count);
    ESP_LOGI(RADAR_DMA_LOG_TAG, "Global Bus Errors  : %lu", s_global_bus_error_count);
    ESP_LOGI(RADAR_DMA_LOG_TAG, "========================================");
}
