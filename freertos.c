#ifndef RADAR_CROSS_CORE_IPC_PIPELINE_H
#define RADAR_CROSS_CORE_IPC_PIPELINE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define RADAR_IPC_MAGIC_SYNC_WORD 0x5AA5CAFE
#define RADAR_IPC_MAX_PAYLOAD_SIZE 4096
#define RADAR_IPC_CACHE_LINE_SIZE 64
#define RADAR_IPC_PIPELINE_CAPACITY 128
#define RADAR_IPC_CAPACITY_MASK (RADAR_IPC_PIPELINE_CAPACITY - 1)

#define RADAR_ALIGN_CACHE_LINE __attribute__((aligned(RADAR_IPC_CACHE_LINE_SIZE)))

typedef enum {
    RADAR_IPC_ERR_OK = 0,
    RADAR_IPC_ERR_NULL_PTR,
    RADAR_IPC_ERR_FULL,
    RADAR_IPC_ERR_EMPTY,
    RADAR_IPC_ERR_MAGIC_CORRUPT,
    RADAR_IPC_ERR_CRC_FAIL,
    RADAR_IPC_ERR_PAYLOAD_OVERSIZE,
    RADAR_IPC_ERR_ALIGNMENT,
    RADAR_IPC_ERR_NOT_INITIALIZED,
    RADAR_IPC_ERR_FATAL_COLLISION
} radar_ipc_err_t;

typedef enum {
    RADAR_IPC_PKT_TYPE_ASCAN_RAW = 0x10,
    RADAR_IPC_PKT_TYPE_BSCAN_SLICE = 0x20,
    RADAR_IPC_PKT_TYPE_ANOMALY_ALERT = 0x30,
    RADAR_IPC_PKT_TYPE_SYS_TELEMETRY = 0x40,
    RADAR_IPC_PKT_TYPE_CALIBRATION = 0x50,
    RADAR_IPC_PKT_TYPE_HEARTBEAT = 0xFF
} radar_ipc_packet_type_t;

typedef struct {
    uint32_t magic_word;
    uint32_t sequence_number;
    uint64_t timestamp_us;
    radar_ipc_packet_type_t packet_type;
    uint16_t payload_length;
    uint16_t header_crc16;
    uint32_t payload_crc32;
    uint8_t producer_core_id;
    uint8_t fragmentation_flag;
    uint16_t reserved_padding;
} RADAR_ALIGN_CACHE_LINE radar_ipc_packet_header_t;

typedef struct {
    radar_ipc_packet_header_t header;
    uint8_t payload[RADAR_IPC_MAX_PAYLOAD_SIZE];
} RADAR_ALIGN_CACHE_LINE radar_ipc_frame_t;

typedef struct {
    uint64_t total_frames_pushed;
    uint64_t total_frames_popped;
    uint64_t dropped_frames_queue_full;
    uint64_t corrupted_frames_detected;
    uint64_t total_bytes_transferred;
    float current_throughput_kbps;
    uint32_t max_queue_depth_observed;
    uint32_t producer_stalls;
    uint32_t consumer_starvations;
    uint64_t peak_latency_us;
    uint64_t moving_avg_latency_us;
} radar_ipc_telemetry_t;

typedef struct {
    volatile uint32_t head_index RADAR_ALIGN_CACHE_LINE;
    volatile uint32_t tail_index RADAR_ALIGN_CACHE_LINE;
    
    radar_ipc_frame_t* buffer_array RADAR_ALIGN_CACHE_LINE;
    
    volatile uint32_t pipeline_state_flags;
    uint32_t instance_id;
    
    radar_ipc_telemetry_t telemetry RADAR_ALIGN_CACHE_LINE;
    
    uint8_t is_initialized;
    uint8_t strict_ordering_enforced;
    uint16_t watermark_threshold;
} RADAR_ALIGN_CACHE_LINE radar_ipc_pipeline_ctx_t;

radar_ipc_err_t radar_ipc_pipeline_initialize(radar_ipc_pipeline_ctx_t* ctx, void* memory_pool_ptr);
radar_ipc_err_t radar_ipc_pipeline_deinitialize(radar_ipc_pipeline_ctx_t* ctx);
radar_ipc_err_t radar_ipc_push_from_core1(radar_ipc_pipeline_ctx_t* ctx, radar_ipc_packet_type_t type, const uint8_t* data, uint16_t length);
radar_ipc_err_t radar_ipc_pop_to_core0(radar_ipc_pipeline_ctx_t* ctx, radar_ipc_frame_t* out_frame);
radar_ipc_err_t radar_ipc_peek_next_header(radar_ipc_pipeline_ctx_t* ctx, radar_ipc_packet_header_t* out_header);
radar_ipc_err_t radar_ipc_flush_stale_data(radar_ipc_pipeline_ctx_t* ctx);
radar_ipc_err_t radar_ipc_get_telemetry_snapshot(radar_ipc_pipeline_ctx_t* ctx, radar_ipc_telemetry_t* out_telemetry);
uint32_t radar_ipc_get_current_usage_count(radar_ipc_pipeline_ctx_t* ctx);
void radar_ipc_trigger_software_interrupt_to_core0(void);

#endif