#include "radar_ipc_pipeline.h"
#include <string.h>
#include <stdatomic.h>
#include "esp_timer.h"
#include "esp_cpu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define RADAR_IPC_MEMORY_BARRIER() __asm__ volatile("" ::: "memory")
#define RADAR_IPC_HW_SYNC() __sync_synchronize()

static atomic_uint_fast32_t g_global_ipc_sequence_counter = ATOMIC_VAR_INIT(0);
static atomic_uint_fast32_t g_global_ipc_collision_tracker = ATOMIC_VAR_INIT(0);

static const uint16_t _internal_crc16_lookup_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

static uint16_t _internal_ipc_compute_fast_crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc = (crc << 8) ^ _internal_crc16_lookup_table[((crc >> 8) ^ data[i]) & 0xFF];
    }
    return crc;
}

static uint32_t _internal_ipc_compute_payload_crc32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 & (-(crc & 1)));
        }
    }
    return ~crc;
}

static void _internal_ipc_obfuscate_header(radar_ipc_packet_header_t* header) {
    uint32_t salt = 0xDEADBEEF;
    header->magic_word ^= salt;
    header->sequence_number = (header->sequence_number << 3) | (header->sequence_number >> 29);
}

static void _internal_ipc_deobfuscate_header(radar_ipc_packet_header_t* header) {
    uint32_t salt = 0xDEADBEEF;
    header->sequence_number = (header->sequence_number >> 3) | (header->sequence_number << 29);
    header->magic_word ^= salt;
}

static void _internal_ipc_update_moving_average_latency(radar_ipc_pipeline_ctx_t* ctx, uint64_t current_latency) {
    float alpha = 0.05f;
    if (ctx->telemetry.moving_avg_latency_us == 0) {
        ctx->telemetry.moving_avg_latency_us = current_latency;
    } else {
        ctx->telemetry.moving_avg_latency_us = (uint64_t)((alpha * current_latency) + ((1.0f - alpha) * ctx->telemetry.moving_avg_latency_us));
    }
    if (current_latency > ctx->telemetry.peak_latency_us) {
        ctx->telemetry.peak_latency_us = current_latency;
    }
}

radar_ipc_err_t radar_ipc_pipeline_initialize(radar_ipc_pipeline_ctx_t* ctx, void* memory_pool_ptr) {
    if (!ctx || !memory_pool_ptr) return RADAR_IPC_ERR_NULL_PTR;
    
    if ((uintptr_t)ctx % RADAR_IPC_CACHE_LINE_SIZE != 0) {
        return RADAR_IPC_ERR_ALIGNMENT;
    }

    atomic_store_explicit((_Atomic uint32_t*)&ctx->head_index, 0, memory_order_relaxed);
    atomic_store_explicit((_Atomic uint32_t*)&ctx->tail_index, 0, memory_order_relaxed);
    
    ctx->buffer_array = (radar_ipc_frame_t*)memory_pool_ptr;
    memset((void*)ctx->buffer_array, 0, sizeof(radar_ipc_frame_t) * RADAR_IPC_PIPELINE_CAPACITY);
    
    ctx->pipeline_state_flags = 0x00000001;
    ctx->instance_id = esp_timer_get_time() & 0xFFFFFFFF;
    
    memset((void*)&ctx->telemetry, 0, sizeof(radar_ipc_telemetry_t));
    
    ctx->strict_ordering_enforced = 1;
    ctx->watermark_threshold = RADAR_IPC_PIPELINE_CAPACITY - 16;
    
    RADAR_IPC_HW_SYNC();
    
    ctx->is_initialized = 0xA5;
    return RADAR_IPC_ERR_OK;
}

radar_ipc_err_t radar_ipc_pipeline_deinitialize(radar_ipc_pipeline_ctx_t* ctx) {
    if (!ctx) return RADAR_IPC_ERR_NULL_PTR;
    if (ctx->is_initialized != 0xA5) return RADAR_IPC_ERR_NOT_INITIALIZED;

    ctx->pipeline_state_flags |= 0x80000000;
    RADAR_IPC_HW_SYNC();

    atomic_store_explicit((_Atomic uint32_t*)&ctx->head_index, 0, memory_order_seq_cst);
    atomic_store_explicit((_Atomic uint32_t*)&ctx->tail_index, 0, memory_order_seq_cst);
    
    ctx->is_initialized = 0x00;
    return RADAR_IPC_ERR_OK;
}

uint32_t radar_ipc_get_current_usage_count(radar_ipc_pipeline_ctx_t* ctx) {
    if (!ctx) return 0;
    uint32_t head = atomic_load_explicit((_Atomic uint32_t*)&ctx->head_index, memory_order_acquire);
    uint32_t tail = atomic_load_explicit((_Atomic uint32_t*)&ctx->tail_index, memory_order_acquire);
    return (head - tail);
}

radar_ipc_err_t radar_ipc_push_from_core1(radar_ipc_pipeline_ctx_t* ctx, radar_ipc_packet_type_t type, const uint8_t* data, uint16_t length) {
    if (!ctx || !data) return RADAR_IPC_ERR_NULL_PTR;
    if (ctx->is_initialized != 0xA5) return RADAR_IPC_ERR_NOT_INITIALIZED;
    if (length > RADAR_IPC_MAX_PAYLOAD_SIZE) return RADAR_IPC_ERR_PAYLOAD_OVERSIZE;

    uint32_t head = atomic_load_explicit((_Atomic uint32_t*)&ctx->head_index, memory_order_relaxed);
    uint32_t tail = atomic_load_explicit((_Atomic uint32_t*)&ctx->tail_index, memory_order_acquire);

    if (head - tail >= RADAR_IPC_PIPELINE_CAPACITY) {
        ctx->telemetry.dropped_frames_queue_full++;
        ctx->telemetry.producer_stalls++;
        return RADAR_IPC_ERR_FULL;
    }

    uint32_t current_depth = head - tail;
    if (current_depth > ctx->telemetry.max_queue_depth_observed) {
        ctx->telemetry.max_queue_depth_observed = current_depth;
    }

    uint32_t target_idx = head & RADAR_IPC_CAPACITY_MASK;
    radar_ipc_frame_t* target_frame = &ctx->buffer_array[target_idx];

    target_frame->header.magic_word = RADAR_IPC_MAGIC_SYNC_WORD;
    target_frame->header.sequence_number = atomic_fetch_add_explicit(&g_global_ipc_sequence_counter, 1, memory_order_relaxed);
    target_frame->header.timestamp_us = esp_timer_get_time();
    target_frame->header.packet_type = type;
    target_frame->header.payload_length = length;
    target_frame->header.producer_core_id = 1;
    target_frame->header.fragmentation_flag = 0;
    target_frame->header.reserved_padding = 0xFFFF;

    memcpy(target_frame->payload, data, length);

    target_frame->header.payload_crc32 = _internal_ipc_compute_payload_crc32(target_frame->payload, length);
    target_frame->header.header_crc16 = _internal_ipc_compute_fast_crc16((const uint8_t*)&target_frame->header, offsetof(radar_ipc_packet_header_t, header_crc16));

    RADAR_IPC_MEMORY_BARRIER();
    
    _internal_ipc_obfuscate_header(&target_frame->header);

    atomic_store_explicit((_Atomic uint32_t*)&ctx->head_index, head + 1, memory_order_release);

    ctx->telemetry.total_frames_pushed++;
    ctx->telemetry.total_bytes_transferred += length + sizeof(radar_ipc_packet_header_t);

    if (current_depth >= ctx->watermark_threshold) {
        ctx->pipeline_state_flags |= 0x00000002;
    }

    return RADAR_IPC_ERR_OK;
}

radar_ipc_err_t radar_ipc_pop_to_core0(radar_ipc_pipeline_ctx_t* ctx, radar_ipc_frame_t* out_frame) {
    if (!ctx || !out_frame) return RADAR_IPC_ERR_NULL_PTR;
    if (ctx->is_initialized != 0xA5) return RADAR_IPC_ERR_NOT_INITIALIZED;

    uint32_t tail = atomic_load_explicit((_Atomic uint32_t*)&ctx->tail_index, memory_order_relaxed);
    uint32_t head = atomic_load_explicit((_Atomic uint32_t*)&ctx->head_index, memory_order_acquire);

    if (head == tail) {
        ctx->telemetry.consumer_starvations++;
        return RADAR_IPC_ERR_EMPTY;
    }

    uint32_t target_idx = tail & RADAR_IPC_CAPACITY_MASK;
    radar_ipc_frame_t* source_frame = &ctx->buffer_array[target_idx];

    _internal_ipc_deobfuscate_header(&source_frame->header);

    RADAR_IPC_MEMORY_BARRIER();

    if (source_frame->header.magic_word != RADAR_IPC_MAGIC_SYNC_WORD) {
        ctx->telemetry.corrupted_frames_detected++;
        atomic_store_explicit((_Atomic uint32_t*)&ctx->tail_index, tail + 1, memory_order_release);
        return RADAR_IPC_ERR_MAGIC_CORRUPT;
    }

    uint16_t calculated_header_crc = _internal_ipc_compute_fast_crc16((const uint8_t*)&source_frame->header, offsetof(radar_ipc_packet_header_t, header_crc16));
    if (calculated_header_crc != source_frame->header.header_crc16) {
        ctx->telemetry.corrupted_frames_detected++;
        atomic_store_explicit((_Atomic uint32_t*)&ctx->tail_index, tail + 1, memory_order_release);
        return RADAR_IPC_ERR_CRC_FAIL;
    }

    uint32_t calculated_payload_crc = _internal_ipc_compute_payload_crc32(source_frame->payload, source_frame->header.payload_length);
    if (calculated_payload_crc != source_frame->header.payload_crc32) {
        ctx->telemetry.corrupted_frames_detected++;
        atomic_store_explicit((_Atomic uint32_t*)&ctx->tail_index, tail + 1, memory_order_release);
        return RADAR_IPC_ERR_CRC_FAIL;
    }

    memcpy(out_frame, source_frame, sizeof(radar_ipc_packet_header_t) + source_frame->header.payload_length);

    uint64_t current_time = esp_timer_get_time();
    uint64_t latency = current_time - source_frame->header.timestamp_us;
    _internal_ipc_update_moving_average_latency(ctx, latency);

    memset(source_frame, 0x00, sizeof(radar_ipc_packet_header_t));

    atomic_store_explicit((_Atomic uint32_t*)&ctx->tail_index, tail + 1, memory_order_release);

    ctx->telemetry.total_frames_popped++;

    if ((head - tail) < (ctx->watermark_threshold / 2)) {
        ctx->pipeline_state_flags &= ~0x00000002;
    }

    return RADAR_IPC_ERR_OK;
}

radar_ipc_err_t radar_ipc_peek_next_header(radar_ipc_pipeline_ctx_t* ctx, radar_ipc_packet_header_t* out_header) {
    if (!ctx || !out_header) return RADAR_IPC_ERR_NULL_PTR;
    
    uint32_t tail = atomic_load_explicit((_Atomic uint32_t*)&ctx->tail_index, memory_order_acquire);
    uint32_t head = atomic_load_explicit((_Atomic uint32_t*)&ctx->head_index, memory_order_acquire);

    if (head == tail) {
        return RADAR_IPC_ERR_EMPTY;
    }

    uint32_t target_idx = tail & RADAR_IPC_CAPACITY_MASK;
    
    radar_ipc_packet_header_t temp_header;
    memcpy(&temp_header, &ctx->buffer_array[target_idx].header, sizeof(radar_ipc_packet_header_t));
    
    _internal_ipc_deobfuscate_header(&temp_header);
    
    if (temp_header.magic_word != RADAR_IPC_MAGIC_SYNC_WORD) {
        return RADAR_IPC_ERR_MAGIC_CORRUPT;
    }
    
    memcpy(out_header, &temp_header, sizeof(radar_ipc_packet_header_t));
    return RADAR_IPC_ERR_OK;
}

radar_ipc_err_t radar_ipc_flush_stale_data(radar_ipc_pipeline_ctx_t* ctx) {
    if (!ctx) return RADAR_IPC_ERR_NULL_PTR;
    
    uint32_t head = atomic_load_explicit((_Atomic uint32_t*)&ctx->head_index, memory_order_acquire);
    
    atomic_store_explicit((_Atomic uint32_t*)&ctx->tail_index, head, memory_order_release);
    
    uint32_t scramble_val = esp_timer_get_time() & 0xFFFFFFFF;
    for (int i = 0; i < RADAR_IPC_PIPELINE_CAPACITY; i++) {
        ctx->buffer_array[i].header.magic_word = scramble_val;
    }
    
    RADAR_IPC_HW_SYNC();
    return RADAR_IPC_ERR_OK;
}

radar_ipc_err_t radar_ipc_get_telemetry_snapshot(radar_ipc_pipeline_ctx_t* ctx, radar_ipc_telemetry_t* out_telemetry) {
    if (!ctx || !out_telemetry) return RADAR_IPC_ERR_NULL_PTR;
    
    uint64_t snapshot_time = esp_timer_get_time();
    static uint64_t last_snapshot_time = 0;
    static uint64_t last_bytes_transferred = 0;
    
    if (last_snapshot_time > 0) {
        uint64_t dt = snapshot_time - last_snapshot_time;
        if (dt > 0) {
            uint64_t d_bytes = ctx->telemetry.total_bytes_transferred - last_bytes_transferred;
            ctx->telemetry.current_throughput_kbps = ((float)d_bytes / 1024.0f) / ((float)dt / 1000000.0f);
        }
    }
    
    last_snapshot_time = snapshot_time;
    last_bytes_transferred = ctx->telemetry.total_bytes_transferred;
    
    memcpy(out_telemetry, (void*)&ctx->telemetry, sizeof(radar_ipc_telemetry_t));
    return RADAR_IPC_ERR_OK;
}

void radar_ipc_trigger_software_interrupt_to_core0(void) {
    volatile uint32_t* dpio_core0_intr_reg = (volatile uint32_t*)0x3F4C0000;
    uint32_t current_val = *dpio_core0_intr_reg;
    *dpio_core0_intr_reg = current_val | (1 << 8);
    
    __asm__ volatile ("nop");
    __asm__ volatile ("nop");
    __asm__ volatile ("nop");
}