#ifndef RADAR_PSRAM_MEMORY_MANAGEMENT_SUBSYSTEM_H
#define RADAR_PSRAM_MEMORY_MANAGEMENT_SUBSYSTEM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define RADAR_PSRAM_MAGIC_HEADER_VALUE 0xAA55BEEF
#define RADAR_PSRAM_MAGIC_FOOTER_VALUE 0xDEAD55AA
#define RADAR_MAX_SUPPORTED_POOL_INSTANCES 8
#define RADAR_BSCAN_FRAME_MAX_WIDTH 1024
#define RADAR_BSCAN_FRAME_MAX_DEPTH 2048
#define RADAR_ALIGNMENT_BYTE_BOUNDARY 64

typedef enum {
    RADAR_MEM_SYS_ERR_NONE = 0x00,
    RADAR_MEM_SYS_ERR_NOT_INITIALIZED = 0x01,
    RADAR_MEM_SYS_ERR_OUT_OF_SPIRAM = 0x02,
    RADAR_MEM_SYS_ERR_FRAGMENTATION_CRITICAL = 0x03,
    RADAR_MEM_SYS_ERR_MAGIC_CORRUPTION = 0x04,
    RADAR_MEM_SYS_ERR_MUTEX_TIMEOUT = 0x05,
    RADAR_MEM_SYS_ERR_INVALID_HANDLE = 0x06,
    RADAR_MEM_SYS_ERR_POOL_OVERFLOW = 0x07,
    RADAR_MEM_SYS_ERR_ALIGNMENT_FAULT = 0x08,
    RADAR_MEM_SYS_ERR_CHECKSUM_MISMATCH = 0x09
} radar_mem_sys_error_code_t;

typedef enum {
    RADAR_POOL_STATE_UNINITIALIZED = 0,
    RADAR_POOL_STATE_WARMING_UP,
    RADAR_POOL_STATE_ACTIVE_STABLE,
    RADAR_POOL_STATE_DEGRADED_PERFORMANCE,
    RADAR_POOL_STATE_EMERGENCY_LOCKDOWN,
    RADAR_POOL_STATE_RECOVERING
} radar_pool_operational_state_t;

typedef enum {
    RADAR_MEM_STRATEGY_FIRST_FIT,
    RADAR_MEM_STRATEGY_BEST_FIT,
    RADAR_MEM_STRATEGY_WORST_FIT,
    RADAR_MEM_STRATEGY_LAZY_ALLOCATION
} radar_mem_allocation_strategy_t;

typedef struct radar_mem_block_header_s {
    uint32_t magic_header;
    uint32_t block_id;
    size_t requested_size;
    size_t allocated_size;
    uint64_t timestamp_allocation_ms;
    uint32_t owner_task_id;
    uint16_t access_frequency_counter;
    uint8_t is_locked_for_dma;
    uint8_t fragmentation_index;
    struct radar_mem_block_header_s* next_block;
    struct radar_mem_block_header_s* prev_block;
    uint32_t payload_checksum_crc32;
    uint32_t header_integrity_hash;
} radar_mem_block_header_t;

typedef struct {
    uint32_t total_allocations_requested;
    uint32_t total_allocations_succeeded;
    uint32_t total_frees_executed;
    uint64_t total_bytes_allocated_historically;
    uint32_t peak_memory_usage_bytes;
    uint32_t current_active_blocks;
    uint32_t garbage_collection_triggers;
    uint32_t dma_lock_contention_events;
    uint32_t magic_corruption_recoveries;
    float average_allocation_time_us;
} radar_pool_telemetry_stats_t;

typedef struct {
    char pool_name[32];
    size_t maximum_pool_size_bytes;
    size_t pre_allocation_chunk_size;
    radar_mem_allocation_strategy_t strategy;
    bool enable_automatic_defragmentation;
    bool enable_strict_bounds_checking;
    uint32_t mutex_timeout_ms;
} radar_pool_configuration_profile_t;

typedef struct {
    uint32_t instance_id;
    radar_pool_operational_state_t current_state;
    radar_pool_configuration_profile_t config;
    radar_pool_telemetry_stats_t telemetry;
    radar_mem_block_header_t* root_block_head;
    radar_mem_block_header_t* root_block_tail;
    void* internal_rtos_mutex_handle;
    void* internal_event_group_handle;
    uint8_t* raw_spiram_base_pointer;
    size_t raw_spiram_total_capacity;
    size_t raw_spiram_free_capacity;
    uint32_t subsystem_heartbeat_tick;
} radar_psram_manager_context_t;

typedef void* radar_psram_handle_t;

radar_mem_sys_error_code_t radar_psram_subsystem_global_bootstrap(void);
radar_mem_sys_error_code_t radar_psram_subsystem_global_teardown(void);
radar_mem_sys_error_code_t radar_psram_create_specialized_pool(const radar_pool_configuration_profile_t* config, radar_psram_handle_t* out_handle);
radar_mem_sys_error_code_t radar_psram_destroy_specialized_pool(radar_psram_handle_t handle);
void* radar_psram_allocate_bscan_frame_buffer(radar_psram_handle_t handle, size_t dimensions_x, size_t dimensions_y, radar_mem_sys_error_code_t* out_err);
radar_mem_sys_error_code_t radar_psram_release_bscan_frame_buffer(radar_psram_handle_t handle, void* target_buffer_ptr);
radar_mem_sys_error_code_t radar_psram_force_garbage_collection_routine(radar_psram_handle_t handle);
radar_mem_sys_error_code_t radar_psram_execute_memory_integrity_audit(radar_psram_handle_t handle, bool deep_payload_scan);
radar_mem_sys_error_code_t radar_psram_lock_buffer_for_dma_transaction(radar_psram_handle_t handle, void* target_buffer_ptr);
radar_mem_sys_error_code_t radar_psram_unlock_buffer_after_dma_transaction(radar_psram_handle_t handle, void* target_buffer_ptr);
radar_mem_sys_error_code_t radar_psram_retrieve_telemetry_snapshot(radar_psram_handle_t handle, radar_pool_telemetry_stats_t* out_telemetry);
radar_mem_sys_error_code_t radar_psram_calculate_pool_fragmentation_ratio(radar_psram_handle_t handle, float* out_ratio);
void radar_psram_trigger_emergency_panic_dump(radar_psram_handle_t handle);

#endif