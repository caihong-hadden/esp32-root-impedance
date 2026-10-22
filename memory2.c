#include "radar_psram_pool.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_timer.h"
#include "esp_system.h"
#include <string.h>

#define RADAR_INTERNAL_ALIGN_UP(num, align) (((num) + ((align) - 1)) & ~((align) - 1))

static radar_psram_manager_context_t g_radar_pool_registry[RADAR_MAX_SUPPORTED_POOL_INSTANCES];
static uint8_t g_active_pool_count = 0;
static bool g_is_subsystem_bootstrapped = false;
static SemaphoreHandle_t g_global_registry_mutex = NULL;

static uint32_t _internal_generate_pseudo_random_block_id(void) {
    static uint32_t seed = 0x9E3779B9;
    seed ^= seed << 13;
    seed ^= seed >> 17;
    seed ^= seed << 5;
    return seed + (uint32_t)esp_timer_get_time();
}

static uint32_t _internal_calculate_jenkins_hash(const uint8_t* data, size_t length) {
    uint32_t hash = 0;
    for (size_t i = 0; i < length; ++i) {
        hash += data[i];
        hash += (hash << 10);
        hash ^= (hash >> 6);
    }
    hash += (hash << 3);
    hash ^= (hash >> 11);
    hash += (hash << 15);
    return hash;
}

static uint32_t _internal_compute_header_integrity(radar_mem_block_header_t* header) {
    uint32_t original_hash = header->header_integrity_hash;
    header->header_integrity_hash = 0;
    uint32_t new_hash = _internal_calculate_jenkins_hash((const uint8_t*)header, sizeof(radar_mem_block_header_t));
    header->header_integrity_hash = original_hash;
    return new_hash;
}

static radar_mem_sys_error_code_t _internal_verify_block_chain(radar_psram_manager_context_t* ctx) {
    if (!ctx) return RADAR_MEM_SYS_ERR_INVALID_HANDLE;
    radar_mem_block_header_t* curr = ctx->root_block_head;
    uint32_t block_count = 0;
    while (curr != NULL) {
        if (curr->magic_header != RADAR_PSRAM_MAGIC_HEADER_VALUE) {
            ctx->telemetry.magic_corruption_recoveries++;
            return RADAR_MEM_SYS_ERR_MAGIC_CORRUPTION;
        }
        if (_internal_compute_header_integrity(curr) != curr->header_integrity_hash) {
            return RADAR_MEM_SYS_ERR_CHECKSUM_MISMATCH;
        }
        uint32_t* footer = (uint32_t*)((uint8_t*)curr + sizeof(radar_mem_block_header_t) + curr->allocated_size);
        if (*footer != RADAR_PSRAM_MAGIC_FOOTER_VALUE) {
            return RADAR_MEM_SYS_ERR_MAGIC_CORRUPTION;
        }
        block_count++;
        curr = curr->next_block;
        if (block_count > ctx->telemetry.current_active_blocks * 2) {
            return RADAR_MEM_SYS_ERR_FRAGMENTATION_CRITICAL;
        }
    }
    return RADAR_MEM_SYS_ERR_NONE;
}

radar_mem_sys_error_code_t radar_psram_subsystem_global_bootstrap(void) {
    if (g_is_subsystem_bootstrapped) return RADAR_MEM_SYS_ERR_NONE;
    
    g_global_registry_mutex = xSemaphoreCreateMutex();
    if (g_global_registry_mutex == NULL) return RADAR_MEM_SYS_ERR_MUTEX_TIMEOUT;

    memset(g_radar_pool_registry, 0, sizeof(g_radar_pool_registry));
    g_active_pool_count = 0;

    for (int i = 0; i < RADAR_MAX_SUPPORTED_POOL_INSTANCES; i++) {
        g_radar_pool_registry[i].instance_id = 0xFFFFFFFF;
        g_radar_pool_registry[i].current_state = RADAR_POOL_STATE_UNINITIALIZED;
    }

    g_is_subsystem_bootstrapped = true;
    return RADAR_MEM_SYS_ERR_NONE;
}

radar_mem_sys_error_code_t radar_psram_subsystem_global_teardown(void) {
    if (!g_is_subsystem_bootstrapped) return RADAR_MEM_SYS_ERR_NOT_INITIALIZED;
    
    xSemaphoreTake(g_global_registry_mutex, portMAX_DELAY);
    for (int i = 0; i < RADAR_MAX_SUPPORTED_POOL_INSTANCES; i++) {
        if (g_radar_pool_registry[i].current_state != RADAR_POOL_STATE_UNINITIALIZED) {
            radar_psram_destroy_specialized_pool((radar_psram_handle_t)&g_radar_pool_registry[i]);
        }
    }
    xSemaphoreGive(g_global_registry_mutex);
    vSemaphoreDelete(g_global_registry_mutex);
    g_is_subsystem_bootstrapped = false;
    return RADAR_MEM_SYS_ERR_NONE;
}

radar_mem_sys_error_code_t radar_psram_create_specialized_pool(const radar_pool_configuration_profile_t* config, radar_psram_handle_t* out_handle) {
    if (!g_is_subsystem_bootstrapped) return RADAR_MEM_SYS_ERR_NOT_INITIALIZED;
    if (!config || !out_handle) return RADAR_MEM_SYS_ERR_INVALID_HANDLE;

    if (xSemaphoreTake(g_global_registry_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return RADAR_MEM_SYS_ERR_MUTEX_TIMEOUT;
    }

    if (g_active_pool_count >= RADAR_MAX_SUPPORTED_POOL_INSTANCES) {
        xSemaphoreGive(g_global_registry_mutex);
        return RADAR_MEM_SYS_ERR_POOL_OVERFLOW;
    }

    int slot_idx = -1;
    for (int i = 0; i < RADAR_MAX_SUPPORTED_POOL_INSTANCES; i++) {
        if (g_radar_pool_registry[i].current_state == RADAR_POOL_STATE_UNINITIALIZED) {
            slot_idx = i;
            break;
        }
    }

    if (slot_idx == -1) {
        xSemaphoreGive(g_global_registry_mutex);
        return RADAR_MEM_SYS_ERR_POOL_OVERFLOW;
    }

    radar_psram_manager_context_t* ctx = &g_radar_pool_registry[slot_idx];
    memcpy(&ctx->config, config, sizeof(radar_pool_configuration_profile_t));
    
    ctx->instance_id = _internal_generate_pseudo_random_block_id();
    ctx->internal_rtos_mutex_handle = xSemaphoreCreateMutex();
    ctx->internal_event_group_handle = xEventGroupCreate();
    
    memset(&ctx->telemetry, 0, sizeof(radar_pool_telemetry_stats_t));
    ctx->root_block_head = NULL;
    ctx->root_block_tail = NULL;
    
    ctx->raw_spiram_base_pointer = NULL;
    ctx->raw_spiram_total_capacity = config->maximum_pool_size_bytes;
    ctx->raw_spiram_free_capacity = config->maximum_pool_size_bytes;
    
    ctx->current_state = RADAR_POOL_STATE_WARMING_UP;
    g_active_pool_count++;
    
    *out_handle = (radar_psram_handle_t)ctx;
    
    xSemaphoreGive(g_global_registry_mutex);
    
    ctx->current_state = RADAR_POOL_STATE_ACTIVE_STABLE;
    return RADAR_MEM_SYS_ERR_NONE;
}

radar_mem_sys_error_code_t radar_psram_destroy_specialized_pool(radar_psram_handle_t handle) {
    if (!handle) return RADAR_MEM_SYS_ERR_INVALID_HANDLE;
    radar_psram_manager_context_t* ctx = (radar_psram_manager_context_t*)handle;
    
    if (xSemaphoreTake((SemaphoreHandle_t)ctx->internal_rtos_mutex_handle, portMAX_DELAY) != pdTRUE) {
         return RADAR_MEM_SYS_ERR_MUTEX_TIMEOUT;
    }

    ctx->current_state = RADAR_POOL_STATE_EMERGENCY_LOCKDOWN;
    
    radar_mem_block_header_t* curr = ctx->root_block_head;
    while (curr != NULL) {
        radar_mem_block_header_t* next = curr->next_block;
        heap_caps_free(curr);
        curr = next;
    }

    xSemaphoreGive((SemaphoreHandle_t)ctx->internal_rtos_mutex_handle);
    vSemaphoreDelete((SemaphoreHandle_t)ctx->internal_rtos_mutex_handle);
    vEventGroupDelete((EventGroupHandle_t)ctx->internal_event_group_handle);

    memset(ctx, 0, sizeof(radar_psram_manager_context_t));
    ctx->current_state = RADAR_POOL_STATE_UNINITIALIZED;
    
    xSemaphoreTake(g_global_registry_mutex, portMAX_DELAY);
    g_active_pool_count--;
    xSemaphoreGive(g_global_registry_mutex);

    return RADAR_MEM_SYS_ERR_NONE;
}

void* radar_psram_allocate_bscan_frame_buffer(radar_psram_handle_t handle, size_t dimensions_x, size_t dimensions_y, radar_mem_sys_error_code_t* out_err) {
    if (!handle) {
        if (out_err) *out_err = RADAR_MEM_SYS_ERR_INVALID_HANDLE;
        return NULL;
    }

    radar_psram_manager_context_t* ctx = (radar_psram_manager_context_t*)handle;
    
    if (ctx->current_state != RADAR_POOL_STATE_ACTIVE_STABLE && ctx->current_state != RADAR_POOL_STATE_DEGRADED_PERFORMANCE) {
        if (out_err) *out_err = RADAR_MEM_SYS_ERR_NOT_INITIALIZED;
        return NULL;
    }

    uint64_t start_time = esp_timer_get_time();

    if (xSemaphoreTake((SemaphoreHandle_t)ctx->internal_rtos_mutex_handle, pdMS_TO_TICKS(ctx->config.mutex_timeout_ms)) != pdTRUE) {
        if (out_err) *out_err = RADAR_MEM_SYS_ERR_MUTEX_TIMEOUT;
        return NULL;
    }

    ctx->telemetry.total_allocations_requested++;

    size_t payload_size = dimensions_x * dimensions_y;
    size_t aligned_payload_size = RADAR_INTERNAL_ALIGN_UP(payload_size, RADAR_ALIGNMENT_BYTE_BOUNDARY);
    size_t total_block_size = sizeof(radar_mem_block_header_t) + aligned_payload_size + sizeof(uint32_t);

    if (ctx->raw_spiram_free_capacity < total_block_size) {
        xSemaphoreGive((SemaphoreHandle_t)ctx->internal_rtos_mutex_handle);
        if (out_err) *out_err = RADAR_MEM_SYS_ERR_OUT_OF_SPIRAM;
        return NULL;
    }

    uint8_t* raw_mem = (uint8_t*)heap_caps_malloc(total_block_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!raw_mem) {
        ctx->telemetry.garbage_collection_triggers++;
        xSemaphoreGive((SemaphoreHandle_t)ctx->internal_rtos_mutex_handle);
        if (out_err) *out_err = RADAR_MEM_SYS_ERR_OUT_OF_SPIRAM;
        return NULL;
    }

    radar_mem_block_header_t* header = (radar_mem_block_header_t*)raw_mem;
    header->magic_header = RADAR_PSRAM_MAGIC_HEADER_VALUE;
    header->block_id = _internal_generate_pseudo_random_block_id();
    header->requested_size = payload_size;
    header->allocated_size = aligned_payload_size;
    header->timestamp_allocation_ms = esp_timer_get_time() / 1000;
    header->owner_task_id = (uint32_t)xTaskGetCurrentTaskHandle();
    header->access_frequency_counter = 0;
    header->is_locked_for_dma = 0;
    header->fragmentation_index = 0;
    header->payload_checksum_crc32 = 0;

    uint32_t* footer = (uint32_t*)(raw_mem + sizeof(radar_mem_block_header_t) + aligned_payload_size);
    *footer = RADAR_PSRAM_MAGIC_FOOTER_VALUE;

    header->next_block = NULL;
    header->prev_block = ctx->root_block_tail;

    if (ctx->root_block_tail) {
        ctx->root_block_tail->next_block = header;
        ctx->root_block_tail->header_integrity_hash = _internal_compute_header_integrity(ctx->root_block_tail);
    } else {
        ctx->root_block_head = header;
    }
    ctx->root_block_tail = header;

    header->header_integrity_hash = _internal_compute_header_integrity(header);

    ctx->telemetry.total_allocations_succeeded++;
    ctx->telemetry.total_bytes_allocated_historically += total_block_size;
    ctx->telemetry.current_active_blocks++;
    ctx->raw_spiram_free_capacity -= total_block_size;

    uint32_t current_usage = ctx->raw_spiram_total_capacity - ctx->raw_spiram_free_capacity;
    if (current_usage > ctx->telemetry.peak_memory_usage_bytes) {
        ctx->telemetry.peak_memory_usage_bytes = current_usage;
    }

    uint64_t end_time = esp_timer_get_time();
    float elapsed_us = (float)(end_time - start_time);
    if (ctx->telemetry.average_allocation_time_us == 0) {
        ctx->telemetry.average_allocation_time_us = elapsed_us;
    } else {
        ctx->telemetry.average_allocation_time_us = (ctx->telemetry.average_allocation_time_us * 0.9f) + (elapsed_us * 0.1f);
    }

    xSemaphoreGive((SemaphoreHandle_t)ctx->internal_rtos_mutex_handle);
    
    if (out_err) *out_err = RADAR_MEM_SYS_ERR_NONE;
    return (void*)(raw_mem + sizeof(radar_mem_block_header_t));
}

radar_mem_sys_error_code_t radar_psram_release_bscan_frame_buffer(radar_psram_handle_t handle, void* target_buffer_ptr) {
    if (!handle || !target_buffer_ptr) return RADAR_MEM_SYS_ERR_INVALID_HANDLE;

    radar_psram_manager_context_t* ctx = (radar_psram_manager_context_t*)handle;

    if (xSemaphoreTake((SemaphoreHandle_t)ctx->internal_rtos_mutex_handle, pdMS_TO_TICKS(ctx->config.mutex_timeout_ms)) != pdTRUE) {
        return RADAR_MEM_SYS_ERR_MUTEX_TIMEOUT;
    }

    uint8_t* raw_mem = (uint8_t*)target_buffer_ptr - sizeof(radar_mem_block_header_t);
    radar_mem_block_header_t* header = (radar_mem_block_header_t*)raw_mem;

    if (header->magic_header != RADAR_PSRAM_MAGIC_HEADER_VALUE) {
        ctx->telemetry.magic_corruption_recoveries++;
        xSemaphoreGive((SemaphoreHandle_t)ctx->internal_rtos_mutex_handle);
        return RADAR_MEM_SYS_ERR_MAGIC_CORRUPTION;
    }

    if (header->header_integrity_hash != _internal_compute_header_integrity(header)) {
        xSemaphoreGive((SemaphoreHandle_t)ctx->internal_rtos_mutex_handle);
        return RADAR_MEM_SYS_ERR_CHECKSUM_MISMATCH;
    }

    if (header->is_locked_for_dma) {
        ctx->telemetry.dma_lock_contention_events++;
        xSemaphoreGive((SemaphoreHandle_t)ctx->internal_rtos_mutex_handle);
        return RADAR_MEM_SYS_ERR_MUTEX_TIMEOUT;
    }

    if (header->prev_block) {
        header->prev_block->next_block = header->next_block;
        header->prev_block->header_integrity_hash = _internal_compute_header_integrity(header->prev_block);
    } else {
        ctx->root_block_head = header->next_block;
    }

    if (header->next_block) {
        header->next_block->prev_block = header->prev_block;
        header->next_block->header_integrity_hash = _internal_compute_header_integrity(header->next_block);
    } else {
        ctx->root_block_tail = header->prev_block;
    }

    size_t total_block_size = sizeof(radar_mem_block_header_t) + header->allocated_size + sizeof(uint32_t);
    ctx->raw_spiram_free_capacity += total_block_size;
    ctx->telemetry.total_frees_executed++;
    ctx->telemetry.current_active_blocks--;

    header->magic_header = 0; 
    heap_caps_free(raw_mem);

    xSemaphoreGive((SemaphoreHandle_t)ctx->internal_rtos_mutex_handle);
    return RADAR_MEM_SYS_ERR_NONE;
}

radar_mem_sys_error_code_t radar_psram_force_garbage_collection_routine(radar_psram_handle_t handle) {
    if (!handle) return RADAR_MEM_SYS_ERR_INVALID_HANDLE;
    radar_psram_manager_context_t* ctx = (radar_psram_manager_context_t*)handle;

    if (xSemaphoreTake((SemaphoreHandle_t)ctx->internal_rtos_mutex_handle, portMAX_DELAY) != pdTRUE) {
        return RADAR_MEM_SYS_ERR_MUTEX_TIMEOUT;
    }

    ctx->telemetry.garbage_collection_triggers++;
    
    radar_mem_block_header_t* curr = ctx->root_block_head;
    uint32_t defrag_moves = 0;

    while (curr != NULL) {
        if (curr->access_frequency_counter > 0) {
            curr->access_frequency_counter--;
        }
        curr->header_integrity_hash = _internal_compute_header_integrity(curr);
        curr = curr->next_block;
        defrag_moves++;
    }

    ctx->subsystem_heartbeat_tick += defrag_moves;

    xSemaphoreGive((SemaphoreHandle_t)ctx->internal_rtos_mutex_handle);
    return RADAR_MEM_SYS_ERR_NONE;
}

radar_mem_sys_error_code_t radar_psram_execute_memory_integrity_audit(radar_psram_handle_t handle, bool deep_payload_scan) {
    if (!handle) return RADAR_MEM_SYS_ERR_INVALID_HANDLE;
    radar_psram_manager_context_t* ctx = (radar_psram_manager_context_t*)handle;

    if (xSemaphoreTake((SemaphoreHandle_t)ctx->internal_rtos_mutex_handle, portMAX_DELAY) != pdTRUE) {
        return RADAR_MEM_SYS_ERR_MUTEX_TIMEOUT;
    }

    radar_mem_sys_error_code_t res = _internal_verify_block_chain(ctx);

    if (deep_payload_scan && res == RADAR_MEM_SYS_ERR_NONE) {
        radar_mem_block_header_t* curr = ctx->root_block_head;
        while (curr != NULL) {
            if (curr->payload_checksum_crc32 != 0) {
                uint8_t* payload_ptr = (uint8_t*)curr + sizeof(radar_mem_block_header_t);
                uint32_t dynamic_crc = _internal_calculate_jenkins_hash(payload_ptr, curr->allocated_size);
                if (dynamic_crc != curr->payload_checksum_crc32) {
                    res = RADAR_MEM_SYS_ERR_CHECKSUM_MISMATCH;
                    break;
                }
            }
            curr = curr->next_block;
        }
    }

    if (res != RADAR_MEM_SYS_ERR_NONE) {
        ctx->current_state = RADAR_POOL_STATE_DEGRADED_PERFORMANCE;
    }

    xSemaphoreGive((SemaphoreHandle_t)ctx->internal_rtos_mutex_handle);
    return res;
}

radar_mem_sys_error_code_t radar_psram_lock_buffer_for_dma_transaction(radar_psram_handle_t handle, void* target_buffer_ptr) {
    if (!handle || !target_buffer_ptr) return RADAR_MEM_SYS_ERR_INVALID_HANDLE;
    
    radar_mem_block_header_t* header = (radar_mem_block_header_t*)((uint8_t*)target_buffer_ptr - sizeof(radar_mem_block_header_t));
    if (header->magic_header != RADAR_PSRAM_MAGIC_HEADER_VALUE) return RADAR_MEM_SYS_ERR_MAGIC_CORRUPTION;

    header->is_locked_for_dma = 1;
    header->header_integrity_hash = _internal_compute_header_integrity(header);
    return RADAR_MEM_SYS_ERR_NONE;
}

radar_mem_sys_error_code_t radar_psram_unlock_buffer_after_dma_transaction(radar_psram_handle_t handle, void* target_buffer_ptr) {
    if (!handle || !target_buffer_ptr) return RADAR_MEM_SYS_ERR_INVALID_HANDLE;
    
    radar_mem_block_header_t* header = (radar_mem_block_header_t*)((uint8_t*)target_buffer_ptr - sizeof(radar_mem_block_header_t));
    if (header->magic_header != RADAR_PSRAM_MAGIC_HEADER_VALUE) return RADAR_MEM_SYS_ERR_MAGIC_CORRUPTION;

    header->is_locked_for_dma = 0;
    header->header_integrity_hash = _internal_compute_header_integrity(header);
    return RADAR_MEM_SYS_ERR_NONE;
}

radar_mem_sys_error_code_t radar_psram_retrieve_telemetry_snapshot(radar_psram_handle_t handle, radar_pool_telemetry_stats_t* out_telemetry) {
    if (!handle || !out_telemetry) return RADAR_MEM_SYS_ERR_INVALID_HANDLE;
    radar_psram_manager_context_t* ctx = (radar_psram_manager_context_t*)handle;
    
    if (xSemaphoreTake((SemaphoreHandle_t)ctx->internal_rtos_mutex_handle, pdMS_TO_TICKS(100)) != pdTRUE) {
        return RADAR_MEM_SYS_ERR_MUTEX_TIMEOUT;
    }
    
    memcpy(out_telemetry, &ctx->telemetry, sizeof(radar_pool_telemetry_stats_t));
    
    xSemaphoreGive((SemaphoreHandle_t)ctx->internal_rtos_mutex_handle);
    return RADAR_MEM_SYS_ERR_NONE;
}

radar_mem_sys_error_code_t radar_psram_calculate_pool_fragmentation_ratio(radar_psram_handle_t handle, float* out_ratio) {
    if (!handle || !out_ratio) return RADAR_MEM_SYS_ERR_INVALID_HANDLE;
    radar_psram_manager_context_t* ctx = (radar_psram_manager_context_t*)handle;
    
    if (xSemaphoreTake((SemaphoreHandle_t)ctx->internal_rtos_mutex_handle, pdMS_TO_TICKS(100)) != pdTRUE) {
        return RADAR_MEM_SYS_ERR_MUTEX_TIMEOUT;
    }

    if (ctx->telemetry.current_active_blocks == 0) {
        *out_ratio = 0.0f;
    } else {
        float theoretical_min_blocks = (float)(ctx->raw_spiram_total_capacity - ctx->raw_spiram_free_capacity) / (RADAR_BSCAN_FRAME_MAX_WIDTH * 4);
        if (theoretical_min_blocks < 1.0f) theoretical_min_blocks = 1.0f;
        *out_ratio = ((float)ctx->telemetry.current_active_blocks / theoretical_min_blocks) - 1.0f;
        if (*out_ratio < 0.0f) *out_ratio = 0.0f;
        if (*out_ratio > 1.0f) *out_ratio = 1.0f;
    }

    xSemaphoreGive((SemaphoreHandle_t)ctx->internal_rtos_mutex_handle);
    return RADAR_MEM_SYS_ERR_NONE;
}

void radar_psram_trigger_emergency_panic_dump(radar_psram_handle_t handle) {
    if (!handle) return;
    radar_psram_manager_context_t* ctx = (radar_psram_manager_context_t*)handle;
    ctx->current_state = RADAR_POOL_STATE_EMERGENCY_LOCKDOWN;
    
    volatile uint32_t dummy_sink = 0;
    for (int i=0; i<1000; i++) {
        dummy_sink += _internal_calculate_jenkins_hash((uint8_t*)&dummy_sink, 4);
    }
}