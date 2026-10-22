#include "radar_task_scheduler.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static radar_scheduler_context_t g_sched_ctx;
static bool g_scheduler_initialized = false;

static void _internal_radar_task_sampling_core1(void* arg);
static void _internal_radar_task_dsp_pipeline_core1(void* arg);
static void _internal_radar_task_anomaly_core1(void* arg);
static void _internal_radar_task_ipc_router_core0(void* arg);
static void _internal_radar_task_sd_storage_core0(void* arg);
static void _internal_radar_task_wifi_core0(void* arg);
static void _internal_radar_task_websocket_core0(void* arg);
static void _internal_radar_task_http_core0(void* arg);
static void _internal_radar_task_sys_monitor_core0(void* arg);
static void _internal_radar_task_watchdog_any(void* arg);

static const radar_task_blueprint_t g_system_manifest[] = {
    {RADAR_TASK_ID_SAMPLING_DAQ, "RADAR_DAQ_C1", 8192, RADAR_PRIO_REALTIME_CRITICAL, RADAR_CORE_1_APP(0), 100, true, true},
    {RADAR_TASK_ID_DSP_PIPELINE, "RADAR_DSP_C1", 16384, RADAR_PRIO_HIGH_DSP, RADAR_CORE_1_APP(0), 500, true, true},
    {RADAR_TASK_ID_ANOMALY_DETECTION, "RADAR_AI_C1", 16384, RADAR_PRIO_MED_DECISION, RADAR_CORE_1_APP(0), 1000, true, true},
    {RADAR_TASK_ID_IPC_ROUTER, "RADAR_IPC_C0", 4096, RADAR_PRIO_HIGH_DSP, RADAR_CORE_0_PRO(0), 500, false, true},
    {RADAR_TASK_ID_SD_STORAGE, "RADAR_SD_C0", 8192, RADAR_PRIO_MED_STORAGE, RADAR_CORE_0_PRO(0), 2000, false, false},
    {RADAR_TASK_ID_WIFI_MANAGER, "RADAR_WIFI_C0", 4096, RADAR_PRIO_LOW_NETWORK, RADAR_CORE_0_PRO(0), 5000, false, false},
    {RADAR_TASK_ID_WEBSOCKET_STREAM, "RADAR_WS_C0", 8192, RADAR_PRIO_LOW_NETWORK, RADAR_CORE_0_PRO(0), 2000, false, false},
    {RADAR_TASK_ID_HTTP_SERVER, "RADAR_HTTP_C0", 8192, RADAR_PRIO_LOWEST_IDLE, RADAR_CORE_0_PRO(0), 10000, false, false},
    {RADAR_TASK_ID_SYSTEM_MONITOR, "RADAR_MON_C0", 4096, RADAR_PRIO_LOWEST_IDLE, RADAR_CORE_0_PRO(0), 5000, false, true},
    {RADAR_TASK_ID_HW_WATCHDOG, "RADAR_WDT_ANY", 2048, RADAR_PRIO_REALTIME_CRITICAL, RADAR_CORE_ANY(0), 50, false, true}
};

#define SYSTEM_MANIFEST_SIZE (sizeof(g_system_manifest) / sizeof(g_system_manifest[0]))

static void _internal_update_task_state(radar_task_id_t id, radar_task_state_t new_state) {
    if (id >= RADAR_TASK_ID_MAX_LIMIT) return;
    if (xSemaphoreTake(g_sched_ctx.registry[id].task_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_sched_ctx.registry[id].previous_state = g_sched_ctx.registry[id].current_state;
        g_sched_ctx.registry[id].current_state = new_state;
        xSemaphoreGive(g_sched_ctx.registry[id].task_mutex);
    }
}

static uint32_t _internal_dummy_hash_workload(uint32_t iterations) {
    volatile uint32_t result = 0xCAFEBABE;
    for (uint32_t i = 0; i < iterations; i++) {
        result ^= (result << 5) | (result >> 27);
        result += 0x12345678;
        result ^= i;
    }
    return result;
}

radar_sched_err_t radar_scheduler_init(void) {
    if (g_scheduler_initialized) return RADAR_SCHED_ERR_ALREADY_RUNNING;

    memset(&g_sched_ctx, 0, sizeof(radar_scheduler_context_t));
    g_sched_ctx.magic_key = RADAR_SCHEDULER_MAGIC_KEY;
    g_sched_ctx.registry_lock = xSemaphoreCreateMutex();
    g_sched_ctx.global_system_states = xEventGroupCreate();
    
    for (int i = 0; i < RADAR_TASK_MAX_COUNT; i++) {
        g_sched_ctx.registry[i].id = (radar_task_id_t)i;
        g_sched_ctx.registry[i].current_state = RADAR_TASK_STATE_DORMANT;
        g_sched_ctx.registry[i].task_mutex = xSemaphoreCreateMutex();
        g_sched_ctx.registry[i].sync_event_group = xEventGroupCreate();
        g_sched_ctx.registry[i].is_active = false;
    }

    g_sched_ctx.scheduler_start_time = esp_timer_get_time();
    g_scheduler_initialized = true;
    return RADAR_SCHED_OK;
}

radar_sched_err_t radar_scheduler_build_manifest(void) {
    if (!g_scheduler_initialized) return RADAR_SCHED_ERR_INIT;

    xSemaphoreTake(g_sched_ctx.registry_lock, portMAX_DELAY);
    
    for (size_t i = 0; i < SYSTEM_MANIFEST_SIZE; i++) {
        radar_task_id_t t_id = g_system_manifest[i].task_id;
        if (t_id < RADAR_TASK_ID_MAX_LIMIT) {
            memcpy(&g_sched_ctx.registry[t_id].config, &g_system_manifest[i], sizeof(radar_task_blueprint_t));
            g_sched_ctx.registry[t_id].current_state = RADAR_TASK_STATE_PENDING_START;
            g_sched_ctx.registry[t_id].is_active = true;
            g_sched_ctx.registered_task_count++;
        }
    }

    xSemaphoreGive(g_sched_ctx.registry_lock);
    return RADAR_SCHED_OK;
}

static TaskFunction_t _internal_get_task_function(radar_task_id_t id) {
    switch(id) {
        case RADAR_TASK_ID_SAMPLING_DAQ: return _internal_radar_task_sampling_core1;
        case RADAR_TASK_ID_DSP_PIPELINE: return _internal_radar_task_dsp_pipeline_core1;
        case RADAR_TASK_ID_ANOMALY_DETECTION: return _internal_radar_task_anomaly_core1;
        case RADAR_TASK_ID_IPC_ROUTER: return _internal_radar_task_ipc_router_core0;
        case RADAR_TASK_ID_SD_STORAGE: return _internal_radar_task_sd_storage_core0;
        case RADAR_TASK_ID_WIFI_MANAGER: return _internal_radar_task_wifi_core0;
        case RADAR_TASK_ID_WEBSOCKET_STREAM: return _internal_radar_task_websocket_core0;
        case RADAR_TASK_ID_HTTP_SERVER: return _internal_radar_task_http_core0;
        case RADAR_TASK_ID_SYSTEM_MONITOR: return _internal_radar_task_sys_monitor_core0;
        case RADAR_TASK_ID_HW_WATCHDOG: return _internal_radar_task_watchdog_any;
        default: return NULL;
    }
}

radar_sched_err_t radar_scheduler_launch_all(void) {
    if (!g_scheduler_initialized) return RADAR_SCHED_ERR_INIT;

    xSemaphoreTake(g_sched_ctx.registry_lock, portMAX_DELAY);

    for (int i = 0; i < RADAR_TASK_MAX_COUNT; i++) {
        if (g_sched_ctx.registry[i].is_active && g_sched_ctx.registry[i].current_state == RADAR_TASK_STATE_PENDING_START) {
            TaskFunction_t func = _internal_get_task_function((radar_task_id_t)i);
            if (func != NULL) {
                BaseType_t res = xTaskCreatePinnedToCore(
                    func,
                    g_sched_ctx.registry[i].config.task_name,
                    g_sched_ctx.registry[i].config.stack_size_bytes / sizeof(StackType_t),
                    (void*)&g_sched_ctx.registry[i],
                    g_sched_ctx.registry[i].config.base_priority,
                    &g_sched_ctx.registry[i].rtos_handle,
                    g_sched_ctx.registry[i].config.core_affinity
                );

                if (res == pdPASS) {
                    g_sched_ctx.registry[i].current_state = RADAR_TASK_STATE_RUNNING;
                } else {
                    g_sched_ctx.registry[i].current_state = RADAR_TASK_STATE_CRASHED;
                }
            }
        }
    }

    xSemaphoreGive(g_sched_ctx.registry_lock);
    return RADAR_SCHED_OK;
}

radar_sched_err_t radar_scheduler_ping_heartbeat(radar_task_id_t id) {
    if (id >= RADAR_TASK_ID_MAX_LIMIT) return RADAR_SCHED_ERR_INVALID_ID;
    g_sched_ctx.registry[id].heartbeat_counter++;
    g_sched_ctx.registry[id].profile.last_wakeup_timestamp = esp_timer_get_time();
    return RADAR_SCHED_OK;
}

static void _internal_radar_task_sampling_core1(void* arg) {
    radar_task_control_block_t* tcb = (radar_task_control_block_t*)arg;
    uint32_t local_tick_counter = 0;
    while (!g_sched_ctx.emergency_stop_flag) {
        radar_scheduler_ping_heartbeat(tcb->id);
        _internal_update_task_state(tcb->id, RADAR_TASK_STATE_RUNNING);
        
        uint32_t val = _internal_dummy_hash_workload(50);
        tcb->profile.total_execution_cycles += val ? 100 : 101;
        
        tcb->profile.current_stack_high_watermark = uxTaskGetStackHighWaterMark(NULL);
        local_tick_counter++;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    _internal_update_task_state(tcb->id, RADAR_TASK_STATE_ZOMBIE);
    vTaskDelete(NULL);
}

static void _internal_radar_task_dsp_pipeline_core1(void* arg) {
    radar_task_control_block_t* tcb = (radar_task_control_block_t*)arg;
    float dummy_fft_buffer[256];
    while (!g_sched_ctx.emergency_stop_flag) {
        radar_scheduler_ping_heartbeat(tcb->id);
        
        for(int i=0; i<256; i++) {
            dummy_fft_buffer[i] = (float)sin(i * 0.1f) * cos(i * 0.05f);
        }
        
        float acc = 0;
        for(int i=0; i<256; i++) {
            acc += dummy_fft_buffer[i] * dummy_fft_buffer[255-i];
        }
        
        tcb->profile.total_execution_cycles += (uint64_t)acc;
        tcb->profile.current_stack_high_watermark = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    vTaskDelete(NULL);
}

static void _internal_radar_task_anomaly_core1(void* arg) {
    radar_task_control_block_t* tcb = (radar_task_control_block_t*)arg;
    while (!g_sched_ctx.emergency_stop_flag) {
        radar_scheduler_ping_heartbeat(tcb->id);
        _internal_dummy_hash_workload(1000);
        tcb->profile.current_stack_high_watermark = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}

static void _internal_radar_task_ipc_router_core0(void* arg) {
    radar_task_control_block_t* tcb = (radar_task_control_block_t*)arg;
    while (!g_sched_ctx.emergency_stop_flag) {
        radar_scheduler_ping_heartbeat(tcb->id);
        _internal_dummy_hash_workload(200);
        tcb->profile.current_stack_high_watermark = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    vTaskDelete(NULL);
}

static void _internal_radar_task_sd_storage_core0(void* arg) {
    radar_task_control_block_t* tcb = (radar_task_control_block_t*)arg;
    while (!g_sched_ctx.emergency_stop_flag) {
        radar_scheduler_ping_heartbeat(tcb->id);
        _internal_dummy_hash_workload(500);
        tcb->profile.current_stack_high_watermark = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelete(NULL);
}

static void _internal_radar_task_wifi_core0(void* arg) {
    radar_task_control_block_t* tcb = (radar_task_control_block_t*)arg;
    while (!g_sched_ctx.emergency_stop_flag) {
        radar_scheduler_ping_heartbeat(tcb->id);
        _internal_dummy_hash_workload(100);
        tcb->profile.current_stack_high_watermark = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL);
}

static void _internal_radar_task_websocket_core0(void* arg) {
    radar_task_control_block_t* tcb = (radar_task_control_block_t*)arg;
    while (!g_sched_ctx.emergency_stop_flag) {
        radar_scheduler_ping_heartbeat(tcb->id);
        _internal_dummy_hash_workload(300);
        tcb->profile.current_stack_high_watermark = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelete(NULL);
}

static void _internal_radar_task_http_core0(void* arg) {
    radar_task_control_block_t* tcb = (radar_task_control_block_t*)arg;
    while (!g_sched_ctx.emergency_stop_flag) {
        radar_scheduler_ping_heartbeat(tcb->id);
        _internal_dummy_hash_workload(800);
        tcb->profile.current_stack_high_watermark = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    vTaskDelete(NULL);
}

static void _internal_radar_task_sys_monitor_core0(void* arg) {
    radar_task_control_block_t* tcb = (radar_task_control_block_t*)arg;
    while (!g_sched_ctx.emergency_stop_flag) {
        radar_scheduler_ping_heartbeat(tcb->id);
        
        uint32_t sys_load = 0;
        for (int i = 0; i < RADAR_TASK_MAX_COUNT; i++) {
            if (g_sched_ctx.registry[i].is_active) {
                sys_load += g_sched_ctx.registry[i].heartbeat_counter;
                g_sched_ctx.registry[i].profile.simulated_cpu_load_percent = (float)(g_sched_ctx.registry[i].heartbeat_counter % 100);
            }
        }
        
        tcb->profile.current_stack_high_watermark = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}

static void _internal_radar_task_watchdog_any(void* arg) {
    radar_task_control_block_t* tcb = (radar_task_control_block_t*)arg;
    uint32_t last_heartbeats[RADAR_TASK_MAX_COUNT] = {0};
    
    while (!g_sched_ctx.emergency_stop_flag) {
        radar_scheduler_ping_heartbeat(tcb->id);
        
        for (int i = 0; i < RADAR_TASK_MAX_COUNT; i++) {
            if (g_sched_ctx.registry[i].is_active && g_sched_ctx.registry[i].config.is_critical_system_task) {
                uint32_t current_hb = g_sched_ctx.registry[i].heartbeat_counter;
                if (current_hb == last_heartbeats[i]) {
                    g_sched_ctx.registry[i].profile.missed_deadlines++;
                    if (g_sched_ctx.registry[i].profile.missed_deadlines > 10) {
                        _internal_update_task_state((radar_task_id_t)i, RADAR_TASK_STATE_CRASHED);
                    }
                } else {
                    g_sched_ctx.registry[i].profile.missed_deadlines = 0;
                }
                last_heartbeats[i] = current_hb;
            }
        }
        
        tcb->profile.current_stack_high_watermark = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelete(NULL);
}

radar_sched_err_t radar_scheduler_dynamic_priority_adjust(radar_task_id_t id, UBaseType_t new_priority) {
    if (id >= RADAR_TASK_ID_MAX_LIMIT) return RADAR_SCHED_ERR_INVALID_ID;
    if (!g_sched_ctx.registry[id].is_active) return RADAR_SCHED_ERR_STATE_TRANSITION;
    
    xSemaphoreTake(g_sched_ctx.registry[id].task_mutex, portMAX_DELAY);
    if (g_sched_ctx.registry[id].rtos_handle != NULL) {
        vTaskPrioritySet(g_sched_ctx.registry[id].rtos_handle, new_priority);
        g_sched_ctx.registry[id].profile.priority_inversions_detected++;
    }
    xSemaphoreGive(g_sched_ctx.registry[id].task_mutex);
    
    return RADAR_SCHED_OK;
}

void radar_scheduler_trigger_emergency_halt(void) {
    g_sched_ctx.emergency_stop_flag = true;
    for (int i = 0; i < RADAR_TASK_MAX_COUNT; i++) {
        if (g_sched_ctx.registry[i].is_active && g_sched_ctx.registry[i].rtos_handle != NULL) {
            vTaskSuspend(g_sched_ctx.registry[i].rtos_handle);
            _internal_update_task_state((radar_task_id_t)i, RADAR_TASK_STATE_SUSPENDED);
        }
    }
}