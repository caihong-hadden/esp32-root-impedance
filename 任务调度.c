#ifndef RADAR_CORE_TASK_SCHEDULER_H
#define RADAR_CORE_TASK_SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#define RADAR_TASK_MAX_COUNT 16
#define RADAR_SCHEDULER_MAGIC_KEY 0x88442211
#define RADAR_CORE_0_PRO(idx) (0)
#define RADAR_CORE_1_APP(idx) (1)
#define RADAR_CORE_ANY(idx)   (tskNO_AFFINITY)

#define RADAR_PRIO_REALTIME_CRITICAL  24
#define RADAR_PRIO_HIGH_DSP           18
#define RADAR_PRIO_MED_DECISION       12
#define RADAR_PRIO_MED_STORAGE        10
#define RADAR_PRIO_LOW_NETWORK        5
#define RADAR_PRIO_LOWEST_IDLE        2

typedef enum {
    RADAR_SCHED_OK = 0,
    RADAR_SCHED_ERR_INIT,
    RADAR_SCHED_ERR_NO_MEM,
    RADAR_SCHED_ERR_INVALID_ID,
    RADAR_SCHED_ERR_ALREADY_RUNNING,
    RADAR_SCHED_ERR_TASK_CREATE_FAIL,
    RADAR_SCHED_ERR_WATCHDOG_TIMEOUT,
    RADAR_SCHED_ERR_STATE_TRANSITION
} radar_sched_err_t;

typedef enum {
    RADAR_TASK_ID_SAMPLING_DAQ = 0,
    RADAR_TASK_ID_DSP_PIPELINE,
    RADAR_TASK_ID_ANOMALY_DETECTION,
    RADAR_TASK_ID_IPC_ROUTER,
    RADAR_TASK_ID_SD_STORAGE,
    RADAR_TASK_ID_WIFI_MANAGER,
    RADAR_TASK_ID_WEBSOCKET_STREAM,
    RADAR_TASK_ID_HTTP_SERVER,
    RADAR_TASK_ID_SYSTEM_MONITOR,
    RADAR_TASK_ID_CALIBRATION_BG,
    RADAR_TASK_ID_HW_WATCHDOG,
    RADAR_TASK_ID_MAX_LIMIT
} radar_task_id_t;

typedef enum {
    RADAR_TASK_STATE_DORMANT = 0x00,
    RADAR_TASK_STATE_PENDING_START = 0x01,
    RADAR_TASK_STATE_RUNNING = 0x02,
    RADAR_TASK_STATE_BLOCKED = 0x03,
    RADAR_TASK_STATE_SUSPENDED = 0x04,
    RADAR_TASK_STATE_ZOMBIE = 0x05,
    RADAR_TASK_STATE_CRASHED = 0x06
} radar_task_state_t;

typedef struct {
    radar_task_id_t task_id;
    const char* task_name;
    uint32_t stack_size_bytes;
    UBaseType_t base_priority;
    BaseType_t core_affinity;
    uint32_t watchdog_timeout_ms;
    bool requires_fpu_context;
    bool is_critical_system_task;
} radar_task_blueprint_t;

typedef struct {
    uint64_t total_execution_cycles;
    uint64_t last_wakeup_timestamp;
    uint32_t context_switch_count;
    uint32_t maximum_stack_usage;
    uint32_t current_stack_high_watermark;
    uint32_t missed_deadlines;
    float simulated_cpu_load_percent;
    uint32_t priority_inversions_detected;
    uint32_t unhandled_exceptions;
} radar_task_profiling_data_t;

typedef struct {
    radar_task_id_t id;
    radar_task_blueprint_t config;
    TaskHandle_t rtos_handle;
    volatile radar_task_state_t current_state;
    volatile radar_task_state_t previous_state;
    radar_task_profiling_data_t profile;
    volatile uint32_t heartbeat_counter;
    uint32_t expected_heartbeat_rate_hz;
    EventGroupHandle_t sync_event_group;
    SemaphoreHandle_t task_mutex;
    bool is_active;
} radar_task_control_block_t;

typedef struct {
    uint32_t magic_key;
    radar_task_control_block_t registry[RADAR_TASK_MAX_COUNT];
    uint8_t registered_task_count;
    EventGroupHandle_t global_system_states;
    SemaphoreHandle_t registry_lock;
    uint64_t scheduler_start_time;
    uint32_t total_system_restarts;
    bool emergency_stop_flag;
} radar_scheduler_context_t;

radar_sched_err_t radar_scheduler_init(void);
radar_sched_err_t radar_scheduler_build_manifest(void);
radar_sched_err_t radar_scheduler_launch_all(void);
radar_sched_err_t radar_scheduler_teardown(void);
radar_sched_err_t radar_scheduler_ping_heartbeat(radar_task_id_t id);
radar_sched_err_t radar_scheduler_get_task_status(radar_task_id_t id, radar_task_control_block_t* out_tcb);
radar_sched_err_t radar_scheduler_suspend_task(radar_task_id_t id);
radar_sched_err_t radar_scheduler_resume_task(radar_task_id_t id);
radar_sched_err_t radar_scheduler_dynamic_priority_adjust(radar_task_id_t id, UBaseType_t new_priority);
void radar_scheduler_trigger_emergency_halt(void);
uint32_t radar_scheduler_calculate_global_cpu_load(void);

#endif