#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "chassis_uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    VEHICLE_MOTION_BOOT_INIT = 0,
    VEHICLE_MOTION_SAFE_IDLE,
    VEHICLE_MOTION_PARAM_TUNING,
    VEHICLE_MOTION_MANUAL_TEST,
    VEHICLE_MOTION_AUTO_ARMED,
    VEHICLE_MOTION_AUTO_RUNNING,
    VEHICLE_MOTION_OTA_UPDATE,
    VEHICLE_MOTION_FAULT,
} vehicle_motion_state_t;

typedef enum {
    VEHICLE_DEBUG_VIEW_ONLY = 0,
    VEHICLE_DEBUG_TUNING_ACTIVE,
    VEHICLE_DEBUG_OTA_ACTIVE,
} vehicle_debug_session_t;

typedef enum {
    VEHICLE_FAULT_NONE = 0,
    VEHICLE_FAULT_SENSOR_TIMEOUT,
    VEHICLE_FAULT_LINE_LOST,
    VEHICLE_FAULT_CHASSIS_SEND_FAILED,
    VEHICLE_FAULT_INVALID_COMMAND,
    VEHICLE_FAULT_OTA_FAILED,
} vehicle_fault_reason_t;

typedef struct {
    vehicle_motion_state_t motion_state;
    vehicle_debug_session_t debug_session;
    vehicle_fault_reason_t fault_reason;
    chassis_motion_cmd_t manual_cmd;
    uint32_t manual_deadline_ms;
    uint32_t state_entered_ms;
    uint32_t last_command_ms;
} vehicle_state_snapshot_t;

void vehicle_state_init(void);
void vehicle_state_finish_boot(void);
esp_err_t vehicle_state_enter_tuning(void);
esp_err_t vehicle_state_exit_tuning(void);
esp_err_t vehicle_state_arm_auto(void);
esp_err_t vehicle_state_start_auto(void);
void vehicle_state_stop(void);
esp_err_t vehicle_state_enter_ota_update(void);
void vehicle_state_finish_ota_update(void);
bool vehicle_state_is_ota_update(void);
esp_err_t vehicle_state_clear_fault(void);
void vehicle_state_enter_fault(vehicle_fault_reason_t reason);
esp_err_t vehicle_state_start_manual_test(const chassis_motion_cmd_t *cmd, uint32_t duration_ms);
bool vehicle_state_finish_manual_if_expired(uint32_t now_ms);
void vehicle_state_get_snapshot(vehicle_state_snapshot_t *snapshot);
uint32_t vehicle_state_now_ms(void);

const char *vehicle_motion_state_name(vehicle_motion_state_t state);
const char *vehicle_debug_session_name(vehicle_debug_session_t session);
const char *vehicle_fault_reason_name(vehicle_fault_reason_t reason);

#ifdef __cplusplus
}
#endif
