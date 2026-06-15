#include "vehicle_state.h"

#include <string.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static vehicle_state_snapshot_t s_state;

uint32_t vehicle_state_now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void set_state_locked(vehicle_motion_state_t motion_state,
                             vehicle_debug_session_t debug_session,
                             vehicle_fault_reason_t fault_reason)
{
    s_state.motion_state = motion_state;
    s_state.debug_session = debug_session;
    s_state.fault_reason = fault_reason;
    s_state.state_entered_ms = vehicle_state_now_ms();
    s_state.last_command_ms = s_state.state_entered_ms;
}

void vehicle_state_init(void)
{
    taskENTER_CRITICAL(&s_lock);
    memset(&s_state, 0, sizeof(s_state));
    set_state_locked(VEHICLE_MOTION_BOOT_INIT, VEHICLE_DEBUG_VIEW_ONLY, VEHICLE_FAULT_NONE);
    taskEXIT_CRITICAL(&s_lock);
}

void vehicle_state_finish_boot(void)
{
    taskENTER_CRITICAL(&s_lock);
    if (s_state.motion_state == VEHICLE_MOTION_BOOT_INIT) {
        set_state_locked(VEHICLE_MOTION_SAFE_IDLE, VEHICLE_DEBUG_VIEW_ONLY, VEHICLE_FAULT_NONE);
    }
    taskEXIT_CRITICAL(&s_lock);
}

esp_err_t vehicle_state_enter_remote_bridge(void)
{
    taskENTER_CRITICAL(&s_lock);
    const bool can_enter =
        (s_state.motion_state == VEHICLE_MOTION_SAFE_IDLE ||
         s_state.motion_state == VEHICLE_MOTION_REMOTE_BRIDGE) &&
        (s_state.debug_session == VEHICLE_DEBUG_VIEW_ONLY ||
         s_state.debug_session == VEHICLE_DEBUG_REMOTE_ACTIVE);
    if (!can_enter) {
        taskEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    set_state_locked(VEHICLE_MOTION_REMOTE_BRIDGE, VEHICLE_DEBUG_REMOTE_ACTIVE, VEHICLE_FAULT_NONE);
    taskEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

esp_err_t vehicle_state_exit_remote_bridge(void)
{
    taskENTER_CRITICAL(&s_lock);
    if (s_state.motion_state == VEHICLE_MOTION_SAFE_IDLE &&
        s_state.debug_session == VEHICLE_DEBUG_VIEW_ONLY) {
        taskEXIT_CRITICAL(&s_lock);
        return ESP_OK;
    }
    if (s_state.motion_state != VEHICLE_MOTION_REMOTE_BRIDGE ||
        s_state.debug_session != VEHICLE_DEBUG_REMOTE_ACTIVE) {
        taskEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    set_state_locked(VEHICLE_MOTION_SAFE_IDLE, VEHICLE_DEBUG_VIEW_ONLY, VEHICLE_FAULT_NONE);
    taskEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

bool vehicle_state_is_remote_bridge(void)
{
    bool active = false;
    taskENTER_CRITICAL(&s_lock);
    active = s_state.motion_state == VEHICLE_MOTION_REMOTE_BRIDGE &&
             s_state.debug_session == VEHICLE_DEBUG_REMOTE_ACTIVE;
    taskEXIT_CRITICAL(&s_lock);
    return active;
}

esp_err_t vehicle_state_enter_tuning(void)
{
    taskENTER_CRITICAL(&s_lock);
    if (s_state.motion_state != VEHICLE_MOTION_SAFE_IDLE &&
        s_state.motion_state != VEHICLE_MOTION_PARAM_TUNING) {
        taskEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    s_state.manual_cmd = (chassis_motion_cmd_t){0};
    s_state.manual_deadline_ms = 0;
    set_state_locked(VEHICLE_MOTION_PARAM_TUNING, VEHICLE_DEBUG_TUNING_ACTIVE, VEHICLE_FAULT_NONE);
    taskEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

esp_err_t vehicle_state_exit_tuning(void)
{
    taskENTER_CRITICAL(&s_lock);
    if (s_state.debug_session != VEHICLE_DEBUG_TUNING_ACTIVE) {
        taskEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    s_state.manual_cmd = (chassis_motion_cmd_t){0};
    s_state.manual_deadline_ms = 0;
    set_state_locked(VEHICLE_MOTION_SAFE_IDLE, VEHICLE_DEBUG_VIEW_ONLY, VEHICLE_FAULT_NONE);
    taskEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

esp_err_t vehicle_state_arm_auto(void)
{
    taskENTER_CRITICAL(&s_lock);
    if (s_state.motion_state != VEHICLE_MOTION_SAFE_IDLE || s_state.debug_session != VEHICLE_DEBUG_VIEW_ONLY) {
        taskEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    set_state_locked(VEHICLE_MOTION_AUTO_ARMED, VEHICLE_DEBUG_VIEW_ONLY, VEHICLE_FAULT_NONE);
    taskEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

esp_err_t vehicle_state_start_auto(void)
{
    taskENTER_CRITICAL(&s_lock);
    if (s_state.motion_state != VEHICLE_MOTION_AUTO_ARMED) {
        taskEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    set_state_locked(VEHICLE_MOTION_AUTO_RUNNING, VEHICLE_DEBUG_VIEW_ONLY, VEHICLE_FAULT_NONE);
    taskEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

void vehicle_state_stop(void)
{
    taskENTER_CRITICAL(&s_lock);
    s_state.manual_cmd = (chassis_motion_cmd_t){0};
    s_state.manual_deadline_ms = 0;
    set_state_locked(VEHICLE_MOTION_SAFE_IDLE, VEHICLE_DEBUG_VIEW_ONLY, VEHICLE_FAULT_NONE);
    taskEXIT_CRITICAL(&s_lock);
}

esp_err_t vehicle_state_clear_fault(void)
{
    taskENTER_CRITICAL(&s_lock);
    if (s_state.motion_state != VEHICLE_MOTION_FAULT) {
        taskEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    s_state.manual_cmd = (chassis_motion_cmd_t){0};
    s_state.manual_deadline_ms = 0;
    set_state_locked(VEHICLE_MOTION_SAFE_IDLE, VEHICLE_DEBUG_VIEW_ONLY, VEHICLE_FAULT_NONE);
    taskEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

void vehicle_state_enter_fault(vehicle_fault_reason_t reason)
{
    taskENTER_CRITICAL(&s_lock);
    s_state.manual_cmd = (chassis_motion_cmd_t){0};
    s_state.manual_deadline_ms = 0;
    set_state_locked(VEHICLE_MOTION_FAULT,
                     VEHICLE_DEBUG_VIEW_ONLY,
                     reason == VEHICLE_FAULT_NONE ? VEHICLE_FAULT_INVALID_COMMAND : reason);
    taskEXIT_CRITICAL(&s_lock);
}

esp_err_t vehicle_state_start_manual_test(const chassis_motion_cmd_t *cmd, uint32_t duration_ms)
{
    if (cmd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    taskENTER_CRITICAL(&s_lock);
    if (s_state.motion_state != VEHICLE_MOTION_PARAM_TUNING ||
        s_state.debug_session != VEHICLE_DEBUG_TUNING_ACTIVE) {
        taskEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_STATE;
    }

    s_state.manual_cmd = *cmd;
    if (cmd->vx_mm_s == 0 && cmd->vy_mm_s == 0 && cmd->yaw_mdeg == 0) {
        s_state.manual_deadline_ms = 0;
        set_state_locked(VEHICLE_MOTION_PARAM_TUNING, VEHICLE_DEBUG_TUNING_ACTIVE, VEHICLE_FAULT_NONE);
    } else {
        s_state.manual_deadline_ms = vehicle_state_now_ms() + duration_ms;
        set_state_locked(VEHICLE_MOTION_MANUAL_TEST, VEHICLE_DEBUG_TUNING_ACTIVE, VEHICLE_FAULT_NONE);
    }
    taskEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

bool vehicle_state_finish_manual_if_expired(uint32_t now_ms)
{
    bool expired = false;

    taskENTER_CRITICAL(&s_lock);
    if (s_state.motion_state == VEHICLE_MOTION_MANUAL_TEST &&
        s_state.manual_deadline_ms != 0 &&
        (int32_t)(now_ms - s_state.manual_deadline_ms) >= 0) {
        s_state.manual_cmd = (chassis_motion_cmd_t){0};
        s_state.manual_deadline_ms = 0;
        set_state_locked(VEHICLE_MOTION_PARAM_TUNING, VEHICLE_DEBUG_TUNING_ACTIVE, VEHICLE_FAULT_NONE);
        expired = true;
    }
    taskEXIT_CRITICAL(&s_lock);

    return expired;
}

void vehicle_state_get_snapshot(vehicle_state_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    taskENTER_CRITICAL(&s_lock);
    *snapshot = s_state;
    taskEXIT_CRITICAL(&s_lock);
}

const char *vehicle_motion_state_name(vehicle_motion_state_t state)
{
    switch (state) {
    case VEHICLE_MOTION_BOOT_INIT:
        return "BOOT_INIT";
    case VEHICLE_MOTION_SAFE_IDLE:
        return "SAFE_IDLE";
    case VEHICLE_MOTION_REMOTE_BRIDGE:
        return "REMOTE_BRIDGE";
    case VEHICLE_MOTION_PARAM_TUNING:
        return "PARAM_TUNING";
    case VEHICLE_MOTION_MANUAL_TEST:
        return "MANUAL_TEST";
    case VEHICLE_MOTION_AUTO_ARMED:
        return "AUTO_ARMED";
    case VEHICLE_MOTION_AUTO_RUNNING:
        return "AUTO_RUNNING";
    case VEHICLE_MOTION_FAULT:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}

const char *vehicle_debug_session_name(vehicle_debug_session_t session)
{
    switch (session) {
    case VEHICLE_DEBUG_VIEW_ONLY:
        return "VIEW_ONLY";
    case VEHICLE_DEBUG_REMOTE_ACTIVE:
        return "REMOTE_ACTIVE";
    case VEHICLE_DEBUG_TUNING_ACTIVE:
        return "TUNING_ACTIVE";
    default:
        return "UNKNOWN";
    }
}

const char *vehicle_fault_reason_name(vehicle_fault_reason_t reason)
{
    switch (reason) {
    case VEHICLE_FAULT_NONE:
        return "NONE";
    case VEHICLE_FAULT_SENSOR_TIMEOUT:
        return "SENSOR_TIMEOUT";
    case VEHICLE_FAULT_LINE_LOST:
        return "LINE_LOST";
    case VEHICLE_FAULT_CHASSIS_SEND_FAILED:
        return "CHASSIS_SEND_FAILED";
    case VEHICLE_FAULT_INVALID_COMMAND:
        return "INVALID_COMMAND";
    default:
        return "UNKNOWN";
    }
}
