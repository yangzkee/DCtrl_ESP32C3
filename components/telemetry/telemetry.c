#include "telemetry.h"

#include <stdio.h>
#include <string.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "param_store.h"
#include "vehicle_state.h"

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static line_sensor_sample_t s_line_sample;
static chassis_motion_cmd_t s_motion_cmd;
static telemetry_controller_state_t s_controller_state;

static void sanitize_json_string(char *text)
{
    if (text == NULL) {
        return;
    }

    for (size_t i = 0; text[i] != '\0'; ++i) {
        unsigned char c = (unsigned char)text[i];
        if (c < 0x20 || c == '"' || c == '\\') {
            text[i] = '?';
        }
    }
}

void telemetry_update_line_sample(const line_sensor_sample_t *sample)
{
    if (sample == NULL) {
        return;
    }

    taskENTER_CRITICAL(&s_lock);
    s_line_sample = *sample;
    taskEXIT_CRITICAL(&s_lock);
}

void telemetry_update_motion_cmd(const chassis_motion_cmd_t *cmd)
{
    if (cmd == NULL) {
        return;
    }

    taskENTER_CRITICAL(&s_lock);
    s_motion_cmd = *cmd;
    taskEXIT_CRITICAL(&s_lock);
}

void telemetry_update_controller_state(const telemetry_controller_state_t *state)
{
    if (state == NULL) {
        return;
    }

    taskENTER_CRITICAL(&s_lock);
    s_controller_state = *state;
    taskEXIT_CRITICAL(&s_lock);
}

esp_err_t telemetry_build_json(char *buffer, size_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    line_sensor_sample_t line_sample;
    chassis_motion_cmd_t motion_cmd;
    telemetry_controller_state_t controller_state;
    line_sensor_uart_status_t line_status = {0};
    vehicle_state_snapshot_t vehicle = {0};

    taskENTER_CRITICAL(&s_lock);
    line_sample = s_line_sample;
    motion_cmd = s_motion_cmd;
    controller_state = s_controller_state;
    taskEXIT_CRITICAL(&s_lock);
    line_sensor_uart_get_status(&line_status);
    vehicle_state_get_snapshot(&vehicle);

    char last_frame[sizeof(line_status.last_frame)] = {0};
    strlcpy(last_frame, line_status.last_frame, sizeof(last_frame));
    sanitize_json_string(last_frame);
    char line_phase[sizeof(controller_state.line_phase)] = {0};
    strlcpy(line_phase, controller_state.line_phase, sizeof(line_phase));
    sanitize_json_string(line_phase);
    char recovery_relation[sizeof(controller_state.recovery_relation)] = {0};
    strlcpy(recovery_relation, controller_state.recovery_relation, sizeof(recovery_relation));
    sanitize_json_string(recovery_relation);
    char recovery_stage[sizeof(controller_state.recovery_stage)] = {0};
    strlcpy(recovery_stage, controller_state.recovery_stage, sizeof(recovery_stage));
    sanitize_json_string(recovery_stage);

    int written = snprintf(buffer,
                           buffer_size,
                           "{\"type\":\"telemetry\",\"uptime_ms\":%llu,"
                           "\"param_version\":%lu,"
                           "\"vehicle\":{\"motion_state\":\"%s\",\"debug_session\":\"%s\","
                           "\"fault\":\"%s\",\"state_entered_ms\":%lu,\"last_command_ms\":%lu},"
                           "\"line\":{\"bits\":%u,\"offset\":%d,\"updated_ms\":%lu,"
                           "\"active_count\":%u,\"quality\":%u},"
                           "\"line_uart\":{\"ready\":%s,\"requests\":%lu,\"frames\":%lu,"
                           "\"digital_frames\":%lu,\"invalid_frames\":%lu,\"timeouts\":%lu,"
                           "\"last_error\":%d,\"last_frame\":\"%s\"},"
                           "\"motion\":{\"linear_mm_s\":%ld,\"angular_mdeg_s\":%ld},"
                           "\"controller\":{\"line_phase\":\"%s\",\"lost_line\":%s,\"line_quality\":%u,"
                           "\"active_sensor_count\":%u,\"pid_output_mdeg_s\":%.2f,"
                           "\"recovery\":{\"relation\":\"%s\",\"stage\":\"%s\",\"angle_mdeg\":%ld,"
                           "\"target_mdeg\":%ld,\"direction_mdeg\":%ld}}}",
                           (unsigned long long)(esp_timer_get_time() / 1000ULL),
                           (unsigned long)param_store_version(),
                           vehicle_motion_state_name(vehicle.motion_state),
                           vehicle_debug_session_name(vehicle.debug_session),
                           vehicle_fault_reason_name(vehicle.fault_reason),
                           (unsigned long)vehicle.state_entered_ms,
                           (unsigned long)vehicle.last_command_ms,
                           line_sample.bits,
                           line_sample.offset,
                           (unsigned long)line_sample.updated_ms,
                           controller_state.active_sensor_count,
                           controller_state.line_quality,
                           line_status.ready ? "true" : "false",
                           (unsigned long)line_status.request_count,
                           (unsigned long)line_status.frame_count,
                           (unsigned long)line_status.digital_frame_count,
                           (unsigned long)line_status.invalid_frame_count,
                           (unsigned long)line_status.timeout_count,
                           line_status.last_error,
                           last_frame,
                           (long)motion_cmd.linear_mm_s,
                           (long)motion_cmd.angular_mdeg_s,
                           line_phase[0] == '\0' ? "UNKNOWN" : line_phase,
                           controller_state.lost_line ? "true" : "false",
                           controller_state.line_quality,
                           controller_state.active_sensor_count,
                           controller_state.pid_output_mdeg_s,
                           recovery_relation[0] == '\0' ? "NONE" : recovery_relation,
                           recovery_stage[0] == '\0' ? "NONE" : recovery_stage,
                           (long)controller_state.recovery_angle_mdeg,
                           (long)controller_state.recovery_target_mdeg,
                           (long)controller_state.recovery_direction_mdeg);

    return written > 0 && written < (int)buffer_size ? ESP_OK : ESP_ERR_INVALID_SIZE;
}
