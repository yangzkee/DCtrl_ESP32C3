#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "chassis_uart.h"
#include "esp_err.h"
#include "line_sensor_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool lost_line;
    uint8_t line_quality;
    uint8_t active_sensor_count;
    float pid_output_mdeg_s;
    char line_phase[24];
    char recovery_relation[24];
    char recovery_stage[24];
    int32_t recovery_angle_mdeg;
    int32_t recovery_target_mdeg;
    int32_t recovery_direction_mdeg;
    uint8_t recovery_segment_index;
    uint32_t recovery_elapsed_ms;
    uint32_t recovery_target_ms;
} telemetry_controller_state_t;

typedef struct {
    uint32_t success_count;
    uint32_t error_count;
    uint64_t bytes_received;
    uint64_t bytes_forwarded;
    uint32_t session_id;
    bool session_active;
    uint32_t session_success_count;
    uint32_t session_error_count;
    uint64_t session_bytes_received;
    uint64_t session_bytes_forwarded;
    uint32_t session_started_ms;
    uint32_t session_ended_ms;
    uint32_t session_idle_gap_ms;
    uint32_t last_rx_len;
    uint32_t last_tx_bytes;
    uint32_t last_success_ms;
    uint32_t last_gap_ms;
    uint32_t max_gap_ms;
    int32_t last_error;
    char last_error_code[12];
} telemetry_remote_bridge_state_t;

void telemetry_update_line_sample(const line_sensor_sample_t *sample);
void telemetry_update_motion_cmd(const chassis_motion_cmd_t *cmd);
void telemetry_update_controller_state(const telemetry_controller_state_t *state);
void telemetry_record_remote_bridge_success(size_t rx_len, uint32_t tx_bytes);
void telemetry_record_remote_bridge_error(const char *code, size_t rx_len, esp_err_t err);
void telemetry_end_remote_bridge_session(void);
esp_err_t telemetry_build_json(char *buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif
