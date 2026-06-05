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
} telemetry_controller_state_t;

void telemetry_update_line_sample(const line_sensor_sample_t *sample);
void telemetry_update_motion_cmd(const chassis_motion_cmd_t *cmd);
void telemetry_update_controller_state(const telemetry_controller_state_t *state);
esp_err_t telemetry_build_json(char *buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif
