#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "board_profile.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t bits;
    int8_t offset;
    uint32_t updated_ms;
} line_sensor_sample_t;

typedef struct {
    bool ready;
    uint32_t request_count;
    uint32_t frame_count;
    uint32_t digital_frame_count;
    uint32_t invalid_frame_count;
    uint32_t timeout_count;
    int last_error;
    char last_frame[96];
} line_sensor_uart_status_t;

esp_err_t line_sensor_uart_init(const board_uart_config_t *config);
esp_err_t line_sensor_uart_read_sample(line_sensor_sample_t *sample, uint32_t timeout_ms);
void line_sensor_uart_get_status(line_sensor_uart_status_t *status);
int8_t line_sensor_calculate_offset(uint8_t bits);

#ifdef __cplusplus
}
#endif
