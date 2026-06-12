#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "board_profile.h"
#include "chassis_motion_contracts.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char command[16];
    bool ready;
    int tx_pin;
    int rx_pin;
    int baud_rate;
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t frames;
    uint32_t checksum_ok;
    uint32_t checksum_bad;
    uint8_t last_target;
    uint8_t last_source;
    uint8_t last_a;
    uint8_t last_b;
    uint8_t last_len;
    bool last_checksum_ok;
    char last_payload_hex[97];
    char last_error[24];
} chassis_diag_result_t;

esp_err_t chassis_uart_init(const board_uart_config_t *config);
esp_err_t chassis_uart_send_motion(const chassis_motion_cmd_t *cmd);
esp_err_t chassis_uart_write_raw_frame(const uint8_t *frame, size_t frame_len);
esp_err_t chassis_uart_stop(void);
float chassis_uart_linear_protocol_per_real(void);
esp_err_t chassis_uart_diag_command(const char *command, chassis_diag_result_t *result);
esp_err_t chassis_uart_diag_format_text(const chassis_diag_result_t *result, char *buffer, size_t buffer_size);
esp_err_t chassis_uart_diag_command_text(const char *command, char *buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif
