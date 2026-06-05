#include "chassis_uart.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define DFLINK_FRAME_HEAD 0xDF
#define DFLINK_FRAME_TAIL 0xFD
#define DFLINK_TARGET_ID 0x01
#define DFLINK_SOURCE_ID 0x97
#define DFLINK_A_MOTION 0x02
#define DFLINK_B_MOTION_VELOCITY 0x62
#define DFLINK_MOTION_VELOCITY_PAYLOAD_BYTES 12
#define DFLINK_PI 3.14159265358979323846f
#define DFLINK_MAX_FRAME_BYTES 160
#define DFLINK_MAX_PAYLOAD_BYTES 96
#define DFLINK_DIAG_READ_CHUNK_BYTES 64
#define CHASSIS_LINEAR_PROTOCOL_PER_REAL 10.0f

static const char *TAG = "chassis_uart";
static board_uart_config_t s_config;
static bool s_ready;
static SemaphoreHandle_t s_uart_mutex;

static int32_t dflink_f32_fixed(float value)
{
    float scaled = value * 10000.0f;
    if (scaled >= 0.0f) {
        return (int32_t)(scaled + 0.5f);
    }
    return (int32_t)(scaled - 0.5f);
}

static void write_i32_le(uint8_t *buffer, int32_t value)
{
    buffer[0] = (uint8_t)(value & 0xFF);
    buffer[1] = (uint8_t)((value >> 8) & 0xFF);
    buffer[2] = (uint8_t)((value >> 16) & 0xFF);
    buffer[3] = (uint8_t)((value >> 24) & 0xFF);
}

static void write_dflink_f32(uint8_t *buffer, float value)
{
    write_i32_le(buffer, dflink_f32_fixed(value));
}

static float linear_cmd_to_protocol_vx(int32_t linear_mm_s)
{
    return (float)linear_mm_s / 1000.0f;
}

static float angular_cmd_to_protocol_vz(int32_t angular_mdeg_s)
{
    return ((float)angular_mdeg_s / 1000.0f) * (DFLINK_PI / 180.0f);
}

static uint16_t checksum16(const uint8_t *buffer, size_t len)
{
    uint16_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum = (uint16_t)(sum + buffer[i]);
    }
    return sum;
}

static void reset_diag_result(const char *command, chassis_diag_result_t *result)
{
    memset(result, 0, sizeof(*result));
    snprintf(result->command, sizeof(result->command), "%s", command == NULL ? "status" : command);
    result->ready = s_ready;
    result->tx_pin = s_config.tx_pin;
    result->rx_pin = s_config.rx_pin;
    result->baud_rate = s_config.baud_rate;
    snprintf(result->last_error, sizeof(result->last_error), "%s", s_ready ? "none" : "not_ready");
}

static esp_err_t take_uart_mutex(TickType_t timeout)
{
    if (s_uart_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return xSemaphoreTake(s_uart_mutex, timeout) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

static void give_uart_mutex(void)
{
    if (s_uart_mutex != NULL) {
        xSemaphoreGive(s_uart_mutex);
    }
}

static size_t build_dflink_frame(uint8_t a, uint8_t b, const uint8_t *payload, uint8_t payload_len, uint8_t *frame, size_t frame_size)
{
    if (frame_size < (size_t)payload_len + 9) {
        return 0;
    }

    size_t index = 0;
    frame[index++] = DFLINK_FRAME_HEAD;
    frame[index++] = DFLINK_TARGET_ID;
    frame[index++] = DFLINK_SOURCE_ID;
    frame[index++] = a;
    frame[index++] = b;
    frame[index++] = payload_len;
    if (payload_len > 0 && payload != NULL) {
        memcpy(&frame[index], payload, payload_len);
        index += payload_len;
    }
    frame[index++] = DFLINK_FRAME_TAIL;

    uint16_t sum = checksum16(frame, index);
    frame[index++] = (uint8_t)(sum & 0xFF);
    frame[index++] = (uint8_t)((sum >> 8) & 0xFF);
    return index;
}

static esp_err_t write_bytes_locked(const uint8_t *frame, size_t frame_len)
{
    int written = uart_write_bytes(s_config.port, frame, frame_len);
    if (written != (int)frame_len) {
        return ESP_FAIL;
    }
    return uart_wait_tx_done(s_config.port, pdMS_TO_TICKS(100));
}

static esp_err_t send_dflink_command_locked(uint8_t a,
                                            uint8_t b,
                                            const uint8_t *payload,
                                            uint8_t payload_len,
                                            chassis_diag_result_t *result)
{
    uint8_t frame[DFLINK_MAX_FRAME_BYTES] = {0};
    size_t frame_len = build_dflink_frame(a, b, payload, payload_len, frame, sizeof(frame));
    if (frame_len == 0) {
        snprintf(result->last_error, sizeof(result->last_error), "frame_too_large");
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t err = write_bytes_locked(frame, frame_len);
    if (err == ESP_OK) {
        result->tx_bytes += (uint32_t)frame_len;
    } else {
        snprintf(result->last_error, sizeof(result->last_error), "write_failed");
    }
    return err;
}

static esp_err_t send_motion_velocity_locked(float vx_m_s, float vy_m_s, float vz_rad_s, uint32_t *tx_bytes)
{
    uint8_t payload[DFLINK_MOTION_VELOCITY_PAYLOAD_BYTES] = {0};
    write_dflink_f32(&payload[0], vx_m_s);
    write_dflink_f32(&payload[4], vy_m_s);
    write_dflink_f32(&payload[8], vz_rad_s);

    uint8_t frame[32] = {0};
    size_t frame_len = build_dflink_frame(DFLINK_A_MOTION,
                                          DFLINK_B_MOTION_VELOCITY,
                                          payload,
                                          sizeof(payload),
                                          frame,
                                          sizeof(frame));
    if (frame_len == 0) {
        return ESP_ERR_INVALID_SIZE;
    }
    esp_err_t err = write_bytes_locked(frame, frame_len);
    if (err == ESP_OK && tx_bytes != NULL) {
        *tx_bytes += (uint32_t)frame_len;
    }
    return err;
}

static void bytes_to_hex(const uint8_t *bytes, size_t len, char *out, size_t out_size)
{
    size_t limit = len;
    if (limit > DFLINK_MAX_PAYLOAD_BYTES / 2) {
        limit = DFLINK_MAX_PAYLOAD_BYTES / 2;
    }
    size_t pos = 0;
    for (size_t i = 0; i < limit && pos + 3 < out_size; ++i) {
        pos += (size_t)snprintf(&out[pos], out_size - pos, "%02X", bytes[i]);
    }
    if (len > limit && pos + 4 < out_size) {
        snprintf(&out[pos], out_size - pos, "...");
    }
}

static void record_frame(chassis_diag_result_t *result, const uint8_t *frame, size_t frame_len)
{
    if (frame_len < 9) {
        result->checksum_bad++;
        snprintf(result->last_error, sizeof(result->last_error), "short_frame");
        return;
    }

    uint8_t payload_len = frame[5];
    size_t expected_len = (size_t)payload_len + 9;
    if (frame_len != expected_len || frame[expected_len - 3] != DFLINK_FRAME_TAIL) {
        result->checksum_bad++;
        snprintf(result->last_error, sizeof(result->last_error), "bad_shape");
        return;
    }

    uint16_t expected_sum = (uint16_t)frame[expected_len - 2] |
                            ((uint16_t)frame[expected_len - 1] << 8);
    uint16_t actual_sum = checksum16(frame, expected_len - 2);
    bool checksum_ok = expected_sum == actual_sum;

    result->frames++;
    result->last_target = frame[1];
    result->last_source = frame[2];
    result->last_a = frame[3];
    result->last_b = frame[4];
    result->last_len = payload_len;
    result->last_checksum_ok = checksum_ok;
    result->last_payload_hex[0] = '\0';
    bytes_to_hex(&frame[6], payload_len, result->last_payload_hex, sizeof(result->last_payload_hex));

    if (checksum_ok) {
        result->checksum_ok++;
        snprintf(result->last_error, sizeof(result->last_error), "none");
    } else {
        result->checksum_bad++;
        snprintf(result->last_error, sizeof(result->last_error), "checksum");
    }
}

static void parse_rx_bytes(chassis_diag_result_t *result, const uint8_t *bytes, size_t len)
{
    size_t pos = 0;
    while (pos < len) {
        while (pos < len && bytes[pos] != DFLINK_FRAME_HEAD) {
            pos++;
        }
        if (pos + 6 > len) {
            break;
        }

        uint8_t payload_len = bytes[pos + 5];
        size_t frame_len = (size_t)payload_len + 9;
        if (frame_len > DFLINK_MAX_FRAME_BYTES) {
            result->checksum_bad++;
            snprintf(result->last_error, sizeof(result->last_error), "oversize");
            pos++;
            continue;
        }
        if (pos + frame_len > len) {
            break;
        }
        record_frame(result, &bytes[pos], frame_len);
        pos += frame_len;
    }
}

static void read_diag_responses_locked(chassis_diag_result_t *result, uint32_t total_timeout_ms)
{
    uint8_t buffer[DFLINK_MAX_FRAME_BYTES * 3] = {0};
    size_t used = 0;
    uint32_t slices = total_timeout_ms / 50U;
    if (slices == 0) {
        slices = 1;
    }

    for (uint32_t i = 0; i < slices && used < sizeof(buffer); ++i) {
        uint8_t chunk[DFLINK_DIAG_READ_CHUNK_BYTES] = {0};
        int read = uart_read_bytes(s_config.port, chunk, sizeof(chunk), pdMS_TO_TICKS(50));
        if (read > 0) {
            size_t copy = (size_t)read;
            if (copy > sizeof(buffer) - used) {
                copy = sizeof(buffer) - used;
            }
            memcpy(&buffer[used], chunk, copy);
            used += copy;
            result->rx_bytes += (uint32_t)copy;
        }
    }

    if (used == 0) {
        snprintf(result->last_error, sizeof(result->last_error), "timeout");
        return;
    }
    parse_rx_bytes(result, buffer, used);
}

static esp_err_t run_diag_query_locked(const char *command, chassis_diag_result_t *result)
{
    if (strcmp(command, "ready") == 0) {
        const uint8_t payload[] = {0x02, 0x00};
        ESP_RETURN_ON_ERROR(send_dflink_command_locked(0x0B, 0x06, payload, sizeof(payload), result), TAG, "ready query");
        read_diag_responses_locked(result, 500);
        return ESP_OK;
    }
    if (strcmp(command, "version") == 0) {
        const uint8_t payload[] = {0x02, 0x00};
        ESP_RETURN_ON_ERROR(send_dflink_command_locked(0x0B, 0x03, payload, sizeof(payload), result), TAG, "version query");
        read_diag_responses_locked(result, 500);
        return ESP_OK;
    }
    if (strcmp(command, "params") == 0) {
        const uint8_t payload[] = {0x02, 0x00};
        ESP_RETURN_ON_ERROR(send_dflink_command_locked(0x0B, 0x3F, payload, sizeof(payload), result), TAG, "params query");
        read_diag_responses_locked(result, 700);
        return ESP_OK;
    }
    if (strcmp(command, "odom") == 0) {
        const uint8_t payload[] = {0x01, 0x0A};
        ESP_RETURN_ON_ERROR(send_dflink_command_locked(0x04, 0x80, payload, sizeof(payload), result), TAG, "odom query");
        read_diag_responses_locked(result, 700);
        return ESP_OK;
    }
    if (strcmp(command, "repeat") == 0) {
        const uint8_t access_payload[] = {0x02, 0x00};
        ESP_RETURN_ON_ERROR(send_dflink_command_locked(0x0B, 0x06, access_payload, sizeof(access_payload), result), TAG, "repeat ready");
        read_diag_responses_locked(result, 350);
        ESP_RETURN_ON_ERROR(send_dflink_command_locked(0x0B, 0x03, access_payload, sizeof(access_payload), result), TAG, "repeat version");
        read_diag_responses_locked(result, 350);
        ESP_RETURN_ON_ERROR(send_dflink_command_locked(0x0B, 0x3F, access_payload, sizeof(access_payload), result), TAG, "repeat params");
        read_diag_responses_locked(result, 550);
        return ESP_OK;
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t chassis_uart_init(const board_uart_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL && config->enabled, ESP_ERR_INVALID_ARG, TAG, "missing chassis uart config");

    const uart_config_t uart_config = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(config->port, config->rx_buffer_size, config->tx_buffer_size, 0, NULL, 0), TAG, "install uart");
    ESP_RETURN_ON_ERROR(uart_param_config(config->port, &uart_config), TAG, "config uart");
    ESP_RETURN_ON_ERROR(uart_set_pin(config->port, config->tx_pin, config->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "set pins");

    if (s_uart_mutex == NULL) {
        s_uart_mutex = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(s_uart_mutex != NULL, ESP_ERR_NO_MEM, TAG, "create mutex");
    }

    s_config = *config;
    s_ready = true;
    ESP_LOGI(TAG, "ready");
    return ESP_OK;
}

esp_err_t chassis_uart_send_motion(const chassis_motion_cmd_t *cmd)
{
    ESP_RETURN_ON_FALSE(s_ready && cmd != NULL, ESP_ERR_INVALID_STATE, TAG, "chassis uart not ready");
    ESP_RETURN_ON_ERROR(take_uart_mutex(pdMS_TO_TICKS(100)), TAG, "take uart mutex");

    const float vx_m_s = linear_cmd_to_protocol_vx(cmd->linear_mm_s);
    const float vy_m_s = 0.0f;
    const float vz_rad_s = angular_cmd_to_protocol_vz(cmd->angular_mdeg_s);

    esp_err_t err = send_motion_velocity_locked(vx_m_s, vy_m_s, vz_rad_s, NULL);
    give_uart_mutex();
    return err;
}

esp_err_t chassis_uart_stop(void)
{
    const chassis_motion_cmd_t stop_cmd = {
        .linear_mm_s = 0,
        .angular_mdeg_s = 0,
    };
    return chassis_uart_send_motion(&stop_cmd);
}

float chassis_uart_linear_protocol_per_real(void)
{
    return CHASSIS_LINEAR_PROTOCOL_PER_REAL;
}

esp_err_t chassis_uart_diag_command(const char *command, chassis_diag_result_t *result)
{
    ESP_RETURN_ON_FALSE(result != NULL, ESP_ERR_INVALID_ARG, TAG, "missing diag result");
    const char *selected = (command == NULL || command[0] == '\0' || strcmp(command, "?") == 0) ? "status" : command;
    reset_diag_result(selected, result);
    ESP_RETURN_ON_FALSE(s_ready, ESP_ERR_INVALID_STATE, TAG, "chassis uart not ready");

    if (strcmp(selected, "status") == 0) {
        return ESP_OK;
    }
    if (strcmp(selected, "stop0") == 0 || strcmp(selected, "stop") == 0) {
        esp_err_t stop_err = chassis_uart_stop();
        if (stop_err == ESP_OK) {
            snprintf(result->last_error, sizeof(result->last_error), "none");
            result->tx_bytes = 21;
        } else {
            snprintf(result->last_error, sizeof(result->last_error), "stop_failed");
        }
        return stop_err;
    }
    if (strcmp(selected, "xyz") == 0 || strcmp(selected, "xyz0") == 0 || strcmp(selected, "xyzrx") == 0 ||
        strcmp(selected, "xyz500") == 0 || strcmp(selected, "xyz500rx") == 0) {
        ESP_RETURN_ON_ERROR(take_uart_mutex(pdMS_TO_TICKS(200)), TAG, "take xyz mutex");
        const bool is_zero = strcmp(selected, "xyz0") == 0;
        const bool is_scaled_500 = strcmp(selected, "xyz500") == 0 || strcmp(selected, "xyz500rx") == 0;
        const bool wait_for_rx = strcmp(selected, "xyzrx") == 0 || strcmp(selected, "xyz500rx") == 0;
        const float vx_m_s = is_zero ? 0.0f : (is_scaled_500 ? linear_cmd_to_protocol_vx(500) : 0.05f);
        const float vy_m_s = 0.0f;
        const float vz_rad_s = (is_zero || is_scaled_500) ? 0.0f : 0.10f;
        if (wait_for_rx) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(uart_flush_input(s_config.port));
        }
        esp_err_t xyz_err = send_motion_velocity_locked(vx_m_s, vy_m_s, vz_rad_s, &result->tx_bytes);
        if (xyz_err == ESP_OK && wait_for_rx) {
            read_diag_responses_locked(result, 800);
        }
        give_uart_mutex();
        if (xyz_err != ESP_OK) {
            snprintf(result->last_error, sizeof(result->last_error), "write_failed");
        } else if (!wait_for_rx || result->rx_bytes > 0) {
            snprintf(result->last_error, sizeof(result->last_error), "none");
        }
        return xyz_err;
    }

    ESP_RETURN_ON_ERROR(take_uart_mutex(pdMS_TO_TICKS(200)), TAG, "take diag mutex");
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_flush_input(s_config.port));
    esp_err_t err = run_diag_query_locked(selected, result);
    give_uart_mutex();

    if (err == ESP_ERR_NOT_SUPPORTED) {
        snprintf(result->last_error, sizeof(result->last_error), "unsupported");
    } else if (err != ESP_OK && strcmp(result->last_error, "none") == 0) {
        snprintf(result->last_error, sizeof(result->last_error), "failed");
    }
    return err == ESP_ERR_NOT_SUPPORTED ? ESP_ERR_INVALID_ARG : err;
}

esp_err_t chassis_uart_diag_format_text(const chassis_diag_result_t *result, char *buffer, size_t buffer_size)
{
    ESP_RETURN_ON_FALSE(result != NULL && buffer != NULL && buffer_size > 0, ESP_ERR_INVALID_ARG, TAG, "missing format args");

    int written = snprintf(buffer,
                           buffer_size,
                           "C:%s,ready=%d,txpin=%d,rxpin=%d,baud=%d,scale=%.1f,tx=%lu,rx=%lu,frames=%lu,ok=%lu,bad=%lu,last=%02X%02X,len=%u,ck=%d,err=%s\n",
                           result->command,
                           result->ready ? 1 : 0,
                           result->tx_pin,
                           result->rx_pin,
                           result->baud_rate,
                           chassis_uart_linear_protocol_per_real(),
                           (unsigned long)result->tx_bytes,
                           (unsigned long)result->rx_bytes,
                           (unsigned long)result->frames,
                           (unsigned long)result->checksum_ok,
                           (unsigned long)result->checksum_bad,
                           result->last_a,
                           result->last_b,
                           result->last_len,
                           result->last_checksum_ok ? 1 : 0,
                           result->last_error);
    return written > 0 && written < (int)buffer_size ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

esp_err_t chassis_uart_diag_command_text(const char *command, char *buffer, size_t buffer_size)
{
    chassis_diag_result_t result = {0};
    esp_err_t err = chassis_uart_diag_command(command, &result);
    esp_err_t format_err = chassis_uart_diag_format_text(&result, buffer, buffer_size);
    if (format_err != ESP_OK) {
        return format_err;
    }
    return err;
}
