#include "line_sensor_uart.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#define LINE_SENSOR_DIGITAL_REQUEST "$0,0,1#"
#define LINE_SENSOR_FRAME_BYTES 96

static const char *TAG = "line_sensor_uart";
static board_uart_config_t s_config;
static bool s_ready;
static portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;
static line_sensor_uart_status_t s_status;

static void status_set_ready(bool ready)
{
    taskENTER_CRITICAL(&s_status_lock);
    s_status.ready = ready;
    taskEXIT_CRITICAL(&s_status_lock);
}

static void status_note_request(esp_err_t err)
{
    taskENTER_CRITICAL(&s_status_lock);
    s_status.request_count++;
    s_status.last_error = err;
    taskEXIT_CRITICAL(&s_status_lock);
}

static void status_note_frame(const char *frame)
{
    taskENTER_CRITICAL(&s_status_lock);
    s_status.frame_count++;
    if (frame != NULL) {
        strlcpy(s_status.last_frame, frame, sizeof(s_status.last_frame));
    }
    taskEXIT_CRITICAL(&s_status_lock);
}

static void status_note_result(esp_err_t err)
{
    taskENTER_CRITICAL(&s_status_lock);
    s_status.last_error = err;
    if (err == ESP_OK) {
        s_status.digital_frame_count++;
    } else if (err == ESP_ERR_TIMEOUT) {
        s_status.timeout_count++;
    } else if (err == ESP_ERR_INVALID_RESPONSE) {
        s_status.invalid_frame_count++;
    }
    taskEXIT_CRITICAL(&s_status_lock);
}

int8_t line_sensor_calculate_offset(uint8_t bits)
{
    static const int8_t weights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
    int sum = 0;
    int count = 0;

    for (int i = 0; i < 8; ++i) {
        if (bits & (1U << i)) {
            sum += weights[i];
            count++;
        }
    }

    if (count == 0) {
        return 0;
    }
    return (int8_t)(sum / count);
}

static esp_err_t request_digital_stream(void)
{
    const char request[] = LINE_SENSOR_DIGITAL_REQUEST;
    int written = uart_write_bytes(s_config.port, request, strlen(request));
    esp_err_t err = written == (int)strlen(request) ? ESP_OK : ESP_FAIL;
    status_note_request(err);
    return err;
}

esp_err_t line_sensor_uart_init(const board_uart_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL && config->enabled, ESP_ERR_INVALID_ARG, TAG, "missing line sensor uart config");

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

    s_config = *config;
    s_ready = true;
    status_set_ready(true);
    ESP_RETURN_ON_ERROR(request_digital_stream(), TAG, "request digital stream");
    ESP_LOGI(TAG, "ready");
    return ESP_OK;
}

static uint32_t remaining_ms(uint64_t deadline_us)
{
    uint64_t now_us = esp_timer_get_time();
    if (now_us >= deadline_us) {
        return 0;
    }

    uint64_t remaining_us = deadline_us - now_us;
    uint32_t ms = (uint32_t)((remaining_us + 999ULL) / 1000ULL);
    return ms == 0 ? 1 : ms;
}

static esp_err_t read_ascii_frame(char *frame, size_t frame_size, uint64_t deadline_us)
{
    bool in_frame = false;
    size_t index = 0;

    while (remaining_ms(deadline_us) > 0) {
        uint8_t byte = 0;
        int read = uart_read_bytes(s_config.port, &byte, 1, pdMS_TO_TICKS(remaining_ms(deadline_us)));
        if (read <= 0) {
            return ESP_ERR_TIMEOUT;
        }

        if (byte == '$') {
            in_frame = true;
            index = 0;
        }

        if (!in_frame) {
            continue;
        }

        if (index >= frame_size - 1) {
            in_frame = false;
            index = 0;
            continue;
        }

        frame[index++] = (char)byte;
        if (byte == '#') {
            frame[index] = '\0';
            status_note_frame(frame);
            return ESP_OK;
        }
    }

    return ESP_ERR_TIMEOUT;
}

static esp_err_t parse_digital_frame(const char *frame, uint8_t *bits)
{
    ESP_RETURN_ON_FALSE(frame != NULL && bits != NULL, ESP_ERR_INVALID_ARG, TAG, "missing digital frame args");
    if (strncmp(frame, "$D,", 3) != 0) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint8_t parsed_bits = 0;
    for (int i = 1; i <= 8; ++i) {
        char key[5] = {0};
        snprintf(key, sizeof(key), "x%d:", i);
        const char *field = strstr(frame, key);
        if (field == NULL) {
            return ESP_ERR_INVALID_RESPONSE;
        }

        char *end = NULL;
        long value = strtol(field + strlen(key), &end, 10);
        if (end == field + strlen(key)) {
            return ESP_ERR_INVALID_RESPONSE;
        }
        if (value == 0) {
            parsed_bits |= (uint8_t)(1U << (i - 1));
        }
    }

    *bits = parsed_bits;
    return ESP_OK;
}

esp_err_t line_sensor_uart_read_sample(line_sensor_sample_t *sample, uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(s_ready && sample != NULL, ESP_ERR_INVALID_STATE, TAG, "line sensor uart not ready");

    const uint64_t deadline_us = esp_timer_get_time() + ((uint64_t)timeout_ms * 1000ULL);
    char frame[LINE_SENSOR_FRAME_BYTES] = {0};
    uint8_t byte = 0;

    while (remaining_ms(deadline_us) > 0) {
        esp_err_t err = read_ascii_frame(frame, sizeof(frame), deadline_us);
        if (err != ESP_OK) {
            status_note_result(err);
            request_digital_stream();
            return err;
        }

        err = parse_digital_frame(frame, &byte);
        if (err == ESP_ERR_INVALID_RESPONSE) {
            status_note_result(err);
            continue;
        }
        ESP_RETURN_ON_ERROR(err, TAG, "parse digital frame");
        status_note_result(ESP_OK);

        sample->bits = byte;
        sample->offset = line_sensor_calculate_offset(byte);
        sample->updated_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
        return ESP_OK;
    }

    request_digital_stream();
    status_note_result(ESP_ERR_TIMEOUT);
    return ESP_ERR_TIMEOUT;
}

void line_sensor_uart_get_status(line_sensor_uart_status_t *status)
{
    if (status == NULL) {
        return;
    }

    taskENTER_CRITICAL(&s_status_lock);
    *status = s_status;
    taskEXIT_CRITICAL(&s_status_lock);
}
