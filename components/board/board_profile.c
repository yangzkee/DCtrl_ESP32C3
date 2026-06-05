#include "board_profile.h"

#include "sdkconfig.h"

#define UART_BUFFER_BYTES 256

#if CONFIG_IDF_TARGET_ESP32S3

static const board_profile_t ACTIVE_PROFILE = {
    .target_name = "esp32s3",
    .board_name = "esp32s3-dual-uart-default",
    .chassis_uart = {
        .role = BOARD_UART_ROLE_CHASSIS,
        .port = UART_NUM_1,
        .tx_pin = GPIO_NUM_17,
        .rx_pin = GPIO_NUM_18,
        .baud_rate = 115200,
        .rx_buffer_size = UART_BUFFER_BYTES,
        .tx_buffer_size = UART_BUFFER_BYTES,
        .enabled = true,
    },
    .line_sensor_uart = {
        .role = BOARD_UART_ROLE_LINE_SENSOR,
        .port = UART_NUM_2,
        .tx_pin = GPIO_NUM_15,
        .rx_pin = GPIO_NUM_16,
        .baud_rate = 115200,
        .rx_buffer_size = UART_BUFFER_BYTES,
        .tx_buffer_size = UART_BUFFER_BYTES,
        .enabled = true,
    },
};

#elif CONFIG_IDF_TARGET_ESP32C3

static const board_profile_t ACTIVE_PROFILE = {
    .target_name = "esp32c3",
    .board_name = "esp32c3-portable-dual-uart-profile",
    .chassis_uart = {
        .role = BOARD_UART_ROLE_CHASSIS,
        .port = UART_NUM_1,
        .tx_pin = GPIO_NUM_4,
        .rx_pin = GPIO_NUM_5,
        .baud_rate = 115200,
        .rx_buffer_size = UART_BUFFER_BYTES,
        .tx_buffer_size = UART_BUFFER_BYTES,
        .enabled = true,
    },
    .line_sensor_uart = {
        .role = BOARD_UART_ROLE_LINE_SENSOR,
        .port = UART_NUM_0,
        .tx_pin = GPIO_NUM_2,
        .rx_pin = GPIO_NUM_3,
        .baud_rate = 115200,
        .rx_buffer_size = UART_BUFFER_BYTES,
        .tx_buffer_size = UART_BUFFER_BYTES,
        .enabled = true,
    },
};

#else
#error "Unsupported target. This project currently defines profiles for ESP32-S3 and ESP32-C3."
#endif

const board_profile_t *board_get_active_profile(void)
{
    return &ACTIVE_PROFILE;
}
