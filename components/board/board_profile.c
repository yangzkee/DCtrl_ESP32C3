#include "board_profile.h"

#include "sdkconfig.h"

#define UART_BUFFER_BYTES 256

#if CONFIG_IDF_TARGET_ESP32C3

static const board_profile_t ACTIVE_PROFILE = {
    .target_name = "esp32c3",
    .board_name = "esp32c3-portable-dual-uart-profile",
    .chassis_uart = {
        .role = BOARD_UART_ROLE_CHASSIS,
        // 这根线接的是 DCar 底盘的「3 号串口」（底盘侧命名为 UART3，波特率 115200；
        // 底盘其他串口为 460800）。下面的 UART_NUM_1 是 ESP32-C3 这一端的串口外设编号——
        // ESP32-C3 整颗芯片只有 UART0/UART1 两个串口，没有 UART3，故此处必须是 UART_NUM_1。
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
#error "Unsupported target. This project targets ESP32-C3 only."
#endif

const board_profile_t *board_get_active_profile(void)
{
    return &ACTIVE_PROFILE;
}
