#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BOARD_UART_ROLE_CHASSIS = 0,
    BOARD_UART_ROLE_LINE_SENSOR,
} board_uart_role_t;

typedef struct {
    board_uart_role_t role;
    uart_port_t port;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    int baud_rate;
    int rx_buffer_size;
    int tx_buffer_size;
    bool enabled;
} board_uart_config_t;

typedef struct {
    const char *target_name;
    const char *board_name;
    board_uart_config_t chassis_uart;
    board_uart_config_t line_sensor_uart;
} board_profile_t;

const board_profile_t *board_get_active_profile(void);

#ifdef __cplusplus
}
#endif
