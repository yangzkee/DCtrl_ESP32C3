#!/usr/bin/env python3
"""Exercise the real chassis UART driver against a simulated ESP-IDF HAL."""

from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
STUB = r"""
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG 1
#define ESP_ERR_INVALID_STATE 2
#define ESP_ERR_TIMEOUT 3
#define ESP_ERR_INVALID_SIZE 4
#define ESP_ERR_NOT_SUPPORTED 5
#define ESP_ERR_NO_MEM 6
#define ESP_RETURN_ON_FALSE(test, error, tag, ...) do { (void)(tag); if (!(test)) return (error); } while (0)
#define ESP_RETURN_ON_ERROR(call, tag, ...) do { (void)(tag); esp_err_t e_ = (call); if (e_ != ESP_OK) return e_; } while (0)
#define ESP_ERROR_CHECK_WITHOUT_ABORT(call) ((void)(call))
#define ESP_LOGI(tag, ...) ((void)(tag))
typedef int uart_port_t;
typedef int gpio_num_t;
enum { UART_NUM_0, UART_NUM_1 };
enum { GPIO_NUM_2=2, GPIO_NUM_3, GPIO_NUM_4, GPIO_NUM_5 };
enum { UART_DATA_8_BITS=8, UART_PARITY_DISABLE=0, UART_STOP_BITS_1=1,
       UART_HW_FLOWCTRL_DISABLE=0, UART_SCLK_DEFAULT=0, UART_PIN_NO_CHANGE=-1 };
typedef struct { int baud_rate, data_bits, parity, stop_bits, flow_ctrl, source_clk; } uart_config_t;
typedef unsigned TickType_t;
typedef void *SemaphoreHandle_t;
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define pdTRUE 1
SemaphoreHandle_t xSemaphoreCreateMutex(void);
int xSemaphoreTake(SemaphoreHandle_t, TickType_t);
int xSemaphoreGive(SemaphoreHandle_t);
esp_err_t uart_driver_install(uart_port_t, int, int, int, void *, int);
esp_err_t uart_param_config(uart_port_t, const uart_config_t *);
esp_err_t uart_set_pin(uart_port_t, int, int, int, int);
int uart_write_bytes(uart_port_t, const void *, size_t);
esp_err_t uart_wait_tx_done(uart_port_t, TickType_t);
int uart_read_bytes(uart_port_t, void *, uint32_t, TickType_t);
esp_err_t uart_flush_input(uart_port_t);
"""


def main():
    with tempfile.TemporaryDirectory(prefix="dctrl-uart-test-") as tmp:
        tmp = Path(tmp)
        (tmp / "platform.h").write_text(STUB)
        for name in ("esp_err.h", "esp_check.h", "esp_log.h", "driver/gpio.h",
                     "driver/uart.h", "freertos/FreeRTOS.h", "freertos/semphr.h"):
            target = tmp / name
            target.parent.mkdir(exist_ok=True)
            target.write_text('#include "platform.h"\n')
        (tmp / "sdkconfig.h").write_text("#define CONFIG_IDF_TARGET_ESP32C3 1\n")
        executable = tmp / "uart-test"
        cmd = ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-O1",
               "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-I", str(tmp)]
        for component in ("board", "chassis"):
            cmd += ["-I", str(ROOT / "components" / component / "include")]
        cmd += [str(ROOT / path) for path in (
            "tools/chassis_uart_regression.c", "components/board/board_profile.c",
            "components/chassis/chassis_uart.c", "components/chassis/dflink_motion_codec.c",
            "components/chassis/miniapp_motion_mapping.c")]
        subprocess.run(cmd + ["-o", str(executable)], check=True, timeout=30)
        subprocess.run([str(executable)], check=True, timeout=5)


if __name__ == "__main__":
    main()
