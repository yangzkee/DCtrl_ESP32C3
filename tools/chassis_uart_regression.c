#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "chassis_uart.h"

/* Simulated HAL: this verifies driver integration, not physical timing or wiring. */
static uint8_t tx[32768], rx[64];
static size_t tx_size, rx_size, rx_pos, read_chunk = 64;
static uart_config_t configured;
static bool short_write, tx_timeout, mutex_busy, no_reply, bad_reply;

static int32_t read_s32(const uint8_t *p)
{ return (int32_t)((uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24); }

static void check_mapped_velocity(const uint8_t *input, const uint8_t *output)
{
    assert(memcmp(input, output, 6) == 0 && output[18] == 0xFD);
    for (size_t axis = 0; axis < 3; ++axis) {
        const int32_t expected = read_s32(input + 6 + axis * 4);
        assert(read_s32(output + 6 + axis * 4) == (axis == 0 ? -expected : expected));
    }
    uint16_t sum = 0;
    for (int i = 0; i < 19; ++i) sum += output[i];
    assert(output[19] == (uint8_t)sum && output[20] == (uint8_t)(sum >> 8));
}

static void check_remote_transport(void)
{
    chassis_motion_cmd_t cmd = {.vx_mm_s=250, .vy_mm_s=0, .angular_mdeg_s=90000};
    const chassis_motion_cmd_t physical = {.vx_mm_s=20, .vy_mm_s=0, .angular_mdeg_s=90000};
    uint8_t frame[21], physical_frame[21];
    assert(chassis_motion_encode(&cmd, frame, sizeof(frame)) == 21);
    assert(chassis_motion_encode(&physical, physical_frame, sizeof(physical_frame)) == 21);
    /* Every BLE split point must map exactly once, never emit half a frame. */
    for (size_t split = 1; split < sizeof(frame); ++split) {
        tx_size = 0;
        uint32_t written = 0;
        assert(chassis_uart_write_dflink(frame, split, &written) == ESP_OK);
        assert(written == 0 && tx_size == 0);
        assert(chassis_uart_write_dflink(frame + split, sizeof(frame) - split, &written) == ESP_OK);
        assert(written == 21 && tx_size == 21);
        check_mapped_velocity(physical_frame, tx);
        /* Old miniapp 0.25 maps to the same physical 0.02 m/s as local gear 1. */
        assert(chassis_uart_send_motion(&physical) == ESP_OK);
        assert(tx_size == 42 && memcmp(tx, tx + 21, 21) == 0);
    }
    uint8_t batch[231];
    for (size_t i = 0; i < 11; ++i) memcpy(batch + i * 21, frame, 21);
    tx_size = 0;
    uint32_t written = 0;
    assert(chassis_uart_write_dflink(batch, 230, &written) == ESP_OK);
    assert(tx_size == 210 && written == 210);
    assert(chassis_uart_write_dflink(batch + 230, 1, &written) == ESP_OK);
    assert(tx_size == 231 && written == 231);
    for (size_t i = 0; i < 11; ++i) check_mapped_velocity(physical_frame, tx + i * 21);

    tx_size = 0;
    assert(chassis_uart_write_dflink(frame, 8, NULL) == ESP_OK);
    assert(chassis_uart_stop() == ESP_OK); // Discard partial bytes on stop/disconnect.
    assert(tx_size == 21);
    assert(chassis_uart_write_dflink(frame, 21, NULL) == ESP_OK);
    assert(tx_size == 42);
    check_mapped_velocity(physical_frame, tx + 21);
    tx_size = 0;
    frame[19] ^= 1;
    assert(chassis_uart_write_dflink(frame, 21, NULL) == ESP_ERR_INVALID_ARG && tx_size == 0);
    frame[19] ^= 1;
    assert(chassis_uart_write_dflink(frame, 21, NULL) == ESP_OK && tx_size == 21);
    assert(chassis_uart_write_dflink(NULL, 1, NULL) == ESP_ERR_INVALID_ARG);
    assert(chassis_uart_write_dflink(frame, 0, NULL) == ESP_ERR_INVALID_ARG);
    const uint8_t oversized[] = {0xDF, 1, 0x97, 2, 0x62, 255};
    assert(chassis_uart_write_dflink(oversized, sizeof(oversized), NULL) == ESP_ERR_INVALID_SIZE);
    assert(chassis_uart_write_dflink(frame, 21, NULL) == ESP_OK);
    tx_size = 0;
}

static void check_port(uart_port_t port) { assert(port == UART_NUM_1); }
SemaphoreHandle_t xSemaphoreCreateMutex(void) { return &configured; }
int xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t timeout)
{ assert(mutex != NULL && timeout > 0); return !mutex_busy; }
int xSemaphoreGive(SemaphoreHandle_t mutex) { assert(mutex != NULL); return pdTRUE; }
esp_err_t uart_driver_install(uart_port_t port, int rb, int tb, int queue, void *handle, int flags)
{
    check_port(port); assert(rb >= 256 && tb >= 256);
    assert(queue == 0 && handle == NULL && flags == 0); return ESP_OK;
}
esp_err_t uart_param_config(uart_port_t port, const uart_config_t *config)
{ check_port(port); configured = *config; return ESP_OK; }
esp_err_t uart_set_pin(uart_port_t port, int tx_pin, int rx_pin, int rts, int cts)
{
    check_port(port); assert(tx_pin == 4 && rx_pin == 5);
    assert(rts == UART_PIN_NO_CHANGE && cts == UART_PIN_NO_CHANGE); return ESP_OK;
}
int uart_write_bytes(uart_port_t port, const void *bytes, size_t size)
{
    check_port(port);
    if (short_write) return (int)size - 1;
    assert(tx_size + size <= sizeof(tx));
    memcpy(tx + tx_size, bytes, size); tx_size += size;
    const uint8_t *frame = bytes;
    if (size >= 9 && frame[3] == 0x0B && !no_reply) {
        const uint8_t reply[] = {0xDF, 0x97, 1, 0x8C, frame[4], 2, 1, 0, 0xFD, 0, 0};
        rx[0] = 0x55; /* Ignore leading noise. */
        memcpy(rx + 1, reply, sizeof(reply));
        uint16_t sum = 0;
        for (size_t i = 0; i < sizeof(reply) - 2; ++i) sum += reply[i];
        rx[10] = (uint8_t)sum ^ (bad_reply ? 1 : 0); rx[11] = (uint8_t)(sum >> 8);
        rx_size = 12; rx_pos = 0;
    }
    return (int)size;
}
esp_err_t uart_wait_tx_done(uart_port_t port, TickType_t timeout)
{ check_port(port); assert(timeout > 0); return tx_timeout ? ESP_ERR_TIMEOUT : ESP_OK; }
int uart_read_bytes(uart_port_t port, void *out, uint32_t capacity, TickType_t timeout)
{
    check_port(port); assert(timeout > 0);
    size_t count = rx_size - rx_pos;
    if (count > read_chunk) count = read_chunk;
    if (count > capacity) count = capacity;
    memcpy(out, rx + rx_pos, count); rx_pos += count; return (int)count;
}
esp_err_t uart_flush_input(uart_port_t port)
{ check_port(port); rx_size = rx_pos = 0; return ESP_OK; }

int main(void)
{
    const board_profile_t *board = board_get_active_profile();
    assert(board->chassis_uart.baud_rate == 460800);
    assert(board->line_sensor_uart.baud_rate == 115200);
    assert(board->line_sensor_uart.port == UART_NUM_0);
    assert(board->line_sensor_uart.tx_pin == 2 && board->line_sensor_uart.rx_pin == 3);
    chassis_diag_result_t result;
    assert(chassis_uart_diag_command("status", &result) == ESP_ERR_INVALID_STATE);
    assert(chassis_uart_init(NULL) == ESP_ERR_INVALID_ARG);
    assert(chassis_uart_init(&board->chassis_uart) == ESP_OK);
    assert(configured.baud_rate == 460800 && configured.data_bits == 8);
    assert(configured.parity == UART_PARITY_DISABLE && configured.stop_bits == UART_STOP_BITS_1);
    assert(configured.flow_ctrl == UART_HW_FLOWCTRL_DISABLE);
    assert(chassis_uart_diag_command("status", &result) == ESP_OK);
    assert(result.ready && result.baud_rate == 460800 && result.tx_pin == 4 && result.rx_pin == 5);
    char text[256];
    assert(chassis_uart_diag_format_text(&result, text, sizeof(text)) == ESP_OK);
    assert(strstr(text, "baud=460800") != NULL);

    /* 1000 velocities must share the corrected (-Vx, +Vy, +Wz) output mapping. */
    for (int i = 0; i < 1000; ++i) {
        chassis_motion_cmd_t cmd = {.vx_mm_s = i - 500, .vy_mm_s = 500 - i,
                                    .angular_mdeg_s = (i - 500) * 100};
        uint8_t frame[21];
        assert(chassis_motion_encode(&cmd, frame, sizeof(frame)) == sizeof(frame));
        assert(chassis_uart_send_motion(&cmd) == ESP_OK);
        check_mapped_velocity(frame, tx + i * sizeof(frame));
    }
    assert(tx_size == 21000); tx_size = 0;
    check_remote_transport();
    assert(chassis_uart_diag_command("stop0", &result) == ESP_OK);
    const uint8_t stop[] = {0xDF,1,0x97,2,0x62,12,0,0,0,0,0,0,0,0,0,0,0,0,0xFD,0xE4,2};
    assert(tx_size == sizeof(stop) && memcmp(tx, stop, sizeof(stop)) == 0);
    short_write = true;
    assert(chassis_uart_stop() == ESP_FAIL);
    short_write = false; tx_timeout = true;
    assert(chassis_uart_stop() == ESP_ERR_TIMEOUT);
    tx_timeout = false; mutex_busy = true;
    assert(chassis_uart_stop() == ESP_ERR_TIMEOUT);
    mutex_busy = false;

    const char *queries[] = {"ready", "version", "params"};
    const size_t chunks[] = {2, 5, 64};
    for (size_t i = 0; i < 3; ++i) {
        read_chunk = chunks[i];
        for (size_t j = 0; j < 3; ++j) {
            assert(chassis_uart_diag_command(queries[j], &result) == ESP_OK);
            assert(result.frames == 1 && result.checksum_ok == 1 && result.checksum_bad == 0);
            assert(result.last_source == 1 && result.last_target == 0x97);
        }
    }
    assert(chassis_uart_diag_command("repeat", &result) == ESP_OK);
    assert(result.frames == 3 && result.checksum_ok == 3);
    bad_reply = true;
    assert(chassis_uart_diag_command("ready", &result) == ESP_OK);
    assert(result.checksum_bad == 1 && result.checksum_ok == 0);
    bad_reply = false; no_reply = true;
    assert(chassis_uart_diag_command("ready", &result) == ESP_OK);
    assert(result.rx_bytes == 0 && strcmp(result.last_error, "timeout") == 0);
    assert(chassis_uart_diag_command("unsupported", &result) == ESP_ERR_INVALID_ARG);
    puts("UART HAL simulation: 460800/8N1, independent sensor, 1000 mapped frames, 20 BLE splits, 11 packed frames, local/remote equivalence, partial reset, stop, errors and fragmented replies: PASS");
    return 0;
}
