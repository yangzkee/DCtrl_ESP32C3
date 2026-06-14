#include "esp_log.h"

#include "board_profile.h"
#include "chassis_uart.h"
#include "debug_server.h"
#include "line_sensor_uart.h"
#include "nvs_flash.h"
#include "param_store.h"
#include "vehicle_state.h"

static const char *TAG = "DCar-Liner";

static esp_err_t init_nvs_flash(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

void app_main(void)
{
    const board_profile_t *profile = board_get_active_profile();

    ESP_ERROR_CHECK(init_nvs_flash());
    ESP_ERROR_CHECK(param_store_init());
    vehicle_state_init();

    ESP_LOGI(TAG, "boot target=%s board=%s", profile->target_name, profile->board_name);
    ESP_LOGI(TAG, "chassis uart=%d tx=%d rx=%d baud=%d",
             profile->chassis_uart.port,
             profile->chassis_uart.tx_pin,
             profile->chassis_uart.rx_pin,
             profile->chassis_uart.baud_rate);
    ESP_LOGI(TAG, "line sensor uart=%d tx=%d rx=%d baud=%d",
             profile->line_sensor_uart.port,
             profile->line_sensor_uart.tx_pin,
             profile->line_sensor_uart.rx_pin,
             profile->line_sensor_uart.baud_rate);

    ESP_ERROR_CHECK(chassis_uart_init(&profile->chassis_uart));
    ESP_ERROR_CHECK(line_sensor_uart_init(&profile->line_sensor_uart));
    ESP_ERROR_CHECK(debug_server_start(NULL));
    vehicle_state_finish_boot();
}
