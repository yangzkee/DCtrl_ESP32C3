#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *softap_ssid_prefix;
    const char *softap_password;
    uint8_t channel;
    uint8_t max_connections;
    bool start_wifi;
    bool start_ble_placeholder;
} debug_server_config_t;

esp_err_t debug_server_start(const debug_server_config_t *config);
esp_err_t debug_server_start_wifi_fallback(void);

#ifdef __cplusplus
}
#endif
