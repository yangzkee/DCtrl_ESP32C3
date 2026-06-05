#pragma once

#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t debug_protocol_handle_message(const char *request_json, char *response_json, size_t response_size);

#ifdef __cplusplus
}
#endif
