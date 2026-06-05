#pragma once

#include "esp_err.h"

#include "board_profile.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t line_trace_controller_start(const board_profile_t *profile);

#ifdef __cplusplus
}
#endif
