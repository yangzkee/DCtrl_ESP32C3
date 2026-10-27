#include "line_sensor_uart.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if !defined(DCTRL_TRACK_WHITE) || (DCTRL_TRACK_WHITE != 0 && DCTRL_TRACK_WHITE != 1)
#error "Build must select black (0) or white (1) line tracking"
#endif

const char *line_sensor_target_color(void)
{
    return DCTRL_TRACK_WHITE ? "white" : "black";
}

int8_t line_sensor_calculate_offset(uint8_t bits)
{
    // x1 is the leftmost sensor when facing forward from the vehicle.
    static const int8_t weights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
    int sum = 0;
    int count = 0;
    for (int i = 0; i < 8; ++i) {
        if (bits & (1U << i)) {
            sum += weights[i];
            count++;
        }
    }
    return count == 0 ? 0 : (int8_t)(sum / count);
}

esp_err_t line_sensor_parse_digital_frame(const char *frame, uint8_t *bits)
{
    if (frame == NULL || bits == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (strncmp(frame, "$D,", 3) != 0) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const char *cursor = frame + 3;
    uint8_t parsed = 0;
    for (int i = 1; i <= 8; ++i) {
        char key[5];
        snprintf(key, sizeof(key), "x%d:", i);
        if (strncmp(cursor, key, 3) != 0 || (cursor[3] != '0' && cursor[3] != '1')) {
            return ESP_ERR_INVALID_RESPONSE;
        }
        if (cursor[3] - '0' == DCTRL_TRACK_WHITE) {
            parsed |= (uint8_t)(1U << (i - 1));
        }
        cursor += 4;
        if (*cursor++ != (i == 8 ? '#' : ',')) {
            return ESP_ERR_INVALID_RESPONSE;
        }
    }
    if (*cursor != '\0') {
        return ESP_ERR_INVALID_RESPONSE;
    }
    *bits = parsed;
    return ESP_OK;
}
