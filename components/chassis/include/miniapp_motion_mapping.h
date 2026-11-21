#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Existing miniapp input -> physical velocity, before shared direction mapping.
// Continuous radial map: 0 -> 0, 0.25 -> 0.02, 1 -> 0.05, 3 -> 0.10 m/s.
// No gear is inferred or retained. Wz, zero stop and other valid commands are unchanged.
// Can map in place, after BLE frame reassembly. Does not affect local line following.
bool miniapp_motion_map_frame(const uint8_t *input, size_t length,
                             uint8_t *output, size_t capacity);
