#include "chassis_motion_contracts.h"

#include <string.h>

static void write_fixed(uint8_t *out, float value)
{
    const float scaled = value * 10000.0f;
    const int32_t fixed = (int32_t)(scaled >= 0 ? scaled + 0.5f : scaled - 0.5f);
    const uint32_t bits = (uint32_t)fixed;
    for (int i = 0; i < 4; ++i) {
        out[i] = (uint8_t)(bits >> (8 * i));
    }
}

size_t chassis_motion_encode(const chassis_motion_cmd_t *cmd, uint8_t *frame, size_t capacity)
{
    if (cmd == NULL || frame == NULL || capacity < DFLINK_MOTION_FRAME_BYTES) {
        return 0;
    }
    const uint8_t header[] = {DFLINK_FRAME_HEAD, DFLINK_TARGET_ID, DFLINK_SOURCE_ID,
                              DFLINK_A_MOTION, DFLINK_B_MOTION_VELOCITY,
                              DFLINK_MOTION_VELOCITY_PAYLOAD_BYTES};
    memcpy(frame, header, sizeof(header));
    write_fixed(frame + 6, (float)cmd->vx_mm_s / 1000.0f);
    write_fixed(frame + 10, (float)cmd->vy_mm_s / 1000.0f);
    // mdeg/s -> rad/s. Sending frequency never scales this velocity.
    write_fixed(frame + 14, ((float)cmd->angular_mdeg_s / 1000.0f) * (3.14159265358979323846f / 180.0f));
    frame[18] = DFLINK_FRAME_TAIL;
    uint16_t sum = 0;
    for (int i = 0; i < 19; ++i) {
        sum = (uint16_t)(sum + frame[i]);
    }
    frame[19] = (uint8_t)sum;
    frame[20] = (uint8_t)(sum >> 8);
    return DFLINK_MOTION_FRAME_BYTES;
}

bool chassis_dflink_frame_valid(const uint8_t *frame, size_t length)
{
    if (frame == NULL || length < 9 ||
        frame[0] != DFLINK_FRAME_HEAD || length != (size_t)frame[5] + 9 ||
        frame[length - 3] != DFLINK_FRAME_TAIL) {
        return false;
    }
    uint16_t sum = 0;
    for (size_t i = 0; i < length - 2; ++i) sum = (uint16_t)(sum + frame[i]);
    return frame[length - 2] == (uint8_t)sum && frame[length - 1] == (uint8_t)(sum >> 8);
}

bool chassis_motion_map_output(const uint8_t *frame, size_t length, uint8_t *output, size_t capacity)
{
    if (output == NULL || capacity < length || !chassis_dflink_frame_valid(frame, length)) {
        return false;
    }
    const bool velocity = frame[3] == DFLINK_A_MOTION && frame[4] == DFLINK_B_MOTION_VELOCITY;
    uint32_t fields[3] = {0};
    if (velocity) {
        if (length != DFLINK_MOTION_FRAME_BYTES) return false;
        for (size_t axis = 0; axis < 3; ++axis) {
            const uint8_t *p = frame + 6 + axis * 4;
            fields[axis] = (uint32_t)p[0] | (uint32_t)p[1] << 8 |
                           (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
            // INT32_MIN has no representable opposite. Never wrap it into motion.
            if (axis == 0 && fields[axis] == UINT32_C(0x80000000)) return false;
        }
    }
    memmove(output, frame, length);
    if (!velocity) return true;
    for (size_t axis = 0; axis < 3; ++axis) {
        // Field result: forward/backward was correct; undo Y/Wz inversion only.
        const uint32_t mapped = axis == 0 ? 0U - fields[axis] : fields[axis];
        for (size_t byte = 0; byte < 4; ++byte) {
            output[6 + axis * 4 + byte] = (uint8_t)(mapped >> (byte * 8));
        }
    }
    uint16_t sum = 0;
    for (size_t i = 0; i < length - 2; ++i) sum = (uint16_t)(sum + output[i]);
    output[length - 2] = (uint8_t)sum;
    output[length - 1] = (uint8_t)(sum >> 8);
    return true;
}
