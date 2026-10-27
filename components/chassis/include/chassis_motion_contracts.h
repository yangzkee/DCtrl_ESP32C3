#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DFLINK_FRAME_HEAD 0xDF
#define DFLINK_FRAME_TAIL 0xFD
#define DFLINK_TARGET_ID 0x01
#define DFLINK_SOURCE_ID 0x97
#define DFLINK_A_MOTION 0x02
#define DFLINK_B_MOTION_VELOCITY 0x62
#define DFLINK_MOTION_VELOCITY_PAYLOAD_BYTES 12

typedef struct {
    int32_t vx_mm_s;
    int32_t vy_mm_s;
    // DFLink Wz angular velocity: positive is left / counterclockwise.
    int32_t angular_mdeg_s;
} chassis_motion_cmd_t;

#define DFLINK_MOTION_FRAME_BYTES 21
size_t chassis_motion_encode(const chassis_motion_cmd_t *cmd, uint8_t *frame, size_t capacity);

#ifdef __cplusplus
}
#endif
