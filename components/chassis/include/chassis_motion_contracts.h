#pragma once

#include <stdbool.h>
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
    // DFLink Motion_Velocity Vz is a per-command Z angle increment.
    int32_t yaw_mdeg;
} chassis_motion_cmd_t;

#ifdef __cplusplus
}
#endif
