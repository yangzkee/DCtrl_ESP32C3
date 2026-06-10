#pragma once

#include "chassis_motion_contracts.h"
#include "vehicle_state.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CONTROL_CHASSIS_ACTION_NONE = 0,
    CONTROL_CHASSIS_ACTION_SEND_MOTION,
    CONTROL_CHASSIS_ACTION_STOP,
} control_chassis_action_t;

typedef enum {
    CONTROL_VEHICLE_ACTION_NONE = 0,
    CONTROL_VEHICLE_ACTION_STOP_TO_IDLE,
    CONTROL_VEHICLE_ACTION_ENTER_FAULT,
} control_vehicle_action_t;

typedef struct {
    control_chassis_action_t chassis_action;
    chassis_motion_cmd_t chassis_cmd;
    control_vehicle_action_t vehicle_action;
    vehicle_fault_reason_t fault_reason;
} control_plan_t;

#ifdef __cplusplus
}
#endif
