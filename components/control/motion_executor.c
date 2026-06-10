#include "motion_executor.h"

#include <string.h>

void motion_executor_build_output(const motion_intent_t *intent, line_trace_policy_output_t *output)
{
    if (intent == 0 || output == 0) {
        return;
    }

    memset(output, 0, sizeof(*output));
    output->phase = intent->phase;
    output->cmd = intent->cmd;
    output->should_send_motion = intent->should_send_motion;
    output->should_stop = intent->should_stop;
    output->enter_fault = intent->enter_fault;
    output->fault_reason = intent->fault_reason;
    output->lost_line = intent->lost_line;
    output->line_quality = intent->line_quality;
    output->active_sensor_count = intent->active_sensor_count;
    output->pid_output_mdeg_s = intent->pid_output_mdeg_s;
    output->recovery_relation = intent->recovery_relation;
    output->recovery_angle_mdeg = intent->recovery_angle_mdeg;
    output->recovery_target_mdeg = intent->recovery_target_mdeg;
    output->recovery_direction_mdeg = intent->recovery_direction_mdeg;
}
