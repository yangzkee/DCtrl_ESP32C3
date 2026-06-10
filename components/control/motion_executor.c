#include "motion_executor.h"

#include <string.h>

void motion_executor_build_output(const motion_intent_t *intent, line_trace_policy_output_t *output)
{
    if (intent == 0 || output == 0) {
        return;
    }

    memset(output, 0, sizeof(*output));
    output->phase = intent->phase;
    output->plan = intent->plan;
    output->lost_line = intent->lost_line;
    output->line_quality = intent->line_quality;
    output->active_sensor_count = intent->active_sensor_count;
    output->pid_output_mdeg_s = intent->pid_output_mdeg_s;
    output->recovery_relation = intent->recovery_relation;
    output->recovery_stage = intent->recovery_stage;
    output->recovery_angle_mdeg = intent->recovery_angle_mdeg;
    output->recovery_target_mdeg = intent->recovery_target_mdeg;
    output->recovery_direction_mdeg = intent->recovery_direction_mdeg;
}
