#pragma once

#include "line_trace_policy.h"
#include "motion_policy.h"

#ifdef __cplusplus
extern "C" {
#endif

void motion_executor_build_output(const motion_intent_t *intent, line_trace_policy_output_t *output);

#ifdef __cplusplus
}
#endif
