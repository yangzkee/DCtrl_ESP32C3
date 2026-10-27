#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "chassis_motion_contracts.h"
#include "line_interpreter.h"
#include "line_trace_policy.h"
#include "motion_inputs.h"
#include "motion_contracts.h"

static int32_t read_s32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | (uint32_t)p[1] << 8 |
                     (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24);
}

static void check_sensor(void)
{
    assert(strcmp(line_sensor_target_color(), DCTRL_TRACK_WHITE ? "white" : "black") == 0);
    for (unsigned mask = 0; mask < 256; ++mask) {
        char frame[80] = "$D,";
        size_t used = 3;
        for (int i = 0; i < 8; ++i) {
            used += (size_t)snprintf(frame + used, sizeof(frame) - used, "x%d:%u%c",
                                    i + 1, (mask >> i) & 1U, i == 7 ? '#' : ',');
        }
        uint8_t bits = 0;
        assert(line_sensor_parse_digital_frame(frame, &bits) == ESP_OK);
        assert(bits == (DCTRL_TRACK_WHITE ? mask : (uint8_t)~mask));
    }
    const char *bad[] = {"", "$0,0,1#", "$D,x1:0#",
        "$D,x1:2,x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x8:0#",
        "$D,x1:0,x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x8:0",
        "$D,x1:0,x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x8:0#junk"};
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) {
        uint8_t bits = 0x55;
        assert(line_sensor_parse_digital_frame(bad[i], &bits) == ESP_ERR_INVALID_RESPONSE);
        assert(bits == 0x55);
    }
    assert(line_sensor_calculate_offset(1) == -7);
    assert(line_sensor_calculate_offset(128) == 7);
    assert(line_sensor_calculate_offset(0x18) == 0);
}

static void check_wire(void)
{
    const uint8_t golden_stop[21] = {0xDF, 1, 0x97, 2, 0x62, 12,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xFD, 0xE4, 2};
    chassis_motion_cmd_t cmd = {0};
    uint8_t frame[21];
    assert(chassis_motion_encode(&cmd, frame, sizeof(frame)) == 21);
    assert(memcmp(frame, golden_stop, 21) == 0);
    cmd = (chassis_motion_cmd_t){.vx_mm_s = 500, .vy_mm_s = -250, .angular_mdeg_s = 90000};
    assert(chassis_motion_encode(&cmd, frame, sizeof(frame)) == 21);
    assert(read_s32(frame + 6) == 5000);
    assert(read_s32(frame + 10) == -2500);
    assert(read_s32(frame + 14) == 15708); // +90 deg/s = +pi/2 rad/s.
    cmd.angular_mdeg_s = -90000;
    assert(chassis_motion_encode(&cmd, frame, sizeof(frame)) == 21);
    assert(read_s32(frame + 14) == -15708);
    assert(chassis_motion_encode(&cmd, frame, 20) == 0);
    assert(chassis_motion_encode(NULL, frame, 21) == 0);
}

static void check_policy(void)
{
    line_trace_policy_config_t config = {.base_speed_mm_s = 250, .max_turn_mdeg_s = 8000,
                                         .kp = 9000, .integral_limit = 40};
    line_trace_policy_runtime_t runtime;
    line_trace_policy_init(&runtime);
    line_trace_policy_input_t input = {.run_mode = LINE_TRACE_RUN_AUTO_RUNNING,
                                       .sensor_status = LINE_TRACE_SENSOR_OK};
    line_trace_policy_output_t out;
    const uint8_t masks[] = {1, 0x18, 128};
    const int signs[] = {1, 0, -1};
    for (int i = 0; i < 3; ++i) {
        input.sample.bits = masks[i];
        input.sample.offset = line_sensor_calculate_offset(masks[i]);
        line_trace_policy_step(&runtime, &config, &input, &out);
        assert(out.phase == LINE_TRACE_PHASE_TRACK_LINE);
        assert(out.plan.chassis_cmd.vx_mm_s > 0);
        const int32_t wz = out.plan.chassis_cmd.angular_mdeg_s;
        assert(signs[i] > 0 ? wz > 0 : signs[i] < 0 ? wz < 0 : wz == 0);
        assert(wz >= -8000 && wz <= 8000);
    }
    input.sample = (line_sensor_sample_t){0};
    const uint32_t durations[] = {800, 1600, 3200, 6400, 12800, 25600, 30000};
    input.now_ms = 100;
    for (int i = 0; i < 7; ++i) {
        line_trace_policy_step(&runtime, &config, &input, &out);
        assert(out.phase == LINE_TRACE_PHASE_LINE_LOST);
        assert(out.plan.chassis_cmd.vx_mm_s == 0);
        assert(out.plan.chassis_cmd.angular_mdeg_s == (i % 2 == 0 && i != 6 ? 8000 : -8000));
        assert(out.recovery_segment_index == i);
        input.now_ms += durations[i];
    }
    line_trace_policy_step(&runtime, &config, &input, &out);
    assert(out.plan.vehicle_action == CONTROL_VEHICLE_ACTION_STOP_TO_IDLE);
    assert(out.plan.chassis_action == CONTROL_CHASSIS_ACTION_STOP);
    assert(out.plan.chassis_cmd.vx_mm_s == 0 && out.plan.chassis_cmd.angular_mdeg_s == 0);

    line_trace_policy_init(&runtime);
    input.now_ms = 0;
    line_trace_policy_step(&runtime, &config, &input, &out);
    input.sample = (line_sensor_sample_t){.bits = 0x18, .offset = 0};
    input.now_ms = 20;
    line_trace_policy_step(&runtime, &config, &input, &out);
    assert(out.phase == LINE_TRACE_PHASE_TRACK_LINE && !out.lost_line);
    input.run_mode = LINE_TRACE_RUN_AUTO_ARMED;
    line_trace_policy_step(&runtime, &config, &input, &out);
    assert(out.plan.chassis_action == CONTROL_CHASSIS_ACTION_NONE);
    input.run_mode = LINE_TRACE_RUN_DISABLED;
    line_trace_policy_step(&runtime, &config, &input, &out);
    assert(out.plan.chassis_action == CONTROL_CHASSIS_ACTION_NONE);
    assert(motion_inputs_run_mode_from_vehicle(NULL) == LINE_TRACE_RUN_DISABLED);
}

int main(void)
{
    check_sensor();
    check_wire();
    check_policy();
    printf("%s: 256 sensor patterns, malformed frames, DF-Link units/stop, steering, search, reacquire: PASS\n",
           line_sensor_target_color());
    return 0;
}
