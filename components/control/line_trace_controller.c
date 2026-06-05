#include "line_trace_controller.h"

#include <string.h>

#include "chassis_uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "line_sensor_uart.h"
#include "line_trace_policy.h"
#include "param_store.h"
#include "telemetry.h"
#include "vehicle_state.h"

static const char *TAG = "DCar-LinerCtl";

static int32_t get_int_param(const char *id, int32_t fallback)
{
    int32_t value = fallback;
    ESP_ERROR_CHECK_WITHOUT_ABORT(param_store_get_int(id, &value));
    return value;
}

static float get_float_param(const char *id, float fallback)
{
    float value = fallback;
    ESP_ERROR_CHECK_WITHOUT_ABORT(param_store_get_float(id, &value));
    return value;
}

static void speed_profile_from_gear(int32_t gear, int32_t *base_speed_mm_s, int32_t *max_turn_mdeg_s)
{
    switch (gear) {
    case 1:
        *base_speed_mm_s = 250;
        *max_turn_mdeg_s = 8000;
        break;
    case 3:
        *base_speed_mm_s = 1000;
        *max_turn_mdeg_s = 10000;
        break;
    case 2:
    default:
        *base_speed_mm_s = 600;
        *max_turn_mdeg_s = 10000;
        break;
    }
}

static bool get_bool_param(const char *id, bool fallback)
{
    bool value = fallback;
    ESP_ERROR_CHECK_WITHOUT_ABORT(param_store_get_bool(id, &value));
    return value;
}

static line_trace_run_mode_t run_mode_from_vehicle(const vehicle_state_snapshot_t *vehicle)
{
    if (vehicle == NULL) {
        return LINE_TRACE_RUN_DISABLED;
    }

    switch (vehicle->motion_state) {
    case VEHICLE_MOTION_MANUAL_TEST:
        return LINE_TRACE_RUN_MANUAL_TEST;
    case VEHICLE_MOTION_AUTO_ARMED:
        return LINE_TRACE_RUN_AUTO_ARMED;
    case VEHICLE_MOTION_AUTO_RUNNING:
        return LINE_TRACE_RUN_AUTO_RUNNING;
    default:
        return LINE_TRACE_RUN_DISABLED;
    }
}

static void read_policy_config(line_trace_policy_config_t *config)
{
    int32_t base_speed_mm_s = 600;
    int32_t max_turn_mdeg_s = 10000;
    speed_profile_from_gear(get_int_param("speed.gear", 2), &base_speed_mm_s, &max_turn_mdeg_s);

    config->base_speed_mm_s = base_speed_mm_s;
    config->max_turn_mdeg_s = max_turn_mdeg_s;
    config->kp = get_float_param("pid.kp", 9000.0f);
    config->ki = get_float_param("pid.ki", 0.0f);
    config->kd = get_float_param("pid.kd", 0.0f);
    config->integral_limit = get_float_param("pid.integral_limit", 40.0f);
}

static void apply_inversion_if_needed(line_sensor_sample_t *sample)
{
    if (sample == NULL || !get_bool_param("sensor.invert_bits", false)) {
        return;
    }
    sample->bits = (uint8_t)~sample->bits;
    sample->offset = line_sensor_calculate_offset(sample->bits);
}

static void update_policy_telemetry(const line_trace_policy_output_t *output)
{
    if (output == NULL) {
        return;
    }

    telemetry_update_motion_cmd(&output->cmd);
    telemetry_controller_state_t state = {
        .lost_line = output->lost_line,
        .line_quality = output->line_quality,
        .active_sensor_count = output->active_sensor_count,
        .pid_output_mdeg_s = output->pid_output_mdeg_s,
    };
    strlcpy(state.line_phase,
            line_trace_policy_phase_name(output->phase),
            sizeof(state.line_phase));
    telemetry_update_controller_state(&state);
}

static void apply_policy_output(const line_trace_policy_output_t *output)
{
    if (output == NULL) {
        return;
    }

    update_policy_telemetry(output);
    if (output->enter_fault) {
        vehicle_state_enter_fault(output->fault_reason);
    }

    esp_err_t err = ESP_OK;
    if (output->should_send_motion) {
        err = chassis_uart_send_motion(&output->cmd);
    } else if (output->should_stop) {
        err = chassis_uart_stop();
    }

    if (err != ESP_OK) {
        vehicle_state_enter_fault(VEHICLE_FAULT_CHASSIS_SEND_FAILED);
        ESP_ERROR_CHECK_WITHOUT_ABORT(chassis_uart_stop());
    }
}

static void controller_task(void *arg)
{
    (void)arg;
    line_trace_policy_runtime_t policy_runtime = {0};
    line_trace_policy_init(&policy_runtime);

    while (true) {
        const int32_t timeout_ms = get_int_param("sensor.timeout_ms", 50);
        const int32_t loop_period_ms = get_int_param("control.loop_period_ms", 20);
        vehicle_state_finish_manual_if_expired(vehicle_state_now_ms());

        vehicle_state_snapshot_t vehicle = {0};
        vehicle_state_get_snapshot(&vehicle);
        line_trace_policy_input_t input = {
            .run_mode = run_mode_from_vehicle(&vehicle),
            .sensor_status = LINE_TRACE_SENSOR_OK,
            .manual_cmd = vehicle.manual_cmd,
        };

        if (input.run_mode != LINE_TRACE_RUN_MANUAL_TEST &&
            vehicle.motion_state != VEHICLE_MOTION_OTA_UPDATE) {
            esp_err_t err = line_sensor_uart_read_sample(&input.sample, timeout_ms);
            if (err == ESP_OK) {
                apply_inversion_if_needed(&input.sample);
                telemetry_update_line_sample(&input.sample);
            } else if (err == ESP_ERR_TIMEOUT) {
                input.sensor_status = LINE_TRACE_SENSOR_TIMEOUT;
            } else {
                ESP_LOGW(TAG, "line sensor read failed: %s", esp_err_to_name(err));
                input.sensor_status = LINE_TRACE_SENSOR_ERROR;
            }
        }

        line_trace_policy_config_t config = {0};
        read_policy_config(&config);
        line_trace_policy_output_t output = {0};
        line_trace_policy_step(&policy_runtime, &config, &input, &output);
        apply_policy_output(&output);
        vTaskDelay(pdMS_TO_TICKS(loop_period_ms));
    }
}

esp_err_t line_trace_controller_start(const board_profile_t *profile)
{
    ESP_RETURN_ON_FALSE(profile != NULL, ESP_ERR_INVALID_ARG, TAG, "missing board profile");

    BaseType_t ok = xTaskCreate(controller_task, "dcar_liner_ctl", 4096, NULL, 5, NULL);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
