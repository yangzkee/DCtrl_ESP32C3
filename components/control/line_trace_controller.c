#include "line_trace_controller.h"

#include <string.h>

#include "chassis_uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "line_sensor_uart.h"
#include "line_trace_policy.h"
#include "motion_contracts.h"
#include "motion_inputs.h"
#include "param_store.h"
#include "telemetry.h"
#include "vehicle_state.h"

static const char *TAG = "DCar-LinerCtl";
static portMUX_TYPE s_task_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_controller_task;
typedef enum {
    CONTROLLER_TASK_STOPPED = 0,
    CONTROLLER_TASK_STARTING,
    CONTROLLER_TASK_RUNNING,
} controller_task_state_t;
static controller_task_state_t s_controller_task_state = CONTROLLER_TASK_STOPPED;

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

static bool get_bool_param(const char *id, bool fallback)
{
    bool value = fallback;
    ESP_ERROR_CHECK_WITHOUT_ABORT(param_store_get_bool(id, &value));
    return value;
}

static void read_policy_config(line_trace_policy_config_t *config)
{
    const int32_t gear = get_int_param("speed.gear", 2);
    int32_t base_speed_mm_s = 600;
    int32_t max_turn_mdeg_s = 10000;
    motion_speed_profile_from_gear(gear, &base_speed_mm_s, &max_turn_mdeg_s);

    config->base_speed_mm_s = base_speed_mm_s;
    config->max_turn_mdeg_s = max_turn_mdeg_s;
    config->kp = get_float_param("pid.kp", 9000.0f);
    config->ki = get_float_param("pid.ki", 0.0f);
    config->kd = get_float_param("pid.kd", 0.0f);
    config->integral_limit = get_float_param("pid.integral_limit", 40.0f);
}

static void update_policy_telemetry(const line_trace_policy_output_t *output)
{
    if (output == NULL) {
        return;
    }

    telemetry_update_motion_cmd(&output->plan.chassis_cmd);
    telemetry_controller_state_t state = {
        .lost_line = output->lost_line,
        .line_quality = output->line_quality,
        .active_sensor_count = output->active_sensor_count,
        .pid_output_mdeg_s = output->pid_output_mdeg_s,
        .recovery_angle_mdeg = output->recovery_angle_mdeg,
        .recovery_target_mdeg = output->recovery_target_mdeg,
        .recovery_direction_mdeg = output->recovery_direction_mdeg,
        .recovery_segment_index = output->recovery_segment_index,
        .recovery_elapsed_ms = output->recovery_elapsed_ms,
        .recovery_target_ms = output->recovery_target_ms,
    };
    strlcpy(state.line_phase,
            line_trace_policy_phase_name(output->phase),
            sizeof(state.line_phase));
    strlcpy(state.recovery_relation,
            line_trace_recovery_relation_name(output->recovery_relation),
            sizeof(state.recovery_relation));
    strlcpy(state.recovery_stage,
            line_trace_recovery_stage_name(output->recovery_stage),
            sizeof(state.recovery_stage));
    telemetry_update_controller_state(&state);
}

static void apply_policy_output(const line_trace_policy_output_t *output)
{
    if (output == NULL) {
        return;
    }

    update_policy_telemetry(output);
    if (output->plan.vehicle_action == CONTROL_VEHICLE_ACTION_ENTER_FAULT) {
        vehicle_state_enter_fault(output->plan.fault_reason);
    } else if (output->plan.vehicle_action == CONTROL_VEHICLE_ACTION_STOP_TO_IDLE) {
        vehicle_state_stop();
    }

    esp_err_t err = ESP_OK;
    if (output->plan.chassis_action == CONTROL_CHASSIS_ACTION_SEND_MOTION) {
        err = chassis_uart_send_motion(&output->plan.chassis_cmd);
    } else if (output->plan.chassis_action == CONTROL_CHASSIS_ACTION_STOP) {
        err = chassis_uart_stop();
    }

    if (err != ESP_OK) {
        vehicle_state_enter_fault(VEHICLE_FAULT_CHASSIS_SEND_FAILED);
        ESP_ERROR_CHECK_WITHOUT_ABORT(chassis_uart_stop());
    }
}

static bool controller_should_run(const vehicle_state_snapshot_t *vehicle)
{
    if (vehicle == NULL) {
        return false;
    }
    return vehicle->motion_state == VEHICLE_MOTION_MANUAL_TEST ||
           vehicle->motion_state == VEHICLE_MOTION_AUTO_ARMED ||
           vehicle->motion_state == VEHICLE_MOTION_AUTO_RUNNING;
}

static void controller_task(void *arg)
{
    (void)arg;
    line_trace_policy_runtime_t policy_runtime = {0};
    line_trace_policy_init(&policy_runtime);

    while (true) {
        const int32_t timeout_ms = get_int_param("sensor.timeout_ms", 50);
        const int32_t loop_period_ms = get_int_param("control.loop_period_ms", 20);
        const bool manual_expired = vehicle_state_finish_manual_if_expired(vehicle_state_now_ms());
        if (manual_expired) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(chassis_uart_stop());
            const chassis_motion_cmd_t stop_cmd = {0};
            telemetry_update_motion_cmd(&stop_cmd);
        }

        vehicle_state_snapshot_t vehicle = {0};
        vehicle_state_get_snapshot(&vehicle);
        if (!controller_should_run(&vehicle)) {
            ESP_LOGI(TAG, "controller idle; task exiting");
            break;
        }

        line_trace_policy_input_t input = {0};
        motion_inputs_from_vehicle(&input, &vehicle);
        input.now_ms = vehicle_state_now_ms();

        if (input.run_mode != LINE_TRACE_RUN_MANUAL_TEST &&
            vehicle.motion_state != VEHICLE_MOTION_OTA_UPDATE) {
            esp_err_t err = line_sensor_uart_read_sample(&input.sample, timeout_ms);
            if (err == ESP_OK) {
                motion_inputs_apply_line_inversion(&input.sample, get_bool_param("sensor.invert_bits", false));
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

    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
    taskENTER_CRITICAL(&s_task_lock);
    if (s_controller_task_state == CONTROLLER_TASK_STARTING ||
        (s_controller_task_state == CONTROLLER_TASK_RUNNING &&
         s_controller_task == current_task)) {
        s_controller_task = NULL;
        s_controller_task_state = CONTROLLER_TASK_STOPPED;
    }
    taskEXIT_CRITICAL(&s_task_lock);
    vTaskDelete(NULL);
}

esp_err_t line_trace_controller_start(const board_profile_t *profile)
{
    (void)profile;

    taskENTER_CRITICAL(&s_task_lock);
    if (s_controller_task_state != CONTROLLER_TASK_STOPPED) {
        taskEXIT_CRITICAL(&s_task_lock);
        return ESP_OK;
    }
    s_controller_task_state = CONTROLLER_TASK_STARTING;
    taskEXIT_CRITICAL(&s_task_lock);

    TaskHandle_t task = NULL;
    BaseType_t ok = xTaskCreate(controller_task, "dcar_liner_ctl", 4096, NULL, 5, &task);
    if (ok != pdPASS) {
        taskENTER_CRITICAL(&s_task_lock);
        if (s_controller_task_state == CONTROLLER_TASK_STARTING) {
            s_controller_task_state = CONTROLLER_TASK_STOPPED;
            s_controller_task = NULL;
        }
        taskEXIT_CRITICAL(&s_task_lock);
        return ESP_ERR_NO_MEM;
    }

    taskENTER_CRITICAL(&s_task_lock);
    if (s_controller_task_state == CONTROLLER_TASK_STARTING) {
        s_controller_task = task;
        s_controller_task_state = CONTROLLER_TASK_RUNNING;
    }
    taskEXIT_CRITICAL(&s_task_lock);
    return ESP_OK;
}
