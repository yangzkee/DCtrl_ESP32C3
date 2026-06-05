#include "debug_protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "chassis_uart.h"
#include "esp_log.h"
#include "param_store.h"
#include "telemetry.h"
#include "vehicle_state.h"

static const char *TAG = "debug_protocol";

static const char *type_name(param_type_t type)
{
    switch (type) {
    case PARAM_TYPE_INT:
        return "int";
    case PARAM_TYPE_FLOAT:
        return "float";
    case PARAM_TYPE_BOOL:
        return "bool";
    default:
        return "unknown";
    }
}

static void add_param_value(cJSON *object, const char *name, param_type_t type, param_value_t value)
{
    switch (type) {
    case PARAM_TYPE_INT:
        cJSON_AddNumberToObject(object, name, value.i32);
        break;
    case PARAM_TYPE_FLOAT:
        cJSON_AddNumberToObject(object, name, value.f32);
        break;
    case PARAM_TYPE_BOOL:
        cJSON_AddBoolToObject(object, name, value.boolean);
        break;
    default:
        cJSON_AddNullToObject(object, name);
        break;
    }
}

static esp_err_t copy_json_response(cJSON *root, char *response_json, size_t response_size)
{
    char *printed = cJSON_PrintUnformatted(root);
    if (printed == NULL) {
        return ESP_ERR_NO_MEM;
    }

    int written = snprintf(response_json, response_size, "%s", printed);
    cJSON_free(printed);
    return written > 0 && written < (int)response_size ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

static esp_err_t write_error_response(char *response_json, size_t response_size, const char *message)
{
    int written = snprintf(response_json,
                           response_size,
                           "{\"type\":\"error\",\"message\":\"%s\"}",
                           message == NULL ? "unknown" : message);
    return written > 0 && written < (int)response_size ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

static bool tuning_is_active(void)
{
    vehicle_state_snapshot_t snapshot = {0};
    vehicle_state_get_snapshot(&snapshot);
    return snapshot.motion_state == VEHICLE_MOTION_PARAM_TUNING &&
           snapshot.debug_session == VEHICLE_DEBUG_TUNING_ACTIVE;
}

static esp_err_t require_tuning(char *response_json, size_t response_size)
{
    if (tuning_is_active()) {
        return ESP_OK;
    }
    return write_error_response(response_json, response_size, "enter_tuning_required");
}

static esp_err_t write_state_response(char *response_json, size_t response_size, const char *type)
{
    vehicle_state_snapshot_t snapshot = {0};
    vehicle_state_get_snapshot(&snapshot);

    int written = snprintf(response_json,
                           response_size,
                           "{\"type\":\"%s\",\"motion_state\":\"%s\","
                           "\"debug_session\":\"%s\",\"fault\":\"%s\"}",
                           type,
                           vehicle_motion_state_name(snapshot.motion_state),
                           vehicle_debug_session_name(snapshot.debug_session),
                           vehicle_fault_reason_name(snapshot.fault_reason));
    return written > 0 && written < (int)response_size ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

static esp_err_t write_state_error(char *response_json, size_t response_size, const char *message)
{
    vehicle_state_snapshot_t snapshot = {0};
    vehicle_state_get_snapshot(&snapshot);

    int written = snprintf(response_json,
                           response_size,
                           "{\"type\":\"error\",\"message\":\"%s\","
                           "\"motion_state\":\"%s\",\"debug_session\":\"%s\",\"fault\":\"%s\"}",
                           message,
                           vehicle_motion_state_name(snapshot.motion_state),
                           vehicle_debug_session_name(snapshot.debug_session),
                           vehicle_fault_reason_name(snapshot.fault_reason));
    return written > 0 && written < (int)response_size ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

static esp_err_t handle_get_schema(char *response_json, size_t response_size)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return ESP_ERR_NO_MEM;
    }

    cJSON *params = cJSON_AddArrayToObject(root, "params");
    if (params == NULL) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddStringToObject(root, "type", "schema");
    cJSON_AddNumberToObject(root, "version", param_store_version());

    for (size_t i = 0; i < param_store_count(); ++i) {
        const param_definition_t *definition = param_store_get_definition(i);
        param_value_t value = {0};
        if (definition == NULL || param_store_get(definition->id, &value) != ESP_OK) {
            continue;
        }

        cJSON *item = cJSON_CreateObject();
        if (item == NULL) {
            cJSON_Delete(root);
            return ESP_ERR_NO_MEM;
        }
        cJSON_AddItemToArray(params, item);
        cJSON_AddStringToObject(item, "id", definition->id);
        cJSON_AddStringToObject(item, "label", definition->label);
        cJSON_AddStringToObject(item, "unit", definition->unit);
        cJSON_AddStringToObject(item, "type", type_name(definition->type));
        cJSON_AddBoolToObject(item, "persistent", definition->persistent);
        add_param_value(item, "min", definition->type, definition->min_value);
        add_param_value(item, "max", definition->type, definition->max_value);
        add_param_value(item, "default", definition->type, definition->default_value);
        add_param_value(item, "value", definition->type, value);
    }

    esp_err_t err = copy_json_response(root, response_json, response_size);
    cJSON_Delete(root);
    return err;
}

static esp_err_t handle_get_params(char *response_json, size_t response_size)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return ESP_ERR_NO_MEM;
    }

    cJSON *params = cJSON_AddObjectToObject(root, "params");
    if (params == NULL) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddStringToObject(root, "type", "params");
    cJSON_AddNumberToObject(root, "version", param_store_version());

    for (size_t i = 0; i < param_store_count(); ++i) {
        const param_definition_t *definition = param_store_get_definition(i);
        param_value_t value = {0};
        if (definition == NULL || param_store_get(definition->id, &value) != ESP_OK) {
            continue;
        }
        add_param_value(params, definition->id, definition->type, value);
    }

    esp_err_t err = copy_json_response(root, response_json, response_size);
    cJSON_Delete(root);
    return err;
}

static esp_err_t handle_set_param(cJSON *root, char *response_json, size_t response_size)
{
    esp_err_t tuning_err = require_tuning(response_json, response_size);
    if (tuning_err != ESP_OK || !tuning_is_active()) {
        return tuning_err;
    }

    const cJSON *id = cJSON_GetObjectItemCaseSensitive(root, "id");
    const cJSON *value = cJSON_GetObjectItemCaseSensitive(root, "value");
    if (!cJSON_IsString(id) || value == NULL) {
        return write_error_response(response_json, response_size, "missing id or value");
    }

    const param_definition_t *definition = param_store_find_definition(id->valuestring);
    if (definition == NULL) {
        return write_error_response(response_json, response_size, "unknown parameter");
    }

    param_value_t raw = {0};
    switch (definition->type) {
    case PARAM_TYPE_INT:
        if (!cJSON_IsNumber(value)) {
            return write_error_response(response_json, response_size, "value must be number");
        }
        raw.i32 = (int32_t)value->valuedouble;
        break;
    case PARAM_TYPE_FLOAT:
        if (!cJSON_IsNumber(value)) {
            return write_error_response(response_json, response_size, "value must be number");
        }
        raw.f32 = (float)value->valuedouble;
        break;
    case PARAM_TYPE_BOOL:
        if (!cJSON_IsBool(value)) {
            return write_error_response(response_json, response_size, "value must be bool");
        }
        raw.boolean = cJSON_IsTrue(value);
        break;
    default:
        return write_error_response(response_json, response_size, "unsupported parameter type");
    }

    esp_err_t err = param_store_set(definition->id, raw);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "set param %s failed: %s", definition->id, esp_err_to_name(err));
        return write_error_response(response_json, response_size, "parameter value out of range");
    }

    cJSON *out = cJSON_CreateObject();
    cJSON_AddStringToObject(out, "type", "param_updated");
    cJSON_AddStringToObject(out, "id", definition->id);
    cJSON_AddNumberToObject(out, "version", param_store_version());
    add_param_value(out, "value", definition->type, raw);
    err = copy_json_response(out, response_json, response_size);
    cJSON_Delete(out);
    return err;
}

static esp_err_t handle_state_transition(esp_err_t (*transition)(void),
                                         char *response_json,
                                         size_t response_size,
                                         const char *response_type,
                                         const char *error_message)
{
    esp_err_t err = transition();
    if (err != ESP_OK) {
        return write_state_error(response_json, response_size, error_message);
    }
    return write_state_response(response_json, response_size, response_type);
}

static esp_err_t handle_stop(char *response_json, size_t response_size)
{
    vehicle_state_stop();
    return write_state_response(response_json, response_size, "stopped");
}

static esp_err_t handle_manual_motion(cJSON *root, char *response_json, size_t response_size)
{
    if (!tuning_is_active()) {
        return write_error_response(response_json, response_size, "enter_tuning_required");
    }

    const int32_t default_linear = 500;
    const int32_t default_angular = 1500;
    const int32_t max_linear = 1500;
    const int32_t max_angular = 10000;
    const int32_t min_duration_ms = 50;
    const int32_t max_duration_ms = 3000;
    int32_t duration_ms = 500;
    chassis_motion_cmd_t cmd = {0};

    const cJSON *duration = cJSON_GetObjectItemCaseSensitive(root, "duration_ms");
    if (cJSON_IsNumber(duration)) {
        duration_ms = (int32_t)duration->valuedouble;
    }
    if (duration_ms < min_duration_ms || duration_ms > max_duration_ms) {
        return write_error_response(response_json, response_size, "manual duration out of range");
    }

    const cJSON *motion = cJSON_GetObjectItemCaseSensitive(root, "motion");
    if (cJSON_IsString(motion)) {
        if (strcmp(motion->valuestring, "forward") == 0) {
            cmd.linear_mm_s = default_linear;
        } else if (strcmp(motion->valuestring, "backward") == 0) {
            cmd.linear_mm_s = -default_linear;
        } else if (strcmp(motion->valuestring, "left") == 0) {
            cmd.angular_mdeg_s = default_angular;
        } else if (strcmp(motion->valuestring, "right") == 0) {
            cmd.angular_mdeg_s = -default_angular;
        } else if (strcmp(motion->valuestring, "stop") == 0) {
            cmd.linear_mm_s = 0;
            cmd.angular_mdeg_s = 0;
        } else {
            return write_error_response(response_json, response_size, "unknown manual motion");
        }
    } else {
        const cJSON *linear = cJSON_GetObjectItemCaseSensitive(root, "linear_mm_s");
        const cJSON *angular = cJSON_GetObjectItemCaseSensitive(root, "angular_mdeg_s");
        if (!cJSON_IsNumber(linear) && !cJSON_IsNumber(angular)) {
            return write_error_response(response_json, response_size, "missing manual motion");
        }
        cmd.linear_mm_s = cJSON_IsNumber(linear) ? (int32_t)linear->valuedouble : 0;
        cmd.angular_mdeg_s = cJSON_IsNumber(angular) ? (int32_t)angular->valuedouble : 0;
    }

    if (cmd.linear_mm_s < -max_linear || cmd.linear_mm_s > max_linear ||
        cmd.angular_mdeg_s < -max_angular || cmd.angular_mdeg_s > max_angular) {
        return write_error_response(response_json, response_size, "manual motion exceeds safe limit");
    }

    esp_err_t err = vehicle_state_start_manual_test(&cmd, (uint32_t)duration_ms);
    if (err != ESP_OK) {
        return write_state_error(response_json, response_size, "manual_motion_invalid_state");
    }
    return write_state_response(response_json, response_size, "manual_motion_started");
}

static esp_err_t handle_chassis_diag(cJSON *root, char *response_json, size_t response_size)
{
    const cJSON *command = cJSON_GetObjectItemCaseSensitive(root, "command");
    const char *command_text = cJSON_IsString(command) ? command->valuestring : "status";

    chassis_diag_result_t result = {0};
    esp_err_t diag_err = chassis_uart_diag_command(command_text, &result);

    cJSON *out = cJSON_CreateObject();
    if (out == NULL) {
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddStringToObject(out, "type", "chassis_diag");
    cJSON_AddStringToObject(out, "command", result.command);
    cJSON_AddBoolToObject(out, "ok", diag_err == ESP_OK);
    cJSON_AddBoolToObject(out, "ready", result.ready);
    cJSON_AddNumberToObject(out, "tx_pin", result.tx_pin);
    cJSON_AddNumberToObject(out, "rx_pin", result.rx_pin);
    cJSON_AddNumberToObject(out, "baud", result.baud_rate);
    cJSON_AddNumberToObject(out, "linear_protocol_per_real", chassis_uart_linear_protocol_per_real());
    cJSON_AddNumberToObject(out, "tx_bytes", result.tx_bytes);
    cJSON_AddNumberToObject(out, "rx_bytes", result.rx_bytes);
    cJSON_AddNumberToObject(out, "frames", result.frames);
    cJSON_AddNumberToObject(out, "checksum_ok", result.checksum_ok);
    cJSON_AddNumberToObject(out, "checksum_bad", result.checksum_bad);
    cJSON_AddNumberToObject(out, "last_target", result.last_target);
    cJSON_AddNumberToObject(out, "last_source", result.last_source);
    cJSON_AddNumberToObject(out, "last_a", result.last_a);
    cJSON_AddNumberToObject(out, "last_b", result.last_b);
    cJSON_AddNumberToObject(out, "last_len", result.last_len);
    cJSON_AddBoolToObject(out, "last_checksum_ok", result.last_checksum_ok);
    cJSON_AddStringToObject(out, "last_payload_hex", result.last_payload_hex);
    cJSON_AddStringToObject(out, "last_error", result.last_error);
    esp_err_t err = copy_json_response(out, response_json, response_size);
    cJSON_Delete(out);
    return err;
}

esp_err_t debug_protocol_handle_message(const char *request_json, char *response_json, size_t response_size)
{
    if (request_json == NULL || response_json == NULL || response_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *root = cJSON_Parse(request_json);
    if (root == NULL) {
        return write_error_response(response_json, response_size, "invalid json");
    }

    const cJSON *type = cJSON_GetObjectItemCaseSensitive(root, "type");
    if (!cJSON_IsString(type)) {
        cJSON_Delete(root);
        return write_error_response(response_json, response_size, "missing type");
    }

    esp_err_t err = ESP_OK;
    if (strcmp(type->valuestring, "get_schema") == 0) {
        err = handle_get_schema(response_json, response_size);
    } else if (strcmp(type->valuestring, "get_params") == 0) {
        err = handle_get_params(response_json, response_size);
    } else if (strcmp(type->valuestring, "set_param") == 0) {
        err = handle_set_param(root, response_json, response_size);
    } else if (strcmp(type->valuestring, "save_params") == 0) {
        err = require_tuning(response_json, response_size);
        if (err == ESP_OK && tuning_is_active()) {
            err = param_store_save();
            if (err == ESP_OK) {
                snprintf(response_json, response_size, "{\"type\":\"params_saved\",\"version\":%lu}", (unsigned long)param_store_version());
            } else {
                write_error_response(response_json, response_size, "save failed");
            }
        }
    } else if (strcmp(type->valuestring, "reset_defaults") == 0) {
        err = require_tuning(response_json, response_size);
        if (err == ESP_OK && tuning_is_active()) {
            param_store_reset_defaults();
            snprintf(response_json, response_size, "{\"type\":\"defaults_restored\",\"version\":%lu}", (unsigned long)param_store_version());
        }
    } else if (strcmp(type->valuestring, "enter_tuning") == 0) {
        err = handle_state_transition(vehicle_state_enter_tuning,
                                      response_json,
                                      response_size,
                                      "tuning_entered",
                                      "enter_tuning_invalid_state");
    } else if (strcmp(type->valuestring, "exit_tuning") == 0) {
        err = handle_state_transition(vehicle_state_exit_tuning,
                                      response_json,
                                      response_size,
                                      "tuning_exited",
                                      "exit_tuning_invalid_state");
    } else if (strcmp(type->valuestring, "arm_auto") == 0) {
        err = handle_state_transition(vehicle_state_arm_auto,
                                      response_json,
                                      response_size,
                                      "auto_armed",
                                      "arm_auto_invalid_state");
    } else if (strcmp(type->valuestring, "start_auto") == 0) {
        err = handle_state_transition(vehicle_state_start_auto,
                                      response_json,
                                      response_size,
                                      "auto_started",
                                      "start_auto_invalid_state");
    } else if (strcmp(type->valuestring, "stop") == 0) {
        err = handle_stop(response_json, response_size);
    } else if (strcmp(type->valuestring, "clear_fault") == 0) {
        err = handle_state_transition(vehicle_state_clear_fault,
                                      response_json,
                                      response_size,
                                      "fault_cleared",
                                      "clear_fault_invalid_state");
    } else if (strcmp(type->valuestring, "manual_motion") == 0) {
        err = handle_manual_motion(root, response_json, response_size);
    } else if (strcmp(type->valuestring, "chassis_diag") == 0) {
        err = handle_chassis_diag(root, response_json, response_size);
    } else if (strcmp(type->valuestring, "get_telemetry") == 0) {
        err = telemetry_build_json(response_json, response_size);
    } else {
        err = write_error_response(response_json, response_size, "unknown message type");
    }

    cJSON_Delete(root);
    return err;
}
