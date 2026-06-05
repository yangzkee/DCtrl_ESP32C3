#include "param_store.h"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"

typedef struct {
    param_definition_t definition;
    param_value_t value;
} param_entry_t;

static const char *TAG = "param_store";
static SemaphoreHandle_t s_lock;
static nvs_handle_t s_nvs;
static bool s_nvs_open;
static uint32_t s_version;

static param_entry_t s_params[] = {
    {
        .definition = {
            .id = "pid.kp",
            .nvs_key = "pid_kp",
            .label = "PID P gain",
            .unit = "mdeg/s",
            .type = PARAM_TYPE_FLOAT,
            .min_value = {.f32 = 7000.0f},
            .max_value = {.f32 = 11000.0f},
            .default_value = {.f32 = 9000.0f},
            .persistent = true,
        },
    },
    {
        .definition = {
            .id = "pid.ki",
            .nvs_key = "pid_ki",
            .label = "PID I gain",
            .unit = "mdeg/s",
            .type = PARAM_TYPE_FLOAT,
            .min_value = {.f32 = 0.0f},
            .max_value = {.f32 = 300.0f},
            .default_value = {.f32 = 0.0f},
            .persistent = true,
        },
    },
    {
        .definition = {
            .id = "pid.kd",
            .nvs_key = "pid_kd",
            .label = "PID D gain",
            .unit = "mdeg/s",
            .type = PARAM_TYPE_FLOAT,
            .min_value = {.f32 = 0.0f},
            .max_value = {.f32 = 3000.0f},
            .default_value = {.f32 = 0.0f},
            .persistent = true,
        },
    },
    {
        .definition = {
            .id = "pid.integral_limit",
            .nvs_key = "pid_ilim",
            .label = "PID integral limit",
            .unit = "offset",
            .type = PARAM_TYPE_FLOAT,
            .min_value = {.f32 = 0.0f},
            .max_value = {.f32 = 200.0f},
            .default_value = {.f32 = 40.0f},
            .persistent = true,
        },
    },
    {
        .definition = {
            .id = "speed.gear",
            .nvs_key = "spd_gear",
            .label = "Speed gear",
            .unit = "",
            .type = PARAM_TYPE_INT,
            .min_value = {.i32 = 1},
            .max_value = {.i32 = 3},
            .default_value = {.i32 = 2},
            .persistent = true,
        },
    },
    {
        .definition = {
            .id = "control.base_speed_mm_s",
            .nvs_key = "base_spd",
            .label = "Base speed",
            .unit = "mm/s",
            .type = PARAM_TYPE_INT,
            .min_value = {.i32 = 0},
            .max_value = {.i32 = 1000},
            .default_value = {.i32 = 600},
            .persistent = true,
        },
    },
    {
        .definition = {
            .id = "control.max_turn_mdeg_s",
            .nvs_key = "max_turn",
            .label = "Max turn delta per command",
            .unit = "mdeg/cmd",
            .type = PARAM_TYPE_INT,
            .min_value = {.i32 = 0},
            .max_value = {.i32 = 10000},
            .default_value = {.i32 = 10000},
            .persistent = true,
        },
    },
    {
        .definition = {
            .id = "control.loop_period_ms",
            .nvs_key = "loop_ms",
            .label = "Control loop period",
            .unit = "ms",
            .type = PARAM_TYPE_INT,
            .min_value = {.i32 = 5},
            .max_value = {.i32 = 100},
            .default_value = {.i32 = 20},
            .persistent = true,
        },
    },
    {
        .definition = {
            .id = "sensor.timeout_ms",
            .nvs_key = "sns_to",
            .label = "Sensor timeout",
            .unit = "ms",
            .type = PARAM_TYPE_INT,
            .min_value = {.i32 = 5},
            .max_value = {.i32 = 500},
            .default_value = {.i32 = 50},
            .persistent = true,
        },
    },
    {
        .definition = {
            .id = "sensor.invert_bits",
            .nvs_key = "sns_inv",
            .label = "Treat white sensor value as active",
            .unit = "",
            .type = PARAM_TYPE_BOOL,
            .min_value = {.boolean = false},
            .max_value = {.boolean = true},
            .default_value = {.boolean = false},
            .persistent = true,
        },
    },
};

static size_t entry_count(void)
{
    return sizeof(s_params) / sizeof(s_params[0]);
}

static void lock_store(void)
{
    if (s_lock != NULL) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
}

static void unlock_store(void)
{
    if (s_lock != NULL) {
        xSemaphoreGive(s_lock);
    }
}

static bool value_in_range(const param_definition_t *definition, param_value_t value)
{
    switch (definition->type) {
    case PARAM_TYPE_INT:
        return value.i32 >= definition->min_value.i32 && value.i32 <= definition->max_value.i32;
    case PARAM_TYPE_FLOAT:
        return value.f32 >= definition->min_value.f32 && value.f32 <= definition->max_value.f32;
    case PARAM_TYPE_BOOL:
        return true;
    default:
        return false;
    }
}

static param_entry_t *find_entry(const char *id)
{
    if (id == NULL) {
        return NULL;
    }

    for (size_t i = 0; i < entry_count(); ++i) {
        if (strcmp(s_params[i].definition.id, id) == 0) {
            return &s_params[i];
        }
    }
    return NULL;
}

static void apply_defaults(void)
{
    for (size_t i = 0; i < entry_count(); ++i) {
        s_params[i].value = s_params[i].definition.default_value;
    }
}

static void load_persisted_values(void)
{
    if (!s_nvs_open) {
        return;
    }

    for (size_t i = 0; i < entry_count(); ++i) {
        param_entry_t *entry = &s_params[i];
        const param_definition_t *definition = &entry->definition;
        if (!definition->persistent) {
            continue;
        }

        esp_err_t err = ESP_ERR_NOT_FOUND;
        param_value_t value = entry->value;

        switch (definition->type) {
        case PARAM_TYPE_INT:
            err = nvs_get_i32(s_nvs, definition->nvs_key, &value.i32);
            break;
        case PARAM_TYPE_FLOAT: {
            size_t length = sizeof(value.f32);
            err = nvs_get_blob(s_nvs, definition->nvs_key, &value.f32, &length);
            if (err == ESP_OK && length != sizeof(value.f32)) {
                err = ESP_ERR_NVS_INVALID_LENGTH;
            }
            break;
        }
        case PARAM_TYPE_BOOL: {
            uint8_t stored = 0;
            err = nvs_get_u8(s_nvs, definition->nvs_key, &stored);
            value.boolean = stored != 0;
            break;
        }
        default:
            break;
        }

        if (err == ESP_OK && value_in_range(definition, value)) {
            entry->value = value;
        } else if (err != ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "ignore invalid persisted param %s: %s", definition->id, esp_err_to_name(err));
        }
    }
}

esp_err_t param_store_init(void)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(s_lock != NULL, ESP_ERR_NO_MEM, TAG, "create mutex");
    }

    lock_store();
    apply_defaults();

    esp_err_t err = nvs_open("line_params", NVS_READWRITE, &s_nvs);
    if (err == ESP_OK) {
        s_nvs_open = true;
        load_persisted_values();
    } else {
        ESP_LOGW(TAG, "NVS param namespace unavailable: %s", esp_err_to_name(err));
    }

    s_version++;
    unlock_store();
    return ESP_OK;
}

size_t param_store_count(void)
{
    return entry_count();
}

const param_definition_t *param_store_get_definition(size_t index)
{
    if (index >= entry_count()) {
        return NULL;
    }
    return &s_params[index].definition;
}

const param_definition_t *param_store_find_definition(const char *id)
{
    param_entry_t *entry = find_entry(id);
    return entry == NULL ? NULL : &entry->definition;
}

esp_err_t param_store_get(const char *id, param_value_t *value)
{
    ESP_RETURN_ON_FALSE(value != NULL, ESP_ERR_INVALID_ARG, TAG, "missing value output");

    lock_store();
    param_entry_t *entry = find_entry(id);
    if (entry == NULL) {
        unlock_store();
        return ESP_ERR_NOT_FOUND;
    }
    *value = entry->value;
    unlock_store();
    return ESP_OK;
}

esp_err_t param_store_set(const char *id, param_value_t value)
{
    lock_store();
    param_entry_t *entry = find_entry(id);
    if (entry == NULL) {
        unlock_store();
        return ESP_ERR_NOT_FOUND;
    }

    if (!value_in_range(&entry->definition, value)) {
        unlock_store();
        return ESP_ERR_INVALID_ARG;
    }

    entry->value = value;
    s_version++;
    unlock_store();
    return ESP_OK;
}

esp_err_t param_store_get_int(const char *id, int32_t *value)
{
    param_value_t raw = {0};
    ESP_RETURN_ON_FALSE(value != NULL, ESP_ERR_INVALID_ARG, TAG, "missing int output");
    ESP_RETURN_ON_ERROR(param_store_get(id, &raw), TAG, "get int");
    *value = raw.i32;
    return ESP_OK;
}

esp_err_t param_store_get_float(const char *id, float *value)
{
    param_value_t raw = {0};
    ESP_RETURN_ON_FALSE(value != NULL, ESP_ERR_INVALID_ARG, TAG, "missing float output");
    ESP_RETURN_ON_ERROR(param_store_get(id, &raw), TAG, "get float");
    *value = raw.f32;
    return ESP_OK;
}

esp_err_t param_store_get_bool(const char *id, bool *value)
{
    param_value_t raw = {0};
    ESP_RETURN_ON_FALSE(value != NULL, ESP_ERR_INVALID_ARG, TAG, "missing bool output");
    ESP_RETURN_ON_ERROR(param_store_get(id, &raw), TAG, "get bool");
    *value = raw.boolean;
    return ESP_OK;
}

esp_err_t param_store_save(void)
{
    ESP_RETURN_ON_FALSE(s_nvs_open, ESP_ERR_INVALID_STATE, TAG, "NVS not open");

    lock_store();
    for (size_t i = 0; i < entry_count(); ++i) {
        param_entry_t *entry = &s_params[i];
        const param_definition_t *definition = &entry->definition;
        if (!definition->persistent) {
            continue;
        }

        esp_err_t err = ESP_OK;
        switch (definition->type) {
        case PARAM_TYPE_INT:
            err = nvs_set_i32(s_nvs, definition->nvs_key, entry->value.i32);
            break;
        case PARAM_TYPE_FLOAT:
            err = nvs_set_blob(s_nvs, definition->nvs_key, &entry->value.f32, sizeof(entry->value.f32));
            break;
        case PARAM_TYPE_BOOL:
            err = nvs_set_u8(s_nvs, definition->nvs_key, entry->value.boolean ? 1 : 0);
            break;
        default:
            err = ESP_ERR_INVALID_ARG;
            break;
        }

        if (err != ESP_OK) {
            unlock_store();
            return err;
        }
    }

    esp_err_t commit_err = nvs_commit(s_nvs);
    unlock_store();
    return commit_err;
}

esp_err_t param_store_reset_defaults(void)
{
    lock_store();
    apply_defaults();
    s_version++;
    unlock_store();
    return ESP_OK;
}

uint32_t param_store_version(void)
{
    return s_version;
}
