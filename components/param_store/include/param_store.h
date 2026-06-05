#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PARAM_TYPE_INT = 0,
    PARAM_TYPE_FLOAT,
    PARAM_TYPE_BOOL,
} param_type_t;

typedef union {
    int32_t i32;
    float f32;
    bool boolean;
} param_value_t;

typedef struct {
    const char *id;
    const char *nvs_key;
    const char *label;
    const char *unit;
    param_type_t type;
    param_value_t min_value;
    param_value_t max_value;
    param_value_t default_value;
    bool persistent;
} param_definition_t;

esp_err_t param_store_init(void);
size_t param_store_count(void);
const param_definition_t *param_store_get_definition(size_t index);
const param_definition_t *param_store_find_definition(const char *id);
esp_err_t param_store_get(const char *id, param_value_t *value);
esp_err_t param_store_set(const char *id, param_value_t value);
esp_err_t param_store_get_int(const char *id, int32_t *value);
esp_err_t param_store_get_float(const char *id, float *value);
esp_err_t param_store_get_bool(const char *id, bool *value);
esp_err_t param_store_save(void);
esp_err_t param_store_reset_defaults(void);
uint32_t param_store_version(void);

#ifdef __cplusplus
}
#endif
