#include "debug_server_ble.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "chassis_uart.h"
#include "debug_server.h"
#include "debug_protocol.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "host/ble_att.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "param_store.h"
#include "telemetry.h"
#include "vehicle_state.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "debug_server_ble";

#define BLE_DEVICE_NAME_PREFIX "DCtrl"
#define BLE_REQUEST_BYTES 1024
#define BLE_RESPONSE_BYTES 4096
#define BLE_DIRECT_RESPONSE_BYTES 180
#define BLE_CHUNK_DATA_BYTES 160
#define BLE_CHUNK_JSON_BYTES 512
#define REMOTE_BRIDGE_MAX_WRITE_BYTES 244
#define BLE_NAME_NAMESPACE "dcar_ble"
#define BLE_NAME_KEY "name"
#define BLE_NAME_LEGACY_SUFFIX_KEY "suffix"

static uint8_t s_addr_type;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_tx_value_handle;
static uint16_t s_remote_tx_value_handle;
static bool s_notify_enabled;
static bool s_remote_notify_enabled;
static bool s_started;
static char s_device_name[24] = BLE_DEVICE_NAME_PREFIX;
static nvs_handle_t s_name_nvs;
static bool s_name_nvs_open;
static size_t s_chunk_offset;
static uint32_t s_chunk_seq;
static size_t s_rx_stream_len;
static char s_last_request[BLE_REQUEST_BYTES];
static char s_last_response[BLE_RESPONSE_BYTES] = "{\"type\":\"ble_ready\",\"status\":\"ok\"}";
static char s_rx_stream[BLE_REQUEST_BYTES];

static const ble_uuid128_t s_service_uuid =
    BLE_UUID128_INIT(0x43, 0x52, 0x41, 0x4c, 0x7c, 0x0f, 0xc7, 0xb5,
                     0x9a, 0x4b, 0x4d, 0x8d, 0x01, 0x00, 0x3a, 0x7b);
static const ble_uuid128_t s_rx_uuid =
    BLE_UUID128_INIT(0x43, 0x52, 0x41, 0x4c, 0x7c, 0x0f, 0xc7, 0xb5,
                     0x9a, 0x4b, 0x4d, 0x8d, 0x02, 0x00, 0x3a, 0x7b);
static const ble_uuid128_t s_tx_uuid =
    BLE_UUID128_INIT(0x43, 0x52, 0x41, 0x4c, 0x7c, 0x0f, 0xc7, 0xb5,
                     0x9a, 0x4b, 0x4d, 0x8d, 0x03, 0x00, 0x3a, 0x7b);

static const ble_uuid128_t s_remote_service_uuid =
    BLE_UUID128_INIT(0x01, 0x00, 0x43, 0x4d, 0x1f, 0x7b, 0x67, 0x9c,
                     0x4e, 0x4f, 0x9b, 0x5b, 0x01, 0x00, 0x1f, 0x6d);
static const ble_uuid128_t s_remote_rx_uuid =
    BLE_UUID128_INIT(0x01, 0x00, 0x43, 0x4d, 0x1f, 0x7b, 0x67, 0x9c,
                     0x4e, 0x4f, 0x9b, 0x5b, 0x02, 0x00, 0x1f, 0x6d);
static const ble_uuid128_t s_remote_tx_uuid =
    BLE_UUID128_INIT(0x01, 0x00, 0x43, 0x4d, 0x1f, 0x7b, 0x67, 0x9c,
                     0x4e, 0x4f, 0x9b, 0x5b, 0x03, 0x00, 0x1f, 0x6d);

static int ble_gap_event(struct ble_gap_event *event, void *arg);
static int gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg);
static int process_rx_json(const char *data, size_t len);
static int write_remote_request(struct ble_gatt_access_ctxt *ctxt);

static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[])
        {
            {
                .uuid = &s_rx_uuid.u,
                .access_cb = gatt_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &s_tx_uuid.u,
                .access_cb = gatt_access_cb,
                .val_handle = &s_tx_value_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {0},
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_remote_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[])
        {
            {
                .uuid = &s_remote_rx_uuid.u,
                .access_cb = gatt_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &s_remote_tx_uuid.u,
                .access_cb = gatt_access_cb,
                .val_handle = &s_remote_tx_value_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {0},
        },
    },
    {0},
};

static void notify_response_ready(void)
{
    if (!s_notify_enabled || s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }

    char note[96];
    int written = snprintf(note,
                           sizeof(note),
                           "{\"type\":\"ble_response_ready\",\"bytes\":%u}",
                           (unsigned)strlen(s_last_response));
    if (written <= 0 || written >= (int)sizeof(note)) {
        return;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(note, strlen(note));
    if (om == NULL) {
        return;
    }

    int rc = ble_gatts_notify_custom(s_conn_handle, s_tx_value_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "notify failed rc=%d", rc);
    }
}

static void set_text_response(const char *text)
{
    snprintf(s_last_response, sizeof(s_last_response), "%s", text);
    s_chunk_offset = 0;
    s_chunk_seq = 0;
    ESP_LOGI(TAG, "BLE compact TX %s", s_last_response);
    notify_response_ready();
}

static int read_tx_response(struct ble_gatt_access_ctxt *ctxt)
{
    size_t len = strlen(s_last_response);
    if (len <= BLE_DIRECT_RESPONSE_BYTES) {
        if (ctxt->offset > len) {
            return BLE_ATT_ERR_INVALID_OFFSET;
        }

        int rc = os_mbuf_append(ctxt->om, s_last_response + ctxt->offset, len - ctxt->offset);
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    size_t start = s_chunk_offset < len ? s_chunk_offset : len;
    size_t remaining = len - start;
    size_t raw_chunk = remaining > BLE_CHUNK_DATA_BYTES ? BLE_CHUNK_DATA_BYTES : remaining;
    size_t next_offset = start + raw_chunk;
    bool done = next_offset >= len;

    char chunk_json[BLE_CHUNK_JSON_BYTES];
    int written = snprintf(chunk_json,
                           sizeof(chunk_json),
                           "{\"type\":\"ble_chunk\",\"seq\":%lu,\"offset\":%u,\"total\":%u,"
                           "\"done\":%s,\"data\":\"",
                           (unsigned long)s_chunk_seq,
                           (unsigned)start,
                           (unsigned)len,
                           done ? "true" : "false");
    if (written <= 0 || written >= (int)sizeof(chunk_json)) {
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    size_t pos = (size_t)written;
    for (size_t i = 0; i < raw_chunk; ++i) {
        unsigned char ch = (unsigned char)s_last_response[start + i];
        if (pos + 7 >= sizeof(chunk_json)) {
            return BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (ch == '"' || ch == '\\') {
            chunk_json[pos++] = '\\';
            chunk_json[pos++] = (char)ch;
        } else if (ch == '\n') {
            chunk_json[pos++] = '\\';
            chunk_json[pos++] = 'n';
        } else if (ch == '\r') {
            chunk_json[pos++] = '\\';
            chunk_json[pos++] = 'r';
        } else if (ch == '\t') {
            chunk_json[pos++] = '\\';
            chunk_json[pos++] = 't';
        } else if (ch < 0x20) {
            int esc = snprintf(chunk_json + pos, sizeof(chunk_json) - pos, "\\u%04x", ch);
            if (esc != 6) {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            pos += 6;
        } else {
            chunk_json[pos++] = (char)ch;
        }
    }

    if (pos + 3 >= sizeof(chunk_json)) {
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    chunk_json[pos++] = '"';
    chunk_json[pos++] = '}';
    chunk_json[pos] = '\0';

    size_t chunk_json_len = pos;
    if (ctxt->offset > chunk_json_len) {
        return BLE_ATT_ERR_INVALID_OFFSET;
    }

    int rc = os_mbuf_append(ctxt->om,
                            chunk_json + ctxt->offset,
                            chunk_json_len - ctxt->offset);
    if (rc == 0 && ctxt->offset == 0) {
        s_chunk_offset = next_offset;
        s_chunk_seq++;
    }
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static bool looks_like_complete_json(const char *data, size_t len)
{
    size_t start = 0;
    while (start < len && (data[start] == ' ' || data[start] == '\r' ||
                           data[start] == '\n' || data[start] == '\t')) {
        start++;
    }
    if (start >= len || data[start] != '{') {
        return false;
    }

    size_t end = len;
    while (end > start && (data[end - 1] == ' ' || data[end - 1] == '\r' ||
                           data[end - 1] == '\n' || data[end - 1] == '\t')) {
        end--;
    }
    return end > start && data[end - 1] == '}';
}

static bool is_compact_prefix(char ch)
{
    return ch == 'G' || ch == 'S' || ch == 'W' || ch == 'N' || ch == 'C';
}

static esp_err_t set_gap_device_name(void)
{
    int rc = ble_svc_gap_device_name_set(s_device_name);
    return rc == 0 ? ESP_OK : ESP_FAIL;
}

static esp_err_t reset_device_name(void)
{
    if (!s_name_nvs_open) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = nvs_erase_key(s_name_nvs, BLE_NAME_KEY);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        return err;
    }
    esp_err_t legacy_err = nvs_erase_key(s_name_nvs, BLE_NAME_LEGACY_SUFFIX_KEY);
    if (legacy_err != ESP_OK && legacy_err != ESP_ERR_NVS_NOT_FOUND) {
        return legacy_err;
    }
    err = nvs_commit(s_name_nvs);
    if (err != ESP_OK) {
        return err;
    }

    snprintf(s_device_name, sizeof(s_device_name), "%s", BLE_DEVICE_NAME_PREFIX);
    return set_gap_device_name();
}

static int read_pid_param(const char *id, int fallback)
{
    float value = (float)fallback;
    ESP_ERROR_CHECK_WITHOUT_ABORT(param_store_get_float(id, &value));
    return (int)(value + 0.5f);
}

static int read_gear_param(void)
{
    int32_t value = 2;
    ESP_ERROR_CHECK_WITHOUT_ABORT(param_store_get_int("speed.gear", &value));
    return (int)value;
}

static int write_pid_param(const char *id, int value)
{
    param_value_t param_value = {
        .f32 = (float)value,
    };
    return param_store_set(id, param_value);
}

static int write_gear_param(int value)
{
    param_value_t param_value = {
        .i32 = value,
    };
    return param_store_set("speed.gear", param_value);
}

static int process_rx_compact(const char *data, size_t len)
{
    size_t start = 0;
    while (start < len && (data[start] == ' ' || data[start] == '\r' ||
                           data[start] == '\n' || data[start] == '\t')) {
        start++;
    }
    size_t end = len;
    while (end > start && (data[end - 1] == ' ' || data[end - 1] == '\r' ||
                           data[end - 1] == '\n' || data[end - 1] == '\t')) {
        end--;
    }
    if (end <= start || !is_compact_prefix(data[start])) {
        return process_rx_json(data, len);
    }
    if (end - start >= 64) {
        set_text_response("E:PARSE\n");
        return 0;
    }

    char frame[64];
    memcpy(frame, data + start, end - start);
    frame[end - start] = '\0';
    ESP_LOGI(TAG, "BLE compact RX %s", frame);

    if (strcmp(frame, "G") == 0) {
        char response[40];
        snprintf(response,
                 sizeof(response),
                 "P%d,%d,%d,%d\n",
                 read_pid_param("pid.kp", 9000),
                 read_pid_param("pid.ki", 0),
                 read_pid_param("pid.kd", 0),
                 read_gear_param());
        set_text_response(response);
        return 0;
    }

    if (strcmp(frame, "W1") == 0) {
        esp_err_t err = debug_server_start_wifi_fallback();
        set_text_response(err == ESP_OK ? "OK\n" : "E:BUSY\n");
        return 0;
    }

    if (frame[0] == 'C') {
        const char *command = frame + 1;
        if (command[0] == '\0' || strcmp(command, "?") == 0) {
            command = "status";
        }
        esp_err_t err = chassis_uart_diag_command_text(command, s_last_response, sizeof(s_last_response));
        if (err == ESP_ERR_INVALID_ARG) {
            set_text_response("E:CHASSIS_CMD\n");
        } else if (err != ESP_OK && s_last_response[0] == '\0') {
            set_text_response("E:CHASSIS\n");
        } else {
            s_chunk_offset = 0;
            s_chunk_seq = 0;
            notify_response_ready();
        }
        return 0;
    }

    if (strcmp(frame, "N") == 0) {
        char response[40];
        snprintf(response, sizeof(response), "N%s\n", s_device_name);
        set_text_response(response);
        return 0;
    }

    if (strncmp(frame, "N=", 2) == 0) {
        esp_err_t err = ESP_OK;
        if (strcmp(frame + 2, "*") == 0) {
            err = reset_device_name();
        } else {
            set_text_response("E:FIXED\n");
            return 0;
        }
        set_text_response(err == ESP_OK ? "OK\n" : "E:SAVE\n");
        return 0;
    }

    if (frame[0] != 'S') {
        set_text_response("E:PARSE\n");
        return 0;
    }

    int kp = 0;
    int ki = 0;
    int kd = 0;
    int gear = 0;
    char extra = '\0';
    int parsed = sscanf(frame + 1, "%d,%d,%d,%d%c", &kp, &ki, &kd, &gear, &extra);
    if (parsed != 4) {
        set_text_response("E:PARSE\n");
        return 0;
    }

    esp_err_t tune_err = vehicle_state_enter_tuning();
    if (tune_err != ESP_OK) {
        vehicle_state_snapshot_t snapshot = {0};
        vehicle_state_get_snapshot(&snapshot);
        set_text_response(snapshot.motion_state == VEHICLE_MOTION_FAULT ? "E:FAULT\n" : "E:BUSY\n");
        return 0;
    }

    esp_err_t err = ESP_OK;
    err = write_pid_param("pid.kp", kp);
    if (err == ESP_OK) {
        err = write_pid_param("pid.ki", ki);
    }
    if (err == ESP_OK) {
        err = write_pid_param("pid.kd", kd);
    }
    if (err == ESP_OK) {
        err = write_gear_param(gear);
    }
    if (err == ESP_OK) {
        err = param_store_save();
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(vehicle_state_exit_tuning());

    if (err == ESP_ERR_INVALID_ARG || err == ESP_ERR_INVALID_STATE) {
        set_text_response("E:RANGE\n");
    } else if (err != ESP_OK) {
        set_text_response("E:SAVE\n");
    } else {
        set_text_response("OK\n");
    }
    return 0;
}

static int process_rx_frame(const char *data, size_t len)
{
    size_t start = 0;
    while (start < len && (data[start] == ' ' || data[start] == '\r' ||
                           data[start] == '\n' || data[start] == '\t')) {
        start++;
    }
    if (start < len && is_compact_prefix(data[start])) {
        return process_rx_compact(data, len);
    }
    return process_rx_json(data, len);
}

static int process_rx_json(const char *data, size_t len)
{
    if (len <= 0 || len >= BLE_REQUEST_BYTES) {
        snprintf(s_last_response,
                 sizeof(s_last_response),
                 "{\"type\":\"error\",\"message\":\"ble request too large\",\"max\":%u}",
                 (unsigned)(BLE_REQUEST_BYTES - 1));
        s_rx_stream_len = 0;
        notify_response_ready();
        return len == 0 ? BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    memcpy(s_last_request, data, len);
    s_last_request[len] = '\0';
    ESP_LOGI(TAG, "BLE RX %s", s_last_request);

    esp_err_t err = debug_protocol_handle_message(s_last_request, s_last_response, sizeof(s_last_response));
    if (err != ESP_OK) {
        snprintf(s_last_response,
                 sizeof(s_last_response),
                 "{\"type\":\"error\",\"message\":\"protocol failed\",\"esp_err\":%d}",
                 err);
    }
    s_chunk_offset = 0;
    s_chunk_seq = 0;

    ESP_LOGI(TAG, "BLE TX ready bytes=%u", (unsigned)strlen(s_last_response));
    notify_response_ready();
    return 0;
}

static int append_rx_stream(const char *data, size_t len, bool *complete)
{
    *complete = false;
    for (size_t i = 0; i < len; ++i) {
        char ch = data[i];
        if (ch == '\n') {
            *complete = true;
            return 0;
        }
        if (ch == '\r') {
            continue;
        }
        if (s_rx_stream_len >= BLE_REQUEST_BYTES - 1) {
            snprintf(s_last_response,
                     sizeof(s_last_response),
                     "{\"type\":\"error\",\"message\":\"ble request stream too large\",\"max\":%u}",
                     (unsigned)(BLE_REQUEST_BYTES - 1));
            s_rx_stream_len = 0;
            notify_response_ready();
            return BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        s_rx_stream[s_rx_stream_len++] = ch;
    }
    return 0;
}

static int write_rx_request(struct ble_gatt_access_ctxt *ctxt)
{
    int len = OS_MBUF_PKTLEN(ctxt->om);
    if (len <= 0 || len >= BLE_REQUEST_BYTES) {
        snprintf(s_last_response,
                 sizeof(s_last_response),
                 "{\"type\":\"error\",\"message\":\"ble request too large\",\"max\":%u}",
                 (unsigned)(BLE_REQUEST_BYTES - 1));
        notify_response_ready();
        return len <= 0 ? BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    int rc = os_mbuf_copydata(ctxt->om, 0, len, s_last_request);
    if (rc != 0) {
        snprintf(s_last_response,
                 sizeof(s_last_response),
                 "{\"type\":\"error\",\"message\":\"ble request copy failed\",\"rc\":%d}",
                 rc);
        notify_response_ready();
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (s_rx_stream_len == 0 && looks_like_complete_json(s_last_request, (size_t)len)) {
        return process_rx_frame(s_last_request, (size_t)len);
    }

    bool complete = false;
    rc = append_rx_stream(s_last_request, (size_t)len, &complete);
    if (rc != 0) {
        return rc;
    }
    if (!complete) {
        ESP_LOGD(TAG, "BLE RX chunk pending bytes=%u", (unsigned)s_rx_stream_len);
        return 0;
    }

    int process_rc = process_rx_frame(s_rx_stream, s_rx_stream_len);
    s_rx_stream_len = 0;
    return process_rc;
}

static void remote_notify_text(const char *text)
{
    if (!s_remote_notify_enabled || s_conn_handle == BLE_HS_CONN_HANDLE_NONE || text == NULL) {
        return;
    }
    struct os_mbuf *om = ble_hs_mbuf_from_flat(text, strlen(text));
    if (om == NULL) {
        return;
    }
    int rc = ble_gatts_notify_custom(s_conn_handle, s_remote_tx_value_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "remote notify failed rc=%d", rc);
    }
}

static bool remote_bridge_is_active(void)
{
    return vehicle_state_is_remote_bridge();
}

static int write_remote_request(struct ble_gatt_access_ctxt *ctxt)
{
    int len = OS_MBUF_PKTLEN(ctxt->om);
    if (len <= 0 || len > REMOTE_BRIDGE_MAX_WRITE_BYTES) {
        telemetry_record_remote_bridge_error("LEN", len > 0 ? (size_t)len : 0, ESP_ERR_INVALID_SIZE);
        remote_notify_text("E:LEN\n");
        return 0;
    }

    esp_err_t state_err = vehicle_state_enter_remote_bridge();
    if (state_err != ESP_OK) {
        ESP_LOGW(TAG, "remote bridge rejected outside SAFE_IDLE/REMOTE_BRIDGE");
        telemetry_record_remote_bridge_error("BUSY", (size_t)len, state_err);
        remote_notify_text("E:BUSY\n");
        return 0;
    }

    uint8_t bytes[REMOTE_BRIDGE_MAX_WRITE_BYTES] = {0};
    int rc = os_mbuf_copydata(ctxt->om, 0, len, bytes);
    if (rc != 0) {
        telemetry_record_remote_bridge_error("COPY", (size_t)len, ESP_FAIL);
        remote_notify_text("E:COPY\n");
        return BLE_ATT_ERR_UNLIKELY;
    }

    uint32_t tx_bytes = 0;
    esp_err_t err = chassis_uart_write_raw(bytes, (size_t)len, &tx_bytes);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "remote bridge UART write failed: %s", esp_err_to_name(err));
        telemetry_record_remote_bridge_error("UART", (size_t)len, err);
        remote_notify_text("E:UART\n");
        return 0;
    }

    telemetry_record_remote_bridge_success((size_t)len, tx_bytes);
    ESP_LOGD(TAG, "remote bridge forwarded %lu byte(s)", (unsigned long)tx_bytes);
    return 0;
}

static int gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    const ble_uuid_t *uuid = ctxt->chr->uuid;
    if (ble_uuid_cmp(uuid, &s_tx_uuid.u) == 0 && ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return read_tx_response(ctxt);
    }
    if (ble_uuid_cmp(uuid, &s_rx_uuid.u) == 0 && ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return write_rx_request(ctxt);
    }
    if (ble_uuid_cmp(uuid, &s_remote_rx_uuid.u) == 0 && ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return write_remote_request(ctxt);
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static void start_advertising(void)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (const uint8_t *)s_device_name;
    fields.name_len = strlen(s_device_name);
    fields.name_is_complete = 1;
    fields.uuids128 = &s_remote_service_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "set adv fields failed rc=%d", rc);
        return;
    }

    struct ble_gap_adv_params params;
    memset(&params, 0, sizeof(params));
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    params.itvl_min = BLE_GAP_ADV_FAST_INTERVAL2_MIN;
    params.itvl_max = BLE_GAP_ADV_FAST_INTERVAL2_MAX;

    rc = ble_gap_adv_start(s_addr_type, NULL, BLE_HS_FOREVER, &params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "start advertising failed rc=%d", rc);
        return;
    }

    ESP_LOGI(TAG, "BLE advertising name=%s remote=6d1f0001-5b9b-4f4e-9c67-7b1f4d430001 tuning=7b3a0001-8d4d-4b9a-b5c7-0f7c4c415243", s_device_name);
}

static void request_fast_conn_params(uint16_t conn_handle)
{
    struct ble_gap_upd_params conn_params = {
        .itvl_min = 12,             /* 12 * 1.25ms = 15ms */
        .itvl_max = 12,             /* 12 * 1.25ms = 15ms */
        .latency = 0,
        .supervision_timeout = 400, /* 400 * 10ms = 4000ms */
        .min_ce_len = 0,
        .max_ce_len = 0,
    };
    int rc = ble_gap_update_params(conn_handle, &conn_params);
    if (rc != 0) {
        ESP_LOGW(TAG, "conn param update request failed rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "requested conn interval 15ms latency=0 timeout=4000ms");
    }
}

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_rx_stream_len = 0;
            ESP_LOGI(TAG, "BLE connected conn=%u", s_conn_handle);
            /* Request a short connection interval so high-rate remote velocity frames
             * reach the chassis promptly. The phone's default here is 30ms, which (with
             * the 50ms send cadence and per-frame Vz increment) leaves periodic gaps.
             * Re-requested on the MTU event because a connect-time request collides with
             * the phone's initial MTU exchange (HCI 0x2A different-transaction-collision). */
            request_fast_conn_params(s_conn_handle);
            {
                struct ble_gap_conn_desc desc0;
                if (ble_gap_conn_find(s_conn_handle, &desc0) == 0) {
                    ESP_LOGI(TAG, "conn@connect itvl_units=%u (x1.25ms) latency=%u timeout_units=%u (x10ms)",
                             desc0.conn_itvl, desc0.conn_latency, desc0.supervision_timeout);
                }
            }
        } else {
            ESP_LOGW(TAG, "BLE connect failed status=%d", event->connect.status);
            start_advertising();
        }
        break;
    case BLE_GAP_EVENT_CONN_UPDATE: {
        struct ble_gap_conn_desc descu;
        if (ble_gap_conn_find(event->conn_update.conn_handle, &descu) == 0) {
            ESP_LOGI(TAG, "conn_update status=%d itvl_units=%u (x1.25ms) latency=%u timeout_units=%u (x10ms)",
                     event->conn_update.status, descu.conn_itvl, descu.conn_latency, descu.supervision_timeout);
        }
        break;
    }
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE disconnected reason=%d", event->disconnect.reason);
        vehicle_state_snapshot_t snapshot = {0};
        vehicle_state_get_snapshot(&snapshot);
        const bool remote_was_active = remote_bridge_is_active();
        if (snapshot.motion_state == VEHICLE_MOTION_AUTO_ARMED ||
            snapshot.motion_state == VEHICLE_MOTION_AUTO_RUNNING ||
            snapshot.motion_state == VEHICLE_MOTION_MANUAL_TEST ||
            snapshot.motion_state == VEHICLE_MOTION_REMOTE_BRIDGE ||
            remote_was_active) {
            ESP_LOGW(TAG, "BLE disconnected while vehicle active; stopping chassis");
            ESP_ERROR_CHECK_WITHOUT_ABORT(chassis_uart_stop());
            const chassis_motion_cmd_t stop_cmd = {0};
            telemetry_update_motion_cmd(&stop_cmd);
            vehicle_state_stop();
        }
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_notify_enabled = false;
        s_remote_notify_enabled = false;
        s_rx_stream_len = 0;
        telemetry_end_remote_bridge_session();
        start_advertising();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_tx_value_handle) {
            s_notify_enabled = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "BLE TX notify=%d", s_notify_enabled);
        } else if (event->subscribe.attr_handle == s_remote_tx_value_handle) {
            s_remote_notify_enabled = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "remote TX notify=%d", s_remote_notify_enabled);
        }
        break;
    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "BLE mtu=%u", event->mtu.value);
        /* MTU exchange is done, so this re-request avoids the connect-time collision. */
        request_fast_conn_params(event->mtu.conn_handle);
        break;
    default:
        break;
    }

    return 0;
}

static void on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &s_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "infer addr type failed rc=%d", rc);
        return;
    }
    start_advertising();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE reset reason=%d", reason);
}

static int gatt_init(void)
{
    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc != 0) {
        return rc;
    }
    return ble_gatts_add_svcs(s_gatt_svcs);
}

static void build_device_name(void)
{
    snprintf(s_device_name, sizeof(s_device_name), "%s", BLE_DEVICE_NAME_PREFIX);

    esp_err_t err = nvs_open(BLE_NAME_NAMESPACE, NVS_READWRITE, &s_name_nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "open BLE name NVS failed err=%d, using default name", err);
        return;
    }
    s_name_nvs_open = true;
    esp_err_t erase_name_err = nvs_erase_key(s_name_nvs, BLE_NAME_KEY);
    esp_err_t erase_suffix_err = nvs_erase_key(s_name_nvs, BLE_NAME_LEGACY_SUFFIX_KEY);
    if ((erase_name_err == ESP_OK || erase_name_err == ESP_ERR_NVS_NOT_FOUND) &&
        (erase_suffix_err == ESP_OK || erase_suffix_err == ESP_ERR_NVS_NOT_FOUND)) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_commit(s_name_nvs));
    }
}

static void host_task(void *param)
{
    (void)param;
    ESP_LOGI(TAG, "BLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t debug_server_ble_start(void)
{
    if (s_started) {
        return ESP_OK;
    }

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble init failed err=%d", err);
        return err;
    }

    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    ble_att_set_preferred_mtu(512);
    build_device_name();

    int rc = gatt_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "gatt init failed rc=%d", rc);
        return ESP_FAIL;
    }

    rc = ble_svc_gap_device_name_set(s_device_name);
    if (rc != 0) {
        ESP_LOGE(TAG, "set device name failed rc=%d", rc);
        return ESP_FAIL;
    }

    nimble_port_freertos_init(host_task);
    s_started = true;
    ESP_LOGI(TAG, "BLE debug transport started");
    return ESP_OK;
}
