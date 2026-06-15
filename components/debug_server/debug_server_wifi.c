#include "debug_server.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "chassis_uart.h"
#include "debug_protocol.h"
#include "debug_server_ble.h"
#include "esp_check.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "vehicle_state.h"

#define DEBUG_RESPONSE_BYTES 8192
#define DEBUG_HTTPD_STACK_BYTES 8192
#define DEBUG_HTTPD_MAX_URI_HANDLERS 20
#define DEBUG_HTTPD_WAIT_TIMEOUT_SEC 5
#define DEBUG_WS_REQUEST_MAX_BYTES 1024

static const char *TAG = "debug_server_wifi";
static httpd_handle_t s_httpd;
static bool s_wifi_started;

static const char INDEX_HTML[] =
    "<!doctype html><html><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>DCar-Liner Debug</title></head><body>"
    "<h1>DCar-Liner Debug</h1>"
    "<p>HTTP/WebSocket debug service is running. This page is only a fallback diagnostic page.</p>"
    "<p><a href=\"/api/schema\">/api/schema</a> "
    "<a href=\"/api/telemetry\">/api/telemetry</a> "
    "<a href=\"/api/params\">/api/params</a> "
    "<a href=\"/api/health\">/api/health</a></p></body></html>";

static void set_common_response_headers(httpd_req_t *req, const char *content_type)
{
    httpd_resp_set_type(req, content_type);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    httpd_resp_set_hdr(req, "Expires", "0");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Connection", "close");
}

static esp_err_t send_and_close(httpd_req_t *req, const char *body, ssize_t body_len)
{
    esp_err_t err = httpd_resp_send(req, body, body_len);
    int sockfd = httpd_req_to_sockfd(req);
    if (s_httpd != NULL && sockfd >= 0) {
        httpd_sess_trigger_close(s_httpd, sockfd);
    }
    return err;
}

static esp_err_t send_protocol_request(httpd_req_t *req, const char *request_json)
{
    char *response = calloc(1, DEBUG_RESPONSE_BYTES);
    if (response == NULL) {
        set_common_response_headers(req, "application/json");
        httpd_resp_set_status(req, "503 Service Unavailable");
        return send_and_close(req, "{\"type\":\"error\",\"message\":\"no response buffer\"}", HTTPD_RESP_USE_STRLEN);
    }

    esp_err_t err = debug_protocol_handle_message(request_json, response, DEBUG_RESPONSE_BYTES);

    set_common_response_headers(req, "application/json");
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "API %s failed: %s", req->uri, esp_err_to_name(err));
        httpd_resp_set_status(req, "500 Internal Server Error");
        int written = snprintf(response,
                               DEBUG_RESPONSE_BYTES,
                               "{\"type\":\"error\",\"message\":\"%s\",\"uri\":\"%s\"}",
                               esp_err_to_name(err),
                               req->uri);
        if (written <= 0 || written >= DEBUG_RESPONSE_BYTES) {
            free(response);
            return send_and_close(req, "{\"type\":\"error\",\"message\":\"api failed\"}", HTTPD_RESP_USE_STRLEN);
        }
    } else {
        ESP_LOGI(TAG, "API %s OK", req->uri);
    }

    err = send_and_close(req, response, HTTPD_RESP_USE_STRLEN);
    free(response);
    return err;
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    set_common_response_headers(req, "text/html");
    return send_and_close(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t schema_get_handler(httpd_req_t *req)
{
    return send_protocol_request(req, "{\"type\":\"get_schema\"}");
}

static esp_err_t params_get_handler(httpd_req_t *req)
{
    return send_protocol_request(req, "{\"type\":\"get_params\"}");
}

static esp_err_t telemetry_get_handler(httpd_req_t *req)
{
    return send_protocol_request(req, "{\"type\":\"get_telemetry\"}");
}

static esp_err_t health_get_handler(httpd_req_t *req)
{
    char response[256];
    int written = snprintf(response,
                           sizeof(response),
                           "{\"type\":\"health\",\"status\":\"ok\","
                           "\"free_heap\":%lu,\"httpd_stack_free_words\":%lu}",
                           (unsigned long)esp_get_free_heap_size(),
                           (unsigned long)uxTaskGetStackHighWaterMark(NULL));

    set_common_response_headers(req, "application/json");
    if (written <= 0 || written >= (int)sizeof(response)) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return send_and_close(req, "{\"type\":\"error\",\"message\":\"health truncated\"}", HTTPD_RESP_USE_STRLEN);
    }
    return send_and_close(req, response, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WebSocket client connected");
        return ESP_OK;
    }

    httpd_ws_frame_t frame = {0};
    frame.type = HTTPD_WS_TYPE_TEXT;
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);
    if (err != ESP_OK) {
        return err;
    }

    if (frame.len == 0 || frame.len > DEBUG_WS_REQUEST_MAX_BYTES) {
        return ESP_ERR_INVALID_SIZE;
    }

    char *request = calloc(frame.len + 1, 1);
    if (request == NULL) {
        return ESP_ERR_NO_MEM;
    }

    frame.payload = (uint8_t *)request;
    err = httpd_ws_recv_frame(req, &frame, frame.len);
    if (err != ESP_OK) {
        free(request);
        return err;
    }

    char *response = calloc(1, DEBUG_RESPONSE_BYTES);
    if (response == NULL) {
        free(request);
        return ESP_ERR_NO_MEM;
    }

    err = debug_protocol_handle_message(request, response, DEBUG_RESPONSE_BYTES);
    free(request);

    if (err != ESP_OK) {
        snprintf(response, DEBUG_RESPONSE_BYTES, "{\"type\":\"error\",\"message\":\"protocol failed\"}");
    }

    httpd_ws_frame_t response_frame = {
        .final = true,
        .fragmented = false,
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)response,
        .len = strlen(response),
    };
    err = httpd_ws_send_frame(req, &response_frame);
    free(response);
    return err;
}

static esp_err_t start_http_server(void)
{
    if (s_httpd != NULL) {
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = DEBUG_HTTPD_STACK_BYTES;
    config.max_uri_handlers = DEBUG_HTTPD_MAX_URI_HANDLERS;
    config.recv_wait_timeout = DEBUG_HTTPD_WAIT_TIMEOUT_SEC;
    config.send_wait_timeout = DEBUG_HTTPD_WAIT_TIMEOUT_SEC;
    config.lru_purge_enable = true;

    ESP_RETURN_ON_ERROR(httpd_start(&s_httpd, &config), TAG, "start httpd");

    const httpd_uri_t root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
    };
    const httpd_uri_t schema = {
        .uri = "/api/schema",
        .method = HTTP_GET,
        .handler = schema_get_handler,
    };
    const httpd_uri_t schema_trailing = {
        .uri = "/api/schema/",
        .method = HTTP_GET,
        .handler = schema_get_handler,
    };
    const httpd_uri_t scheme_alias = {
        .uri = "/api/scheme",
        .method = HTTP_GET,
        .handler = schema_get_handler,
    };
    const httpd_uri_t params = {
        .uri = "/api/params",
        .method = HTTP_GET,
        .handler = params_get_handler,
    };
    const httpd_uri_t params_trailing = {
        .uri = "/api/params/",
        .method = HTTP_GET,
        .handler = params_get_handler,
    };
    const httpd_uri_t telemetry = {
        .uri = "/api/telemetry",
        .method = HTTP_GET,
        .handler = telemetry_get_handler,
    };
    const httpd_uri_t telemetry_trailing = {
        .uri = "/api/telemetry/",
        .method = HTTP_GET,
        .handler = telemetry_get_handler,
    };
    const httpd_uri_t health = {
        .uri = "/api/health",
        .method = HTTP_GET,
        .handler = health_get_handler,
    };
    const httpd_uri_t health_trailing = {
        .uri = "/api/health/",
        .method = HTTP_GET,
        .handler = health_get_handler,
    };
    const httpd_uri_t websocket = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .is_websocket = true,
    };

    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &root), TAG, "register root");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &schema), TAG, "register schema");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &schema_trailing), TAG, "register schema slash");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &scheme_alias), TAG, "register scheme alias");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &params), TAG, "register params");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &params_trailing), TAG, "register params slash");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &telemetry), TAG, "register telemetry");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &telemetry_trailing), TAG, "register telemetry slash");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &health), TAG, "register health");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &health_trailing), TAG, "register health slash");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &websocket), TAG, "register websocket");
    return ESP_OK;
}

static void make_softap_ssid(const char *prefix, char *ssid, size_t ssid_size)
{
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(ssid, ssid_size, "%s-%02X%02X%02X", prefix, mac[3], mac[4], mac[5]);
}

static esp_err_t start_wifi_softap(const debug_server_config_t *config)
{
    if (s_wifi_started) {
        return ESP_OK;
    }

    char ssid[33] = {0};
    make_softap_ssid(config->softap_ssid_prefix, ssid, sizeof(ssid));

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init");
    esp_err_t event_err = esp_event_loop_create_default();
    if (event_err != ESP_OK && event_err != ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(event_err, TAG, "event loop");
    }
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t wifi_init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&wifi_init), TAG, "wifi init");

    wifi_config_t wifi_config = {0};
    wifi_config.ap.ssid_len = strnlen(ssid, sizeof(wifi_config.ap.ssid));
    memcpy(wifi_config.ap.ssid, ssid, wifi_config.ap.ssid_len);
    wifi_config.ap.channel = config->channel;
    wifi_config.ap.max_connection = config->max_connections;

    if (config->softap_password != NULL && strlen(config->softap_password) >= 8) {
        size_t password_len = strnlen(config->softap_password, sizeof(wifi_config.ap.password));
        ESP_RETURN_ON_FALSE(password_len < sizeof(wifi_config.ap.password),
                            ESP_ERR_INVALID_ARG,
                            TAG,
                            "softap password too long");
        memcpy(wifi_config.ap.password, config->softap_password, password_len + 1);
        wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    } else {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "set wifi mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &wifi_config), TAG, "set ap config");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start");

    s_wifi_started = true;
    ESP_LOGI(TAG, "Wi-Fi debug SoftAP ssid=%s ip=http://192.168.4.1", ssid);
    return ESP_OK;
}

esp_err_t debug_server_start(const debug_server_config_t *config)
{
    static const debug_server_config_t default_config = {
        .softap_ssid_prefix = "DCar-Liner",
        .softap_password = "DCar-Liner123",
        .channel = 6,
        .max_connections = 2,
        .start_wifi = false,
        .start_ble_placeholder = true,
    };

    const debug_server_config_t *active_config = config == NULL ? &default_config : config;
    ESP_RETURN_ON_FALSE(active_config->softap_ssid_prefix != NULL, ESP_ERR_INVALID_ARG, TAG, "missing ssid prefix");

    if (active_config->start_ble_placeholder) {
        ESP_RETURN_ON_ERROR(debug_server_ble_start(), TAG, "start ble debug transport");
    }

    if (active_config->start_wifi) {
        ESP_RETURN_ON_ERROR(start_wifi_softap(active_config), TAG, "start softap");
        ESP_RETURN_ON_ERROR(start_http_server(), TAG, "start http");
    }

    return ESP_OK;
}

esp_err_t debug_server_start_wifi_fallback(void)
{
    const debug_server_config_t wifi_config = {
        .softap_ssid_prefix = "DCar-Liner",
        .softap_password = "DCar-Liner123",
        .channel = 6,
        .max_connections = 2,
        .start_wifi = true,
        .start_ble_placeholder = false,
    };

    ESP_RETURN_ON_ERROR(start_wifi_softap(&wifi_config), TAG, "start fallback softap");
    ESP_RETURN_ON_ERROR(start_http_server(), TAG, "start fallback http");
    return ESP_OK;
}
