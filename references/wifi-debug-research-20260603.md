# ESP32 Wi-Fi Debug Pattern Research

Date: 2026-06-03

## Problem To Solve

The ESP32-C3 root page is consistently reachable, but JSON API pages such as `/api/schema`, `/api/params`, and `/api/telemetry` open only rarely. The local code review found a direct structural risk: each HTTP API handler allocated an 8192-byte response buffer on the HTTP server task stack, while ESP-IDF's local `HTTPD_DEFAULT_CONFIG()` uses a 4096-byte default stack in `/Users/schwartz/esp/esp-idf/components/esp_http_server/include/esp_http_server.h`.

## Researched Projects And Examples

| # | Source | Wi-Fi debug/config pattern observed | Takeaway for this firmware |
|---|---|---|---|
| 1 | [ESP-IDF HTTP server simple](https://github.com/espressif/esp-idf/tree/master/examples/protocols/http_server/simple) | Small handlers, explicit URI registration, chunked response examples, LRU purge use. | Keep handlers small and avoid large stack buffers. |
| 2 | [ESP-IDF HTTP RESTful server](https://github.com/espressif/esp-idf/tree/master/examples/protocols/http_server/restful_server) | REST API plus frontend; supports PC-side frontend development through a proxy before deploying files. | Keep ESP32 as API backend; develop the main tuning UI on the computer. |
| 3 | [ESP-IDF HTTP file serving](https://github.com/espressif/esp-idf/tree/master/examples/protocols/http_server/file_serving) | Sends file/directory output in chunks instead of building one large page on stack. | Use heap or chunks for larger payloads. |
| 4 | [ESP-IDF captive portal](https://github.com/espressif/esp-idf/tree/master/examples/protocols/http_server/captive_portal) | SoftAP portal with increased socket capacity and LRU purge. | Browser clients open several sockets; short connections and LRU help stability. |
| 5 | [ESP-IDF WebSocket echo](https://github.com/espressif/esp-idf/tree/master/examples/protocols/http_server/ws_echo_server) | WebSocket endpoint is a first-class HTTP server handler. | Keep `/ws` for live tuning, but allocate larger payloads away from stack. |
| 6 | [ESP-IDF async handlers](https://github.com/espressif/esp-idf/tree/master/examples/protocols/http_server/async_handlers) | Long work is moved out of the HTTP task; server capacity is configured explicitly. | Avoid blocking or heavy memory work in the HTTP handler. |
| 7 | [ESP-IDF persistent sockets](https://github.com/espressif/esp-idf/tree/master/examples/protocols/http_server/persistent_sockets) | Persistent sessions are deliberate and require session context management. | For this small debug API, close normal HTTP API sessions after each response. |
| 8 | [ESP-IDF Wi-Fi SoftAP](https://github.com/espressif/esp-idf/tree/master/examples/wifi/getting_started/softAP) | Minimal AP setup with explicit AP config. | Keep SoftAP simple and deterministic for field tuning. |
| 9 | [ESP-IDF SoftAP + STA](https://github.com/espressif/esp-idf/tree/master/examples/wifi/softap_sta) | AP and station can coexist, but complexity increases. | Stay AP-only for first bring-up; add STA later only if needed. |
| 10 | [ESP-IDF Wi-Fi provisioning manager](https://github.com/espressif/esp-idf/tree/master/examples/provisioning/wifi_prov_mgr) | Separates provisioning transport from application logic. | Keep debug protocol independent of Wi-Fi/BLE transport. |
| 11 | [Tasmota](https://github.com/arendst/Tasmota) | Mature local web UI, HTTP command surface, serial fallback, OTA, local control. | Preserve serial recovery and keep HTTP commands small and observable. |
| 12 | [WLED](https://github.com/wled/WLED) | On-device UI plus JSON API/WebSocket for live LED tuning. | Use WebSocket for frequent telemetry/parameter updates, HTTP for snapshots. |
| 13 | [ESPHome](https://github.com/esphome/esphome) | Remote control is built on explicit API contracts rather than ad hoc pages. | Treat `/api/schema`, `/api/params`, `/api/telemetry`, and `/ws` as stable contracts. |
| 14 | [ESP Easy](https://github.com/letscontrolit/ESPEasy) | Web-based local configuration, device/plugin state, serial fallback. | Keep module diagnostics exposed as plain API values. |
| 15 | [ESPurna](https://github.com/xoseperez/espurna) | Web UI and API for ESP8266/ESP32 devices, with MQTT/HTTP separation. | Separate transport choices from parameter semantics. |
| 16 | [tzapu/WiFiManager](https://github.com/tzapu/WiFiManager) | Captive portal for configuration, small request handlers, custom params. | Use portal ideas only; do not import Arduino stack. |
| 17 | [tonyp7/esp32-wifi-manager](https://github.com/tonyp7/esp32-wifi-manager) | ESP-IDF captive portal and Wi-Fi manager. | ESP-IDF-native Wi-Fi manager patterns fit our ESP32-C3 target. |
| 18 | [khoih-prog/ESP_WiFiManager](https://github.com/khoih-prog/ESP_WiFiManager) | Runtime credential and custom parameter portal. | Dynamic params are useful, but our NVS `param_store` remains the source of truth. |
| 19 | [khoih-prog/ESPAsync_WiFiManager](https://github.com/khoih-prog/ESPAsync_WiFiManager) | Async server and custom parameter portal for ESP32/ESP8266. | Async style is useful for responsiveness; avoid pulling Arduino dependencies. |
| 20 | [khoih-prog/ESP_WiFiManager_Lite](https://github.com/khoih-prog/ESP_WiFiManager_Lite) | Memory-reduced credential/config portal. | Keep our debug server lightweight and API-first. |
| 21 | [khoih-prog/ESPAsync_WiFiManager_Lite](https://github.com/khoih-prog/ESPAsync_WiFiManager_Lite) | Lightweight async portal with dynamic parameters. | Avoid heavy full UI on ESP32; use small API payloads. |
| 22 | [Hieromon/AutoConnect](https://github.com/Hieromon/AutoConnect) | Runtime WLAN configuration with captive portal, custom pages, JSON-defined screens. | JSON-defined controls maps well to our `/api/schema`. |
| 23 | [prampec/IotWebConf](https://github.com/prampec/IotWebConf) | Non-blocking AP web configuration for ESP8266/ESP32. | Handlers should remain non-blocking and compact. |
| 24 | [Juerd/ESP-WiFiSettings](https://github.com/Juerd/ESP-WiFiSettings) | ESP32 Arduino Wi-Fi settings portal. | Store settings explicitly; do not rely on browser state. |
| 25 | [ayushsharma82/NetWizard](https://github.com/ayushsharma82/NetWizard) | Captive portal for IoT network setup. | Human-friendly portal is useful later, not required for this bring-up API. |
| 26 | [ayushsharma82/ESPConnect](https://github.com/ayushsharma82/ESPConnect) | Simple captive portal for ESP8266/ESP32. | Simple is better for field debug than a large embedded UI. |
| 27 | [cotestatnt/esp-fs-webserver](https://github.com/cotestatnt/esp-fs-webserver) | Filesystem web server, Wi-Fi manager, editor. | Keep file serving separate from runtime control. |
| 28 | [cotestatnt/async-esp-fs-webserver](https://github.com/cotestatnt/async-esp-fs-webserver) | Async web server, Wi-Fi manager, web editor. | Long UI assets belong in files or PC-side tools, not stack buffers. |
| 29 | [mathieucarbou/ESPAsyncWebServer](https://github.com/mathieucarbou/ESPAsyncWebServer) | Asynchronous HTTP/WebSocket server library. | Its model reinforces small callbacks and async live updates, but not a direct dependency. |
| 30 | [hoeken/PsychicHttp](https://github.com/hoeken/PsychicHttp) | ESP32 HTTP/WebSocket server built on ESP-IDF HTTP server. | Confirms ESP-IDF HTTP server can be robust when configured and memory-managed carefully. |

## Adopted Pattern

- Use ESP-IDF HTTP server directly to keep the ESP32-C3 implementation portable.
- Explicitly configure the HTTP server stack size instead of relying on the 4096-byte default.
- Allocate large JSON response buffers on the heap, then free them after sending.
- Add `Cache-Control`, `Pragma`, `Expires`, and `Connection: close` headers for normal HTTP API reads.
- Add `Access-Control-Allow-Origin: *` so a computer-side web tool can call ESP32 APIs.
- Keep the ESP32 root page as a tiny fallback diagnostic page.
- Add `/api/health` as a small, low-risk endpoint to test HTTP reachability and heap/stack headroom before testing larger JSON endpoints.

## Not Adopted

- No Arduino `WebServer`, `ESPAsyncWebServer`, WiFiManager, or AutoConnect dependency was imported.
- No full web app is hosted on the ESP32 for this stage.
- No STA/provisioning workflow was added yet; SoftAP remains the bring-up path.
