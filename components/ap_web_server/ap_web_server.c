#include "ap_web_server.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "wifi_manager.h"

static const char *TAG = "AP_WebServer";

#define AP_SSID "FurnaceConfig"
#define AP_PASSWORD ""
#define AP_IP "192.168.4.1"

static const char *html_template = 
    "<!DOCTYPE html>"
    "<html>"
    "<head>"
    "<meta charset=\"UTF-8\">"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
    "<title>Furnace Controller - WiFi Setup</title>"
    "<style>"
    "body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background: #f0f0f0; }"
    ".container { max-width: 400px; margin: 50px auto; background: white; padding: 30px; border-radius: 8px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }"
    "h1 { color: #333; font-size: 24px; margin-bottom: 20px; text-align: center; }"
    "label { display: block; margin-bottom: 8px; color: #555; font-weight: bold; }"
    "input { width: 100%%; padding: 12px; margin-bottom: 20px; border: 1px solid #ddd; border-radius: 4px; box-sizing: border-box; }"
    "button { width: 100%%; padding: 14px; background: #4CAF50; color: white; border: none; border-radius: 4px; font-size: 16px; cursor: pointer; }"
    "button:hover { background: #45a049; }"
    ".note { font-size: 12px; color: #888; margin-top: 20px; text-align: center; }"
    ".current { font-size: 13px; color: #666; margin-bottom: 15px; padding: 10px; background: #f5f5f5; border-radius: 4px; }"
    ".status { font-size: 14px; color: #d32f2f; margin-bottom: 15px; padding: 10px; background: #ffebee; border-radius: 4px; text-align: center; }"
    ".status-ok { background: #e8f5e9; color: #2e7d32; }"
    "</style>"
    "</head>"
    "<body>"
    "<div class=\"container\">"
    "<h1>WiFi Setup</h1>"
    "<div class=\"status\">Could not connect to WiFi</div>"
    "<div class=\"current\">Current network: <strong>%s</strong></div>"
    "<form method=\"POST\" action=\"/save\">"
    "<label for=\"ssid\">WiFi Network (SSID)</label>"
    "<input type=\"text\" id=\"ssid\" name=\"ssid\" value=\"%s\" placeholder=\"Enter WiFi network name\" required>"
    "<label for=\"password\">WiFi Password</label>"
    "<input type=\"password\" id=\"password\" name=\"password\" placeholder=\"Enter WiFi password\">"
    "<button type=\"submit\">Connect</button>"
    "</form>"
    "<p class=\"note\">Enter your home WiFi credentials to connect the Furnace Controller.</p>"
    "</div>"
    "</body>"
    "</html>";

static char html_page[2000];

static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, strlen(html_page));
    return ESP_OK;
}

static void url_decode(char *str)
{
    char *src = str;
    char *dst = str;
    while (*src) {
        if (*src == '%' && src[1] && src[2]) {
            char hex[3] = {src[1], src[2], '\0'};
            char decoded = (char)strtol(hex, NULL, 16);
            *dst++ = decoded;
            src += 3;
        } else if (*src == '+') {
            *dst++ = ' ';
            src++;
        } else {
            *dst++ = *src++;
        }
    }
    *dst = '\0';
}

static int find_param(const char *data, size_t len, const char *key, char *value, size_t val_size)
{
    char *search = malloc(len + 1);
    if (!search) return 0;
    memcpy(search, data, len);
    search[len] = '\0';

    char *found = strstr(search, key);
    int result = 0;

    if (found) {
        found += strlen(key);
        if (*found == '=') found++;
        char *end = strchr(found, '&');
        if (!end) end = search + len;
        size_t copy_len = (end - found) < (val_size - 1) ? (end - found) : (val_size - 1);
        memcpy(value, found, copy_len);
        value[copy_len] = '\0';
        url_decode(value);
        result = 1;
    }

    free(search);
    return result;
}

static esp_err_t save_post_handler(httpd_req_t *req)
{
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        ESP_LOGE(TAG, "Failed to receive POST data, ret=%d", ret);
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    ESP_LOGI(TAG, "POST data received: %s", buf);

    char ssid[32] = {0};
    char password[64] = {0};

    find_param(buf, ret, "ssid", ssid, sizeof(ssid));
    find_param(buf, ret, "password", password, sizeof(password));

    ESP_LOGI(TAG, "Parsed - SSID: '%s', Password: '%s'", ssid, password);

    if (strlen(ssid) == 0) {
        ESP_LOGE(TAG, "SSID is empty!");
        httpd_resp_send(req, "SSID is required!", 19);
        return ESP_FAIL;
    }

    esp_err_t err = wifi_save_credentials(ssid, password);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save WiFi credentials: %s", esp_err_to_name(err));
        const char *fail_msg = "Failed to save credentials. Please try again.";
        httpd_resp_send(req, fail_msg, strlen(fail_msg));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Credentials saved successfully, restarting...");
    httpd_resp_send(req, "Credentials saved! Restarting...", 34);

    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();

    return ESP_OK;
}

static httpd_uri_t root_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler
};

static httpd_uri_t save_uri = {
    .uri = "/save",
    .method = HTTP_POST,
    .handler = save_post_handler
};

static httpd_handle_t start_web_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &save_uri);
        ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);
    }
    return server;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "AP started, SSID: %s", AP_SSID);
    }
}

void ap_web_server_start(const char *current_ssid, const char *current_password)
{
    // Prepare safe strings for display
    char ssid_display[32] = {0};
    char pass_display[64] = {0};

    if (current_ssid) {
        strncpy(ssid_display, current_ssid, sizeof(ssid_display) - 1);
    }
    if (current_password) {
        strncpy(pass_display, current_password, sizeof(pass_display) - 1);
    }

    // Generate HTML page with current values
    int len = snprintf(html_page, sizeof(html_page), html_template,
                       ssid_display[0] ? ssid_display : "(none)",
                       ssid_display);
    if (len < 0 || len >= sizeof(html_page)) {
        ESP_LOGE(TAG, "HTML page buffer too small");
    }

    ESP_LOGI(TAG, "AP mode - showing current SSID: %s", ssid_display);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid_len = strlen(AP_SSID),
            .max_connection = 4,
            .authmode = WIFI_AUTH_OPEN
        }
    };
    strncpy((char *)wifi_config.ap.ssid, AP_SSID, sizeof(wifi_config.ap.ssid));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "AP mode started. Connect to SSID: %s", AP_SSID);
    ESP_LOGI(TAG, "Open http://%s in your browser", AP_IP);

    start_web_server();

    // Block indefinitely - this function never returns
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}