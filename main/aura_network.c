#include "aura_network.h"
#include "wifi_credentials.h"
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lwip/sockets.h"
#include "mdns.h"

#define AURA_PORT 4242
static const char *TAG = "aura_network";
static bool initialized;
static volatile bool running, has_ip;
static TaskHandle_t server_task;
static SemaphoreHandle_t socket_lock;
static int client = -1;
static aura_network_command_cb on_command;

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        has_ip = true;
        ip_event_got_ip_t *event = data;
        ESP_LOGW(TAG, "Aura Wi-Fi ready: " IPSTR ":%d", IP2STR(&event->ip_info.ip), AURA_PORT);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        has_ip = false; // Server task retries with a bounded interval; HID stays running.
        wifi_event_sta_disconnected_t *event = data;
        ESP_LOGW(TAG, "Wi-Fi disconnected: reason=%u", event->reason);
    }
}
static void close_client(void)
{
    if (client >= 0) { shutdown(client, SHUT_RDWR); close(client); client = -1; }
}
bool aura_network_connected(void)
{
    if (!socket_lock) return false;
    xSemaphoreTake(socket_lock, portMAX_DELAY);
    bool connected = running && has_ip && client >= 0;
    xSemaphoreGive(socket_lock);
    return connected;
}
void aura_network_send(uint8_t channel, const uint8_t *data, size_t size)
{
    if (!socket_lock || !size || size > 20) return;
    uint8_t packet[22] = {channel, (uint8_t)size};
    memcpy(packet + 2, data, size);
    xSemaphoreTake(socket_lock, portMAX_DELAY);
    if (running && has_ip && client >= 0) {
        // Never accumulate an unbounded telemetry backlog. A stalled reader reconnects.
        int sent = send(client, packet, size + 2, MSG_DONTWAIT);
        if (sent != (int)size + 2) close_client();
    }
    xSemaphoreGive(socket_lock);
}
static void network_task(void *arg)
{
    int listener = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    struct sockaddr_in address = {.sin_family = AF_INET, .sin_port = htons(AURA_PORT),
                                  .sin_addr.s_addr = htonl(INADDR_ANY)};
    int yes = 1;
    setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    if (listener < 0 || bind(listener, (struct sockaddr *)&address, sizeof(address)) < 0 ||
        listen(listener, 1) < 0) {
        ESP_LOGE(TAG, "TCP listener failed: %d", errno);
        if (listener >= 0) close(listener);
        server_task = NULL;
        vTaskDelete(NULL);
        return;
    }
    uint8_t input[22];
    size_t used = 0;
    TickType_t next_retry = 0;
    while (running) {
        if (!has_ip) {
            xSemaphoreTake(socket_lock, portMAX_DELAY); close_client(); xSemaphoreGive(socket_lock);
            used = 0;
            TickType_t now = xTaskGetTickCount();
            if ((int32_t)(now - next_retry) >= 0) {
                esp_wifi_connect();
                next_retry = now + pdMS_TO_TICKS(10000);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        xSemaphoreTake(socket_lock, portMAX_DELAY);
        int fd = client;
        xSemaphoreGive(socket_lock);
        fd_set reads; FD_ZERO(&reads); FD_SET(fd >= 0 ? fd : listener, &reads);
        struct timeval timeout = {.tv_usec = 100000};
        int selected = select((fd >= 0 ? fd : listener) + 1, &reads, NULL, NULL, &timeout);
        if (selected <= 0) continue;
        if (fd < 0) {
            int accepted = accept(listener, NULL, NULL);
            xSemaphoreTake(socket_lock, portMAX_DELAY);
            if (running) client = accepted; else if (accepted >= 0) close(accepted);
            xSemaphoreGive(socket_lock);
            used = 0;
            if (accepted >= 0) ESP_LOGW(TAG, "Aura app connected over Wi-Fi");
            continue;
        }
        xSemaphoreTake(socket_lock, portMAX_DELAY);
        int count = client == fd ? recv(fd, input + used, sizeof(input) - used, MSG_DONTWAIT) : -1;
        if (count <= 0 && client == fd) close_client();
        xSemaphoreGive(socket_lock);
        if (count <= 0) { used = 0; continue; }
        used += count;
        while (used >= 2) {
            size_t length = input[1];
            if (input[0] != 0 || !length || length > 20) {
                xSemaphoreTake(socket_lock, portMAX_DELAY); close_client(); xSemaphoreGive(socket_lock);
                used = 0; break;
            }
            if (used < length + 2) break;
            on_command(input + 2, length);
            used -= length + 2;
            memmove(input, input + length + 2, used);
        }
    }
    xSemaphoreTake(socket_lock, portMAX_DELAY); close_client(); xSemaphoreGive(socket_lock);
    close(listener);
    server_task = NULL;
    vTaskDelete(NULL);
}
esp_err_t aura_network_start(aura_network_command_cb callback)
{
    if (running) return ESP_OK;
    if (!initialized) {
        ESP_ERROR_CHECK(esp_netif_init());
        esp_err_t event_result = esp_event_loop_create_default();
        if (event_result != ESP_OK && event_result != ESP_ERR_INVALID_STATE) return event_result;
        if (!esp_netif_create_default_wifi_sta()) return ESP_ERR_NO_MEM;
        wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
        esp_err_t result = esp_wifi_init(&config);
        if (result != ESP_OK) return result;
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL));
        socket_lock = xSemaphoreCreateMutex();
        if (!socket_lock) return ESP_ERR_NO_MEM;
        initialized = true;
    }
    wifi_config_t config = {0};
    strlcpy((char *)config.sta.ssid, AURA_WIFI_SSID, sizeof(config.sta.ssid));
    strlcpy((char *)config.sta.password, AURA_WIFI_PASSWORD, sizeof(config.sta.password));
    config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    esp_err_t result = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (result == ESP_OK) result = esp_wifi_set_mode(WIFI_MODE_STA);
    if (result == ESP_OK) result = esp_wifi_set_config(WIFI_IF_STA, &config);
    if (result == ESP_OK) result = esp_wifi_start();
    if (result != ESP_OK) return result;
    running = true; has_ip = false; on_command = callback;
    // Espressif coexistence arbitrates the shared radio. Never toggle BT to send Wi-Fi.
    // The board sends live control and telemetry while it has external VIN.
    // Avoid modem-sleep delivery bursts, which are visible as brief freezes in
    // the desktop control view when Classic Bluetooth is also active.
    esp_wifi_set_ps(WIFI_PS_NONE);
    result = mdns_init();
    if (result == ESP_OK) {
        mdns_hostname_set("aura-main-board");
        mdns_instance_name_set("Aura Main Board");
        mdns_service_add(NULL, "_aura", "_tcp", AURA_PORT, NULL, 0);
    }
    if (xTaskCreate(network_task, "aura_network", 4096, NULL, 4, &server_task) != pdPASS) {
        aura_network_stop(); return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
void aura_network_stop(void)
{
    if (!running) return;
    running = false;
    while (server_task) vTaskDelay(pdMS_TO_TICKS(10));
    mdns_free();
    esp_wifi_stop();
    has_ip = false;
}

void aura_network_print_status(void)
{
    wifi_ap_record_t ap = {0};
    esp_err_t result = esp_wifi_sta_get_ap_info(&ap);
    ESP_LOGW(TAG, "Wi-Fi running=%d IP=%d client=%d AP=%s RSSI=%d", running,
             has_ip, aura_network_connected(), esp_err_to_name(result),
             result == ESP_OK ? ap.rssi : 0);
}
