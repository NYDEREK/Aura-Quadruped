#include "dualsense_report.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "ps5_transport.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"

#include "dualsense.h"

#define DUALSENSE_NVS_NAMESPACE "dualsense"
#define DUALSENSE_NVS_BDA_KEY "bda"
#define DUALSENSE_SCAN_SECONDS 15

static const char *TAG = "dualsense";
static portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;
static dualsense_snapshot_t state;
static dualsense_scan_device_t scanned_devices[DUALSENSE_MAX_SCAN_DEVICES];
static size_t scanned_device_count;
static volatile bool active_device;
static volatile bool connecting;
static TickType_t connect_deadline;
static esp_bd_addr_t active_address;
static esp_bd_addr_t candidate_bda;
static bool candidate_available;
static bool hidh_initialized;
static uint32_t raw_reports, rejected_reports;
static uint16_t last_raw_length;
static uint8_t last_raw_id;
static TickType_t last_input_tick;

// Bluetooth invokes the L2CAP callback from BTU_TASK on CPU 0.  Keep that
// callback bounded: parsing a continuous DualSense stream there can starve
// the controller's idle task and trip the watchdog.
typedef struct {
    uint8_t size;
    uint8_t data[78];
} dualsense_input_report_t;
static QueueHandle_t report_queue;

static void set_connection_state(dualsense_connection_state_t next)
{
    portENTER_CRITICAL(&state_lock);
    state.state = next;
    portEXIT_CRITICAL(&state_lock);
}

static void set_address(const uint8_t address[6])
{
    portENTER_CRITICAL(&state_lock);
    memcpy(state.address, address, sizeof(state.address));
    portEXIT_CRITICAL(&state_lock);
}

static bool saved_address(esp_bd_addr_t address)
{
    nvs_handle_t handle;
    size_t size = ESP_BD_ADDR_LEN;
    if (nvs_open(DUALSENSE_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) return false;
    const esp_err_t result = nvs_get_blob(handle, DUALSENSE_NVS_BDA_KEY, address, &size);
    nvs_close(handle);
    return result == ESP_OK && size == ESP_BD_ADDR_LEN;
}

static void save_address(const uint8_t address[6])
{
    nvs_handle_t handle;
    if (nvs_open(DUALSENSE_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) return;
    if (nvs_set_blob(handle, DUALSENSE_NVS_BDA_KEY, address, ESP_BD_ADDR_LEN) == ESP_OK)
        nvs_commit(handle);
    nvs_close(handle);
}

static void erase_saved_address(void)
{
    nvs_handle_t handle;
    if (nvs_open(DUALSENSE_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) return;
    nvs_erase_key(handle, DUALSENSE_NVS_BDA_KEY);
    nvs_commit(handle);
    nvs_close(handle);
}

static void read_discovery_name(esp_bt_gap_cb_param_t *param,
                                char name[DUALSENSE_DEVICE_NAME_SIZE])
{
    uint8_t *eir = NULL;
    uint8_t eir_length = 0;
    name[0] = '\0';

    for (int i = 0; i < param->disc_res.num_prop; ++i) {
        esp_bt_gap_dev_prop_t *property = &param->disc_res.prop[i];
        if (property->type == ESP_BT_GAP_DEV_PROP_BDNAME && property->val) {
            const size_t length = property->len < DUALSENSE_DEVICE_NAME_SIZE - 1
                                      ? property->len : DUALSENSE_DEVICE_NAME_SIZE - 1;
            memcpy(name, property->val, length);
            name[length] = '\0';
        } else if (property->type == ESP_BT_GAP_DEV_PROP_EIR && property->val) {
            eir = property->val;
            eir_length = property->len;
        }
    }

    if (!name[0] && eir && eir_length) {
        uint8_t found_length = 0;
        uint8_t *found = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME,
                                                      &found_length);
        if (!found)
            found = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME,
                                                &found_length);
        if (found) {
            const size_t length = found_length < DUALSENSE_DEVICE_NAME_SIZE - 1
                                      ? found_length : DUALSENSE_DEVICE_NAME_SIZE - 1;
            memcpy(name, found, length);
            name[length] = '\0';
        }
    }
}

static int8_t read_discovery_rssi(esp_bt_gap_cb_param_t *param)
{
    for (int i = 0; i < param->disc_res.num_prop; ++i) {
        esp_bt_gap_dev_prop_t *property = &param->disc_res.prop[i];
        if (property->type == ESP_BT_GAP_DEV_PROP_RSSI && property->val && property->len >= 1)
            return *(int8_t *)property->val;
    }
    return 0;
}

static void remember_discovered_device(esp_bt_gap_cb_param_t *param)
{
    dualsense_scan_device_t device = {0};
    memcpy(device.address, param->disc_res.bda, ESP_BD_ADDR_LEN);
    device.rssi = read_discovery_rssi(param);
    read_discovery_name(param, device.name);

    portENTER_CRITICAL(&state_lock);
    size_t index = scanned_device_count;
    for (size_t i = 0; i < scanned_device_count; ++i) {
        if (memcmp(scanned_devices[i].address, device.address, ESP_BD_ADDR_LEN) == 0) {
            index = i;
            break;
        }
    }
    if (index < DUALSENSE_MAX_SCAN_DEVICES) {
        scanned_devices[index] = device;
        if (index == scanned_device_count) ++scanned_device_count;
    }
    portEXIT_CRITICAL(&state_lock);

    ESP_LOGI(TAG, "found Classic BT device %02X:%02X:%02X:%02X:%02X:%02X%s%s",
             device.address[0], device.address[1], device.address[2], device.address[3],
             device.address[4], device.address[5], device.name[0] ? " " : "",
             device.name[0] ? device.name : "");
}

static bool read_candidate(esp_bd_addr_t address)
{
    bool available;
    portENTER_CRITICAL(&state_lock);
    available = candidate_available;
    if (available) memcpy(address, candidate_bda, ESP_BD_ADDR_LEN);
    portEXIT_CRITICAL(&state_lock);
    return available;
}

static void clear_candidate(void)
{
    portENTER_CRITICAL(&state_lock);
    candidate_available = false;
    portEXIT_CRITICAL(&state_lock);
}

static esp_err_t open_selected_controller(void)
{
    esp_bd_addr_t address;
    if (!read_candidate(address)) return ESP_ERR_NOT_FOUND;
    if (active_device) return ESP_ERR_INVALID_STATE;

    clear_candidate();
    set_connection_state(DUALSENSE_STATE_CONNECTING);
    memcpy(active_address, address, sizeof(active_address));
    connecting = true;
    connect_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(12000);
    esp_err_t result = ps5_transport_connect(address);
    if (result != ESP_OK) {
        connecting = false;
        clear_candidate();
        set_connection_state(DUALSENSE_STATE_ERROR);
    }
    return result;
}

static void bt_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT:
        remember_discovered_device(param);
        break;
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            if (read_candidate((esp_bd_addr_t){0})) {
                if (!active_device) open_selected_controller();
            } else if (!active_device && !connecting) {
                set_connection_state(DUALSENSE_STATE_READY);
                ESP_LOGW(TAG, "Bluetooth scan finished");
            }
        }
        break;
    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGW(TAG, "BT mode=%u interval=%u", param->mode_chg.mode, param->mode_chg.interval);
        break;
    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        ESP_LOGW(TAG, "BT ACL disconnected: reason=0x%02x handle=%u",
                 param->acl_disconn_cmpl_stat.reason, param->acl_disconn_cmpl_stat.handle);
        if (!memcmp(param->acl_disconn_cmpl_stat.bda, active_address, ESP_BD_ADDR_LEN)) {
            active_device = false;
            connecting = false;
            clear_candidate();
            portENTER_CRITICAL(&state_lock);
            state.has_input = false;
            state.state = DUALSENSE_STATE_READY;
            portEXIT_CRITICAL(&state_lock);
        }
        break;
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
            ESP_LOGW(TAG, "DualSense pairing accepted");
        else
            ESP_LOGW(TAG, "DualSense pairing failed: status=%u", param->auth_cmpl.stat);
        break;
    case ESP_BT_GAP_CFM_REQ_EVT:
        // ESP_BT_IO_CAP_NONE gives a Just Works confirmation.
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_PIN_REQ_EVT: {
        esp_bt_pin_code_t pin = {0};
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 0, pin);
        break;
    }
    default:
        break;
    }
}

static void transport_opened(const uint8_t address[6])
{
    connecting = false;
        memcpy(active_address, address, sizeof(active_address));
        active_device = true;
        set_address(active_address);
        save_address(active_address);
        portENTER_CRITICAL(&state_lock);
        state.has_saved_controller = true;
        state.sample_count = 0;
        state.has_input = false;
        portEXIT_CRITICAL(&state_lock);
        clear_candidate();
        set_connection_state(DUALSENSE_STATE_CONNECTING);
        ESP_LOGW(TAG, "DualSense HID connected");

}
static void transport_closed(void)
{
    active_device = false; connecting = false; clear_candidate();
    portENTER_CRITICAL(&state_lock);
    state.has_input = false; state.state = DUALSENSE_STATE_READY;
    portEXIT_CRITICAL(&state_lock);
}
static void transport_input(const uint8_t *data, size_t size)
{
    if (!report_queue || !active_device || !data || size < 2 || size > 78) return;
    dualsense_input_report_t report = {.size = (uint8_t)size};
    memcpy(report.data, data, size);
    (void)xQueueSend(report_queue, &report, 0);
}

static void process_input_report(const dualsense_input_report_t *report)
{
    const uint8_t id = report->data[0];
    const size_t length = report->size - 1;
    dualsense_controls_t controls;
    bool has_battery = false;
    uint8_t battery_percent = 0;

    // Full Bluetooth report 0x31 stores capacity in the low nibble of
    // absolute byte 54: levels 0...10 correspond to 0...100 percent.
    if (id == 0x31 && report->size > 54) {
        const uint8_t level = report->data[54] & 0x0f;
        if (level <= 10) {
            has_battery = true;
            battery_percent = level * 10;
        }
    }

    portENTER_CRITICAL(&state_lock);
    ++raw_reports;
    last_raw_length = report->size;
    last_raw_id = id;
    portEXIT_CRITICAL(&state_lock);

    if (!dualsense_decode(id, report->data + 1, length, &controls)) {
        portENTER_CRITICAL(&state_lock);
        ++rejected_reports;
        portEXIT_CRITICAL(&state_lock);
        static unsigned unknown;
        if (unknown++ < 3)
            ESP_LOGW(TAG, "Unrecognized HID report id=%02x size=%u", id, (unsigned)length);
        return;
    }

    portENTER_CRITICAL(&state_lock);
    state.state = DUALSENSE_STATE_CONNECTED;
    state.has_input = true;
    state.report_id = id;
    state.report_length = length > 255 ? 255 : length;
    state.left_x = controls.axes[0]; state.left_y = controls.axes[1];
    state.right_x = controls.axes[2]; state.right_y = controls.axes[3];
    state.left_trigger = controls.triggers[0]; state.right_trigger = controls.triggers[1];
    memcpy(state.buttons, controls.buttons, sizeof(state.buttons));
    if (has_battery) {
        state.has_battery = true;
        state.battery_percent = battery_percent;
    }
    last_input_tick = xTaskGetTickCount();
    ++state.sample_count;
    portEXIT_CRITICAL(&state_lock);
}

void dualsense_poll(void)
{
    ps5_transport_poll();
    dualsense_input_report_t report;
    while (report_queue && xQueueReceive(report_queue, &report, 0) == pdTRUE)
        process_input_report(&report);
    // Mark stale controls once; do not restart the radio or reconnect in a loop.
    bool stale = false;
    TickType_t now = xTaskGetTickCount();
    portENTER_CRITICAL(&state_lock);
    if (state.has_input && (TickType_t)(now - last_input_tick) > pdMS_TO_TICKS(2000)) {
        state.has_input = false;
        stale = true;
    }
    portEXIT_CRITICAL(&state_lock);
    if (stale) ESP_LOGW(TAG, "HID input stale: no decoded reports for 2 seconds");
    if (connecting && (int32_t)(xTaskGetTickCount() - connect_deadline) >= 0) {
        connecting = false;
        clear_candidate();
        ps5_transport_disconnect();
        set_connection_state(DUALSENSE_STATE_ERROR);
        ESP_LOGW(TAG, "HID connection timed out; discovery remains available");
    }
}

void dualsense_set_indicators(uint8_t red, uint8_t green, uint8_t blue, uint8_t players)
{
    ps5_transport_set_indicators(red, green, blue, players);
}

esp_err_t dualsense_init(void)
{
    if (hidh_initialized) return ESP_OK;

    portENTER_CRITICAL(&state_lock);
    memset(&state, 0, sizeof(state));
    memset(scanned_devices, 0, sizeof(scanned_devices));
    scanned_device_count = 0;
    candidate_available = false;
    state.initialized = true;
    state.state = DUALSENSE_STATE_READY;
    portEXIT_CRITICAL(&state_lock);

    esp_bd_addr_t known;
    if (saved_address(known)) {
        set_address(known);
        portENTER_CRITICAL(&state_lock);
        state.has_saved_controller = true;
        portEXIT_CRITICAL(&state_lock);
    }

    ESP_LOGW(TAG, "HID stage: register Classic GAP callback");
    esp_err_t result = esp_bt_gap_register_callback(bt_gap_callback);
    if (result != ESP_OK) return result;

    ESP_LOGW(TAG, "HID stage: configure pairing");
    esp_bt_io_cap_t io_capability = ESP_BT_IO_CAP_NONE;
    result = esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &io_capability,
                                           sizeof(io_capability));
    if (result != ESP_OK) return result;
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code = {0};
    result = esp_bt_gap_set_pin(pin_type, 0, pin_code);
    if (result != ESP_OK) return result;
    result = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    if (result != ESP_OK) return result;

    report_queue = xQueueCreate(128, sizeof(dualsense_input_report_t));
    if (!report_queue) return ESP_ERR_NO_MEM;

    result = ps5_transport_init((ps5_transport_callbacks_t){
        .opened = transport_opened, .closed = transport_closed, .input = transport_input,
    });
    if (result != ESP_OK) {
        vQueueDelete(report_queue);
        report_queue = NULL;
        return result;
    }
    hidh_initialized = true;
    ESP_LOGW(TAG, "DualSense HID host ready");
    return ESP_OK;
}

esp_err_t dualsense_start_pairing(void)
{
    if (!hidh_initialized || active_device) return ESP_ERR_INVALID_STATE;
    if (connecting) {
        connecting = false;
        clear_candidate();
        ps5_transport_disconnect();
    }
    esp_bt_gap_cancel_discovery();
    portENTER_CRITICAL(&state_lock);
    memset(scanned_devices, 0, sizeof(scanned_devices));
    scanned_device_count = 0;
    candidate_available = false;
    portEXIT_CRITICAL(&state_lock);
    set_connection_state(DUALSENSE_STATE_SCANNING);
    const esp_err_t result = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY,
                                                         DUALSENSE_SCAN_SECONDS, 0);
    if (result != ESP_OK) set_connection_state(DUALSENSE_STATE_ERROR);
    else ESP_LOGW(TAG, "scanning Classic BT devices; hold Create + PS on DualSense");
    return result;
}

esp_err_t dualsense_connect_discovered(const uint8_t address[6])
{
    if (!hidh_initialized || active_device || !address) return ESP_ERR_INVALID_STATE;

    // Device-result frames can reach the app just as a new discovery update
    // is pending. The selected address is authoritative, so retain it directly
    // and let HID report a real failure only if the controller disappeared.
    portENTER_CRITICAL(&state_lock);
    memcpy(candidate_bda, address, ESP_BD_ADDR_LEN);
    candidate_available = true;
    portEXIT_CRITICAL(&state_lock);

    set_address(address);
    // Queue cancel and connect directly. Waiting for a discovery-stop event can
    // deadlock when discovery has already ended before the selection arrives.
    esp_bt_gap_cancel_discovery();
    return open_selected_controller();
}

esp_err_t dualsense_connect_saved(void)
{
    if (!hidh_initialized || active_device) return ESP_ERR_INVALID_STATE;
    esp_bd_addr_t address;
    if (!saved_address(address)) return ESP_ERR_NOT_FOUND;

    set_address(address);
    portENTER_CRITICAL(&state_lock);
    memcpy(candidate_bda, address, ESP_BD_ADDR_LEN);
    candidate_available = true;
    portEXIT_CRITICAL(&state_lock);
    return open_selected_controller();
}

esp_err_t dualsense_disconnect(void)
{
    if (!active_device && !connecting) return ESP_ERR_NOT_FOUND;
    connecting = false;
    clear_candidate();
    return ps5_transport_disconnect();
}

esp_err_t dualsense_forget(void)
{
    if (active_device) return ESP_ERR_INVALID_STATE;
    esp_bd_addr_t address;
    if (!saved_address(address)) return ESP_ERR_NOT_FOUND;
    const esp_err_t result = esp_bt_gap_remove_bond_device(address);
    erase_saved_address();
    portENTER_CRITICAL(&state_lock);
    state.has_saved_controller = false;
    memset(state.address, 0, sizeof(state.address));
    portEXIT_CRITICAL(&state_lock);
    return result == ESP_OK || result == ESP_FAIL ? ESP_OK : result;
}

esp_err_t dualsense_deinit(void)
{
    if (!hidh_initialized) return ESP_OK;
    esp_bt_gap_cancel_discovery();
    if (active_device || connecting) ps5_transport_disconnect();
    connecting = false;
    active_device = false;
    ps5_transport_deinit();
    if (report_queue) {
        vQueueDelete(report_queue);
        report_queue = NULL;
    }
    const esp_err_t result = ESP_OK;
    hidh_initialized = false;
    portENTER_CRITICAL(&state_lock);
    state.initialized = false;
    state.state = DUALSENSE_STATE_OFF;
    portEXIT_CRITICAL(&state_lock);
    return result;
}

void dualsense_get_snapshot(dualsense_snapshot_t *snapshot)
{
    if (!snapshot) return;
    portENTER_CRITICAL(&state_lock);
    *snapshot = state;
    portEXIT_CRITICAL(&state_lock);
}

size_t dualsense_get_scanned_devices(dualsense_scan_device_t *devices, size_t capacity)
{
    if (!devices || !capacity) return 0;
    portENTER_CRITICAL(&state_lock);
    const size_t count = scanned_device_count < capacity ? scanned_device_count : capacity;
    memcpy(devices, scanned_devices, count * sizeof(*devices));
    portEXIT_CRITICAL(&state_lock);
    return count;
}

void dualsense_print_diagnostics(void)
{
    portENTER_CRITICAL(&state_lock);
    uint32_t raw = raw_reports, rejected = rejected_reports;
    uint16_t length = last_raw_length;
    uint8_t id = last_raw_id;
    portEXIT_CRITICAL(&state_lock);
    printf("HID RX: raw=%lu rejected=%lu last_id=0x%02x length=%u heap=%lu\n",
           (unsigned long)raw, (unsigned long)rejected, id, length,
           (unsigned long)esp_get_free_heap_size());
}
