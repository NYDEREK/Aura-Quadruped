// Aura ESP-IDF port of the L2CAP approach in esp-ps5.
// Originally created by Hamza Yeşilmen (HamzaYslmn).
// Source: https://github.com/HamzaYslmn/esp-ps5
// Sponsor: https://github.com/sponsors/HamzaYslmn
// See ../diagnostics/libraries/esp-ps5/LICENSE. This port is modified.
#include "ps5_transport.h"
#include "dualsense_output.h"
#include <string.h>
#include "stack/l2c_api.h"
#include "stack/btm_api.h"
#include "osi/allocator.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ps5_l2cap";
static uint16_t control, interrupt;
static bool control_ready, interrupt_ready, announced, streaming, initialized;
static uint8_t peer[6], sequence;
static uint8_t lightbar_red = 255, lightbar_green = 96, lightbar_blue, player_leds;
static portMUX_TYPE indicator_lock = portMUX_INITIALIZER_UNLOCKED;
static unsigned kicks;
static TickType_t last_kick;
static ps5_transport_callbacks_t callbacks;
// The Bluedroid L2CAP API keeps this configuration object beyond the callback
// that submitted it.  It must therefore have static lifetime; a stack-local
// object eventually corrupts the connection state under sustained input.
static tL2CAP_CFG_INFO l2cap_cfg;

static void channel_down(uint16_t cid) {
    if (cid == control) { control = 0; control_ready = false; }
    if (cid == interrupt) { interrupt = 0; interrupt_ready = false; }
    if (announced && !(control_ready && interrupt_ready)) {
        announced = false; streaming = false; callbacks.closed();
    }
}
static void send_packet(uint16_t cid, const uint8_t *data, size_t size) {
    if (!cid) return;
    BT_HDR *buffer = osi_calloc(sizeof(BT_HDR) + L2CAP_MIN_OFFSET + size);
    if (!buffer) return;
    buffer->len = size; buffer->offset = L2CAP_MIN_OFFSET;
    memcpy(buffer->data + buffer->offset, data, size);
    uint8_t result = L2CA_DataWrite(cid, buffer);
    // L2CAP owns the buffer on every return path.
    ESP_LOGI(TAG, "TX cid=%u bytes=%u result=%u", cid, (unsigned)size, result);
}
static void send_lightbar(void) {
    uint8_t packet[DUALSENSE_LED_PACKET_SIZE];
    portENTER_CRITICAL(&indicator_lock);
    const uint8_t red = lightbar_red, green = lightbar_green, blue = lightbar_blue;
    const uint8_t players = player_leds, seq = sequence++;
    portEXIT_CRITICAL(&indicator_lock);
    dualsense_led_packet(packet, seq, red, green, blue, players);
    send_packet(interrupt, packet, sizeof(packet));
}
static void enable_stream(void) { send_lightbar(); }

void ps5_transport_set_indicators(uint8_t red, uint8_t green, uint8_t blue, uint8_t players) {
    portENTER_CRITICAL(&indicator_lock);
    lightbar_red = red;
    lightbar_green = green;
    lightbar_blue = blue;
    player_leds = players & 0x1f;
    portEXIT_CRITICAL(&indicator_lock);
    if (announced && interrupt_ready) send_lightbar();
}
static void configured(uint16_t cid, tL2CAP_CFG_INFO *cfg) {
    if (cfg->result != L2CAP_CFG_OK) return;
    if (cid == control) control_ready = true;
    if (cid == interrupt) interrupt_ready = true;
    ESP_LOGI(TAG, "configured cid=%u control=%d interrupt=%d", cid, control_ready, interrupt_ready);
    if (control_ready && interrupt_ready && !announced) {
        announced = true; kicks = 0; streaming = false;
        callbacks.opened(peer);
        const uint8_t feature[] = {0x53, 0xf4, 0x43, 0x02};
        send_packet(control, feature, sizeof(feature));
        last_kick = xTaskGetTickCount();
    }
}
static void incoming(BD_ADDR address, uint16_t cid, uint16_t psm, uint8_t id) {
    if ((control || interrupt) && memcmp(peer, address, 6)) {
        L2CA_ConnectRsp(address, id, cid, L2CAP_CONN_NO_RESOURCES, 0); return;
    }
    memcpy(peer, address, 6);
    if (psm == 0x11) control = cid; else interrupt = cid;
    // DualSense expects the normal HID L2CAP pending -> accepted sequence.
    L2CA_ConnectRsp(address, id, cid, L2CAP_CONN_PENDING, L2CAP_CONN_PENDING);
    L2CA_ConnectRsp(address, id, cid, L2CAP_CONN_OK, L2CAP_CONN_OK);
    L2CA_ConfigReq(cid, &l2cap_cfg);
}
static void connected(uint16_t cid, uint16_t result) {
    ESP_LOGI(TAG, "connect cid=%u result=%u", cid, result);
    if (result != L2CAP_CONN_OK) { channel_down(cid); return; }
    L2CA_ConfigReq(cid, &l2cap_cfg);
}
static void config_request(uint16_t cid, tL2CAP_CFG_INFO *cfg) {
    cfg->result = L2CAP_CFG_OK; L2CA_ConfigRsp(cid, cfg);
}
static void disconnected(uint16_t cid, bool ack) {
    ESP_LOGW(TAG, "disconnect cid=%u", cid);
    if (ack) L2CA_DisconnectRsp(cid);
    channel_down(cid);
}
static void disconnect_confirm(uint16_t cid, uint16_t result) { channel_down(cid); }
static void received(uint16_t cid, BT_HDR *buffer) {
    const uint8_t *p = buffer->data + buffer->offset;
    if (cid == interrupt && buffer->len >= 2 && p[0] == 0xa1) {
        if (p[1] == 0x31) streaming = true;
        callbacks.input(p + 1, buffer->len - 1);
    }
    osi_free(buffer);
}
static void congestion(uint16_t cid, bool congested) { ESP_LOGW(TAG, "congestion cid=%u value=%d", cid, congested); }
static tL2CAP_APPL_INFO handlers = {
    .pL2CA_ConnectInd_Cb = incoming, .pL2CA_ConnectCfm_Cb = connected,
    .pL2CA_ConfigInd_Cb = config_request, .pL2CA_ConfigCfm_Cb = configured,
    .pL2CA_DisconnectInd_Cb = disconnected, .pL2CA_DisconnectCfm_Cb = disconnect_confirm,
    .pL2CA_DataInd_Cb = received, .pL2CA_CongestionStatus_Cb = congestion,
};
esp_err_t ps5_transport_init(ps5_transport_callbacks_t cb) {
    callbacks = cb;
    memset(&l2cap_cfg, 0, sizeof(l2cap_cfg));
    if (!L2CA_Register(0x11, &handlers)) return ESP_FAIL;
    if (!L2CA_Register(0x13, &handlers)) { L2CA_Deregister(0x11); return ESP_FAIL; }
    BTM_SetSecurityLevel(false, "Aura HIDC", BTM_SEC_SERVICE_FIRST_EMPTY, 0, 0x11, 0, 0);
    BTM_SetSecurityLevel(false, "Aura HIDI", BTM_SEC_SERVICE_FIRST_EMPTY + 1, 0, 0x13, 0, 0);
    initialized = true; return ESP_OK;
}
esp_err_t ps5_transport_connect(const uint8_t address[6]) {
    if (!initialized || control || interrupt) return ESP_ERR_INVALID_STATE;
    memcpy(peer, address, 6);
    control = L2CA_ConnectReq(0x11, peer);
    return control ? ESP_OK : ESP_FAIL;
}
esp_err_t ps5_transport_disconnect(void) {
    if (control) L2CA_DisconnectReq(control);
    if (interrupt) L2CA_DisconnectReq(interrupt);
    return ESP_OK;
}
void ps5_transport_poll(void) {
    if (announced && !streaming && kicks < 10 &&
        (TickType_t)(xTaskGetTickCount() - last_kick) >= pdMS_TO_TICKS(400)) {
        last_kick = xTaskGetTickCount(); ++kicks; enable_stream();
    }
}
void ps5_transport_deinit(void) {
    if (!initialized) return;
    ps5_transport_disconnect();
    L2CA_Deregister(0x11); L2CA_Deregister(0x13);
    control = interrupt = 0; control_ready = interrupt_ready = announced = streaming = initialized = false;
}
