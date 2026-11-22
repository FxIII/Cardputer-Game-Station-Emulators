#include "espnow_transport.h"
#include <esp_now.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <string.h>

static void on_data_recv(const uint8_t *mac_addr, const uint8_t *data, int len) {
    if (!mac_addr || !data || len <= 0) return;

    // Dynamically register peer if missing to allow unicast reply
    if (!esp_now_is_peer_exist(mac_addr)) {
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, mac_addr, 6);
        peerInfo.channel = 1;
        peerInfo.encrypt = false;
        esp_now_add_peer(&peerInfo);
    }

    // Immediate echo of identical payload to sender, zero serial overhead
    esp_now_send(mac_addr, data, len);
}

extern "C" bool espnow_transport_init(void) {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&cfg);
    if (err != ESP_OK) return false;

    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        return false;
    }

    // Register broadcast peer by default
    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_peer_info_t bcast_peer = {};
    memcpy(bcast_peer.peer_addr, broadcast_mac, 6);
    bcast_peer.channel = 1;
    bcast_peer.encrypt = false;
    esp_now_add_peer(&bcast_peer);

    // Register receive callback
    esp_now_register_recv_cb(on_data_recv);

    return true;
}
