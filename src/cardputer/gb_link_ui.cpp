#include "gb_link_ui.h"
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <vector>
#include <string>
#include <algorithm>
#include "cardputer/VerticalSelector.h"
#include "share/emu_log_cpp.h"

#define GB_PAIR_MAGIC 0x47424C4B // 'GBLK'

enum GbPairMsgType : uint8_t {
    GB_PAIR_MSG_BEACON  = 0x01,
    GB_PAIR_MSG_CONNECT = 0x02,
    GB_PAIR_MSG_ACK     = 0x03
};

#pragma pack(push, 1)
struct GbPairPacket {
    uint32_t magic;
    uint8_t  msg_type;
    uint8_t  sender_mac[6];
    uint8_t  target_mac[6];
};
#pragma pack(pop)

struct DiscoveredPeer {
    uint8_t  mac[6];
    uint32_t last_seen_ms;
};

GbLinkConfig g_gb_link_config = {
    .peer_mac = {0},
    .enabled = false
};

static uint8_t s_broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static Preferences s_prefs;

// Pairing state
static volatile bool s_pairing_connected = false;
static uint8_t s_partner_mac[6] = {0};
static uint8_t s_my_mac[6] = {0};
static std::vector<DiscoveredPeer> s_discovered_peers;
static portMUX_TYPE s_pair_mux = portMUX_INITIALIZER_UNLOCKED;

static void format_mac_short(const uint8_t* mac, char* out, size_t len) {
    snprintf(out, len, "%02X:%02X:%02X", mac[3], mac[4], mac[5]);
}

// --- ESP-NOW Receive Callback during Pairing ---
static void on_pair_recv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    if (data_len < (int)sizeof(GbPairPacket)) return;

    const GbPairPacket* pkt = (const GbPairPacket*)data;
    if (pkt->magic != GB_PAIR_MAGIC) return;

    if (pkt->msg_type == GB_PAIR_MSG_BEACON) {
        // Discovered an announcer
        portENTER_CRITICAL_ISR(&s_pair_mux);
        bool found = false;
        uint32_t now = millis();
        for (auto& p : s_discovered_peers) {
            if (memcmp(p.mac, pkt->sender_mac, 6) == 0) {
                p.last_seen_ms = now;
                found = true;
                break;
            }
        }
        if (!found && s_discovered_peers.size() < 8) {
            DiscoveredPeer dp;
            memcpy(dp.mac, pkt->sender_mac, 6);
            dp.last_seen_ms = now;
            s_discovered_peers.push_back(dp);
        }
        portEXIT_CRITICAL_ISR(&s_pair_mux);
    }
    else if (pkt->msg_type == GB_PAIR_MSG_CONNECT) {
        // Someone is connecting to me!
        if (memcmp(pkt->target_mac, s_my_mac, 6) == 0) {
            memcpy(s_partner_mac, pkt->sender_mac, 6);
            s_pairing_connected = true;

            // Send ACK back
            GbPairPacket ackPkt = {};
            ackPkt.magic = GB_PAIR_MAGIC;
            ackPkt.msg_type = GB_PAIR_MSG_ACK;
            memcpy(ackPkt.sender_mac, s_my_mac, 6);
            memcpy(ackPkt.target_mac, pkt->sender_mac, 6);

            if (!esp_now_is_peer_exist(pkt->sender_mac)) {
                esp_now_peer_info_t pi = {};
                memcpy(pi.peer_addr, pkt->sender_mac, 6);
                pi.channel = 1;
                pi.encrypt = false;
                esp_now_add_peer(&pi);
            }
            esp_now_send(pkt->sender_mac, (uint8_t*)&ackPkt, sizeof(ackPkt));
        }
    }
    else if (pkt->msg_type == GB_PAIR_MSG_ACK) {
        // Partner acknowledged our connection!
        if (memcmp(pkt->target_mac, s_my_mac, 6) == 0) {
            memcpy(s_partner_mac, pkt->sender_mac, 6);
            s_pairing_connected = true;
        }
    }
}

// --- NVS Storage ---
static bool load_saved_peer(uint8_t* out_mac) {
    s_prefs.begin("gblink", true);
    bool valid = s_prefs.getBool("has_peer", false);
    if (valid) {
        s_prefs.getBytes("peer_mac", out_mac, 6);
    }
    s_prefs.end();
    return valid;
}

static void save_peer(const uint8_t* mac) {
    s_prefs.begin("gblink", false);
    s_prefs.putBool("has_peer", true);
    s_prefs.putBytes("peer_mac", mac, 6);
    s_prefs.end();
}

static void clear_saved_peer() {
    s_prefs.begin("gblink", false);
    s_prefs.remove("has_peer");
    s_prefs.remove("peer_mac");
    s_prefs.end();
}

// --- Radio Init ---
static bool s_radio_ready = false;

static bool init_pairing_radio() {
    if (!s_radio_ready) {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        esp_wifi_init(&cfg);
        esp_wifi_set_storage(WIFI_STORAGE_RAM);
        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_start();
        esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
        esp_now_init();
        s_radio_ready = true;
    }

    esp_wifi_get_mac(WIFI_IF_STA, s_my_mac);

    // Register broadcast peer
    if (!esp_now_is_peer_exist(s_broadcast_mac)) {
        esp_now_peer_info_t bpi = {};
        memcpy(bpi.peer_addr, s_broadcast_mac, 6);
        bpi.channel = 1;
        bpi.encrypt = false;
        esp_now_add_peer(&bpi);
    }

    esp_now_register_recv_cb(on_pair_recv);
    return true;
}

// --- Share My ID (Announcer / Wait) ---
static GbLinkConfig run_share_my_id(CardputerView& display, CardputerInput& input) {
    GbLinkConfig config = { {0}, false };
    init_pairing_radio();

    s_pairing_connected = false;
    memset(s_partner_mac, 0, 6);

    GbPairPacket beacon = {};
    beacon.magic = GB_PAIR_MAGIC;
    beacon.msg_type = GB_PAIR_MSG_BEACON;
    memcpy(beacon.sender_mac, s_my_mac, 6);

    char shortId[16];
    format_mac_short(s_my_mac, shortId, sizeof(shortId));

    display.topBar("SHARING ID", false, false);

    // Pulisce l'area principale sotto la topbar
    M5Cardputer.Display.fillRect(0, 25, M5Cardputer.Display.width(), M5Cardputer.Display.height() - 25, BACKGROUND_COLOR);

    M5Cardputer.Display.fillRoundRect(10, 35, M5Cardputer.Display.width() - 20, 92, DEFAULT_ROUND_RECT, TFT_BLACK);
    M5Cardputer.Display.drawRoundRect(10, 35, M5Cardputer.Display.width() - 20, 92, DEFAULT_ROUND_RECT, PRIMARY_COLOR);
    M5Cardputer.Display.setTextColor(PRIMARY_COLOR);
    M5Cardputer.Display.setTextSize(TEXT_BIG);

    char idStr[32];
    snprintf(idStr, sizeof(idStr), "ID: [%s]", shortId);
    int offset = (M5Cardputer.Display.width() - (int)(strlen(idStr) * 12)) / 2;
    M5Cardputer.Display.setCursor(offset > 10 ? offset : 15, 52);
    M5Cardputer.Display.printf("%s", idStr);

    M5Cardputer.Display.setTextSize(TEXT_WIDE);
    M5Cardputer.Display.setTextColor(TEXT_COLOR);
    const char* l2 = "Waiting partner...";
    int l2Off = (M5Cardputer.Display.width() - (int)(strlen(l2) * 8)) / 2;
    M5Cardputer.Display.setCursor(l2Off > 10 ? l2Off : 15, 80);
    M5Cardputer.Display.printf("%s", l2);

    M5Cardputer.Display.setTextSize(TEXT_SMALL);
    const char* l3 = "Press ESC to cancel";
    int l3Off = (M5Cardputer.Display.width() - (int)(strlen(l3) * 6)) / 2;
    M5Cardputer.Display.setCursor(l3Off > 10 ? l3Off : 15, 105);
    M5Cardputer.Display.printf("%s", l3);

    uint32_t lastBeacon = 0;
    while (!s_pairing_connected) {
        uint32_t now = millis();
        if (now - lastBeacon >= 250) {
            lastBeacon = now;
            esp_now_send(s_broadcast_mac, (uint8_t*)&beacon, sizeof(beacon));
        }

        char key = input.readChar();
        if (key == KEY_ESC_CUSTOM || key == KEY_ARROW_LEFT) {
            display.subMessage("Cancelled", 600);
            return config;
        }
        delay(10);
    }

    // Connected!
    save_peer(s_partner_mac);
    memcpy(config.peer_mac, s_partner_mac, 6);
    config.enabled = true;

    char partnerShort[16];
    format_mac_short(s_partner_mac, partnerShort, sizeof(partnerShort));
    char msg[32];
    snprintf(msg, sizeof(msg), "Linked: [%s]", partnerShort);

    display.topBar("LINK CONNECTED", false, false);
    display.subMessage(msg, 1200);

    return config;
}

// --- Find Partner (Scanner / Connect) ---
static GbLinkConfig run_find_partner(CardputerView& display, CardputerInput& input) {
    GbLinkConfig config = { {0}, false };
    init_pairing_radio();

    portENTER_CRITICAL(&s_pair_mux);
    s_discovered_peers.clear();
    portEXIT_CRITICAL(&s_pair_mux);

    display.topBar("SCANNING PEERS", false, false);
    display.subMessage("Searching...", 0);

    VerticalSelector selector(display, input);

    int selectedIdx = -1;
    for (;;) {
        char key = input.readChar();
        if (key == KEY_ESC_CUSTOM || key == KEY_ARROW_LEFT) {
            display.subMessage("Cancelled", 600);
            return config;
        }

        std::vector<DiscoveredPeer> peersCopy;
        portENTER_CRITICAL(&s_pair_mux);
        peersCopy = s_discovered_peers;
        portEXIT_CRITICAL(&s_pair_mux);

        if (!peersCopy.empty()) {
            std::vector<std::string> options;
            for (const auto& p : peersCopy) {
                char item[24];
                char sh[16];
                format_mac_short(p.mac, sh, sizeof(sh));
                snprintf(item, sizeof(item), "Peer: [%s]", sh);
                options.push_back(item);
            }

            int choice = selector.select("SELECT PARTNER", options, false, false);
            if (choice >= 0 && choice < (int)peersCopy.size()) {
                selectedIdx = choice;
                break;
            } else if (choice == -1) {
                display.subMessage("Cancelled", 600);
                return config;
            }
        }
        delay(50);
    }

    DiscoveredPeer chosen = s_discovered_peers[selectedIdx];

    // Invia pacchetto di connessione
    GbPairPacket connPkt = {};
    connPkt.magic = GB_PAIR_MAGIC;
    connPkt.msg_type = GB_PAIR_MSG_CONNECT;
    memcpy(connPkt.sender_mac, s_my_mac, 6);
    memcpy(connPkt.target_mac, chosen.mac, 6);

    if (!esp_now_is_peer_exist(chosen.mac)) {
        esp_now_peer_info_t pi = {};
        memcpy(pi.peer_addr, chosen.mac, 6);
        pi.channel = 1;
        pi.encrypt = false;
        esp_now_add_peer(&pi);
    }

    s_pairing_connected = false;
    for (int i = 0; i < 5; ++i) {
        esp_now_send(chosen.mac, (uint8_t*)&connPkt, sizeof(connPkt));
        delay(25);
    }

    // Attendi ACK brevemente
    uint32_t waitStart = millis();
    while (!s_pairing_connected && (millis() - waitStart < 1500)) {
        delay(10);
    }

    save_peer(chosen.mac);
    memcpy(config.peer_mac, chosen.mac, 6);
    config.enabled = true;

    char chosenShort[16];
    format_mac_short(chosen.mac, chosenShort, sizeof(chosenShort));
    char msg[32];
    snprintf(msg, sizeof(msg), "Linked: [%s]", chosenShort);

    display.topBar("LINK CONNECTED", false, false);
    display.subMessage(msg, 1200);

    return config;
}

// --- Main Menu Entrance ---
GbLinkConfig gb_link_show_menu(CardputerView& display, CardputerInput& input) {
    uint8_t savedMac[6] = {0};
    bool hasSaved = load_saved_peer(savedMac);

    VerticalSelector selector(display, input);

    while (true) {
        std::vector<std::string> options;
        options.push_back("Single Player");

        char lastOpt[24] = {0};
        if (hasSaved) {
            char sh[16];
            format_mac_short(savedMac, sh, sizeof(sh));
            snprintf(lastOpt, sizeof(lastOpt), "Last: [%s]", sh);
            options.push_back(lastOpt);
        }

        options.push_back("Find Partner");
        options.push_back("Share My ID");

        if (hasSaved) {
            options.push_back("Clear Saved Partner");
        }

        int choice = selector.select("GB LINK CABLE", options, false, false);

        // Single Player o ESC
        if (choice == -1 || choice == 0) {
            g_gb_link_config.enabled = false;
            return g_gb_link_config;
        }

        int idx = 1;
        if (hasSaved) {
            // Cliccato "Last: [XX:XX:XX]"
            if (choice == idx) {
                memcpy(g_gb_link_config.peer_mac, savedMac, 6);
                g_gb_link_config.enabled = true;

                char sh[16];
                format_mac_short(savedMac, sh, sizeof(sh));
                char msg[32];
                snprintf(msg, sizeof(msg), "Using: [%s]", sh);

                display.topBar("GB LINK CABLE", false, false);
                display.subMessage(msg, 800);
                return g_gb_link_config;
            }
            idx++;
        }

        // Cliccato "Find Partner"
        if (choice == idx) {
            g_gb_link_config = run_find_partner(display, input);
            if (g_gb_link_config.enabled) return g_gb_link_config;
            hasSaved = load_saved_peer(savedMac);
            continue;
        }
        idx++;

        // Cliccato "Share My ID"
        if (choice == idx) {
            g_gb_link_config = run_share_my_id(display, input);
            if (g_gb_link_config.enabled) return g_gb_link_config;
            hasSaved = load_saved_peer(savedMac);
            continue;
        }
        idx++;

        // Cliccato "Clear Saved Partner"
        if (hasSaved && choice == idx) {
            clear_saved_peer();
            hasSaved = false;
            display.topBar("GB LINK CABLE", false, false);
            display.subMessage("Partner Cleared", 800);
            continue;
        }
    }
}
