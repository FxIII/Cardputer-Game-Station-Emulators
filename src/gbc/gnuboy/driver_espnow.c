/*
 * driver_espnow.c
 *
 * ESP-NOW transport driver for GnuBoy Link Cable emulation on M5Cardputer.
 * Binary packet format (3 bytes): [TYPE: 'M' or 'S'][SEQ: uint8_t][VAL: uint8_t]
 */

#include "driver_espnow.h"
#include <stdio.h>
#include <string.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <rom/ets_sys.h>

#include "gnuboy.h"
#include "hw.h"

#define hw GB

#define MASTER_BLOCKING_TIMEOUT_US  100000  /* 100 ms timeout for Master waiting for Slave reply (allineato a driver_pipe) */
#define SLAVE_HOLD_TIMEOUT_US        60000  /* 60 ms hold for Slave to arm R_SC (allineato a driver_pipe) */

static uint8_t s_peer_mac[6] = {0};
static bool s_has_peer = false;
static bool s_espnow_initialized = false;

/* Reception state updated by ESP-NOW callback (Core 0) and consumed by emulator (Core 1) */
static volatile bool s_has_pending = false;
static volatile uint8_t s_pending_type = 0;   /* 'M' or 'S' */
static volatile uint8_t s_pending_seq = 0;
static volatile uint8_t s_pending_val = 0;
static volatile uint64_t s_hold_start_us = 0;

/* Master & Slave sequence numbers */
static uint8_t s_master_tx_seq = 0;
static uint8_t s_expected_reply_seq = 0;
static uint8_t s_slave_last_rx_seq = 0;
static bool s_is_master_mode = false;
static bool s_waiting_for_reply = false;

/*
 * ESP-NOW receive callback (runs on Core 0 Wi-Fi task)
 */
static void espnow_rx_callback(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    if (!data || len < 3) return;
    if (s_has_peer && memcmp(mac_addr, s_peer_mac, 6) != 0) return;

    uint8_t type = data[0];
    uint8_t seq  = data[1];
    uint8_t val  = data[2];

    if (type != 'M' && type != 'S') return;

    /* 1. Master mode: waiting for Slave reply */
    if (s_is_master_mode) {
        if (type == 'S') {
            if (seq == s_expected_reply_seq) {
                /* Exact match */
                s_pending_type = type;
                s_pending_seq = seq;
                s_pending_val = val;
                s_has_pending = true;
                s_is_master_mode = false;
                s_waiting_for_reply = false;
                return;
            } else if ((uint8_t)(s_expected_reply_seq - seq) < 128) {
                /* Stale reply, ignore */
                return;
            } else {
                /* Resync */
                s_expected_reply_seq = seq;
                s_pending_type = type;
                s_pending_seq = seq;
                s_pending_val = val;
                s_has_pending = true;
                s_is_master_mode = false;
                s_waiting_for_reply = false;
                return;
            }
        } else if (type == 'M') {
            /* Collision */
            s_pending_type = type;
            s_pending_seq = seq;
            s_pending_val = val;
            s_has_pending = true;
            s_is_master_mode = false;
            s_waiting_for_reply = false;
            return;
        }
    }

    /* 2. Slave mode / Receiver */
    if (type == 'M') {
        s_slave_last_rx_seq = seq;
        s_pending_type = type;
        s_pending_seq = seq;
        s_pending_val = val;
        s_hold_start_us = (uint64_t)esp_timer_get_time();
        s_has_pending = true;
    }
}

static void ensure_espnow_init(void)
{
    if (!s_espnow_initialized) {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        esp_wifi_init(&cfg);
        esp_wifi_set_storage(WIFI_STORAGE_RAM);
        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_start();
        esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
        esp_now_init();
        s_espnow_initialized = true;
    }
}

void driver_espnow_set_peer(const uint8_t *peer_mac)
{
    ensure_espnow_init();

    if (peer_mac) {
        memcpy(s_peer_mac, peer_mac, 6);
        s_has_peer = true;

        /* Ensure peer is registered in ESP-NOW on channel 1 */
        esp_now_peer_info_t peer;
        memset(&peer, 0, sizeof(peer));
        memcpy(peer.peer_addr, s_peer_mac, 6);
        peer.channel = 1;
        peer.encrypt = false;
        if (!esp_now_is_peer_exist(s_peer_mac)) {
            esp_now_add_peer(&peer);
        }
    } else {
        s_has_peer = false;
    }
}

static void espnow_init(void)
{
    ensure_espnow_init();

    s_has_pending = false;
    s_is_master_mode = false;
    s_waiting_for_reply = false;
    s_master_tx_seq = 0;
    s_expected_reply_seq = 0;
    s_slave_last_rx_seq = 0;
    s_hold_start_us = 0;

    esp_now_register_recv_cb(espnow_rx_callback);
}

static void espnow_cleanup(void)
{
    s_has_pending = false;
    s_is_master_mode = false;
    s_waiting_for_reply = false;
}

static void espnow_send(bool is_master, uint8_t byte)
{
    if (!s_has_peer) return;

    uint8_t pkt[3];
    if (is_master) {
        s_master_tx_seq++;
        s_expected_reply_seq = s_master_tx_seq;
        s_is_master_mode = true;
        s_waiting_for_reply = true;

        pkt[0] = 'M';
        pkt[1] = s_master_tx_seq;
        pkt[2] = byte;
    } else {
        s_is_master_mode = false;
        s_waiting_for_reply = false;

        pkt[0] = 'S';
        pkt[1] = s_slave_last_rx_seq;
        pkt[2] = byte;
    }

    esp_now_send(s_peer_mac, pkt, sizeof(pkt));
}

static bool espnow_received(bool blocking)
{
    /* If Master requested blocking wait, wait up to MASTER_BLOCKING_TIMEOUT_US */
    if (blocking && !s_has_pending) {
        uint64_t start = (uint64_t)esp_timer_get_time();
        while (!s_has_pending) {
            uint64_t now = (uint64_t)esp_timer_get_time();
            if (now - start >= MASTER_BLOCKING_TIMEOUT_US) {
                break;
            }
            ets_delay_us(50);
        }
    }

    if (!s_has_pending) {
        return false;
    }

    /* If incoming packet is from Master */
    if (s_pending_type == 'M') {
        /* If we are armed as Slave, deliver immediately! */
        if ((R_SC & 0x81) == 0x80) {
            return true;
        }

        /* If we are armed as Master (Collision scenario), deliver immediately */
        if ((R_SC & 0x81) == 0x81) {
            return true;
        }

        /* Slave Hold Mechanism: wait up to SLAVE_HOLD_TIMEOUT_US for ROM to arm R_SC */
        uint64_t now = (uint64_t)esp_timer_get_time();
        if (now - s_hold_start_us < SLAVE_HOLD_TIMEOUT_US) {
            return false; /* Keep holding */
        }

        /* Hold expired without arming: send passive 0xFF reply so Master doesn't hang */
        s_has_pending = false;
        uint8_t pkt[3] = {'S', s_slave_last_rx_seq, 0xFF};
        esp_now_send(s_peer_mac, pkt, sizeof(pkt));
        return false;
    }

    /* Master receiving Slave reply: deliver immediately */
    return true;
}

static uint8_t espnow_read(bool *out_is_master)
{
    if (out_is_master) {
        *out_is_master = (s_pending_type == 'M');
    }
    uint8_t val = s_pending_val;
    s_has_pending = false;
    return val;
}

static void espnow_cancel(uint8_t reason)
{
    (void)reason;
    s_waiting_for_reply = false;
    s_is_master_mode = false;
    s_has_pending = false;
    s_hold_start_us = 0;
}

serial_driver_t driver_espnow = {
    .name = "espnow",
    .init = espnow_init,
    .cleanup = espnow_cleanup,
    .send = espnow_send,
    .received = espnow_received,
    .read = espnow_read,
    .cancel = espnow_cancel
};
