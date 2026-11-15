#ifndef GB_LINK_UI_H
#define GB_LINK_UI_H

#include <stdint.h>
#include <stdbool.h>
#include "cardputer/CardputerView.h"
#include "cardputer/CardputerInput.h"

struct GbLinkConfig {
    uint8_t peer_mac[6];
    bool enabled;
};

extern GbLinkConfig g_gb_link_config;

// Mostra il menu Link prima di avviare il gioco Game Boy
GbLinkConfig gb_link_show_menu(CardputerView& display, CardputerInput& input);

#endif // GB_LINK_UI_H
