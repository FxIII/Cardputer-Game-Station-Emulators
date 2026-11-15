#ifndef DRIVER_ESPNOW_H
#define DRIVER_ESPNOW_H

#include <stdint.h>
#include <stdbool.h>
#include "serial.h"

#ifdef __cplusplus
extern "C" {
#endif

extern serial_driver_t driver_espnow;

/* Configura il MAC del peer e abilita il driver */
void driver_espnow_set_peer(const uint8_t *peer_mac);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_ESPNOW_H */
