#ifndef DRIVER_ESPNOW_H
#define DRIVER_ESPNOW_H

#include <stdint.h>
#include <stdbool.h>
#include "serial.h"

#ifdef __cplusplus
extern "C" {
#endif

extern serial_driver_t driver_espnow;

/* Configure peer MAC and enable driver */
void driver_espnow_set_peer(const uint8_t *peer_mac);

/* Print link latency and telemetry statistics to USB Serial */
void driver_espnow_dump_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_ESPNOW_H */
