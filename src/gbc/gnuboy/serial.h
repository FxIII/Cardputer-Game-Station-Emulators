#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <stdbool.h>
#include <stdint.h>
#include "gnuboy.h"

/*
 * Driver interface for Game Boy Link cable transport.
 */
typedef struct serial_driver {
	const char *name;
	int  (*init)(void);
	void (*cleanup)(void);
	void (*send)(bool is_master, uint8_t byte);
	bool (*received)(bool blocking);
	uint8_t (*read)(bool *out_is_master);
	void (*cancel)(uint8_t reason);
} serial_driver_t;

extern serial_driver_t driver_null;

void serial_init(void);
void serial_reset(void);
void serial_set_driver(serial_driver_t *drv);
serial_driver_t *serial_get_driver(void);

void serial_write_sb(byte b);
byte serial_read_sb(void);
void serial_write_sc(byte b);
byte serial_read_sc(void);

void serial_advance(int cycles);

#endif /* __SERIAL_H__ */
