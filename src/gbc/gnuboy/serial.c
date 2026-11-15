/*
 * serial.c
 *
 * Game Boy Serial Link emulation core & Null driver for Cardputer GnuBoy.
 */

#include <stdio.h>
#include <string.h>
#include "gnuboy.h"
#include "hw.h"
#include "serial.h"

#define hw GB

/*
 * -------------------------------------------------------------
 * Null Driver: Fallback when no cable/network peer is connected.
 * -------------------------------------------------------------
 */
static void null_send(bool is_master, uint8_t byte)
{
	(void)is_master;
	(void)byte;
}

static bool null_received(bool blocking)
{
	(void)blocking;
	return false;
}

static uint8_t null_read(bool *out_is_master)
{
	if (out_is_master)
		*out_is_master = false;
	return 0xFF;
}

static void null_cancel(uint8_t reason)
{
	(void)reason;
}

serial_driver_t driver_null = {
	.name = "null",
	.init = NULL,
	.cleanup = NULL,
	.send = null_send,
	.received = null_received,
	.read = null_read,
	.cancel = null_cancel
};

/*
 * -------------------------------------------------------------
 * Core Game Boy Serial State & Wrappers
 * -------------------------------------------------------------
 */
static serial_driver_t *active_driver = &driver_null;

static struct {
	int timer;
	bool pending_tx;
	bool just_armed;
	bool just_disarmed;
	bool switched_to_master;
	bool switched_to_slave;
	bool master_has_early_rx;
	uint8_t master_early_rx_byte;
	bool master_early_from_master;
	int slave_cooldown;
} serial_state;

static inline void set_slave_cooldown(void)
{
	serial_state.slave_cooldown = ((hw.hwtype == GB_HW_CGB) && (R_SC & 0x02)) ? 256 : 4096;
}

static inline bool drv_received(bool blocking)
{
	return (active_driver && active_driver->received) ? active_driver->received(blocking) : false;
}

static inline uint8_t drv_read(bool *out_is_master)
{
	return (active_driver && active_driver->read) ? active_driver->read(out_is_master) : 0xFF;
}

static inline void drv_send(bool is_master, uint8_t byte)
{
	if (active_driver && active_driver->send)
		active_driver->send(is_master, byte);
}

static inline void drv_cancel(uint8_t reason)
{
	if (active_driver && active_driver->cancel)
		active_driver->cancel(reason);
}

static inline void raise_serial_interrupt(void)
{
	gb_hw_interrupt(IF_SERIAL, 1);
	gb_hw_interrupt(IF_SERIAL, 0);
}

void serial_set_driver(serial_driver_t *drv)
{
	if (active_driver && active_driver->cleanup)
		active_driver->cleanup();

	active_driver = drv ? drv : &driver_null;

	if (active_driver->init)
		active_driver->init();
}

serial_driver_t *serial_get_driver(void)
{
	return active_driver;
}

void serial_init(void)
{
	serial_set_driver(&driver_null);
	serial_reset();
}

void serial_reset(void)
{
	serial_state.timer = 0;
	serial_state.pending_tx = false;
	serial_state.just_armed = false;
	serial_state.just_disarmed = false;
	serial_state.switched_to_master = false;
	serial_state.switched_to_slave = false;
	serial_state.master_has_early_rx = false;
	serial_state.slave_cooldown = 0;
	drv_cancel(0x00);
}

void serial_write_sb(byte b)
{
	R_SB = b;
}

byte serial_read_sb(void)
{
	return R_SB;
}

void serial_write_sc(byte b)
{
	byte old_sc = R_SC;
	byte new_sc = b;

	serial_state.just_armed         = !(old_sc & 0x80) &&  (new_sc & 0x80);
	serial_state.just_disarmed      =  (old_sc & 0x80) && !(new_sc & 0x80);
	serial_state.switched_to_master = !(old_sc & 0x01) &&  (new_sc & 0x01);
	serial_state.switched_to_slave  =  (old_sc & 0x01) && !(new_sc & 0x01);

	if (serial_state.just_armed && (new_sc & 0x01))
		serial_state.pending_tx = true;
	else if (serial_state.just_disarmed)
	{
		serial_state.pending_tx = false;
		serial_state.timer = 0;
		serial_state.master_has_early_rx = false;
		drv_cancel(0x01);
	}

	R_SC = b;
}

byte serial_read_sc(void)
{
	return R_SC;
}

/*
 * Advance serial link timing by given CPU cycles (called synchronously from CPU loop).
 */
void serial_advance(int cycles)
{
	/* Cooldown guard for Slave reception */
	if (serial_state.slave_cooldown > 0)
	{
		int remaining = serial_state.slave_cooldown - cycles;
		serial_state.slave_cooldown = (remaining > 0) ? remaining : 0;
	}

	/* 1. GOT SOMETHING (non-blocking poll) */
	/* If we are Slave and still in cooldown, do not poll/receive yet! */
	if ((R_SC & 0x81) == 0x80 && serial_state.slave_cooldown > 0)
	{
		/* Wait until cooldown finishes */
	}
	else if (drv_received(false))
	{
		bool from_master = false;
		uint8_t in = drv_read(&from_master);

		// SLAVE: send register (as Slave: false), update register, reset armed bit, trigger interrupt - done
		if ((R_SC & 0x81) == 0x80) {
			if (!from_master) return; // ignore non-master packets
			set_slave_cooldown();
			drv_send(false, R_SB);
			R_SB = in;
			R_SC &= 0x7F;
			raise_serial_interrupt();
			return;
		}

		// MASTER: peer replied early! Hold the byte until the 4096 hardware cycles finish!
		if ((R_SC & 0x81) == 0x81) {
			serial_state.master_has_early_rx = true;
			serial_state.master_early_rx_byte = in;
			serial_state.master_early_from_master = from_master;
			return;
		}

		/* UNARMED PEER RECEIVING MASTER PROBE:
		 * Hardware pull-up simulation. Master clocked the wire, but our shift register is not enabled.
		 * Send passive 0xFF back to Master so it doesn't hang in timeout. No local interrupt/register change.
		 */
		if (from_master) {
			drv_send(false, 0xFF);
			return;
		}

		return;
	}

	/* ARMED MASTER */
	if ((R_SC & 0x81) == 0x81)
	{
		// FIRST RUN: send register (as Master: true), setup hardware timer (4096 cycles), reset first run
		if (serial_state.pending_tx)
		{
			drv_send(true, R_SB);
			serial_state.timer = ((hw.hwtype == GB_HW_CGB) && (R_SC & 0x02)) ? 256 : 4096;
			serial_state.pending_tx = false;
			serial_state.master_has_early_rx = false;
			return;
		}
		// ELAPSE TIME
		serial_state.timer -= cycles;
		// HARDWARE SHIFT CLOCK FINISHED (4096 cycles elapsed)
		if (serial_state.timer <= 0)
		{
			/* 1. Reply arrived early during the 4096 cycles */
			if (serial_state.master_has_early_rx)
			{
				serial_state.master_has_early_rx = false;
				uint8_t in = serial_state.master_early_rx_byte;
				if (serial_state.master_early_from_master)
				{
					// Master collision: two masters probed simultaneously
					drv_cancel(0x03);
					serial_state.timer = 0;
					R_SB = in;
					R_SC &= 0x7F;
					raise_serial_interrupt();
					return;
				}

				serial_state.timer = 0;
				R_SB = in;
				R_SC &= 0x7F;
				raise_serial_interrupt();
				return;
			}

			/* 2. No reply yet: perform generous blocking wait */
			if (drv_received(true))
			{
				bool from_master = false;
				uint8_t in = drv_read(&from_master);

				if (from_master)
				{
					// Master collision
					drv_cancel(0x03);
					serial_state.timer = 0;
					R_SB = in;
					R_SC &= 0x7F;
					raise_serial_interrupt();
					return;
				}

				serial_state.timer = 0;
				R_SB = in;
				R_SC &= 0x7F;
				raise_serial_interrupt();
				return;
			}

			// TRUE TIMEOUT: abort send, pretend read 0XFF, reset armed bit, trigger interrupt - done
			drv_cancel(0x02);
			R_SB = 0xFF;
			R_SC &= 0x7F;
			raise_serial_interrupt();
			return;
		}
		return;
	}
}
