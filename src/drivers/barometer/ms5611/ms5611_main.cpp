/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ms5611.cpp
 * Driver for the MS5611 and MS5607 barometric pressure sensor connected via I2C or SPI.
 */

#include "ms5611.h"

#include <platforms/px4_getopt.h>

enum MS56XX_DEVICE_TYPES {
	MS56XX_DEVICE   = 0,
	MS5611_DEVICE	= 5611,
	MS5607_DEVICE	= 5607,
};

enum MS5611_BUS {
	MS5611_BUS_ALL = 0,
	MS5611_BUS_I2C_INTERNAL,
	MS5611_BUS_I2C_EXTERNAL,
	MS5611_BUS_SPI_INTERNAL,
	MS5611_BUS_SPI_EXTERNAL
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int ms5611_main(int argc, char *argv[]);

/**
 * Local functions in support of the shell command.
 */
namespace ms5611
{

/*
  list of supported bus configurations
 */
struct ms5611_bus_option {
	enum MS5611_BUS busid;
	MS5611_constructor interface_constructor;
	uint8_t busnum;
	MS5611 *dev;
} bus_options[] = {
#if defined(PX4_SPIDEV_EXT_BARO) && defined(PX4_SPI_BUS_EXT)
	{ MS5611_BUS_SPI_EXTERNAL, &MS5611_spi_interface, PX4_SPI_BUS_EXT, NULL },
#endif
#ifdef PX4_SPIDEV_BARO
	{ MS5611_BUS_SPI_INTERNAL, &MS5611_spi_interface, PX4_SPI_BUS_BARO, NULL },
#endif
#ifdef PX4_I2C_BUS_ONBOARD
	{ MS5611_BUS_I2C_INTERNAL, &MS5611_i2c_interface, PX4_I2C_BUS_ONBOARD, NULL },
#endif
#ifdef PX4_I2C_BUS_EXPANSION
	{ MS5611_BUS_I2C_EXTERNAL, &MS5611_i2c_interface, PX4_I2C_BUS_EXPANSION, NULL },
#endif
#ifdef PX4_I2C_BUS_EXPANSION1
	{ MS5611_BUS_I2C_EXTERNAL, &MS5611_i2c_interface, PX4_I2C_BUS_EXPANSION1, NULL },
#endif
#ifdef PX4_I2C_BUS_EXPANSION2
	{ MS5611_BUS_I2C_EXTERNAL, &MS5611_i2c_interface, PX4_I2C_BUS_EXPANSION2, NULL },
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

bool	start_bus(struct ms5611_bus_option &bus, enum MS56XX_DEVICE_TYPES device_type);
struct ms5611_bus_option &find_bus(enum MS5611_BUS busid);
int	start(enum MS5611_BUS busid, enum MS56XX_DEVICE_TYPES device_type);
void	test(enum MS5611_BUS busid);
void	reset(enum MS5611_BUS busid);
int	info();
void	usage();

/**
 * MS5611 crc4 cribbed from the datasheet
 */
bool
crc4(uint16_t *n_prom)
{
	int16_t cnt;
	uint16_t n_rem;
	uint16_t crc_read;
	uint8_t n_bit;

	n_rem = 0x00;

	/* save the read crc */
	crc_read = n_prom[7];

	/* remove CRC byte */
	n_prom[7] = (0xFF00 & (n_prom[7]));

	for (cnt = 0; cnt < 16; cnt++) {
		/* uneven bytes */
		if (cnt & 1) {
			n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);

		} else {
			n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
		}

		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;

			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	/* final 4 bit remainder is CRC value */
	n_rem = (0x000F & (n_rem >> 12));
	n_prom[7] = crc_read;

	/* return true if CRCs match */
	return (0x000F & crc_read) == (n_rem ^ 0x00);
}

/**
 * Start the driver.
 */
bool
start_bus(struct ms5611_bus_option &bus, enum MS56XX_DEVICE_TYPES device_type)
{
	if (bus.dev != nullptr) {
		errx(1, "bus option already started");
	}

	prom_u prom_buf;
	device::Device *interface = bus.interface_constructor(prom_buf, bus.busnum);

	if (interface->init() != OK) {
		delete interface;
		warnx("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	bus.dev = new MS5611(interface, prom_buf, device_type);

	if (bus.dev != nullptr && OK != bus.dev->init()) {
		delete bus.dev;
		bus.dev = NULL;
		return false;
	}

	return true;
}

/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
int
start(enum MS5611_BUS busid, enum MS56XX_DEVICE_TYPES device_type)
{
	uint8_t i;
	bool started = false;

	for (i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == MS5611_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != MS5611_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started = started | start_bus(bus_options[i], device_type);
	}

	if (!started) {
		return 1;
	}

	// one or more drivers started OK
	return 0;
}

/**
 * find a bus structure for a busid
 */
struct ms5611_bus_option &find_bus(enum MS5611_BUS busid)
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == MS5611_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
			return bus_options[i];
		}
	}

	errx(1, "bus %u not started", (unsigned)busid);
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		struct ms5611_bus_option &bus = bus_options[i];

		if (bus.dev != nullptr) {
			bus.dev->print_info();
		}
	}

	return 0;
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'test2'");
	warnx("options:");
	warnx("    -X    (external I2C bus)");
	warnx("    -I    (intternal I2C bus)");
	warnx("    -S    (external SPI bus)");
	warnx("    -s    (internal SPI bus)");
	warnx("    -T    5611|5607 (default 5611)");
	warnx("    -T    0 (autodetect version)");

}

} // namespace

int
ms5611_main(int argc, char *argv[])
{
	enum MS5611_BUS busid = MS5611_BUS_ALL;
	enum MS56XX_DEVICE_TYPES device_type = MS5611_DEVICE;
	int ch;
	int myoptind = 1;
	const char *myoptarg = NULL;

	/* jump over start/off/etc and look at options first */
	while ((ch = px4_getopt(argc, argv, "T:XISs", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			busid = MS5611_BUS_I2C_EXTERNAL;
			break;

		case 'I':
			busid = MS5611_BUS_I2C_INTERNAL;
			break;

		case 'S':
			busid = MS5611_BUS_SPI_EXTERNAL;
			break;

		case 's':
			busid = MS5611_BUS_SPI_INTERNAL;
			break;

		case 'T': {
				int val = atoi(myoptarg);

				if (val == 5611) {
					device_type = MS5611_DEVICE;
					break;

				} else if (val == 5607) {
					device_type = MS5607_DEVICE;
					break;

				} else if (val == 0) {
					device_type = MS56XX_DEVICE;
					break;
				}
			}

		/* FALLTHROUGH */

		default:
			ms5611::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		ms5611::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		return ms5611::start(busid, device_type);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		ms5611::test(busid);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		return ms5611::info();
	}

	PX4_ERR("unrecognised command, try 'start', 'test', or 'info'");
	return -1;
}
