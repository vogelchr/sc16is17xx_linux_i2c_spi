#include "sc16is7xx.h"

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define DEFAULT_DEVICE    "/dev/i2c-0"
#define DEFAULT_I2C_ADDR  0x4d

void
usage(char *argv0)
{
	fprintf(stderr,"Usage: %s [options]\n", argv0);
}

int
main(int argc, char **argv)
{
	int spi_i2c_fd;
	char *spi_i2c_devname = DEFAULT_DEVICE;
	struct sc16is7xx *sc16is7xx;
	int i2c_addr = DEFAULT_I2C_ADDR;
	int i;

	while ((i=getopt(argc, argv, "d:a:h")) != -1) {
		switch (i) {
		case 'd':
			spi_i2c_devname = optarg;
			break;
		case 'a':
			i2c_addr = atoi(optarg);
			break;
		case 'h':
			usage(argv[0]);
			exit(1);
			break;
		}
	}

	if ((spi_i2c_fd = open(spi_i2c_devname, O_RDWR)) == -1) {
		perror(spi_i2c_devname);
		exit(1);
	}

	sc16is7xx = sc16is7xx_new(spi_i2c_fd, -1, i2c_addr, 0/*flags*/);
	if (!sc16is7xx) {
		fprintf(stderr, "Could not create sc16is7xx instance.\n");
		exit(1);
	}

	return 0;
}
