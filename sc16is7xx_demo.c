#define _XOPEN_SOURCE 1 /* for ptsname */
#define _GNU_SOURCE   1 /* "" */

#include "sc16is7xx.h"

#include <unistd.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/select.h>
#include <fcntl.h>

#define DEFAULT_DEVICE    "/dev/i2c-0"
#define DEFAULT_I2C_ADDR  0x4d
#define DEFAULT_BAUDRATE  115200

void
usage(char *argv0)
{
	fprintf(stderr,"Usage: %s [options]\n", argv0);
	fprintf(stderr,"  -d  DEV    i2c or spi device node (df: %s)\n",
		DEFAULT_DEVICE);
	fprintf(stderr,"  -a  ADDR   select i2c address (0 = spi, def: %d)\n",
		DEFAULT_I2C_ADDR);
	fprintf(stderr,"  -b  BAUD   select baudrate (df: %d)\n",
		DEFAULT_BAUDRATE);
	fprintf(stderr,"  -g  0xNN    set gpio port to value N\n");
}

int
main(int argc, char **argv)
{
	int spi_i2c_fd;
	char *spi_i2c_devname = DEFAULT_DEVICE;
	struct sc16is7xx *sc16is7xx;
	ssize_t txlvl, rxlvl;
	unsigned char buf[128];
	int i2c_addr = DEFAULT_I2C_ADDR;
	int ptyfd, nfds, i, j;
	fd_set readfds, writefds;
	struct timeval to;
	int baudrate = DEFAULT_BAUDRATE;
	unsigned int xfers, txcounter, rxcounter;
	int gpio_out = -1;

	while ((i=getopt(argc, argv, "d:a:b:g:h")) != -1) {
		switch (i) {
		case 'd':
			spi_i2c_devname = optarg;
			break;
		case 'a':
			i2c_addr = atoi(optarg);
			break;
		case 'b':
			baudrate = atoi(optarg);
			break;
		case 'g':
			gpio_out = strtoul(optarg, NULL, 0);
			break;
		case 'h':
			usage(argv[0]);
			exit(1);
			break;
		}
	}

	ptyfd = open("/dev/ptmx", O_RDWR|O_NOCTTY|O_NONBLOCK);
	if (ptyfd == -1) {
		perror("/dev/ptmx");
		exit(1);
	}

	if (ptsname_r(ptyfd, (char*)buf, sizeof(buf))) {
		perror("ptsname_r()");
		exit(1);
	}

	fprintf(stderr,"Virtual terminal is %s.\n", buf);;

	if (grantpt(ptyfd) == -1) {
		perror("grantpt()");
		exit(1);
	}

	if (unlockpt(ptyfd) == -1) {
		perror("unlockpt()");
		exit(1);
	}

	if ((spi_i2c_fd = open(spi_i2c_devname, O_RDWR|O_NOCTTY)) == -1) {
		perror(spi_i2c_devname);
		exit(1);
	}

	sc16is7xx = sc16is7xx_new(spi_i2c_fd, i2c_addr, 0/*flags*/);
	if (!sc16is7xx) {
		fprintf(stderr, "Could not create sc16is7xx instance.\n");
		exit(1);
	}

	if (sc16is7xx_set_baud(sc16is7xx, baudrate) == -1) {
		perror("sc16is7xx_set_baud()");
		exit(1);
	}

	if (gpio_out != -1) {
		fprintf(stderr, "setting GPIO to 0x%04x\n", gpio_out & 0xff);
		for (i=7; i>=0; i--)
			fprintf(stderr," D%d=%d", i, !!(gpio_out & (1<<i)));
		fprintf(stderr,"\n");
		if (sc16is7xx_gpio_out(sc16is7xx, 0xff, gpio_out & 0xff) == -1) {
			perror("sc16is7xx_gpio_out");
			exit(1);
		}
	}

	fprintf(stderr,"Baudrate is %d.\n\n", baudrate);

	/* main loop */
	while (1) {
		FD_ZERO(&readfds);
		FD_ZERO(&writefds);
		nfds=0;

		/* are we able to write? */
		txlvl = sc16is7xx_get_txlvl(sc16is7xx);
		if (txlvl == -1) {
			perror("sc16is7xx_get_txlvl()");
			exit(1);
		}

		if (txlvl > 0) {
			if (txlvl > (signed int)sizeof(buf))
				txlvl = (signed int)sizeof(buf);
			/* so register for reading on pty*/
			FD_SET(ptyfd, &readfds);
			if (nfds < ptyfd + 1)
				nfds = ptyfd + 1;
		}

		/* is there something in the receive buffer? */
		rxlvl = sc16is7xx_get_rxlvl(sc16is7xx);
		if (rxlvl == -1) {
			perror("sc16is7xx_get_rxlvl()");
			exit(1);
		}

		if (rxlvl > 0) {
			if (rxlvl > (signed int)sizeof(buf))
				rxlvl = (signed int)sizeof(buf);
			/* so register for writing to the pty */
			FD_SET(ptyfd, &writefds);
			if (nfds < ptyfd + 1)
				nfds = ptyfd + 1;
		}

		to.tv_sec = 0;
		to.tv_usec = (rxlvl > 0) ? 0 : 1000; /* 1ms */

		i = select(nfds, &readfds, &writefds, NULL, &to);
		if (i == -1) {
			perror("select()");
			exit(1);
		}

		sc16is7xx_stats(sc16is7xx, &xfers, &txcounter, &rxcounter);
		fprintf(stderr,"\r%u %u %u %4d %4d\r",
			xfers, txcounter, rxcounter, txlvl, rxlvl);
		fflush(stderr);

		/* amuse Marco */
		if ((txlvl==64) && (rxlvl==0) && (gpio_out != -1)) {
			gpio_out = gpio_out ^ 0xff;
			if (sc16is7xx_gpio_out(sc16is7xx, 0xff, gpio_out & 0xff) == -1) {
				perror("sc16is7xx_gpio_out");
				exit(1);
			}
		}

		if (i == 0)
			continue;

		/* check if we can read from pty -> write to uart */
		if (FD_ISSET(ptyfd, &readfds)) {
			i = read(ptyfd, buf, txlvl);
			if (i < 0) {
				perror("read(pty)");
				exit(1);
			}

			j = sc16is7xx_write(sc16is7xx, buf, i);
			if (j < 0) {
				perror("sc16is7xx_write()");
				exit(1);
			}

			if (i != j) {
				fprintf(stderr,
					"Wrote only %d of %d bytes to UART!\n",
					j, i);
			}
		}

		if (FD_ISSET(ptyfd, &writefds)) {
			i = sc16is7xx_read(sc16is7xx, buf, sizeof(buf));
			if (i < 0) {
				perror("sc16is7xx_read()");
				exit(1);
			}

			j = write(ptyfd, buf, i);
			if (j < 0) {
				perror("write(pty)");
				exit(1);
			}

			if (i != j) {
				fprintf(stderr,
					"Wrote only %d of %d bytes to pty.",
					j, i);
			}
		}

	} /* while(1) */

	return 0;
}

