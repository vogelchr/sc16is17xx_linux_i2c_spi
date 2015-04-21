#ifndef _SC16IS7XX_H
#define _SC16IS7XX_H

#include <stdlib.h>
#include <unistd.h>

extern struct sc16is7xx *
sc16is7xx_new(int spi_i2c_fd, int i2c_addr, unsigned int flags);

extern int
sc16is7xx_set_baud(struct sc16is7xx *p, int baudrate);

extern ssize_t
sc16is7xx_read(struct sc16is7xx *p, void *buf, size_t length);

extern ssize_t
sc16is7xx_write(struct sc16is7xx *p, const void *buf, size_t length);

extern ssize_t
sc16is7xx_get_rxlvl(struct sc16is7xx *p);

extern ssize_t
sc16is7xx_get_txlvl(struct sc16is7xx *p);

extern int
sc16is7xx_gpio_out(struct sc16is7xx *p, unsigned char iodir, unsigned char outval);

extern int
sc16is7xx_gpio_in(struct sc16is7xx *p, unsigned char *inval);


extern void
sc16is7xx_stats(struct sc16is7xx *p,
	unsigned int *xfers,
	unsigned int *txcounter,
	unsigned int *rxcounter);

#endif
