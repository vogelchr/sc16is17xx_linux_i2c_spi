#ifndef _SC16IS7XX_H
#define _SC16IS7XX_H

extern struct sc16is7xx *
sc16is7xx_new(int spi_i2c_fd, int irq_fd, int i2c_addr, unsigned int flags);

#endif