#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdlib.h>
#include <string.h>

struct sc16is7xx {
	int spi_i2c_fd;
	int irq_fd;
	int i2c_addr;
	unsigned int flags;
};

struct sc16is7xx *
sc16is7xx_new(int spi_i2c_fd, int irq_fd, int i2c_addr, unsigned int flags)
{
	struct sc16is7xx *p;
	p = calloc(1, sizeof *p);

	p->spi_i2c_fd = spi_i2c_fd;
	p->irq_fd = irq_fd;
	p->i2c_addr = i2c_addr;
	p->flags = flags;

	return p;
}

static int
sc16is7xx_reg_rd_i2c(struct sc16is7xx *p, unsigned char regnum, unsigned char *val)
{
	struct i2c_msg msgs[2];
	struct i2c_rdwr_ioctl_data i2c_ioctl;

	bzero(&msgs, sizeof(msgs));
	bzero(&i2c_ioctl, sizeof(i2c_ioctl));

	msgs[0].addr = msgs[1].addr = p->i2c_addr;

	msgs[0].flags = 0; /* write */
	msgs[0].buf  = &regnum;
	msgs[0].len  = sizeof(regnum);

	msgs[0].flags = I2C_M_RD; /* read */
	msgs[1].buf  = val;
	msgs[1].len  = sizeof(*val);

	i2c_ioctl.msgs = msgs;
	i2c_ioctl.nmsgs = 2;

	return ioctl(p->spi_i2c_fd, I2C_RDWR, &i2c_ioctl);
}

static int
sc16is7xx_reg_wr_i2c(struct sc16is7xx *p,
                     unsigned char regnum, unsigned char val)
{
	unsigned char buf[2];
	struct i2c_msg msg;
	struct i2c_rdwr_ioctl_data i2c_ioctl;

	bzero(&msg, sizeof(msg));
	bzero(&i2c_ioctl, sizeof(i2c_ioctl));

	buf[0] = regnum;
	buf[1] = val;

	msg.addr = p->i2c_addr;
	msg.flags = 0; /* write */
	msg.buf  = buf;
	msg.len  = sizeof(buf);

	i2c_ioctl.msgs = &msg;
	i2c_ioctl.nmsgs = 1;

	return ioctl(p->spi_i2c_fd, I2C_RDWR, &i2c_ioctl);
}

static int
sc16is7xx_reg_wr(struct sc16is7xx *p,
                 unsigned char regnum, unsigned char val)
{
	if (p->i2c_addr)
		return sc16is7xx_reg_wr_i2c(p, regnum, val);
}

static int
sc16is7xx_reg_rd(struct sc16is7xx *p, unsigned char regnum, unsigned char *val)
{
	if (p->i2c_addr)
		return sc16is7xx_reg_rd_i2c(p, regnum, val);
}
