#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define DEFAULT_FOSC  14745600 /* 14.7456 MHz */

static inline unsigned char REG_NUM(int ch, int a) {
	return (a&0x0f)<<3 | (ch&0x03)<<1;
}

static inline unsigned char FLIP(unsigned char b) {
	b = ( b       << 4) | ( b       >> 4); /* 0000xxxx <<4 xxxx0000 >>4 */
	b = ((b&0x33) << 2) | ((b&0xcc) >> 2); /* 00xx00xx <<2 xx00xx00 >>2 */
	b = ((b&0x55) << 1) | ((b&0xaa) >> 1); /* 0x0x0x0x <<1 x0x0x0x0 >>1 */
	return b;
}

enum UART_REG_NAMES {
	RHR       = 0x00, /* read */
	THR       = 0x00, /* write */
	IER       = 0x01,
	FCR       = 0x02,
	IIR       = 0x02,
	LCR       = 0x03,
	MCR       = 0x04,
	LSR       = 0x05,
	MSR       = 0x06,
	SPR       = 0x07,
	TCR       = 0x06, /* MCR[2]=1 EFR[4]=1 */
	TLR       = 0x07, /* "" */
	TXLVL     = 0x08,
	RXLVL     = 0x09,
	IODIR     = 0x0A,
	IOSTATE   = 0x0B,
	IOINTENA  = 0x0C,
	IOCONTROL = 0x0E,
	EFCR      = 0x0F,
	DLL       = 0x00, /* LCR[7]=1 & LCR != 0xBF: Special reg. */
	DLH       = 0x01, /* "" */
	EFR       = 0x02, /* LCR = 0xBF : Enhanced Register */
	XON1      = 0x04, /* "" */
	XON2      = 0x05, /* "" */
	XOFF1     = 0x06, /* "" */
	XOFF2     = 0x07, /* "" */
	END_MARK  = 0xffff
};

struct sc16is7xx {
	int spi_i2c_fd;
	int i2c_addr;
	unsigned int flags;
	unsigned long fosc; /* in Hz */
	unsigned int xfers;
	unsigned int rxcounter;
	unsigned int txcounter;
};

/* register read/write combined for SPI */
static int
sc16is7xx_reg_xfer_spi(struct sc16is7xx *p,
                     unsigned char regnum, unsigned char txval, unsigned char *rxval)
{
	unsigned char txbuf[2], rxbuf[2];
	struct spi_ioc_transfer xfer;

	txbuf[0] = regnum;
	txbuf[1] = txval;
	rxbuf[0] = rxbuf[1] = '\0';

	bzero(&xfer, sizeof(xfer));
	xfer.rx_buf = (__u64)(unsigned long)rxbuf;
	xfer.tx_buf = (__u64)(unsigned long)txbuf;
	xfer.len    = sizeof(txbuf);
//	xfer.cs_change = 1; /* deselect device after xfer */

	/* one single transfer */
	if (ioctl(p->spi_i2c_fd,SPI_IOC_MESSAGE(1),&xfer) == -1) {
		perror("ioctl(SPI_IOC_MESSAGE)");
		fprintf(stderr,"spi xfer error: %02x %02x --> %02x %02x\n",
			txbuf[0], txbuf[1], rxbuf[0], rxbuf[1]);
		return -1;
	}

	p->xfers++;

	if (rxval)
		*rxval = rxbuf[1];

	return 0;
}

static int
sc16is7xx_reg_xfer_spi_fifo(
	struct sc16is7xx *p,
	unsigned char regnum,
	unsigned char *txbuf,
	unsigned char *rxbuf,
	int length)
{
	struct spi_ioc_transfer xfer[2];

	bzero(&xfer, sizeof(xfer));

	xfer[0].rx_buf = 0;
	xfer[0].tx_buf = (__u64)(unsigned long)&regnum;
	xfer[0].len    = 1;

	xfer[1].rx_buf = (__u64)(unsigned long)rxbuf;
	xfer[1].tx_buf = (__u64)(unsigned long)txbuf;;
	xfer[1].len    = length;
	xfer[1].cs_change = 1; /* deselect chip */

	/* one single transfer */
	if (ioctl(p->spi_i2c_fd,SPI_IOC_MESSAGE(2),&xfer) == -1) {
		perror("ioctl(SPI_IOC_MESSAGE)");
		return -1;
	}

	p->xfers++;
	return 0;

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

	msgs[1].flags = I2C_M_RD; /* read */
	msgs[1].buf  = val;
	msgs[1].len  = sizeof(*val);

	i2c_ioctl.msgs = msgs;
	i2c_ioctl.nmsgs = 2;

	if (ioctl(p->spi_i2c_fd, I2C_RDWR, &i2c_ioctl) != 2)
		return -1;

	p->xfers++;

	return 0;
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

	if (ioctl(p->spi_i2c_fd, I2C_RDWR, &i2c_ioctl) != 1)
		return -1;

	p->xfers++;

	return 0;
}

static int
sc16is7xx_reg_wr_raw(struct sc16is7xx *p,
                     unsigned char regnum, unsigned char val)
{
	if (p->i2c_addr == 0) {/* spi */
		regnum &= ~0x81; /* clear R/#W bit, undef bit */
		return sc16is7xx_reg_xfer_spi(p, regnum, val, NULL);
	}
	return sc16is7xx_reg_wr_i2c(p, regnum & 0x7f, val);
}

static int
sc16is7xx_reg_rd_raw(struct sc16is7xx *p,
                     unsigned char regnum, unsigned char *val)
{
	if (p->i2c_addr == 0) {
		regnum |=  0x80; /* set R/#W bit */
		regnum &= ~0x01; /* clear undef bit */
		return sc16is7xx_reg_xfer_spi(p, regnum, 0, val);
	}
	return sc16is7xx_reg_rd_i2c(p, regnum & 0x7f, val);
}

static int
sc16is7xx_reg_wr(struct sc16is7xx *p, int reg, unsigned char val)
{
	return sc16is7xx_reg_wr_raw(p, REG_NUM(0,reg), val);
}

static int
sc16is7xx_reg_rd(struct sc16is7xx *p, int reg, unsigned char *val)
{
	return sc16is7xx_reg_rd_raw(p, REG_NUM(0,reg), val);
}

int
sc16is7xx_set_baud(struct sc16is7xx *p, unsigned int baud)
{
	unsigned char tmp_lcr;
	unsigned long divisor = p->fosc / (16 * baud);

	printf("Setting divisor to 0x%lx (baud=%u, osc=%lu)",
		divisor, baud, p->fosc);

	/* enable access to divisor registers */
	if (sc16is7xx_reg_rd(p, LCR, &tmp_lcr))
		return -1;
	if (sc16is7xx_reg_wr(p, LCR, 0x80))
		return -1;

	/* write divisor */
	if (sc16is7xx_reg_wr(p, DLL, divisor & 0xff))
		return -1;
	if (sc16is7xx_reg_wr(p, DLH, (divisor >> 8) & 0xff))
		return -1;

	/* hide divisor latch registers again */
	if (sc16is7xx_reg_wr(p, LCR, tmp_lcr & ~0x80))
		return -1;
	return 0;
}

ssize_t
sc16is7xx_read(struct sc16is7xx *p, void *buf, size_t length) {
	unsigned char rxlvl, lsr;
	ssize_t ret=0;

	if (length == 0) {
		fprintf(stderr,"Length == 0???\n");
		return 0;
	}

	if (sc16is7xx_reg_rd(p, LSR, &lsr))
		return -1;

	if (!(lsr & 0x01)) { /* no data in receiver */
		return 0;
	}

	if (lsr & 0x02) {
		fprintf(stderr,"\nOverflow! Reset FIFO.\n");
		/* reset read fifo */
		if (sc16is7xx_reg_wr(p, FCR, 0x01))
			return -1;
		return 0;
	}

	if (sc16is7xx_reg_rd(p, RXLVL, &rxlvl))
		return -1;

	if (rxlvl == 0) {
		fprintf(stderr,"RXLVL == 0, even though LSR[0] set!\n");
		return 0;
	}

	if (length > rxlvl)
		length = rxlvl;
	ret = length;

	if (p->i2c_addr == 0) {/* SPI */
		if (sc16is7xx_reg_xfer_spi_fifo(p,REG_NUM(0, RHR) | 0x80,
		       NULL/*tx*/, buf/*rx*/, length) == -1)
			return -1;
		p->rxcounter += length;
		return ret;
	}

	/* I2C */
	while (length) {
		if (sc16is7xx_reg_rd(p, RHR, (unsigned char*)buf))
			return -1;
		buf++;
		length--;
	}

	p->rxcounter += ret;

	return ret;
}

ssize_t
sc16is7xx_write(struct sc16is7xx *p, const void *buf, size_t length) {
	unsigned char txlvl;
	ssize_t ret=0;

	if (sc16is7xx_reg_rd(p, TXLVL, &txlvl))
		return -1;

	if (txlvl == 0)
		return 0;

	if (length > txlvl)
		length = txlvl;
	ret = length;

	while (length) {
		if (sc16is7xx_reg_wr(p, THR, *(const unsigned char*)buf))
			return -1;
		buf++;
		length--;
	}

	p->txcounter += ret;

	return ret;
}

ssize_t
sc16is7xx_get_rxlvl(struct sc16is7xx *p) {
	unsigned char rxlvl;
	if (sc16is7xx_reg_rd(p, RXLVL, &rxlvl))
		return -1;
	return rxlvl;
}

ssize_t
sc16is7xx_get_txlvl(struct sc16is7xx *p) {
	unsigned char txlvl;
	if (sc16is7xx_reg_rd(p, TXLVL, &txlvl))
		return -1;
	return txlvl;
}

void
sc16is7xx_stats(struct sc16is7xx *p,
	unsigned int *xfers,
	unsigned int *txcounter,
	unsigned int *rxcounter
){
	if (xfers)
		*xfers = p->xfers;
	if (txcounter)
		*txcounter = p->txcounter;
	if (rxcounter)
		*rxcounter = p->rxcounter;
}

struct sc16is7xx_initdata {
	unsigned int regnum;
	unsigned char val;
};

static const struct sc16is7xx_initdata sc16is7xx_initdata[] = {
	{ LCR,      0x03 }, /* 8 N 1 */
	{ FCR,      0x07 }, /* enable Fifo */
	{ SPR,      0x42 }, /* random data to scratch register */
	{ IODIR,    0x01 },
	{ IOSTATE,  0x0c },
	{ END_MARK, 0x00 }
};

int
sc16is7xx_gpio_out(struct sc16is7xx *p,
	unsigned char iodir,
	unsigned char outval
) {
	if (sc16is7xx_reg_wr(p, IODIR, iodir))
		return -1;
	if (sc16is7xx_reg_wr(p, IOSTATE, outval))
		return -1;
	return 0;
}

int
sc16is7xx_gpio_in(struct sc16is7xx *p, unsigned char *inval) {
	if (sc16is7xx_reg_rd(p, IOSTATE, inval))
		return -1;
	return 0;
}


struct sc16is7xx *
sc16is7xx_new(int spi_i2c_fd, int i2c_addr, unsigned int flags)
{
	struct sc16is7xx *p;
	const struct sc16is7xx_initdata *idt;
	unsigned char val;
	__u32 mode32;
	int i,j;

	p = calloc(1, sizeof *p);

	p->spi_i2c_fd = spi_i2c_fd;
	p->i2c_addr = i2c_addr;
	p->flags = flags;
	p->fosc = DEFAULT_FOSC;

	if (p->i2c_addr == 0) { /* spi */
		mode32 = SPI_MODE_0;

		if(ioctl(p->spi_i2c_fd,SPI_IOC_WR_MODE32,&mode32)){
			perror("ioctl(SPI_IOC_WR_MODE)");
			goto err_out;
		}
	}


	for (idt = sc16is7xx_initdata; idt->regnum != END_MARK; idt++) {
		printf("Init: reg %d = %d (0x%02x)\n",idt->regnum,
			idt->val, idt->val);
		if (sc16is7xx_reg_wr(p, idt->regnum, idt->val) == -1) {
			perror("Cannot write init data");
			goto err_out;
		}
	}

	for (i=0; i<16; i++) {
		j = sc16is7xx_reg_rd(p,i, &val);
		if (j != 0) {
			fprintf(stderr,"reg_rd returned %d\n", j);
			perror("ioctl()");
			continue;
		}
		printf("Reg %d -> %u (0x%02x)\n", i, val, val);
	}


	if (sc16is7xx_reg_rd(p, SPR, &val)) {
		perror("sc16is7xx_read(SPR)");
		goto err_out;
	}

	if (val != 0x42) {
		fprintf(stderr,"Value in SPR register not read back!\n");
		goto err_out;
	}

	if (sc16is7xx_set_baud(p, 9600)) {
		perror("sc16is7xx_set_baud");
		goto err_out;
	}


	return p;

err_out:
	free(p);
	return NULL;
}

