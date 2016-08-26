/*
 *     comedi/drivers/daq_gert.c
 *
 *     COMEDI - Linux Control and Measurement Device Interface
 *     Copyright (C) 1998 David A. Schleef <ds@schleef.org>
 *
 *     This program is free software; you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation; either version 2 of the License, or
 *     (at your option) any later version.
 *
 *     This program is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with this program; if not, write to the Free Software
 *     Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

/*
Driver: "experimental" daq_gert in progress ... for 3.18+ kernels
 * 
 * 
 *  Added daq_gert.o to the COMEDI_MISC_DRIVERS
 *  comedi/drivers/Makefile
 *  obj-$(CONFIG_COMEDI_DAQ_GERT)           += daq_gert.o
 *  Added ARM to the comedi Kconfig file
 *  Added daq_gert to the comedi Kconfig file
 * 

 config COMEDI
   tristate "Data acquisition support (comedi)"
   depends on m
   depends on BROKEN || FRV || M32R || MN10300 || SUPERH || TILE || X86 || ARM
        ---help---
          Enable support a wide range of data acquisition devices
          for Linux.
 
* section COMEDI_MISC_DRIVERS

config COMEDI_DAQ_GERT
        tristate "GERTBOARD support"
        depends on COMEDI_KCOMEDILIB
        ---help---
       Enable support for a raspi gertboard

       To compile this driver as a module, choose M here: the module will be
       called daq_gert.

 * 
 * 
Description: GERTBOARD daq-gert
Author: Fred Brooks <nsaspook@nsaspook.com>
 * 
Most of the actual GPIO setup code was copied from
 * 
WiringPI 

 ***********************************************************************
 * This file is part of wiringPi:
 *      https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 * 
 Also 
 * 
 * Driver for Broadcom BCM2708 SPI Controllers
 *
 * Copyright (C) 2012 Chris Boot
 *
 * This driver is inspired by:
 * spi-ath79.c, Copyright (C) 2009-2011 Gabor Juhos <juhosg@openwrt.org>
 * spi-atmel.c, Copyright (C) 2006 Atmel Corporation

Devices: [] GERTBOARD (daq_gert)
Status: inprogress (DIO 95%) (AI 80%) AO (80%) (My code cleanup 55%)
Updated: Fri, 25 Feb 2015 12:07:20 +0000

The DAQ-GERT appears in Comedi as a  digital I/O subdevice (0) with
17 or 21 channels, a analog input subdevice (1) with 2 single-ended channels,
a analog output subdevice(2) with 2 channels

Digital:  The comedi channel 0 corresponds to the GPIO WPi table order
channel numbers [0..7] will be outputs, [8..16/20] will be inputs
 * 0/2
 * 1/3
 * 4
 * 7    SPI CE1
 * 8    SPI CE0
 * 9    SPI SO
 * 10   SPI SI
 * 11   SPI CLK
 * 14   UART
 * 15   UART
 * 17
 * 18   PWM
 * 21/27
 * 22
 * 23
 * 24
 * 25
 * 
 The BCM2835 has 54 GPIO pins.
      BCM2835 data sheet, Page 90 onwards.
      There are 6 control registers, each control the functions of a block
      of 10 pins.
      Each control register has 10 sets of 3 bits per GPIO pin:

      000 = GPIO Pin X is an input
      001 = GPIO Pin X is an output
      100 = GPIO Pin X takes alternate function 0
      101 = GPIO Pin X takes alternate function 1
      110 = GPIO Pin X takes alternate function 2
      111 = GPIO Pin X takes alternate function 3
      011 = GPIO Pin X takes alternate function 4
      010 = GPIO Pin X takes alternate function 5

 So the 3 bits for port X are:
      X / 10 + ((X % 10) * 3)

Digital direction configuration: [0..1] are input only due to pullups,
 * all other ports can be input or outputs

Analog: The input  range is 0 to 1023 for 0.0 to 3.3 or 2.048 volts 
IRQ is assigned but not used.

 *  PIC Slave Info:
 * 
 * bit 7 high for commands sent from the MASTER
 * bit 6 0 send lower or 1 send upper byte ADC result first
 * bits 3..0 port address
 *
 * bit 7 low  for config data sent in CMD_DUMMY per uC type
 * bits 6 config bit code always 1
 * bit	5 0=ADC ref VDD, 1=ADC rec FVR=2.048
 * bit  4 0=10bit adc, 1=12bit adc
 * bits 3..0 number of ADC channels
 * 
 */

#include "../comedidev.h"
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/log2.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/mutex.h>

/* Function stubs */
void (*pinMode) (int pin, int mode);
void (*pullUpDnControl) (int pin, int pud);
void (*digitalWrite) (int pin, int value);
void (*setPadDrive) (int group, int value);
int (*digitalRead) (int pin);

static struct spi_device *comedi_spi_ai, *comedi_spi_ao;
static struct bcm2708_spi *comedi_bs;

struct comedi_control {
    struct spi_message msg;
    struct spi_transfer transfer;
    u8 *tx_buff;
    u8 *rx_buff;
};
static struct comedi_control comedi_ctl;

struct spi_adc_type {
    uint16_t range : 1;
    uint16_t bits : 1;
    uint16_t link : 1;
    uint16_t pic18 : 2;
    uint16_t chan : 4;
};
struct spi_adc_type spi_adc, spi_dac;

struct pic_platform_data {
    uint16_t conv_delay_usecs;
};

static struct pic_platform_data pic_info_pic18 = {
    .conv_delay_usecs = 35
};

#define CSnA    0       /* GPIO 8  Gertboard ADC */
#define CSnB    1       /* GPIO 7  Gertboard DAC */

#define SPI_BUFF_SIZE 16

/* PIC Slave commands */
#define CMD_ADC_GO	0b10000000      // send data low byte first
#define CMD_ADC_GO_H	0b11000000      // send data high byte first
#define CMD_ADC_DATA	0b10010000
#define CMD_ADC_DIAG	0b11110000
#define CMD_DUMMY_CFG	0b01000000

/* SPI register offsets */
#define SPI_CS			0x00
#define SPI_FIFO		0x04
#define SPI_CLK			0x08
#define SPI_DLEN		0x0c
#define SPI_LTOH		0x10
#define SPI_DC			0x14

/* Bitfields in CS */
#define SPI_CS_LEN_LONG		0x02000000
#define SPI_CS_DMA_LEN		0x01000000
#define SPI_CS_CSPOL2		0x00800000
#define SPI_CS_CSPOL1		0x00400000
#define SPI_CS_CSPOL0		0x00200000
#define SPI_CS_RXF		0x00100000
#define SPI_CS_RXR		0x00080000
#define SPI_CS_TXD		0x00040000
#define SPI_CS_RXD		0x00020000
#define SPI_CS_DONE		0x00010000
#define SPI_CS_LEN		0x00002000
#define SPI_CS_REN		0x00001000
#define SPI_CS_ADCS		0x00000800
#define SPI_CS_INTR		0x00000400
#define SPI_CS_INTD		0x00000200
#define SPI_CS_DMAEN		0x00000100
#define SPI_CS_TA		0x00000080
#define SPI_CS_CSPOL		0x00000040
#define SPI_CS_CLEAR_RX		0x00000020
#define SPI_CS_CLEAR_TX		0x00000010
#define SPI_CS_CPOL		0x00000008
#define SPI_CS_CPHA		0x00000004
#define SPI_CS_CS_10		0x00000002
#define SPI_CS_CS_01		0x00000001

#define SPI_TIMEOUT_MS	150

#define DRV_NAME_SPI	"bcm2708_spi"

struct bcm2708_spi {
    spinlock_t lock;
    void __iomem *base;
    int irq;
    struct clk *clk;
    bool stopping;

    struct list_head queue;
    struct workqueue_struct *workq;
    struct work_struct work;
    struct completion done;

    const u8 *tx_buf;
    u8 *rx_buf;
    int len;
};

struct bcm2708_spi_state {
    u32 cs;
    u16 cdiv;
};

static inline u32 bcm2708_rd(struct bcm2708_spi *bs, unsigned reg) {
    return readl(bs->base + reg);
}

static inline void bcm2708_wr(struct bcm2708_spi *bs, unsigned reg, u32 val) {
    writel(val, bs->base + reg);
}

static inline void bcm2708_rd_fifo(struct bcm2708_spi *bs, int len) {
    u8 byte;

    while (len--) {
        byte = bcm2708_rd(bs, SPI_FIFO);
        if (bs->rx_buf)
            *bs->rx_buf++ = byte;
    }
}

static inline void bcm2708_wr_fifo(struct bcm2708_spi *bs, int len) {
    u8 byte;

    if (len > bs->len)
        len = bs->len;

    while (len--) {
        byte = bs->tx_buf ? *bs->tx_buf++ : 0;
        bcm2708_wr(bs, SPI_FIFO, byte);
        bs->len--;
    }
}

static irqreturn_t bcm2708_spi_interrupt(int irq, void *dev_id) {
    struct spi_master *master = dev_id;
    struct bcm2708_spi *bs = spi_master_get_devdata(master);
    u32 cs;

    spin_lock(&bs->lock);

    cs = bcm2708_rd(bs, SPI_CS);

    if (cs & SPI_CS_DONE) {
        if (bs->len) { /* first interrupt in a transfer */
            /* fill the TX fifo with up to 16 bytes */
            bcm2708_wr_fifo(bs, 16);
        } else { /* transfer complete */
            /* disable interrupts */
            cs &= ~(SPI_CS_INTR | SPI_CS_INTD);
            bcm2708_wr(bs, SPI_CS, cs);

            /* drain RX FIFO */
            while (cs & SPI_CS_RXD) {
                bcm2708_rd_fifo(bs, 1);
                cs = bcm2708_rd(bs, SPI_CS);
            }

            /* wake up our bh */
            complete(&bs->done);
        }
    } else if (cs & SPI_CS_RXR) {
        /* read 12 bytes of data */
        bcm2708_rd_fifo(bs, 12);

        /* write up to 12 bytes */
        bcm2708_wr_fifo(bs, 12);
    }

    spin_unlock(&bs->lock);

    return IRQ_HANDLED;
}

static int bcm2708_setup_state(struct spi_master *master,
        struct device *dev, struct bcm2708_spi_state *state,
        u32 hz, u8 csel, u8 mode, u8 bpw) {
    struct bcm2708_spi *bs = spi_master_get_devdata(master);
    int cdiv;
    unsigned long bus_hz;
    u32 cs = 0;

    bus_hz = clk_get_rate(bs->clk);

    if (hz >= bus_hz) {
        cdiv = 2; /* bus_hz / 2 is as fast as we can go */
    } else if (hz) {
        cdiv = DIV_ROUND_UP(bus_hz, hz);

        /* CDIV must be a power of 2, so round up */
        cdiv = roundup_pow_of_two(cdiv);

        if (cdiv > 65536) {
            dev_dbg(dev,
                    "setup: %d Hz too slow, cdiv %u; min %ld Hz\n",
                    hz, cdiv, bus_hz / 65536);
            return -EINVAL;
        } else if (cdiv == 65536) {
            cdiv = 0;
        } else if (cdiv == 1) {
            cdiv = 2; /* 1 gets rounded down to 0; == 65536 */
        }
    } else {
        cdiv = 0;
    }

    switch (bpw) {
        case 8:
            break;
        default:
            dev_dbg(dev, "setup: invalid bits_per_word %u (must be 8)\n",
                    bpw);
            return -EINVAL;
    }

    if (mode & SPI_CPOL)
        cs |= SPI_CS_CPOL;
    if (mode & SPI_CPHA)
        cs |= SPI_CS_CPHA;

    if (!(mode & SPI_NO_CS)) {
        if (mode & SPI_CS_HIGH) {
            cs |= SPI_CS_CSPOL;
            cs |= SPI_CS_CSPOL0 << csel;
        }

        cs |= csel;
    } else {
        cs |= SPI_CS_CS_10 | SPI_CS_CS_01;
    }

    if (state) {
        state->cs = cs;
        state->cdiv = cdiv;
    }

    return 0;
}

static int bcm2708_process_transfer(struct bcm2708_spi *bs,
        struct spi_message *msg, struct spi_transfer *xfer) {
    struct spi_device *spi = msg->spi;
    struct bcm2708_spi_state state, *stp;
    int ret;
    u32 cs;

    if (bs->stopping)
        return -ESHUTDOWN;

    if (xfer->bits_per_word || xfer->speed_hz) {
        ret = bcm2708_setup_state(spi->master, &spi->dev, &state,
                spi->max_speed_hz, spi->chip_select, spi->mode,
                spi->bits_per_word);
        if (ret)
            return ret;

        stp = &state;
    } else {
        stp = spi->controller_state;
    }

    reinit_completion(&bs->done);
    bs->tx_buf = xfer->tx_buf;
    bs->rx_buf = xfer->rx_buf;
    bs->len = xfer->len;

    cs = stp->cs | SPI_CS_INTR | SPI_CS_INTD | SPI_CS_TA;

    bcm2708_wr(bs, SPI_CLK, stp->cdiv);
    bcm2708_wr(bs, SPI_CS, cs);

    ret = wait_for_completion_timeout(&bs->done,
            msecs_to_jiffies(SPI_TIMEOUT_MS));
    if (ret == 0) {
        dev_err(&spi->dev, "transfer timed out\n");
        return -ETIMEDOUT;
    }

    if (xfer->delay_usecs)
        udelay(xfer->delay_usecs);

    if (list_is_last(&xfer->transfer_list, &msg->transfers) ||
            xfer->cs_change) {
        /* clear TA and interrupt flags */
        bcm2708_wr(bs, SPI_CS, stp->cs);
    }

    msg->actual_length += (xfer->len - bs->len);
    return 0;
}

static void bcm2708_work(struct work_struct *work) {
    struct bcm2708_spi *bs = container_of(work, struct bcm2708_spi, work);
    unsigned long flags;
    struct spi_message *msg;
    struct spi_transfer *xfer;
    int status = 0;

    spin_lock_irqsave(&bs->lock, flags);
    while (!list_empty(&bs->queue)) {
        msg = list_first_entry(&bs->queue, struct spi_message, queue);
        list_del_init(&msg->queue);
        spin_unlock_irqrestore(&bs->lock, flags);

        list_for_each_entry(xfer, &msg->transfers, transfer_list) {
            status = bcm2708_process_transfer(bs, msg, xfer);
            if (status)
                break;
        }

        msg->status = status;
        msg->complete(msg->context);

        spin_lock_irqsave(&bs->lock, flags);
    }
    spin_unlock_irqrestore(&bs->lock, flags);
}

static int bcm2708_spi_setup(struct spi_device *spi) {
    struct bcm2708_spi *bs = spi_master_get_devdata(spi->master);
    struct bcm2708_spi_state *state;
    int ret;

    /* Create SPI channels for Comedi*/
    if (spi->chip_select == CSnA) { /* we need a device to talk to the ADC */
        spi->mode = SPI_CS_CS_10 | SPI_CS_CS_01; /* mode 3 */
        comedi_spi_ai = spi; /* get a copy of the slave device */
        dev_dbg(&spi->dev,
                "setup: cd %d: %d Hz, bpw %u, mode 0x%x\n",
                spi->chip_select, spi->max_speed_hz, spi->bits_per_word,
                spi->mode);
    }
    if (spi->chip_select == CSnB) { /* we need a device to talk to the DAC */
        spi->mode = SPI_CS_CS_10 | SPI_CS_CS_01; /* mode 3 */
        comedi_spi_ao = spi; /* get a copy of the slave device */
        dev_dbg(&spi->dev,
                "setup: cd %d: %d Hz, bpw %u, mode 0x%x\n",
                spi->chip_select, spi->max_speed_hz, spi->bits_per_word,
                spi->mode);
    }

    if (bs->stopping)
        return -ESHUTDOWN;

    if (!(spi->mode & SPI_NO_CS) &&
            (spi->chip_select > spi->master->num_chipselect)) {
        dev_dbg(&spi->dev,
                "setup: invalid chipselect %u (%u defined)\n",
                spi->chip_select, spi->master->num_chipselect);
        return -EINVAL;
    }

    state = spi->controller_state;
    if (!state) {
        state = kzalloc(sizeof (*state), GFP_KERNEL);
        if (!state)
            return -ENOMEM;

        spi->controller_state = state;
    }

    ret = bcm2708_setup_state(spi->master, &spi->dev, state,
            spi->max_speed_hz, spi->chip_select, spi->mode,
            spi->bits_per_word);
    if (ret < 0) {
        kfree(state);
        spi->controller_state = NULL;
    }

    dev_dbg(&spi->dev,
            "setup: cd %d: %d Hz, bpw %u, mode 0x%x -> CS=%08x CDIV=%04x\n",
            spi->chip_select, spi->max_speed_hz, spi->bits_per_word,
            spi->mode, state->cs, state->cdiv);

    return 0;
}

static int bcm2708_spi_transfer(struct spi_device *spi, struct spi_message *msg) {
    struct bcm2708_spi *bs = spi_master_get_devdata(spi->master);
    struct spi_transfer *xfer;
    int ret;
    unsigned long flags;

    if (unlikely(list_empty(&msg->transfers)))
        return -EINVAL;

    if (bs->stopping)
        return -ESHUTDOWN;

    list_for_each_entry(xfer, &msg->transfers, transfer_list) {
        if (!(xfer->tx_buf || xfer->rx_buf) && xfer->len) {
            dev_dbg(&spi->dev, "missing rx or tx buf\n");
            return -EINVAL;
        }

        if (!xfer->bits_per_word || xfer->speed_hz)
            continue;

        ret = bcm2708_setup_state(spi->master, &spi->dev, NULL,
                xfer->speed_hz ? xfer->speed_hz : spi->max_speed_hz,
                spi->chip_select, spi->mode,
                xfer->bits_per_word ? xfer->bits_per_word :
                spi->bits_per_word);
        if (ret)
            return ret;
    }

    msg->status = -EINPROGRESS;
    msg->actual_length = 0;

    spin_lock_irqsave(&bs->lock, flags);
    list_add_tail(&msg->queue, &bs->queue);
    queue_work(bs->workq, &bs->work);
    spin_unlock_irqrestore(&bs->lock, flags);

    return 0;
}

static void bcm2708_spi_cleanup(struct spi_device *spi) {
    if (spi->controller_state) {
        kfree(spi->controller_state);
        spi->controller_state = NULL;
    }
}

static int bcm2708_spi_probe(struct platform_device *pdev) {
    struct resource *regs;
    int irq, err = -ENOMEM;
    struct clk *clk;
    struct spi_master *master;
    struct bcm2708_spi *bs;

    regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!regs) {
        dev_err(&pdev->dev, "could not get IO memory\n");
        return -ENXIO;
    }

    irq = platform_get_irq(pdev, 0);
    if (irq < 0) {
        dev_err(&pdev->dev, "could not get IRQ\n");
        return irq;
    }

    clk = clk_get(&pdev->dev, NULL);
    if (IS_ERR(clk)) {
        dev_err(&pdev->dev, "could not find clk: %ld\n", PTR_ERR(clk));
        return PTR_ERR(clk);
    }


    master = spi_alloc_master(&pdev->dev, sizeof (*bs));
    if (!master) {
        dev_err(&pdev->dev, "spi_alloc_master() failed\n");
        goto out_clk_put;
    }

    /* the spi->mode bits understood by this driver: */
    master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_NO_CS;

    master->bus_num = pdev->id;
    master->num_chipselect = 3;
    master->setup = bcm2708_spi_setup;
    master->transfer = bcm2708_spi_transfer;
    master->cleanup = bcm2708_spi_cleanup;
    platform_set_drvdata(pdev, master);

    bs = spi_master_get_devdata(master);

    spin_lock_init(&bs->lock);
    INIT_LIST_HEAD(&bs->queue);
    init_completion(&bs->done);
    INIT_WORK(&bs->work, bcm2708_work);

    bs->base = ioremap(regs->start, resource_size(regs));
    if (!bs->base) {
        dev_err(&pdev->dev, "could not remap memory\n");
        goto out_master_put;
    }

    bs->workq = create_singlethread_workqueue(dev_name(&pdev->dev));
    if (!bs->workq) {
        dev_err(&pdev->dev, "could not create workqueue\n");
        goto out_iounmap;
    }

    bs->irq = irq;
    bs->clk = clk;
    bs->stopping = false;

    err = request_irq(irq, bcm2708_spi_interrupt, 0, dev_name(&pdev->dev),
            master);
    if (err) {
        dev_err(&pdev->dev, "could not request IRQ: %d\n", err);
        goto out_workqueue;
    }

    /* initialise the hardware */
    clk_enable(clk);
    bcm2708_wr(bs, SPI_CS, SPI_CS_REN | SPI_CS_CLEAR_RX | SPI_CS_CLEAR_TX);

    err = spi_register_master(master);
    if (err) {
        dev_err(&pdev->dev, "could not register SPI master: %d\n", err);
        goto out_free_irq;
    }

    /* make comedi copies */
    comedi_bs = bs;

    dev_info(&pdev->dev, "SPI Controller at 0x%08lx (irq %d)\n",
            (unsigned long) regs->start, irq);
    spi_adc.link = 1;
    return 0;

out_free_irq:
    free_irq(bs->irq, master);
out_workqueue:
    destroy_workqueue(bs->workq);
out_iounmap:
    iounmap(bs->base);
out_master_put:
    spi_master_put(master);
out_clk_put:
    clk_put(clk);
    return err;
}

static int bcm2708_spi_remove(struct platform_device *pdev) {
    struct spi_master *master = platform_get_drvdata(pdev);
    struct bcm2708_spi *bs = spi_master_get_devdata(master);

    spi_adc.link = 0;
    /* reset the hardware and block queue progress */
    spin_lock_irq(&bs->lock);
    bs->stopping = true;
    bcm2708_wr(bs, SPI_CS, SPI_CS_CLEAR_RX | SPI_CS_CLEAR_TX);
    spin_unlock_irq(&bs->lock);

    flush_work(&bs->work);

    clk_disable(bs->clk);
    clk_put(bs->clk);
    free_irq(bs->irq, master);
    iounmap(bs->base);

    spi_unregister_master(master);

    return 0;
}

static struct platform_driver bcm2708_spi_driver = {
    .driver =
    {
        .name = DRV_NAME_SPI,
        .owner = THIS_MODULE,
    },
    .probe = bcm2708_spi_probe,
    .remove = bcm2708_spi_remove,
};

/*

MODULE_DESCRIPTION("SPI controller driver for Broadcom BCM2708");
MODULE_AUTHOR("Chris Boot <bootc@bootc.net>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
 */


/* Handy defines */
#ifndef SZ_16K
#define SZ_16K                          0x00004000
#endif

#define NUM_PINS        17

#define WPI_MODE_PINS            0
#define WPI_MODE_GPIO            1
#define WPI_MODE_GPIO_SYS        2
#define WPI_MODE_PIFACE          3

#define INPUT            0
#define OUTPUT           1
#define PWM_OUTPUT       2

#define LOW              0
#define HIGH             1

#define PUD_OFF          0
#define PUD_DOWN         1
#define PUD_UP           2

#ifndef TRUE
#define TRUE    (1==1)
#define FALSE   (1==2)
#endif

/* BCM Magic */
#define BCM_PASSWORD            0x5A000000

// Port function select bits
#define FSEL_INPT               0b000
#define FSEL_OUTP               0b001
#define FSEL_ALT0               0b100
#define FSEL_ALT0               0b100
#define FSEL_ALT1               0b101
#define FSEL_ALT2               0b110
#define FSEL_ALT3               0b111
#define FSEL_ALT4               0b011
#define FSEL_ALT5               0b010

/*
 Access from ARM Running Linux
      Take from Gert/Doms code. Some of this is not in the manual
      that I can find )-:
 */
#define BCM2708_PERI_BASE       0x20000000
#define GPIO_PADS               (BCM2708_PERI_BASE + 0x100000)
#define CLOCK_BASE              (BCM2708_PERI_BASE + 0x101000)
#define GPIO_BASE               (BCM2708_PERI_BASE + 0x200000)
#define GPIO_TIMER              (BCM2708_PERI_BASE + 0x00B000)
#define GPIO_PWM                (BCM2708_PERI_BASE + 0x20C000)

/* 
#define PAGE_SIZE               (4*1024)
#define BLOCK_SIZE              (4*1024)
 */

/* driver hardware numbers */
#define NUM_DIO_CHAN  17
#define NUM_DIO_CHAN_REV2       21
#define NUM_DIO_OUTPUTS 8
#define DIO_PINS_DEFAULT        0xff
/* for for compat with ni_daq_700 used for driver testing, 2 AI channels */
/* on the real device */

#define NUM_AI_CHAN_EXTENDED 12
#define NUM_AI_CHAN 2
#define NUM_AO_CHAN 2

/* Locals to hold pointers to the hardware */

static volatile uint32_t *gpio;
static int num_ai_chan, num_ao_chan, num_dio_chan = NUM_DIO_CHAN;

/* Global for the RPi board rev */
extern unsigned int system_rev; // from the kernel symbol table exports */
extern unsigned int system_serial_low;
extern unsigned int system_serial_high;

static unsigned int RPisys_rev;
/* The SPI code has found the IO chips or not  */
static int gert_detected = FALSE;
/* default to TRUE in detection code while testing */

static const struct comedi_lrange daqgert_ai_range3_300 = {1,
    {
        RANGE(0, 3.300),
    }};
static const struct comedi_lrange daqgert_ai_range2_048 = {1,
    {
        RANGE(0, 2.048),
    }};

static const struct comedi_lrange daqgert_ao_range = {1,
    {
        RANGE(0, 2.048),
    }};

/*
 Doing it the Arduino way with lookup tables...
      Yes, it's probably more innefficient than all the bit-twidling, but it
      does tend to make it all a bit clearer. At least to me!

 pinToGpio:
      Take a Wiring pin (0 through X) and re-map it to the BCM_GPIO pin
      Cope for 2 different board revisions here
 */
static const int *pinToGpio;

/* From the Original Wiki - GPIO 0 through 7 */
static const int pinToGpioR1 [64] = {
    17, 18, 21, 22, 23, 24, 25, 4,
    0, 1, /* I2C  - SDA0, SCL0            wpi  8 -  9 */
    8, 7, /* SPI  - CE1, CE0              wpi 10 - 11 */
    10, 9, 11, /* SPI  - MOSI, MISO, SCLK      wpi 12 - 14 */
    14, 15, /* UART - Tx, Rx                wpi 15 - 16 */

    /* Padding: */

    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, /* ... 31 */
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, /* . 47 */
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, /* ..63 */
};

/* From the Original Wiki - GPIO 0 through 7:   wpi  0 -  7 */
static const int pinToGpioR2 [64] = {
    17, 18, 27, 22, 23, 24, 25, 4,
    2, 3, /* I2C  - SDA0, SCL0            wpi  8 -  9 */
    8, 7, /* SPI  - CE1, CE0              wpi 10 - 11 */
    10, 9, 11, /* SPI  - MOSI, MISO, SCLK      wpi 12 - 14 */
    14, 15, /* UART - Tx, Rx                wpi 15 - 16 */
    28, 29, 30, 31, /* New GPIOs 8 though 11        wpi 17 - 20 */

    /* Padding: */

    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, /* ... 31 */
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, /* . 47 */
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, /* . 63 */
};

/*
 gpioToGPFSEL:
      Map a BCM_GPIO pin to it's control port. (GPFSEL 0-5)
 */
static const uint8_t gpioToGPFSEL [] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
    3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
};

/*
 gpioToShift
      Define the shift up for the 3 bits per pin in each GPFSEL port
 */
static const uint8_t gpioToShift [] = {
    0, 3, 6, 9, 12, 15, 18, 21, 24, 27,
    0, 3, 6, 9, 12, 15, 18, 21, 24, 27,
    0, 3, 6, 9, 12, 15, 18, 21, 24, 27,
    0, 3, 6, 9, 12, 15, 18, 21, 24, 27,
    0, 3, 6, 9, 12, 15, 18, 21, 24, 27,
};

/*
 gpioToGPSET:
      (Word) offset to the GPIO Set registers for each GPIO pin
 */
static const uint8_t gpioToGPSET [] = {
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8,
};

/*
 gpioToGPCLR:
      (Word) offset to the GPIO Clear registers for each GPIO pin
 */
static const uint8_t gpioToGPCLR [] = {
    10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
    10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
    11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
    11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
};

/*
 gpioToGPLEV:
      (Word) offset to the GPIO Input level registers for each GPIO pin
 */
static const uint8_t gpioToGPLEV [] = {
    13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
    13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
    14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
    14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
};

/*
 * pinMode:
 *      Sets the mode of a pin to be input, output
 ************************************************************
 */

void pinModeGpio(int pin, int mode) {
    int fSel, shift;

    pin &= 63;

    fSel = gpioToGPFSEL [pin];
    shift = gpioToShift [pin];

    /**/ if (mode == INPUT) /* Sets bits to zero = input */
        *(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift));
    else if (mode == OUTPUT)
        *(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift)) | (1 << shift);
}

void pinModeWPi(int pin, int mode) {
    pinModeGpio(pinToGpio [pin & 63], mode);
}

/*
 * digitalWrite:
 *      Set an output bit
 *****************************************************************
 */

void digitalWriteWPi(int pin, int value) {
    pin = pinToGpio [pin & 63];

    if (value == LOW)
        *(gpio + gpioToGPCLR [pin]) = 1 << (pin & 31);
    else
        *(gpio + gpioToGPSET [pin]) = 1 << (pin & 31);
}

void digitalWriteGpio(int pin, int value) {
    pin &= 63;

    if (value == LOW)
        *(gpio + gpioToGPCLR [pin]) = 1 << (pin & 31);
    else
        *(gpio + gpioToGPSET [pin]) = 1 << (pin & 31);
}

/*
 * digitalRead:
 *      Read the value of a given Pin, returning HIGH or LOW
 *******************************************************************
 */

int digitalReadWPi(int pin) {
    pin = pinToGpio [pin & 63];

    if ((*(gpio + gpioToGPLEV [pin]) & (1 << (pin & 31))) != 0)
        return HIGH;
    else
        return LOW;
}

int digitalReadGpio(int pin) {
    pin &= 63;

    if ((*(gpio + gpioToGPLEV [pin]) & (1 << (pin & 31))) != 0)
        return HIGH;
    else
        return LOW;
}

/*
 * piBoardRev:
 *	Return a number representing the hardware revision of the board.
 *	Revision is currently 1 or 2. -1 is returned on error.
 *********************************************************************
 */

int piBoardRev(struct comedi_device *dev) {
    int r = -1;
    static int boardRev = -1;

    if (boardRev != -1)
        return boardRev;

    dev_info(dev->class_dev, "RPi Board Rev %u, Serial %08x%08x\n",
            RPisys_rev, system_serial_high, system_serial_low);

    r = RPisys_rev;

    if (r == -1) {
        return -1;
    }

    if ((r == 2) || (r == 3))
        boardRev = 1;
    else if ((r == 4) || (r == 5) || (r == 6))
        boardRev = 2;
    else {
        return -1;
    }

    return boardRev;
}

/*
 * wiringPiSetup:
 *	Must be called once at the start of your program execution.
 *
 * Default setup: Initialises the system into wiringPi Pin mode and uses the
 *	memory mapped hardware directly.
 ************************************************************************
 */

int wiringPiSetup(struct comedi_device *dev) {
    int boardRev;

    pinMode = pinModeWPi;
    digitalWrite = digitalWriteWPi;
    digitalRead = digitalReadWPi;

    if ((boardRev = piBoardRev(dev)) < 0)
        return -1;

    if (boardRev == 1)
        pinToGpio = pinToGpioR1;
    else
        pinToGpio = pinToGpioR2;
    return 0;
}

/*
 * wiringPiSetupGpio:
 *	Must be called once at the start of your program execution.
 *
 * GPIO setup: Initialises the system into GPIO Pin mode and uses the
 *	memory mapped hardware directly.
 *************************************************************************
 */

int wiringPiSetupGpio(struct comedi_device *dev) {
    int x;

    if ((x = wiringPiSetup(dev)) < 0)
        return x;

    pinMode = pinModeGpio;
    digitalWrite = digitalWriteGpio;
    digitalRead = digitalReadGpio;

    return 0;
}

struct daqgert_board {
    const char *name;
};

/* FIXME Slow brute forced IO bits, 5us reads from userland */

/* need to use (fix) state to optimize changes */
static int daqgert_dio_insn_bits(struct comedi_device *dev,
        struct comedi_subdevice *s,
        struct comedi_insn *insn, unsigned int *data) {
    int pinWPi;

    if (data[0]) { /* write data to pin */
        s->state &= ~data[0];
        s->state |= (data[0] & data[1]);
        /* s->state contains the GPIO bits */
        /* s->io_bits contains the GPIO direction */

        /* OUT testing with gpio pins  */
        /* We need to shift a single bit from state to set or clear the GPIO */
        for (pinWPi = 0; pinWPi < num_dio_chan; pinWPi++) {
            if (!(gert_detected && ((pinWPi >= 10) && (pinWPi <= 14)))) {
                /* Do nothing on SPI AUX pins when detected */
                digitalWriteWPi(pinWPi,
                        (s->state & (0x01 << pinWPi)) >> pinWPi);
            }
        }
    }

    data[1] = s->state & 0xffffff;
    /* IN testing with gpio pins */
    /* Rev #1 num_dio_chan 17 ,Rev #2 num_dio_pins 21 */
    for (pinWPi = 0; pinWPi < num_dio_chan; pinWPi++) {
        if (!(gert_detected && ((pinWPi >= 10) && (pinWPi <= 14)))) {
            data[1] |= digitalReadWPi(pinWPi) << pinWPi; /* shift */
        }
    }
    return insn->n;
}

/* query or change DIO config */
static int daqgert_dio_insn_config(struct comedi_device *dev,
        struct comedi_subdevice *s,
        struct comedi_insn *insn, unsigned int *data) {
    unsigned int wpi_pin = CR_CHAN(insn->chanspec), chan = 1 << wpi_pin;

    switch (data[0]) {
        case INSN_CONFIG_DIO_OUTPUT:
            if (!(gert_detected && ((wpi_pin >= 10) && (wpi_pin <= 14)))) {
                s->io_bits |= chan;
                pinModeWPi(wpi_pin, OUTPUT);
            }
            break;
        case INSN_CONFIG_DIO_INPUT:
            if (!(gert_detected && ((wpi_pin >= 10) && (wpi_pin <= 14)))) {
                s->io_bits &= (~chan);
                pinModeWPi(wpi_pin, INPUT);
            }
            break;
        case INSN_CONFIG_DIO_QUERY:
            data[1] = (s->io_bits & chan) ? COMEDI_OUTPUT : COMEDI_INPUT;
            return insn->n;
        default:
            return -EINVAL;
    }
    dev_dbg(dev->class_dev, "%s: GPIO wpi-pins setting 0x%x\n",
            dev->board_name,
            (unsigned int) s->io_bits);
    return 1;

}

/* Create a message to send to the SPI driver */
static void comedi_spi_msg(unsigned char data, unsigned char cs_select, unsigned char msg_len) {

    if (msg_len > SPI_BUFF_SIZE) msg_len = SPI_BUFF_SIZE;
    spi_message_init(&comedi_ctl.msg);
    comedi_ctl.msg.spi = comedi_spi_ai;
    if (cs_select == CSnB) comedi_ctl.msg.spi = comedi_spi_ao;
    comedi_ctl.tx_buff[0] = data; /* we only set 1 byte but can send many */
    comedi_ctl.transfer.len = msg_len;
    comedi_ctl.transfer.tx_buf = comedi_ctl.tx_buff;
    comedi_ctl.transfer.rx_buf = comedi_ctl.rx_buff;
    spi_message_add_tail(&comedi_ctl.transfer, &comedi_ctl.msg);
}

/* Have the SPI driver execute our message to the selected slave */
static int comedi_do_one_message(unsigned char msgdata, unsigned char cs_select, unsigned char msg_len) {
    int status;

    if (!spi_adc.link) return -ESHUTDOWN;
    comedi_spi_msg(msgdata, cs_select, msg_len);
    status = bcm2708_process_transfer(comedi_bs, &comedi_ctl.msg, &comedi_ctl.transfer);
    return status;
}

/* Talk to the ADC via the SPI */
static int daqgert_ai_rinsn(struct comedi_device *dev,
        struct comedi_subdevice *s,
        struct comedi_insn *insn, unsigned int *data) {

    int n, chan;
    struct pic_platform_data *pic_data = s->private;

    chan = CR_CHAN(insn->chanspec);
    /* convert n samples */
    for (n = 0; n < insn->n; n++) {
        /* Make SPI messages for the type of ADC are we talking to */
        /* The PIC Slave needs 8 bit transfers only */
        if (spi_adc.pic18 > 1) { /*  PIC18 SPI slave device */
            comedi_do_one_message(CMD_ADC_GO_H + chan, CSnA, 1);
            udelay(pic_data->conv_delay_usecs); /* ADC conversion delay */
            comedi_do_one_message(CMD_ADC_DATA, CSnA, 1);
            data[n] = comedi_ctl.rx_buff[0] << 8;
            comedi_do_one_message(CMD_DUMMY_CFG, CSnA, 1);
            data[n] += comedi_ctl.rx_buff[0];
        } else { /* Gertboard device */
            comedi_do_one_message((0b01100000 | ((chan & 0x01) << 4)), CSnA, 2); /* set ADC channel SE, send two bytes */
            data[n] = (comedi_ctl.rx_buff[0]&0b00000011) << 8; /* two bytes were received from the FIFO */
            data[n] += comedi_ctl.rx_buff[1];
        }
    }
    return n;
}

static int daqgert_ao_winsn(struct comedi_device *dev,
        struct comedi_subdevice *s,
        struct comedi_insn *insn, unsigned int *data) {
    unsigned int n, junk;
    unsigned int chan;

    chan = CR_CHAN(insn->chanspec);
    for (n = 0; n < insn->n; n++) {
        junk = data[n]&0xfff; /* strip to 12 bits */
        comedi_ctl.tx_buff[1] = junk & 0xff; /* load lsb SPI data into transfer buffer */
        udelay(15); /*  delay */
        comedi_do_one_message((0b00110000 | ((chan & 0x01) << 7) | (junk >> 8)), CSnB, 2); /* Load DAC channel, send two bytes */
    }
    return n;
}

static int daqgert_ao_rinsn(struct comedi_device *dev,
        struct comedi_subdevice *s,
        struct comedi_insn *insn, unsigned int *data) {
    unsigned int n;
    unsigned int chan;

    chan = CR_CHAN(insn->chanspec);
    for (n = 0; n < insn->n; n++) {
        data[n] = 128;
    }
    return n;
}

static int bcm2708_check_pinmode(void) {
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))
#define GPIO_PULL   *(gpio+37)
#define GPIO_PULLCLK0 *(gpio+38)

    int pin;

    /* enable pull-up on GPIO */
    GPIO_PULL = 2;
    udelay(50);
    /* clock on GPIO */
    GPIO_PULLCLK0 = 0x0000c403;
    udelay(50);
    GPIO_PULL = 0;
    GPIO_PULLCLK0 = 0;

    /* SPI is on GPIO 7..11 */
    for (pin = 7; pin <= 11; pin++) {
        INP_GPIO(pin); /* set mode to GPIO input first */
        SET_GPIO_ALT(pin, 0); /* set mode to ALT 0 */
    }
    /* look for SPI offboard chip responses later */
    /* Just set gert_detected for now */

    gert_detected = TRUE;
    for (pin = 14; pin <= 15; pin++) {
        INP_GPIO(pin); /* set RS-232 mode to GPIO input again */
    }

    return TRUE;
}

static int daqgert_ai_config(struct comedi_device *dev,
        struct comedi_subdevice *s) {

    int pin, detect_code;
    /* SPI data transfers, send a few dummys for config info */
    comedi_do_one_message(CMD_DUMMY_CFG, CSnA, 1);
    comedi_do_one_message(CMD_DUMMY_CFG, CSnA, 1);
    comedi_do_one_message(CMD_DUMMY_CFG, CSnA, 1);
    if ((comedi_ctl.rx_buff[0]&0b11000000) != 0b01000000) {
        comedi_do_one_message(0b01100000, CSnA, 1); /* check for channel 0 SE */
        detect_code = comedi_ctl.rx_buff[0];
        if ((comedi_ctl.rx_buff[0]&0b00000100) == 0) {
            spi_adc.pic18 = 1; /* MCP3002 mode */
            spi_adc.chan = 2;
            spi_adc.range = 0; /* range 2.048 */
            spi_adc.bits = 0; /* 10 bits */
            dev_info(dev->class_dev,
                    "Gertboard ADC chip Board Detected, %i Channels, Range code %i, Bits code %i, PIC code %i, Detect Code %i\n",
                    spi_adc.chan, spi_adc.range, spi_adc.bits, spi_adc.pic18, detect_code);
            return spi_adc.chan;
        }
        spi_adc.pic18 = 0; /* SPI probes found nothing */
        /* look for the gertboard SPI devices .pic18 code 1 */
        dev_info(dev->class_dev, "No GERT Board Found, GPIO pins only. Detect Code %i\n",
                detect_code);
        gert_detected = FALSE;
        for (pin = 7; pin <= 11; pin++) {
            INP_GPIO(pin); /* set mode to GPIO input again */
        }
        return 1; /* dummy chan */
    }
    spi_adc.pic18 = 2; /* PIC18 single-end mode 10 bits */
    spi_adc.chan = comedi_ctl.rx_buff[0]&0x0f;
    spi_adc.range = (comedi_ctl.rx_buff[0]&0b00100000) >> 5;
    spi_adc.bits = (comedi_ctl.rx_buff[0]&0b00010000) >> 4;
    if (spi_adc.bits) {
        spi_adc.pic18 = 3; /* PIC18 diff mode 12 bits */
    }
    dev_info(dev->class_dev,
            "PIC spi slave ADC chip Board Detected, %i Channels, Range code %i, Bits code %i, PIC code %i\n",
            spi_adc.chan, spi_adc.range, spi_adc.bits, spi_adc.pic18);

    return spi_adc.chan;
}

static int daqgert_ao_config(struct comedi_device *dev,
        struct comedi_subdevice *s) {

    /* Stuff here? */
    return NUM_AO_CHAN;
}

static int daqgert_attach(struct comedi_device *dev, struct comedi_devconfig *it) {
    const struct daqgert_board *thisboard = dev->board_ptr;
    struct comedi_subdevice *s;
    int ret, num_subdev = 1, i, d;

    /* Use the kernel system_rev EXPORT_SYMBOL */
    RPisys_rev = system_rev; /* what board are we running on? */
    if (RPisys_rev < 2) {
        dev_err(dev->class_dev, "Invalid RPi board revision! %u\n", RPisys_rev);
        return -EINVAL;
    }

    gpio = ioremap(GPIO_BASE, SZ_16K); /* lets get access to the GPIO base */
    if (!gpio) {
        dev_err(dev->class_dev, "Invalid io base address!\n");
        return -EINVAL;
    }
    dev->iobase = GPIO_BASE; /* filler */

    /* setup the pins in a static matter for now */
    /* PIN mode for all */
    wiringPiSetup(dev);
    for (i = 0; i < NUM_DIO_OUTPUTS; i++) { /* [0..7] OUTPUTS */
        pinModeWPi(i, OUTPUT);
    }
    num_dio_chan = NUM_DIO_CHAN; /* Rev 1 board setup first */
    if (RPisys_rev > 3) /* This a Rev 2 board "I hope" */
        num_dio_chan = NUM_DIO_CHAN_REV2;
    for (i = NUM_DIO_OUTPUTS; i < num_dio_chan; i++) { /* [8..16] or 20 INPUTS */
        pinModeWPi(i, INPUT);
    }

    /* Change pins [GPIO 7..11] to ALT SPI mode if gertboard found */
    /* These are GPIO pin numbers NOT WPi pin numbers */
    bcm2708_check_pinmode(); /* looking for a GERT Board */
    /* assume we have a gertboard */
    dev_info(dev->class_dev, "GertBoard Detection Started\n");
    num_subdev = 3;

    /* Call SPI setup routines */
    platform_driver_probe(&bcm2708_spi_driver, bcm2708_spi_probe);

    dev->board_name = thisboard->name;
    ret = comedi_alloc_subdevices(dev, num_subdev);
    if (ret)
        return ret;

    /* daq-gert dio */
    s = &dev->subdevices[0];
    s->type = COMEDI_SUBD_DIO;
    s->subdev_flags = SDF_READABLE | SDF_WRITABLE;
    s->n_chan = num_dio_chan;
    s->len_chanlist = num_dio_chan;
    s->range_table = &range_digital;
    s->maxdata = 1;
    s->insn_bits = daqgert_dio_insn_bits;
    s->insn_config = daqgert_dio_insn_config;
    s->state = 0;
    s->io_bits = DIO_PINS_DEFAULT; /* set output bits */
    d = s->io_bits;

    if (num_subdev > 1) { /* we have the SPI ADC DAC on board */
        /* daq-gert ai */
        s = &dev->subdevices[1];
        s->private = &pic_info_pic18; /* SPI adc slave conv delay */
        num_ai_chan = daqgert_ai_config(dev, s); /* config SPI ports for ai use */
        s->type = COMEDI_SUBD_AI;
        /* we support single-ended (ground)  */
        s->subdev_flags = SDF_READABLE | SDF_GROUND;
        s->n_chan = num_ai_chan;
        s->len_chanlist = num_ai_chan;
        if (spi_adc.bits) {
            s->maxdata = (1 << 12) - 1;
        } else {
            s->maxdata = (1 << 10) - 1;
        }
        if (spi_adc.range) {
            s->range_table = &daqgert_ai_range2_048;
        } else {
            s->range_table = &daqgert_ai_range3_300;
        }
        s->insn_read = daqgert_ai_rinsn;

        /* daq-gert ao */
        s = &dev->subdevices[2];
        num_ao_chan = daqgert_ao_config(dev, s); /* config SPI ports for ao use */
        s->type = COMEDI_SUBD_AO;
        /* we support single-ended (ground)  */
        s->subdev_flags = SDF_WRITABLE | SDF_GROUND;
        s->n_chan = num_ao_chan;
        s->len_chanlist = num_ao_chan;
        s->maxdata = (1 << 12) - 1;
        s->range_table = &daqgert_ao_range;
        s->insn_write = daqgert_ao_winsn;
        s->insn_read = daqgert_ao_rinsn;
    }

    dev_info(dev->class_dev, "%s attached: GPIO iobase 0x%lx, ioremap 0x%lx, GPIO wpi-pins 0x%x\n",
            dev->driver->driver_name,
            dev->iobase,
            (long unsigned int) gpio,
            (unsigned int) d);

    return 0;
}

static void daqgert_detach(struct comedi_device *dev) {

    iounmap(gpio);
    platform_driver_unregister(&bcm2708_spi_driver);
    dev_info(dev->class_dev, "daq_gert detached\n");
}

static const struct daqgert_board daqgert_boards[] = {
    {
        .name = "daq-gert",
    },
    {
        .name = "daq_gert",
    },
};

static struct comedi_driver daqgert_driver = {
    .driver_name = "daq_gert",
    .module = THIS_MODULE,
    .attach = daqgert_attach,
    .detach = daqgert_detach,
    .board_name = &daqgert_boards[0].name,
    .num_names = ARRAY_SIZE(daqgert_boards),
    .offset = sizeof (struct daqgert_board),
};

static int __init daqgert_init(void) {
    int ret;

    ret = comedi_driver_register(&daqgert_driver);
    if (ret < 0)
        return ret;
    comedi_ctl.tx_buff = kmalloc(SPI_BUFF_SIZE, GFP_KERNEL | GFP_DMA);
    if (!comedi_ctl.tx_buff) {
        return -ENOMEM;
    }
    comedi_ctl.rx_buff = kmalloc(SPI_BUFF_SIZE, GFP_KERNEL | GFP_DMA);
    if (!comedi_ctl.rx_buff) {

        return -ENOMEM;
    }
    return 0;
}
module_init(daqgert_init);

static void __exit daqgert_exit(void) {

    if (comedi_ctl.tx_buff)
        kfree(comedi_ctl.tx_buff);
    if (comedi_ctl.rx_buff)
        kfree(comedi_ctl.rx_buff);

    comedi_driver_unregister(&daqgert_driver);
}
module_exit(daqgert_exit);

MODULE_AUTHOR("Fred Brooks <nsaspook@nsaspook.com>");
MODULE_DESCRIPTION(
        "Comedi driver for RASPI GERTBOARD DIO/AI/AO");
MODULE_VERSION("0.0.07");
MODULE_LICENSE("GPL");
