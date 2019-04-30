/*
 *  LowRISC Secure Digital Host Controller Interface driver
 *
 *  Copyright (C) 2018 LowRISC CIC
 *
 *    Based on toshsd.c
 *    Copyright (C) 2014 Ondrej Zary
 *    Copyright (C) 2007 Richard Betts, All Rights Reserved.
 *
 *	Based on asic3_mmc.c, copyright (c) 2005 SDG Systems, LLC and,
 *	sdhci.c, copyright (C) 2005-2006 Pierre Ossman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/pm.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/platform_device.h>

#include "lowrisc_sd.h"

#define DRIVER_NAME "lowrisc-mmc"
#define LOG(l) printk l
#define LOGV(l) pr_debug l

static volatile uint64_t *led_sd_base;
static uint32_t led_last;

#ifdef CONFIG_LOWRISC_GPIO
#include <asm/uaccess.h>
#include <linux/cdev.h>
#define GPIO_MAJOR  200
#define GPIO_MINOR  0
#define GPIO_DEV_COUNT 2

static int     gpio_open( struct inode *, struct file * );
static ssize_t gpio_read( struct file * ,        char *  , size_t, loff_t *);
static ssize_t gpio_write(struct file * , const  char *  , size_t, loff_t *);
static int     gpio_close(struct inode *, struct file * );
struct file_operations gpio_fops = {
        read    :       gpio_read,
        write   :       gpio_write,
        open    :       gpio_open,
        release :       gpio_close,
        owner   :       THIS_MODULE
};

struct cdev gpio_cdev;

int gpio_init_module(void)
{

	dev_t devno;
	unsigned int count = GPIO_DEV_COUNT; // apply for two minor for two LED
	int err;

	devno = MKDEV(GPIO_MAJOR, GPIO_MINOR);
	register_chrdev_region(devno, count , "myLED");

	// -- initial the char device 
	cdev_init(&gpio_cdev, &gpio_fops);
	gpio_cdev.owner = THIS_MODULE;
	err = cdev_add(&gpio_cdev, devno, count);

	if (err < 0)
	{
		printk("Device Add Error\n");
		return -1;
	}

	printk("This is lowrisc-gpio driver.\n");

        return 0;
}

void gpio_cleanup_module(void)
{
	dev_t devno;

	devno = MKDEV(GPIO_MAJOR, GPIO_MINOR);

	unregister_chrdev_region(devno, GPIO_DEV_COUNT);
	cdev_del(&gpio_cdev);
}

/*
 * file operation: OPEN 
 * */
static int gpio_open(struct inode *inod, struct file *fil)
{
    return 0;
}

/*
 * file operation: READ
 * */
static ssize_t gpio_read(struct file *filp, char *buff, size_t len, loff_t *off)
{
  static const char hex[] = "0123456789ABCDEF";
  
	int led_value = 0;
	short count;
        char msg[5];

        if (*off)
          return 0;
        
        if (led_sd_base)
          led_value = led_sd_base[from_dip];
        
        msg[0] = hex[(led_value >> 12)&0xF];
        msg[1] = hex[(led_value >> 8)&0xF];
        msg[2] = hex[(led_value >> 4)&0xF];
        msg[3] = hex[(led_value >> 0)&0xF];

        if (len > 4)
          len = 4;

	count = raw_copy_to_user(buff, msg, len);

	return len;
}

/*
 * file operation: WRITE
 * */
static ssize_t gpio_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	short count;
        char *endp, msg[7];

        if (*off)
          return 0;
        
        if (len > 6)
          len = 6;
	count = raw_copy_from_user( msg, buff, len );
        msg[len] = 0;
        
        if (led_sd_base)
          {
            led_last = (led_last&red_led) | (simple_strtol(msg, &endp, 16) & ~red_led);
            printk("User msg %s, led=%X", msg, led_last);
            led_sd_base[led_reg] = led_last;
          }
	return len;
}

/*
 * file operation : CLOSE
 * */
static int gpio_close(struct inode *inod, struct file *fil)
{
	return 0;
}
#endif

static void lowrisc_sd_set_led(struct lowrisc_sd_host *host, unsigned char state)
{
  volatile uint64_t *sd_base = host->ioaddr;
  if (!led_sd_base)
    led_sd_base = sd_base;
  if (state)
    led_last |= red_led;
  else
    led_last &= ~red_led;
  led_sd_base[led_reg] = led_last;
}

void sd_align(struct lowrisc_sd_host *host, int d_align)
{
  volatile uint64_t *sd_base = host->ioaddr;
  sd_base[align_reg] = d_align;
}

void sd_clk_div(struct lowrisc_sd_host *host, int clk_div)
{
  volatile uint64_t *sd_base = host->ioaddr;
  /* This section is incomplete */
  sd_base[clk_din_reg] = clk_div;
}

void sd_arg(struct lowrisc_sd_host *host, uint32_t arg)
{
  volatile uint64_t *sd_base = host->ioaddr;
  sd_base[arg_reg] = arg;
}

void sd_cmd(struct lowrisc_sd_host *host, uint32_t cmd)
{
  volatile uint64_t *sd_base = host->ioaddr;
  sd_base[cmd_reg] = cmd;
}

void sd_setting(struct lowrisc_sd_host *host, int setting)
{
  volatile uint64_t *sd_base = host->ioaddr;
  sd_base[setting_reg] = setting;
}

void sd_cmd_start(struct lowrisc_sd_host *host, int sd_cmd)
{
  volatile uint64_t *sd_base = host->ioaddr;
  sd_base[start_reg] = sd_cmd;
}

void sd_reset(struct lowrisc_sd_host *host, int sd_rst, int clk_rst, int data_rst, int cmd_rst)
{
  volatile uint64_t *sd_base = host->ioaddr;
  sd_base[reset_reg] = ((sd_rst&1) << 3)|((clk_rst&1) << 2)|((data_rst&1) << 1)|((cmd_rst&1) << 0);
}

void sd_blkcnt(struct lowrisc_sd_host *host, int d_blkcnt)
{
  volatile uint64_t *sd_base = host->ioaddr;
  sd_base[blkcnt_reg] = d_blkcnt&0xFFFF;
}

void sd_blksize(struct lowrisc_sd_host *host, int d_blksize)
{
  volatile uint64_t *sd_base = host->ioaddr;
  sd_base[blksiz_reg] = d_blksize&0xFFF;
}

void sd_timeout(struct lowrisc_sd_host *host, int d_timeout)
{
  volatile uint64_t *sd_base = host->ioaddr;
  sd_base[timeout_reg] = d_timeout;
}

void sd_irq_en(struct lowrisc_sd_host *host, int mask)
{
  volatile uint64_t *sd_base = host->ioaddr;
  sd_base[irq_en_reg] = mask;
  host->int_en = mask;
  pr_debug("sd_irq_en(%X)\n", mask);
}

static void lowrisc_sd_init(struct lowrisc_sd_host *host)
{

}

/* Set MMC clock / power */
static void __lowrisc_sd_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct lowrisc_sd_host *host = mmc_priv(mmc);
	switch (ios->power_mode) {
	case MMC_POWER_OFF:
	  mdelay(1);
	  break;
	case MMC_POWER_UP:
	  break;
	case MMC_POWER_ON:
#if 0
	  mdelay(20);
#endif          
	  break;
	}

	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_1:
	  host->width_setting = 0;
	  break;
	case MMC_BUS_WIDTH_4:
	  host->width_setting = 0x20;
	  break;
	}
}

static void lowrisc_sd_finish_request(struct lowrisc_sd_host *host)
{
	struct mmc_request *mrq = host->mrq;

	/* Write something to end the command */
	host->mrq = NULL;
	host->cmd = NULL;
	host->data = NULL;

	sd_reset(host, 0,1,0,1);
	sd_cmd_start(host, 0);
	sd_reset(host, 0,1,1,1);
	lowrisc_sd_set_led(host, 0);
	mmc_request_done(host->mmc, mrq);
}
  
static void lowrisc_sd_cmd_irq(struct lowrisc_sd_host *host)
{
	struct mmc_command *cmd = host->cmd;
        volatile uint64_t *sd_base = host->ioaddr;

	LOGV (("lowrisc_sd_cmd_irq\n"));
	
	if (!host->cmd) {
		dev_warn(&host->pdev->dev, "Spurious CMD irq\n");
		return;
	}
	host->cmd = NULL;

        LOGV (("lowrisc_sd_cmd_irq IRQ line %d\n", __LINE__));
	if (cmd->flags & MMC_RSP_PRESENT && cmd->flags & MMC_RSP_136) {
	  int i;
	  LOGV (("lowrisc_sd_cmd_irq IRQ line %d\n", __LINE__));
		/* R2 */
	  for (i = 0;i < 4;i++)
	    {
	    cmd->resp[i] = sd_base[resp0 + (3-i)] << 8;
	    if (i != 3)
	      cmd->resp[i] |= sd_base[resp0 + (2-i)] >> 24;
	    } 
	} else if (cmd->flags & MMC_RSP_PRESENT) {
	  LOGV (("lowrisc_sd_cmd_irq IRQ line %d\n", __LINE__));
		/* R1, R1B, R3, R6, R7 */
	  cmd->resp[0] = sd_base[resp0];
	}

LOGV (("Command IRQ complete %d %d %x\n", cmd->opcode, cmd->error, cmd->flags));

	/* If there is data to handle we will
	 * finish the request in the mmc_data_end_irq handler.*/
	if (host->data)
	  {
	    host->int_en |= SD_CARD_RW_END;
	  }
	else
	  lowrisc_sd_finish_request(host);
}

static void lowrisc_sd_data_end_irq(struct lowrisc_sd_host *host)
{
	struct mmc_data *data = host->data;
	unsigned long flags;
	
	LOGV (("lowrisc_sd_data_end_irq\n"));

	host->data = NULL;

	if (!data) {
		dev_warn(&host->pdev->dev, "Spurious data end IRQ\n");
		return;
	}

        if (data->flags & MMC_DATA_READ)
	  {
            volatile uint64_t *sd_base = 0x1000 + (volatile uint64_t *)(host->ioaddr);
            int len;
	    size_t blksize = data->blksz;

	    local_irq_save(flags);

            BUG_ON(!sg_miter_next(&host->sg_miter));
            BUG_ON(host->sg_miter.length < blksize);
	  	  
	    if (!((sizeof(u64)-1) & (size_t)(host->sg_miter.addr))) // optimise case for aligned buffer
	      {
		u64 *buf = (u64 *)(host->sg_miter.addr);
		for (len = blksize; len > 0; len -= sizeof(u64))
		  {
		  *buf++ = *sd_base++;
		  }
	      }
	    else
	      {
		u8 *buf = host->sg_miter.addr;
		for (len = blksize; len > 0; len -= sizeof(u64))
		  {
		  u64 scratch = *sd_base++;
		  memcpy(buf, &scratch, sizeof(u64));
		  buf += sizeof(u64);
		  }
	      }
            host->sg_miter.consumed = blksize;
	    sg_miter_stop(&host->sg_miter);

	    local_irq_restore(flags);
	  }

	if (data->error == 0)
		data->bytes_xfered = data->blocks * data->blksz;
	else
		data->bytes_xfered = 0;

	LOGV (("Completed data request xfr=%d\n",
	      data->bytes_xfered));

        //	iowrite16(0, host->ioaddr + SD_STOPINTERNAL);

	lowrisc_sd_finish_request(host);
}

static irqreturn_t lowrisc_sd_irq(int irq, void *dev_id)
{
	struct lowrisc_sd_host *host = dev_id;
        volatile uint64_t *sd_base = host->ioaddr;
	u32 int_reg, int_status;
	int error = 0, ret = IRQ_HANDLED;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	int_status = sd_base[irq_stat_resp];
	int_reg = int_status & host->int_en;

	/* nothing to do: it's not our IRQ */
	if (!int_reg) {
		ret = IRQ_NONE;
		goto irq_end;
	}

	LOGV (("lowrisc_sd IRQ status:%x enabled:%x\n", int_status, host->int_en));

	if (sd_base[wait_resp] >= sd_base[timeout_resp]) {
		error = -ETIMEDOUT;
		LOGV (("lowrisc_sd timeout %lld clocks\n", sd_base[timeout_resp]));
	} else if (int_reg & 0) {
		error = -EILSEQ;
		dev_err(&host->pdev->dev, "BadCRC\n");
        }
        
        LOGV (("lowrisc_sd IRQ line %d\n", __LINE__));

	if (error) {
	  LOGV (("lowrisc_sd IRQ line %d\n", __LINE__));
		if (host->cmd)
			host->cmd->error = error;

		if (error == -ETIMEDOUT) {
		  LOGV (("lowrisc_sd IRQ line %d\n", __LINE__));
                  sd_cmd_start(host, 0);
                  sd_setting(host, 0);
		} else {
		  LOGV (("lowrisc_sd IRQ line %d\n", __LINE__));
			lowrisc_sd_init(host);
			__lowrisc_sd_set_ios(host->mmc, &host->mmc->ios);
			goto irq_end;
		}
	}

        LOGV (("lowrisc_sd IRQ line %d\n", __LINE__));

        /* Card insert/remove. The mmc controlling code is stateless. */
	if (int_reg & SD_CARD_CARD_REMOVED_0)
	  {
	    int mask = (host->int_en & ~SD_CARD_CARD_REMOVED_0) | SD_CARD_CARD_INSERTED_0;
	    sd_irq_en(host, mask);
	    printk("Card removed, mask changed to %d\n", mask);
	    mmc_detect_change(host->mmc, 1);
	  }
	
        LOGV (("lowrisc_sd IRQ line %d\n", __LINE__));
	if (int_reg & SD_CARD_CARD_INSERTED_0)
	  {
	    int mask = (host->int_en & ~SD_CARD_CARD_INSERTED_0) | SD_CARD_CARD_REMOVED_0 ;
	    sd_irq_en(host, mask);
	    printk("Card inserted, mask changed to %d\n", mask);
	    lowrisc_sd_init(host);
	    mmc_detect_change(host->mmc, 1);
	  }

        LOGV (("lowrisc_sd IRQ line %d\n", __LINE__));
	/* Command completion */
	if (int_reg & SD_CARD_RESP_END) {
	  LOGV (("lowrisc_sd IRQ line %d\n", __LINE__));

		lowrisc_sd_cmd_irq(host);
		host->int_en &= ~SD_CARD_RESP_END;
	}

        LOGV (("lowrisc_sd IRQ line %d\n", __LINE__));
	/* Data transfer completion */
	if (int_reg & SD_CARD_RW_END) {
	  LOGV (("lowrisc_sd IRQ line %d\n", __LINE__));

		lowrisc_sd_data_end_irq(host);
		host->int_en &= ~SD_CARD_RW_END;
	}
irq_end:
        sd_irq_en(host, host->int_en);
	spin_unlock_irqrestore(&host->lock, flags);
	return ret;
}

static void lowrisc_sd_start_cmd(struct lowrisc_sd_host *host, struct mmc_command *cmd)
{
  int setting = 0;
  int timeout = 1000000;
  struct mmc_data *data = host->data;
  volatile uint64_t *sd_base = host->ioaddr;
  unsigned long flags;
  spin_lock_irqsave(&host->lock, flags);

  LOGV (("Command opcode: %d\n", cmd->opcode));
/*
  if (cmd->opcode == MMC_STOP_TRANSMISSION) {
    sd_cmd(host, SD_STOPINT_ISSUE_CMD12);

    cmd->resp[0] = cmd->opcode;
    cmd->resp[1] = 0;
    cmd->resp[2] = 0;
    cmd->resp[3] = 0;
    
    lowrisc_sd_finish_request(host);
    return;
  }
*/
  if (!(cmd->flags & MMC_RSP_PRESENT))
    setting = 0;
  else if (cmd->flags & MMC_RSP_136)
    setting = 3;
  else if (cmd->flags & MMC_RSP_BUSY)
    setting = 1;
  else
    setting = 1;
  setting |= host->width_setting;
  
  host->cmd = cmd;
  
  if (cmd->opcode == MMC_APP_CMD)
    {
      /* placeholder */
    }
  
  if (cmd->opcode == MMC_GO_IDLE_STATE)
    {
      /* placeholder */
    }

  LOGV (("testing data flags\n"));
  if (data) {
    setting |= 0x4;
    if (data->flags & MMC_DATA_READ)
      setting |= 0x10;
    else
      {
      setting |= 0x8;
      }
  }

  LOGV (("writing registers\n"));
  /* Send the command */
  sd_reset(host, 0,1,0,1);
  sd_align(host, 0);
  sd_arg(host, cmd->arg);
  sd_cmd(host, cmd->opcode);
  sd_setting(host, setting);
  sd_cmd_start(host, 0);
  sd_reset(host, 0,1,1,1);
  sd_timeout(host, timeout);
  /* start the transaction */ 
  sd_cmd_start(host, 1);
  LOGV (("enabling interrupt\n"));
  sd_irq_en(host, sd_base[irq_en_resp] | SD_CARD_RESP_END);
  spin_unlock_irqrestore(&host->lock, flags);
 LOGV (("leaving lowrisc_sd_start_cmd\n"));
}

static void lowrisc_sd_start_data(struct lowrisc_sd_host *host, struct mmc_data *data)
{
	unsigned int flags = SG_MITER_ATOMIC;

	LOGV (("setup data transfer: blocksize %08x  nr_blocks %d, offset: %08x\n",
	      data->blksz, data->blocks, data->sg->offset));

	host->data = data;

	if (data->flags & MMC_DATA_READ)
		flags |= SG_MITER_TO_SG;
	else
		flags |= SG_MITER_FROM_SG;

	sg_miter_start(&host->sg_miter, data->sg, data->sg_len, flags);

	/* Set transfer length and blocksize */
	sd_blkcnt(host, data->blocks);
	sd_blksize(host, data->blksz);

        if (!(data->flags & MMC_DATA_READ))
	  {
            volatile uint64_t *sd_base = 0x1000 + (volatile uint64_t *)(host->ioaddr);
            struct mmc_data *data = host->data;
            if (sg_miter_next(&host->sg_miter))
              {
                int len;
                size_t blksize = data->blksz;
                BUG_ON(host->sg_miter.length < blksize);
		if (!((sizeof(u64)-1) & (size_t)(host->sg_miter.addr))) // optimise case for aligned buffer
		  {
		    u64 *buf = (u64 *)(host->sg_miter.addr);
		    for (len = blksize; len > 0; len -= sizeof(u64))
		      {
			*sd_base++ = *buf++;
		      }
		  }
		else
		  {
		    u8 *buf = host->sg_miter.addr;
		    for (len = blksize; len > 0; len -= sizeof(u64))
		      {
			u64 scratch;
			memcpy(&scratch, buf, sizeof(u64));
			buf += sizeof(u64);
			*sd_base++ = scratch;
		      }
		  }
                host->sg_miter.consumed = blksize;
                sg_miter_stop(&host->sg_miter);
              }
          }
}

/* Process requests from the MMC layer */
static void lowrisc_sd_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct lowrisc_sd_host *host = mmc_priv(mmc);
        volatile uint64_t *sd_base = host->ioaddr;
	unsigned long flags;

	/* abort if card not present */
	if (sd_base[detect_resp]) {
		mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
		return;
	}

	spin_lock_irqsave(&host->lock, flags);

	WARN_ON(host->mrq != NULL);

	host->mrq = mrq;

	if (mrq->data)
		lowrisc_sd_start_data(host, mrq->data);

	lowrisc_sd_set_led(host, 1);

	lowrisc_sd_start_cmd(host, mrq->cmd);

	spin_unlock_irqrestore(&host->lock, flags);
}

static void lowrisc_sd_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct lowrisc_sd_host *host = mmc_priv(mmc);
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	__lowrisc_sd_set_ios(mmc, ios);
	spin_unlock_irqrestore(&host->lock, flags);
}

static int lowrisc_sd_get_ro(struct mmc_host *mmc)
{
	struct lowrisc_sd_host *host = mmc_priv(mmc);
        volatile uint64_t *sd_base = host->ioaddr;
	return sd_base[detect_resp];
}

static int lowrisc_sd_get_cd(struct mmc_host *mmc)
{
	struct lowrisc_sd_host *host = mmc_priv(mmc);
        volatile uint64_t *sd_base = host->ioaddr;

	return !sd_base[detect_resp];
}

static int lowrisc_sd_card_busy(struct mmc_host *mmc)
{
	struct lowrisc_sd_host *host = mmc_priv(mmc);
        volatile uint64_t *sd_base = host->ioaddr;
	return sd_base[start_resp];
}

static struct mmc_host_ops lowrisc_sd_ops = {
	.request = lowrisc_sd_request,
	.set_ios = lowrisc_sd_set_ios,
	.get_ro = lowrisc_sd_get_ro,
	.get_cd = lowrisc_sd_get_cd,
	.card_busy = lowrisc_sd_card_busy,
};


static void lowrisc_sd_powerdown(struct lowrisc_sd_host *host)
{
  volatile uint64_t *sd_base = host->ioaddr;
  /* mask all interrupts */
  sd_base[irq_en_reg] = 0;
  /* disable card clock */
}

static int lowrisc_sd_probe(struct platform_device *pdev)
{
	int ret;
	struct lowrisc_sd_host *host;
	struct mmc_host *mmc;
        struct resource *iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mmc = mmc_alloc_host(sizeof(struct lowrisc_sd_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto release;
	}

	host = mmc_priv(mmc);
	host->mmc = mmc;

	host->pdev = pdev;

	if (!request_mem_region(iomem->start, resource_size(iomem),
		mmc_hostname(host->mmc))) {
		dev_err(&pdev->dev, "cannot request region\n");
		ret = -EBUSY;
		goto release;
	}
        
        led_sd_base = host->ioaddr;
#ifdef CONFIG_LOWRISC_GPIO
        gpio_init_module();
#endif
	host->ioaddr = ioremap(iomem->start, resource_size(iomem));
	if (!host->ioaddr) {
		ret = -ENOMEM;
		goto release;
	}
	printk("lowrisc-digilent-sd: Lowrisc sd platform driver (%llX-%llX) mapped to %lx\n",
               iomem[0].start,
               iomem[0].end,
               (size_t)(host->ioaddr));
        
        host->irq = platform_get_irq(pdev, 0);
        
	/* Set MMC host parameters */
	mmc->ops = &lowrisc_sd_ops;
	mmc->caps = MMC_CAP_4_BIT_DATA;
	mmc->ocr_avail = MMC_VDD_32_33;

	mmc->f_min = 5000000;
	mmc->f_max = 5000000;
	mmc->max_blk_count = 1;
	
	spin_lock_init(&host->lock);

	lowrisc_sd_init(host);

	ret = request_irq(host->irq, lowrisc_sd_irq, 0, DRIVER_NAME, host);
        
	if (ret)
          {
            printk("request_irq failed\n");
            goto unmap;
          }

	mmc_add_host(mmc);

	printk("lowrisc-sd driver loaded, IRQ %d\n", host->irq);
	sd_irq_en(host, SD_CARD_CARD_INSERTED_0 | SD_CARD_CARD_REMOVED_0); /* get an interrupt either way */
	return 0;

unmap:
release:
	mmc_free_host(mmc);
	return ret;
}

static int lowrisc_sd_remove(struct platform_device *pdev)
{
	struct lowrisc_sd_host *host = platform_get_drvdata(pdev);

	mmc_remove_host(host->mmc);
	lowrisc_sd_powerdown(host);
	free_irq(host->irq, host);
	mmc_free_host(host->mmc);
        return 0;
}

static const struct of_device_id lowrisc_sd_of_match[] = {
	{ .compatible = DRIVER_NAME },
	{ }
};

MODULE_DEVICE_TABLE(of, lowrisc_sd_of_match);

static struct platform_driver lowrisc_sd_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = lowrisc_sd_of_match,
	},
	.probe = lowrisc_sd_probe,
	.remove = lowrisc_sd_remove,
};

module_platform_driver(lowrisc_sd_driver);

MODULE_AUTHOR("Jonathan Kimmitt");
MODULE_DESCRIPTION("LowRISC Secure Digital Host Controller Interface driver");
MODULE_LICENSE("GPL");
