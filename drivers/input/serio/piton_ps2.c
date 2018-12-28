/*
 * Piton PS/2 device driver
 *
 * (c) 2005 MontaVista Software, Inc.
 * (c) 2008 Xilinx, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */


#include <linux/module.h>
#include <linux/serio.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/printk.h>

#include <asm/oplib.h>
#include <asm/hypervisor.h>

#define DRIVER_NAME             "piton_ps2"


static int piton_ps2_polling_period = 5;

static int __init setup_piton_ps2_polling_period(char *str)
{
        get_option(&str, &piton_ps2_polling_period);
        return 1;
}
__setup("piton_ps2_polling_period=", setup_piton_ps2_polling_period);

static struct timer_list piton_ps2_timer;

#define PITON_PS2_BASE_ADDR 0xfff0f00000
#define PITON_PS2_DATA_OFFSET 0x4

#define piton_ps2_readb         sunhv_readb
#define piton_ps2_writeb        sunhv_writeb

struct piton_ps2_data {
        int irq;
        spinlock_t lock;
        void __iomem *base_address;     /* virt. address of control registers */
        unsigned int flags;
        struct serio *serio;            /* serio */
        struct device *dev;
};

static struct piton_ps2_data *ps2_data;


/******************************
*  Polling callback function *
/*****************************/

static void piton_ps2_poll(unsigned long data)
{
    struct piton_ps2_data *drvdata = ps2_data;
    u8 flag;
    u8 c;
    //int status;

    flag = piton_ps2_readb(drvdata->base_address);

    if (flag == 0)
    {

    }
    else
    {
        c = piton_ps2_readb(drvdata->base_address + PITON_PS2_DATA_OFFSET);
        drvdata->flags = 1;  // TODO: figure out what is the flag for
        serio_interrupt(drvdata->serio, c, drvdata->flags);
                drvdata->flags = 0;

        printk("Received a scancode: %x\n", c);
    }

    mod_timer(&piton_ps2_timer, jiffies + piton_ps2_polling_period);

}


/*******************/
/* serio callbacks */
/*******************/

/**
 * piton_ps2_write() - dummy function
 * @pserio:     pointer to the serio structure of the PS/2 port
 * @c:          data that needs to be written to the PS/2 port
 *
 */
static int piton_ps2_write(struct serio *pserio, unsigned char c)
{
        return 0;
}

/**
 * piton_ps2_open() - called when a port is opened by the higher layer.
 * @pserio:     pointer to the serio structure of the PS/2 device
 *
 * This function requests initializes the polling timer
 */
static int piton_ps2_open(struct serio *pserio)
{
        struct piton_ps2_data *drvdata = pserio->port_data;
        int error;
        u8 c;

    init_timer(&piton_ps2_timer);
        piton_ps2_timer.function = piton_ps2_poll;
        mod_timer(&piton_ps2_timer, jiffies + piton_ps2_polling_period);


        return 0;               /* success */
}

/**
 * piton_ps2_close() - dummy function
 * @pserio:     pointer to the serio structure of the PS/2 device
 *
 */
static void piton_ps2_close(struct serio *pserio)
{
}

/**
 * piton_ps2_of_probe - probe method for the PS/2 device.
 * @of_dev:     pointer to OF device structure
 * @match:      pointer to the structure used for matching a device
 *
 * This function probes the PS/2 device in the device tree.
 * It initializes the driver data structure and the hardware.
 * It returns 0, if the driver is bound to the PS/2 device, or a negative
 * value if there is an error.
 */
static int piton_ps2_of_probe(struct platform_device *ofdev)
{
        struct piton_ps2_data *drvdata;
        struct serio *serio;
        struct device *dev = &ofdev->dev;
        resource_size_t remap_size, phys_addr;
        unsigned int irq;
        int error;

        dev_info(dev, "Device Tree Probing \'%s\'\n", dev->of_node->name);

        /* Get IRQ for the device */
        irq = irq_of_parse_and_map(dev->of_node, 0);
        /*if (!irq) {
                dev_err(dev, "no IRQ found\n");
                return -ENODEV;
        }*/

        drvdata = kzalloc(sizeof(struct piton_ps2_data), GFP_KERNEL);
        ps2_data = drvdata;
        serio = kzalloc(sizeof(struct serio), GFP_KERNEL);
        if (!drvdata || !serio) {
                error = -ENOMEM;
                goto failed1;
        }

        spin_lock_init(&drvdata->lock);
        drvdata->irq = irq;
        drvdata->serio = serio;
        drvdata->dev = dev;

        phys_addr = PITON_PS2_BASE_ADDR;
        remap_size = 8;

        if (!request_mem_region(phys_addr, remap_size, DRIVER_NAME)) {
                dev_err(dev, "Couldn't lock memory region at 0x%08llX\n",
                        (unsigned long long)phys_addr);
                error = -EBUSY;
                goto failed1;
        }   // TODO: check this function request_mem_region

        /* Fill in configuration data and add them to the list */
        drvdata->base_address = ioremap(phys_addr, remap_size);  // TODO: check it too

        if (drvdata->base_address == NULL) {
                dev_err(dev, "Couldn't ioremap memory at 0x%08llX\n",
                        (unsigned long long)phys_addr);
                error = -EFAULT;
                goto failed2;
        }

        dev_info(dev, "Xilinx PS2 at 0x%08llX mapped to 0x%p, irq=%d\n",
		 (unsigned long long)phys_addr, drvdata->base_address,
		 drvdata->irq);

        serio->id.type = SERIO_8042;    
        serio->write = NULL;  // don't allow to write
        serio->open = piton_ps2_open;
        serio->close = piton_ps2_close;
        serio->port_data = drvdata;
        serio->dev.parent = dev;
        snprintf(serio->name, sizeof(serio->name),
                 "PITON PS/2 at %08llX", (unsigned long long)phys_addr);
        snprintf(serio->phys, sizeof(serio->phys),
                 "piton_ps2/serio at %08llX", (unsigned long long)phys_addr);

        serio_register_port(serio);

        platform_set_drvdata(ofdev, drvdata);
        return 0;               /* success */

failed2:
        release_mem_region(phys_addr, remap_size);
failed1:
        kfree(serio);
        kfree(drvdata);

        return error;
}

/**
 * piton_ps2_of_remove - unbinds the driver from the PS/2 device.
 * @of_dev:     pointer to OF device structure
 *
 * This function is called if a device is physically removed from the system or
 * if the driver module is being unloaded. It frees any resources allocated to
 * the device.
 */
static int piton_ps2_of_remove(struct platform_device *of_dev)
{
        struct piton_ps2_data *drvdata = platform_get_drvdata(of_dev);

        serio_unregister_port(drvdata->serio);
        iounmap(drvdata->base_address);

        release_mem_region(PITON_PS2_BASE_ADDR, 8);

        kfree(drvdata);

        return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id piton_ps2_of_match[] = {
        { .compatible = "piton-ps2-1.00.a", },
        { /* end of list */ },
};
MODULE_DEVICE_TABLE(of, piton_ps2_of_match);

static struct platform_driver piton_ps2_of_driver = {
        .driver = {
                .name = DRIVER_NAME,
                .of_match_table = piton_ps2_of_match,
        },
        .probe          = piton_ps2_of_probe,
        .remove         = piton_ps2_of_remove,
};
module_platform_driver(piton_ps2_of_driver);

MODULE_AUTHOR("Fei Gao");
MODULE_DESCRIPTION("Piton PS/2 driver");
MODULE_LICENSE("GPL");
