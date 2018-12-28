/*
 * linux/drivers/video/v586fb.c -- v586 frame buffer device
 *
 * Copyright (C) 2017
 *
 *      Valptek <contact@valptek.com>
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <asm/uaccess.h>
#include <asm/setup.h>
#include <linux/fb.h>
#include <linux/module.h>
#include <asm/pgtable.h>

#define v586_PHYS_SCREEN_ADDR 0xfff0e00000
#define v586_PHYS_SCREEN_SIZE 0x00100000
/* static void *videomemory; */
static u_long videomemorysize = v586_PHYS_SCREEN_SIZE;
module_param(videomemorysize, ulong, 0);

static struct fb_fix_screeninfo v586fb_fix = {
	.id		= "v586fb",
	.smem_len	= 640*480*2,
	.type		= FB_TYPE_PACKED_PIXELS,
	.visual		= FB_VISUAL_TRUECOLOR,
	.line_length	= 640*2,
	.accel		= FB_ACCEL_NONE,
};

static struct fb_var_screeninfo v586fb_var = {
	.xres		= 640,
	.yres		= 480,
	.xres_virtual	= 640,
	.yres_virtual	= 480,
	.bits_per_pixel	= 16,
    	.red		= {11, 5, 0},
	.green		= {5, 6, 0},
	.blue		= {0, 5, 0},
	.activate	= FB_ACTIVATE_NOW,
	.height		= 230,
	.width		= 300,
	.vmode		= FB_VMODE_NONINTERLACED,
};

static struct fb_ops v586fb_ops = {
	.owner		= THIS_MODULE,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};

static int v586fb_probe(struct platform_device *dev)
{
	struct fb_info *info;

	v586fb_fix.smem_start = v586_PHYS_SCREEN_ADDR;

	info = framebuffer_alloc(sizeof(u32) * 16, &dev->dev);
	if (!info) {
		printk(KERN_ERR "Unable to create v586fb fb_info struct\n");
		return -ENOMEM;
    }

    fb_info(info, "Starting v586fb frame buffer driver probe\n");

	if (!request_mem_region(v586_PHYS_SCREEN_ADDR, v586_PHYS_SCREEN_SIZE,	"v586fb")) {
		printk(KERN_INFO"v586fb: cannot get framebuffer\n");
	}

	info->var = v586fb_var;
	info->fix = v586fb_fix;
	info->fbops = &v586fb_ops;
	info->flags = FBINFO_DEFAULT;  /* not as module for now */
	info->par = NULL;
	info->screen_base = ioremap(v586_PHYS_SCREEN_ADDR,videomemorysize); /* (char *) v586fb_fix.smem_start; */

        info->pseudo_palette = kzalloc(256*4, GFP_KERNEL);

    fb_info(info, "Initialised fb_info struct\n");

	if (fb_alloc_cmap(&info->cmap, 256, 0) < 0) {
		framebuffer_release(info);
		return -ENOMEM;
	}

    fb_info(info, "Allocated cmap\n");

	if (register_framebuffer(info) < 0) {
		printk(KERN_ERR "Unable to register v586fb frame buffer\n");
		fb_dealloc_cmap(&info->cmap);
		framebuffer_release(info);
		return -EINVAL;
	}

    fb_info(info, "Registered framebuffer\n");

	fb_info(info, "v586fb frame buffer alive and kicking !\n");
	return 0;
}

static struct platform_driver v586fb_driver = {
	.probe	= v586fb_probe,
	.driver	= {
		.name	= "v586fb",
	},
};

static struct platform_device v586fb_device = {
	.name	= "v586fb",
};

int __init v586fb_init(void)
{
	int ret = 0;

    printk("Starting v586fb frame buffer driver init\n");

	if (fb_get_options("v586fb", NULL))
		return -ENODEV;

	ret = platform_driver_register(&v586fb_driver);

	if (!ret) {
		ret = platform_device_register(&v586fb_device);
		if (ret)
			platform_driver_unregister(&v586fb_driver);
	}
	return ret;
}

module_init(v586fb_init);
MODULE_LICENSE("GPL");
