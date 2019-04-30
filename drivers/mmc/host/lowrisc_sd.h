/*
 *  LowRISC PCI Secure Digital Host Controller Interface driver
 *
 *  Based on toshsd.h
 *
 *  Copyright (C) 2014 Ondrej Zary
 *  Copyright (C) 2007 Richard Betts, All Rights Reserved.
 *
 *      Based on asic3_mmc.c Copyright (c) 2005 SDG Systems, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

enum {align_reg,clk_din_reg,arg_reg,cmd_reg,
      setting_reg,start_reg,reset_reg,blkcnt_reg,
      blksiz_reg,timeout_reg,clk_pll_reg,irq_en_reg,
      unused1,unused2,unused3,led_reg};

enum {resp0,resp1,resp2,resp3,
      wait_resp,status_resp,packet_resp0,packet_resp1,
      data_wait_resp,trans_cnt_resp,obsolete1,obsolet2,
      detect_resp,xfr_addr_resp,irq_stat_resp,pll_resp,
      align_resp,clk_din_resp,arg_resp,cmd_i_resp,
      setting_resp,start_resp,reset_resp,blkcnt_resp,
      blksize_resp,timeout_resp,clk_pll_resp,irq_en_resp,
      dead1,dead2,dead3,from_dip};

enum {SD_APP_OP_COND=41, data_buffer_offset=0x2000};

enum {SD_CARD_RESP_END=1,SD_CARD_RW_END=2, SD_CARD_CARD_REMOVED_0=4, SD_CARD_CARD_INSERTED_0=8};

enum {red_led = 1 << 21};

struct lowrisc_sd_host {
  struct platform_device *pdev;
  struct mmc_host *mmc;

  spinlock_t lock;

  struct mmc_request *mrq;/* Current request */
  struct mmc_command *cmd;/* Current command */
  struct mmc_data *data;	/* Current data request */

  struct sg_mapping_iter sg_miter; /* for PIO */

  void __iomem *ioaddr; /* mapped address */
  int irq;
  int int_en, width_setting;
};
