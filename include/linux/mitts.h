#ifndef _LINUX_MITTS_H
#define _LINUX_MITTS_H

#include <linux/types.h>

#define N_BINS 10
#define BIN_MASK 0x3F
#define BIN_SHMT 6
#define CFG_SHMT 4
#define ENABLED 0x3
#define SCALE_MASK 0x03FF
#define SCALE_SHMT 16
#define EN_MASK 0x1
#define INT_MASK 0xFFFF

/*
 * MITTS configuration struct, used in mitts_setcfg syscall.
 */
struct mitts_cfg {
	uint8_t en;		/* Is MITTS enabled? */
	uint8_t bins[N_BINS];	/* Bin values for MITTS bins (6 bits) */
	uint16_t interval;	/* Refresh interval in cycles */
	uint16_t scale;		/* Timescale between bins in cycles (10 bits) */
};

#endif
