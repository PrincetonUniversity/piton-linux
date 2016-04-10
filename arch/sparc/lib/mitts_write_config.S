/*
 * mitts_write_config.S: Code for swapping register state for MITTS.
 *
 * Copyright (C) 2016 Princeton University
 */

#define SUNHV_SET_MITTS_REGS 0x2C

#define SUNHV_FAST_TRAP 0x80

	.text
	.align	32
	.globl	mitts_write_config
mitts_write_config:
	mov	SUNHV_SET_MITTS_REGS, %o5
	ta	SUNHV_FAST_TRAP
	retl
	 nop
