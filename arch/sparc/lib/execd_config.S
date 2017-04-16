/*
 * execd_config.S: Code for enabling and disabling execution drafting.
 *
 * Copyright (C) 2017 Princeton University
 */



#define SUNHV_ENABLE_EXECD 0x2e
#define SUNHV_DISABLE_EXECD 0x2f
#define SUNHV_CPU_CHECK_EXECD_REGISTER 0x30
#define SUNHV_FAST_TRAP 0x80

	.text
	.align  32
	.globl  enable_execd
enable_execd:
	mov SUNHV_ENABLE_EXECD, %o5
  	ta  SUNHV_FAST_TRAP
	retl
	nop

	.globl  disable_execd
disable_execd:
  	mov SUNHV_DISABLE_EXECD, %o5
  	ta  SUNHV_FAST_TRAP
	retl
	nop


.globl  check_execd_register
check_execd_register:
  	mov SUNHV_CPU_CHECK_EXECD_REGISTER, %o5
  	ta  SUNHV_FAST_TRAP
  	mov %o1, %o0
	retl
	nop



