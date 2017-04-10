/*
 * execd_config.S: Code for enabling and disabling execution drafting.
 *
 * Copyright (C) 2017 Princeton University
 */



#define SUNHV_ENABLE_EXECD 0x2e
#define SUNHV_DISABLE_EXECD 0x2f
#define SUNHV_FAST_TRAP 0x80

.text
.align  32
.globl  enable_execd
enable_execd:
  mov SUNHV_ENABLE_EXECD, %o5
  ta  SUNHV_FAST_TRAP
retl nop

.globl  disable_execd
disable_execd:
  mov SUNHV_DISABLE_EXECD, %o5
  ta  SUNHV_FAST_TRAP
retl nop




