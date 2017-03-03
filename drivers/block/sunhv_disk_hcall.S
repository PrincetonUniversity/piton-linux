
#define SUNHV_DISK_READ	      0xf0
#define SUNHV_DISK_WRITE      0xf1

#define SUNHV_FAST_TRAP       0x80


    .text
    .align  32
    .globl  sunhv_disk_read
sunhv_disk_read:
    mov     SUNHV_DISK_READ, %o5
    ta      SUNHV_FAST_TRAP
    neg     %o0, %o2
    tst     %o0
    movz    %xcc, %o1, %o0
    retl
    movnz   %xcc, %o2, %o0


    .text
    .align  32
    .globl  sunhv_disk_write
sunhv_disk_write:
    mov     SUNHV_DISK_WRITE, %o5
    ta      SUNHV_FAST_TRAP
    neg     %o0, %o2
    tst     %o0
    movz    %xcc, %o1, %o0
    retl
    movnz   %xcc, %o2, %o0

