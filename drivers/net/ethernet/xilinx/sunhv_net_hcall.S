//
//#define SUNHV_NET_READ	     0xf2
//#define SUNHV_NET_WRITE      0xf3
//
//#define SUNHV_FAST_TRAP      0x80
//
//
//    .text
//    .align  32
//    .globl  sunhv_net_read
//sunhv_net_read:
//    mov     SUNHV_NET_READ, %o5
//    ta      SUNHV_FAST_TRAP
//    mov     %o1, %o0
//    retl
//    nop
//    !neg     %o0, %o2
//    !tst     %o0
//    !movz    %xcc, %o1, %o0
//    !retl
//    !movnz   %xcc, %o2, %o0
//
//
//    .text
//    .align  32
//    .globl  sunhv_net_write
//sunhv_net_write:
//    mov     SUNHV_NET_WRITE, %o5
//    ta      SUNHV_FAST_TRAP
//    mov     %o1, %o0
//    retl
//    nop
//    !neg     %o0, %o2
//    !tst     %o0
//    !movz    %xcc, %o1, %o0
//    !retl
//    !movnz   %xcc, %o2, %o0
