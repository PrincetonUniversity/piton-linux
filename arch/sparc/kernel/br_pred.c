#include <linux/kernel.h>
#include <linux/syscalls.h>
#include <asm/hypervisor.h>

// Values for accessing and modifying BPR ASIs
#define CFG_REG_ASI                 0x1a
#define CFG_RESET_OFFSET            0x20
#define CFG_RIGHT_OFFSET            0x28
#define CFG_WRONG_OFFSET            0x30

SYSCALL_DEFINE1(bpr_reset, unsigned long, br_pred_start) {
    unsigned long reset_offset = CFG_RESET_OFFSET;
   // write to asi to start branch predictor 

   asm volatile("stxa %1, [%0] 0x1a": : "r"(reset_offset), "r"(br_pred_start));
   return 0;
}

SYSCALL_DEFINE1(bpr_read_stats, unsigned long __user *, br_pred_stats) {
    unsigned long num_right = 0;
    unsigned long num_wrong = 0;
    unsigned long right_offset = CFG_RIGHT_OFFSET;
    unsigned long wrong_offset = CFG_WRONG_OFFSET;

    asm volatile("ldxa [%1] 0x1a, %0": "=r"(num_right): "r"(right_offset));
    asm volatile("ldxa [%1] 0x1a, %0": "=r"(num_wrong): "r"(wrong_offset));

    copy_to_user(br_pred_stats, &num_right, sizeof(unsigned long));
    copy_to_user(br_pred_stats + 1, & num_wrong, sizeof(unsigned long));
    printk("Right: %ld, wrong: %ld\n", num_right, num_wrong);
    return 0;
}
