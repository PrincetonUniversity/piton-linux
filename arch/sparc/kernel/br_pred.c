#include <linux/kernel.h>
#include <linux/syscalls.h>
#include <asm/hypervisor.h>

// Values for accessing and modifying BPR ASIs
#define CFG_REG_ASI                 0x1a
#define CFG_RESET_OFFSET            0x20
#define CFG_RIGHT_OFFSET            0x28

SYSCALL_DEFINE1(bpr_reset, unsigned long, br_pred_start) {
    unsigned long reset_offset = CFG_RESET_OFFSET;
    unsigned long reset_high = 1;
   // write to asi to start branch predictor 

   asm volatile("stxa %1, [%0] 0x1a": : "r"(reset_offset), "r"(reset_high));
   return 0;
}
