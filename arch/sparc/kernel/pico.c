#include <linux/kernel.h>
#include <linux/syscalls.h>
#include <asm/hypervisor.h>

SYSCALL_DEFINE1(pico_start, uint32_t __user *, mem) {
    printk("Entered pico syscall\n");
	hcall_pico_start();
    printk("Exiting pico syscall\n");
    return 0;
}

