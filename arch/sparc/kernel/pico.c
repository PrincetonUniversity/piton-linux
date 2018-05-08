#include <linux/kernel.h>
#include <linux/syscalls.h>
#include <asm/hypervisor.h>
#include <asm/byteorder.h>
#include <linux/slab.h>

#define PICO_SD_START  0xfff0200000
#define PICO_SD_STATUS (PICO_SD_START + 0x10)
#define PICO_MEM_START 0x3f000000
#define PICO_MEM_BASE  0x3f000004
#define PICO_CACHED    0x3f000080
#define PICO_UNCACHED  0x3f000088

void memory_test(void) {
    unsigned long mem_address = PICO_MEM_START;
    uint32_t measurement;
	hcall_pico_start(mem_address);
    mem_address = PICO_CACHED;
    measurement = sunhv_net_read((void *)mem_address);
    printk("Before cache read: %08x\n", le32_to_cpu(measurement));
    
    measurement = sunhv_net_read((void *)(mem_address + 4));
    printk("After cache read: %08x\n", le32_to_cpu(measurement));

    mem_address = PICO_UNCACHED;
    measurement = sunhv_net_read((void *)mem_address);
    printk("Before uncache read: %08x\n", le32_to_cpu(measurement));

    measurement = sunhv_net_read((void *)(mem_address +4 ));
    printk("After uncache read: %08x\n", le32_to_cpu(measurement));
    
}

unsigned long create_mem_space(void) {
    // allocate 4 MB
    uint32_t *addr = (uint32_t *)__get_free_pages(GFP_KERNEL, 10);
    uint32_t *pa_kernel_addr;
    printk("Kernel virtual address: %p\n", addr);
    pa_kernel_addr = (uint32_t *)virt_to_phys(addr);
    printk("Kernel physical address: %p\n", pa_kernel_addr);
    *addr = 0xdeadbeef;

    return (unsigned long)pa_kernel_addr;
}

SYSCALL_DEFINE1(pico_setup, int, mem_len) {
    uint32_t base_address;
    uint32_t le_base_address;
    unsigned long mem_address;
    printk("Setting up pico's memory\n");
    mem_address = create_mem_space();
    base_address = (uint32_t)mem_address;
    le_base_address = cpu_to_le32(base_address);
    sunhv_net_write(le_base_address, (void *)PICO_MEM_BASE);

    return mem_address;
}

SYSCALL_DEFINE1(pico_start, uint32_t __user *, start_pc) {
    uint32_t val;
    unsigned long long_pc = (unsigned long)(start_pc);
    printk("Starting pico from address %p\n", start_pc);
    val = sunhv_net_read((void *)start_pc);
    printk("First instruction: %x\n", val);
    hcall_pico_start(cpu_to_le32((uint32_t)long_pc));
    
    printk("Exiting pico syscall\n");
    return 0;
}

SYSCALL_DEFINE1(pico_clear_syscall, unsigned long, mem_region) {
    uint32_t syscall_num;
    uint32_t syscall_arg0, syscall_arg1, syscall_arg2, syscall_arg3;
    uint32_t h_syscall_num, h_syscall_arg0, h_syscall_arg1, h_syscall_arg2, h_syscall_arg3;
    uint32_t syscall_status, val;
    printk("Beginning to service syscalls\n");
    // while true
    while (1) {
        // check the syscall status mem_region
        syscall_status = le32_to_cpu(sunhv_net_read((void *)PICO_SD_STATUS));
        while (syscall_status != 0xdeadbeef) {
            if (syscall_status == 0xffffffff) {
                printk("Got end status\n");
                goto out;
            }
        }

        syscall_num = sunhv_net_read((void *)(PICO_SD_STATUS + 4));
        syscall_arg0 = sunhv_net_read((void *)(PICO_SD_STATUS + 8));
        syscall_arg1 = sunhv_net_read((void *)(PICO_SD_STATUS + 12));
        syscall_arg2 = sunhv_net_read((void *)(PICO_SD_STATUS + 16));
        syscall_arg3 = sunhv_net_read((void *)(PICO_SD_STATUS + 20));

        h_syscall_num = le32_to_cpu(syscall_num);
        h_syscall_arg0 = le32_to_cpu(h_syscall_arg0);
        h_syscall_arg1 = le32_to_cpu(h_syscall_arg1);
        h_syscall_arg2 = le32_to_cpu(h_syscall_arg2);
        h_syscall_arg3 = le32_to_cpu(h_syscall_arg3);

        printk("Got syscall number %x\n", h_syscall_num);

        // syscall would go here
        printk("Clearing address %p\n", (void *)PICO_SD_STATUS);

        val = sunhv_net_read((void *)PICO_SD_STATUS);
        printk("Value before clear: %x\n", val);
        sunhv_net_write(0x0, (void *)PICO_SD_STATUS);
        val = sunhv_net_read((void *)PICO_SD_STATUS);
        printk("Value after clear: %x\n", val);
    }

out: 
    return 0;
}

