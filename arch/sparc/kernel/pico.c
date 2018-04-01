#include <linux/kernel.h>
#include <linux/syscalls.h>
#include <asm/hypervisor.h>
#include <asm/byteorder.h>
#include <linux/slab.h>

#define PICO_MEM_START 0x3f000000
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
    // maybe should be get_free_pages if allocating larger amounts of memory
    uint32_t *addr = (uint32_t *)__get_free_page(GFP_KERNEL);
    uint32_t *pa_kernel_addr;
    printk("Kernel virtual address: %p\n", addr);
    pa_kernel_addr = (uint32_t *)virt_to_phys(addr);
    printk("Kernel physical address: %p\n", pa_kernel_addr);
    *addr = 0xdeadbeef;

    return (unsigned long)pa_kernel_addr;
}

SYSCALL_DEFINE1(pico_setup, int, mem_len) {
    unsigned long mem_address;
    uint32_t le_mem_address;
    printk("Setting up pico's memory\n");
    mem_address = create_mem_space();

    le_mem_address = cpu_to_le32((uint32_t)mem_address);
    printk("Writing little endian address: 0x%08x\n", le_mem_address);
    sunhv_net_write(le_mem_address, (void *)PICO_MEM_START);

    return mem_address;
}

SYSCALL_DEFINE2(pico_start, uint32_t __user *, mem, int, mem_len) {
    uint32_t val;
    printk("Starting pico from address %p\n", mem);
    val = sunhv_net_read((void *)mem);
    printk("First instruction: %x\n", val);
    hcall_pico_start((unsigned long)mem);
    
    printk("Exiting pico syscall\n");
    return 0;
}

