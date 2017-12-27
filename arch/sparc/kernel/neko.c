#include <asm/hypervisor.h>
#include <linux/syscalls.h>

#define NEKO_CMD_ADDR 0xfff0e00000
#define NEKO_BASE_LDS (NEKO_CMD_ADDR + 16)
#define NEKO_PC_START (NEKO_CMD_ADDR + 24)
#define NEKO_INSTR_ADDR (NEKO_CMD_ADDR + 28)
#define NEKO_INSTR_VALUE (NEKO_CMD_ADDR + 32)
#define NEKO_RESET (NEKO_CMD_ADDR + 36)
#define NEKO_GPR_CMD (NEKO_CMD_ADDR + 40)
#define NEKO_SGRP_ADDR (NEKO_CMD_ADDR + 772)
#define NEKO_SGRP_QUAD_0 (NEKO_CMD_ADDR + 776)
#define NEKO_SGRP_QUAD_1 (NEKO_CMD_ADDR + 780)
#define NEKO_SGRP_QUAD_2 (NEKO_CMD_ADDR + 784)
#define NEKO_SGRP_QUAD_3 (NEKO_CMD_ADDR + 788)

#define NEKO_MEM_OP (NEKO_CMD_ADDR + 256)
#define NEKO_MEM_RD_DATA (NEKO_CMD_ADDR + 260) // Address for data to be read from MIAOW and written to memory
#define NEKO_MEM_ADDR (NEKO_CMD_ADDR + 264)
//#define NEKO_MEM_WR_DATA (NEKO_CMD_ADDR + 192) // Address for writing data to MIAOW
//#define NEKO_MEM_WR_EN (NEKO_CMD_ADDR + 196)
#define NEKO_MEM_ACK (NEKO_CMD_ADDR + 260)
#define NEKO_MEM_DONE (NEKO_CMD_ADDR + 264)

//#define NEKO_CYCLE_COUNTER (NEKO_CMD_ADDR + 192)

#define MEM_WR_ACK_WAIT 1
#define MEM_WR_RDY_WAIT 2
#define MEM_WR_LSU_WAIT 3
#define MEM_RD_ACK_WAIT 4
#define MEM_RD_RDY_WAIT 5
#define MEM_RD_LSU_WAIT 6

#define IDLE_STATE 0
#define LD_INSTR_STATE 1

#define RESET_INSTR_CNT 2
#define EXEC_KERNEL_STATE 3
#define CHECK_EXEC_STATUS 4
#define READ_PC_CNT 5

#define LD_MEM_DATA 6
#define RD_MEM_DATA 7
#define RESET_MEM_CNT 8
#define MEM_CNT_INC 9

#define MEM_WR_ADDR_OUT 20
#define KERNEL_DONE_OUT 50

SYSCALL_DEFINE4(neko_exec, uint32_t __user *, ins_mem, uint32_t __user *, data_mem, unsigned long long, ins_mem_len, unsigned long long, data_mem_len) {

    int instrAddrCnt = 0;
    u32 readData;
    int index;
    uint32_t ins_buf[ins_mem_len];
    uint32_t data_buf[data_mem_len];

    //printk("Entered neko syscall\n");
    copy_from_user(ins_buf, ins_mem, ins_mem_len*sizeof(uint32_t));
    copy_from_user(data_buf, data_mem, data_mem_len*sizeof(uint32_t));

    //reset
    sunhv_net_write(0, (void *)NEKO_RESET);
    sunhv_net_write(1, (void *)NEKO_RESET);    
    sunhv_net_write(0, (void *)NEKO_RESET);

    sunhv_net_write(0, (void *)NEKO_BASE_LDS);

    // First we write the instructions into neko's instruction memory
    for (index = 0; index < ins_mem_len; index++) {
        sunhv_net_write(instrAddrCnt, (void *)NEKO_INSTR_ADDR);
        //printk("Writing instruction %x\n", ins_buf[index]);
        sunhv_net_write(ins_buf[index], (void *)NEKO_INSTR_VALUE);
        instrAddrCnt += 1;
    }
    
    // We don't need to do any writing of the data memory because we have the array
    //for (index = 0; index < data_mem_len; index++) {
    //    sunhv_net_write(NEKO_INSTR_ADDR, instrAddrCnt);
    //    sunhv_net_write(NEKO_INSTR_VALUE, data_mem[index]);
    //}

    printk("Starting miaow\n");
    sunhv_net_write(1, (void *)NEKO_CMD_ADDR);

    while (1) {
        readData = sunhv_net_read((void *)NEKO_MEM_OP);

        if (readData == MEM_WR_ACK_WAIT || readData == MEM_RD_ACK_WAIT) {
            int nextValue = (readData == MEM_RD_ACK_WAIT) ? MEM_RD_RDY_WAIT : MEM_WR_RDY_WAIT;
            int address, index_address;
            //printk("Got mem op %d\n", readData);

            address = sunhv_net_read((void *)NEKO_MEM_ADDR);
            index_address = address >> 2;
            
            sunhv_net_write(0, (void *)NEKO_MEM_ACK);
            sunhv_net_write(1, (void *)NEKO_MEM_ACK);
            sunhv_net_write(0, (void *)NEKO_MEM_ACK);

            do {
                readData = sunhv_net_read((void *)NEKO_MEM_OP);
            } while(readData != nextValue);


            if (nextValue == MEM_RD_RDY_WAIT)
            {
                //printk("Address: %x, index_address: %d, data: %x\n", address, index_address, data_buf[index_address]);
                sunhv_net_write(data_buf[index_address], (void *)NEKO_MEM_ACK);
                nextValue = MEM_RD_LSU_WAIT;
            }
            else
            {
                readData = sunhv_net_read((void *)NEKO_MEM_RD_DATA);
                //printk("Address: %x, index_address: %d, data: %x\n", address, index_address, readData);
                data_buf[index_address] = readData;
                nextValue = MEM_WR_LSU_WAIT;
            }

            sunhv_net_write(0, (void *)NEKO_MEM_DONE);
            sunhv_net_write(1, (void *)NEKO_MEM_DONE);
            sunhv_net_write(0, (void *)NEKO_MEM_DONE);
        }

        if(sunhv_net_read((void *)NEKO_CMD_ADDR) == 1)
        {
			uint32_t sgrp_data[4];
            printk("Finished execution\n");
            for (index = 0; index < 16; index += 4) {
                sunhv_net_write(index, (void *)NEKO_SGRP_ADDR);
				sgrp_data[0] = sunhv_net_read((void *)NEKO_SGRP_QUAD_0);
      			sgrp_data[1] = sunhv_net_read((void *)NEKO_SGRP_QUAD_1);
      			sgrp_data[2] = sunhv_net_read((void *)NEKO_SGRP_QUAD_2);
      			sgrp_data[3] = sunhv_net_read((void *)NEKO_SGRP_QUAD_3);
				printk("s%d: %x, s%d: %x, s%d: %x, s%d: %x\n", index, sgrp_data[0],
							index + 1, sgrp_data[1], index + 2, sgrp_data[2],
							index + 3, sgrp_data[3]);
            }

            return 0;
        }
    }
}
