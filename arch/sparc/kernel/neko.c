#include <asm/hypervisor.h>
#include <linux/syscalls.h>

#define NEKO_CMD_ADDR 0xfff0e00000
#define NEKO_BASE_LDS (NEKO_CMD_ADDR + 16)
#define NEKO_INSTR_ADDR (NEKO_CMD_ADDR + 28)
#define NEKO_INSTR_VALUE (NEKO_CMD_ADDR + 32)
#define NEKO_GPR_CMD (NEKO_CMD_ADDR + 40)
#define NEKO_SGRP_ADDR (NEKO_CMD_ADDR + 44)
#define NEKO_SGRP_QUAD_0 (NEKO_CMD_ADDR + 48)
#define NEKO_SGRP_QUAD_1 (NEKO_CMD_ADDR + 52)
#define NEKO_SGRP_QUAD_2 (NEKO_CMD_ADDR + 56)
#define NEKO_SGRP_QUAD_3 (NEKO_CMD_ADDR + 60)

#define NEKO_MEM_OP (NEKO_CMD_ADDR + 128)
#define NEKO_MEM_RD_DATA (NEKO_CMD_ADDR + 132) // Address for data to be read from MIAOW and written to memory
#define NEKO_MEM_ADDR (NEKO_CMD_ADDR + 136)
#define NEKO_MEM_WR_DATA (NEKO_CMD_ADDR + 192) // Address for writing data to MIAOW
#define NEKO_MEM_WR_EN (NEKO_CMD_ADDR + 196)
#define NEKO_MEM_ACK (NEKO_CMD_ADDR + 200)
#define NEKO_MEM_DONE (NEKO_CMD_ADDR + 204)

#define NEKO_CYCLE_COUNTER (NEKO_CMD_ADDR + 192)

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

    copy_from_user(ins_buf, ins_mem, data_mem_len*sizeof(uint32_t));

    sunhv_net_write(0, (void *)NEKO_BASE_LDS);

    // First we write the instructions into neko's instruction memory
    for (index = 0; index < ins_mem_len; index++) {
        sunhv_net_write(instrAddrCnt, (void *)NEKO_INSTR_ADDR);
        sunhv_net_write(ins_buf[index], (void *)NEKO_INSTR_VALUE);
        instrAddrCnt += 1;
    }

    // We don't need to do any writing of the data memory because we have the array
    //for (index = 0; index < data_mem_len; index++) {
    //    sunhv_net_write(NEKO_INSTR_ADDR, instrAddrCnt);
    //    sunhv_net_write(NEKO_INSTR_VALUE, data_mem[index]);
    //}

    sunhv_net_write(1, (void *)NEKO_CMD_ADDR);

    while (1) {
        readData = sunhv_net_read((void *)NEKO_MEM_OP);

        if (readData == MEM_WR_ACK_WAIT || readData == MEM_RD_ACK_WAIT) {
            int nextValue = (readData == MEM_RD_ACK_WAIT) ? MEM_RD_RDY_WAIT : MEM_WR_RDY_WAIT;
            int address;

            address = sunhv_net_read((void *)NEKO_MEM_ADDR);

            sunhv_net_write(0, (void *)NEKO_MEM_ACK);
            sunhv_net_write(1, (void *)NEKO_MEM_ACK);
            sunhv_net_write(0, (void *)NEKO_MEM_ACK);

            do {
                readData = sunhv_net_read((void *)NEKO_MEM_OP);
            } while(readData != nextValue);


            if (nextValue == MEM_RD_RDY_WAIT)
            {
                sunhv_net_write(data_mem[address], (void *)NEKO_MEM_WR_DATA);
                sunhv_net_write(0, (void *)NEKO_MEM_WR_EN);
                sunhv_net_write(1, (void *)NEKO_MEM_WR_EN);
                sunhv_net_write(1, (void *)NEKO_MEM_WR_EN);
                nextValue = MEM_RD_LSU_WAIT;
            }
            else
            {
                readData = sunhv_net_read((void *)NEKO_MEM_RD_DATA);
                data_mem[address] = readData;
                nextValue = MEM_WR_LSU_WAIT;
            }

            sunhv_net_write(0, (void *)NEKO_MEM_DONE);
            sunhv_net_write(1, (void *)NEKO_MEM_DONE);
            sunhv_net_write(0, (void *)NEKO_MEM_DONE);
        }

        if(sunhv_net_read((void *)NEKO_CMD_ADDR) == 1)
        {
            readData = sunhv_net_read((void *)NEKO_CYCLE_COUNTER);
            return readData;
        }
    }
}
