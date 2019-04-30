// See LICENSE for license details.

#ifndef ETH_HEADER_H
#define ETH_HEADER_H

/* Register offsets for the LowRISC Ethernet Core */

/* Register offsets (in bytes) for the LowRISC Core */
#define TXBUFF_OFFSET       0x1000          /* Transmit Buffer */

#define MACLO_OFFSET        0x0800          /* MAC address low 32-bits */
#define MACHI_OFFSET        0x0808          /* MAC address high 16-bits and MAC ctrl */
#define TPLR_OFFSET         0x0810          /* Tx packet length */
#define TFCS_OFFSET         0x0818          /* Tx frame check sequence register */
#define MDIOCTRL_OFFSET     0x0820          /* MDIO Control Register */
#define RFCS_OFFSET         0x0828          /* Rx frame check sequence register(read) and last register(write) */
#define RSR_OFFSET          0x0830          /* Rx status and reset register */
#define RBAD_OFFSET         0x0838          /* Rx bad frame and bad fcs register arrays */
#define RPLR_OFFSET         0x0840          /* Rx packet length register array */

#define RXBUFF_OFFSET       0x4000          /* Receive Buffer */

/* MAC Ctrl Register (MACHI) Bit Masks */
#define MACHI_MACADDR_MASK    0x0000FFFF     /* MAC high 16-bits mask */
#define MACHI_COOKED_MASK     0x00010000     /* obsolete flag */
#define MACHI_LOOPBACK_MASK   0x00020000     /* Rx loopback packets */
#define MACHI_ALLPKTS_MASK    0x00400000     /* Rx all packets (promiscuous mode) */
#define MACHI_IRQ_EN          0x00800000     /* Rx packet interrupt enable */

/* MDIO Control Register Bit Masks */
#define MDIOCTRL_MDIOCLK_MASK 0x00000001    /* MDIO Clock Mask */
#define MDIOCTRL_MDIOOUT_MASK 0x00000002    /* MDIO Output Mask */
#define MDIOCTRL_MDIOOEN_MASK 0x00000004    /* MDIO Output Enable Mask, 3-state enable, high=input, low=output */
#define MDIOCTRL_MDIORST_MASK 0x00000008    /* MDIO Input Mask */
#define MDIOCTRL_MDIOIN_MASK  0x00000008    /* MDIO Input Mask */

/* Transmit Status Register (TPLR) Bit Masks */
#define TPLR_FRAME_ADDR_MASK  0x0FFF0000     /* Tx frame address */
#define TPLR_PACKET_LEN_MASK  0x00000FFF     /* Tx packet length */
#define TPLR_BUSY_MASK        0x80000000     /* Tx busy mask */

/* Receive Status Register (RSR) */
#define RSR_RECV_FIRST_MASK   0x0000000F      /* first available buffer (static) */
#define RSR_RECV_NEXT_MASK    0x000000F0      /* current rx buffer (volatile) */
#define RSR_RECV_LAST_MASK    0x00000F00      /* last available rx buffer (static) */
#define RSR_RECV_DONE_MASK    0x00001000      /* Rx complete */
#define RSR_RECV_IRQ_MASK     0x00002000      /* Rx irq bit */

/* General Ethernet Definitions */
#define HEADER_OFFSET               12      /* Offset to length field */
#define HEADER_SHIFT                16      /* Shift value for length */
#define ARP_PACKET_SIZE             28      /* Max ARP packet size */
#define HEADER_IP_LENGTH_OFFSET     16      /* IP Length Offset */

#endif
