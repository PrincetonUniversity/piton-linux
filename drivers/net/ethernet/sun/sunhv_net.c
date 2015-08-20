#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/socket.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_device.h>

#define DRV_NAME     "sunhv_net"
#define DRV_VERSION  "1.0"
#define DRV_RELDATE  "Mar 20, 2009"


MODULE_VERSION(DRV_VERSION);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("OpenSPARC T1 FPGA sunhv_net network device driver");


static char version[] =
        DRV_NAME ":v" DRV_VERSION " " DRV_RELDATE " \n";


static int max_rx_count_per_intr = 8;

module_param(max_rx_count_per_intr, int, 0);
MODULE_PARM_DESC(max_rx_count_per_intr, "Maximum number of packets to be read from the network device per interrupt");


#define SNET_HDR_SIZE         8
#define SNET_ALIGN_BYTES      8


#define SNET_BUFFER_SIZE      (ETH_FRAME_LEN + SNET_HDR_SIZE + SNET_ALIGN_BYTES)

struct sunhv_net {
    struct net_device          *dev;
    spinlock_t                  lock;
    unsigned char               send_buffer[SNET_BUFFER_SIZE] __attribute__((__aligned__(8)));
    unsigned char               recv_buffer[SNET_BUFFER_SIZE] __attribute__((__aligned__(8)));
    uint64_t                    send_buffer_pa;
    uint64_t                    recv_buffer_pa;
    struct net_device_stats     stats;
};


/*
 * The device driver communicates with the snet device through
 * messages. Every message consists of an snet header followed by
 * optional data.
 *
 * struct snet_hdr {
 *       uint32_t  data_size;
 *       uint32_t  cmd;
 * }
 *
 * The hypervisor doesn't interpret the contents of the header
 * or the data. It simply provides a mechanism to send messages
 * to the snet device.
 *
 * For performance reasons, the hypervisor reads/writes double
 * words even if data count is not a multiple of double word.
 *
 */


#define SNET_CMD_START       0x01
#define SNET_CMD_STOP        0x02
#define SNET_CMD_TX          0x04
#define SNET_CMD_RX          0x08
#define SNET_CMD_MAC         0x10

struct snet_hdr {
    uint32_t  data_size;
    uint32_t  cmd;
};


extern int sunhv_net_read(void *pa, size_t size);
extern int sunhv_net_write(void *pa, size_t size);


/*
 * Todo enhancements: hardware checksum calculation and data copy optimization
 */

static int
set_mac_address(struct sunhv_net *snetp)
{
    struct net_device *dev = snetp->dev;
    int i, error, count;
    uint8_t *data_ptr;
    uint64_t datalen;
    struct snet_hdr *shdr;

    count    = 0;
    datalen  = ETH_ALEN;
    shdr     = (struct snet_hdr *) &snetp->send_buffer;
    data_ptr = (uint8_t *) &snetp->send_buffer[SNET_HDR_SIZE];


    shdr->data_size = datalen;
    shdr->cmd       = SNET_CMD_MAC;
    count += SNET_HDR_SIZE;

    for (i=0 ;i<ETH_ALEN; i++) {
        data_ptr[i] = dev->dev_addr[i];
    }
    count += ETH_ALEN;

    error = sunhv_net_write((void *) snetp->send_buffer_pa, count);
    if (error < 0) {
	printk(KERN_ERR "%s: set_mac_address failed with error %d from sunhv_net_write() \n", dev->name, error);
	return -EIO;
    }

    return 0;
}

static struct sk_buff *
sunhv_net_read_pkt(struct sunhv_net *snetp)
{
    struct sk_buff *skb = NULL;
    unsigned char *ptr;
    struct snet_hdr *shdr;
    int error;
    unsigned int pkt_size;

    error = sunhv_net_read((void *) snetp->recv_buffer_pa, SNET_HDR_SIZE);
    if (error < 0) {
	printk(KERN_ERR "%s: sunhv_net_read_pkt failed with error %d from sunhv_net_read \n", snetp->dev->name, error);
	return NULL;
    }

    shdr     = (struct snet_hdr *) &snetp->recv_buffer;
    pkt_size = shdr->data_size;

    if (pkt_size) {
	error = sunhv_net_read((void *) snetp->recv_buffer_pa, pkt_size);
	if (error < 0) {
	    printk(KERN_ERR "%s: sunhv_net_read_pkt failed with error %d from sunhv_net_read \n", snetp->dev->name, error);
	    return NULL;
	}
	if (error < pkt_size) {
	    printk(KERN_ERR "%s: sunhv_net_read_pkt read %d bytes instead of %d \n", snetp->dev->name, error, pkt_size);
	    return NULL;
	}
	skb = dev_alloc_skb(pkt_size);
	if (skb == NULL) {
	    printk(KERN_INFO "%s: dropping packet because dev_alloc_skb() returned NULL \n", snetp->dev->name);
	    snetp->dev->stats.rx_dropped++;
	    return NULL;
	}
	ptr = skb_put(skb, pkt_size);
	memcpy(ptr, snetp->recv_buffer, pkt_size);
    }

    return skb;
}



static irqreturn_t snet_interrupt(int irq, void *dev_id)
{
    struct sunhv_net *snetp = (struct sunhv_net *) dev_id;
    struct sk_buff *skb = NULL;
    int pkt_count = 0;

    spin_lock(&snetp->lock);

    while (pkt_count < max_rx_count_per_intr) {
	skb = sunhv_net_read_pkt(snetp);
	if (skb == NULL) {
	    break;
	}
	skb->dev = snetp->dev;
	skb->protocol = eth_type_trans(skb, snetp->dev);
	skb->ip_summed = CHECKSUM_NONE;
	netif_rx(skb);
	pkt_count++;
	snetp->dev->stats.rx_bytes += skb->len;
	snetp->dev->stats.rx_packets++;
    }


    spin_unlock(&snetp->lock);

    return IRQ_HANDLED;
}


static int sunhv_net_open(struct net_device *dev)
{
    struct sunhv_net *snetp = netdev_priv(dev);
    int error;
    struct snet_hdr *shdr = (struct snet_hdr *) &snetp->send_buffer;

    shdr->data_size = 0;
    shdr->cmd       = SNET_CMD_START;
    error = sunhv_net_write((void *) snetp->send_buffer_pa, SNET_HDR_SIZE);
    if (error < 0) {
	printk(KERN_ERR "%s: sunhv_net_start failed with error %d from sunhv_net_write() \n", dev->name, error);
	return -EIO;
    }

    return 0;
}

static int sunhv_net_stop(struct net_device *dev)
{
    struct sunhv_net *snetp = netdev_priv(dev);
    int error;
    struct snet_hdr *shdr = (struct snet_hdr *) &snetp->send_buffer;

    shdr->data_size = 0;
    shdr->cmd       = SNET_CMD_STOP;
    error = sunhv_net_write((void *) snetp->send_buffer_pa, SNET_HDR_SIZE);
    if (error < 0) {
	printk(KERN_ERR "%s: sunhv_net_stop failed with error %d from sunhv_net_write() \n", dev->name, error);
	return -EIO;
    }

    return 0;
}


static int sunhv_net_set_mac_address(struct net_device *dev, void *p)
{
    struct sockaddr  *addr = p;
    struct sunhv_net *snetp = netdev_priv(dev);

    if (netif_running(dev)) {
	return -EBUSY;
    }
    memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);

    return set_mac_address(snetp);
}

static int sunhv_net_tx(struct sk_buff *skb, struct net_device *dev)
{
    struct sunhv_net  *snetp = netdev_priv(dev);
    struct snet_hdr *shdr;
    unsigned char *pkt;
    int pkt_len, error;


    pkt     = (unsigned char *) skb->data;
    pkt_len = skb->len;
    shdr    = (struct snet_hdr *) &snetp->send_buffer;

    spin_lock_irq(&snetp->lock);

    memcpy(&snetp->send_buffer[SNET_HDR_SIZE], pkt, pkt_len);

    shdr->data_size = pkt_len;
    shdr->cmd       = SNET_CMD_TX;

    error = sunhv_net_write((void *) snetp->send_buffer_pa, (pkt_len+SNET_HDR_SIZE));
    if (error < 0) {
	printk(KERN_ERR "%s: sunhv_net_tx failed with error %d from sunhv_net_write \n", dev->name, error);
	error = -EIO;
    } else {
        error = NETDEV_TX_OK;
	dev->stats.tx_packets++;
	dev->stats.tx_bytes += pkt_len;
    }

    spin_unlock_irq(&snetp->lock);

    dev_kfree_skb(skb);

    return error;
}

/*static struct net_device_stats *sunhv_net_stats(struct net_device *dev)
{
    struct sunhv_net *snetp = netdev_priv(dev);

    return &snetp->stats;
}*/

static const struct net_device_ops sunhv_net_ops = {
	.ndo_open				= sunhv_net_open,
	.ndo_stop				= sunhv_net_stop,
	.ndo_set_mac_address	= sunhv_net_set_mac_address,
	/*.ndo_get_stats			= sunhv_net_stats,*/ /* Use default */
	.ndo_start_xmit			= sunhv_net_tx,
};

static void sunhv_net_init(struct net_device *dev)
{
    struct sunhv_net *snetp = netdev_priv(dev);

    spin_lock_init(&snetp->lock);

    dev->flags = dev->flags & ~IFF_MULTICAST;
	dev->netdev_ops = &sunhv_net_ops;

    snetp->send_buffer_pa = virt_to_phys(&snetp->send_buffer);
    snetp->recv_buffer_pa = virt_to_phys(&snetp->recv_buffer);

    return;
}

static int sunhv_net_probe(struct platform_device *of_dev)
{
    static unsigned int version_printed = 0;
    struct net_device  *dev;
    struct sunhv_net   *snetp;
    int err;


    if (version_printed++ == 0) {
	printk(KERN_INFO  "%s", version);
    }

    dev = alloc_etherdev(sizeof(struct sunhv_net));
    if (dev == NULL) {
        printk(KERN_ERR "%s: alloc_etherdev() returned NULL \n", DRV_NAME);
        return -ENOMEM;
    }

    snetp = netdev_priv(dev);
    snetp->dev = dev;
    sunhv_net_init(dev);

    err = register_netdev(dev);
    if (err) {
        printk(KERN_ERR "%s: register_netdev() returned error %d. \n", dev->name, err);
	return err;
    }

    dev_set_drvdata(&of_dev->dev, snetp);
    request_irq(of_dev->archdata.irqs[0], snet_interrupt, 0, dev->name, snetp);

    printk(KERN_INFO "%s: sunhv_net_probe succeeded \n", dev->name);
    return 0;
}

static int sunhv_net_remove(struct platform_device *of_dev)
{
    struct sunhv_net  *snetp = dev_get_drvdata(&of_dev->dev);
    struct net_device *dev   = snetp->dev;

    unregister_netdev(dev);
    free_netdev(dev);
    dev_set_drvdata(&of_dev->dev, NULL);

    return 0;
}


static struct of_device_id sunhv_net_match[] = {
	{
		.name = "snet",
	},
	{},
};
MODULE_DEVICE_TABLE(of, sunhv_net_match);

static struct platform_driver sunhv_net_driver = {
	.driver = {
		.name		= "sunhv_net",
		.owner		= THIS_MODULE,
		.of_match_table	= sunhv_net_match,
	},
	.probe		= sunhv_net_probe,
	.remove		= sunhv_net_remove,
};


static int __init sunhv_net_module_init(void)
{
    int err;

    if (tlb_type != hypervisor) {
	return -ENODEV;
    }

    err = platform_driver_register(&sunhv_net_driver);
    if (err != 0) {
        printk(KERN_ERR "%s: of_register_driver returned error %d \n", DRV_NAME, err);
	return err;
    }

    return 0;
}


static void __exit sunhv_net_module_exit(void)
{
    platform_driver_unregister(&sunhv_net_driver);

    return;
}

module_init(sunhv_net_module_init);
module_exit(sunhv_net_module_exit);
