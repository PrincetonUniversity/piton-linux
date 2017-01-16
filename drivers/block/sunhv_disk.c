
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/blkdev.h>



#define DRV_NAME     "sunhv_disk"
#define DRV_VERSION  "1.0"
#define DRV_RELDATE  "Mar 20, 2009"

MODULE_VERSION(DRV_VERSION);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("OpenSPARC T1 FPGA sunhv_disk ramdisk device driver");


static char version[] =
        DRV_NAME ":v" DRV_VERSION " " DRV_RELDATE " \n";




extern int sunhv_disk_read(unsigned long disk_offset, unsigned long req_raddr, unsigned long req_size);
extern int sunhv_disk_write(unsigned long disk_offset, unsigned long req_raddr, unsigned long req_size);



#define DKL_NSECT_OFFSET          0x1d0
#define DKL_MAGIC_OFFSET          0x1fc

#define DKL_MAGIC		  0xDABE


#define SUNHV_DISK_NMINORS        1
#define SUNHV_DISK_SECTOR_SIZE    512



static void sunhv_disk_make_request(struct request_queue *queue, struct bio *bio);
static sector_t get_disk_n_sectors(void);



static int sunhv_disk_major = 0;
static char *sunhv_disk_name = "sunhv_disk";

static struct gendisk  *sunhv_gendisk;

struct block_device_operations   sunhv_disk_bdev_ops = {};



static sector_t get_disk_n_sectors(void)
{
    unsigned char first_sector[SUNHV_DISK_SECTOR_SIZE]  __attribute__  ((aligned(8)));
    uint32_t *iptr;
    uint16_t *sptr;
    sector_t n_sectors;
    unsigned long real_addr;
    int error;


    memset(first_sector, 0, SUNHV_DISK_SECTOR_SIZE);

    n_sectors = 0;
    real_addr = virt_to_phys(first_sector);

    error = sunhv_disk_read(0, real_addr, SUNHV_DISK_SECTOR_SIZE);
    if (error < 0) {
	printk(KERN_ERR "%s: Couldn't read first sector because of error %d from sunhv_disk_read \n", DRV_NAME, error);
    } else {
	sptr = (uint16_t *) (first_sector + DKL_MAGIC_OFFSET);
	if (*sptr == DKL_MAGIC) {
	    iptr = (uint32_t *) (first_sector + DKL_NSECT_OFFSET);
	    n_sectors = *iptr;
	} else {
	    printk(KERN_ERR "%s: Didn't find magic number in the first sector. found 0x%x expected 0x%x \n", DRV_NAME, *sptr, DKL_MAGIC);
	}
    }

    return n_sectors;
}


static void sunhv_disk_make_request(struct request_queue *queue, struct bio *bio)
{
    struct bio_vec bvec;
    struct bvec_iter iter;
    int result, total_len, error, direction;
    unsigned long real_addr, disk_offset;


    direction   = bio_data_dir(bio);
    disk_offset = bio->bi_iter.bi_sector * SUNHV_DISK_SECTOR_SIZE;

    total_len = 0;
    result = 0;

    bio_for_each_segment(bvec, bio, iter) {
	real_addr  = (unsigned long) page_to_phys(bvec.bv_page);
	real_addr += bvec.bv_offset;
	if (direction) {
	    error = sunhv_disk_write(disk_offset, real_addr, bvec.bv_len);
	    if (error < 0) {
	        printk(KERN_ERR "%s: sunhv_disk_write failed with error %d \n", DRV_NAME, error);
		result = -EIO;
		break;
	    }
	} else {
	    error = sunhv_disk_read(disk_offset, real_addr, bvec.bv_len);
	    if (error < 0) {
		printk(KERN_ERR "%s: sunhv_disk_read failed with error %d \n", DRV_NAME, error);
		result = -EIO;
		break;
	    }
	}
	disk_offset += bvec.bv_len;
	total_len   += bvec.bv_len;
    }

    bio_endio(bio);//, result);
}


static int sunhv_disk_init(void)
{
    static unsigned int version_printed = 0;
    struct request_queue *queue = NULL;
    int result;
    sector_t n_sectors;


    if (version_printed++ == 0) {
	printk(KERN_INFO  "%s", version);
    }

    n_sectors = get_disk_n_sectors();
    if (n_sectors == 0) {
        return -EIO;
    }

    result = register_blkdev(sunhv_disk_major, sunhv_disk_name);
    if (result <= 0) {
        printk(KERN_ERR "%s: register_blkdev returned error %d \n", DRV_NAME, result);
	return -EIO;
    }
    if (sunhv_disk_major == 0) {
	sunhv_disk_major = result;
    }

    queue = blk_alloc_queue(GFP_KERNEL);
    if (queue == NULL) {
        printk(KERN_ERR "%s: blk_alloc_queue() returned NULL. \n", DRV_NAME);
        goto fail;
    }

    sunhv_gendisk = alloc_disk(SUNHV_DISK_NMINORS);
    if (sunhv_gendisk == NULL) {
        printk(KERN_ERR "%s: alloc_disk() returned NULL. \n", DRV_NAME);
        goto fail;
    }

    blk_queue_make_request(queue, sunhv_disk_make_request);

    sunhv_gendisk->queue = queue;
    sunhv_gendisk->major = sunhv_disk_major;
    sunhv_gendisk->first_minor = 0;
    snprintf(sunhv_gendisk->disk_name, 32, "%s", sunhv_disk_name);
    sunhv_gendisk->fops = &sunhv_disk_bdev_ops;
    set_capacity(sunhv_gendisk, n_sectors);
    add_disk(sunhv_gendisk);

    return 0;


fail:
    unregister_blkdev(sunhv_disk_major, "sunhv_disk");

    if (queue) {
	blk_cleanup_queue(queue);
    }

    return -EIO;
}


static void sunhv_disk_exit(void)
{
    del_gendisk(sunhv_gendisk);
    put_disk(sunhv_gendisk);
    blk_cleanup_queue(sunhv_gendisk->queue);

    unregister_blkdev(sunhv_disk_major, sunhv_disk_name);

    return;
}


module_init(sunhv_disk_init);
module_exit(sunhv_disk_exit);
