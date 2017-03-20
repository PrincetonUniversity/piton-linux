/* 
 *
 * Code for making a hash of the text section of a elf file 
 *
 * 
 */

// include the relevant files
#include <linux/file.h>
#include <linux/sched.h>
#include <linux/crypto.h>
#include <linux/sched.h>

// the entry used for the hash table 
struct hash_table_entry {
	struct task_struct *current_task;
	struct hash_table_entry *next;
	struct hash_table_entry *prev;
}

struct hash_table_roots {
	struct hash_table_entry *next;
}

#define NUMBER_OF_BUCKETS 1000

static void init_hash_table_entrries(void);
static int calculate_hash(struct file *file);
static int add_tohash_table(struct task_struct *p);
static int remove_fromHash_table(struct task_struct *p);
static int delete_hash_entry(struct task_struct *p);
static struct task_struct *find_similar_task(struct task_struct *p);













