/* 
 *
 * Code for making a hash of the text section of a elf file 
 *
 * 
 */

#ifndef _LINUX_EXECD_H
#define _LINUX_EXECD_H

/* include the relevant files */
#include <linux/file.h>
#include <linux/sched.h>

#define NUMBER_OF_BUCKETS 1000

typedef enum {
	FULL_HASH, 
	FIRST_HALF_HASH, 
	SECOND_HALF_HASH,
	FIRST_QUARTER_HASH,
	SECOND_QUARTER_HASH,
	THIRD_QUARTER_HASH,
	FOURTH_QUARTER_HASH,

	ON_LIST,
	NOT_ON_LIST
} hash_IDs;

/* the entry used for the hash table */ 
struct hash_table_entry {
	struct task_struct *current_task;
	struct hash_table_entry  *next;
	struct hash_table_entry  *prev;
	hash_IDs hash_number;
};

struct hash_table_roots {
	struct hash_table_entry *next;
};

/* the hash table; probably need to initialize this when the computer gets started */
extern struct hash_table_entry hash_table[NUMBER_OF_BUCKETS];

int enable_execd(void);
int disable_execd(void);
int check_execd_register(void);

void init_hash_table_entries(void);
int calculate_hash(struct task_struct *p);
int add_tohashes_table(struct task_struct *p);
int remove_fromHash_table(struct task_struct *p);
int delete_hash_entry(struct task_struct *p);
struct task_struct *find_similar_task(struct task_struct *p);


#endif










