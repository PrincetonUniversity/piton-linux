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
#include <linux/execdrafting_temp>

#define NUMBER_OF_BUCKETS 1000

/* the entry used for the hash table */ 
struct hash_table_entry {
	struct task_struct *current_task;
	struct hash_table_entry  *next;
	struct hash_table_entry  *prev;
};

struct hash_table_roots {
	struct hash_table_entry *next;
};

/* the hash table; probably need to initialize this when the computer gets started */
extern struct hash_table_entry hash_table[NUMBER_OF_BUCKETS];

void init_hash_table_entrries(void);
int calculate_hash(char *file);
int add_tohash_table(struct task_struct *p);
int remove_fromHash_table(struct task_struct *p);
int delete_hash_entry(struct task_struct *p);
struct task_struct *find_similar_task(struct task_struct *p);


#endif










