/* 
 *
 * Code for making a hash of the text section of a elf file 
 *
 * 
 */

// include the relevant files
#include <linux/fs.h>
#include <linux/execdrafting.h>
#include <linux/elf.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/hash.h>
#include <linux/types.h>
#include <crypto/hash.h>
#include <crypto/md5.h>
#include <linux/module.h> 
#include <asm/uaccess.h> 
#include <linux/mm.h> 

#include <linux/init.h> 
#include <linux/syscalls.h> 
#include <linux/fcntl.h> 
#include <linux/smp.h>


/* to remove later */ 
#include <linux/printk.h>

struct hash_table_entry hash_table[NUMBER_OF_BUCKETS];

/* called when the computer is switched on */
void init_hash_table_entries(void) {
	int i;
	for (i = 0; i < NUMBER_OF_BUCKETS; i++)  {
		hash_table[i].next = NULL;
		hash_table[i].prev = NULL;
		hash_table[i].current_task = NULL;
	}
}

/* convert the hash to an int and find the bucket for the hash */
int get_hash_bucket(u8 *hash) {

	unsigned long res;
	int bucket;
	kstrtoul((const char *) hash, 10 /* base */, &res);

	bucket = res % NUMBER_OF_BUCKETS;
	if (bucket < 0) 
		bucket *= -1;

	return bucket; 
}

/* this function add the process to the hash table based on the 
 process' hash value */
int add_tohashes_table(struct task_struct *p) {
 
 	int hash_int;
 	int i;
 	u8 *current_hash;
 	hash_IDs hash_number;
 	struct hash_table_entry *hash_entry;

 	struct hash_table_entry *new_entry;
 	if (p == NULL) return -1;

 	printk("The processor number %d\n", smp_processor_id());

 	/*calculate the hash */
	for (i = 0; i < 7; i++) {

		if (i == 0) {
			current_hash = (u8 *)&p->execd_hash;
			hash_number = FULL_HASH;
			hash_entry = (struct hash_table_entry *)p->execd_hash_entry;
		}
		else if (i == 1) {
			current_hash = (u8 *)&p->execd_half_1_hash;
			hash_number = FIRST_HALF_HASH;
			hash_entry = (struct hash_table_entry *)p->execd_half_1_hash_entry;
		}
		else if (i == 2) {
			current_hash = (u8 *)&p->execd_half_2_hash;
			hash_number = SECOND_HALF_HASH;
			hash_entry = (struct hash_table_entry *)p->execd_half_2_hash_entry;
		}
		else if (i == 3) {
			current_hash = (u8 *)&p->execd_quarter_1_hash;
			hash_number = FIRST_QUARTER_HASH;
			hash_entry = (struct hash_table_entry *)p->execd_quarter_1_hash_entry;
		}
		else if (i == 4) {
			current_hash = (u8 *)&p->execd_quarter_2_hash;
			hash_number = SECOND_QUARTER_HASH;
			hash_entry = (struct hash_table_entry *)p->execd_quarter_2_hash_entry;
		}
		else if (i == 5) {
			current_hash = (u8 *)&p->execd_quarter_3_hash;
			hash_number = THIRD_QUARTER_HASH;
			hash_entry = (struct hash_table_entry *)p->execd_quarter_2_hash_entry;
		}
		else if (i == 6) {
			current_hash = (u8 *)&p->execd_quarter_4_hash;
			hash_number = FOURTH_QUARTER_HASH;
			hash_entry = (struct hash_table_entry *)p->execd_quarter_4_hash_entry;
		}

		hash_int = get_hash_bucket(current_hash);
 		if (hash_int < 0) return -1;

 		new_entry = (struct hash_table_entry *) kzalloc((size_t)sizeof(struct hash_table_entry), GFP_KERNEL);
 		if (new_entry == NULL) return -1;
 		if (hash_table == NULL) return -1;

 		/* when adding for the first time */
 		if (hash_table[hash_int].next != NULL) 
 			hash_table[hash_int].next->prev = new_entry;

 		new_entry->current_task =  p;
 		new_entry->hash_number =  hash_number;

 		new_entry->next = hash_table[hash_int].next;
 		new_entry->prev = &hash_table[hash_int];
 		hash_table[hash_int].next = new_entry;
 		hash_entry = new_entry;
 	} 

 	return 0; 
 }

 /* finds and returns a task that is similar to task referenced by p */
struct task_struct *find_similar_task(struct task_struct *p) {

 	int hash_int;
 	struct hash_table_entry *temp;
 	int i, highest_score;
 	u8 *current_hash;
 	struct task_struct *list;
 	struct task_struct *matched_task;

 	if (p == NULL) return NULL;
 	if (p->execd_hash == NULL) return NULL;

 	list = NULL;
 	matched_task = NULL;

 	for (i = 0; i < 7; i++) {

 		if (i == 0) {
			current_hash = (u8 *)&p->execd_hash;
		}
		else if (i == 1) {
			current_hash = (u8 *)&p->execd_half_1_hash;
		}
		else if (i == 2) {
			current_hash = (u8 *)&p->execd_half_2_hash;
		}
		else if (i == 3) {
			current_hash = (u8 *)&p->execd_quarter_1_hash;
		}
		else if (i == 4) {
			current_hash = (u8 *)&p->execd_quarter_2_hash;
		}
		else if (i == 5) {
			current_hash = (u8 *)&p->execd_quarter_3_hash;
		}
		else if (i == 6) {
			current_hash = (u8 *)&p->execd_quarter_4_hash;
		}

 		hash_int = get_hash_bucket(current_hash);

 		temp = hash_table[hash_int].next;

 		while (temp != NULL) {
 			if (strncmp((const char *)temp->current_task->execd_hash, (const char *)p->execd_hash, 32) == 0) {
 				if (temp->hash_number == FULL_HASH)
 					return temp->current_task;

 				else if ((temp->hash_number == FIRST_HALF_HASH) || (temp->hash_number == SECOND_HALF_HASH))
 					temp->current_task->matching_score += 2;

 				else
 					temp->current_task->matching_score += 1;
 			}

 			temp = temp->next;

 			if (temp->current_task->on_list == (int) NOT_ON_LIST) {
 				temp->current_task->on_list = (int) ON_LIST;
				temp->current_task->next_on_similarity_list = list;
				list = temp->current_task;
 			}	
 		} 
 	}

 	highest_score = 0;
 	while (list != NULL) {
 		if (list->matching_score > highest_score) 
 			matched_task = list;
 		
 		list = list->next_on_similarity_list;

 		matched_task->matching_score = 0;
 		matched_task->next_on_similarity_list = NULL;
 	}

 	return matched_task; 
 }

/* this function calculates the hashes associated with the text section of the code */
int calculate_hash(struct task_struct *p) {

	Elf64_Ehdr *ehdr; 
 	Elf64_Shdr *sectionHeader;
 	Elf64_Shdr *sh_strtab;
 	uint64_t sectionSize, sectionOffset;
 	loff_t position;  
 	struct file *file;
	struct shash_desc desc;
	char *buffer;
	char *section_names; 
	mm_segment_t oldfs;
	int i;
	u8 *current_hash;
	const u8 * data;
	unsigned int len;

 	/* read the header */
 	if (p == NULL) return -1;
 	if (p->filename == NULL) return -1;
 	position = 0;
 	oldfs = get_fs();
 	set_fs(KERNEL_DS);
 	file = NULL;
 	ehdr = NULL;

 	file = filp_open(p->filename,O_RDONLY,0);
 	ehdr = kzalloc((size_t)sizeof(Elf64_Ehdr), GFP_KERNEL);

 	if ((ehdr == NULL) || (file == NULL)) return -1;
 	vfs_read(file, (char *) ehdr, sizeof(Elf64_Ehdr), &position);  	
	
	position = ehdr->e_shoff;
	buffer = kzalloc((size_t)(ehdr->e_shnum * ehdr->e_shentsize), GFP_KERNEL);
	if (buffer == NULL) return -1; 
	vfs_read(file, buffer, ehdr->e_shnum * ehdr->e_shentsize, &position);

	/* get the section with the names, this is the section with the section headers */
	sh_strtab = (Elf64_Shdr*)&buffer[(ehdr->e_shstrndx) * sizeof(Elf64_Shdr)];
	position = sh_strtab->sh_offset;
	section_names = kzalloc((size_t)(sh_strtab->sh_size), GFP_KERNEL);
	vfs_read(file, section_names, sh_strtab->sh_size, &position);

	for (i = 0; i < ehdr->e_shnum; i++) {
		position = i * sizeof(Elf64_Shdr);
		sectionHeader = (Elf64_Shdr*)&buffer[position];
		if(!strcmp((const char *)&section_names[sectionHeader->sh_name], ".text")) {
			position = sectionHeader->sh_offset;
			sectionSize = sectionHeader->sh_size;
			break;
		} 
		sectionHeader = NULL; 
	}

	if (sectionHeader == NULL) return -1;  

	/* read the section if we found it and compute the has of the text section */
	kfree((const void *) buffer);
	buffer = kzalloc((size_t)sectionSize, GFP_KERNEL);
	if (buffer == NULL) return -1; 
	vfs_read(file, buffer, sectionSize, &position); 

	/*calculate the hashes */
	for (i = 0; i < 7; i++) {

		/* get the right pointer for the beginning of the buffer the the size of the data */
		if (i == 0) {
			current_hash = (u8 *) &current->execd_hash;
			len = sectionSize;
			data = (const u8 *) &buffer;
		}
		else if (i == 1) {
			current_hash = (u8 *)&current->execd_half_1_hash;
			len = sectionSize / 2;
			data = (const u8 *) &buffer[0];
			printk("execd_half_1_hash %s\n", buffer[len]);
		}
		else if (i == 2) {
			current_hash = (u8 *)&current->execd_half_2_hash;
			len = (sectionSize / 2) + (sectionSize % 2);
			data = (const u8 *) &buffer[len];
			printk("execd_half_2_hash %s\n", buffer[len]);
		}
		else if (i == 3) {
			current_hash = (u8 *)&current->execd_quarter_1_hash;
			len = sectionSize / 4;
			data = (const u8 *) &buffer[0];
			printk("execd_quarter_1_hash %s\n", buffer[len]);
		}
		else if (i == 4) {
			current_hash = (u8 *)&current->execd_quarter_2_hash;
			len = sectionSize / 4;
			data = (const u8 *) &buffer[len];
			printk("execd_quarter_2_hash %s\n", buffer[len]);
		}
		else if (i == 5) {
			current_hash = (u8 *)&current->execd_quarter_3_hash;
			len = sectionSize / 4;
			data = (const u8 *) &buffer[2 * len];
			printk("execd_quarter_3_hash %s\n", buffer[len]);
		}
		else if (i == 6) {
			current_hash = (u8 *)&current->execd_quarter_4_hash;
			len = (sectionSize / 2) + (sectionSize % 4);
			data = (const u8 *) &buffer[3 * len];
			printk("execd_quarter_4_hash %s\n", buffer[len]);
		}

		/* calculate the has and free it to be used by the other calls */
		desc.tfm = crypto_alloc_shash("md5", CRYPTO_ALG_TYPE_SHASH, CRYPTO_ALG_ASYNC);
		crypto_shash_init(&desc);
		crypto_shash_finup(&desc, (const u8 *)data, (unsigned int) len, current_hash);
		crypto_free_shash(desc.tfm);

		printk("Hash bucket %d\n", get_hash_bucket(current_hash));
	}

	/* initialize the matching score for this process */
	p->matching_score = 0;
	p->on_list = NOT_ON_LIST;
	
	/*free allocations */
	kfree((const void *) buffer); 
	kfree((const void *) ehdr);
	kfree((const void *) section_names);

	add_tohashes_table(p);
	find_similar_task(p);
	remove_fromHash_table(p);

	/*resteore the file system */
	filp_close(file, NULL);
 	set_fs(oldfs); 
	return 0;
 }

/* this function removes the process from the hash table */
int remove_fromHash_table(struct task_struct *p) {

 	struct hash_table_entry *hash_entry;
 	int i;

 	if (p == NULL) return -1;

 	/* get the hashes and remove them from the lists */
	for (i = 0; i < 7; i++) {

		if (i == 0) 	 hash_entry = (struct hash_table_entry *)p->execd_hash_entry;
		else if (i == 1) hash_entry = (struct hash_table_entry *)p->execd_half_1_hash_entry;
		else if (i == 2) hash_entry = (struct hash_table_entry *)p->execd_half_2_hash_entry;
		else if (i == 3) hash_entry = (struct hash_table_entry *)p->execd_quarter_1_hash_entry;
		else if (i == 4) hash_entry = (struct hash_table_entry *)p->execd_quarter_2_hash_entry;
		else if (i == 5) hash_entry = (struct hash_table_entry *)p->execd_quarter_3_hash_entry;
		else if (i == 6) hash_entry = (struct hash_table_entry *)p->execd_quarter_4_hash_entry;

		if (hash_entry == NULL) return -1;

 		if (hash_entry->prev != NULL) hash_entry->prev->next = hash_entry->next;
 		if (hash_entry->next != NULL) hash_entry->next->prev = hash_entry->prev;
 		hash_entry->prev = NULL;
 		hash_entry->next = NULL; 
 	}

	return 0;
 }

/* this function removes and deletes the process hash entry */
 int delete_hash_entry(struct task_struct *p) {
 	
 	if (remove_fromHash_table(p) == -1) return -1;
 	kfree((const void *) p->execd_hash_entry);
 	kfree((const void *) p->execd_half_1_hash_entry);
 	kfree((const void *) p->execd_half_2_hash_entry);
 	kfree((const void *) p->execd_quarter_1_hash_entry);
 	kfree((const void *) p->execd_quarter_2_hash_entry);
 	kfree((const void *) p->execd_quarter_3_hash_entry);
 	kfree((const void *) p->execd_quarter_4_hash_entry);

 	p->execd_hash_entry = NULL;
	p->execd_half_1_hash_entry = NULL;
	p->execd_half_2_hash_entry = NULL;
	p->execd_quarter_1_hash_entry = NULL;
	p->execd_quarter_2_hash_entry = NULL;
	p->execd_quarter_3_hash_entry = NULL;
	p->execd_quarter_4_hash_entry = NULL;
 	
 	return 0;
 }





















