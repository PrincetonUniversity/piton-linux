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

struct hash_table_entry hash_table[NUMBER_OF_BUCKETS];

/* called when the computer is switched on */
void init_hash_table_entrries(void) {
	int i;
	for (i = 0; i < NUMBER_OF_BUCKETS; i++)  {
		hash_table[i].next = NULL;
		hash_table[i].prev = NULL;
		hash_table[i].current_task = NULL;
	}
}

int calculate_hash(char *file) {

 	Elf32_Ehdr ehdr; 
 	Elf32_Shdr *sectionHeader;
 	uint64_t i, sectionSize, sectionOffset;
 	loff_t position = 0;
 	/*char *buffer; */
	struct shash_desc desc;
	char buffer[sizeof(Elf32_Ehdr)];

 	
 	/* read the header */
 	position = 0;
 	if (file == NULL) return -1;
 	/*vfs_read(file, (char *)&buffer, sizeof(Elf32_Ehdr), &position); */

	/* get the section headers *
	position = ehdr.e_shoff;
	buffer = (char *)kmalloc((size_t)(ehdr.e_shnum * ehdr.e_shentsize), __GFP_REPEAT);
	if (buffer == NULL) return -1; 
	vfs_read(file, buffer, ehdr.e_shnum * ehdr.e_shentsize, &position);

	/* find the section header for the text section of the file *
	for (i = 0; i < ehdr.e_shnum; i++) {
		position = i * sizeof(Elf32_Ehdr);
		sectionHeader = (Elf32_Shdr*)&buffer[position];
		if(!strcmp((const char *)&sectionHeader->sh_name, ".text")) {
			sectionOffset = sectionHeader->sh_offset;
			sectionSize = sectionHeader->sh_size;
			break;
		}
		sectionHeader = NULL;
	}

	if (sectionHeader == NULL) return -1;

	/* read the section if we found it and compute the has of the text section *
	kfree((const void *) buffer);
	buffer = (char *)kmalloc((size_t)sectionSize, __GFP_REPEAT);
	if (buffer == NULL) return -1; 
	vfs_read(file, buffer, sectionSize, &sectionOffset);

    /*
	sg_init_one(&sg, buffer, sectionSize);
	desc.tfm = crypto_alloc_hash("md5", 0, CRYPTO_ALG_ASYNC);
	crypto_hash_init(&desc);
	crypto_hash_update(&desc, &sg, sectionSize);
	crypto_hash_final(&desc, current->execd_hash);
	crypto_free_hash(desc.tfm); 

	md5_init(&desc);
	md5_update(&desc, (const u8 *)buffer, (unsigned int) sectionSize);
	md5_final(&desc, (u8 *)&current->execd_hash);
	*

	desc.tfm = crypto_alloc_shash("md5", CRYPTO_ALG_TYPE_SHASH, CRYPTO_ALG_ASYNC);
	crypto_shash_init(&desc);
	crypto_shash_finup(&desc, (const u8 *)buffer, (unsigned int) sectionSize, (u8 *)&current->execd_hash);
	crypto_free_shash(desc.tfm);
	kfree((const void *) buffer); */

	return 0;
 }

/* convert the hash to an int and find the bucket for the hash */
int get_hash_bucket(struct task_struct *p) {
	/*
	 unsigned long res;
	 kstrtoul((const char *) p->execd_hash, 10 /* base *, &res);
	return (int) (res / NUMBER_OF_BUCKETS);
	*/
	return 0;
}

/* this function add the process to the hash table based on the 
 process' hash value */
int add_tohash_table(struct task_struct *p) {
 	
   /*
 	int hash_int;
 	struct hash_table_entry *new_entry;
 	if (p == NULL) return -1;

 	hash_int = get_hash_bucket(p);
 	new_entry = (struct hash_table_entry *)kmalloc((size_t)sizeof(struct hash_table_entry), __GFP_REPEAT);
 	if (new_entry == NULL) return -1;

 	hash_table[hash_int].next->prev = new_entry;

 	new_entry->current_task =  p;
 	new_entry->next = hash_table[hash_int].next;
 	new_entry->prev = &hash_table[hash_int];

 	hash_table[hash_int].next = new_entry;
 	p->hash_entry = new_entry;
 	*/
 	return 0;
 }

/* finds and returns a task that is similar to task referenced by p */
struct task_struct *find_similar_task(struct task_struct *p) {

 	/*int hash_int;
 	struct hash_table_entry *temp;

 	if (p == NULL) return NULL;
 	if (p->execd_hash == NULL) return NULL;

 	hash_int = get_hash_bucket(p);
 	temp = hash_table[hash_int].next;

 	while (temp != NULL) {
 		if (strncmp((const char *)temp->current_task->execd_hash, (const char *)p->execd_hash, 32) == 0) 
 			return temp->current_task;	

 		temp = temp->next;	
 	} */
 	return p;
 }

/* this function removes the process from the hash table */
int remove_fromHash_table(struct task_struct *p) {

 	/*struct hash_table_entry *temp;
 	if (p == NULL) return -1;

	temp = p->hash_entry;
	if (temp == NULL) return -1;

 	temp->prev->next = temp->next;
 	temp->next->prev = temp->prev;
 	temp->prev = NULL;
 	temp->next = NULL; */
 	return 0;
 }

/* this function removes and deletes the process hash entry */
 int delete_hash_entry(struct task_struct *p) {
 	/*
 	if (remove_fromHash_table(p) == -1) return -1;
 	kfree((const void *) p->hash_entry);
 	p->hash_entry = NULL;
 	*/
 	return 0;
 }





















