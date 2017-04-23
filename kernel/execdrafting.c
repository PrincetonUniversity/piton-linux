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


/* to remove later */ 
#include <linux/printk.h>

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

int calculate_hash(struct task_struct *p) {

	Elf64_Ehdr *ehdr; 
 	Elf64_Shdr *sectionHeader;
 	Elf64_Shdr *sh_strtab;
 	uint64_t i, sectionSize, sectionOffset;
 	loff_t position;  
 	struct file *file;
	struct shash_desc desc;
	char *buffer;
	char *section_names; 
	mm_segment_t oldfs;
	

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

	/*calculate the hash */
	desc.tfm = crypto_alloc_shash("md5", CRYPTO_ALG_TYPE_SHASH, CRYPTO_ALG_ASYNC);
	crypto_shash_init(&desc);
	crypto_shash_finup(&desc, (const u8 *)buffer, (unsigned int) sectionSize, (u8 *)&current->execd_hash);
	crypto_free_shash(desc.tfm);

	/*free allocations */
	kfree((const void *) buffer); 
	kfree((const void *) ehdr);
	kfree((const void *) section_names);

	/*resteore the file system */
	filp_close(file, NULL);
 	set_fs(oldfs); 

 	printk("Correct here 5 \n");

 	add_tohash_table(p);

	return 0;
 }

/* convert the hash to an int and find the bucket for the hash */
int get_hash_bucket(struct task_struct *p) {
	 unsigned long res;
	 int bucket;
	 kstrtoul((const char *) p->execd_hash, 10 /* base */, &res);

	bucket = res % NUMBER_OF_BUCKETS;

	printk("res %ld \n", res);
	printk("res %d \n", bucket);

	if (bucket < 0) 
		bucket *= -1;

	return bucket;

/* this function add the process to the hash table based on the 
 process' hash value */
int add_tohash_table(struct task_struct *p) {
 	
 
 	int hash_int;
 	struct hash_table_entry *new_entry;
 	if (p == NULL) return -1;


 	printk("add_tohash_table 1 \n");

 	if (p->execd_hash == NULL) return -1;

 	hash_int = get_hash_bucket(p);

 	if (hash_int < 0) return -1;

 	printk("add_tohash_table %d \n", hash_int);

 	new_entry = (struct hash_table_entry *)kzalloc((size_t)sizeof(struct hash_table_entry), GFP_KERNEL);
 	if (new_entry == NULL) return -1;

 	printk("add_tohash_table 3 \n");

 	if (hash_table == NULL) return -1;
 	if (hash_table[hash_int] == NULL) return -1;

 	hash_table[hash_int].next->prev = new_entry;

 	new_entry->current_task =  p;
 	new_entry->next = hash_table[hash_int].next;
 	new_entry->prev = &hash_table[hash_int];

 	hash_table[hash_int].next = new_entry;
 	p->hash_entry = new_entry; 

 	printk("add_tohash_table 4 \n");
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





















