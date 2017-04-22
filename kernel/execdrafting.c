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
 	Elf64_Phdr *phdr;
 	Elf64_Shdr *sh_strtab
 	uint64_t i, sectionSize, sectionOffset;
 	loff_t position;  
 	struct file *file;
	struct shash_desc desc;
	char *buffer, section_names; 
	mm_segment_t oldfs;
	

 	/* read the header */
 	if (p == NULL) return -1;
 	if (p->filename == NULL) return -1;
 	position = 0;
 	oldfs = get_fs();
 	set_fs(KERNEL_DS);
 	/*file = filp_open(p->program_filename->name, O_RDONLY, FMODE_READ); */
 	file = NULL;
 	ehdr = NULL;
 	/*file = file_open_name(p->program_filename, O_RDONLY, FMODE_READ); */

 	file = filp_open(p->filename,O_RDONLY,0);
 	ehdr = kzalloc((size_t)sizeof(Elf64_Ehdr), GFP_KERNEL);

 	if ((ehdr == NULL) || (file == NULL)) return -1;
 	vfs_read(file, (char *) ehdr, sizeof(Elf64_Ehdr), &position);  	

 	printk("Correct positon 1: %d\n", position);

	/* get the section headers */
	printk("sizeof(Elf64_Ehdr) : %d\n", (int)sizeof(Elf64_Ehdr));
	printk("ehdr->e_shoff : %d\n", ehdr->e_shoff);
	printk("ehdr->e_phentsize : %d\n", ehdr->e_phentsize);
	printk("ehdr->e_phnum : %d\n", ehdr->e_phnum);
	printk("ehdr->e_phoff : %d\n", ehdr->e_phoff);
	printk("ehdr->e_shstrndx : %d\n", ehdr->e_shstrndx);
	printk("ehdr->e_shnum : %d\n", ehdr->e_shnum);
	printk("ehdr->e_shentsize : %d\n", ehdr->e_shentsize);

	/*
	Print the offset of the section header table because it seems like we do not have a program header
	*/

	/*position = vfs_llseek(file, 0, SEEK_END);
	printk("After vfs_llseek: %d\n", position); */

	
	position = ehdr->e_shoff;
	buffer = kzalloc((size_t)(ehdr->e_shnum * ehdr->e_shentsize), GFP_KERNEL);
	if (buffer == NULL) return -1; 
	vfs_read(file, buffer, ehdr->e_shnum * ehdr->e_shentsize, &position);

	/*file->f_op->read(file, (char *)buffer, ehdr->e_shnum * ehdr->e_shentsize, &position);*/

	printk("Correct positon 2: %d\n", position);

	/*position = ehdr->e_phoff;
	phdr = kzalloc((size_t)sizeof(Elf64_Phdr), GFP_KERNEL);
	if (phdr == NULL) return -1;
	vfs_read(file, (char *)phdr, sizeof(Elf64_Phdr) , &position); 
	printk("phdr->p_offset : %d\n", phdr->p_offset);
	printk("phdr->p_filesz : %d\n", phdr->p_filesz);
	printk("Correct positon 3: %d\n", position);*/

	/*vfs_llseek(struct file *file, loff_t offset, int whence); */

	/* find the section header for the text section of the file */
	/* get the section with the names, this is the section with the section headers */
	sh_strtab = (Elf64_Shdr*)&buffer[(ehdr->e_shstrndx) * sizeof(Elf64_Shdr)];
	position = sh_strtab->sh_offset;
	vfs_read(file, section_names, sh_strtab->sh_size, &position);


	printk("Correct here 2 \n");
	for (i = 0; i < ehdr->e_shnum; i++) {

		position = i * sizeof(Elf64_Shdr);
		sectionHeader = (Elf64_Shdr*)&buffer[position];

		printk("%2d: %4d '%s'\n", i, sectionHeader->sh_name, section_names[sectionHeader->sh_name]);

		if(!strcmp((const char *)&section_names[sectionHeader->sh_name], ".text")) {
			sectionOffset = sectionHeader->sh_offset;
			sectionSize = sectionHeader->sh_size;
			printk("Found it \n");
			break;
		}
		sectionHeader = NULL; 

	}

	if (sectionHeader == NULL) return -1; 
	printk("Correct here 3 \n");  

	/* read the section if we found it and compute the has of the text section *
	kfree((const void *) buffer);
	buffer = kzalloc((size_t)sectionSize, GFP_KERNEL);
	if (buffer == NULL) return -1; 
	vfs_read(file, buffer, sectionSize, &sectionOffset); 
	printk("Correct here 4 \n"); */

   /*
	sg_init_one(&sg, buffer, sectionSize);
	desc.tfm = crypto_alloc_hash("md5", 0, CRYPTO_ALG_ASYNC);
	crypto_hash_init(&desc);
	crypto_hash_update(&desc, &sg, sectionSize);
	crypto_hash_final(&desc, current->execd_hash);
	crypto_free_hash(desc.tfm); 

	md5_init(&desc);
	md5_update(&desc, (const u8 *)buffer, (unsigned int) sectionSize);
	md5_final(&desc, (u8 *)&current->execd_hash); *

	desc.tfm = crypto_alloc_shash("md5", CRYPTO_ALG_TYPE_SHASH, CRYPTO_ALG_ASYNC);
	crypto_shash_init(&desc);
	crypto_shash_finup(&desc, (const u8 *)buffer, (unsigned int) sectionSize, (u8 *)&current->execd_hash);
	crypto_free_shash(desc.tfm);
	kfree((const void *) buffer); 
	kfree(ehdr);*/

	filp_close(file, NULL);
 	set_fs(oldfs); 

 	printk("Correct here 5 \n");

	return 0;
 }

/* convert the hash to an int and find the bucket for the hash */
int get_hash_bucket(struct task_struct *p) {
	/*
	 unsigned long res;
	 kstrtoul((const char *) p->execd_hash, 10 *//* base *, &res);
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





















