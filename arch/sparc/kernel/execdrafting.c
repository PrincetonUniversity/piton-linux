/* 
 *
 * Code for making a hash of the text section of a elf file 
 *
 * 
 */

// include the relevant files
#include <linux/file.h>
#include <include/asm/current.h>
#include <include/linux/elf.h>
#include <linux/slab.h>
#include <include/linux/crypto.h>

 static int calculate_hash(struct file *file) {

 	Elf32_Ehdr ehdr; 
 	Elf32_Shdr *sectionHeader;
 	uint64_t i, sectionSize, sectionOffset, position;
 	char *buffer;
 	struct scatterlist sg;
	struct hash_desc desc;
 	
 	// read the header 
 	position = 0;
 	vfs_read(file, (char *)&ehdr, sizeof(Elf32_Ehdr), &position);

	// get the section headers
	position = ehdr.e_shoff;
	if (buffer == NULL) return -1; 
	buffer = (char *)kmalloc((size_t)(ehdr.e_shnum * ehdr.e_shentsize), __GFP_REPEAT);
	vfs_read(file, buffer, ehdr.e_shnum * ehdr.e_shentsize, &position);

	// find the section header for the text section of the file
	for (i = 0; i < ehdr.e_shnum; i++) {
		position = i * sizeof(Elf32_Ehdr);
		sectionHeader = (Elf32_Shdr*)&buffer[position];
		if(!strcmp(sectionHeader->sh_name, ".text")) {
			sectionOffset = sectionHeader->sh_offset;
			sectionSize = sectionHeader->sh_size;
			break;
		}
		sectionHeader = NULL;
	}

	if (sectionHeader == NULL) return -1;

	// read the section if we found it and compute the has of the text section 
	free(buffer);
	buffer = (char *)kmalloc((size_t)sectionSize, __GFP_REPEAT);
	if (buffer == NULL) return -1; 
	vfs_read(file, buffer, sectionSize, &sectionOffset);

	sg_init_one(&sg, buffer, sectionSize);
	desc.tfm = crypto_alloc_hash("md5", 0, CRYPTO_ALG_ASYNC);
	crypto_hash_init(&desc);
	crypto_hash_update(&desc, &sg, sectionSize);
	crypto_hash_final(&desc, current->execd_hash);
	crypto_free_hash(desc.tfm);
	free(buffer);
	
	return 0;
 }

 static int add_tohash_table(struct hash_desc *desc) {

 	// ch
 }




















