/* 
 *
 * Code for making a hash of the text section of a elf file 
 *
 * 
 */

// include the relevant files
#include <linux/file.h>
#include <include/linux/crypto.h>

 static int calculate_hash(struct file *file);
 static int add_tohash_table(struct hash_desc *desc);