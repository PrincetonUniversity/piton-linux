/*
 * mitts.h
 *
 * Copyright (C) 2016 Princeton University
 */

#ifndef _SPARC64_MITTS_H
#define _SPARC64_MITTS_H

#ifndef __ASSEMBLY__

/*
 * Write r1 and r2 to MITTS configuration registers.
 * Function defined in assembly in lib/mitts_write_config.S
 */
#ifdef CONFIG_OPENPITON_MITTS
extern int mitts_write_config(unsigned long, unsigned long);
#else
static inline int mitts_write_config(unsigned long r1, unsigned long r2) { return 0; }
#endif /* CONFIG_OPENPITON_MITTS */

#endif /* !__ASSEMBLY__ */

#endif /* _SPARC64_MITTS_H */
