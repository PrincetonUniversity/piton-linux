#include <linux/string.h>

//#define XEMACLITE_DEBUG

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#undef AL_DEBUG
#ifdef XEMACLITE_DEBUG
	#define AL_DEBUG(fmt, args...) printk(KERN_ALERT "DEBUG %s:%3d: " fmt, __FILENAME__, __LINE__, ## args)
#else
	#define AL_DEBUG(fmt, args...) /* do nothing */
#endif

#define AL_ADDRVAL(addr, val) AL_DEBUG("Addr = %p, val = %x", addr, val)

#undef AL_ERROR
#define AL_ERROR(fmt, args...) printk(KERN_ALERT "ERROR: %s:%3d: " fmt, __FILENAME__, __LINE__, ## args)
